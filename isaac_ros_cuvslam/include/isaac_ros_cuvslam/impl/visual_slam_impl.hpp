// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_CUVSLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
#define ISAAC_ROS_CUVSLAM__IMPL__VISUAL_SLAM_IMPL_HPP_

#include <cuda_runtime.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#define BOOST_THREAD_PROVIDES_FUTURE
#define BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION
#include "boost/outcome/result.hpp"
#include "boost/thread.hpp"
#include "boost/thread/future.hpp"
#include "cuvslam/cuvslam2.h"
#include "cuvslam/ground_constraint2.h"
#include "cv_bridge/cv_bridge.hpp"
#include "isaac_common/messaging/message_stream_synchronizer.hpp"
#include "isaac_ros_cuvslam/impl/landmarks_vis_helper.hpp"
#include "isaac_ros_cuvslam/impl/limited_vector.hpp"
#include "isaac_ros_cuvslam/impl/localizer_vis_helper.hpp"
#include "isaac_ros_cuvslam/impl/message_stream_sequencer.hpp"
#include "isaac_ros_cuvslam/impl/pose_cache.hpp"
#include "isaac_ros_cuvslam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_cuvslam/impl/types.hpp"
#include "isaac_ros_cuvslam/visual_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

// 学习入口：VisualSlamNode 私有实现的状态与方法；图像同步、IMU 交织、SDK 所有权及定位 future 在此聚合。
namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

constexpr int32_t kMaxNumCameraParameters = 12;

// 实现对象持有跟踪资源；接口层节点引用与 CUDA 流的生命周期必须长于当前处理。
struct VisualSlamNode::VisualSlamImpl
{
  // Asynchronous response for CUVSLAM_LocalizeInExistDb()
  // promise 把 SDK 定位回调的位姿/错误转换为 Boost future。
  struct LocalizeInExistDbContext
  {
    using Response = boost::outcome_v2::result<cuvslam::Pose, std::string>;
    boost::promise<Response> response_promise;
  };


  // 声明实现构造：配置多相机同步、IMU 交织、TF 和 CUDA 同步流。
  explicit VisualSlamImpl(VisualSlamNode & vslam_node);

  // 声明清理跟踪/可视化对象并销毁 CUDA 流的析构路径。
  ~VisualSlamImpl();

  // Helpers to initialize the cuvslam tracker.
  // 用首次 CameraInfo、所需 IMU 和 TF 外参建立 SDK 相机 rig 与跟踪器。
  void Initialize();

  // 检查标定消息路数；VIO 还需要首次 IMU 消息。
  bool IsReadyForInitialization() const;

  // 用 cuvslam_odometry 是否存在表示跟踪器初始化状态。
  bool IsInitialized() const;

  // 释放 SDK 对象、清空位姿缓存和初始化消息，并尝试结束未完成的定位 promise。
  void Exit();

  // Create the configuration for the cuvslam tracker.
  // 将 ROS 跟踪模式/图像预处理/RGBD 参数转换成 SDK odometry 配置。
  cuvslam::Odometry::Config CreateOdometryConfiguration();

  // Create the configuration for the cuvslam slam.
  // 将地图大小、回环间隔、平面约束及可视化配置写入 SDK SLAM 配置。
  cuvslam::Slam::Config CreateSlamConfiguration();

  // Helper function to publish a transform to the tf tree.
  // 以 target 为父坐标系、source 为子坐标系发布给定 pose。
  void PublishFrameTransform(
    rclcpp::Time stamp, const tf2::Transform & pose, const std::string & target,
    const std::string & source);

  // Helper to get latest transform from the tf tree.
  // 查询 TF 最新可用变换；初始化外参在此路径使用最新值而非传入传感器采集时刻。
  tf2::Transform GetLatestTransform(
    const std::string & target, const std::string & source);

  // Helper to publish the estimated velocity from odometry.
  // 把位姿缓存估计的线/角速度可视化为两个箭头。
  void PublishOdometryVelocity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher);

  // Helper to publish the estimated gravity vector.
  // 仅 VIO 且 SDK 已有重力估计时换基并发布可视化箭头。
  void PublishGravity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerType>::SharedPtr publisher);

  // Callbacks for subscribers.
  // 初始化前缓存首个 IMU，初始化后进入快流排序器。
  void CallbackImu(const ImuType::ConstSharedPtr & msg);
  // 初始化后按相机/掩码/深度索引和采集时间进入图像同步器。
  void CallbackImage(int index, const ImageType & image_view);
  // 初始化前缓存各路标定，达到条件时发起初始化。
  void CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg);

  // Callback for synchronizer.
  // 将多相机同步组作为慢流交给 IMU/图像交织器。
  void CallbackSynchronizedImages(
    int64_t current_ts, const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs);

  // Callback for sequencer
  // 跟踪核心：登记 IMU→取得 Buffer 句柄→Track→换基→TF/位姿/里程计/状态。
  void UpdatePose(
    const std::vector<ImuType::ConstSharedPtr> & imu_msgs,
    const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs);

  // Save the current map to disk.
  // 同步等待异步 SDK 保存回调，返回实际保存结果。
  bool SaveMap(const std::string & map_folder_path);

  // Core functionality to localize in a map. Expects and returns the pose in cuvslam conventions.
  // 验证地图目录与 .mdb 文件后用 cuVSLAM 坐标的提示发起异步定位。
  boost::future<LocalizeInExistDbContext::Response> CuvslamInternalLocalizeInMapAsync(
    const std::string & map_folder_path,
    const cuvslam::Pose & pose_hint);

  // Localize in an existing map asynchronously. Expects and returns the pose in ROS conventions.
  // 将 ROS 提示换到 map/cuVSLAM 坐标，返回会转回 ROS map 位姿的 future。
  boost::future<std::optional<PoseType>> LocalizeInMapAsync(
    const std::string & map_folder_path,
    const PoseType & pose_hint,
    const std::string & frame_id);

  // Synchonous wrapper for LocalizeInMapAsync.
  // 同步包装定位 future，调用路径会等待结果。
  std::optional<PoseType> LocalizeInMap(
    const std::string & map_folder_path,
    const PoseType & pose_hint,
    const std::string & frame_id);

  // Check and handle the localization future status
  // 非阻塞检查受管理的定位 future，记录结果后清空它。
  void CheckLocalizationStatus();

  // Reference to the ros node.
  VisualSlamNode & node;

  // Synchronizer used to sync all image messages.
  using Synchronizer = isaac_common::messaging::MessageStreamSynchronizer<ImageType>;
  // 先同步图像索引组，再把整组交给 sequencer；CameraInfo 属于初始化标定而非每帧同步成员。
  Synchronizer sync;

  // Sequencer used to sequence imu and image messages.
  using Sequencer = MessageStreamSequencer<ImuType::ConstSharedPtr,
      std::vector<std::pair<int, ImageType>>>;
  Sequencer sequencer;

  // cuVSLAM API objects
  std::unique_ptr<cuvslam::Odometry> cuvslam_odometry;
  cuvslam::Odometry::State odometry_state;
  std::shared_ptr<cuvslam::Slam> cuvslam_slam;  // Shared pointer for async visualization helpers
  std::unique_ptr<cuvslam::GroundConstraint> ground_constraint;

  // Define cameras for cuVSLAM. 2 for stereo camera.
  std::vector<cuvslam::Camera> cuvslam_cameras;

  // Helper classes for tf listening and publishing.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer{nullptr};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_publisher{nullptr};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher{nullptr};

  limited_vector<PoseStampedType> vo_path;
  limited_vector<PoseStampedType> slam_path;

  // Point cloud to visualize tracks
  // LandmarksVisHelper observations_vis_helper;
  // Point cloud to visualize landmarks
  LandmarksVisHelper landmarks_vis_helper;
  // Loop closure landmarks
  LandmarksVisHelper lc_landmarks_vis_helper;
  // PoseGraph visualization
  PoseGraphVisHelper pose_graph_helper;
  // Localizer visualization
  LocalizerVisHelper localizer_helper;
  // Localizer: map landmarks
  LandmarksVisHelper localizer_landmarks_vis_helper;
  // Localizer: tracked landmarks
  LandmarksVisHelper localizer_observations_vis_helper;
  // Localizer: loop closure landmarks
  LandmarksVisHelper localizer_lc_landmarks_vis_helper;

  // Pose cache. For velocity calculation.
  PoseCache pose_cache;

  // Velocity cache. For velocity covariance calculation.
  VelocityCache velocity_cache;

  // Container to calulate execution time statictics.
  limited_vector<double> track_execution_times;

  // Timestamp of the last time CUVSLAM_Track was called in nanoseconds.
  int64_t last_track_ts;

  // Initial messages required for initialization.
  std::optional<ImuType::ConstSharedPtr> initial_imu_message;
  std::map<int, std::optional<CameraInfoType::ConstSharedPtr>> initial_camera_info_messages;

  LocalizeInExistDbContext localize_in_exist_db_context;
  bool localized_in_exist_map_ = false;

  // Store the localization future to prevent it from being destroyed before callback completes
  std::optional<boost::future<std::optional<PoseType>>> localization_future_;
  mutable std::mutex localization_mutex_;

  // CUDA stream used for rosidl::Buffer ReadHandle synchronization between the producer
  // (upstream node) and cuVSLAM. Created in the constructor and destroyed in the destructor.
  // 此流用于 Buffer 生产者/消费者同步，SDK Track 还可能使用自己的内部 CUDA 流。
  cudaStream_t cuda_stream_{};
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_CUVSLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
