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

#ifndef ISAAC_ROS_CUVSLAM__VISUAL_SLAM_NODE_HPP_
#define ISAAC_ROS_CUVSLAM__VISUAL_SLAM_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <atomic>

#include "isaac_ros_cuvslam/impl/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

// 学习入口：参数与消息接口的完整声明；仓库名仍是 isaac_ros_visual_slam，ROS 算法包已改为 isaac_ros_cuvslam。
namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// IMU 噪声参数结构；保留原字段单位说明，calibration_frequency 随后写入 SDK 的 frequency。
struct IMUParams
{
  double gyroscope_noise_density;       // rad / (s * srqt(Hz))
  double gyroscope_random_walk;         // rad / (s ^ 2 * srqt(Hz))
  double accelerometer_noise_density;   // m / (s ^ 2 * srqt(Hz))
  double accelerometer_random_walk;     // m / (s ^ 3 * srqt(Hz))
  double calibration_frequency;
};

/// Tracking mode enumeration
/// Defines the type of tracking algorithm used by the visual SLAM system
// 跟踪算法选择：多相机、视觉惯性、RGBD；这些整数也用于 launch 参数。
enum class TrackingMode : int
{
  MULTICAMERA = 0,  ///< Default multicamera stereo tracking
  VIO = 1,          ///< Visual-Inertial Odometry (IMU fusion required)
  RGBD = 2          ///< RGBD depth-based tracking
};

/// Convert TrackingMode enum to string representation
// 仅为日志把模式整数转换为可读名称；未知数值返回 Unknown。
inline const char * TrackingModeToString(int mode)
{
  switch (mode) {
    case static_cast<int>(TrackingMode::MULTICAMERA):
      return "Multicamera";
    case static_cast<int>(TrackingMode::VIO):
      return "VIO (IMU fusion)";
    case static_cast<int>(TrackingMode::RGBD):
      return "RGBD";
    default:
      return "Unknown";
  }
}

// ROS 节点持有参数和接口，私有实现封装同步器、cuVSLAM 对象和 TF/轨迹发布。
class VisualSlamNode : public rclcpp::Node
{
public:
  // 声明组件节点构造函数；实现同时建立订阅服务并预热 CUDA。
  explicit VisualSlamNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
  // 声明析构路径，等待定位线程并按配置保存地图。
  virtual ~VisualSlamNode();

private:
  // Functional Parameters:
  // Number of cameras used. If a single stereocamera is used this has to be set to 2.
  // This is the single source of truth, ie. if you set num_cameras=2 but pass 4
  // input_camera_optical_frames we will only use the first 2.
  const uint num_cameras_;

  // Functional Parameters:
  // Number of input masks used.
  const uint num_input_masks_;

  // Multicamera mode: moderate (0), performance (1) or precision (2).
  // Multicamera odometry settings will be adjusted depending on a chosen strategy.
  // Default is performance (1).
  const uint multicam_mode_;

  // The minimum number of images used. This configures how many camera images can be dropped
  // without stopping tracking.
  const uint min_num_images_;

  // Maximum value of acceptable timestamp deltas between the camera images to still consider their
  // timestamp identical. Should be < 0.5 / fps_value.
  const double sync_matching_threshold_ms_;

  // Number of pixels on the outer edges of the frame will be masked
  // during the feature selection stage. Values are defined for the left camera
  // of the stereo pair (camera at even index in the list of cameras). For the right camera
  // (camera at odd index in the list of cameras), the left and right borders will be mirrored.
  const uint border_mask_top_;
  const uint border_mask_bottom_;
  const uint border_mask_left_;
  const uint border_mask_right_;

  // Disable this if the input images have already passed through a denoising filter.
  const bool enable_image_denoising_;

  // Enable fast and robust left-to-right tracking for rectified cameras with
  // principal points on the horizontal line. Disable this if input images are
  // not already rectified.
  const bool rectified_images_;

  // If enabled, visual odometry poses will be modified such that the camera moves
  // on a horizontal plane.
  const bool enable_ground_constraint_in_odometry_;

  // If enabled, slam poses will be modified such that the camera moves on a horizontal plane.
  const bool enable_ground_constraint_in_slam_;

  // Use localization and mapping (SLAM).
  // If it is false, it is only Visual Odometry and no map will get produced.
  const bool enable_localization_n_mapping_;

  // Tracking mode: determines the type of odometry algorithm used.
  // Stored as int for ROS 2 parameter compatibility, but should be compared
  // against TrackingMode enum values: MULTICAMERA (0), VIO (1), or RGBD (2).
  const int tracking_mode_;

  // Enable IMU debug mode
  const bool enable_debug_imu_mode_;

  // Data structure to store imu noise params
  const IMUParams imu_params_;

  // Depth scale factor to convert raw depth values to meters
  const float depth_scale_factor_;

  // Camera index that the depth image is aligned with
  const int32_t depth_camera_id_;

  // Enable stereo tracking between depth-aligned camera and other cameras
  const bool depth_enable_stereo_tracking_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value). cuVSLAM library will print out warning messages
  // if rate of incoming image pair is lower than the threshold value.
  const double image_jitter_threshold_ms_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value).
  const double imu_jitter_threshold_ms_;

  // If set will save the generated map when shutting down the node to the specified path. Requires
  // enable_localization_n_mapping_ to be true.
  const std::string save_map_folder_path_;

  // Default map that is used to localize in. Can be changed at runtime with the "LoadMapSrv"
  // service. In order to use the map an initial pose hint also has to be provided with the
  // "LocalizeInMap" service or the "initialpose" topic. Using maps requires
  // enable_localization_n_mapping_ to be true.
  std::string load_map_folder_path_;

  // If true will try to localize in the map around [0,0,0]. Requires that load_map_folder_path_ is
  // set.
  bool localize_on_startup_;

  // Radius of the area on the horizontal plane used for localization in meters.
  const float localizer_horizontal_radius_;

  // Radius along the vertical axis used for localization in meters.
  const float localizer_vertical_radius_;

  // Step size along each axis on the horizontal plane for localization in meters.
  const float localizer_horizontal_step_;

  // Step size along vectical axis for localization in meters.
  const float localizer_vertical_step_;

  // Step size around vectical axis for localization in radians. Must be less then 2 Pi.
  const float localizer_angular_step_;

  // Maximum number of poses that is stored in the SLAM map. Default is 300.
  // Adding more poses will invoke reduction of the map until it`s size matches this constant.
  const uint slam_max_map_size_;

  // Minimum time interval between loop closures. Default is 500 ms.
  const uint slam_throttling_time_ms_;

  // This node assumes the following frame hierarchy:
  // map - odom - base - ... - camera_i
  // 三种参考系分别承载地图修正、连续里程计与机器人基座；不要把 map 与 odom 的位姿直接当成同一数值。
  const std::string map_frame_;
  const std::string odom_frame_;
  const std::string base_frame_;

  // Per default the frames from the camera info messages will be used. Use this parameter to
  // override this.
  const std::vector<std::string> camera_optical_frames_;

  // Per default the frame from the imu message will be used. Use this parameter to override this.
  const std::string imu_frame_;

  // Message Parameters:
  // Buffer size of image buffer used in synchronizer.
  const uint image_buffer_size_;

  // Buffer size of imu buffer.
  const uint imu_buffer_size_;

  // The QoS used for the image and subscriptions
  const rclcpp::QoS image_qos_;
  const rclcpp::QoS imu_qos_;

  // Output Parameters:
  // Enable this to override the timestamps of all outputs to the current time.
  // This is helpful when playing back with rosbags and allows to ignore the
  // recording time.
  const bool override_publishing_stamp_;

  // Publish output frames hierarchy.
  const bool publish_map_to_odom_tf_;
  const bool publish_odom_to_base_tf_;

  // Invert the odom -> base transform published to the tf tree.
  // This is useful for maintaining the one-parent rule in systems with multiple odometry sources.
  const bool invert_map_to_odom_tf_;
  const bool invert_odom_to_base_tf_;

  // Debug/Visualization Parameters:
  // Enable slam data visualization (Landmarks, Pose Graph, etc).
  // Demands additional hardware resources.
  const bool enable_slam_visualization_;

  // Enable view observations.
  const bool enable_observations_view_;

  // Enable view landmarks for map and loop closure events.
  const bool enable_landmarks_view_;

  // Max size of the buffer for pose trail visualization.
  const uint path_max_size_;

  // Verbosity for cuvslam, the larger the more verbose.
  const int verbosity_;

  // Enable this flag to dump the received images, imu messages, timestamps and
  // camera infos at the location specified by debug_dump_path_.
  const bool enable_debug_mode_;

  // Path to directory to store the dump data. Only used when enable_debug_mode_ is true.
  const std::string debug_dump_path_;

  // Enable requesting another hint if current hint fails to initialize cuvslam localization.
  const bool enable_request_hint_;

  // Callback group
  const rclcpp::CallbackGroup::SharedPtr localize_in_map_callback_group_;

  // Subscribers
  // 图像使用标准 sensor_msgs::Image，release-5.0 的 GPU 传输通过其 Buffer 后端实现。
  const std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
  image_subs_;
  // 图像使用标准 sensor_msgs::Image，release-5.0 的 GPU 传输通过其 Buffer 后端实现。
  const std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
  segmentation_masks_subs_;
  // 图像使用标准 sensor_msgs::Image，release-5.0 的 GPU 传输通过其 Buffer 后端实现。
  const std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
  depth_image_subs_;
  const std::vector<rclcpp::Subscription<CameraInfoType>::SharedPtr> camera_info_subs_;
  const rclcpp::Subscription<ImuType>::SharedPtr imu_sub_;
  const rclcpp::Subscription<PoseWithCovarianceStampedType>::SharedPtr initial_pose_sub_;

  // Publishers: Visual SLAM
  const rclcpp::Publisher<VisualSlamStatusType>::SharedPtr visual_slam_status_pub_;
  const rclcpp::Publisher<PoseStampedType>::SharedPtr tracking_vo_pose_pub_;
  const rclcpp::Publisher<PoseWithCovarianceStampedType>::SharedPtr
    tracking_vo_pose_covariance_pub_;
  const rclcpp::Publisher<OdometryType>::SharedPtr tracking_odometry_pub_;
  const rclcpp::Publisher<PathType>::SharedPtr tracking_vo_path_pub_;
  const rclcpp::Publisher<PathType>::SharedPtr tracking_slam_path_pub_;
  const rclcpp::Publisher<DiagnosticArrayType>::SharedPtr diagnostics_pub_;

  // Sends a message to global localization to generate a pose hint
  const rclcpp::Publisher<PoseWithCovarianceStampedType>::SharedPtr trigger_hint_pub_;

  // Visualization for odometry
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_observations_pub_;
  const rclcpp::Publisher<MarkerType>::SharedPtr vis_gravity_pub_;
  const rclcpp::Publisher<MarkerArrayType>::SharedPtr vis_vo_velocity_pub_;
  const rclcpp::Publisher<OdometryType>::SharedPtr vis_slam_odometry_pub_;

  // Visualization for map and "loop closure"
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_landmarks_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_loop_closure_pub_;
  const rclcpp::Publisher<PoseArrayType>::SharedPtr vis_posegraph_nodes_pub_;
  const rclcpp::Publisher<MarkerType>::SharedPtr vis_posegraph_edges_pub_;
  const rclcpp::Publisher<MarkerType>::SharedPtr vis_posegraph_edges2_pub_;

  // Visualization for localization
  const rclcpp::Publisher<MarkerArrayType>::SharedPtr vis_localizer_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_landmarks_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_observations_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_loop_closure_pub_;

  // Slam Services
  const rclcpp::Service<SrvReset>::SharedPtr reset_srv_;
  const rclcpp::Service<SrvGetAllPoses>::SharedPtr get_all_poses_srv_;
  const rclcpp::Service<SrvSetSlamPose>::SharedPtr set_slam_pose_srv_;
  const rclcpp::Service<SrvFilePath>::SharedPtr save_map_srv_;
  const rclcpp::Service<SrvFilePath>::SharedPtr load_map_srv_;
  const rclcpp::Service<SrvLocalizeInMap>::SharedPtr localize_in_map_srv_;

  // Callback functions for services.
  // 重置服务调用实现层 Exit，清除跟踪器及初始化消息；当前响应直接设为成功。
  void CallbackReset(
    const std::shared_ptr<SrvReset::Request> req,
    std::shared_ptr<SrvReset::Response> res);
  // 读取 SLAM 轨迹并逐个把 cuVSLAM 坐标基转换为 ROS 坐标基，保留 SDK 纳秒时间戳。
  void CallbackGetAllPoses(
    const std::shared_ptr<SrvGetAllPoses::Request> req,
    std::shared_ptr<SrvGetAllPoses::Response> res);
  // 将服务请求中的 ROS 位姿换基为 cuVSLAM 位姿，再设置 SLAM 状态；要求跟踪器已初始化。
  void CallbackSetSlamPose(
    const std::shared_ptr<SrvSetSlamPose::Request> req,
    std::shared_ptr<SrvSetSlamPose::Response> res);
  // 在已初始化时调用同步保存包装；注意此回调未使用 SaveMap 的 bool 返回值，响应成功不等同于落盘成功。
  void CallbackSaveMap(
    const std::shared_ptr<SrvFilePath::Request> req,
    std::shared_ptr<SrvFilePath::Response> res);
  // 仅记录后续定位使用的地图目录，不在本回调中读取数据库或立即完成定位。
  void CallbackLoadMap(
    const std::shared_ptr<SrvFilePath::Request> req,
    std::shared_ptr<SrvFilePath::Response> res);
  // 创建受管理线程发起异步定位；success=true 表示已发起，完成结果由后续跟踪时检查 future。
  void CallbackLocalizeInMap(
    const std::shared_ptr<SrvLocalizeInMap::Request> req,
    std::shared_ptr<SrvLocalizeInMap::Response> res);

  // Callback functions for subscribers.
  // 把 IMU 消息交给实现层缓存或按时间排序；VIO 模式才建立此订阅。
  void CallbackImu(const ImuType::ConstSharedPtr & msg);
  // 把索引图像交给实现层同步；图像、掩码、RGBD 深度共用索引空间。
  void CallbackImage(int index, const ImageType & msg);
  // 把相机索引和标定交给实现层，供首次构建相机 rig 使用。
  void CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg);
  // 使用消息 frame_id 中的位姿提示同步定位；失败时按配置请求新提示或关闭 ROS。
  void CallbackInitialPose(const PoseWithCovarianceStampedType::ConstSharedPtr & msg);

  // 只前向声明实现对象；其定义中的同步与 CUDA 句柄生命周期见 impl/visual_slam_impl.hpp/.cpp。
  struct VisualSlamImpl;
  std::unique_ptr<VisualSlamImpl> impl_;

  // Thread management for localization
  std::thread localization_thread_;
  std::atomic<bool> localization_thread_running_{false};
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_CUVSLAM__VISUAL_SLAM_NODE_HPP_
