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

#include <algorithm>

#include "Eigen/Dense"
#include "isaac_ros_cuvslam/impl/cuvslam_ros_conversion.hpp"
#include "isaac_ros_cuvslam/impl/types.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "rclcpp/logging.hpp"

// 学习入口：标定、位姿、协方差与图像 Buffer 的 ROS↔cuVSLAM 边界；换基和内存视图是不同操作。
namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

constexpr char kFisheye[] = "fisheye4";
constexpr char kBrown[] = "brown5k";
constexpr char kPinhole[] = "pinhole";
constexpr char kPolynomial[] = "polynomial";

// ROS' DISTORTION MODELS:
// 1.PLUMB_BOB
// 2.EQUIDISTANT
// 3.RATIONAL_POLYNOMIAL
// CUVSLAM' DISTORTION MODELS:
// 1.brown5k
// 2.fisheye4
// 3.polynomial
// ROS' PLUMB_BOB = cuVSLAM' brown5k
// 本地畸变模型枚举用于模板特化，并不是 ROS 消息中的字符串编码。
enum DistortionModel
{
  BROWN,
  FISHEYE,
  PINHOLE,
  RATIONAL_POLYNOMIAL
};

template<DistortionModel T = DistortionModel::PINHOLE>
// 默认针孔模型不需要畸变参数；模板特化处理其它模型。
void FillDistortion(const CameraInfoType::ConstSharedPtr & msg, cuvslam::Camera & camera)
{
  (void)msg;  // unused
  camera.distortion.model = cuvslam::Distortion::Model::Pinhole;
  camera.distortion.parameters.clear();
}

template<>
// 检查五个参数并重排为 k1/k2/k3/p1/p2；多出的参数不参与转换。
void FillDistortion<DistortionModel::BROWN>(
  const CameraInfoType::ConstSharedPtr & msg, cuvslam::Camera & camera)
{
  if (msg->d.size() < 5) {
    throw std::runtime_error(
            "Brown distortion model requires 5 parameters, received " +
            std::to_string(msg->d.size()) + " (excess parameters are ignored).");
  }
  camera.distortion.model = cuvslam::Distortion::Model::Brown;
  camera.distortion.parameters = {
    // radial distortion coeffs
    static_cast<float>(msg->d[0]),  // k1
    static_cast<float>(msg->d[1]),  // k2
    static_cast<float>(msg->d[4]),  // k3
    // tangential distortion coeffs
    static_cast<float>(msg->d[2]),  // p1
    static_cast<float>(msg->d[3]),  // p2
  };
}

template<>
// 检查四个鱼眼参数并按 SDK 所需顺序写入。
void FillDistortion<DistortionModel::FISHEYE>(
  const CameraInfoType::ConstSharedPtr & msg, cuvslam::Camera & camera)
{
  if (msg->d.size() < 4) {
    throw std::runtime_error(
            "Fisheye distortion model requires 4 parameters, received " +
            std::to_string(msg->d.size()) + " (excess parameters are ignored).");
  }
  camera.distortion.model = cuvslam::Distortion::Model::Fisheye;
  camera.distortion.parameters = {
    // fisheye distortion coeffs
    static_cast<float>(msg->d[0]),  // k1
    static_cast<float>(msg->d[1]),  // k2
    static_cast<float>(msg->d[2]),  // k3
    static_cast<float>(msg->d[3]),  // k4
  };
}

template<>
// 检查八个有理多项式参数，保留径向与切向系数的 SDK 顺序。
void FillDistortion<DistortionModel::RATIONAL_POLYNOMIAL>(
  const CameraInfoType::ConstSharedPtr & msg, cuvslam::Camera & camera)
{
  if (msg->d.size() < 8) {
    throw std::runtime_error(
            "Polynomial distortion model requires 8 parameters, received " +
            std::to_string(msg->d.size()) + " (excess parameters are ignored).");
  }
  camera.distortion.model = cuvslam::Distortion::Model::Polynomial;
  camera.distortion.parameters = {
    // radial distortion coeffs
    static_cast<float>(msg->d[0]),  // k1
    static_cast<float>(msg->d[1]),  // k2
    // tangential distortion coeffs
    static_cast<float>(msg->d[2]),  // t1
    static_cast<float>(msg->d[3]),  // t2
    // radial distortion coeffs
    static_cast<float>(msg->d[4]),  // k3
    static_cast<float>(msg->d[5]),  // k4
    static_cast<float>(msg->d[6]),  // k5
    static_cast<float>(msg->d[7]),  // k6
  };
}

// 把 ROS distortion_model 映射到 SDK；空模型或全零系数回退针孔，未知模型抛异常。
void FillCameraDistortion(const CameraInfoType::ConstSharedPtr & msg, cuvslam::Camera & camera)
{
  // Fallback to pinhole distortion if no distortion was provided.
  std::string distortion_model_name = msg->distortion_model;
  const auto & dist_params = msg->d;

  if (distortion_model_name == kPinhole || distortion_model_name.empty() ||
    std::all_of(dist_params.begin(), dist_params.end(), [](const auto & p) {return p == 0.0;}))
  {
    return FillDistortion<DistortionModel::PINHOLE>(msg, camera);
  }
  if (distortion_model_name == sensor_msgs::distortion_models::PLUMB_BOB) {
    return FillDistortion<DistortionModel::BROWN>(msg, camera);
  }
  if (distortion_model_name == sensor_msgs::distortion_models::EQUIDISTANT) {
    return FillDistortion<DistortionModel::FISHEYE>(msg, camera);
  }
  if (distortion_model_name == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    return FillDistortion<DistortionModel::RATIONAL_POLYNOMIAL>(msg, camera);
  }
  throw std::runtime_error("Unsupported distortion model.");
}

// 从 K 的 fx/fy/cx/cy 和消息宽高创建相机内参，然后填充畸变模型。
void FillIntrinsics(const CameraInfoType::ConstSharedPtr & msg, cuvslam::Camera & camera)
{
  camera.size = {static_cast<int32_t>(msg->width), static_cast<int32_t>(msg->height)};
  camera.principal = {static_cast<float>(msg->k[2]), static_cast<float>(msg->k[5])};
  camera.focal = {static_cast<float>(msg->k[0]), static_cast<float>(msg->k[4])};
  FillCameraDistortion(msg, camera);
}

// 把 ROS 基座坐标基下的 optical 相机外参转成 cuVSLAM rig_from_camera。
void FillExtrinsics(
  const tf2::Transform & base_canonical_pose_camera_optical,
  cuvslam::Camera & camera)
{
  // Converting base_canonical_pose_camera_optical (yes, base and camera are in different frames)
  // so that both are in the cuvslam frame. Cuvslam and optical frames are the same now,
  // so skip multiplication by identity optical_pose_cuvslam.
  const tf2::Transform cv_base_pose_cv_camera =
    cuvslam_pose_canonical * base_canonical_pose_camera_optical;

  camera.rig_from_camera = TocuVSLAMPose(cv_base_pose_cv_camera);
}

// Helper function that converts transform into cuvslam::Pose
// 序列化 tf2 位姿：四元数按 x/y/z/w，平移按 x/y/z；此函数本身不换坐标基。
cuvslam::Pose TocuVSLAMPose(const tf2::Transform & tf_mat)
{
  cuvslam::Pose cuvslamPose;
  const tf2::Quaternion rotation(tf_mat.getRotation());
  cuvslamPose.rotation[0] = rotation.x();
  cuvslamPose.rotation[1] = rotation.y();
  cuvslamPose.rotation[2] = rotation.z();
  cuvslamPose.rotation[3] = rotation.w();
  const tf2::Vector3 & translation = tf_mat.getOrigin();
  cuvslamPose.translation[0] = translation.x();
  cuvslamPose.translation[1] = translation.y();
  cuvslamPose.translation[2] = translation.z();
  return cuvslamPose;
}

// Helper function to convert cuvslam pose estimate to tf2::Transform
// 把 SDK 四元数 x/y/z/w 与三维平移恢复为 tf2 位姿；此函数本身不换坐标基。
tf2::Transform FromcuVSLAMPose(
  const cuvslam::Pose & cuvslam_pose)
{
  const auto & r = cuvslam_pose.rotation;
  const tf2::Quaternion rotation(r[0], r[1], r[2], r[3]);
  const tf2::Vector3 translation(cuvslam_pose.translation[0], cuvslam_pose.translation[1],
    cuvslam_pose.translation[2]);

  return tf2::Transform(rotation, translation);
}

// Helper funtion to change basis from frame source to frame target
// target_pose_source = Transformation(Rotation only; translation is zero) matrix between target
// and source.
// source_pose_source = Transformation(Rotation and translation) inside source frame.
// It is any arbritary transformation in source frame.
// 对同一刚体变换做换基 T_new=R*T_old*R^-1；不能把它误用为单边相机外参组合。
tf2::Transform ChangeBasis(
  const tf2::Transform & target_pose_source, const tf2::Transform & source_pose_source)
{
  return target_pose_source * source_pose_source * target_pose_source.inverse();
}

// Helper function to create cuvslam::ImageEncoding from string using sensor_msgs image encoding.
// 仅接受 mono8/rgb8 跟踪图像；其它编码抛 invalid_argument。
cuvslam::Image::Encoding TocuVSLAMImageEncoding(const std::string & image_encoding)
{
  if (image_encoding == sensor_msgs::image_encodings::MONO8) {
    return cuvslam::Image::Encoding::MONO;
  }
  if (image_encoding == sensor_msgs::image_encodings::RGB8) {
    return cuvslam::Image::Encoding::RGB;
  }
  throw std::invalid_argument("Received unknown image encoding: " + image_encoding);
}

// Helper function to create cuvslam::Image from a rosidl::Buffer-backed image.
// The caller must keep the supplied ReadHandle alive until cuvslam has finished
// using the returned cuvslam::Image so that the GPU memory and CUDA-event
// synchronization remain valid.
// 包装 CUDA 读句柄为 SDK 图像视图；width/height/pitch 来自消息，pixels 不在此处复制。
cuvslam::Image TocuVSLAMImage(
  int32_t camera_index,
  const sensor_msgs::msg::Image & image,
  const cuda_buffer_backend::ReadHandle & read_handle,
  const int64_t & acqtime_ns)
{
  cuvslam::Image cuvslam_image;
  cuvslam_image.timestamp_ns = acqtime_ns;
  // ReadHandle 必须覆盖 Track 的使用期；这里保留 GPU 指针而不是创建独立像素所有权。
  cuvslam_image.pixels = read_handle.get_ptr();
  cuvslam_image.width = image.width;
  cuvslam_image.height = image.height;
  cuvslam_image.camera_index = camera_index;
  cuvslam_image.pitch = image.step;
  cuvslam_image.encoding = TocuVSLAMImageEncoding(image.encoding);
  cuvslam_image.data_type = cuvslam::Image::DataType::UINT8;
  cuvslam_image.is_gpu_mem = true;
  return cuvslam_image;
}

// Helper function to create cuvslam::Image depth image from a rosidl::Buffer-backed image.
// The caller must keep the supplied ReadHandle alive until cuvslam has finished
// using the returned cuvslam::Image.
// 包装 RGBD 深度视图，16UC1/mono16 为 UINT16，32FC1 为 FLOAT32；深度缩放另在配置中。
cuvslam::Image TocuVSLAMDepthImage(
  int32_t camera_index,
  const sensor_msgs::msg::Image & image,
  const cuda_buffer_backend::ReadHandle & read_handle,
  const int64_t & acqtime_ns)
{
  cuvslam::Image cuvslam_depth_image;
  cuvslam_depth_image.timestamp_ns = acqtime_ns;
  cuvslam_depth_image.pixels = read_handle.get_ptr();
  cuvslam_depth_image.width = image.width;
  cuvslam_depth_image.height = image.height;
  cuvslam_depth_image.camera_index = camera_index;
  cuvslam_depth_image.pitch = image.step;
  // Depth images are single channel (MONO)
  cuvslam_depth_image.encoding = cuvslam::Image::Encoding::MONO;

  // Determine the data type based on the ROS encoding
  const std::string & encoding = image.encoding;
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    encoding == sensor_msgs::image_encodings::MONO16)
  {
    cuvslam_depth_image.data_type = cuvslam::Image::DataType::UINT16;
  } else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    cuvslam_depth_image.data_type = cuvslam::Image::DataType::FLOAT32;
  } else {
    throw std::invalid_argument("Unsupported depth image encoding: " + encoding);
  }

  cuvslam_depth_image.is_gpu_mem = true;
  return cuvslam_depth_image;
}

// Helper function to pass IMU data to cuVSLAM
// 换基线加速度与角速度三维向量并保留纳秒时间戳；输入遵循 ROS IMU 消息单位。
cuvslam::ImuMeasurement TocuVSLAMImuMeasurement(
  const ImuType::ConstSharedPtr & msg_imu, const int64_t & acqtime_ns)
{
  cuvslam::ImuMeasurement measurement;

  const auto & in_lin_acc = msg_imu->linear_acceleration;
  const auto & in_ang_vel = msg_imu->angular_velocity;

  const tf2::Vector3 ros_lin_acc(in_lin_acc.x, in_lin_acc.y, in_lin_acc.z);
  const tf2::Vector3 ros_ang_vel(in_ang_vel.x, in_ang_vel.y, in_ang_vel.z);

  // convert from ros frame to cuvslam
  // 仅旋转向量坐标基，不改变线加速度 m/s² 与角速度 rad/s 的数值单位。
  const tf2::Vector3 cuvslam_lin_acc = cuvslam_pose_canonical * ros_lin_acc;
  const tf2::Vector3 cuvslam_ang_vel = cuvslam_pose_canonical * ros_ang_vel;

  // Set linear accelerations
  measurement.linear_accelerations[0] = cuvslam_lin_acc.x();
  measurement.linear_accelerations[1] = cuvslam_lin_acc.y();
  measurement.linear_accelerations[2] = cuvslam_lin_acc.z();
  // Set angular velocity
  measurement.angular_velocities[0] = cuvslam_ang_vel.x();
  measurement.angular_velocities[1] = cuvslam_ang_vel.y();
  measurement.angular_velocities[2] = cuvslam_ang_vel.z();

  measurement.timestamp_ns = acqtime_ns;

  return measurement;
}

// 将 6×6 协方差从旋转在前重排为 ROS 平移在前，再分别旋转平移/旋转块的坐标基。
Eigen::Matrix<float, 6, 6> FromcuVSLAMCovariance(const cuvslam::PoseCovariance & covariance)
{
  tf2::Quaternion quat = canonical_pose_cuvslam.getRotation();
  Eigen::Quaternion<float> q(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Matrix<float, 3, 3> canonical_pose_cuvslam_mat = q.matrix();

  Eigen::Matrix<float, 6, 6> block_canonical_pose_cuvslam = Eigen::Matrix<float, 6, 6>::Zero();
  block_canonical_pose_cuvslam.block<3, 3>(0, 0) = canonical_pose_cuvslam_mat;
  block_canonical_pose_cuvslam.block<3, 3>(3, 3) = canonical_pose_cuvslam_mat;
  Eigen::Matrix<float, 6, 6> covariance_mat =
    Eigen::Map<const Eigen::Matrix<float, 6, 6,
      Eigen::StorageOptions::AutoAlign>>(covariance.data());
  Eigen::Matrix<float, 6, 6> ros_covariance_mat = Eigen::Matrix<float, 6, 6>::Zero();

  // The covariance matrix from cuVSLAM arranges elements as follows:
  // (rotation about X axis, rotation about Y axis, rotation about Z axis, x, y, z)
  // However, in ROS, the order is:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  ros_covariance_mat.block<3, 3>(0, 0) = covariance_mat.block<3, 3>(3, 3);
  ros_covariance_mat.block<3, 3>(0, 3) = covariance_mat.block<3, 3>(3, 0);
  ros_covariance_mat.block<3, 3>(3, 0) = covariance_mat.block<3, 3>(0, 3);
  ros_covariance_mat.block<3, 3>(3, 3) = covariance_mat.block<3, 3>(0, 0);
  Eigen::Matrix<float, 6, 6> covariance_mat_change_basis =
    block_canonical_pose_cuvslam * ros_covariance_mat * block_canonical_pose_cuvslam.transpose();
  return covariance_mat_change_basis;
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia
