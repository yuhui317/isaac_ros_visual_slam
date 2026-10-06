# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# 学习入口：RealSense 双目红外图像与 IMU 的 VIO 示例；相机驱动和 CUDA/ROS 环境属于外部运行条件。
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# 把相机驱动节点和 VisualSlamNode 容器一起返回；这里只描述连接关系，尚未验证实机运行。
def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    # 开启两路红外与 200 Hz IMU，关闭彩色和深度；这些参数是该示例配置，不能套用到任意相机。
    realsense_camera_node = Node(
        name='camera',
        namespace='',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': False,
            'enable_depth': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.infra_profile': '640,360,60',
            'depth_module.profile': '640,360,60',  # For backwards compatibility
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,
            'accel_fps': 200,  # set to 250 for rs d435i
            'unite_imu_method': 2
        }],
    )

    # tracking_mode=1 明确选择 VIO；base_frame 与两路 optical frame 必须由相机驱动的 TF 标定链连接。
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_cuvslam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': True,
            'tracking_mode': 1,  # VIO mode (IMU fusion)
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 19.00,
            'base_frame': 'camera_link',
            'imu_frame': 'camera_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
        }],
        # 每路 image 与 camera_info 配对，同时把 camera/imu 接入 visual_slam/imu。
        remappings=[
            ('visual_slam/image_0', 'camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', 'camera/infra1/camera_info'),
            ('visual_slam/image_1', 'camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', 'camera/infra2/camera_info'),
            ('visual_slam/imu', 'camera/imu'),
        ],
    )

    # 使用 ROS 组合组件容器加载 C++ 插件；GPU 缓冲后端由运行环境提供。
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([visual_slam_launch_container, realsense_camera_node])
