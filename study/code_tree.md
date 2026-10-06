# isaac_ros_visual_slam · 完整代码树

[初步分析](analysis.md) · [源码定位](code_symbols.md) · [范围](annotation_scope.json) · [验证](annotation_validation.json)

固定基线 `e4dc8f59816dfca387bcb92364ecaf9207216475`，来源 `release-5.0`，学习分支 `codex/study/isaac-ros-code-analysis`。

77 个基线文件、1 个 gitlink；本轮注释 8 个原始文件。树不包含 `.git` 或缓存，固定基线自带的资源仍完整列出。

```text
isaac_ros_visual_slam/
├── .gitattributes
├── .gitignore
├── .gitmodules  [只读参考；未注释]
├── CONTRIBUTING.md
├── LICENSE
├── README.md  [只读参考；未注释]
├── SECURITY.md
├── cuvslam  [外部 submodule @ 33da60662b27c0c5b63e0597bd778066af2731ed；内部未分析]
├── isaac_ros_cuvslam/
│   ├── CMakeLists.txt  [只读参考；未注释]
│   ├── config/
│   │   ├── foxglove_bridge_launch.xml
│   │   ├── multi_realsense.yaml
│   │   └── zed.yaml
│   ├── foxglove_layouts/
│   │   ├── foxglove_layout_multi_realsense.json
│   │   └── foxglove_layout_realsense_mask.json
│   ├── include/
│   │   └── isaac_ros_cuvslam/
│   │       ├── impl/
│   │       │   ├── cuvslam_ros_conversion.hpp  [原样保留；未展开]
│   │       │   ├── has_subscribers.hpp  [原样保留；未展开]
│   │       │   ├── landmarks_vis_helper.hpp  [原样保留；未展开]
│   │       │   ├── limited_vector.hpp  [原样保留；未展开]
│   │       │   ├── localizer_vis_helper.hpp  [原样保留；未展开]
│   │       │   ├── message_stream_sequencer.hpp  [本轮中文注释]
│   │       │   ├── pose_cache.hpp  [原样保留；未展开]
│   │       │   ├── posegraph_vis_helper.hpp  [原样保留；未展开]
│   │       │   ├── stopwatch.hpp  [原样保留；未展开]
│   │       │   ├── types.hpp  [只读参考；未注释]
│   │       │   ├── visual_slam_impl.hpp  [本轮中文注释]
│   │       │   └── viz_helper.hpp  [原样保留；未展开]
│   │       └── visual_slam_node.hpp  [本轮中文注释]
│   ├── launch/
│   │   ├── isaac_ros_visual_slam.launch.py  [原样保留；未展开]
│   │   ├── isaac_ros_visual_slam_core.launch.py  [本轮中文注释]
│   │   ├── isaac_ros_visual_slam_isaac_sim.launch.py  [原样保留；未展开]
│   │   ├── isaac_ros_visual_slam_multi_realsense.launch.py  [原样保留；未展开]
│   │   ├── isaac_ros_visual_slam_realsense.launch.py  [本轮中文注释]
│   │   ├── isaac_ros_visual_slam_realsense_mask.launch.py  [原样保留；未展开]
│   │   └── isaac_ros_visual_slam_realsense_rgbd.launch.py  [原样保留；未展开]
│   ├── package.xml  [只读参考；未注释]
│   ├── rviz/
│   │   ├── default.cfg.rviz
│   │   ├── isaac_sim.cfg.rviz
│   │   ├── multi_realsense.cfg.rviz
│   │   ├── realsense.cfg.rviz
│   │   ├── realsense_rgbd.cfg.rviz
│   │   ├── vslam_keepall.cfg.rviz
│   │   ├── vslam_segformer.rviz
│   │   └── zed.cfg.rviz
│   ├── src/
│   │   ├── impl/
│   │   │   ├── cuvslam_ros_conversion.cpp  [本轮中文注释]
│   │   │   ├── landmarks_vis_helper.cpp  [原样保留；未展开]
│   │   │   ├── localizer_vis_helper.cpp  [原样保留；未展开]
│   │   │   ├── pose_cache.cpp  [原样保留；未展开]
│   │   │   ├── posegraph_vis_helper.cpp  [原样保留；未展开]
│   │   │   ├── visual_slam_impl.cpp  [本轮中文注释]
│   │   │   └── viz_helper.cpp  [原样保留；未展开]
│   │   ├── visual_slam_main.cpp  [原样保留；未展开]
│   │   └── visual_slam_node.cpp  [本轮中文注释]
│   └── test/
│       ├── __init__.py  [原样保留；未展开]
│       ├── helpers.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_pol_multi_cam_imu.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_pol_rgbd_cam.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_pol_single_cam_imu.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_srv_get_all_poses.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_srv_load_map.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_srv_localize_in_map.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_srv_reset.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_srv_save_map.py  [原样保留；未展开]
│       ├── isaac_ros_visual_slam_srv_set_slam_pose.py  [原样保留；未展开]
│       ├── test_cases/
│       │   └── rosbags/
│       │       ├── r2b_galileo/
│       │       │   ├── cuvslam_map/
│       │       │   │   └── data.mdb
│       │       │   ├── metadata.yaml
│       │       │   ├── nova/
│       │       │   │   └── calibration/
│       │       │   │       └── isaac_calibration.urdf
│       │       │   └── r2b_galileo_0.mcap
│       │       └── rgbd_static/
│       │           ├── metadata.yaml
│       │           └── rosbag2_rs455_rgbd.mcap
│       └── test_message_stream_sequencer.cpp  [原样保留；未展开]
├── isaac_ros_visual_slam_interfaces/
│   ├── CMakeLists.txt
│   ├── msg/
│   │   └── VisualSlamStatus.msg
│   ├── package.xml
│   └── srv/
│       ├── FilePath.srv
│       ├── GetAllPoses.srv
│       ├── LocalizeInMap.srv
│       ├── Reset.srv
│       └── SetSlamPose.srv
└── study/
    ├── analysis.md  [新增学习材料]
    ├── annotation_scope.json  [新增学习材料]
    ├── annotation_validation.json  [新增学习材料]
    ├── code_symbols.md  [新增学习材料]
    ├── code_tree.md  [新增学习材料]
    ├── generate_study_index.py  [新增学习材料]
    ├── repository.json  [新增学习材料]
    └── verify_annotations.py  [新增学习材料]
```

## 本轮阅读入口

| 文件 | 范围 | 阅读深度 |
|---|---|---|
| [.gitmodules](../.gitmodules) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [README.md](../README.md) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [isaac_ros_cuvslam/CMakeLists.txt](../isaac_ros_cuvslam/CMakeLists.txt) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [isaac_ros_cuvslam/include/isaac_ros_cuvslam/impl/message_stream_sequencer.hpp](../isaac_ros_cuvslam/include/isaac_ros_cuvslam/impl/message_stream_sequencer.hpp) | 中文注释 | full |
| [isaac_ros_cuvslam/include/isaac_ros_cuvslam/impl/types.hpp](../isaac_ros_cuvslam/include/isaac_ros_cuvslam/impl/types.hpp) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [isaac_ros_cuvslam/include/isaac_ros_cuvslam/impl/visual_slam_impl.hpp](../isaac_ros_cuvslam/include/isaac_ros_cuvslam/impl/visual_slam_impl.hpp) | 中文注释 | full |
| [isaac_ros_cuvslam/include/isaac_ros_cuvslam/visual_slam_node.hpp](../isaac_ros_cuvslam/include/isaac_ros_cuvslam/visual_slam_node.hpp) | 中文注释 | full |
| [isaac_ros_cuvslam/launch/isaac_ros_visual_slam_core.launch.py](../isaac_ros_cuvslam/launch/isaac_ros_visual_slam_core.launch.py) | 中文注释 | full |
| [isaac_ros_cuvslam/launch/isaac_ros_visual_slam_realsense.launch.py](../isaac_ros_cuvslam/launch/isaac_ros_visual_slam_realsense.launch.py) | 中文注释 | full |
| [isaac_ros_cuvslam/package.xml](../isaac_ros_cuvslam/package.xml) | 只读参考，未注释 | reference: 版本/依赖/类型核对；未登记全文阅读深度 |
| [isaac_ros_cuvslam/src/impl/cuvslam_ros_conversion.cpp](../isaac_ros_cuvslam/src/impl/cuvslam_ros_conversion.cpp) | 中文注释 | full |
| [isaac_ros_cuvslam/src/impl/visual_slam_impl.cpp](../isaac_ros_cuvslam/src/impl/visual_slam_impl.cpp) | 中文注释 | full |
| [isaac_ros_cuvslam/src/visual_slam_node.cpp](../isaac_ros_cuvslam/src/visual_slam_node.cpp) | 中文注释 | full |

目录树和行号是定位材料，不代表所有代码已理解或运行。子模块只保留固定引用，内部文件不在本轮范围。

重新生成：`python study/generate_study_index.py`。再运行 `python study/verify_annotations.py --report study/annotation_validation.json` 审查源码保留情况；两个工具均不运行上游项目。
