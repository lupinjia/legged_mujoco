# 介绍
基本架构与原始的[unitree_sdk2](https://github.com/unitreerobotics/unitree_mujoco)一致

## 额外依赖

- [opencv 4.12.0](https://github.com/opencv/opencv/tree/4.12.0)

## 特点

- 添加了亚博无线手柄的支持
  针对[亚博USB无线手柄](https://yahboom.com/study_module/PS2)的协议添加了手柄映射代码(simulate/src/physics_joystick.h)

- 添加了对深度图像获取的支持(C++与Python)

  在`go2.xml`中添加了camera元素，并通过mujoco接口获取了深度图像数据. 深度图数据通过cyclonedds被发布到"rt/depthimage"话题(目前仅在Python中可用dds). 使用了unitree_sdk2py的接口来进行dds的发布和订阅. 

- 按下Enter键来追踪机器人的基座, 按下Esc键使用自由相机

## URDF 转换为 MuJoCo

`urdf2mjcf` 工具可将 URDF 机器人描述转换为 MuJoCo MJCF。安装方法、命令行选项和示例请参阅
[`urdf2mjcf/README.md`](urdf2mjcf/README.md)。转换器可以在 `robots/<name>/` 下生成机器人文件；
生成的 MJCF 可能仍需手动检查和完善。网格文件根据 URDF 中的网格路径解析，本仓库不包含 N1 机器人资源。
