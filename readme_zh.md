# 介绍
基本架构与原始的[unitree_sdk2](https://github.com/unitreerobotics/unitree_mujoco)一致

## 特点

- 添加了亚博无线手柄的支持
  针对[亚博USB无线手柄](https://yahboom.com/study_module/PS2)的协议添加了手柄映射代码(simulate/src/physics_joystick.h)

- 按下Enter键来追踪机器人的基座, 按下Esc键使用自由相机

## URDF 转换为 MuJoCo

`urdf2mjcf` 工具可将 URDF 机器人描述转换为 MuJoCo MJCF。安装方法、命令行选项和示例请参阅
[`urdf2mjcf/README.md`](urdf2mjcf/README.md)。转换器可以在 `robots/<name>/` 下生成机器人文件；
生成的 MJCF 可能仍需手动检查和完善。网格文件根据 URDF 中的网格路径解析，本仓库不包含 N1 机器人资源。
