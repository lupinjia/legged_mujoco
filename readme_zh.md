# legged_mujoco

本项目的架构与 [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) 相同。

## 依赖

- Ubuntu 24.04
- MuJoCo 3.13.0

  ```bash
  git clone https://github.com/google-deepmind/mujoco.git && cd mujoco
  # 切换到指定版本
  git checkout  3.13.0
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_C_COMPILER=/usr/bin/gcc-13 -DCMAKE_CXX_COMPILER=/usr/bin/g++-13 # 如果安装了多个版本的 gcc，请指定 gcc-13
  make -j$(nproc)
  sudo make install
  ```
- gcc-13

## 安装

- 将本仓库克隆到本地

  ```bash
  git clone https://github.com/lupinjia/legged_mujoco.git
  ```

- 编译代码

  ```bash
  # 如果已激活 conda 环境，请先退出，以避免编译错误
  conda deactivate

  cd legged_mujoco/simulate
  mkdir build && cd build
  cmake ..
  make
  ```

## 特性

- 支持 Yahboom USB 无线手柄和 betop USB 手柄（C++）

  根据 [Yahboom USB Wireless Joystick](https://yahboom.com/study_module/PS2) 的代理接口和 betop USB 手柄，添加了手柄映射代码（simulate/src/physics_joystick.h）。

  如需使用其他类型的手柄，请将 `simulate/config.yaml` 中的 `joystick_type` 改为相应值。

- 键盘遥操作（无需游戏手柄）

  在 `simulate/config.yaml` 中设置 `use_joystick: 1` 和 `joystick_type: "keyboard"`。按住的按键由
  `simulate/src/keyboard_joystick.h`（一个 `GlfwAdapter` 子类）读取，因此即使查看器窗口处于焦点状态也能使用。
  默认映射如下：

  | 输入 | 按键 |
  | ----- | ---- |
  | 左摇杆（`lx`/`ly`） | `A`/`D`（左/右）、`S`/`W`（后退/前进） |
  | 右摇杆（`rx`/`ry`） | `J`/`L`、`K`/`I` |
  | 方向键（`left`/`right`/`up`/`down`） | 方向键（箭头键） |
  | 按钮 | `Space`=A、`Left Shift`=B、`Q`=X、`E`=Y、`1`=L1、`2`=R1、`3`=start、`4`=select |

  可编辑 `simulate/src/keyboard_joystick.h` 中的 `update()` 以重新映射按键。请注意，左/右方向键也用于 MuJoCo 的向前/向后单步快捷操作。

- 按 `Enter` 跟踪机器人的 base_link，按 `Esc` 切换到自由相机（仅限 C++）

## URDF 转换为 MuJoCo

`urdf2mjcf` 工具可将 URDF 机器人描述转换为 MuJoCo MJCF。安装方法、命令行选项
和示例请参阅 [`urdf2mjcf/README.md`](urdf2mjcf/README.md)。转换器可以在
`robots/<name>/` 下创建机器人目录；请根据需要检查并手动完善生成的 MJCF。网格文件
根据 URDF 中的网格路径解析，本仓库不包含 N1 机器人的资源文件。

## 演示

![](https://github.com/lupinjia/demo_imgs/blob/master/unitree_mujoco_demo.gif)
