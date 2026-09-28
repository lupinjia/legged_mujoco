# legged_mujoco

The architecture is the same as [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco). 

## Additional Dependencies

- [mujoco 3.2.7](https://github.com/google-deepmind/mujoco/tree/3.2.7)

## Features

- Add support for Yahboom USB wireless joystick and betop USB joystick (C++)

  Added joystick mapping code (simulate/src/physics_joystick.h) according to the proxy of [Yahboom USB Wireless Joystick](https://yahboom.com/study_module/PS2) and betop USB joystick

  Change the `joystick_type` in `simulate/config.yaml` to other values if you want to use joystick of other types

- Press `Enter` to track base_link of the robot, press `Esc` to use free camera (only in C++)

## URDF to MuJoCo

The `urdf2mjcf` helper converts a URDF robot description to MuJoCo MJCF. See
[`urdf2mjcf/README.md`](urdf2mjcf/README.md) for installation, command-line options,
and an example. The converter can create a robot directory under `robots/<name>/`;
review and refine the generated MJCF manually where needed. Mesh files are resolved
from the URDF's mesh paths, and this repository does not include N1 robot assets.

## Demo

![](https://github.com/lupinjia/demo_imgs/blob/master/unitree_mujoco_demo.gif)
