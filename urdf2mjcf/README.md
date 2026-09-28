# urdf2mjcf

`urdf2mjcf` converts a robot description in URDF format to MuJoCo MJCF. Install
the local package from the repository root:

```sh
python -m pip install -e ./urdf2mjcf
```

This provides both the `urdf2mjcf` command and the equivalent
`python -m urdf2mjcf` entry point.

## Convert a robot

For example, place the URDF and any referenced assets in a robot directory and
write the converted model alongside it:

```sh
urdf2mjcf path/to/my_robot.urdf --output robots/my_robot/my_robot.xml
```

The module form accepts the same arguments:

```sh
python -m urdf2mjcf path/to/my_robot.urdf --output robots/my_robot/my_robot.xml
```

The output is MJCF and can be loaded by MuJoCo. Depending on the robot and the
intended simulation, manual MJCF inspection and refinements may still be needed.
Mesh references are resolved using the paths in the URDF (including paths
relative to the URDF); keep referenced mesh files available when converting and
when loading the result. No N1 robot or mesh assets are bundled with this
repository.

## View a converted model

Open the generated MJCF in MuJoCo's native viewer after conversion:

```sh
urdf2mjcf path/to/my_robot.urdf --output robots/my_robot/my_robot.xml
urdf2mjcf-view robots/my_robot/my_robot.xml
```

The viewer requires a usable desktop graphics display (including GLFW/OpenGL
support). Launching blocks the terminal until the viewer window is closed. Mesh
assets referenced by the MJCF must remain accessible relative to the model file.
Alternatively, MuJoCo's built-in module can be used directly:
`python -m mujoco.viewer --mjcf=robots/my_robot/my_robot.xml`.

Visual geometry is retained by default alongside collision geometry. A MuJoCo
configuration supplied with `--mujoco_node` can intentionally set
`<compiler discardvisual="true"/>` to omit visual geometry.

When the URDF has exactly one root link with no incoming joint and its converted
body has no joint, the converter adds a free joint named `<root>_free_joint`.
This gives the model a floating base (7 position coordinates and 6 degrees of
freedom); constrain the base in MJCF if the simulation requires it to be fixed.
Models with ambiguous roots or no matching root body are left unchanged.

Generated MJCF defines `collision` and `visual` geometry classes under the
top-level `<default>` element and assigns them to imported geoms. Visual geoms
retain their non-contacting visual settings; collision geoms (including the
optional ground plane) use collision settings. Existing custom geom classes
are left unchanged.

Unnamed imported geoms are named from their enclosing link body: a single visual
or collision geom gets `<link>_visual` or `<link>_collision`; multiple geoms of
either class receive zero-based suffixes (for example, `<link>_collision0`).
Explicit geom names are preserved, and generated names are adjusted if needed
to avoid collisions with existing geom names.

For a unique root link, conversion also adds an `imu` site with frame-quaternion,
gyroscope, and accelerometer sensors unless a conflicting `imu` site already
exists elsewhere. Each imported named non-free joint receives position and
velocity sensors plus a motor actuator; fixed joints omitted by MuJoCo are not
given sensors or actuators. Existing sensor and actuator definitions are retained
and reused when they already match a generated name and target.

## Options

- `--sensor_config PATH`: add sensors described by a sensor configuration file.
- `--mujoco_node PATH`: provide an XML file containing global MuJoCo configuration
  (for example, `<mujoco><option timestep="0.002"/></mujoco>`).
- `--ground`: include a ground plane.
- `--lighting`: include lighting in the generated model.
- `--format`: pretty-print the MJCF output (requires the optional `lxml` package).

The sensor configuration is XML. Put sensor sites under named bodies and sensors
in a `<sensor>` element, for example:

```xml
<mujoco>
  <body name="base"><site name="gyro_site"/></body>
  <sensor><gyro name="base_gyro" site="gyro_site"/></sensor>
</mujoco>
```

Pass this file with `--sensor_config sensors.xml`; the named body must exist in
the converted model. For example, to write `robots/my_robot/my_robot.xml`:

```sh
urdf2mjcf path/to/my_robot.urdf --output robots/my_robot/my_robot.xml \
  --sensor_config path/to/sensors.xml \
  --mujoco_node path/to/mujoco_config.xml --ground --lighting
```

Run `urdf2mjcf --help` for the complete argument syntax and defaults.

## Attribution and licensing

This package is adapted from the upstream `Wiki-GRx-MJCF` project. The
upstream Apache License, Version 2.0, is included in `LICENSE`. The adapted
`core.py` also retains code covered by the separate Fraunhofer IPA
BSD-3-Clause notice included at the end of that file. These notices apply to
their respective code; this package does not assert a single license for every
file.
