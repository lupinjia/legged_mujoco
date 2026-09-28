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
