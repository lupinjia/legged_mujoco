# Project notes

- The Python URDF-to-MJCF converter is a separate package in `urdf2mjcf/`; install with `python -m pip install -e ./urdf2mjcf` (add `[format]` for optional `--format` support).
- Run converter checks with `python -m unittest discover -s tests -p 'test_urdf2mjcf.py'`. The tests require the converter's MuJoCo and defusedxml dependencies; without them, the integration cases skip.
- Convert with `urdf2mjcf input.urdf --output robots/<name>/<name>.xml`. Optional `--sensor_config` and `--mujoco_node` both take XML file paths. Mesh assets must remain accessible; review generated MJCF before using it in a simulation.
- Open a converted model with `urdf2mjcf-view robots/<name>/<name>.xml`; it calls MuJoCo's blocking native viewer and needs a graphical desktop. In headless tests, mock `mujoco.viewer.launch_from_path` rather than opening a window.
- Keep `<compiler discardvisual="false"/>` by default when importing URDF: MuJoCo otherwise drops visual geoms. Resolve both visual and collision mesh paths relative to the URDF; explicit compiler overrides may still disable visuals.
- Assign exported visual/collision geom classes only after MuJoCo's XML pass-through, which serializes explicit geom properties; do not leave duplicate per-geom attributes that would override class defaults. The collision class uses `group="3"` (visual uses `group="1"`).
- MuJoCo's saved XML already contains indentation whitespace; give newly inserted `<default>` nodes explicit `text`/`tail` whitespace so the normal (non-`--format`) serializer keeps them on separate indented lines.
- After all MuJoCo pass-throughs, add one freejoint to the unique unjointed URDF root body (unless it already has a joint); MuJoCo then gives that floating base 7 position coordinates and 6 velocity degrees of freedom.
- Name unnamed imported geoms after their link and class: `<link>_visual`/`<link>_collision` for singles, or zero-based numeric suffixes for multiples; preserve explicit names and avoid global collisions.
- Add default IMU site/sensors and named joint position/velocity sensors plus motors after the final MuJoCo pass-through. Anchor IMU sensors to the unique root body's `imu` site; skip free/fixed joints, strip a terminal `_joint` for sensor/motor names, and preserve explicit sensor/actuator definitions.
- The converter is adapted from `~/Wiki-GRx-MJCF`; keep its upstream Apache license and the Fraunhofer BSD notice in `core.py` when modifying or distributing it.
