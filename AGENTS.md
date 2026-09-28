# Project notes

- The Python URDF-to-MJCF converter is a separate package in `urdf2mjcf/`; install with `python -m pip install -e ./urdf2mjcf` (add `[format]` for optional `--format` support).
- Run converter checks with `python -m unittest discover -s tests -p 'test_urdf2mjcf.py'`. The tests require the converter's MuJoCo and defusedxml dependencies; without them, the integration cases skip.
- Convert with `urdf2mjcf input.urdf --output robots/<name>/<name>.xml`. Optional `--sensor_config` and `--mujoco_node` both take XML file paths. Mesh assets must remain accessible; review generated MJCF before using it in a simulation.
- The converter is adapted from `~/Wiki-GRx-MJCF`; keep its upstream Apache license and the Fraunhofer BSD notice in `core.py` when modifying or distributing it.
