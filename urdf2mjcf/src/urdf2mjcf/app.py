"""URDF to MJCF conversion pipeline."""

from pathlib import Path
from typing import Optional, Union

from .core import Element, add_mujoco_node, pass_through_mujoco, populate_sensors, resolve_uris
from .default_elements.ground import add_ground
from .default_elements.lighting import add_lighting


def full_pipeline(
    urdf_file_path: Union[str, Path],
    urdf: Element,
    mujoco_node: Optional[Element] = None,
    sensor_config: Optional[Element] = None,
    default_ground: bool = True,
    default_lighting: bool = True,
) -> Element:
    """Convert a parsed URDF document to an MJCF Element."""
    resolve_uris(urdf, Path(urdf_file_path).expanduser().resolve().parent)
    add_mujoco_node(urdf, mujoco_node)
    mjcf = pass_through_mujoco(urdf)
    if sensor_config is not None:
        populate_sensors(mjcf, sensor_config)
        mjcf = pass_through_mujoco(mjcf)
    if default_ground:
        add_ground(mjcf)
    if default_lighting:
        add_lighting(mjcf)
    return mjcf
