"""Default directional light for generated MJCF."""

from typing import Dict, Optional

from ..core import Element, SubElement
from ..utils import _update_with_defaults

DEFAULT_LIGHT = {"pos": "0 0 1000", "castshadow": "true"}


def add_lighting(mjcf: Element, light_attrib: Optional[Dict[str, str]] = None) -> None:
    """Append default lighting to an MJCF document."""
    worldbody = mjcf.find("./worldbody")
    if worldbody is None:
        worldbody = SubElement(mjcf, "worldbody")
    SubElement(worldbody, "light", _update_with_defaults(light_attrib, DEFAULT_LIGHT))
