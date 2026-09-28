"""Default ground plane for generated MJCF."""

from typing import Dict, Optional

from ..core import Element, SubElement
from ..utils import _update_with_defaults

DEFAULT_GROUND_TEXTURE = {
    "name": "texplane", "builtin": "checker", "height": "512", "width": "512",
    "rgb1": ".2 .3 .4", "rgb2": ".1 .15 .2", "type": "2d",
}
DEFAULT_GROUND_MATERIAL = {
    "name": "MatPlane", "reflectance": "0.5", "shininess": "0.01", "specular": "0.1",
    "texrepeat": "1 1", "texture": "texplane", "texuniform": "true",
}
DEFAULT_GROUND_GEOM = {
    "name": "ground_plane", "type": "plane", "size": "5 5 10", "material": "MatPlane", "rgba": "1 1 1 1",
}


def add_ground(
    mjcf: Element,
    texture_attrib: Optional[Dict[str, str]] = None,
    material_attrib: Optional[Dict[str, str]] = None,
    geom_attrib: Optional[Dict[str, str]] = None,
) -> None:
    """Append the default checkerboard ground plane to an MJCF document."""
    asset = SubElement(mjcf, "asset")
    SubElement(asset, "texture", _update_with_defaults(texture_attrib, DEFAULT_GROUND_TEXTURE))
    SubElement(asset, "material", _update_with_defaults(material_attrib, DEFAULT_GROUND_MATERIAL))
    worldbody = mjcf.find("./worldbody")
    if worldbody is None:
        worldbody = SubElement(mjcf, "worldbody")
    SubElement(worldbody, "geom", _update_with_defaults(geom_attrib, DEFAULT_GROUND_GEOM))
