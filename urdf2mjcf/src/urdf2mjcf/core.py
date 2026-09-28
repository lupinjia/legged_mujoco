"""Core URDF-to-MJCF conversion helpers.

Copyright (c) 2022 Fraunhofer IPA; see the license notice at the bottom.
"""

from os import close, unlink
from pathlib import Path
from tempfile import mkstemp
from typing import AnyStr, Dict, IO, Optional, Union
from urllib.parse import urlparse
from xml.etree.ElementTree import Element, SubElement, tostring

from defusedxml.ElementTree import parse
from mujoco import MjModel, mj_saveLastXML


def resolve_uris(urdf: Element, base_path: Optional[Union[str, Path]] = None) -> None:
    """Resolve visual and collision mesh URIs relative to the source URDF directory."""
    base = Path(base_path).expanduser().resolve() if base_path is not None else None
    for mesh_node in urdf.findall(".//visual/*/mesh[@filename]") + urdf.findall(
        ".//collision/*/mesh[@filename]"
    ):
        uri = mesh_node.get("filename")
        if uri is None:
            raise ValueError(f"Mesh node without filename: '{mesh_node.tag}' : {mesh_node.attrib}")
        if base is None:
            continue
        parsed = urlparse(uri)
        if parsed.scheme == "file":
            mesh_path = Path(parsed.path)
        elif parsed.scheme:
            # Preserve non-file URIs (for example package://) for MuJoCo to interpret.
            continue
        else:
            mesh_path = Path(uri)
        if not mesh_path.is_absolute():
            mesh_path = base / mesh_path
        mesh_node.set("filename", str(mesh_path.resolve()))


def parse_element(source: Union[str, Path, IO[AnyStr], None], **kwargs) -> Optional[Element]:
    """Safely parse an XML source to an Element; ``None`` remains ``None``."""
    return None if source is None else parse(source, **kwargs).getroot()  # type: ignore[arg-type]


def pass_through_mujoco(model_xml: Element) -> Element:
    """Load and export XML through MuJoCo, removing the temporary file safely."""
    parsed_model = MjModel.from_xml_string(tostring(model_xml, encoding="unicode"))
    tmp_fd, tmp_name = mkstemp(prefix="tmp_mjcf_", suffix=".xml")
    close(tmp_fd)
    try:
        mj_saveLastXML(tmp_name, parsed_model)
        backloaded_model_xml = parse_element(Path(tmp_name))
        if backloaded_model_xml is None:
            raise ValueError("MuJoCo produced an empty XML document")
        return backloaded_model_xml
    finally:
        try:
            unlink(tmp_name)
        except FileNotFoundError:
            pass


def add_mujoco_node(urdf: Element, mujoco_node: Optional[Element] = None) -> None:
    """Add compiler and global MuJoCo configuration nodes to a URDF."""
    mujoco_node = Element("mujoco") if mujoco_node is None else mujoco_node
    existing = urdf.find("./mujoco")
    if existing is not None:
        urdf.remove(existing)

    attributes: Dict[str, Dict[str, str]] = {
        "./compiler": {"strippath": "false", "fusestatic": "false", "discardvisual": "false"},
        "./compiler/lengthrange": {},
        "./option": {},
        "./option/flag": {},
        "./size": {},
    }
    for node in (existing, mujoco_node):
        if node is None:
            continue
        for xpath, attrib in attributes.items():
            for configured in node.findall(xpath):
                attrib.update(configured.attrib)

    node = SubElement(urdf, "mujoco")
    compiler = SubElement(node, "compiler", attributes["./compiler"])
    SubElement(compiler, "lengthrange", attributes["./compiler/lengthrange"])
    option = SubElement(node, "option", attributes["./option"])
    SubElement(option, "flag", attributes["./option/flag"])
    SubElement(node, "size", attributes["./size"])


def populate_sensors(mjcf: Element, sensor_config: Element) -> None:
    """Add sensor configuration sites and sensors to an MJCF document."""
    for body_node in sensor_config.findall("./body"):
        body_name = body_node.get("name")
        if body_name is None:
            raise ValueError(f"Bad sensor configuration; body node has no name ({body_node.attrib})")
        body_in_mjcf = mjcf.find(f".//body[@name='{body_name}']")
        if body_in_mjcf is None:
            raise ValueError(f"No body in MJCF with name '{body_name}'")
        body_in_mjcf.extend(body_node.findall("./site"))
    mjcf.extend(sensor_config.findall("./sensor"))


# Copyright (c) 2022 Fraunhofer IPA
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
#    and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse
#    or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
