"""URDF to MJCF conversion pipeline."""

from pathlib import Path
from typing import Optional, Union

from .core import (
    Element,
    SubElement,
    add_mujoco_node,
    pass_through_mujoco,
    populate_sensors,
    resolve_uris,
)
from .default_elements.ground import add_ground
from .default_elements.lighting import add_lighting


def _add_root_freejoint(mjcf: Element, root_link_names: set) -> None:
    """Add a free joint when exactly one imported URDF root body is available."""
    if len(root_link_names) != 1:
        return
    worldbody = mjcf.find("./worldbody")
    if worldbody is None:
        return
    root_name = next(iter(root_link_names))
    root_bodies = [body for body in worldbody.findall("./body") if body.get("name") == root_name]
    if len(root_bodies) != 1:
        return
    root_body = root_bodies[0]
    if root_body.find("./freejoint") is not None or root_body.find("./joint") is not None:
        return

    freejoint = Element("freejoint", {"name": f"{root_name}_free_joint"})
    if len(root_body):
        indentation = root_body.text if root_body.text and not root_body.text.strip() else "\n      "
        root_body.text = indentation
        freejoint.tail = indentation
    else:
        root_body.text = "\n      "
        freejoint.tail = "\n    "
    root_body.insert(0, freejoint)


def _assign_geom_classes(mjcf: Element, link_names: set) -> None:
    """Define collision/visual defaults and annotate imported geometry."""
    defaults = Element("default")
    collision = SubElement(defaults, "default", {"class": "collision"})
    collision_geom = SubElement(
        collision,
        "geom",
        {"contype": "1", "conaffinity": "1", "group": "3"},
    )
    visual = SubElement(defaults, "default", {"class": "visual"})
    visual_geom = SubElement(
        visual,
        "geom",
        {"contype": "0", "conaffinity": "0", "group": "1", "density": "0"},
    )
    defaults.text = "\n    "
    collision.text = "\n      "
    collision.tail = "\n    "
    collision_geom.tail = "\n    "
    visual.text = "\n      "
    visual.tail = "\n  "
    visual_geom.tail = "\n    "

    compiler = mjcf.find("./compiler")
    if compiler is not None:
        compiler_index = list(mjcf).index(compiler)
        compiler.tail = "\n\n  "
        mjcf.insert(compiler_index + 1, defaults)
    else:
        mjcf.insert(0, defaults)
    defaults.tail = "\n\n  "

    worldbody = mjcf.find("./worldbody")
    if worldbody is None:
        return
    geoms = []
    for body in worldbody.findall(".//body"):
        if body.get("name") in link_names:
            geoms.extend(body.findall("./geom"))
    geoms.extend(worldbody.findall("./geom[@name='ground_plane']"))
    for geom in geoms:
        if geom.get("class") is not None:
            continue
        if geom.get("name") == "ground_plane":
            geom.set("class", "collision")
        elif (
            geom.get("group") == "1"
            and geom.get("contype") == "0"
            and geom.get("conaffinity") == "0"
        ):
            geom.set("class", "visual")
        elif geom.get("group") in (None, "0"):
            geom.set("class", "collision")

        geom_class = geom.get("class")
        inherited_defaults = {
            "collision": {"contype": "1", "conaffinity": "1", "group": "3"},
            "visual": {
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
                "density": "0",
            },
        }.get(geom_class, {})
        for attribute, default_value in inherited_defaults.items():
            is_legacy_collision_group = (
                geom_class == "collision"
                and attribute == "group"
                and geom.get(attribute) == "0"
            )
            if geom.get(attribute) == default_value or is_legacy_collision_group:
                del geom.attrib[attribute]

    _name_link_geoms(mjcf, link_names)


def _name_link_geoms(mjcf: Element, link_names: set) -> None:
    """Name unnamed visual/collision geoms directly attached to URDF link bodies."""
    worldbody = mjcf.find("./worldbody")
    if worldbody is None:
        return
    used_names = {
        name for geom in mjcf.findall(".//geom") if (name := geom.get("name")) is not None
    }

    def unique_name(candidate: str) -> str:
        name = candidate
        suffix = 1
        while name in used_names:
            name = f"{candidate}_{suffix}"
            suffix += 1
        used_names.add(name)
        return name

    for body in worldbody.findall(".//body"):
        link_name = body.get("name")
        if link_name not in link_names:
            continue
        for geom_class in ("visual", "collision"):
            class_geoms = [
                geom for geom in body.findall("./geom") if geom.get("class") == geom_class
            ]
            unnamed_geoms = [geom for geom in class_geoms if geom.get("name") is None]
            for index, geom in enumerate(unnamed_geoms):
                base_name = f"{link_name}_{geom_class}"
                candidate = base_name if len(class_geoms) == 1 else f"{base_name}{index}"
                geom.set("name", unique_name(candidate))


def _ensure_top_level_section(mjcf: Element, tag: str) -> Element:
    section = mjcf.find(f"./{tag}")
    if section is not None:
        return section
    if tag == "actuator":
        later_sections = {"sensor", "keyframe"}
    else:
        later_sections = {"keyframe"}
    index = next(
        (i for i, child in enumerate(mjcf) if child.tag in later_sections),
        len(mjcf),
    )
    section = Element(tag)
    if index > 0:
        mjcf[index - 1].tail = "\n\n  "
    section.tail = "\n\n  "
    mjcf.insert(index, section)
    return section


def _append_section_element(section: Element, tag: str, attributes: dict) -> Element:
    child = Element(tag, attributes)
    if len(section):
        section[-1].tail = "\n    "
    else:
        section.text = "\n    "
    child.tail = "\n  "
    section.append(child)
    return child


def _ensure_named_definition(
    section: Element,
    tag: str,
    attributes: dict,
    target_attributes: tuple,
) -> None:
    name = attributes["name"]
    existing = next(
        (child for child in section.findall("./*") if child.get("name") == name),
        None,
    )
    if existing is not None:
        if existing.tag == tag and all(
            existing.get(attribute) == attributes[attribute]
            for attribute in target_attributes
        ):
            return
        # Preserve a custom definition that already owns this name.
        return
    _append_section_element(section, tag, attributes)


def _add_default_sensors_and_actuators(
    mjcf: Element,
    root_link_names: set,
    link_names: set,
) -> None:
    """Add default IMU and actuators for imported movable joints, without replacing user definitions."""
    worldbody = mjcf.find("./worldbody")
    if worldbody is None:
        return

    root_body = None
    if len(root_link_names) == 1:
        root_name = next(iter(root_link_names))
        matches = [
            body for body in worldbody.findall("./body") if body.get("name") == root_name
        ]
        if len(matches) == 1:
            root_body = matches[0]

    has_imu_site = False
    if root_body is not None:
        root_site = next(
            (site for site in root_body.findall("./site") if site.get("name") == "imu"),
            None,
        )
        global_imu_sites = [
            site for site in mjcf.findall(".//site") if site.get("name") == "imu"
        ]
        if root_site is not None:
            has_imu_site = True
        elif not global_imu_sites:
            site = Element("site", {"name": "imu"})
            indentation = root_body.text if root_body.text and not root_body.text.strip() else "\n      "
            root_body.text = indentation
            freejoint = root_body.find("./freejoint")
            if freejoint is not None:
                insert_at = list(root_body).index(freejoint) + 1
            else:
                insert_at = next(
                    (
                        index
                        for index, child in enumerate(root_body)
                        if child.tag in {"inertial", "joint", "geom", "body"}
                    ),
                    0,
                )
            if insert_at > 0:
                previous = root_body[insert_at - 1]
                previous_tail = previous.tail
                previous.tail = indentation
            else:
                previous_tail = None
            closing_indent = (
                previous_tail
                if previous_tail and not previous_tail.strip()
                else indentation[:-2] if indentation.endswith("  ") else "\n    "
            )
            site.tail = indentation if insert_at < len(root_body) else closing_indent
            root_body.insert(insert_at, site)
            has_imu_site = True

    sensor_section = None
    if has_imu_site:
        sensor_section = _ensure_top_level_section(mjcf, "sensor")
        for tag, attributes, targets in (
            ("framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu"}, ("objtype", "objname")),
            ("gyro", {"name": "imu_gyro", "site": "imu"}, ("site",)),
            ("accelerometer", {"name": "imu_accel", "site": "imu"}, ("site",)),
        ):
            _ensure_named_definition(sensor_section, tag, attributes, targets)

    imported_joints = []
    for body in worldbody.findall(".//body"):
        if body.get("name") not in link_names:
            continue
        for joint in body.findall("./joint"):
            name = joint.get("name")
            if name and joint.get("type") != "free":
                imported_joints.append(name)

    if imported_joints:
        if sensor_section is None:
            sensor_section = _ensure_top_level_section(mjcf, "sensor")
        actuator_section = _ensure_top_level_section(mjcf, "actuator")
        for joint_name in imported_joints:
            label = joint_name[:-6] if joint_name.endswith("_joint") else joint_name
            _ensure_named_definition(
                sensor_section,
                "jointpos",
                {"name": f"{label}_pos", "joint": joint_name},
                ("joint",),
            )
            _ensure_named_definition(
                sensor_section,
                "jointvel",
                {"name": f"{label}_vel", "joint": joint_name},
                ("joint",),
            )
            _ensure_named_definition(
                actuator_section,
                "motor",
                {"name": f"{label}_motor", "joint": joint_name},
                ("joint",),
            )


def full_pipeline(
    urdf_file_path: Union[str, Path],
    urdf: Element,
    mujoco_node: Optional[Element] = None,
    sensor_config: Optional[Element] = None,
    default_ground: bool = True,
    default_lighting: bool = True,
) -> Element:
    """Convert a parsed URDF document to an MJCF Element."""
    link_names = {
        name for link in urdf.findall("./link") if (name := link.get("name")) is not None
    }
    child_links = {
        name
        for joint in urdf.findall("./joint")
        if (child := joint.find("./child")) is not None
        and (name := child.get("link")) is not None
    }
    root_link_names = link_names - child_links
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
    _add_root_freejoint(mjcf, root_link_names)
    _assign_geom_classes(mjcf, link_names)
    _add_default_sensors_and_actuators(mjcf, root_link_names, link_names)
    return mjcf
