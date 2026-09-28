"""Integration checks for the optional urdf2mjcf package."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = ROOT / "urdf2mjcf" / "src"
DEPENDENCIES_AVAILABLE = all(
    importlib.util.find_spec(name) is not None for name in ("mujoco", "defusedxml")
)


@unittest.skipUnless(DEPENDENCIES_AVAILABLE, "requires mujoco and defusedxml")
class Urdf2MjcfIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.work = Path(self.temp_dir.name)
        self.urdf = self.work / "minimal.urdf"
        self.urdf.write_text(
            '''<?xml version="1.0"?>
<robot name="minimal">
  <link name="base">
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="arm">
    <inertial><mass value="0.2"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <joint name="arm_joint" type="fixed">
    <parent link="base"/><child link="arm"/><origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
''',
            encoding="utf-8",
        )

    def convert(self, *options):
        output = self.work / "robots" / "minimal" / "minimal.xml"
        output.parent.mkdir(parents=True, exist_ok=True)
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join(
            [str(PACKAGE_SRC), env.get("PYTHONPATH", "")]
        ).rstrip(os.pathsep)
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "urdf2mjcf",
                str(self.urdf),
                "--output",
                str(output),
                *options,
            ],
            cwd=self.work,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)
        self.assertTrue(output.is_file(), msg=result.stdout + result.stderr)
        return output

    def test_mesh_free_urdf_converts_to_loadable_mjcf(self):
        import mujoco

        output = self.convert()
        root = ET.parse(output).getroot()
        base = root.find("./worldbody/body[@name='base']")
        self.assertIsNotNone(base)
        if base is None:
            self.fail("converted model is missing the URDF root body")
        self.assertEqual(base[0].tag, "freejoint")
        self.assertEqual(base[0].get("name"), "base_free_joint")
        self.assertEqual(len(root.findall(".//freejoint")), 1)
        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.app import _add_root_freejoint
        finally:
            sys.path.pop(0)
        _add_root_freejoint(root, {"base"})
        self.assertEqual(len(root.findall(".//freejoint")), 1)

        model = mujoco.MjModel.from_xml_path(str(output))
        self.assertGreater(model.nbody, 0)
        self.assertEqual(model.nq, 7)
        self.assertEqual(model.nv, 6)

    def test_root_freejoint_skips_ambiguous_or_missing_root_bodies(self):
        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.app import _add_root_freejoint
        finally:
            sys.path.pop(0)

        multiple_roots = ET.fromstring(
            '<mujoco><worldbody><body name="first"/><body name="second"/></worldbody></mujoco>'
        )
        _add_root_freejoint(multiple_roots, {"first", "second"})
        self.assertFalse(multiple_roots.findall(".//freejoint"))

        missing_root = ET.fromstring('<mujoco><worldbody><body name="other"/></worldbody></mujoco>')
        _add_root_freejoint(missing_root, {"root"})
        self.assertFalse(missing_root.findall(".//freejoint"))

    def test_visual_and_collision_geometries_are_preserved_by_default(self):
        import mujoco

        self.urdf.write_text(
            '''<robot name="geometry">
  <link name="visual_link">
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
    <visual><geometry><box size="0.2 0.3 0.4"/></geometry></visual>
  </link>
  <link name="collision_link">
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
    <collision><geometry><sphere radius="0.15"/></geometry></collision>
  </link>
</robot>''',
            encoding="utf-8",
        )

        output = self.convert()
        root = ET.parse(output).getroot()
        lines = output.read_text(encoding="utf-8").splitlines()
        compiler_line = next(i for i, line in enumerate(lines) if line.startswith("  <compiler"))
        default_line = next(i for i, line in enumerate(lines) if line == "  <default>")
        self.assertLess(compiler_line, default_line)
        self.assertEqual(lines[default_line + 1], '    <default class="collision">')
        self.assertEqual(lines[default_line + 2], '      <geom contype="1" conaffinity="1" group="3" />')
        self.assertEqual(lines[default_line + 3], "    </default>")
        self.assertEqual(lines[default_line + 4], '    <default class="visual">')
        self.assertEqual(
            lines[default_line + 5],
            '      <geom contype="0" conaffinity="0" group="1" density="0" />',
        )
        self.assertEqual(lines[default_line + 6], "    </default>")
        self.assertEqual(lines[default_line + 7], "  </default>")
        self.assertEqual(lines[default_line + 8], "")
        defaults = root.find("./default")
        self.assertIsNotNone(defaults)
        if defaults is None:
            self.fail("converted model is missing geometry class defaults")
        self.assertIsNotNone(defaults.find("./default[@class='collision']/geom"))
        self.assertIsNotNone(defaults.find("./default[@class='visual']/geom"))
        geoms = root.findall("./worldbody//geom")
        visual_geoms = [geom for geom in geoms if geom.get("class") == "visual"]
        collision_geoms = [geom for geom in geoms if geom.get("class") == "collision"]
        self.assertTrue(any(geom.get("class") == "visual" for geom in visual_geoms))
        self.assertTrue(any(geom.get("class") == "collision" for geom in collision_geoms))
        self.assertTrue(any(geom.get("type") == "box" for geom in visual_geoms))
        self.assertTrue(any(geom.get("type") is None for geom in collision_geoms))
        self.assertEqual(visual_geoms[0].get("name"), "visual_link_visual")
        self.assertEqual(collision_geoms[0].get("name"), "collision_link_collision")
        for geom in visual_geoms:
            for attribute in ("contype", "conaffinity", "group", "density"):
                self.assertIsNone(geom.get(attribute), f"visual {attribute} should inherit its class default")
        for geom in collision_geoms:
            for attribute in ("contype", "conaffinity", "group"):
                self.assertIsNone(geom.get(attribute), f"collision {attribute} should inherit its class default")
        model = mujoco.MjModel.from_xml_path(str(output))
        self.assertGreaterEqual(model.ngeom, 2)
        body_ids = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id): body_id
            for body_id in range(model.nbody)
        }
        visual_body = body_ids["visual_link"]
        collision_body = body_ids["collision_link"]
        visual_id = next(
            geom_id
            for geom_id in range(model.ngeom)
            if model.geom_bodyid[geom_id] == visual_body
        )
        collision_id = next(
            geom_id
            for geom_id in range(model.ngeom)
            if model.geom_bodyid[geom_id] == collision_body
        )
        self.assertEqual(
            (model.geom_contype[visual_id], model.geom_conaffinity[visual_id], model.geom_group[visual_id]),
            (0, 0, 1),
        )
        self.assertEqual(
            (model.geom_contype[collision_id], model.geom_conaffinity[collision_id], model.geom_group[collision_id]),
            (1, 1, 3),
        )

        visual_default_geom = root.find("./default/default[@class='visual']/geom")
        self.assertIsNotNone(visual_default_geom)
        if visual_default_geom is None:
            self.fail("visual class is missing its geom defaults")
        visual_default_geom.set("group", "4")
        changed_output = output.with_name("changed-default.xml")
        ET.ElementTree(root).write(changed_output, encoding="utf-8", xml_declaration=True)
        changed_model = mujoco.MjModel.from_xml_path(str(changed_output))
        self.assertEqual(changed_model.geom_group[visual_id], 4)

    def test_multiple_link_geometries_get_indexed_unique_names(self):
        import mujoco

        self.urdf.write_text(
            '''<robot name="multi">
  <link name="multi_link">
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
    <visual><geometry><sphere radius="0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
</robot>''',
            encoding="utf-8",
        )
        output = self.convert()
        root = ET.parse(output).getroot()
        geoms = root.findall("./worldbody/body[@name='multi_link']/geom")
        names_by_class = {
            geom_class: [geom.get("name") for geom in geoms if geom.get("class") == geom_class]
            for geom_class in ("visual", "collision")
        }
        self.assertEqual(names_by_class["visual"], ["multi_link_visual0", "multi_link_visual1"])
        self.assertEqual(
            names_by_class["collision"],
            ["multi_link_collision0", "multi_link_collision1"],
        )
        model = mujoco.MjModel.from_xml_path(str(output))
        model_geom_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            for geom_id in range(model.ngeom)
        ]
        self.assertEqual(len(model_geom_names), len(set(model_geom_names)))
        self.assertEqual(set(model_geom_names), set(names_by_class["visual"] + names_by_class["collision"]))

    def test_user_compiler_discardvisual_override_is_preserved(self):
        mujoco_config = self.work / "mujoco.xml"
        mujoco_config.write_text(
            '<mujoco><compiler discardvisual="true"/></mujoco>', encoding="utf-8"
        )
        output = self.convert("--mujoco_node", str(mujoco_config))
        root = ET.parse(output).getroot()
        geoms = root.findall("./worldbody//geom")
        self.assertFalse(any(geom.get("class") == "visual" for geom in geoms))
        self.assertTrue(any(geom.get("class") == "collision" for geom in geoms))

    def test_ground_and_lighting_are_optional_model_elements(self):
        output = self.convert("--ground", "--lighting")
        root = ET.parse(output).getroot()
        self.assertTrue(root.findall(".//geom[@type='plane']"))
        self.assertEqual(root.find(".//geom[@type='plane']").get("class"), "collision")
        self.assertEqual(len(root.findall(".//freejoint")), 1)
        self.assertTrue(root.findall(".//light"))
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(output))
        plane_id = next(
            geom_id
            for geom_id in range(model.ngeom)
            if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) == "ground_plane"
        )
        self.assertEqual(
            (model.geom_contype[plane_id], model.geom_conaffinity[plane_id], model.geom_group[plane_id]),
            (1, 1, 3),
        )

    def test_sensor_configuration_attaches_to_named_body(self):
        sensor_config = self.work / "sensors.xml"
        sensor_config.write_text(
            """<mujoco>
  <body name="base"><site name="gyro_site"/></body>
  <sensor><gyro name="base_gyro" site="gyro_site"/></sensor>
</mujoco>""",
            encoding="utf-8",
        )
        mujoco_config = self.work / "mujoco.xml"
        mujoco_config.write_text(
            '<mujoco><option timestep="0.002"/></mujoco>', encoding="utf-8"
        )
        output = self.convert(
            "--sensor_config",
            str(sensor_config),
            "--mujoco_node",
            str(mujoco_config),
        )
        root = ET.parse(output).getroot()
        self.assertIsNotNone(root.find("./default/default[@class='collision']/geom"))
        self.assertIsNotNone(root.find("./default/default[@class='visual']/geom"))
        self.assertEqual(len(root.findall(".//freejoint")), 1)
        sensor = root.find(".//sensor/gyro[@name='base_gyro']")
        self.assertIsNotNone(sensor)
        if sensor is None:
            self.fail("converted model is missing the configured gyro")
        site_name = sensor.get("site")
        if not site_name:
            self.fail("configured gyro does not reference a site")
        site = root.find(f".//body[@name='base']/site[@name='{site_name}']")
        self.assertIsNotNone(site)
        import mujoco

        mujoco.MjModel.from_xml_path(str(output))

    def test_default_imu_joint_sensors_and_motors_coexist_with_custom_sensor(self):
        import mujoco

        self.urdf.write_text(
            '''<robot name="sensed">
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
  <link name="arm"><inertial><mass value="0.5"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
  <link name="hand"><inertial><mass value="0.2"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
  <link name="fixed_link"/>
  <joint name="shoulder_joint" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="20" velocity="2"/></joint>
  <joint name="wrist" type="continuous"><parent link="arm"/><child link="hand"/><axis xyz="0 1 0"/><limit effort="10" velocity="2"/></joint>
  <joint name="fixed_mount" type="fixed"><parent link="base"/><child link="fixed_link"/></joint>
</robot>''',
            encoding="utf-8",
        )
        sensor_config = self.work / "custom_sensors.xml"
        sensor_config.write_text(
            '''<mujoco>
  <body name="base"><site name="custom_imu"/></body>
  <sensor>
    <jointpos name="shoulder_pos" joint="shoulder_joint"/>
    <gyro name="imu_gyro" site="custom_imu"/>
  </sensor>
</mujoco>''',
            encoding="utf-8",
        )
        output = self.convert("--sensor_config", str(sensor_config))
        root = ET.parse(output).getroot()
        root_body = root.find("./worldbody/body[@name='base']")
        self.assertIsNotNone(root_body)
        self.assertIsNotNone(root_body.find("./site[@name='imu']"))
        lines = output.read_text(encoding="utf-8").splitlines()
        root_body_line = next(i for i, line in enumerate(lines) if line == '    <body name="base">')
        imu_site_line = next(i for i, line in enumerate(lines) if line == '      <site name="imu" />')
        inertial_line = next(i for i, line in enumerate(lines) if line.startswith("      <inertial"))
        self.assertLess(root_body_line, imu_site_line)
        self.assertLess(imu_site_line, inertial_line)
        body_depth = 0
        root_body_close = None
        for index in range(root_body_line, len(lines)):
            if "<body " in lines[index] and not lines[index].rstrip().endswith("/>"):
                body_depth += 1
            if "</body>" in lines[index]:
                body_depth -= 1
                if body_depth == 0:
                    root_body_close = lines[index]
                    break
        self.assertEqual(root_body_close, "    </body>")

        sensor_nodes = root.findall("./sensor/*")
        sensors_by_name = {sensor.get("name"): sensor for sensor in sensor_nodes}
        self.assertEqual(
            set(sensors_by_name),
            {
                "imu_quat",
                "imu_gyro",
                "imu_accel",
                "shoulder_pos",
                "shoulder_vel",
                "wrist_pos",
                "wrist_vel",
            },
        )
        self.assertEqual(sensors_by_name["imu_quat"].tag, "framequat")
        self.assertEqual(sensors_by_name["imu_quat"].get("objtype"), "site")
        self.assertEqual(sensors_by_name["imu_quat"].get("objname"), "imu")
        self.assertEqual(sensors_by_name["imu_gyro"].get("site"), "custom_imu")
        self.assertEqual(sensors_by_name["imu_accel"].get("site"), "imu")
        self.assertEqual(sensors_by_name["shoulder_pos"].get("joint"), "shoulder_joint")
        self.assertEqual(sensors_by_name["shoulder_vel"].get("joint"), "shoulder_joint")
        self.assertEqual(sensors_by_name["wrist_pos"].get("joint"), "wrist")
        self.assertEqual(sensors_by_name["wrist_vel"].get("joint"), "wrist")
        self.assertNotIn("fixed_mount_pos", sensors_by_name)
        actuators = root.findall("./actuator/*")
        self.assertEqual(
            [(actuator.tag, actuator.get("name"), actuator.get("joint")) for actuator in actuators],
            [
                ("motor", "shoulder_motor", "shoulder_joint"),
                ("motor", "wrist_motor", "wrist"),
            ],
        )

        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.app import _add_default_sensors_and_actuators
        finally:
            sys.path.pop(0)
        _add_default_sensors_and_actuators(root, {"base"}, {"base", "arm", "hand", "fixed_link"})
        self.assertEqual(len(root.findall("./sensor/*")), 7)
        self.assertEqual(len(root.findall("./actuator/*")), 2)

        model = mujoco.MjModel.from_xml_path(str(output))
        self.assertEqual(model.nsensor, 7)
        self.assertEqual(model.nsensordata, 14)
        self.assertEqual(model.nu, 2)
        model_sensor_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id)
            for sensor_id in range(model.nsensor)
        ]
        self.assertEqual(len(model_sensor_names), len(set(model_sensor_names)))

    def test_ambiguous_root_skips_imu_but_keeps_joint_defaults(self):
        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.app import _add_default_sensors_and_actuators
        finally:
            sys.path.pop(0)

        mjcf = ET.fromstring(
            '<mujoco><worldbody><body name="root_a"><joint name="joint_a"/></body>'
            '<body name="root_b"><joint name="joint_b"/></body></worldbody>'
            '<actuator><motor name="custom_motor" joint="joint_a"/></actuator></mujoco>'
        )
        _add_default_sensors_and_actuators(mjcf, {"root_a", "root_b"}, {"root_a", "root_b"})
        self.assertFalse(mjcf.findall(".//site[@name='imu']"))
        self.assertFalse(mjcf.findall("./sensor/framequat"))
        self.assertEqual(
            {(sensor.tag, sensor.get("name"), sensor.get("joint")) for sensor in mjcf.findall("./sensor/*")},
            {
                ("jointpos", "joint_a_pos", "joint_a"),
                ("jointvel", "joint_a_vel", "joint_a"),
                ("jointpos", "joint_b_pos", "joint_b"),
                ("jointvel", "joint_b_vel", "joint_b"),
            },
        )
        actuator_names = [actuator.get("name") for actuator in mjcf.findall("./actuator/*")]
        self.assertEqual(actuator_names, ["custom_motor", "joint_a_motor", "joint_b_motor"])

    def test_custom_geometries_are_not_classified(self):
        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.app import _assign_geom_classes
        finally:
            sys.path.pop(0)

        mjcf = ET.fromstring(
            '<mujoco><worldbody><body name="robot_link">'
            '<geom name="recognized_visual" group="1" contype="0" conaffinity="0" density="0" friction="0.2 0.1 0.01"/>'
            '<geom name="recognized_collision" contype="1" conaffinity="1" group="0" friction="0.3 0.2 0.02"/>'
            '</body><geom name="custom" type="sphere" size="1"/>'
            '<geom name="custom_group" group="2" type="sphere" size="1"/>'
            '<geom name="already_classed" class="custom" type="sphere" size="1"/></worldbody></mujoco>'
        )
        _assign_geom_classes(mjcf, {"robot_link"})
        visual = mjcf.find(".//geom[@name='recognized_visual']")
        collision = mjcf.find(".//geom[@name='recognized_collision']")
        self.assertEqual(visual.get("class"), "visual")
        self.assertEqual(collision.get("class"), "collision")
        for attribute in ("contype", "conaffinity", "group", "density"):
            self.assertIsNone(visual.get(attribute))
        for attribute in ("contype", "conaffinity", "group"):
            self.assertIsNone(collision.get(attribute))
        self.assertEqual(visual.get("friction"), "0.2 0.1 0.01")
        self.assertEqual(collision.get("friction"), "0.3 0.2 0.02")
        self.assertIsNone(mjcf.find(".//geom[@name='custom']").get("class"))
        self.assertIsNone(mjcf.find(".//geom[@name='custom_group']").get("class"))
        self.assertEqual(mjcf.find(".//geom[@name='already_classed']").get("class"), "custom")

    def test_geom_naming_preserves_existing_names_and_is_idempotent(self):
        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.app import _name_link_geoms
        finally:
            sys.path.pop(0)

        mjcf = ET.fromstring(
            '<mujoco><worldbody><body name="link">'
            '<geom name="explicit_visual" class="visual"/>'
            '<geom class="visual"/><geom class="collision"/>'
            '</body><geom name="link_collision" type="sphere" size="1"/></worldbody></mujoco>'
        )
        _name_link_geoms(mjcf, {"link"})
        _name_link_geoms(mjcf, {"link"})
        self.assertEqual(mjcf.find(".//geom[@name='explicit_visual']").get("name"), "explicit_visual")
        visual_names = [
            geom.get("name")
            for geom in mjcf.findall("./worldbody/body/geom[@class='visual']")
        ]
        self.assertEqual(visual_names, ["explicit_visual", "link_visual0"])
        self.assertEqual(mjcf.find("./worldbody/body/geom[@class='collision']").get("name"), "link_collision_1")
        self.assertEqual(mjcf.find("./worldbody/geom").get("name"), "link_collision")


@unittest.skipUnless(DEPENDENCIES_AVAILABLE, "requires mujoco and defusedxml")
class MeshUriResolverTests(unittest.TestCase):
    def test_visual_and_collision_mesh_uris_resolve_relative_to_urdf(self):
        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.core import resolve_uris
        finally:
            sys.path.pop(0)

        urdf = ET.fromstring(
            '<robot name="mesh"><link name="base">'
            '<visual><geometry><mesh filename="meshes/visual.stl"/></geometry></visual>'
            '<collision><geometry><mesh filename="meshes/collision.stl"/></geometry></collision>'
            '</link></robot>'
        )
        urdf_dir = Path(tempfile.gettempdir()) / "urdf2mjcf-resolver-test"

        resolve_uris(urdf, urdf_dir)

        for kind in ("visual", "collision"):
            with self.subTest(kind=kind):
                mesh = urdf.find(f".//{kind}/geometry/mesh")
                self.assertIsNotNone(mesh)
                if mesh is None:
                    self.fail(f"fixture is missing its {kind} mesh")
                filename = mesh.get("filename")
                if filename is None:
                    self.fail(f"fixture {kind} mesh is missing its filename")
                resolved = Path(filename)
                self.assertEqual(
                    resolved,
                    (urdf_dir / "meshes" / f"{kind}.stl").resolve(),
                )
                self.assertFalse(resolved.exists(), "resolver test should not need a physical mesh")


class MuJoCoViewerCliTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(PACKAGE_SRC))
        from urdf2mjcf import view

        cls.view = view

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(PACKAGE_SRC))

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.model = Path(self.temp_dir.name) / "model.xml"
        self.model.write_text("<mujoco/>", encoding="utf-8")

    def test_launch_uses_resolved_path_and_returns_success(self):
        from unittest.mock import patch

        with patch("mujoco.viewer.launch_from_path") as launch:
            result = self.view.main([str(self.model)])
        self.assertEqual(result, 0)
        launch.assert_called_once_with(str(self.model.resolve()))

    def test_missing_model_reports_actionable_argument_error(self):
        from contextlib import redirect_stderr
        from io import StringIO

        stderr = StringIO()
        with redirect_stderr(stderr), self.assertRaises(SystemExit) as error:
            self.view.main([str(self.model.parent / "missing.xml")])
        self.assertEqual(error.exception.code, 2)
        self.assertIn("model file does not exist", stderr.getvalue())

    def test_help_does_not_launch_viewer(self):
        from contextlib import redirect_stdout
        from io import StringIO

        stdout = StringIO()
        with redirect_stdout(stdout), self.assertRaises(SystemExit) as error:
            self.view.main(["--help"])
        self.assertEqual(error.exception.code, 0)
        self.assertIn("MODEL.xml", stdout.getvalue())

    def test_installed_console_entrypoint_help(self):
        executable = Path(sys.executable).parent / "urdf2mjcf-view"
        if not executable.is_file():
            self.skipTest("urdf2mjcf-view console script is not installed in this environment")
        result = subprocess.run(
            [str(executable), "--help"], capture_output=True, text=True, check=False
        )
        self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)
        self.assertIn("MODEL.xml", result.stdout)


if __name__ == "__main__":
    unittest.main()
