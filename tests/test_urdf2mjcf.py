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
        model = mujoco.MjModel.from_xml_path(str(output))
        self.assertGreater(model.nbody, 0)

    def test_ground_and_lighting_are_optional_model_elements(self):
        output = self.convert("--ground", "--lighting")
        root = ET.parse(output).getroot()
        self.assertTrue(root.findall(".//geom[@type='plane']"))
        self.assertTrue(root.findall(".//light"))

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
        sensor = root.find(".//sensor/gyro[@name='base_gyro']")
        self.assertIsNotNone(sensor)
        if sensor is None:
            self.fail("converted model is missing the configured gyro")
        site_name = sensor.get("site")
        if not site_name:
            self.fail("configured gyro does not reference a site")
        site = root.find(f".//body[@name='base']/site[@name='{site_name}']")
        self.assertIsNotNone(site)


@unittest.skipUnless(DEPENDENCIES_AVAILABLE, "requires mujoco and defusedxml")
class MeshUriResolverTests(unittest.TestCase):
    def test_collision_mesh_uri_resolves_relative_to_urdf_without_loading_asset(self):
        sys.path.insert(0, str(PACKAGE_SRC))
        try:
            from urdf2mjcf.core import resolve_uris
        finally:
            sys.path.pop(0)

        urdf = ET.fromstring(
            '<robot name="mesh"><link name="base"><collision>'
            '<geometry><mesh filename="meshes/collision.stl"/></geometry>'
            '</collision></link></robot>'
        )
        urdf_dir = Path(tempfile.gettempdir()) / "urdf2mjcf-resolver-test"

        resolve_uris(urdf, urdf_dir)

        mesh = urdf.find(".//collision/geometry/mesh")
        self.assertIsNotNone(mesh)
        if mesh is None:
            self.fail("fixture is missing its collision mesh")
        filename = mesh.get("filename")
        if filename is None:
            self.fail("fixture mesh is missing its filename")
        resolved = Path(filename)
        self.assertEqual(resolved, (urdf_dir / "meshes" / "collision.stl").resolve())
        self.assertFalse(resolved.exists(), "resolver test should not need a physical mesh")


if __name__ == "__main__":
    unittest.main()
