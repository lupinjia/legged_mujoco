"""Command-line argument definitions for urdf2mjcf."""

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser, RawDescriptionHelpFormatter
from typing import Optional


class ArgFormatter(ArgumentDefaultsHelpFormatter, RawDescriptionHelpFormatter):
    pass


def cli(parser: Optional[ArgumentParser] = None) -> ArgumentParser:
    """Build the CLI parser."""
    parser = parser or ArgumentParser(
        prog="urdf2mjcf",
        description="Convert a URDF model into MuJoCo MJCF format.",
        formatter_class=ArgFormatter,
    )
    parser.add_argument("input", metavar="INPUT", nargs="?", default="-", help="URDF input path (default: stdin)")
    parser.add_argument("--output", default="-", metavar="OUTPUT", help="MJCF output path (default: stdout)")
    parser.add_argument("--sensor_config", "--sensor-config", dest="sensor_config", help="XML sensor configuration")
    parser.add_argument("--mujoco_node", "--mujoco-node", dest="mujoco_node", help="XML global MuJoCo configuration")
    parser.add_argument("--ground", dest="default_ground", action="store_true", help="add the default ground plane")
    parser.add_argument("--lighting", dest="default_lighting", action="store_true", help="add default lighting")
    parser.add_argument("--format", dest="format", action="store_true", help="pretty-print MJCF output")
    parser.add_argument("--version", action="version", version="1.0.0")
    return parser
