"""CLI execution and XML output handling."""

import sys
from pathlib import Path
from typing import Optional
from xml.etree.ElementTree import tostring

from .cli import cli


def _read_xml(path: str):
    from .core import parse_element

    if path == "-":
        return parse_element(sys.stdin.buffer)
    with open(path, "rb") as stream:
        return parse_element(stream)


def _serialize(element, pretty: bool) -> bytes:
    xml = tostring(element, encoding="UTF-8", xml_declaration=True)
    if pretty:
        try:
            from lxml import etree
        except ImportError as exc:
            raise RuntimeError("--format requires the optional 'lxml' dependency") from exc
        return etree.tostring(etree.fromstring(xml), pretty_print=True, xml_declaration=True, encoding="UTF-8")
    return xml


def main(argv: Optional[list] = None) -> int:
    args = cli().parse_args(argv)
    try:
        if args.input == "-":
            input_path = Path.cwd() / "stdin.urdf"
        else:
            input_path = Path(args.input).expanduser().resolve()
        urdf = _read_xml(args.input)
        if urdf is None:
            raise ValueError("input URDF is empty")
        sensor = _read_xml(args.sensor_config) if args.sensor_config else None
        mujoco_node = _read_xml(args.mujoco_node) if args.mujoco_node else None
        from .app import full_pipeline

        result = full_pipeline(
            urdf_file_path=input_path,
            urdf=urdf,
            sensor_config=sensor,
            mujoco_node=mujoco_node,
            default_ground=args.default_ground,
            default_lighting=args.default_lighting,
        )
        output = _serialize(result, args.format)
        if args.output == "-":
            sys.stdout.buffer.write(output)
            sys.stdout.buffer.flush()
        else:
            output_path = Path(args.output).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(output)
        return 0
    except Exception as exc:
        print(f"urdf2mjcf: {exc}", file=sys.stderr)
        return 1
