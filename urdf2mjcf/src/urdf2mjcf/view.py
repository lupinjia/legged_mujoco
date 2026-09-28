"""Launch the native MuJoCo viewer for an MJCF model."""

import argparse
from pathlib import Path
from typing import Optional, Sequence


def _existing_model_path(value: str) -> Path:
    path = Path(value).expanduser().resolve()
    if not path.exists():
        raise argparse.ArgumentTypeError(f"model file does not exist: {path}")
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"model path is not a file: {path}")
    return path


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="urdf2mjcf-view",
        description="Open an MJCF model in the native MuJoCo viewer.",
    )
    parser.add_argument("model", metavar="MODEL.xml", type=_existing_model_path)
    args = parser.parse_args(argv)

    try:
        from mujoco.viewer import launch_from_path
    except (ImportError, RuntimeError) as exc:
        parser.error(
            "MuJoCo's viewer is unavailable. Install MuJoCo with viewer support "
            f"and use a machine with a working graphics display ({exc})."
        )

    try:
        launch_from_path(str(args.model))
    except Exception as exc:
        parser.error(
            f"could not launch the MuJoCo viewer for {args.model}: {exc}. "
            "Check that the model and its assets are valid and that a usable "
            "desktop graphics display is available."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
