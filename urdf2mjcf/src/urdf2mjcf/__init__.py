"""URDF to MuJoCo MJCF conversion."""

__version__ = "1.0.0"
__all__ = ["__version__", "full_pipeline", "parse_element"]


def __getattr__(name):
    # Keep metadata and CLI help importable without loading optional/runtime deps.
    if name == "full_pipeline":
        from .app import full_pipeline

        return full_pipeline
    if name == "parse_element":
        from .core import parse_element

        return parse_element
    raise AttributeError(name)
