"""Small utility functions used by the package."""

from typing import Dict, Optional, TypeVar

K = TypeVar("K")
V = TypeVar("V")


def _update_with_defaults(
    dictionary: Optional[Dict[K, V]] = None, defaults: Optional[Dict[K, V]] = None
) -> Dict[K, V]:
    updated = {} if defaults is None else dict(defaults)
    updated.update(dictionary if dictionary is not None else {})
    return updated
