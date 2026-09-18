"""Locate or fetch the optional Panda model without enlarging the distribution."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from urllib.request import urlopen


def panda_archive(assets_dir: Path) -> bytes:
    """Read a verified local archive, or download it once into the user cache.

    ``SERVO_PY_PANDA_ARCHIVE`` selects an existing ZIP for offline use. Source
    checkouts can use their original ZIP; wheels and sdists omit that file.
    Downloaded content is verified before an atomic cache update. Nothing is
    extracted or written to the installed package directory.
    """
    manifest = json.loads((assets_dir / "panda-source.json").read_text())
    digest = manifest["archive_sha256"]

    def verified(data: bytes, source: Path | str) -> bytes:
        if hashlib.sha256(data).hexdigest() != digest:
            raise ValueError(f"Panda archive SHA-256 mismatch: {source}")
        return data

    override = os.environ.get("SERVO_PY_PANDA_ARCHIVE")
    local = Path(override).expanduser() if override else assets_dir / "panda.zip"
    if override or local.is_file():
        # An explicit offline path must not silently fall back to the network.
        return verified(local.read_bytes(), local)

    cache_root = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    if not cache_root.is_absolute():
        cache_root = Path.home() / ".cache"
    cache = cache_root / "servo-py" / "panda" / digest / "panda.zip"
    if cache.is_file():
        data = cache.read_bytes()
        if hashlib.sha256(data).hexdigest() == digest:
            return data

    url = manifest["archive_url"]
    temporary = None
    try:
        cache.parent.mkdir(parents=True, exist_ok=True)
        print(f"Downloading Panda model ({manifest['archive_bytes'] / 1e6:.1f} MB) to {cache}", file=sys.stderr)
        with urlopen(url, timeout=30) as response:
            data = verified(response.read(manifest["archive_bytes"] + 1), url)
        with tempfile.NamedTemporaryFile(dir=cache.parent, suffix=".tmp", delete=False) as output:
            temporary = Path(output.name)
            output.write(data)
        temporary.replace(cache)
        return data
    except (OSError, ValueError) as exc:
        raise RuntimeError(
            f"Could not prepare the Panda model: {exc}. Download {url} on a connected "
            "machine and set SERVO_PY_PANDA_ARCHIVE=/path/to/panda.zip for offline use."
        ) from exc
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
