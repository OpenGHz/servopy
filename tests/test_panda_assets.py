"""Network-free checks for optional model downloads and offline cache reuse."""
import hashlib
from io import BytesIO
import json
from pathlib import Path
from urllib.error import URLError

import pytest

from servo_py.examples import _panda_assets as assets


@pytest.fixture
def archive(tmp_path, monkeypatch):
    data = b"a fixed Panda archive fixture"
    digest = hashlib.sha256(data).hexdigest()
    source = tmp_path / "assets"
    source.mkdir()
    (source / "panda-source.json").write_text(json.dumps({
        "archive_sha256": digest, "archive_bytes": len(data),
        "archive_url": "https://models.example.invalid/panda.zip",
    }))
    monkeypatch.delenv("SERVO_PY_PANDA_ARCHIVE", raising=False)
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))

    def no_network(*args, **kwargs):
        pytest.fail("Unexpected network request")

    monkeypatch.setattr(assets, "urlopen", no_network)
    cached = tmp_path / "cache/servo-py/panda" / digest / "panda.zip"
    return source, data, cached


def test_checkout_model_works_without_network_or_cache(archive):
    source, data, cached = archive
    (source / "panda.zip").write_bytes(data)
    assert assets.panda_archive(source) == data
    assert not cached.parent.exists()


def test_explicit_offline_archive_takes_priority(archive, tmp_path, monkeypatch):
    source, data, cached = archive
    (source / "panda.zip").write_bytes(b"outdated checkout archive")
    offline = tmp_path / "offline.zip"
    offline.write_bytes(data)
    monkeypatch.setenv("SERVO_PY_PANDA_ARCHIVE", str(offline))
    assert assets.panda_archive(source) == data
    assert not cached.parent.exists()


@pytest.mark.parametrize("present", [False, True])
def test_invalid_offline_archive_does_not_fall_back_to_network(archive, tmp_path, monkeypatch, present):
    source, _, _ = archive
    offline = tmp_path / "offline.zip"
    if present:
        offline.write_bytes(b"incorrect archive")
    monkeypatch.setenv("SERVO_PY_PANDA_ARCHIVE", str(offline))
    with pytest.raises(ValueError if present else FileNotFoundError):
        assets.panda_archive(source)


def test_download_is_verified_cached_and_reused_offline(archive, monkeypatch, capsys):
    source, data, cached = archive
    requests = []

    def download(url, *, timeout):
        requests.append(url)
        assert timeout > 0
        return BytesIO(data)

    monkeypatch.setattr(assets, "urlopen", download)
    assert assets.panda_archive(source) == data
    assert cached.read_bytes() == data
    assert list(cached.parent.iterdir()) == [cached]
    assert assets.panda_archive(source) == data
    assert len(requests) == 1
    output = capsys.readouterr()
    assert not output.out  # Preserve JSON on the demo's stdout.
    assert "Downloading Panda" in output.err


@pytest.mark.parametrize("payload", [b"incorrect archive", b"a fixed Panda archive fixture plus unwanted data"])
def test_unverified_download_is_never_cached(archive, monkeypatch, payload):
    source, _, cached = archive
    monkeypatch.setattr(assets, "urlopen", lambda *args, **kwargs: BytesIO(payload))
    with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
        assets.panda_archive(source)
    assert list(cached.parent.iterdir()) == []


def test_network_failure_explains_offline_option(archive, monkeypatch):
    source, _, cached = archive

    def disconnected(*args, **kwargs):
        raise URLError("offline")

    monkeypatch.setattr(assets, "urlopen", disconnected)
    with pytest.raises(RuntimeError, match="SERVO_PY_PANDA_ARCHIVE"):
        assets.panda_archive(source)
    assert list(cached.parent.iterdir()) == []


def test_corrupt_cache_is_repaired(archive, monkeypatch):
    source, data, cached = archive
    cached.parent.mkdir(parents=True)
    cached.write_bytes(b"incomplete old download")
    monkeypatch.setattr(assets, "urlopen", lambda *args, **kwargs: BytesIO(data))
    assert assets.panda_archive(source) == data
    assert cached.read_bytes() == data


def test_failed_cache_update_cleans_temporary_file(archive, monkeypatch):
    source, data, cached = archive
    monkeypatch.setattr(assets, "urlopen", lambda *args, **kwargs: BytesIO(data))

    def cannot_replace(*args, **kwargs):
        raise OSError("cache write failed")

    monkeypatch.setattr(Path, "replace", cannot_replace)
    with pytest.raises(RuntimeError, match="cache write failed"):
        assets.panda_archive(source)
    assert list(cached.parent.iterdir()) == []
