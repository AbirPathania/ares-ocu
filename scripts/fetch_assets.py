"""Fetch the CC0 models and textures the renderer uses from Poly Haven.

    .venv/bin/python scripts/fetch_assets.py

Everything lands under assets/polyhaven/, which is not committed: it is a
download, about 70 MB, and the renderer falls back to its procedural scenery
when it is absent. Each file is checked against the size and MD5 the Poly Haven
API reports, and anything that does not match is deleted. Poly Haven content is
CC0 (public domain); see https://polyhaven.com/license.

The API asks clients to identify themselves, which the User-Agent does.
"""

from __future__ import annotations

import hashlib
import json
import ssl
import sys
import urllib.request
from pathlib import Path

API = "https://api.polyhaven.com"
AGENT = "ares-ocu-demonstrator (github.com/AbirPathania/ares-ocu)"
ROOT = Path(__file__).resolve().parent.parent / "assets" / "polyhaven"

MODELS = [
    "namaqualand_boulder_02",
    "namaqualand_boulder_03",
    "namaqualand_boulder_04",
    "namaqualand_boulder_05",
    "namaqualand_boulder_06",
    "namaqualand_rocks_01",
    "namaqualand_stones_01",
    "othonna_cerarioides",
    "grass_medium_01",
    "grass_medium_02",
    "grass_bermuda_01",
]
TEXTURES = ["dry_ground_01", "gravelly_sand", "aerial_grass_rock", "brown_mud_dry", "rocky_terrain_02"]
TEXTURE_MAPS = ("Diffuse", "nor_gl", "Rough")
RESOLUTION = "1k"


def _context() -> ssl.SSLContext:
    """Verified TLS. The python.org builds on macOS ship without a CA bundle, so
    use certifi's; certificate checking is never turned off."""
    try:
        import certifi

        return ssl.create_default_context(cafile=certifi.where())
    except ImportError:
        return ssl.create_default_context()


CONTEXT = _context()


def fetch_json(path: str) -> dict:
    request = urllib.request.Request(f"{API}{path}", headers={"User-Agent": AGENT})
    with urllib.request.urlopen(request, timeout=60, context=CONTEXT) as response:
        return json.load(response)


def download(url: str, target: Path, size: int | None, md5: str | None) -> None:
    if target.exists() and (size is None or target.stat().st_size == size):
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(url, headers={"User-Agent": AGENT})
    digest = hashlib.md5()
    with urllib.request.urlopen(request, timeout=120, context=CONTEXT) as response, open(target, "wb") as out:
        while chunk := response.read(1 << 16):
            digest.update(chunk)
            out.write(chunk)
    if (size is not None and target.stat().st_size != size) or (md5 and digest.hexdigest() != md5):
        target.unlink()
        raise RuntimeError(f"{target.name}: size or checksum does not match what Poly Haven reports")


def fetch_model(asset: str) -> None:
    files = fetch_json(f"/files/{asset}")["gltf"][RESOLUTION]["gltf"]
    base = ROOT / "models" / asset
    download(files["url"], base / files["url"].rsplit("/", 1)[-1], files.get("size"), files.get("md5"))
    for relative, entry in files.get("include", {}).items():
        download(entry["url"], base / relative, entry.get("size"), entry.get("md5"))
    print("model  ", asset)


def fetch_texture(asset: str) -> None:
    files = fetch_json(f"/files/{asset}")
    base = ROOT / "textures" / asset
    for name in TEXTURE_MAPS:
        entry = files.get(name, {}).get(RESOLUTION, {})
        chosen = entry.get("jpg") or entry.get("png")
        if chosen is None:
            print("  (no", name, "for", asset, ")")
            continue
        download(chosen["url"], base / f"{name}.{chosen['url'].rsplit('.', 1)[-1]}", chosen.get("size"), chosen.get("md5"))
    print("texture", asset)


def main() -> int:
    failures = 0
    for asset in MODELS:
        try:
            fetch_model(asset)
        except Exception as error:  # keep going: one missing asset must not stop the rest
            failures += 1
            print("FAILED", asset, error, file=sys.stderr)
    for asset in TEXTURES:
        try:
            fetch_texture(asset)
        except Exception as error:
            failures += 1
            print("FAILED", asset, error, file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
