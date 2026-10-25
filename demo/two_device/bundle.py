"""Build the self-contained replica bundle for the second device.

    python demo/two_device/bundle.py [-o OUT.pyz]

The output is a zip that Python can run directly:

    python two_device_replica.pyz --id aircraft --connect MAC_IP:9000

It contains the real ocu/core/mesh_state.py and ocu/core/constants.py, copied
byte for byte from this repository, empty __init__.py stubs for the two
packages, replica.py, dashboard.py, and a MANIFEST.txt of SHA-256 hashes.
Standard library only; the bundle never imports PySide6.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

MAIN = "import sys\n\nimport replica\n\nsys.exit(replica.main())\n"

# (path inside the bundle, path in the repository or None for generated text)
FILES = [
    ("replica.py", os.path.join(HERE, "replica.py")),
    ("dashboard.py", os.path.join(HERE, "dashboard.py")),
    ("ocu/core/mesh_state.py", os.path.join(REPO, "ocu", "core", "mesh_state.py")),
    ("ocu/core/constants.py", os.path.join(REPO, "ocu", "core", "constants.py")),
]

DEFAULT_OUT = os.path.join(HERE, "dist", "two_device_replica.pyz")
_FIXED_TIME = (2026, 1, 1, 0, 0, 0)


def _add(zf, name, data):
    info = zipfile.ZipInfo(name, _FIXED_TIME)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    zf.writestr(info, data)


def build(out=DEFAULT_OUT):
    """Write the bundle to out and return its path. Deterministic for the same inputs."""
    entries = [("__main__.py", MAIN.encode("ascii")), ("ocu/__init__.py", b""), ("ocu/core/__init__.py", b"")]
    for name, src in FILES:
        with open(src, "rb") as fh:
            entries.append((name, fh.read()))
    manifest = "".join(
        "%s  %s\n" % (hashlib.sha256(data).hexdigest(), name) for name, data in sorted(entries)
    )
    entries.append(("MANIFEST.txt", manifest.encode("ascii")))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with zipfile.ZipFile(out, "w") as zf:
        for name, data in entries:
            _add(zf, name, data)
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description="Build the two-device replica bundle.")
    p.add_argument("-o", "--out", default=DEFAULT_OUT)
    args = p.parse_args(sys.argv[1:] if argv is None else argv)
    out = build(args.out)
    with open(out, "rb") as fh:
        data = fh.read()
    print("wrote %s (%d bytes, sha256 %s)" % (out, len(data), hashlib.sha256(data).hexdigest()[:16]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
