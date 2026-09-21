#!/usr/bin/python3
"""Run onshape-to-robot with Basic auth instead of the retired HMAC scheme.

Onshape rejects the vendored client's 'On <key>:HmacSHA256:<sig>' Authorization
header for newly-issued API keys (401 on every endpoint; a bare 204 on
/api/users/sessioninfo, which reads as success but is really "no session").
Basic auth with the same key pair returns 200 on all endpoints, so we patch the
header builder at runtime rather than editing the installed package.

Usage:  source ./onshape.sh && /usr/bin/python3 export_onshape.py [robot_dir]
"""
import base64
import datetime
import random
import string
import sys

from onshape_to_robot.onshape_api.onshape import Onshape


def _make_headers(self, method, path, query={}, headers={}):
    date = datetime.datetime.now(datetime.UTC).strftime("%a, %d %b %Y %H:%M:%S GMT")
    nonce = "".join(random.choice(string.digits + string.ascii_letters) for _ in range(25))
    ctype = headers.get("Content-Type") or "application/json"
    req_headers = {
        "Content-Type": ctype,
        "Date": date,
        "On-Nonce": nonce,
        "Authorization": "Basic "
        + base64.b64encode(self._access_key + b":" + self._secret_key).decode("utf-8"),
        "User-Agent": "onshape-to-robot (basic-auth patch)",
        "Accept": "application/json",
    }
    for h in headers:
        req_headers[h] = headers[h]
    return req_headers


Onshape._make_headers = _make_headers

EXPECTED_BASE_MASS_KG = 0.203401  # whole drone, weighed on a scale
TOLERANCE_KG = 0.0005


def check_mass(urdf_path="model.urdf"):
    """Fail loudly if the export lost mass (e.g. a part lost its material).

    Onshape's assembly-level mass override is NOT read by onshape-to-robot --
    it builds link inertials by merging per-part mass properties -- so a
    material-less part silently produces a too-light model.
    """
    import xml.etree.ElementTree as ET

    root = ET.parse(urdf_path).getroot()
    base = [l for l in root.iter("link") if l.get("name") == "base_link"][0]
    mass = float(base.find("inertial/mass").get("value"))
    delta = abs(mass - EXPECTED_BASE_MASS_KG)
    if delta > TOLERANCE_KG:
        print(
            f"\n!! MASS CHECK FAILED: base_link = {mass:.6f} kg, "
            f"expected {EXPECTED_BASE_MASS_KG:.6f} kg (off by {delta*1000:.1f} g).\n"
            "!! A part has probably lost its material assignment in Onshape.\n"
            "!! Do NOT propagate this to model.sdf.",
            file=sys.stderr,
        )
        return 1
    print(f"* Mass check OK: base_link = {mass:.6f} kg")
    return 0


if __name__ == "__main__":
    from onshape_to_robot.export import main

    if len(sys.argv) < 2:
        sys.argv.append(".")
    main()
    sys.exit(check_mass())
