"""The two-device demo: replicas, the bundle, and the guarantees the video relies on.

Everything here runs on loopback. The link cuts are real socket events (closed
sockets, or a proxy that swallows traffic so the heartbeat timeout has to fire),
not a software flag.
"""

import hashlib
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
import zipfile

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO = os.path.join(REPO, "demo", "two_device")
sys.path.insert(0, DEMO)

import bundle  # noqa: E402
import replica  # noqa: E402
from ocu.core.mesh_state import MeshState, VoxelObservation, merge, reconcile, voxel_key  # noqa: E402


# --- helpers ------------------------------------------------------------------


def wait_for(cond, timeout=8.0, what="condition"):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return
        time.sleep(0.02)
    raise AssertionError("timed out waiting for " + what)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class CutProxy:
    """A TCP forwarder between two replicas that can be cut like a dead network.

    mode "forward": traffic passes.
    mode "closed":  nothing listens and every open pipe is closed (a hard drop).
    mode "blackhole": connections are accepted and open pipes stay open, but no
        byte is delivered either way (a Wi-Fi cut: silence, no FIN).
    """

    def __init__(self, target_port):
        self.target_port = target_port
        self.port = free_port()
        self.mode = "forward"
        self.pipes = []
        self.lock = threading.Lock()
        self.stop = False
        self.listener = None
        self._listen()

    def _listen(self):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", self.port))
        s.listen(8)
        s.settimeout(0.2)
        self.listener = s
        threading.Thread(target=self._accept, args=(s,), daemon=True).start()

    def _accept(self, s):
        while not self.stop:
            try:
                c, _ = s.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            if self.mode == "blackhole":
                with self.lock:
                    self.pipes.append(c)
                threading.Thread(target=self._drain, args=(c,), daemon=True).start()
                continue
            try:
                u = socket.create_connection(("127.0.0.1", self.target_port), timeout=1.0)
            except OSError:
                c.close()
                continue
            with self.lock:
                self.pipes += [c, u]
            threading.Thread(target=self._pump, args=(c, u), daemon=True).start()
            threading.Thread(target=self._pump, args=(u, c), daemon=True).start()

    def _drain(self, c):
        try:
            while c.recv(4096):
                pass
        except OSError:
            pass

    def _pump(self, src, dst):
        try:
            while True:
                data = src.recv(65536)
                if not data:
                    break
                if self.mode != "forward":
                    continue  # blackhole: swallow
                dst.sendall(data)
        except OSError:
            pass
        finally:
            if self.mode == "forward":
                for x in (src, dst):
                    try:
                        x.close()
                    except OSError:
                        pass

    def blackhole(self):
        self.mode = "blackhole"

    def cut_closed(self):
        self.mode = "closed"
        try:
            self.listener.close()
        except OSError:
            pass
        with self.lock:
            pipes, self.pipes = self.pipes, []
        for p in pipes:
            try:
                p.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                p.close()
            except OSError:
                pass

    def restore(self):
        if self.mode == "closed":
            self._listen()
        self.mode = "forward"

    def close(self):
        self.stop = True
        self.cut_closed()


@pytest.fixture
def pair():
    """rover listens; aircraft reaches it through a CutProxy."""
    made = []

    def make(rover_offset=0.0, aircraft_offset=0.0):
        rover = replica.Replica("rover", listen_port=0, dash_port=None, clock_offset_s=rover_offset, echo=False).start()
        proxy = CutProxy(rover.bound_port)
        aircraft = replica.Replica(
            "aircraft", connect=("127.0.0.1", proxy.port), dash_port=None, clock_offset_s=aircraft_offset, echo=False
        ).start()
        made.append((rover, aircraft, proxy))
        wait_for(lambda: rover.state()["link"] == "LINKED" and aircraft.state()["link"] == "LINKED", what="link up")
        return rover, aircraft, proxy

    yield make
    for rover, aircraft, proxy in made:
        aircraft.stop()
        rover.stop()
        proxy.close()


def both_down(a, b):
    return a.state()["link"] == "DOWN" and b.state()["link"] == "DOWN"


def synced(a, b):
    return (
        a.state()["link"] == "LINKED"
        and b.state()["link"] == "LINKED"
        and a.state()["last_delta"] is not None
        and b.state()["last_delta"] is not None
    )


def sync_count(r):
    return sum(1 for e in r.events if "sync done" in e)


def oracle_delta(a, b):
    """The delta the pure-Python reconcile() reports for these two maps."""
    ma, mb = MeshState("a"), MeshState("b")
    for o in a.mesh.snapshot().values():
        ma.apply(o)
    for o in b.mesh.snapshot().values():
        mb.apply(o)
    delta = reconcile(ma, mb)
    return delta, ma.snapshot()


# --- convergence over a real link cut ------------------------------------------


def run_cut_scenario(pair, cut, restore, aircraft_offset=0.0, rover_offset=0.0):
    rover, aircraft, proxy = pair(rover_offset=rover_offset, aircraft_offset=aircraft_offset)
    for i in range(3):
        rover.run_command("obs %d 1 0.9 mine" % i)
    wait_for(lambda: len(aircraft.mesh) == 3, what="live path")
    wait_for(lambda: rover.state()["hash"] == aircraft.state()["hash"], what="equal hashes before cut")
    before_syncs = (sync_count(rover), sync_count(aircraft))

    cut(proxy)
    wait_for(lambda: both_down(rover, aircraft), what="both sides detect the cut")

    rover.run_command("obs 10 10 0.8 ied")
    rover.run_command("obs 11 10 0.7 person")
    rover.run_command("obs 20 20 0.80 mine")  # conflict voxel, rover's version
    aircraft.run_command("obs 30 30 0.9 vehicle")
    aircraft.run_command("obs 31 30 0.6 unknown")
    aircraft.run_command("obs 32 30 0.5 person")
    aircraft.run_command("obs 20 20 0.80 ied")  # conflict voxel, aircraft's version
    assert rover.state()["divergence"] == 3
    assert aircraft.state()["divergence"] == 4
    assert rover.state()["hash"] != aircraft.state()["hash"]
    assert rover.state()["hash_state"] == "DIFFER"

    expected_delta, expected_map = oracle_delta(rover, aircraft)
    restore(proxy)
    wait_for(
        lambda: sync_count(rover) > before_syncs[0] and sync_count(aircraft) > before_syncs[1],
        timeout=12.0,
        what="reconnect and sync",
    )
    wait_for(lambda: rover.state()["hash_state"] == "MATCH" and aircraft.state()["hash_state"] == "MATCH", what="MATCH")
    return rover, aircraft, expected_delta, expected_map


def test_hard_cut_by_closing_the_socket_then_restore_converges_with_exact_delta(pair):
    rover, aircraft, expected_delta, expected_map = run_cut_scenario(
        pair, lambda p: p.cut_closed(), lambda p: p.restore()
    )
    assert rover.state()["hash"] == aircraft.state()["hash"]
    assert rover.mesh.snapshot() == aircraft.mesh.snapshot() == expected_map
    assert replica.canonical_bytes(rover.mesh.snapshot()) == replica.canonical_bytes(aircraft.mesh.snapshot())
    # 2 voxels only rover has, 3 only aircraft has, 1 held differently by both.
    assert expected_delta == 6
    for r in (rover, aircraft):
        d = r.state()["last_delta"]
        assert d["voxels"] == 6
        assert d["bytes_tx"] > 0 and d["bytes_rx"] > 0
    assert rover.state()["last_delta"]["sent"] == 3  # its 2 new voxels and the conflict
    assert rover.state()["last_delta"]["received"] == 4  # their 3 new voxels and the conflict
    assert aircraft.state()["last_delta"]["sent"] == 4
    assert rover.state()["divergence"] == 0 and aircraft.state()["divergence"] == 0
    # The 3 voxels shared before the cut were not resent, and the whole map was not resent.
    assert len(rover.mesh) == 9
    assert rover.state()["last_delta"]["sent"] + rover.state()["last_delta"]["received"] < 2 * len(rover.mesh)


def test_silent_cut_is_detected_by_missed_heartbeats_then_converges(pair):
    cut_at = {}
    rover, aircraft, proxy = pair()
    rover.run_command("obs 1 1 0.9 mine")
    wait_for(lambda: len(aircraft.mesh) == 1)
    cut_at["t"] = time.time()
    proxy.blackhole()
    wait_for(lambda: both_down(rover, aircraft), what="heartbeat timeout")
    elapsed = time.time() - cut_at["t"]
    # 3 missed heartbeats at 0.5 s is 1.5 s of silence; allow for the last heartbeat
    # having arrived just before the cut and for the 0.25 s polling step.
    assert 0.9 <= elapsed <= 3.0, elapsed
    assert "no heartbeat" in rover.state()["cut_reason"]
    assert "no heartbeat" in aircraft.state()["cut_reason"]
    assert rover.state()["sim_banner"] is False  # a real cut never shows the rehearsal banner

    rover.run_command("obs 5 5 0.7 ied")
    aircraft.run_command("obs 6 6 0.6 person")
    syncs = (sync_count(rover), sync_count(aircraft))
    proxy.restore()
    wait_for(lambda: sync_count(rover) > syncs[0] and sync_count(aircraft) > syncs[1], timeout=12.0, what="resync")
    wait_for(lambda: rover.state()["hash_state"] == "MATCH" and aircraft.state()["hash_state"] == "MATCH")
    assert rover.mesh.snapshot() == aircraft.mesh.snapshot()
    assert rover.state()["last_delta"]["voxels"] == 2


@pytest.mark.parametrize("skew_s", [600.0, -600.0])
def test_clock_skew_of_ten_minutes_still_converges_to_the_same_map(pair, skew_s):
    rover, aircraft, expected_delta, expected_map = run_cut_scenario(
        pair, lambda p: p.cut_closed(), lambda p: p.restore(), aircraft_offset=skew_s
    )
    assert rover.state()["hash"] == aircraft.state()["hash"]
    assert rover.mesh.snapshot() == aircraft.mesh.snapshot() == expected_map
    # Equal confidence, so the timestamp decides. The skewed clock is 600 s ahead or behind.
    winner = rover.mesh.get(voxel_key(20, 20, 0))
    assert winner.source_id == ("aircraft" if skew_s > 0 else "rover")
    # Skew only matters on a tie: a higher confidence wins whatever the clocks say.
    assert rover.mesh.get(voxel_key(30, 30, 0)).confidence == 0.9


def test_conflict_command_resolves_identically_on_both_sides(pair):
    rover, aircraft, proxy = pair()
    proxy.cut_closed()
    wait_for(lambda: both_down(rover, aircraft))
    rover.run_command("conflict 4 4")
    aircraft.run_command("conflict 4 4")
    a = rover.mesh.get(voxel_key(4, 4, 0))
    b = aircraft.mesh.get(voxel_key(4, 4, 0))
    assert a != b and a.threat_type != b.threat_type  # a deliberate disagreement
    syncs = (sync_count(rover), sync_count(aircraft))
    proxy.restore()
    wait_for(lambda: sync_count(rover) > syncs[0] and sync_count(aircraft) > syncs[1], timeout=12.0)
    expected = merge(a, b)
    assert rover.mesh.get(voxel_key(4, 4, 0)) == aircraft.mesh.get(voxel_key(4, 4, 0)) == expected
    assert merge(b, a) == expected
    assert rover.state()["last_delta"]["voxels"] == 1


def test_live_path_sends_back_a_better_observation_so_the_sender_converges():
    r = replica.Replica("rover", listen_port=0, dash_port=None, echo=False).start()
    try:
        r.run_command("obs 7 7 0.95 ied")
        c = socket.create_connection(("127.0.0.1", r.bound_port), timeout=2)
        c.settimeout(3)
        c.sendall(b'{"t":"hello","id":"peer","v":1}\n')
        wait_for(lambda: r.state()["link"] == "LINKED")
        weaker = [140, 140, 0, 5, 0.4, "mine", "peer"]
        c.sendall((json.dumps({"t": "obs", "o": weaker}) + "\n").encode())
        seen = b""
        end = time.time() + 3
        fix = None
        while time.time() < end and fix is None:
            seen += c.recv(65536)
            for line in seen.split(b"\n"):
                if line.startswith(b'{"t":"obs"'):
                    fix = json.loads(line)
        assert fix is not None and fix.get("fix") is True
        assert fix["o"][4] == 0.95 and fix["o"][5] == "ied"
        c.close()
    finally:
        r.stop()


def test_byte_counters_are_taken_at_the_socket(pair):
    rover, aircraft, proxy = pair()
    time.sleep(1.2)
    # What one side sent, the other received, once the pipes are quiet. Heartbeats keep
    # flowing, so compare with a tolerance of a couple of in-flight heartbeats.
    s, r = rover.state(), aircraft.state()
    assert s["tx_bytes"] > 0 and r["rx_bytes"] > 0
    assert abs(s["tx_bytes"] - r["rx_bytes"]) < 400
    assert abs(r["tx_bytes"] - s["rx_bytes"]) < 400


# --- rehearsal cut ----------------------------------------------------------------


def test_simulated_cut_shows_the_banner_and_a_real_cut_never_does():
    rover = replica.Replica("rover", listen_port=0, dash_port=None, allow_simulate=True, echo=False).start()
    aircraft = replica.Replica("aircraft", connect=("127.0.0.1", rover.bound_port), dash_port=None, echo=False).start()
    try:
        wait_for(lambda: rover.state()["link"] == "LINKED" and aircraft.state()["link"] == "LINKED")
        assert rover.state()["sim_banner"] is False
        assert aircraft.run_command("simulate-cut").startswith("refused")  # not enabled there
        rover.run_command("simulate-cut")
        wait_for(lambda: both_down(rover, aircraft))
        assert rover.state()["sim_banner"] is True
        assert aircraft.state()["sim_banner"] is True  # the peer is told, so it is labeled too
        time.sleep(0.8)
        assert rover.state()["link"] == "DOWN"  # no reconnect while the simulated cut is held
        rover.run_command("obs 1 1 0.9 mine")
        rover.run_command("simulate-restore")
        wait_for(lambda: rover.state()["link"] == "LINKED" and aircraft.state()["link"] == "LINKED", timeout=12.0)
        wait_for(lambda: len(aircraft.mesh) == 1)
        assert rover.state()["sim_banner"] is True  # stays labeled until a real cut replaces it
    finally:
        aircraft.stop()
        rover.stop()


# --- address policy and message validation ---------------------------------------


@pytest.mark.parametrize(
    "addr,ok",
    [
        ("127.0.0.1", True), ("::1", True), ("10.1.2.3", True), ("172.16.0.1", True), ("172.31.255.255", True),
        ("192.168.1.20", True), ("169.254.10.10", True), ("fe80::1", True), ("fd12:3456::1", True),
        ("::ffff:192.168.1.5", True),
        ("8.8.8.8", False), ("1.1.1.1", False), ("172.32.0.1", False), ("192.169.0.1", False),
        ("100.64.0.1", False), ("0.0.0.0", False), ("2001:4860:4860::8888", False), ("not-an-ip", False),
        ("::ffff:8.8.8.8", False),
    ],
)
def test_private_address_policy(addr, ok):
    assert replica.is_private_address(addr) is ok


def test_connecting_to_a_public_address_is_refused_before_any_socket_is_opened(monkeypatch):
    opened = []
    real = socket.create_connection

    def spy(address, *a, **k):
        opened.append(address)
        return real(address, *a, **k)

    monkeypatch.setattr(socket, "create_connection", spy)
    with pytest.raises(ValueError):
        replica.resolve_private("8.8.8.8", 9000)
    r = replica.Replica("rover", connect=("8.8.8.8", 9000), dash_port=None, echo=False).start()
    try:
        wait_for(lambda: any("non-private" in e for e in r.events), what="refusal logged")
        time.sleep(0.3)
        assert opened == []
        assert r.state()["link"] == "DOWN"
    finally:
        r.stop()


def test_accepting_from_a_public_address_is_refused(monkeypatch):
    real = replica.is_private_address
    monkeypatch.setattr(replica, "is_private_address", lambda a: False if a == "127.0.0.1" else real(a))
    r = replica.Replica("rover", listen_port=0, dash_port=None, echo=False).start()
    try:
        c = socket.create_connection(("127.0.0.1", r.bound_port), timeout=2)
        wait_for(lambda: any("REFUSED" in e for e in r.events), what="refusal logged")
        c.settimeout(2)
        assert c.recv(100) == b""  # closed without a hello
        assert r.state()["link"] == "DOWN"
        c.close()
    finally:
        r.stop()


def test_bad_messages_from_a_peer_are_dropped_and_do_not_corrupt_the_map():
    r = replica.Replica("rover", listen_port=0, dash_port=None, echo=False).start()
    try:
        c = socket.create_connection(("127.0.0.1", r.bound_port), timeout=2)
        c.sendall(b'{"t":"hello","id":"evil","v":1}\n')
        wait_for(lambda: r.state()["link"] == "LINKED")
        bad = [
            b"this is not json\n",
            b'{"t":"obs","o":[1,2,0,5,5.0,"mine","x"]}\n',  # confidence above 1
            b'{"t":"obs","o":[1,2,0,5,NaN,"mine","x"]}\n',  # NaN
            b'{"t":"obs","o":[1,2,"0",5,0.5,"mine","x"]}\n',  # wrong type
            b'{"t":"obs","o":[1,2,0]}\n',  # wrong shape
            b'{"t":"nope"}\n',
            b'{"t":"digest","d":{"1,2":"abc"}}\n',  # bad key
        ]
        for line in bad:
            c.sendall(line)
        c.sendall(b'{"t":"obs","o":[1,2,0,5,0.5,"mine","x"]}\n')  # a valid one after the junk
        wait_for(lambda: len(r.mesh) == 1, what="valid observation applied after junk")
        assert r.state()["link"] == "LINKED"
        assert r.mesh.get((1, 2, 0)).confidence == 0.5
        c.close()
    finally:
        r.stop()


def test_canonical_hash_is_independent_of_insertion_order():
    obs = [
        VoxelObservation((i, 0, 0), 100 + i, 0.5, "mine", "rover") for i in range(20)
    ]
    a, b = MeshState("a"), MeshState("b")
    a.apply_all(obs)
    b.apply_all(list(reversed(obs)))
    assert replica.map_hash(a.snapshot()) == replica.map_hash(b.snapshot())
    assert len(replica.map_hash(a.snapshot())) == 12


# --- dashboard ------------------------------------------------------------------


def test_dashboard_serves_the_footer_and_state_and_accepts_commands():
    r = replica.Replica("rover", listen_port=0, dash_port=0, echo=False).start()
    try:
        base = "http://127.0.0.1:%d" % r.dash_port
        page = urllib.request.urlopen(base + "/").read().decode("utf-8")
        assert (
            "Real devices, real network, real link cut. "
            "Observations are injected by the operator, not from sensors."
        ) in page
        assert "http://" not in page and "https://" not in page  # no CDN, no external fonts
        req = urllib.request.Request(base + "/cmd", data=json.dumps({"cmd": "obs 1 2 0.8 mine"}).encode(), method="POST")
        assert "recorded" in json.loads(urllib.request.urlopen(req).read())["msg"]
        st = json.loads(urllib.request.urlopen(base + "/state").read())
        assert st["voxels"] == 1 and st["link"] == "DOWN" and st["grid"][0][2] == "rover"
        assert not any(ord(ch) in (0x2013, 0x2014) for ch in page)
    finally:
        r.stop()


# --- the bundle ----------------------------------------------------------------


@pytest.fixture(scope="module")
def built_bundle(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("bundle") / "two_device_replica.pyz")
    bundle.build(out)
    return out


BLOCKER = (
    "import sys\n"
    "class _Block:\n"
    "    def find_spec(self, name, path=None, target=None):\n"
    "        if name.split('.')[0] in ('PySide6', 'shiboken6', 'numpy', 'moderngl', 'opensimplex'):\n"
    "            raise ImportError('blocked: ' + name)\n"
    "sys.meta_path.insert(0, _Block())\n"
)


def test_bundled_files_are_byte_identical_to_the_repo(built_bundle):
    with zipfile.ZipFile(built_bundle) as zf:
        for inside, src in [
            ("ocu/core/mesh_state.py", os.path.join(REPO, "ocu", "core", "mesh_state.py")),
            ("ocu/core/constants.py", os.path.join(REPO, "ocu", "core", "constants.py")),
            ("replica.py", os.path.join(DEMO, "replica.py")),
            ("dashboard.py", os.path.join(DEMO, "dashboard.py")),
        ]:
            with open(src, "rb") as fh:
                assert zf.read(inside) == fh.read(), inside
        assert zf.read("ocu/__init__.py") == b"" and zf.read("ocu/core/__init__.py") == b""
        names = set(zf.namelist())
        assert names == {
            "__main__.py", "MANIFEST.txt", "replica.py", "dashboard.py",
            "ocu/__init__.py", "ocu/core/__init__.py", "ocu/core/mesh_state.py", "ocu/core/constants.py",
        }
        manifest = zf.read("MANIFEST.txt").decode()
        for line in manifest.splitlines():
            digest, name = line.split("  ")
            assert hashlib.sha256(zf.read(name)).hexdigest() == digest


def test_bundle_is_deterministic(tmp_path):
    a, b = str(tmp_path / "a.pyz"), str(tmp_path / "b.pyz")
    bundle.build(a)
    bundle.build(b)
    with open(a, "rb") as fa, open(b, "rb") as fb:
        assert fa.read() == fb.read()


def test_unpacked_bundle_imports_and_runs_under_isolated_python_without_pyside(built_bundle, tmp_path):
    clean = str(tmp_path / "unpacked")
    with zipfile.ZipFile(built_bundle) as zf:
        zf.extractall(clean)
    code = (
        BLOCKER
        + "import sys\nsys.path.insert(0, %r)\n" % clean
        + "import replica\n"
        + "assert replica.selftest() == 0\n"
        + "assert 'PySide6' not in sys.modules\n"
        + "import ocu.core.mesh_state as m\n"
        + "assert m.__file__.startswith(%r), m.__file__\n" % clean
    )
    out = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True, cwd=str(tmp_path), timeout=60)
    assert out.returncode == 0, out.stdout + out.stderr
    assert "selftest OK" in out.stdout
    assert "PySide6 imported: False" in out.stdout


def test_zipped_bundle_selftest_runs_directly_under_isolated_python(built_bundle, tmp_path):
    out = subprocess.run([sys.executable, "-I", built_bundle, "--selftest"], capture_output=True, text=True,
                         cwd=str(tmp_path), timeout=60)
    assert out.returncode == 0, out.stdout + out.stderr
    assert "selftest OK" in out.stdout


@pytest.mark.skipif(not os.path.exists("/usr/bin/python3"), reason="no system python to stand in for the tablet")
def test_bundle_runs_on_the_system_python_which_has_no_pyside(built_bundle, tmp_path):
    exe = "/usr/bin/python3"
    ver = subprocess.run([exe, "-c", "import sys;print(sys.version_info[:2] >= (3, 9))"], capture_output=True, text=True)
    if ver.stdout.strip() != "True":
        pytest.skip("system python is older than 3.9")
    has_pyside = subprocess.run([exe, "-I", "-c", "import PySide6"], capture_output=True).returncode == 0
    assert not has_pyside
    out = subprocess.run([exe, "-I", built_bundle, "--selftest"], capture_output=True, text=True, cwd=str(tmp_path), timeout=60)
    assert out.returncode == 0, out.stdout + out.stderr
    assert "selftest OK" in out.stdout


def test_two_bundled_processes_cut_diverge_and_reconverge_through_the_command_line(built_bundle, tmp_path):
    """The path the demo actually uses: two processes, stdin commands, the HTTP state."""
    lp, rp, d1, d2 = free_port(), free_port(), free_port(), free_port()
    cmd = [sys.executable, "-I", built_bundle]
    log1, log2 = open(str(tmp_path / "rover.log"), "w"), open(str(tmp_path / "aircraft.log"), "w")
    rover = subprocess.Popen(cmd + ["--id", "rover", "--listen", str(lp), "--dash-port", str(d1), "--simulate-cut"],
                             stdin=subprocess.PIPE, stdout=log1, stderr=subprocess.STDOUT, text=True, cwd=str(tmp_path))
    aircraft = subprocess.Popen(cmd + ["--id", "aircraft", "--connect", "127.0.0.1:%d" % lp, "--dash-port", str(d2),
                                       "--clock-offset-s", "600"],
                                stdin=subprocess.PIPE, stdout=log2, stderr=subprocess.STDOUT, text=True, cwd=str(tmp_path))

    def st(port):
        try:
            return json.loads(urllib.request.urlopen("http://127.0.0.1:%d/state" % port, timeout=2).read())
        except OSError:  # not listening yet
            return {"link": "DOWN", "voxels": -1, "hash": None, "hash_state": None, "last_delta": None, "sim_banner": False}

    def say(proc, line):
        proc.stdin.write(line + "\n")
        proc.stdin.flush()

    try:
        wait_for(lambda: st(d1)["link"] == "LINKED" and st(d2)["link"] == "LINKED", timeout=15, what="link up")
        say(rover, "obs 1 1 0.9 mine")
        wait_for(lambda: st(d2)["voxels"] == 1)
        say(rover, "simulate-cut")
        wait_for(lambda: st(d1)["link"] == "DOWN" and st(d2)["link"] == "DOWN")
        assert st(d1)["sim_banner"] is True
        say(rover, "obs 2 2 0.8 ied")
        say(rover, "conflict 5 5")
        say(aircraft, "obs 9 9 0.7 person")
        say(aircraft, "conflict 5 5")
        wait_for(lambda: st(d1)["voxels"] == 3 and st(d2)["voxels"] == 3)
        assert st(d1)["hash"] != st(d2)["hash"]
        say(rover, "simulate-restore")
        wait_for(lambda: st(d1)["hash_state"] == "MATCH" and st(d1)["last_delta"] and st(d1)["last_delta"]["voxels"] == 3,
                 timeout=15, what="MATCH after restore")
        a, b = st(d1), st(d2)
        assert a["hash"] == b["hash"] and a["voxels"] == b["voxels"] == 4
        assert a["last_delta"]["voxels"] == 3 and b["last_delta"]["voxels"] == 3
    finally:
        for p in (rover, aircraft):
            try:
                p.stdin.write("quit\n")
                p.stdin.flush()
            except OSError:
                pass
        for p in (rover, aircraft):
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()


def test_dump_writes_the_canonical_map_that_the_hash_is_taken_over(tmp_path):
    r = replica.Replica("rover", listen_port=0, dash_port=None, echo=False)
    r.run_command("obs 1 1 0.9 mine")
    r.run_command("obs 2 2 0.5 person")
    path = str(tmp_path / "map.json")
    msg = r.run_command("dump " + path)
    with open(path, "rb") as fh:
        data = fh.read()
    assert data.rstrip(b"\n") == replica.canonical_bytes(r.mesh.snapshot())
    assert hashlib.sha256(data.rstrip(b"\n")).hexdigest()[:12] in msg
