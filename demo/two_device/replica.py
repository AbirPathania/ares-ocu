"""One replica of the shared map, for the two-device link-cut demonstration.

Run one of these on each device. They hold the same CRDT map (the real
ocu.core.mesh_state code), talk over TCP, and converge after a link cut.

Standard library only, Python 3.9 or newer. No PySide6, no third-party
packages, so it runs on a tablet.

WARNING: the peer protocol is unauthenticated and unencrypted. It is a demo
for a trusted local network. It refuses any peer address that is not
loopback, link-local or RFC1918 private (and IPv6 unique-local).

What this is not: the observations are typed in by an operator. Nothing here
comes from a sensor, and nothing runs on a schedule.

Wire protocol: newline-delimited JSON, one object per line.

  hello   {"t":"hello","id":ID,"v":1}               first line each way
  hb      {"t":"hb","h":MAPHASH12,"n":COUNT}        every 0.5 s while linked
  digest  {"t":"digest","d":{"x,y,z":HASH12,...}}   sent once on link up
  sync    {"t":"sync","o":[OBS,...]}                voxels the peer lacks or holds differently
  sync_end{"t":"sync_end","n":COUNT}                this side has sent all of its delta
  obs     {"t":"obs","o":OBS}                       live path, one new local observation
  bye     {"t":"bye","sim":BOOL}                    deliberate close; sim marks a rehearsal cut

  OBS = [x, y, z, timestamp_us, confidence, threat_type, source_id]
"""

from __future__ import annotations

import argparse
import hashlib
import http.server
import ipaddress
import json
import os
import select
import socket
import sys
import threading
import time
from collections import deque

try:
    from ocu.core.mesh_state import MeshState, VoxelObservation, voxel_key
except ImportError:  # run straight from a repo checkout
    _root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if not os.path.isdir(os.path.join(_root, "ocu")):
        raise
    sys.path.insert(0, _root)
    from ocu.core.mesh_state import MeshState, VoxelObservation, voxel_key

try:
    from ocu.core.constants import THREAT_CLASSES
except ImportError:  # pragma: no cover
    THREAT_CLASSES = ("mine", "ied", "person", "vehicle", "disturbed_soil", "safe_object", "unknown")

import dashboard

HEARTBEAT_S = 0.5
MISSED_HEARTBEATS = 3
LINK_TIMEOUT_S = HEARTBEAT_S * MISSED_HEARTBEATS
HELLO_TIMEOUT_S = 2.0
SYNC_BATCH = 100
MAX_LINE_BYTES = 8_000_000
MAX_GRID_VOXELS = 20000
CONFLICT_CONFIDENCE = 0.80

WARNING = (
    "WARNING: unauthenticated demo protocol. Use only on a trusted local network. "
    "Peers outside loopback, link-local and private (RFC1918) ranges are refused."
)

# --- address policy ---------------------------------------------------------

_PRIVATE_V4 = [ipaddress.ip_network(n) for n in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")]
_PRIVATE_V6 = [ipaddress.ip_network("fc00::/7")]


def is_private_address(addr: str) -> bool:
    """True for loopback, link-local and private ranges only."""
    try:
        ip = ipaddress.ip_address(addr.split("%")[0])
    except ValueError:
        return False
    if ip.version == 6 and ip.ipv4_mapped is not None:
        ip = ip.ipv4_mapped
    if ip.is_loopback or ip.is_link_local:
        return True
    nets = _PRIVATE_V4 if ip.version == 4 else _PRIVATE_V6
    return any(ip in n for n in nets)


def resolve_private(host: str, port: int) -> str:
    """Resolve host and return one address, refusing if any result is not private."""
    infos = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
    addrs = [i[4][0] for i in infos]
    if not addrs:
        raise ValueError("no address for %s" % host)
    bad = [a for a in addrs if not is_private_address(a)]
    if bad:
        raise ValueError("refusing non-private address %s for host %s" % (bad[0], host))
    return addrs[0]


# --- wire format and canonical hashes ---------------------------------------


def obs_to_wire(o: VoxelObservation) -> list:
    k = o.voxel_key
    return [k[0], k[1], k[2], o.timestamp_us, o.confidence, o.threat_type, o.source_id]


def obs_from_wire(w) -> VoxelObservation:
    """Parse and validate one observation from a peer. Raises ValueError."""
    if not isinstance(w, list) or len(w) != 7:
        raise ValueError("bad observation shape")
    x, y, z, t, c, typ, src = w
    for v in (x, y, z, t):
        if type(v) is not int or abs(v) > 2 ** 62:
            raise ValueError("bad integer field")
    if type(c) not in (int, float):
        raise ValueError("bad confidence")
    if not isinstance(typ, str) or not isinstance(src, str) or len(typ) > 64 or len(src) > 64:
        raise ValueError("bad string field")
    return VoxelObservation((x, y, z), t, float(c), typ, src)


def canonical_bytes(snapshot) -> bytes:
    """The map as bytes: rows sorted, no whitespace, so equal maps give equal bytes."""
    rows = sorted(obs_to_wire(o) for o in snapshot.values())
    return json.dumps(rows, separators=(",", ":"), sort_keys=True).encode("ascii")


def map_hash(snapshot) -> str:
    return hashlib.sha256(canonical_bytes(snapshot)).hexdigest()[:12]


def voxel_hash(o: VoxelObservation) -> str:
    row = json.dumps(obs_to_wire(o), separators=(",", ":")).encode("ascii")
    return hashlib.sha256(row).hexdigest()[:12]


def key_str(k) -> str:
    return "%d,%d,%d" % k


def key_parse(s: str):
    parts = s.split(",")
    if len(parts) != 3:
        raise ValueError("bad key")
    return (int(parts[0]), int(parts[1]), int(parts[2]))


# --- one TCP link -----------------------------------------------------------


class Link:
    """A connected socket. Counts every byte it sends and receives."""

    def __init__(self, replica, sock, role, addr):
        self.r = replica
        self.sock = sock
        self.role = role
        self.addr = addr
        self.send_lock = threading.Lock()
        self.closed = threading.Event()
        self.close_lock = threading.Lock()
        self.last_rx = time.monotonic()
        self.attached = False
        self.peer_id = None
        self.sent_sync_end = False
        self.peer_sync_end = False
        self.sync_sent_keys = set()
        self.sync_recv_keys = set()
        self.bytes = {"tx_digest": 0, "rx_digest": 0, "tx_sync": 0, "rx_sync": 0}
        self.sync_finalized = False
        sock.settimeout(2.0)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    # sending

    def send(self, msg, bucket=None) -> bool:
        data = (json.dumps(msg, separators=(",", ":")) + "\n").encode("ascii")
        with self.send_lock:
            if self.closed.is_set():
                return False
            try:
                self.sock.sendall(data)
            except OSError:
                self.close("send failed", "real")
                return False
        self.r._count_tx(len(data))
        if bucket:
            self.bytes["tx_" + bucket] += len(data)
        return True

    def close(self, reason, kind="real", send_bye=False):
        with self.close_lock:
            if self.closed.is_set():
                return
            if send_bye:
                try:
                    data = (json.dumps({"t": "bye", "sim": kind == "simulated"}) + "\n").encode("ascii")
                    self.sock.sendall(data)
                    self.r._count_tx(len(data))
                except OSError:
                    pass
            self.closed.set()
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
        self.r._detach(self, reason, kind)

    # receiving

    def run(self):
        self.send({"t": "hello", "id": self.r.replica_id, "v": 1})
        hello_deadline = time.monotonic() + HELLO_TIMEOUT_S
        buf = b""
        while not self.closed.is_set():
            try:
                ready = select.select([self.sock], [], [], 0.25)[0]
            except (OSError, ValueError):
                self.close("socket error", "real")
                return
            if ready:
                try:
                    chunk = self.sock.recv(65536)
                except OSError:
                    self.close("socket error", "real")
                    return
                if not chunk:
                    self.close("peer closed the connection", "real")
                    return
                self.r._count_rx(len(chunk))
                self.last_rx = time.monotonic()
                buf += chunk
                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    line, buf = buf[:nl], buf[nl + 1:]
                    self._handle_line(line, len(line) + 1)
                    if self.closed.is_set():
                        return
                if len(buf) > MAX_LINE_BYTES:
                    self.close("peer line too long", "real")
                    return
            else:
                now = time.monotonic()
                if not self.attached and now > hello_deadline:
                    self.close("no hello from peer", "real")
                    return
                if self.attached and now - self.last_rx > LINK_TIMEOUT_S:
                    self.close(
                        "no heartbeat for %.1f s (%d missed)" % (now - self.last_rx, MISSED_HEARTBEATS),
                        "real",
                    )
                    return

    def _handle_line(self, line, nbytes):
        try:
            msg = json.loads(line.decode("utf-8"))
            if not isinstance(msg, dict) or not isinstance(msg.get("t"), str):
                raise ValueError("not a message")
            self._dispatch(msg, nbytes)
        except (ValueError, KeyError, TypeError) as exc:
            self.r.log("dropped a bad message from peer: %s" % exc)

    def _dispatch(self, msg, nbytes):
        t = msg["t"]
        if t == "hello":
            if self.attached:
                return
            pid = msg.get("id")
            if not isinstance(pid, str) or not (0 < len(pid) <= 64):
                raise ValueError("bad peer id")
            self.peer_id = pid
            if not self.r._attach(self):
                self.close("another link is already up", "local")
                return
            self.attached = True
            threading.Thread(target=self._heartbeats, daemon=True).start()
            self.r._send_digest(self)
        elif not self.attached:
            raise ValueError("message before hello")
        elif t == "hb":
            self.r._on_heartbeat(msg)
        elif t == "digest":
            self.bytes["rx_digest"] += nbytes
            self.r._on_digest(self, msg)
        elif t == "sync":
            self.bytes["rx_sync"] += nbytes
            self.r._on_sync(self, msg)
        elif t == "sync_end":
            self.bytes["rx_sync"] += nbytes
            self.peer_sync_end = True
            self.r._maybe_finalize_sync(self)
        elif t == "obs":
            self.r._on_live_obs(self, msg)
        elif t == "bye":
            self.r._note_peer_bye(bool(msg.get("sim")))
        else:
            raise ValueError("unknown message type %r" % t)

    def _heartbeats(self):
        while not self.closed.wait(HEARTBEAT_S):
            if not self.r._send_hb(self):
                return


# --- the replica ------------------------------------------------------------


class Replica:
    def __init__(
        self,
        replica_id,
        listen_port=None,
        connect=None,
        bind="0.0.0.0",
        dash_port=8080,
        dash_host="127.0.0.1",
        clock_offset_s=0.0,
        allow_simulate=False,
        echo=True,
    ):
        self.replica_id = replica_id
        self.listen_port = listen_port
        self.connect = connect  # (host, port) or None
        self.bind = bind
        self.dash_port = dash_port
        self.dash_host = dash_host
        self.clock_offset_s = float(clock_offset_s)
        self.allow_simulate = allow_simulate
        self.echo = echo

        self.lock = threading.RLock()
        self.mesh = MeshState(replica_id)
        self.link = None
        self.link_since = time.monotonic()
        self.peer_id = None
        self.peer_hash = None
        self.peer_count = None
        self.peer_view = {}
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.count_lock = threading.Lock()
        self.sim_cut = False
        self.last_cut_kind = None  # "real", "simulated", "peer-simulated"
        self.last_cut_reason = ""
        self._peer_sim_pending = False
        self.last_delta = None
        self.hb_seq = 0
        self.events = deque(maxlen=200)
        self.stop_event = threading.Event()
        self.listener = None
        self.httpd = None
        self.bound_port = None
        self._threads = []
        self._last_match = None
        self._log_lock = threading.Lock()

    # logging and counters

    def log(self, text):
        stamp = time.strftime("%H:%M:%S") + ".%03d" % (int(time.time() * 1000) % 1000)
        line = "[%s] %s: %s" % (stamp, self.replica_id, text)
        with self._log_lock:
            self.events.append("%s  %s" % (stamp, text))
            if self.echo:
                print(line, flush=True)

    def _count_tx(self, n):
        with self.count_lock:
            self.tx_bytes += n

    def _count_rx(self, n):
        with self.count_lock:
            self.rx_bytes += n

    def now_us(self):
        return int((time.time() + self.clock_offset_s) * 1_000_000)

    # lifecycle

    def start(self):
        if self.listen_port is not None:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.bind, self.listen_port))
            s.listen(4)
            s.settimeout(0.25)
            self.listener = s
            self.bound_port = s.getsockname()[1]
            self.log("listening on %s:%d" % (self.bind, self.bound_port))
            self._spawn(self._accept_loop)
        if self.connect is not None:
            self.log("will connect to %s:%d" % self.connect)
            self._spawn(self._connect_loop)
        if self.dash_port is not None:
            handler = _make_handler(self)
            self.httpd = http.server.ThreadingHTTPServer((self.dash_host, self.dash_port), handler)
            self.httpd.daemon_threads = True
            self.dash_port = self.httpd.server_address[1]
            self._spawn(self.httpd.serve_forever)
            self.log("dashboard at http://%s:%d/" % (self.dash_host, self.dash_port))
        return self

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    def stop(self):
        self.stop_event.set()
        link = self.link
        if link is not None:
            link.close("replica stopped", "local")
        if self.listener is not None:
            try:
                self.listener.close()
            except OSError:
                pass
        if self.httpd is not None:
            try:
                self.httpd.shutdown()
                self.httpd.server_close()
            except OSError:
                pass

    # connection management

    def _accept_loop(self):
        while not self.stop_event.is_set():
            try:
                sock, addr = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            if not is_private_address(addr[0]):
                self.log("REFUSED connection from non-private address %s" % addr[0])
                sock.close()
                continue
            if self.sim_cut:
                sock.close()
                continue
            old = self.link
            if old is not None and time.monotonic() - old.last_rx < HEARTBEAT_S * 2:
                sock.close()
                continue
            if old is not None:
                old.close("replaced by a new connection", "real")
            t = threading.Thread(target=self._run_link, args=(sock, "accepted", addr), daemon=True)
            t.start()

    def _connect_loop(self):
        host, port = self.connect
        warned = None
        while not self.stop_event.is_set():
            if self.sim_cut or self.link is not None:
                self.stop_event.wait(0.2)
                continue
            try:
                ip = resolve_private(host, port)
            except (ValueError, OSError) as exc:
                if warned != str(exc):
                    self.log("cannot connect: %s" % exc)
                    warned = str(exc)
                self.stop_event.wait(1.0)
                continue
            try:
                sock = socket.create_connection((ip, port), timeout=1.0)
            except OSError:
                self.stop_event.wait(0.5)
                continue
            self._run_link(sock, "connected", (ip, port))

    def _run_link(self, sock, role, addr):
        link = Link(self, sock, role, addr)
        try:
            link.run()
        finally:
            link.close("link ended", "real")

    def _attach(self, link) -> bool:
        with self.lock:
            if self.link is not None and self.link is not link:
                return False
            if self.sim_cut:
                return False
            self.link = link
            self.link_since = time.monotonic()
            self.peer_id = link.peer_id
            self.peer_hash = None
            self.peer_count = None
            self.peer_view = {}
            self._last_match = None
            self._peer_sim_pending = False
        self.log("LINK UP with %s (%s %s:%s)" % (link.peer_id, link.role, link.addr[0], link.addr[1]))
        return True

    def _detach(self, link, reason, kind):
        with self.lock:
            if self.link is not link:
                return
            self.link = None
            self.link_since = time.monotonic()
            self.last_cut_reason = reason
            if kind == "real" and self._peer_sim_pending:
                kind = "peer-simulated"
            self._peer_sim_pending = False
            if kind in ("simulated", "peer-simulated"):
                self.last_cut_kind = kind
            elif kind == "real":
                self.last_cut_kind = "real"
        self.log("LINK DOWN: %s" % reason)

    def _note_peer_bye(self, simulated):
        if simulated:
            with self.lock:
                self._peer_sim_pending = True
            self.log("peer reports a SIMULATED cut (rehearsal)")

    # protocol steps

    def _send_hb(self, link) -> bool:
        with self.lock:
            h = map_hash(self.mesh.snapshot())
            n = len(self.mesh)
        return link.send({"t": "hb", "h": h, "n": n})

    def _on_heartbeat(self, msg):
        h, n = msg.get("h"), msg.get("n")
        if not isinstance(h, str) or len(h) > 64 or type(n) is not int:
            raise ValueError("bad heartbeat")
        with self.lock:
            self.peer_hash = h
            self.peer_count = n
            mine = map_hash(self.mesh.snapshot())
            match = mine == h
            changed = match != self._last_match
            self._last_match = match
        if changed:
            self.log("map hash %s vs peer %s: %s" % (mine, h, "MATCH" if match else "DIFFER"))

    def _send_digest(self, link):
        with self.lock:
            d = {key_str(k): voxel_hash(o) for k, o in self.mesh.snapshot().items()}
        link.send({"t": "digest", "d": d}, "digest")

    def _on_digest(self, link, msg):
        d = msg.get("d")
        if not isinstance(d, dict):
            raise ValueError("bad digest")
        theirs = {}
        for ks, h in d.items():
            if not isinstance(h, str):
                raise ValueError("bad digest entry")
            theirs[key_parse(ks)] = h
        with self.lock:
            self.peer_view = dict(theirs)
            mine = self.mesh.snapshot()
            to_send = [
                o for k, o in sorted(mine.items()) if theirs.get(k) != voxel_hash(o)
            ]
        for i in range(0, len(to_send), SYNC_BATCH):
            batch = to_send[i:i + SYNC_BATCH]
            if not link.send({"t": "sync", "o": [obs_to_wire(o) for o in batch]}, "sync"):
                return
            link.sync_sent_keys.update(o.voxel_key for o in batch)
            with self.lock:
                for o in batch:
                    if o.voxel_key not in theirs:
                        self.peer_view[o.voxel_key] = voxel_hash(o)
        link.send({"t": "sync_end", "n": len(to_send)}, "sync")
        link.sent_sync_end = True
        self.log("sync: peer digest has %d voxels, sending %d the peer lacks or holds differently"
                 % (len(theirs), len(to_send)))
        self._maybe_finalize_sync(link)

    def _on_sync(self, link, msg):
        items = msg.get("o")
        if not isinstance(items, list):
            raise ValueError("bad sync batch")
        parsed = [obs_from_wire(w) for w in items]
        with self.lock:
            for o in parsed:
                self.mesh.apply(o)
                self.peer_view[o.voxel_key] = voxel_hash(o)
        link.sync_recv_keys.update(o.voxel_key for o in parsed)

    def _maybe_finalize_sync(self, link):
        if link.sync_finalized or not (link.sent_sync_end and link.peer_sync_end):
            return
        link.sync_finalized = True
        sent, recv = len(link.sync_sent_keys), len(link.sync_recv_keys)
        union = len(link.sync_sent_keys | link.sync_recv_keys)
        delta = {
            "voxels": union,
            "sent": sent,
            "received": recv,
            "bytes_tx": link.bytes["tx_sync"],
            "bytes_rx": link.bytes["rx_sync"],
            "digest_bytes_tx": link.bytes["tx_digest"],
            "digest_bytes_rx": link.bytes["rx_digest"],
            "at": time.monotonic(),
        }
        with self.lock:
            self.last_delta = delta
            total = len(self.mesh)
        self.log(
            "sync done: delta %d voxels (sent %d, received %d), delta bytes tx %d rx %d, "
            "digest bytes tx %d rx %d, map now %d voxels"
            % (union, sent, recv, delta["bytes_tx"], delta["bytes_rx"],
               delta["digest_bytes_tx"], delta["digest_bytes_rx"], total)
        )
        self._send_hb(link)

    def _on_live_obs(self, link, msg):
        o = obs_from_wire(msg.get("o"))
        with self.lock:
            self.mesh.apply(o)
            self.peer_view[o.voxel_key] = voxel_hash(o)
            stored = self.mesh.get(o.voxel_key)
        self.log("live obs from %s at voxel %s (conf %.2f %s)" % (o.source_id, key_str(o.voxel_key), o.confidence, o.threat_type))
        if stored != o and not msg.get("fix"):
            # We hold a better observation of this voxel. Send it back so the sender converges.
            link.send({"t": "obs", "o": obs_to_wire(stored), "fix": True})
            with self.lock:
                self.peer_view[o.voxel_key] = voxel_hash(stored)
        self._send_hb(link)

    # operator actions

    def add_observation(self, x, y, confidence, threat_type, label="obs"):
        o = VoxelObservation(
            voxel_key=voxel_key(x, y, 0.0),
            timestamp_us=self.now_us(),
            confidence=confidence,
            threat_type=threat_type,
            source_id=self.replica_id,
        )
        with self.lock:
            changed = self.mesh.apply(o)
            stored = self.mesh.get(o.voxel_key)
            link = self.link
        k = key_str(o.voxel_key)
        if not changed:
            self.log("%s at (%.2f, %.2f) voxel %s: no change, existing observation wins" % (label, x, y, k))
            return "no change, the existing observation at %s wins" % k
        self.log("%s at (%.2f, %.2f) voxel %s conf %.2f %s" % (label, x, y, k, confidence, threat_type))
        if link is not None and link.attached:
            if link.send({"t": "obs", "o": obs_to_wire(stored)}):
                with self.lock:
                    self.peer_view[o.voxel_key] = voxel_hash(stored)
        return "recorded at voxel %s" % k

    def conflict_type(self):
        """A type that differs between the two usual replica ids; operator may override."""
        return "ied" if hashlib.sha256(self.replica_id.encode()).digest()[0] % 2 else "mine"

    def simulate_cut(self):
        if not self.allow_simulate:
            return "refused: start with --simulate-cut to enable rehearsal cuts"
        self.sim_cut = True
        with self.lock:
            self.last_cut_kind = "simulated"
            link = self.link
        self.log("SIMULATED CUT (software only, rehearsal)")
        if link is not None:
            link.close("simulated cut (software)", "simulated", send_bye=True)
        return "simulated cut active"

    def simulate_restore(self):
        if not self.allow_simulate:
            return "refused: start with --simulate-cut to enable rehearsal cuts"
        self.sim_cut = False
        self.log("simulated cut released, reconnecting")
        return "simulated cut released"

    def run_command(self, line):
        """Run one operator command. Returns a one-line result."""
        parts = line.split()
        if not parts:
            return ""
        cmd = parts[0].lower()
        try:
            if cmd == "obs":
                if len(parts) != 5:
                    return "usage: obs X Y CONF TYPE   (meters, 0 to 1, one of %s)" % ", ".join(THREAT_CLASSES)
                x, y, conf = float(parts[1]), float(parts[2]), float(parts[3])
                typ = parts[4].lower()
                if typ not in THREAT_CLASSES:
                    return "unknown TYPE %r; use one of %s" % (typ, ", ".join(THREAT_CLASSES))
                if not (conf == conf and 0.0 <= conf <= 1.0):
                    return "CONF must be a number from 0 to 1"
                if not (abs(x) < 1e6 and abs(y) < 1e6):
                    return "X and Y out of range"
                return self.add_observation(x, y, conf, typ)
            if cmd == "conflict":
                if len(parts) not in (3, 4, 5):
                    return "usage: conflict X Y [CONF [TYPE]]"
                x, y = float(parts[1]), float(parts[2])
                conf = float(parts[3]) if len(parts) > 3 else CONFLICT_CONFIDENCE
                typ = parts[4].lower() if len(parts) > 4 else self.conflict_type()
                if typ not in THREAT_CLASSES:
                    return "unknown TYPE %r" % typ
                if not (conf == conf and 0.0 <= conf <= 1.0):
                    return "CONF must be a number from 0 to 1"
                return self.add_observation(x, y, conf, typ, label="conflict")
            if cmd == "simulate-cut":
                return self.simulate_cut()
            if cmd == "simulate-restore":
                return self.simulate_restore()
            if cmd == "status":
                s = self.state()
                return "%s link=%s voxels=%d divergence=%d hash=%s peer=%s tx=%d rx=%d" % (
                    s["id"], s["link"], s["voxels"], s["divergence"], s["hash"], s["peer_hash"], s["tx_bytes"], s["rx_bytes"])
            if cmd == "help":
                return ("commands: obs X Y CONF TYPE | conflict X Y [CONF [TYPE]] | status | "
                        "simulate-cut | simulate-restore | quit")
        except ValueError as exc:
            return "bad command: %s" % exc
        return "unknown command %r (try help)" % cmd

    # state for the dashboard

    def _divergent_keys(self, snapshot, linked, mine_hash):
        if linked and self.peer_hash is not None and self.peer_hash == mine_hash:
            return set()
        return {k for k, o in snapshot.items() if self.peer_view.get(k) != voxel_hash(o)}

    def state(self):
        with self.lock:
            snap = self.mesh.snapshot()
            link = self.link
            linked = link is not None and link.attached
            mine = map_hash(snap)
            div_keys = self._divergent_keys(snap, linked, mine)
            peer_hash = self.peer_hash
            ld = None
            if self.last_delta is not None:
                ld = dict(self.last_delta)
                ld["age_s"] = round(time.monotonic() - ld.pop("at"), 1)
            grid = []
            for k, o in list(snap.items())[:MAX_GRID_VOXELS]:
                grid.append([k[0], k[1], o.source_id, o.threat_type, o.confidence, 1 if k in div_keys else 0])
            sim_banner = self.sim_cut or (self.last_cut_kind in ("simulated", "peer-simulated"))
            out = {
                "id": self.replica_id,
                "peer_id": self.peer_id if linked else None,
                "link": "LINKED" if linked else "DOWN",
                "since_s": round(time.monotonic() - self.link_since, 1),
                "cut_reason": "" if linked else self.last_cut_reason,
                "voxels": len(snap),
                "divergence": len(div_keys),
                "hash": mine,
                "peer_hash": peer_hash,
                "hash_state": None if peer_hash is None else ("MATCH" if peer_hash == mine else "DIFFER"),
                "hash_live": linked,
                "tx_bytes": self.tx_bytes,
                "rx_bytes": self.rx_bytes,
                "last_delta": ld,
                "sim_banner": sim_banner,
                "sim_active": self.sim_cut,
                "allow_simulate": self.allow_simulate,
                "clock_offset_s": self.clock_offset_s,
                "grid": grid,
                "voxel_m": 0.05,
                "threat_classes": list(THREAT_CLASSES),
                "events": list(self.events)[-60:],
            }
        return out


# --- dashboard HTTP ----------------------------------------------------------


def _make_handler(replica):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _allowed(self):
            if is_private_address(self.client_address[0]):
                return True
            self.send_error(403, "non-private client")
            return False

        def _reply(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not self._allowed():
                return
            if self.path == "/state":
                self._reply(200, json.dumps(replica.state()).encode("utf-8"), "application/json")
            elif self.path in ("/", "/index.html"):
                self._reply(200, dashboard.PAGE.encode("utf-8"), "text/html; charset=utf-8")
            else:
                self.send_error(404)

        def do_POST(self):
            if not self._allowed():
                return
            if self.path != "/cmd":
                self.send_error(404)
                return
            try:
                n = int(self.headers.get("Content-Length", "0"))
                if n > 4096:
                    raise ValueError("too large")
                cmd = json.loads(self.rfile.read(n).decode("utf-8")).get("cmd", "")
                if not isinstance(cmd, str):
                    raise ValueError("bad cmd")
            except (ValueError, AttributeError):
                self.send_error(400)
                return
            result = replica.run_command(cmd)
            self._reply(200, json.dumps({"msg": result}).encode("utf-8"), "application/json")

    return Handler


# --- command line -------------------------------------------------------------


def selftest():
    import platform

    from ocu.core import mesh_state

    a = VoxelObservation((1, 2, 0), 1000, 0.9, "mine", "rover")
    b = VoxelObservation((1, 2, 0), 1000, 0.9, "ied", "aircraft_1")
    digest = hashlib.sha256(mesh_state.__loader__.get_data(mesh_state.__file__)).hexdigest()
    print("python", platform.python_version())
    print("mesh_state", mesh_state.__file__)
    print("mesh_state sha256", digest)
    print("merge commutative on full tie:", mesh_state.merge(a, b) == mesh_state.merge(b, a))
    print("PySide6 imported:", "PySide6" in sys.modules)
    ok = mesh_state.merge(a, b) == mesh_state.merge(b, a) and "PySide6" not in sys.modules
    print("selftest", "OK" if ok else "FAILED")
    return 0 if ok else 1


def parse_args(argv):
    p = argparse.ArgumentParser(description="One replica of the shared map for the two-device demo.")
    p.add_argument("--id", dest="replica_id", help="replica id, for example rover or aircraft")
    p.add_argument("--listen", type=int, metavar="PORT", help="accept the peer on this TCP port")
    p.add_argument("--connect", metavar="HOST:PORT", help="connect to the peer; retries until it is up")
    p.add_argument("--bind", default="0.0.0.0", help="address to listen on (default 0.0.0.0)")
    p.add_argument("--dash-port", type=int, default=8080, help="dashboard port (default 8080)")
    p.add_argument("--dash-host", default="127.0.0.1", help="dashboard bind address (default 127.0.0.1)")
    p.add_argument("--clock-offset-s", type=float, default=0.0, help="skew this replica's clock by N seconds")
    p.add_argument("--simulate-cut", action="store_true",
                   help="rehearsal only: enable the simulate-cut command, which drops the link in software")
    p.add_argument("--no-stdin", action="store_true", help="do not read commands from stdin")
    p.add_argument("--selftest", action="store_true", help="check the bundled code and exit")
    args = p.parse_args(argv)
    if args.selftest:
        return args
    if not args.replica_id:
        p.error("--id is required")
    if args.listen is None and not args.connect:
        p.error("give --listen PORT, --connect HOST:PORT, or both")
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.selftest:
        return selftest()
    print(WARNING, flush=True)
    connect = None
    if args.connect:
        host, _, port = args.connect.rpartition(":")
        if not host or not port.isdigit():
            print("--connect needs HOST:PORT", file=sys.stderr)
            return 2
        connect = (host, int(port))
    rep = Replica(
        args.replica_id,
        listen_port=args.listen,
        connect=connect,
        bind=args.bind,
        dash_port=args.dash_port,
        dash_host=args.dash_host,
        clock_offset_s=args.clock_offset_s,
        allow_simulate=args.simulate_cut,
    )
    rep.start()
    if args.clock_offset_s:
        rep.log("clock skewed by %+.1f s" % args.clock_offset_s)
    if args.simulate_cut:
        rep.log("REHEARSAL controls enabled: simulate-cut is software only")
    print("commands: obs X Y CONF TYPE | conflict X Y [CONF [TYPE]] | status | quit", flush=True)
    try:
        if args.no_stdin:
            rep.stop_event.wait()
        else:
            while not rep.stop_event.is_set():
                line = sys.stdin.readline()
                if line == "":
                    rep.stop_event.wait()  # stdin closed: keep serving
                    break
                line = line.strip()
                if line.lower() in ("quit", "exit"):
                    break
                result = rep.run_command(line)
                if result:
                    print(result, flush=True)
    except KeyboardInterrupt:
        pass
    rep.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
