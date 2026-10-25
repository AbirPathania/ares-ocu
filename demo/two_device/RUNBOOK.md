# Two-device shared-map demo: runbook

Two real devices run the same merge code (`ocu/core/mesh_state.py`), talk over a
real Wi-Fi network, and are cut apart by turning Wi-Fi off on one of them.

**The honest label, which is also the fixed footer on both dashboards:**
"Real devices, real network, real link cut. Observations are injected by the
operator, not from sensors."

Say it that way on camera. This is not a field result. No sensor is involved
and nothing runs on a schedule.

## What was and was not tested

Tested on the Mac, on loopback, by the automated tests and by two real
processes through a proxy that swallows traffic: the merge, the cut detection
by missed heartbeats, the reconnect delta, the byte counters, the dashboard, and
the bundle running under `python -I` on Python 3.9.6 and 3.13.

**Not tested:** the Android tablet, Termux, a real Wi-Fi cut, how long Android
takes to rejoin Wi-Fi, and any iPad app. If something on the tablet behaves
differently from this runbook, trust the tablet and tell me what it did.

iPad: I have not tested a-Shell or Pythonista. Both ship Python 3 with the
standard library, which is all the replica needs, but iOS limits background
networking and I do not know how either app behaves when Wi-Fi drops and returns.
A second laptop running the same commands as the Mac is the safe alternative.

## 1. Build the bundle on the Mac

From the repository root:

```bash
python3 demo/two_device/bundle.py
```

This writes `demo/two_device/dist/two_device_replica.pyz`. It holds the real
`mesh_state.py` and `constants.py` copied byte for byte, the replica, the
dashboard, and a `MANIFEST.txt` of SHA-256 hashes. It needs only Python 3.9 or
newer and imports nothing outside the standard library.

Use the same bundle on both devices so the two run identical code.

Check it on the Mac first:

```bash
python3 -I demo/two_device/dist/two_device_replica.pyz --selftest
```

Expect the last line `selftest OK` and the line `merge commutative on full tie: True`.

## 2. Mac setup

Find the Mac's Wi-Fi address (use `en1` if `en0` prints nothing):

```bash
ipconfig getifaddr en0
```

The first time the replica listens, macOS may ask whether to allow incoming
connections for Python. Choose Allow. If the tablet cannot connect, look at
System Settings, Network, Firewall.

## 3. Android tablet setup (Termux)

1. Install Termux from F-Droid, not from Google Play. The Play Store build is
   outdated and may fail to install packages.
2. In Termux: `pkg update`, then `pkg install python`.
3. Run `termux-wake-lock` so Android does not suspend Termux. Keep the screen on
   and the tablet on its charger while filming.
4. Android Settings, Apps, Termux, Battery: set it to Unrestricted (the wording
   varies; it is the option that disables battery optimization for the app).
5. Put the bundle on the tablet. Pick one:

   **Over Wi-Fi from the Mac.** On the Mac:

   ```bash
   cd demo/two_device/dist && python3 -m http.server 8000
   ```

   On the tablet, in Termux, with the Mac's address from step 2:

   ```bash
   curl -O http://MAC_IP:8000/two_device_replica.pyz
   ```

   Then stop the Mac's `http.server` with Control-C. It serves that folder to
   anyone on the network while it runs.

   **Over USB.** `adb push demo/two_device/dist/two_device_replica.pyz /sdcard/Download/`, then in Termux
   run `termux-setup-storage` once and `cp ~/storage/downloads/two_device_replica.pyz ~/`.

6. Check that the file arrived intact. On the Mac: `shasum -a 256 demo/two_device/dist/two_device_replica.pyz`.
   On the tablet: `sha256sum two_device_replica.pyz`. The two hashes must match.
7. On the tablet: `python -I two_device_replica.pyz --selftest`. Expect `selftest OK`.
   Note the `mesh_state sha256` line. It must equal the one the Mac printed.

The tablet's own address: Settings, Wi-Fi, tap the connected network. Termux can
also print it with `ifconfig` after `pkg install net-tools`. You only need the
Mac's address on the tablet, not the reverse.

## 4. Start the two replicas

Both devices must be on the same Wi-Fi network. The Mac listens and the tablet
connects. Either may be started first; the connecting side retries every half
second until the other is up, and both reconnect on their own after a cut.

Mac, in a terminal at the repository root:

```bash
python3 demo/two_device/dist/two_device_replica.pyz --id rover --listen 9000
```

Tablet, in Termux (replace MAC_IP):

```bash
python -I two_device_replica.pyz --id aircraft --connect MAC_IP:9000
```

Each device then serves its own dashboard on its own port 8080, reachable only
from that device. Open `http://127.0.0.1:8080/` in a browser on each one. Both
show LINKED within a second or two.

The startup line "WARNING: unauthenticated demo protocol" is intended. The
protocol has no authentication or encryption. Use it on a network you trust. The
replica refuses any peer that is not on a private, link-local or loopback address.

If the Mac's port 8080 is taken, add `--dash-port 8090` and use that port.

## 5. Commands

Type these at the terminal of the replica, or use the buttons under the map on
its dashboard.

| command | what it does |
|---|---|
| `obs X Y CONF TYPE` | record an observation at X, Y meters, for example `obs 12 5 0.9 mine`. TYPE is one of mine, ied, person, vehicle, disturbed_soil, safe_object, unknown |
| `conflict X Y` | record a deliberately different observation at that voxel: confidence 0.80, type `mine` or `ied` depending on this replica's id |
| `status` | one line of state |
| `dump [FILE]` | write this replica's whole map as canonical JSON, for diffing |
| `quit` | stop |

Nothing is generated for you. Every observation is typed or clicked.

## 6. The real cut

Turn **Wi-Fi off on the tablet** (the Wi-Fi switch in its settings or quick panel). No software flag is involved. Each side notices because heartbeats, sent
every 0.5 s, stop arriving; after 3 missed heartbeats (about 1.5 s of silence,
so 1 to 2 seconds after the cut) it shows DOWN and the reason
"no heartbeat for 1.5 s (3 missed)".

Restore by turning Wi-Fi back on. Android may need a few seconds to rejoin the
network. The tablet then reconnects by itself.

`--simulate-cut` exists for rehearsal only. It drops the link in software, and a
bold "SIMULATED CUT" banner stays on both dashboards until a real cut replaces it.
Do not use it for the video.

## 7. Filming sequence, about 75 seconds

Frame both screens together, phone camera at 1080p. The type is large on purpose.
Do not stop the processes between shots; their state is the demonstration.

**Take 1: the cut (about 50 seconds)**

1. 0 to 8 s. Both dashboards show LINKED, equal voxel counts, equal map hash,
   MATCH, and the footer. Type `obs 10 5 0.9 mine` on the Mac. The voxel appears
   on the tablet's map almost at once (on loopback it took about a millisecond; on Wi-Fi I have not measured it).
2. 8 to 15 s. Turn Wi-Fi off on the tablet, with the hand and screen in frame.
   Both dashboards go DOWN within about two seconds.
3. 15 to 35 s. Inject different observations on each side. On the Mac, `obs 4 4 0.8 ied`,
   `obs 6 4 0.7 person`. On the tablet, tap Add observation two or three times with different X and Y.
   Then `conflict 20 20` on the Mac and the Conflict button with X 20, Y 20 on the tablet. Divergence rises on
   each side one voxel at a time, the map hashes read DIFFER, and the maps show rings around voxels the other side does not have.
4. 35 to 50 s. Turn Wi-Fi on. Wait for LINKED on both. The "last reconnect
   delta" tile shows the number of voxels that crossed and the bytes. Divergence
   returns to 0 and the map hashes read MATCH with the same 12 characters on both
   screens. Read the hash aloud.

**Take 2: the clock skew (about 25 seconds)**

Restart the tablet's replica with its clock set ten minutes wrong:

```bash
python -I two_device_replica.pyz --id aircraft --connect MAC_IP:9000 --clock-offset-s 600
```

Its dashboard shows "clock skewed +600 s". Repeat a short version of take 1: cut,
`conflict 20 20` on both sides, restore. The hashes read MATCH again.

Know what the audience will see: both replicas record the conflict at the same
confidence, so the timestamp decides, and the skewed clock is 600 s ahead, so the
tablet's observation wins on both devices. That is the merge rule working
(confidence first, then timestamp, then source id, then type), and it is also the
reason a bad clock can win ties. Say so if asked. Higher confidence beats any clock.

## 8. What to say on camera

1. "These are two real devices on a real network, and I cut the link by turning off Wi-Fi."
2. "The observations are typed in by me. They are not from sensors."
3. "The next step is this same merge running on vehicle hardware."

## 9. Reading the numbers

- **Divergence**: voxels this device holds that its peer, by the last digest it
  heard, does not hold or holds differently. It counts only this side's holdings.
- **Last reconnect delta**: voxels that crossed in the reconnect phase, with each
  voxel counted once even if both sides sent it (a conflict), plus the bytes of
  those voxels in each direction. The digest each side sends first is counted
  separately. Voxels that both sides already agreed on are not resent.
- **Bytes sent and received** are counted where the program calls the socket.
  During a real cut, bytes written to the socket but never delivered still count
  as sent, so one side's sent total will not equal the other side's received total.
- **Map hash**: SHA-256 of the whole map in canonical form, first 12 hex characters.
  While the link is up the peer's hash arrives in each heartbeat. While it is down,
  the peer hash shown is the last one heard and is labeled that way.

## 10. If the reconnect shows DIFFER

Do not retake. A persistent DIFFER after reconnect is a real bug, not a bad take,
and the evidence is exactly what is on screen now.

First rule out the benign cases. Wait 5 seconds with both dashboards LINKED and
nobody typing. A DIFFER that lasts under a second right after a command or a
reconnect is a heartbeat arriving before the data it describes. If it clears, it is not a bug.

If it stays:

1. Do not restart either process. Photograph both screens, including the log.
2. On each device, run `dump`. That writes `replica_rover_map.json` and
   `replica_aircraft_map.json` in the folder the replica was started from. Copy one to the other machine and run `diff` on the two files. The
   rows that differ are the voxels the replicas disagree about.
3. Run `python -I two_device_replica.pyz --selftest` on both and compare the
   `mesh_state sha256` lines. If they differ, the devices ran different code,
   which is a setup error and not a merge error.
4. Copy both terminals' output. Send me the two map files, the logs and the photographs.

Possible causes, in the order I would look: different code on the two devices; a
voxel whose short digest hash collided, which should be astronomically rare; a
real defect in the sync or live path that the loopback tests did not reach.

## 11. Tests that back this runbook

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests/test_two_device.py tests/test_mesh_state.py
```

They cover a closed-socket cut and a silent cut that must be found by missed
heartbeats, the exact delta count after divergent injection, a conflict on one
voxel, clock offsets of +600 and -600 seconds, refusal of public addresses,
malformed peer messages, and the bundle importing and running under `python -I`
without PySide6 and byte-identical to the repository's `mesh_state.py`.
