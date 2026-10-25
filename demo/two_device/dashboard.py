"""The dashboard page served by replica.py. One self-contained HTML string.

No CDN, no external fonts, no network requests except to this replica.
Large type so it reads at 1080p when filmed from a phone.
"""

from __future__ import annotations

FOOTER = (
    "Real devices, real network, real link cut. "
    "Observations are injected by the operator, not from sensors."
)

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shared map replica</title>
<style>
  :root {
    --bg: #0b0f14; --panel: #121923; --line: #223041; --text: #e8eef5; --dim: #8fa3b8;
    --ok: #35d07f; --bad: #ff5d5d; --warn: #ffb020; --local: #38bdf8; --peer: #fb923c;
  }
  * { box-sizing: border-box; }
  html, body { margin: 0; background: var(--bg); color: var(--text);
    font-family: -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
  body { padding: 14px 18px 84px 18px; font-size: 22px; }
  header { display: flex; align-items: baseline; gap: 28px; flex-wrap: wrap; }
  h1 { margin: 0; font-size: 44px; letter-spacing: 1px; }
  #link { font-size: 44px; font-weight: 800; padding: 2px 18px; border-radius: 8px; }
  #link.up { background: var(--ok); color: #04210f; }
  #link.down { background: var(--bad); color: #2a0505; }
  #since { font-size: 26px; color: var(--dim); }
  #sim { display: none; margin: 10px 0; padding: 12px 18px; background: var(--warn); color: #1a1100;
    font-size: 40px; font-weight: 900; text-align: center; border-radius: 8px; letter-spacing: 2px; }
  #tiles { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; margin: 12px 0; }
  .tile { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 10px 14px; }
  .tile .k { color: var(--dim); font-size: 20px; text-transform: uppercase; letter-spacing: 1px; }
  .tile .v { font-size: 40px; font-weight: 700; font-variant-numeric: tabular-nums; word-break: break-all; }
  .tile .s { color: var(--dim); font-size: 20px; }
  .match { color: var(--ok); } .differ { color: var(--bad); }
  #main { display: grid; grid-template-columns: 1.15fr 1fr; gap: 12px; }
  #mapbox, #logbox { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 10px; }
  canvas { width: 100%; height: auto; display: block; background: #0a1017; border-radius: 6px; }
  #legend { font-size: 20px; color: var(--dim); margin-top: 6px; }
  #legend b { display: inline-block; width: 16px; height: 16px; margin: 0 6px 0 14px; vertical-align: -2px; }
  #log { height: 430px; overflow-y: auto; font-family: ui-monospace, Menlo, Consolas, monospace; font-size: 18px;
    white-space: pre-wrap; color: #c8d6e5; }
  #ctl { margin-top: 10px; display: flex; flex-wrap: wrap; gap: 8px; align-items: center; font-size: 20px; }
  #ctl input, #ctl select { font-size: 22px; padding: 6px 8px; background: #0a1017; color: var(--text);
    border: 1px solid var(--line); border-radius: 6px; width: 110px; }
  #ctl select { width: 190px; }
  button { font-size: 22px; padding: 8px 16px; border-radius: 6px; border: 1px solid var(--line);
    background: #1b2838; color: var(--text); cursor: pointer; }
  button:hover { background: #24364c; }
  #msg { color: var(--dim); font-size: 18px; min-height: 22px; width: 100%; }
  footer { position: fixed; left: 0; right: 0; bottom: 0; background: #05080c; border-top: 2px solid var(--line);
    padding: 12px 18px; font-size: 26px; font-weight: 700; text-align: center; color: var(--text); }
  @media (max-width: 1100px) { #main { grid-template-columns: 1fr; } #tiles { grid-template-columns: repeat(2, 1fr); } }
</style>
</head>
<body>
<header>
  <h1 id="rid">replica</h1>
  <span id="link" class="down">DOWN</span>
  <span id="since"></span>
</header>
<div id="sim">SIMULATED CUT</div>
<div id="tiles">
  <div class="tile"><div class="k">Voxels</div><div class="v" id="voxels">0</div><div class="s" id="clock"></div></div>
  <div class="tile"><div class="k">Divergence</div><div class="v" id="div">0</div><div class="s">held here, not matching peer</div></div>
  <div class="tile"><div class="k">Map hash</div><div class="v" id="hash"></div><div class="s">SHA-256, first 12</div></div>
  <div class="tile"><div class="k">Peer hash</div><div class="v" id="phash"></div><div class="s" id="hstate"></div></div>
  <div class="tile"><div class="k">Bytes sent</div><div class="v" id="tx">0</div></div>
  <div class="tile"><div class="k">Bytes received</div><div class="v" id="rx">0</div></div>
  <div class="tile" style="grid-column: span 2"><div class="k">Last reconnect delta</div>
    <div class="v" id="delta">none yet</div><div class="s" id="delta2"></div></div>
</div>
<div id="main">
  <div id="mapbox">
    <canvas id="cv" width="900" height="620"></canvas>
    <div id="legend"></div>
    <div id="ctl">
      X <input id="x" value="2.0"> Y <input id="y" value="3.0"> Conf <input id="c" value="0.90">
      <select id="t"></select>
      <button id="bobs">Add observation</button>
      <button id="bcon">Conflict at X, Y</button>
      <span id="simbtns" style="display:none">
        <button id="bcut">Simulate cut</button><button id="bres">Simulate restore</button></span>
      <div id="msg"></div>
    </div>
  </div>
  <div id="logbox"><div id="log"></div></div>
</div>
<footer>__FOOTER__</footer>
<script>
"use strict";
var $ = function (id) { return document.getElementById(id); };
var LOCAL_COLOR = "#38bdf8", PEER_COLOR = "#fb923c", OTHER_COLOR = "#a3a3a3";
var typesFilled = false;

function fmtBytes(n) { return n.toLocaleString("en-US") + " B"; }

function draw(s) {
  var cv = $("cv"), g = cv.getContext("2d"), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H);
  var vm = s.voxel_m, pts = s.grid;
  var minX = 0, maxX = 10, minY = 0, maxY = 10;
  for (var i = 0; i < pts.length; i++) {
    var wx = pts[i][0] * vm, wy = pts[i][1] * vm;
    if (wx < minX) minX = wx; if (wx > maxX) maxX = wx;
    if (wy < minY) minY = wy; if (wy > maxY) maxY = wy;
  }
  var pad = 1.0; minX -= pad; maxX += pad; minY -= pad; maxY += pad;
  var span = Math.max(maxX - minX, (maxY - minY) * W / H);
  var scale = (W - 40) / span;
  function px(wx) { return 30 + (wx - minX) * scale; }
  function py(wy) { return H - 30 - (wy - minY) * scale; }
  var step = 1; while (span / step > 12) step *= 2;
  g.strokeStyle = "#17222f"; g.fillStyle = "#6b7f93"; g.font = "16px sans-serif"; g.lineWidth = 1;
  for (var gx = Math.ceil(minX / step) * step; gx <= maxX; gx += step) {
    g.beginPath(); g.moveTo(px(gx), 0); g.lineTo(px(gx), H); g.stroke(); g.fillText(gx + " m", px(gx) + 3, H - 8);
  }
  for (var gy = Math.ceil(minY / step) * step; gy <= maxY; gy += step) {
    g.beginPath(); g.moveTo(0, py(gy)); g.lineTo(W, py(gy)); g.stroke(); g.fillText(gy + " m", 4, py(gy) - 3);
  }
  var r = Math.max(7, vm * scale);
  for (var j = 0; j < pts.length; j++) {
    var p = pts[j], cx = px(p[0] * vm), cy = py(p[1] * vm);
    g.fillStyle = p[2] === s.id ? LOCAL_COLOR : (s.peer_id && p[2] === s.peer_id ? PEER_COLOR : OTHER_COLOR);
    g.fillRect(cx - r / 2, cy - r / 2, r, r);
    if (p[5]) { g.strokeStyle = "#ffffff"; g.lineWidth = 3; g.strokeRect(cx - r / 2 - 4, cy - r / 2 - 4, r + 8, r + 8); }
  }
  var lg = $("legend");
  lg.textContent = "";
  function add(color, text) {
    var b = document.createElement("b"); b.style.background = color; lg.appendChild(b);
    lg.appendChild(document.createTextNode(text));
  }
  add(LOCAL_COLOR, "seen by " + s.id);
  add(PEER_COLOR, "seen by " + (s.peer_id || "peer"));
  var ring = document.createElement("b"); ring.style.border = "3px solid #fff"; ring.style.width = "10px"; ring.style.height = "10px";
  lg.appendChild(ring); lg.appendChild(document.createTextNode("peer lacks it or holds it differently"));
}

function render(s) {
  $("rid").textContent = s.id;
  var l = $("link"); l.textContent = s.link; l.className = s.link === "LINKED" ? "up" : "down";
  $("since").textContent = (s.link === "LINKED" ? "linked" : "down") + " for " + s.since_s.toFixed(1) + " s" +
    (s.link === "DOWN" && s.cut_reason ? "  (" + s.cut_reason + ")" : "");
  $("sim").style.display = s.sim_banner ? "block" : "none";
  $("voxels").textContent = s.voxels;
  $("clock").textContent = s.clock_offset_s ? "clock skewed " + (s.clock_offset_s > 0 ? "+" : "") + s.clock_offset_s + " s" : "clock: local";
  $("div").textContent = s.divergence;
  $("div").className = "v " + (s.divergence ? "differ" : "match");
  $("hash").textContent = s.hash;
  $("phash").textContent = s.peer_hash || "unknown";
  var hs = $("hstate");
  if (s.hash_state) {
    hs.textContent = s.hash_state + (s.hash_live ? "" : " (last known, link down)");
    hs.className = "s " + (s.hash_state === "MATCH" ? "match" : "differ");
    hs.style.fontWeight = "800"; hs.style.fontSize = "30px";
  } else { hs.textContent = "no peer hash yet"; hs.className = "s"; hs.style.fontWeight = ""; hs.style.fontSize = ""; }
  $("tx").textContent = fmtBytes(s.tx_bytes);
  $("rx").textContent = fmtBytes(s.rx_bytes);
  var d = s.last_delta;
  if (d) {
    $("delta").textContent = d.voxels + " voxels, " + fmtBytes(d.bytes_tx + d.bytes_rx);
    $("delta2").textContent = "sent " + d.sent + " (" + fmtBytes(d.bytes_tx) + "), received " + d.received + " (" +
      fmtBytes(d.bytes_rx) + "); digest " + fmtBytes(d.digest_bytes_tx + d.digest_bytes_rx) + "; " + d.age_s + " s ago";
  } else { $("delta").textContent = "none yet"; $("delta2").textContent = ""; }
  $("simbtns").style.display = s.allow_simulate ? "inline" : "none";
  if (!typesFilled) {
    var sel = $("t");
    s.threat_classes.forEach(function (t) { var o = document.createElement("option"); o.value = t; o.textContent = t; sel.appendChild(o); });
    typesFilled = true;
  }
  var log = $("log"), stick = log.scrollTop + log.clientHeight >= log.scrollHeight - 30;
  log.textContent = s.events.join("\\n");
  if (stick) log.scrollTop = log.scrollHeight;
  draw(s);
}

function tick() {
  fetch("/state", { cache: "no-store" }).then(function (r) { return r.json(); }).then(render)
    .catch(function () { $("link").textContent = "NO REPLICA"; $("link").className = "down"; });
}

function send(cmd) {
  fetch("/cmd", { method: "POST", body: JSON.stringify({ cmd: cmd }) }).then(function (r) { return r.json(); })
    .then(function (j) { $("msg").textContent = j.msg; tick(); });
}
function xy() { return $("x").value + " " + $("y").value; }
$("bobs").onclick = function () { send("obs " + xy() + " " + $("c").value + " " + $("t").value); };
$("bcon").onclick = function () { send("conflict " + xy()); };
$("bcut").onclick = function () { send("simulate-cut"); };
$("bres").onclick = function () { send("simulate-restore"); };
tick(); setInterval(tick, 500);
</script>
</body>
</html>
""".replace("__FOOTER__", FOOTER)
