#!/usr/bin/env python3
import fcntl
import json
import os
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from flask import Flask, jsonify, request, send_file
except ImportError as exc:
    raise SystemExit("Flask is required: python3 -m pip install flask") from exc


def acquire_singleton_lock(name: str):
    lock_file = open(f"/tmp/{name}.lock", "w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise SystemExit(f"{name} is already running; refusing duplicate start") from exc
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file


HTML = """<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Refuge Conveyor Control</title>
  <style>
    :root { color-scheme: light; --blue:#2563eb; --green:#16a34a; --red:#dc2626; --ink:#111827; --muted:#6b7280; --line:#d9dee7; --bg:#f4f6f8; }
    * { box-sizing: border-box; }
    body { margin:0; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background:var(--bg); color:var(--ink); }
    main { width:min(1780px, calc(100vw - 40px)); margin:22px auto 40px; }
    h1 { font-size:34px; margin:0 0 18px; letter-spacing:0; }
    h2 { font-size:24px; margin:0 0 14px; }
    section { background:#fff; border:1px solid #e4e7ec; border-radius:8px; padding:18px; margin:16px 0; box-shadow:0 4px 14px rgba(15,23,42,.08); }
    .row { display:flex; flex-wrap:wrap; align-items:center; gap:10px; margin:8px 0; }
    input, select { height:39px; border:1px solid #cfd6e0; border-radius:6px; padding:0 10px; font-size:16px; background:white; min-width:90px; }
    button { height:39px; border:0; border-radius:6px; padding:0 15px; font-size:16px; color:white; background:var(--blue); cursor:pointer; }
    button.gray { background:#4b5563; }
    button.green { background:var(--green); }
    button.red { background:var(--red); }
    button.orange { background:#ea580c; }
    button.purple { background:#7c3aed; }
    .status-grid { display:grid; grid-template-columns:repeat(6, minmax(120px,1fr)); gap:10px; }
    .floor-grid { display:grid; grid-template-columns:repeat(8, minmax(105px,1fr)); gap:10px; }
    .tile { background:#f1f3f7; border-radius:8px; min-height:64px; display:flex; flex-direction:column; align-items:center; justify-content:center; }
    .tile small { color:var(--muted); }
    .tile strong { font-size:20px; }
    table { width:100%; border-collapse:collapse; table-layout:fixed; }
    th, td { border:1px solid #e0e4ea; padding:10px; text-align:center; }
    th { background:#f1f3f7; }
    .cal-table input { width:100%; min-width:0; text-align:center; }
    .cal-table th, .cal-table td { padding:7px; }
    h3 { font-size:18px; margin:12px 0 8px; }
    .compare-layout { display:grid; grid-template-columns:1fr 1fr; gap:14px; align-items:start; margin-top:12px; }
    .viz-panel { border:1px solid #e0e4ea; border-radius:8px; padding:12px; background:#fbfcfe; }
    .viz-title { display:flex; align-items:center; gap:8px; margin:12px 0 8px; }
    .live-badge { display:none; border-radius:999px; background:#dc2626; color:white; padding:2px 8px; font-size:11px; letter-spacing:0; }
    .live-badge.on { display:inline-flex; align-items:center; }
    .live-note { min-height:20px; margin:-2px 0 6px 50px; }
    .sim-image-wrap { border:1px solid #e0e4ea; border-radius:8px; padding:12px; background:#fbfcfe; margin-top:12px; }
    .plan-banner { border:1px solid #cfd6e0; border-left:6px solid var(--blue); border-radius:8px; padding:12px 14px; margin:12px 0; background:#f8fafc; }
    .plan-banner small { display:block; color:var(--muted); font-size:13px; margin-bottom:4px; }
    .plan-banner strong { display:block; font-size:22px; line-height:1.25; }
    .plan-banner .sub { color:var(--muted); font-size:14px; margin-top:3px; }
    .sim-image { display:block; width:100%; max-height:760px; object-fit:contain; background:#fff; border:1px solid #e5e7eb; border-radius:6px; }
    .sim-image.empty { min-height:280px; display:flex; align-items:center; justify-content:center; color:var(--muted); }
    .beltviz { display:grid; grid-template-columns:42px 1fr; gap:8px; align-items:center; margin:9px 0; }
    .beltname { color:var(--muted); font-weight:700; text-align:right; }
    .beltbar { position:relative; height:42px; border:1px solid #cfd6e0; border-radius:6px; background:linear-gradient(90deg,#f8fafc,#eef2f7); overflow:hidden; }
    .beltbar.live-moving { border-color:#2563eb; box-shadow:0 0 0 2px rgba(37,99,235,.16) inset; background:repeating-linear-gradient(115deg,#eef6ff 0,#eef6ff 12px,#dbeafe 12px,#dbeafe 24px); background-size:40px 40px; animation:liveFlow 900ms linear infinite; }
    .beltbar::before { content:"250"; position:absolute; left:0; top:0; bottom:0; width:var(--gap-pct); border-right:2px dashed #94a3b8; color:#64748b; font-size:11px; padding:2px 4px; pointer-events:none; }
    @keyframes liveFlow { from { background-position:0 0; } to { background-position:40px 0; } }
    .pkg { position:absolute; top:7px; height:28px; border-radius:5px; border:1px solid rgba(15,23,42,.25); color:white; font-size:12px; font-weight:700; display:flex; align-items:center; justify-content:center; overflow:hidden; white-space:nowrap; min-width:18px; }
    .pkg.target { outline:3px solid #f59e0b; outline-offset:-2px; }
    .plan-table { max-height:190px; overflow:auto; border:1px solid #e0e4ea; border-radius:8px; }
    .log { height:220px; overflow:auto; background:#0f172a; color:#dbeafe; border-radius:8px; padding:12px; font:14px/1.45 ui-monospace, SFMono-Regular, Menlo, monospace; white-space:pre-wrap; }
    .small { color:var(--muted); font-size:14px; }
    @media (max-width: 900px) {
      main { width:calc(100vw - 20px); margin:14px auto; }
      .status-grid { grid-template-columns:repeat(2, minmax(120px,1fr)); }
      .floor-grid { grid-template-columns:repeat(2, minmax(120px,1fr)); }
      .compare-layout { grid-template-columns:1fr; }
      input, select, button { flex:1 1 130px; min-width:0; }
    }
  </style>
</head>
<body>
<main>
  <h1>Refuge Circulation Conveyor Control</h1>

  <section>
    <h2>System</h2>
    <div class="row">
      <input id="raw" placeholder='예: BOX 2 / BOXID 7 3 / START 7 / MOVE 1 1 100'>
      <button onclick="sendRaw()">Send</button>
      <button class="red" onclick="cmd({cmd:'stop'})">STOP</button>
      <button class="orange" onclick="cmd({cmd:'clear_fault'})">CLEAR FAULT</button>
      <button class="gray" onclick="cmd({cmd:'status'})">STATUS</button>
      <button class="orange" onclick="cmd({cmd:'clear'})">CLEAR DB</button>
      <button class="purple" onclick="cmd({cmd:'zero'})">ZERO</button>
    </div>
    <div class="small" id="lastSend">Last send: -</div>
  </section>

  <section>
    <h2>Status</h2>
    <div class="status-grid">
      <div class="tile"><small>Mode</small><strong id="mode">-</strong></div>
      <div class="tile"><small>Boxes</small><strong id="boxes">0</strong></div>
      <div class="tile"><small>Target</small><strong id="target">0</strong></div>
      <div class="tile"><small>Complete</small><strong id="complete">0</strong></div>
      <div class="tile"><small>Refuge</small><strong id="refuge">0</strong></div>
      <div class="tile"><small>Moving</small><strong id="moving">-</strong></div>
    </div>
    <div class="row"><strong>Fault:</strong><span id="fault" style="color:#dc2626">-</span></div>
  </section>

  <section>
    <h2>Floor Hardware / Calibration Target</h2>
    <div class="row">
      <label>Calibration Floor</label>
      <select id="calFloor" onchange="onFloorChanged()"><option value="1">Floor 1</option><option value="2">Floor 2</option></select>
      <span class="small">Active supervisor floor: <strong id="activeFloorLabel">-</strong></span>
      <button class="gray" onclick="floorCmd({cmd:'status'})">FLOOR STATUS</button>
      <button class="red" onclick="floorCmd({cmd:'stop'})">FLOOR STOP</button>
      <button class="orange" onclick="floorCmd({cmd:'clear_fault'})">FLOOR CLEAR FAULT</button>
      <button class="purple" onclick="floorCmd({cmd:'zero'})">FLOOR ZERO</button>
    </div>
    <div class="floor-grid">
      <div class="tile"><small>Bridge</small><strong id="floorBridge">-</strong></div>
      <div class="tile"><small>Port</small><strong id="floorPort">-</strong></div>
      <div class="tile"><small>Rx Age</small><strong id="floorRxAge">-</strong></div>
      <div class="tile"><small>Moving</small><strong id="floorMoving">-</strong></div>
      <div class="tile"><small>CH0</small><strong id="floorTof0">-</strong></div>
      <div class="tile"><small>CH2</small><strong id="floorTof2">-</strong></div>
      <div class="tile"><small>CH4</small><strong id="floorTof4">-</strong></div>
      <div class="tile"><small>CH6</small><strong id="floorTof6">-</strong></div>
    </div>
    <div class="row small">
      <span>Selected floor encoder: <strong id="floorEnc">-</strong></span>
      <span>rpm: <strong id="floorRpm">-</strong></span>
      <span>last event: <strong id="floorLastEvent">-</strong></span>
    </div>
  </section>

  <section>
    <h2>Digital Twin Compare</h2>
    <div class="row">
      <label>Sim Floor</label><strong id="twinFloor">1</strong>
      <label>Target</label><input id="twinTarget" value="1">
      <button class="green" onclick="twinCmd({cmd:'predict', target:num('twinTarget')})">PREDICT FROM CURRENT DB</button>
      <button onclick="twinCmd({cmd:'plan', target:num('twinTarget')})">SIM PLAN</button>
      <button class="purple" onclick="twinCmd({cmd:'next_move', target:num('twinTarget')})">SIM NEXT MOVE</button>
      <button class="green" onclick="twinCmd({cmd:'sim_auto', target:num('twinTarget')})">SIM AUTO RUN</button>
      <button class="red" onclick="twinCmd({cmd:'sim_auto_stop'})">STOP SIM AUTO</button>
      <button class="gray" onclick="twinCmd({cmd:'clear'})">CLEAR TWIN</button>
      <label>Auto PNG</label><select id="autoPlanImages"><option value="0">OFF</option><option value="1">ON</option></select>
      <button onclick="applyRenderSettings()">SET RENDER</button>
      <span id="twinState" class="small">-</span>
    </div>
    <div class="status-grid">
      <div class="tile"><small>Sim Result</small><strong id="twinResult">-</strong></div>
      <div class="tile"><small>Steps</small><strong id="twinSteps">-</strong></div>
      <div class="tile"><small>Refuge</small><strong id="twinRefuge">-</strong></div>
      <div class="tile"><small>Matched</small><strong id="twinMatched">-</strong></div>
      <div class="tile"><small>Belt Diff</small><strong id="twinBeltDiff">-</strong></div>
      <div class="tile"><small>Max Pos Diff</small><strong id="twinMaxDiff">-</strong></div>
    </div>
    <div class="row"><strong>Sim Message:</strong><span id="twinMessage">-</span></div>
    <div class="row"><strong>Next Move:</strong><span id="twinNextMove">-</span></div>
    <div class="sim-image-wrap">
      <h3>MATLAB Simulation View</h3>
      <div class="plan-banner">
        <small id="currentPlanLabel">Current Plan</small>
        <strong id="currentPlanText">-</strong>
        <div id="currentPlanSub" class="sub">-</div>
      </div>
      <img id="simImage" class="sim-image" alt="MATLAB simulation view">
      <div id="simImageEmpty" class="small">시뮬 명령을 실행하면 MATLAB 2D 층별 화면이 여기에 표시됩니다.</div>
    </div>
    <div class="compare-layout">
      <div class="viz-panel">
        <h3 class="viz-title">Actual Current <span id="actualLiveBadge" class="live-badge">LIVE</span></h3>
        <div id="actualLiveNote" class="small live-note">-</div>
        <div id="actualViz"></div>
      </div>
      <div class="viz-panel">
        <h3>Sim Predicted</h3>
        <div id="simViz"></div>
      </div>
    </div>
    <h3>Sim Move Plan</h3>
    <div class="plan-table">
      <table>
        <thead><tr><th>#</th><th>Belt</th><th>Dir</th><th>mm</th><th>Message</th></tr></thead>
        <tbody id="planRows"></tbody>
      </table>
    </div>
    <h3>Position Compare</h3>
    <table>
      <thead><tr><th>ID</th><th>Actual</th><th>Predicted</th><th>Pos Diff</th><th>Belt</th></tr></thead>
      <tbody id="twinRows"></tbody>
    </table>
  </section>

  <section>
    <h2>Package DB Add</h2>
    <div class="row">
      <strong>우체국 박스 preset 추가</strong>
      <select id="boxType"><option>1</option><option>2</option><option>3</option><option>4</option></select>
      <input id="boxId" placeholder="비우면 자동 ID">
      <button onclick="addBox()">ADD BOX</button>
      <button onclick="quickBox(1)">BOX 1</button><button onclick="quickBox(2)">BOX 2</button>
      <button onclick="quickBox(3)">BOX 3</button><button onclick="quickBox(4)">BOX 4</button>
      <button class="purple" onclick="loadFixedTestDb()">1층 고정 DB 2313132211321133221</button>
      <button class="purple" onclick="loadFixedTestDbFloor2()">2층 고정 DB 2314323131243423</button>
      <button class="green" onclick="loadComplete()">B4 수동 상차 완료</button>
    </div>
    <div class="row">
      <strong>임의 크기 순차 추가</strong>
      <input id="seqId" placeholder="비우면 자동">
      <input id="seqLong" value="135"><input id="seqShort" value="90"><input id="seqHeight" value="75">
      <button onclick="addSeq()">ADD SEQ</button>
    </div>
    <div class="row">
      <strong>직접 배치</strong>
      <input id="posId" value="1">
      <select id="posBelt"><option>1</option><option>2</option><option>3</option><option>4</option></select>
      <input id="posPos" placeholder="Pos">
      <input id="posLong" value="135"><input id="posShort" value="90"><input id="posHeight" value="75">
      <button onclick="addPos()">ADD / ADDPOS</button>
    </div>
  </section>

  <section>
    <h2>Target / Auto</h2>
    <div class="row">
      <label>Target Package ID</label><input id="targetId" value="1">
      <button class="green" onclick="startSimTarget()">START TARGET</button>
      <button class="gray" onclick="cmd({cmd:'start', id:num('targetId')})">LEGACY START</button>
      <button class="red" onclick="stopAll()">STOP</button>
      <button class="orange" onclick="cmd({cmd:'refuged'})">REFUGED</button>
    </div>
  </section>

  <section>
    <h2>Manual Belt Move</h2>
    <div class="row">
      <label>Belt</label><select id="moveBelt"><option>1</option><option>2</option><option>3</option><option>4</option></select>
      <label>Dir</label><select id="moveDir"><option value="1">+1 Forward</option><option value="-1">-1 Reverse</option></select>
      <label>mm</label><input id="moveMm" value="100">
      <button onclick="floorCmd({cmd:'move', belt:num('moveBelt'), dir:num('moveDir'), mm:num('moveMm')})">MOVE</button>
    </div>
  </section>

  <section>
    <h2>Settings</h2>
    <div class="row">
      <label>RPM</label><input id="rpm" value="30">
      <button onclick="floorCmd({cmd:'set', key:'rpm', value:num('rpm')})">SET RPM</button>
      <label>TOF</label><select id="tof"><option value="1">ON</option><option value="0">OFF</option></select>
      <button onclick="floorCmd({cmd:'set', key:'tof', value:num('tof')})">SET TOF</button>
      <label>ToF Deadband</label><input id="tofDeadband" value="1.0">
      <button onclick="floorCmd({cmd:'set', key:'tof_deadband', value:num('tofDeadband')})">SET DEADBAND</button>
      <label>Refuge</label><select id="refugeMode"><option value="manual">MANUAL</option><option value="auto">AUTO</option></select>
      <button onclick="cmd({cmd:'set', key:'refuge', value:document.getElementById('refugeMode').value})">SET REFUGE</button>
    </div>
    <div class="row">
      <label>KP</label><input id="pidKp" value="0.8">
      <label>KI</label><input id="pidKi" value="0.30">
      <label>KD</label><input id="pidKd" value="0.0">
      <button onclick="applyPid()">SET PID</button>
      <label>Slowdown mm</label><input id="slowdownMm" value="60">
      <label>Min RPM</label><input id="minMoveRpm" value="25">
      <label>PWM Step</label><input id="pwmStep" value="25">
      <label>Compact Rev RPM</label><input id="compactReverseRpm" value="200">
      <button onclick="applyMotionTuning()">SET MOTION TUNING</button>
    </div>
    <div class="row">
      <label>MMCOUNT Belt</label><select id="mmBelt"><option>1</option><option>2</option><option>3</option><option>4</option></select>
      <label>Dir</label><select id="mmDir"><option value="1">+1</option><option value="-1">-1</option></select>
      <input id="mmCount" value="0.127500">
      <button onclick="floorCmd({cmd:'set', key:'mmcount', belt:num('mmBelt'), dir:num('mmDir'), value:num('mmCount')})">SET MMCOUNT</button>
      <label>Move Scale</label><input id="moveScale" value="1.000000">
      <button onclick="floorCmd({cmd:'set', key:'move_scale', belt:num('mmBelt'), dir:num('mmDir'), value:num('moveScale')})">SET SCALE</button>
      <label>Offset mm</label><input id="moveOffset" value="0.000">
      <button onclick="floorCmd({cmd:'set', key:'move_offset', belt:num('mmBelt'), dir:num('mmDir'), value:num('moveOffset')})">SET OFFSET</button>
    </div>
    <h3>Encoder Calibration</h3>
    <table class="cal-table">
      <thead><tr><th>Belt</th><th>+ mm/count</th><th>- mm/count</th><th>+ scale</th><th>- scale</th><th>+ offset</th><th>- offset</th></tr></thead>
      <tbody id="encoderCalRows"></tbody>
    </table>
    <div class="row">
      <button class="green" onclick="applyEncoderTable()">APPLY ENCODER TABLE</button>
      <span class="small">mm/count는 엔코더 환산값, scale/offset은 명령 거리 보정값입니다.</span>
    </div>
    <h3>Distance Range Scale</h3>
    <table class="cal-table">
      <thead><tr><th>Belt</th><th>Dir</th><th>&le;20mm</th><th>&le;100mm</th><th>&le;250mm</th><th>&gt;250mm</th></tr></thead>
      <tbody id="distanceScaleRows"></tbody>
    </table>
    <div class="row">
      <button class="green" onclick="applyDistanceScaleTable()">APPLY DISTANCE SCALE</button>
      <span class="small">짧은 move 오버슈트를 줄이는 거리구간별 추가 scale입니다.</span>
    </div>
    <h3>Belt Geometry</h3>
    <table class="cal-table">
      <thead><tr><th>Belt</th><th>Length mm</th></tr></thead>
      <tbody id="beltLengthRows"></tbody>
    </table>
    <div class="row">
      <button class="green" onclick="applyBeltLengths()">APPLY BELT LENGTHS</button>
      <span class="small">실제 DB와 MATLAB 시뮬이 같이 쓰는 벨트 길이입니다.</span>
    </div>
    <h3>ToF Decision Thresholds</h3>
    <div class="row">
      <label>Correction</label><select id="tofCorrEnabled"><option value="1">ON</option><option value="0">OFF</option></select>
      <label>Step mm</label><input id="tofCorrStep" value="2">
      <label>Max mm</label><input id="tofCorrMax" value="15">
      <label>Initial underrun mm</label><input id="tofUnderrun" value="2">
      <label>Empty extra mm</label><input id="tofEmptyExtra" value="0">
      <button onclick="applyTofCorrection()">SET TOF CORRECTION</button>
    </div>
    <table class="cal-table">
      <thead><tr><th>CH</th><th>Name</th><th>Box offset mm</th><th>Empty >= mm</th></tr></thead>
      <tbody id="tofThresholdRows"></tbody>
    </table>
    <div class="row">
      <button class="green" onclick="applyTofThresholds()">APPLY TOF THRESHOLDS</button>
      <span class="small">CH0/2/4/6만 사용합니다. Box 기준은 250 - 박스 폭방향 크기 + offset 입니다.</span>
    </div>
  </section>

  <section>
    <h2>Package DB</h2>
    <table>
      <thead><tr><th>ID</th><th>Seq</th><th>Belt</th><th>Pos</th><th>Long</th><th>Short</th><th>Axis</th><th>Action</th></tr></thead>
      <tbody id="db"></tbody>
    </table>
  </section>

  <section>
    <h2>ToF Sensors</h2>
    <table><thead><tr id="tofHead"></tr></thead><tbody><tr id="tofVals"></tr></tbody></table>
  </section>

  <section>
    <h2>Event Log</h2>
    <div class="log" id="log"></div>
  </section>
</main>
<script>
const tofNames = ['CH0 B1 Gap','CH1 B1 Transfer','CH2 B2 Gap','CH3 B2 Transfer','CH4 B3 Gap','CH5 B3 Transfer','CH6 B4 Gap','CH7 B4 Transfer'];
const tofDisplayChannels = [0, 2, 4, 6];
const tofDecisionChannels = tofDisplayChannels;
const encoderMmDefaults = [0.1275, 0.124522, 0.126006, 0.125565];
const encoderMmNegDefaults = [0.1255, 0.124522, 0.126006, 0.125565];
const beltLenDefaults = [498, 1080, 498, 1080];
const distanceBinDefaults = [20, 100, 250, 100000];
const distanceScaleDefaults = [0.45, 0.93, 0.94, 0.92];
const distanceScaleByBeltDefaults = {
  1: [[0.45, 0.93, 0.94, 0.92], [0.45, 0.93, 0.94, 0.92]],
  2: [[0.45, 0.9762, 0.94, 0.9452], [0.45, 0.9762, 0.94, 0.9452]],
  3: [[0.45, 0.93, 0.94, 0.92], [0.45, 0.93, 0.94, 0.92]],
  4: [[0.45, 0.93, 0.94, 0.9964], [0.45, 0.93, 0.94, 0.9964]],
};
const tofEmptyDefaults = {0:220, 2:234, 4:233, 6:220};
const tofBoxOffsetDefaults = {0:0, 2:0, 4:0, 6:0};
document.getElementById('tofHead').innerHTML = tofDisplayChannels.map(ch => `<th>${tofNames[ch]}</th>`).join('');
document.getElementById('encoderCalRows').innerHTML = [1,2,3,4].map(b => `
  <tr>
    <td>B${b}</td>
    <td><input id="mm_${b}_p" value="${encoderMmDefaults[b - 1].toFixed(6)}"></td>
    <td><input id="mm_${b}_n" value="${encoderMmNegDefaults[b - 1].toFixed(6)}"></td>
    <td><input id="scale_${b}_p" value="1.000000"></td>
    <td><input id="scale_${b}_n" value="1.000000"></td>
    <td><input id="offset_${b}_p" value="0.000"></td>
    <td><input id="offset_${b}_n" value="0.000"></td>
  </tr>`).join('');
document.getElementById('beltLengthRows').innerHTML = [1,2,3,4].map(b => `
  <tr>
    <td>B${b}</td>
    <td><input id="belt_len_${b}" value="${beltLenDefaults[b - 1].toFixed(1)}"></td>
  </tr>`).join('');
document.getElementById('distanceScaleRows').innerHTML = [1,2,3,4].flatMap(b => [1,-1].map(dir => {
  const suffix = dir > 0 ? 'p' : 'n';
  return `<tr>
    <td>B${b}</td>
    <td>${dir > 0 ? '+1' : '-1'}</td>
    ${(distanceScaleByBeltDefaults[b][dir > 0 ? 0 : 1] || distanceScaleDefaults).map((value, index) => `<td><input id="dist_${b}_${suffix}_${index + 1}" value="${value.toFixed(4)}"></td>`).join('')}
  </tr>`;
})).join('');
document.getElementById('tofThresholdRows').innerHTML = tofDecisionChannels.map(ch => `
  <tr>
    <td>CH${ch}</td>
    <td>${tofNames[ch].replace('CH' + ch + ' ', '')}</td>
    <td><input id="tof_box_offset_${ch}" value="${tofBoxOffsetDefaults[ch]}"></td>
    <td><input id="tof_empty_${ch}" value="${tofEmptyDefaults[ch]}"></td>
  </tr>`).join('');
document.querySelectorAll('#encoderCalRows input,#beltLengthRows input,#distanceScaleRows input,#tofThresholdRows input,#tofCorrEnabled,#tofCorrStep,#tofCorrMax,#tofUnderrun,#tofEmptyExtra,#tofDeadband,#rpm,#slowdownMm,#minMoveRpm,#pwmStep,#autoPlanImages').forEach(el => {
  el.addEventListener('input', () => { el.dataset.touched = '1'; });
});
let lastTwinImageVersion = -1;
let currentBeltLen = beltLenDefaults.slice();
let latestStatus = {};
let latestFloors = {};
let latestActualDb = [];
let latestTargetId = 0;
function num(id){ return Number(document.getElementById(id).value); }
function val(id){ return document.getElementById(id).value.trim(); }
function selectedFloor(){ return Math.max(1, Math.min(2, Number(document.getElementById('calFloor').value || 1))); }
async function cmd(payload){
  const wrapped = payload.floor ? payload : {...payload, floor:selectedFloor()};
  await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(wrapped)});
  document.getElementById('lastSend').textContent = 'Last send: ' + JSON.stringify(wrapped);
}
async function twinCmd(payload){
  const wrapped = payload.floor ? payload : {...payload, floor:selectedFloor()};
  await fetch('/api/twin_cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(wrapped)});
  document.getElementById('twinState').textContent = 'Twin send: ' + JSON.stringify(wrapped);
}
async function floorCmd(payload){
  const wrapped = {...payload, floor:selectedFloor()};
  await fetch('/api/floor_cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(wrapped)});
  document.getElementById('lastSend').textContent = `Last floor send: F${wrapped.floor} ` + JSON.stringify(payload);
}
function onFloorChanged(){
  document.getElementById('calFloor').dataset.touched = '1';
  updateFloorPanel();
}
async function startSimTarget(){
  const floor = selectedFloor();
  document.getElementById('twinTarget').value = document.getElementById('targetId').value;
  await twinCmd({cmd:'sim_auto', target:num('targetId'), floor});
}
async function stopAll(){
  await twinCmd({cmd:'sim_auto_stop'});
  await cmd({cmd:'stop'});
}
function sendRaw(){ const text = val('raw'); if(text) cmd({raw:text}); }
function addBox(){ const id=val('boxId'); const p={cmd:'box', type:num('boxType')}; if(id) p.id=Number(id); cmd(p); }
function quickBox(t){ cmd({cmd:'box', type:t}); }
function loadFixedTestDb(){ cmd({cmd:'test_db', floor:1}); }
function loadFixedTestDbFloor2(){ cmd({cmd:'test_db_floor2', floor:2}); }
function addSeq(){ const p={cmd:'seq', long:num('seqLong'), short:num('seqShort'), height:num('seqHeight')}; const id=val('seqId'); if(id) p.id=Number(id); cmd(p); }
function addPos(){ cmd({cmd:'addpos', id:num('posId'), belt:num('posBelt'), pos:num('posPos'), long:num('posLong'), short:num('posShort'), height:num('posHeight')}); }
function loadComplete(){ twinCmd({cmd:'manual_load', type:num('boxType')}); }
async function applyEncoderTable(){
  for (const b of [1,2,3,4]) {
    await floorCmd({cmd:'set', key:'mmcount', belt:b, dir:1, value:num(`mm_${b}_p`)});
    await floorCmd({cmd:'set', key:'mmcount', belt:b, dir:-1, value:num(`mm_${b}_n`)});
    await floorCmd({cmd:'set', key:'distcal', belt:b, dir:1, scale:num(`scale_${b}_p`), offset:num(`offset_${b}_p`)});
    await floorCmd({cmd:'set', key:'distcal', belt:b, dir:-1, scale:num(`scale_${b}_n`), offset:num(`offset_${b}_n`)});
  }
}
async function applyBeltLengths(){
  const lengths = [1,2,3,4].map(b => num(`belt_len_${b}`));
  for (const b of [1,2,3,4]) {
    await cmd({cmd:'set', key:'belt_length', belt:b, value:lengths[b - 1]});
  }
  await twinCmd({cmd:'set_belt_lengths', lengths});
}
async function applyDistanceScaleTable(){
  for (const b of [1,2,3,4]) {
    for (const dir of [1,-1]) {
      const suffix = dir > 0 ? 'p' : 'n';
      for (const bin of [1,2,3,4]) {
        await floorCmd({cmd:'set', key:'distbin', belt:b, dir, bin, scale:num(`dist_${b}_${suffix}_${bin}`), offset:0});
      }
    }
  }
}
async function applyTofThresholds(){
  for (const ch of tofDecisionChannels) {
    await twinCmd({cmd:'set_tof_threshold', channel:ch, box_offset:num(`tof_box_offset_${ch}`), empty:num(`tof_empty_${ch}`)});
  }
}
async function applyTofCorrection(){
  await twinCmd({cmd:'set_tof_correction', enabled:num('tofCorrEnabled'), step_mm:num('tofCorrStep'), max_mm:num('tofCorrMax'), underrun_mm:num('tofUnderrun'), empty_extra_mm:num('tofEmptyExtra')});
}
async function applyPid(){
  await floorCmd({cmd:'set', key:'kp', value:num('pidKp')});
  await floorCmd({cmd:'set', key:'ki', value:num('pidKi')});
  await floorCmd({cmd:'set', key:'kd', value:num('pidKd')});
}
async function applyMotionTuning(){
  await floorCmd({cmd:'set', key:'slowdown', value:num('slowdownMm')});
  await floorCmd({cmd:'set', key:'minrpm', value:num('minMoveRpm')});
  await floorCmd({cmd:'set', key:'pwmstep', value:num('pwmStep')});
  await floorCmd({cmd:'set', key:'compact_reverse_rpm', value:num('compactReverseRpm')});
}
async function applyRenderSettings(){
  await twinCmd({cmd:'set_render', auto_plan_images:num('autoPlanImages')});
}
function axisFor(b){ return (b.belt === 1 || b.belt === 3) ? b.long_side : b.short_side; }
async function refresh(){
  const r = await fetch('/api/state'); const s = await r.json();
  latestFloors = s.floors || {};
  const selected = selectedFloor();
  const floorInfo = latestFloors[String(selected)] || latestFloors[selected] || {};
  const st = floorInfo.status || s.status || {};
  const db = floorInfo.db || s.db || [];
  const logs = floorInfo.logs || s.logs || [];
  const twin = floorInfo.twin || s.twin || {};
  latestStatus = st;
  latestActualDb = db;
  latestTargetId = Number(document.getElementById('twinTarget').value || 0);
  mode.textContent = st.mode || '-'; boxes.textContent = st.boxes ?? 0; target.textContent = st.target ?? 0;
  complete.textContent = st.complete ?? 0; refuge.textContent = st.refuge ?? 0;
  moving.textContent = st.hardware_moving ? `B${st.pending_move?.belt + 1 || '-'}` : '-';
  fault.textContent = st.fault || '-';
  if (!document.getElementById('twinTarget').dataset.touched && st.target) document.getElementById('twinTarget').value = st.target;
  activeFloorLabel.textContent = st.floor ? `F${st.floor}` : '-';
  updateFloorPanel();
  updateEncoderInputs(st.encoder_calibration || {});
  updateDistanceScaleInputs(st.encoder_calibration || {});
  updateBeltLengthInputs(st.belt_len_mm || ((twin.geometry || {}).belt_len_mm) || beltLenDefaults);
  if (st.tof_deadband_mm !== undefined) setUntouched('tofDeadband', st.tof_deadband_mm, 1);
  updateMotionTuningInputs(st.motion_tuning || {});
  renderTwin(twin, db);
  updateTofInputs((twin || {}).tof_thresholds || {});
  const tof = st.tof || [];
  document.getElementById('tofVals').innerHTML = tofDisplayChannels.map(ch => `<td>${tof[ch] ?? '-'}</td>`).join('');
  document.getElementById('db').innerHTML = db.map(b => `<tr><td>P${b.id}</td><td>${b.seq}</td><td>B${b.belt+1}</td><td>${Number(b.pos).toFixed(1)}</td><td>${Number(b.long_side).toFixed(1)}</td><td>${Number(b.short_side).toFixed(1)}</td><td>${Number(axisFor(b)).toFixed(1)}</td><td><button class="green" onclick="cmd({cmd:'start',id:${b.id}})">START</button></td></tr>`).join('');
  log.textContent = logs.map(x => `${x.time} ${x.event}${x.detail ? ' ' + x.detail : ''}`).join('\\n');
}
function updateFloorPanel(){
  const floor = selectedFloor();
  const info = latestFloors[String(floor)] || latestFloors[floor] || {};
  const bridge = info.bridge_state || {};
  const telemetry = info.telemetry || {};
  const event = info.last_event || {};
  const tof = telemetry.tof || [];
  const enc = telemetry.enc || [];
  const rpm = telemetry.rpm || [];
  floorBridge.textContent = bridge.connected ? 'CONNECTED' : 'OFFLINE';
  floorPort.textContent = bridge.port || '-';
  floorRxAge.textContent = bridge.last_rx_age_sec !== undefined ? `${Number(bridge.last_rx_age_sec).toFixed(1)}s` : '-';
  floorMoving.textContent = telemetry.moving ? `B${telemetry.active_belt || '-'}` : '-';
  floorTof0.textContent = tof[0] ?? '-';
  floorTof2.textContent = tof[2] ?? '-';
  floorTof4.textContent = tof[4] ?? '-';
  floorTof6.textContent = tof[6] ?? '-';
  floorEnc.textContent = Array.isArray(enc) && enc.length ? enc.join(', ') : '-';
  floorRpm.textContent = Array.isArray(rpm) && rpm.length ? rpm.map(v => Number(v).toFixed(1)).join(', ') : '-';
  floorLastEvent.textContent = event.event || '-';
}
document.getElementById('twinTarget').addEventListener('input', () => { document.getElementById('twinTarget').dataset.touched = '1'; });
function setUntouched(id, value, digits){
  const el = document.getElementById(id);
  if (!el || el.dataset.touched) return;
  const numValue = Number(value);
  el.value = Number.isFinite(numValue) ? numValue.toFixed(digits) : value;
}
function updateEncoderInputs(cal){
  const mm = cal.mmcount || [];
  const scale = cal.move_scale || [];
  const offset = cal.move_offset || [];
  for (let b = 1; b <= 4; b++) {
    if (mm[b-1]) {
      setUntouched(`mm_${b}_p`, mm[b-1][0], 6);
      setUntouched(`mm_${b}_n`, mm[b-1][1], 6);
    }
    if (scale[b-1]) {
      setUntouched(`scale_${b}_p`, scale[b-1][0], 6);
      setUntouched(`scale_${b}_n`, scale[b-1][1], 6);
    }
    if (offset[b-1]) {
      setUntouched(`offset_${b}_p`, offset[b-1][0], 3);
      setUntouched(`offset_${b}_n`, offset[b-1][1], 3);
    }
  }
}
function updateDistanceScaleInputs(cal){
  const scale = cal.distance_scale || [];
  for (let b = 1; b <= 4; b++) {
    for (const [dirIndex, suffix] of [[0, 'p'], [1, 'n']]) {
      const row = scale[b-1] && scale[b-1][dirIndex] ? scale[b-1][dirIndex] : null;
      if (!row) continue;
      for (let bin = 1; bin <= 4; bin++) {
        if (row[bin-1] !== undefined) setUntouched(`dist_${b}_${suffix}_${bin}`, row[bin-1], 4);
      }
    }
  }
}
function updateBeltLengthInputs(lengths){
  if (!Array.isArray(lengths) || lengths.length < 4) return;
  currentBeltLen = lengths.slice(0, 4).map(v => Number(v) || 0);
  for (let b = 1; b <= 4; b++) setUntouched(`belt_len_${b}`, currentBeltLen[b - 1], 1);
}
function updateTofInputs(th){
  const boxOffset = th.box_offset || [];
  const empty = th.empty || [];
  const channels = th.decision_channels || tofDecisionChannels;
  for (const ch of channels) {
    if (boxOffset[ch] !== undefined) setUntouched(`tof_box_offset_${ch}`, boxOffset[ch], 1);
    if (empty[ch] !== undefined) setUntouched(`tof_empty_${ch}`, empty[ch], 1);
  }
  if (th.correction_enabled !== undefined && !tofCorrEnabled.dataset.touched) tofCorrEnabled.value = th.correction_enabled ? '1' : '0';
  if (th.step_mm !== undefined) setUntouched('tofCorrStep', th.step_mm, 1);
  if (th.max_mm !== undefined) setUntouched('tofCorrMax', th.max_mm, 1);
  if (th.underrun_mm !== undefined) setUntouched('tofUnderrun', th.underrun_mm, 1);
  if (th.empty_extra_mm !== undefined) setUntouched('tofEmptyExtra', th.empty_extra_mm, 1);
}
function updateMotionTuningInputs(mt){
  if (mt.default_rpm !== undefined) setUntouched('rpm', mt.default_rpm, 1);
  if (mt.kp !== undefined) setUntouched('pidKp', mt.kp, 2);
  if (mt.ki !== undefined) setUntouched('pidKi', mt.ki, 2);
  if (mt.kd !== undefined) setUntouched('pidKd', mt.kd, 1);
  if (mt.slowdown_mm !== undefined) setUntouched('slowdownMm', mt.slowdown_mm, 1);
  if (mt.min_move_rpm !== undefined) setUntouched('minMoveRpm', mt.min_move_rpm, 1);
  if (mt.pwm_step !== undefined) setUntouched('pwmStep', mt.pwm_step, 0);
  if (mt.compact_reverse_rpm !== undefined) setUntouched('compactReverseRpm', mt.compact_reverse_rpm, 1);
}
function planPackageId(move, fallbackText){
  if (move && move.handoff_id) return `P${move.handoff_id}`;
  const text = `${move?.message || ''} ${fallbackText || ''}`;
  const m = text.match(/P(\d+)/i);
  return m ? `P${m[1]}` : '';
}
function describeMove(move, fallbackMessage){
  if (!move) return '-';
  const belt = Number(move.belt || 0);
  const mm = Number(move.mm || 0);
  const dir = Number(move.dir || 0);
  const dirText = dir > 0 ? '정방향' : dir < 0 ? '역방향' : '정지';
  const msg = move.message || fallbackMessage || '';
  const pkg = planPackageId(move, msg);
  const parts = [];
  if (belt > 0 && mm > 0) parts.push(`B${belt} ${mm.toFixed(1)}mm ${dirText} 이동`);
  if (move.handoff_receiver && pkg) parts.push(`${pkg} Handoff -> B${move.handoff_receiver}`);
  else if (/HANDOFF/i.test(msg) && pkg) parts.push(`${pkg} Handoff`);
  if (/REFUGE/i.test(msg)) parts.push(pkg ? `${pkg} 피신 처리` : '피신 처리');
  if (/COMPACT/i.test(msg)) parts.push('밀착');
  return parts.length ? parts.join(' / ') : (msg || '-');
}
function updateCurrentPlanBanner(twin, plan, auto, moves){
  let label = 'Current Plan';
  let text = '-';
  let sub = '-';
  if (auto.executing) {
    label = 'Executing';
    text = describeMove(auto.executing, auto.message);
    sub = auto.message || 'actual command sent';
  } else if (auto.active && /WAIT_MANUAL_REFUGE/i.test(auto.message || '')) {
    label = 'Waiting';
    text = auto.message || 'WAIT_MANUAL_REFUGE';
    sub = '박스를 손으로 빼고 REFUGED를 누르면 현재 DB로 재계획합니다.';
  } else if (moves.length) {
    label = 'Next Plan';
    text = describeMove(moves[0], moves[0].message);
    sub = moves[0].message || 'MATLAB next move';
  } else if (plan.executed) {
    label = 'Last Sent';
    text = describeMove(plan.executed, plan.executed.message);
    sub = 'last manual sim command';
  } else if (auto.message) {
    label = auto.active ? 'Auto State' : 'State';
    text = auto.message;
    sub = twin.last_error || '-';
  }
  currentPlanLabel.textContent = label;
  currentPlanText.textContent = text;
  currentPlanSub.textContent = sub;
}
function renderTwin(twin, actualDb){
  const pred = twin.prediction || {};
  const plan = twin.plan || {};
  const cmp = twin.comparison || {};
  const auto = twin.auto || {};
  const moves = plan.moves || [];
  const ms = twin.matlab_server || {};
  if (ms.render_auto_plan_images !== undefined && !autoPlanImages.dataset.touched) {
    autoPlanImages.value = ms.render_auto_plan_images ? '1' : '0';
  }
  twinFloor.textContent = twin.floor ?? 3;
  twinState.textContent = auto.active ? `AUTO ${auto.step || 0}: ${auto.message || ''}` : (twin.running ? 'MATLAB sim running...' : (twin.last_error ? 'ERROR' : 'READY'));
  twinResult.textContent = twin.running ? 'RUNNING' : (pred.success === true ? 'OK' : pred.success === false ? 'FAIL' : '-');
  twinSteps.textContent = pred.steps ?? '-';
  twinRefuge.textContent = pred.refuge ?? '-';
  twinMatched.textContent = cmp.matched ?? '-';
  twinBeltDiff.textContent = cmp.belt_mismatch ?? '-';
  twinMaxDiff.textContent = cmp.max_pos_error !== undefined ? `${cmp.max_pos_error} mm` : '-';
  twinMessage.textContent = twin.last_error || pred.message || '-';
  twinNextMove.textContent = auto.executing ? `auto sent B${auto.executing.belt} ${auto.executing.dir > 0 ? '+' : '-'} ${auto.executing.mm} mm` : (moves.length ? `B${moves[0].belt} ${moves[0].dir > 0 ? '+' : '-'} ${Number(moves[0].mm).toFixed(1)} mm (${moves[0].message || 'sim'})` : (plan.executed ? `sent B${plan.executed.belt} ${plan.executed.dir > 0 ? '+' : '-'} ${plan.executed.mm} mm` : '-'));
  updateCurrentPlanBanner(twin, plan, auto, moves);
  const imageVersion = Number(twin.image_version || 0);
  if (imageVersion > 0 && imageVersion !== lastTwinImageVersion) {
    lastTwinImageVersion = imageVersion;
    simImage.src = `/api/twin_image?floor=${selectedFloor()}&v=${imageVersion}&t=${Date.now()}`;
    simImage.style.display = 'block';
    simImageEmpty.style.display = 'none';
  } else if (imageVersion <= 0 && !simImage.getAttribute('src')) {
    simImage.style.display = 'none';
    simImageEmpty.style.display = 'block';
  }
  const simDb = plan.sim_db || pred.predicted_db || [];
  renderLiveActualViz();
  simViz.innerHTML = renderBeltViz(simDb || [], Number(document.getElementById('twinTarget').value || 0));
  planRows.innerHTML = moves.slice(0, 30).map((m, i) => `<tr><td>${i + 1}</td><td>B${m.belt}</td><td>${m.dir > 0 ? '+1' : '-1'}</td><td>${Number(m.mm).toFixed(1)}</td><td>${m.message || ''}</td></tr>`).join('');
  const rows = cmp.rows || [];
  twinRows.innerHTML = rows.slice(0, 30).map(r => `<tr><td>P${r.id}</td><td>B${r.actual_belt} ${Number(r.actual_pos).toFixed(1)}</td><td>B${r.pred_belt} ${Number(r.pred_pos).toFixed(1)}</td><td>${Number(r.pos_error).toFixed(1)} mm</td><td>${r.belt_match ? 'OK' : 'DIFF'}</td></tr>`).join('');
}
function colorFor(id){
  const colors = ['#2563eb','#16a34a','#dc2626','#7c3aed','#0891b2','#ea580c','#4f46e5','#0f766e'];
  return colors[Math.abs(Number(id) || 0) % colors.length];
}
function moveSpeedMmPerSec(status, pending){
  const rpm = Number((status.motion_tuning || {}).default_rpm || 10);
  const gt2PulleyMmPerRev = 40.0;
  const speed = rpm * gt2PulleyMmPerRev / 60.0;
  return Math.max(4, Math.min(80, speed));
}
function liveMoveProgress(status){
  const pending = status.pending_move || null;
  if (!pending || !status.hardware_moving || !pending.started) return {active:false, progress:0, target:0, belt:-1, dir:0};
  const target = Math.max(0, Number(pending.target_mm ?? pending.mm ?? 0));
  const startedAt = Number(pending.started_at || pending.issued_at || 0);
  const elapsed = Math.max(0, Date.now() / 1000 - startedAt);
  const progress = Math.max(0, Math.min(target, elapsed * moveSpeedMmPerSec(status, pending)));
  return {
    active: target > 0,
    progress,
    target,
    belt: Number(pending.belt),
    dir: Number(pending.dir || 0),
    reason: pending.reason || '',
  };
}
function buildLiveDb(db, status){
  const live = liveMoveProgress(status);
  if (!live.active || live.belt < 0 || !live.dir) return (db || []);
  return (db || []).map(box => {
    const copy = {...box};
    if (Number(copy.belt) === live.belt) copy.pos = Number(copy.pos || 0) + live.dir * live.progress;
    return copy;
  });
}
function renderLiveActualViz(){
  const live = liveMoveProgress(latestStatus || {});
  const db = buildLiveDb(latestActualDb || [], latestStatus || {});
  actualViz.innerHTML = renderBeltViz(db, latestTargetId, live);
  if (live.active) {
    actualLiveBadge.classList.add('on');
    const sign = live.dir > 0 ? '+' : '-';
    actualLiveNote.textContent = `B${live.belt + 1} ${sign}${live.progress.toFixed(1)} / ${live.target.toFixed(1)} mm ${live.reason ? '(' + live.reason + ')' : ''}`;
  } else {
    actualLiveBadge.classList.remove('on');
    actualLiveNote.textContent = '-';
  }
}
function liveLoop(){
  renderLiveActualViz();
  requestAnimationFrame(liveLoop);
}
function renderBeltViz(db, targetId, live){
  const rows = [];
  for(let belt=0; belt<4; belt++){
    const boxes = (db || []).filter(b => Number(b.belt) === belt).map(b => {
      const axis = axisFor(b);
      const len = currentBeltLen[belt] || beltLenDefaults[belt];
      const tail = Number(b.pos) - axis / 2;
      const front = Number(b.pos) + axis / 2;
      const left = Math.max(0, Math.min(100, tail / len * 100));
      const right = Math.max(0, Math.min(100, front / len * 100));
      const width = Math.max(2.5, right - left);
      const title = `P${b.id} B${belt + 1} ${Number(b.pos).toFixed(1)}mm`;
      const cls = Number(b.id) === Number(targetId) ? 'pkg target' : 'pkg';
      return `<div class="${cls}" title="${title}" style="left:${left}%;width:${width}%;background:${colorFor(b.id)}">P${b.id}</div>`;
    }).join('');
    const gapPct = Math.min(100, 250 / (currentBeltLen[belt] || beltLenDefaults[belt]) * 100);
    const liveClass = live && live.active && Number(live.belt) === belt ? ' live-moving' : '';
    rows.push(`<div class="beltviz"><div class="beltname">B${belt + 1}</div><div class="beltbar${liveClass}" style="--gap-pct:${gapPct}%">${boxes}</div></div>`);
  }
  return rows.join('');
}
requestAnimationFrame(liveLoop);
setInterval(refresh, 500); refresh();
</script>
</body>
</html>"""


class WebControl(Node):
    def __init__(self):
        super().__init__("refuge_web_control")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter("twin_render_dir", "/tmp/refuge_twin_render")
        self.declare_parameter("twin_render_dir_floor1", "/tmp/refuge_twin_render_f1")
        self.declare_parameter("twin_render_dir_floor2", "/tmp/refuge_twin_render_f2")
        self.status = {}
        self.db = []
        self.twin = {}
        self.logs = deque(maxlen=80)
        self.state_lock = threading.Lock()
        self.floor_states = {
            floor: {
                "telemetry": {},
                "bridge_state": {},
                "last_event": {},
                "status": {},
                "db": [],
                "twin": {},
                "logs": [],
                "telemetry_at": 0.0,
                "bridge_at": 0.0,
                "event_at": 0.0,
                "status_at": 0.0,
                "db_at": 0.0,
                "twin_at": 0.0,
            }
            for floor in (1, 2)
        }

        self.cmd_pub = self.create_publisher(String, "/refuge/control_cmd", 10)
        self.twin_cmd_pub = self.create_publisher(String, "/refuge/twin_cmd", 10)
        self.floor_control_pubs = {
            floor: self.create_publisher(String, f"/refuge/floor{floor}/control_cmd", 10)
            for floor in (1, 2)
        }
        self.floor_twin_cmd_pubs = {
            floor: self.create_publisher(String, f"/refuge/floor{floor}/twin_cmd", 10)
            for floor in (1, 2)
        }
        self.floor_cmd_pubs = {
            floor: self.create_publisher(String, f"/refuge/floor{floor}/arduino_cmd", 10)
            for floor in (1, 2)
        }
        self.create_subscription(String, "/refuge/status", self.status_callback, 10)
        self.create_subscription(String, "/refuge/db", self.db_callback, 10)
        self.create_subscription(String, "/refuge/twin_state", self.twin_callback, 10)
        self.create_subscription(String, "/refuge/log", self.log_callback, 50)
        for floor in (1, 2):
            self.create_subscription(String, f"/refuge/floor{floor}/telemetry", self.make_floor_callback(floor, "telemetry"), 10)
            self.create_subscription(String, f"/refuge/floor{floor}/bridge_state", self.make_floor_callback(floor, "bridge_state"), 10)
            self.create_subscription(String, f"/refuge/floor{floor}/events", self.make_floor_callback(floor, "last_event"), 50)
            self.create_subscription(String, f"/refuge/floor{floor}/status", self.make_floor_callback(floor, "status"), 10)
            self.create_subscription(String, f"/refuge/floor{floor}/db", self.make_floor_callback(floor, "db"), 10)
            self.create_subscription(String, f"/refuge/floor{floor}/twin_state", self.make_floor_callback(floor, "twin"), 10)
            self.create_subscription(String, f"/refuge/floor{floor}/log", self.make_floor_callback(floor, "logs"), 50)

        self.app = Flask(__name__)
        self.configure_routes()

    @staticmethod
    def clamp_floor(value):
        try:
            floor = int(value)
        except (TypeError, ValueError):
            floor = 1
        return max(1, min(2, floor))

    def make_floor_callback(self, floor: int, key: str):
        def callback(msg: String):
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                payload = {"event": "raw", "line": msg.data}
            now = time.time()
            with self.state_lock:
                state = self.floor_states.setdefault(floor, {})
                if key == "logs":
                    logs = state.setdefault("logs", [])
                    if not isinstance(logs, list):
                        logs = []
                    logs.append(self.format_log_item(payload))
                    state["logs"] = logs[-80:]
                else:
                    state[key] = payload
                if key == "telemetry":
                    state["telemetry_at"] = now
                elif key == "bridge_state":
                    state["bridge_at"] = now
                elif key == "last_event":
                    state["event_at"] = now
                elif key == "status":
                    state["status_at"] = now
                elif key == "db":
                    state["db_at"] = now
                elif key == "twin":
                    state["twin_at"] = now
                elif key == "logs":
                    state["logs_at"] = now
        return callback

    def publish_control(self, payload):
        floor = self.clamp_floor(payload.get("floor", 0)) if "floor" in payload else 0
        data = json.dumps(payload, separators=(",", ":"))
        if floor in self.floor_control_pubs:
            self.floor_control_pubs[floor].publish(String(data=data))
            return True
        self.get_logger().warning(f"rejecting control command without explicit floor: {payload}")
        return False

    def publish_twin_control(self, payload):
        floor = self.clamp_floor(payload.get("floor", 0)) if "floor" in payload else 0
        data = json.dumps(payload, separators=(",", ":"))
        if floor in self.floor_twin_cmd_pubs:
            self.floor_twin_cmd_pubs[floor].publish(String(data=data))
            return True
        self.get_logger().warning(f"rejecting twin command without explicit floor: {payload}")
        return False

    def publish_floor_serial(self, floor: int, command: str):
        pub = self.floor_cmd_pubs.get(floor)
        if pub is None:
            raise ValueError(f"unsupported floor: {floor}")
        pub.publish(String(data=command))

    def floor_command_to_serial(self, payload):
        if "raw" in payload:
            return str(payload["raw"]).strip()
        cmd = str(payload.get("cmd", "")).lower()
        if cmd in {"stop", "stop_all"}:
            return "STOP"
        if cmd in {"clear_fault", "clearfault"}:
            return "CLEAR_FAULT"
        if cmd == "zero":
            return "ZERO"
        if cmd in {"status", "tel", "telemetry"}:
            return "STATUS"
        if cmd == "move":
            belt = int(payload["belt"])
            direction = int(payload["dir"])
            mm = float(payload["mm"])
            rpm = float(payload.get("rpm", 45.0))
            command = f"MOVE {belt} {direction} {mm:.2f} {rpm:.2f}"
            tof_stop = payload.get("tof_stop")
            if isinstance(tof_stop, dict):
                try:
                    ch = int(tof_stop["channel"])
                    mode = str(tof_stop["mode"]).lower()
                    th = float(tof_stop["threshold"])
                    if mode in {"box", "empty"}:
                        command += f" TOF {ch} {mode} {th:.2f}"
                except (KeyError, TypeError, ValueError):
                    pass
            return command
        if cmd in {"run_for", "timed_run"}:
            belt = int(payload["belt"])
            direction = int(payload.get("dir", -1))
            rpm = float(payload.get("rpm", 45.0))
            sec = float(payload.get("sec", payload.get("duration_sec", 0.0)))
            return f"AUXRUN {belt} {direction} {rpm:.2f} {int(round(sec * 1000.0))}"
        if cmd == "set":
            key = str(payload.get("key", "")).lower()
            value = payload.get("value")
            simple_keys = {
                "tof": "TOF",
                "rpm": "RPM",
                "kp": "KP",
                "ki": "KI",
                "kd": "KD",
                "slowdown": "SLOWDOWN",
                "minrpm": "MINRPM",
                "pwmstep": "PWMSTEP",
                "tof_deadband": "TOF_DEADBAND",
                "tofdeadband": "TOF_DEADBAND",
            }
            if key in simple_keys:
                return f"SET {simple_keys[key]} {float(value):.6g}"
            if key == "mmcount":
                suffix = f" {int(payload['dir'])}" if "dir" in payload else ""
                return f"SET MMCOUNT {int(payload['belt'])} {float(value):.6f}{suffix}"
            if key in {"move_scale", "movescale"}:
                suffix = f" {int(payload['dir'])}" if "dir" in payload else ""
                return f"SET MOVE_SCALE {int(payload['belt'])} {float(value):.6f}{suffix}"
            if key in {"move_offset", "moveoffset"}:
                suffix = f" {int(payload['dir'])}" if "dir" in payload else ""
                return f"SET MOVE_OFFSET {int(payload['belt'])} {float(value):.3f}{suffix}"
            if key == "distcal":
                return (
                    f"SET DISTCAL {int(payload['belt'])} {int(payload['dir'])} "
                    f"{float(payload['scale']):.6f} {float(payload.get('offset', 0.0)):.3f}"
                )
            if key in {"distbin", "distance_bin", "distancebin"}:
                return (
                    f"SET DISTBIN {int(payload['belt'])} {int(payload['dir'])} {int(payload['bin'])} "
                    f"{float(payload['scale']):.6f} {float(payload.get('offset', 0.0)):.3f}"
                )
        return None

    def selected_floor_is_active_supervisor(self, floor: int) -> bool:
        now = time.time()
        with self.state_lock:
            state = self.floor_states.get(floor, {})
            status_at = float(state.get("status_at") or 0.0)
            status = dict(state.get("status") or {})
        if status_at > 0.0 and now - status_at < 5.0 and status:
            return True
        try:
            active_floor = int(self.status.get("floor") or 0)
        except (TypeError, ValueError):
            active_floor = 0
        return active_floor == floor

    def floor_snapshot(self):
        now = time.time()
        with self.state_lock:
            snapshot = json.loads(json.dumps(self.floor_states))
        for state in snapshot.values():
            for stamp_key, age_key in (
                ("telemetry_at", "telemetry_age_sec"),
                ("bridge_at", "bridge_age_sec"),
                ("event_at", "event_age_sec"),
                ("status_at", "status_age_sec"),
                ("db_at", "db_age_sec"),
                ("twin_at", "twin_age_sec"),
                ("logs_at", "logs_age_sec"),
            ):
                stamp = float(state.get(stamp_key) or 0.0)
                state[age_key] = round(now - stamp, 3) if stamp > 0.0 else 9999.0
        return snapshot

    def status_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.status = payload
        self.update_floor_from_payload(payload, "status")

    def db_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.db = payload
        self.update_floor_from_payload(payload, "db")

    def twin_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.twin = payload
        self.update_floor_from_payload(payload, "twin")

    def update_floor_from_payload(self, payload, key: str):
        floor = None
        if isinstance(payload, dict):
            floor = payload.get("floor")
        elif isinstance(payload, list):
            floors = {
                int(row.get("floor"))
                for row in payload
                if isinstance(row, dict) and row.get("floor") is not None
            }
            if len(floors) == 1:
                floor = floors.pop()
        if floor is None:
            return
        floor = self.clamp_floor(floor)
        now = time.time()
        with self.state_lock:
            state = self.floor_states.setdefault(floor, {})
            state[key] = payload
            state[f"{key}_at"] = now

    @staticmethod
    def format_log_item(item):
        detail = " ".join(f"{k}={v}" for k, v in item.items() if k not in {"event", "level"})
        return {
            "time": int(time.time()),
            "event": item.get("event", "event"),
            "detail": detail,
        }

    def log_callback(self, msg: String):
        try:
            item = json.loads(msg.data)
        except json.JSONDecodeError:
            item = {"event": "raw", "detail": msg.data}
        self.logs.append(self.format_log_item(item))

    def configure_routes(self):
        @self.app.get("/")
        def index():
            return HTML

        @self.app.get("/api/state")
        def state():
            return jsonify({
                "status": self.status,
                "db": self.db,
                "twin": self.twin,
                "floors": self.floor_snapshot(),
                "logs": list(self.logs),
            })

        @self.app.get("/api/twin_image")
        def twin_image():
            floor = self.clamp_floor(request.args.get("floor", 0))
            if floor == 1:
                render_dir = self.get_parameter("twin_render_dir_floor1").get_parameter_value().string_value
            elif floor == 2:
                render_dir = self.get_parameter("twin_render_dir_floor2").get_parameter_value().string_value
            else:
                render_dir = self.get_parameter("twin_render_dir").get_parameter_value().string_value
            path = os.path.join(render_dir, "latest.png")
            if not os.path.exists(path):
                fallback_dir = self.get_parameter("twin_render_dir").get_parameter_value().string_value
                fallback_path = os.path.join(fallback_dir, "latest.png")
                if os.path.exists(fallback_path):
                    path = fallback_path
                else:
                    return ("", 404)
            response = send_file(path, mimetype="image/png", max_age=0)
            response.headers["Cache-Control"] = "no-store, max-age=0"
            return response

        @self.app.post("/api/cmd")
        def command():
            payload = request.get_json(force=True, silent=True) or {}
            if "raw" in payload:
                floor = self.clamp_floor(payload.get("floor", 0)) if "floor" in payload else 0
                if floor not in self.floor_control_pubs:
                    return jsonify({"ok": False, "error": "floor is required for raw commands"}), 400
                self.floor_control_pubs[floor].publish(String(data=str(payload["raw"])))
                return jsonify({"ok": True, "floor": floor})
            if not self.publish_control(payload):
                return jsonify({"ok": False, "error": "floor is required"}), 400
            return jsonify({"ok": True, "floor": self.clamp_floor(payload.get("floor", 1))})

        @self.app.post("/api/floor_cmd")
        def floor_command():
            payload = request.get_json(force=True, silent=True) or {}
            floor = self.clamp_floor(payload.get("floor", 1))
            payload["floor"] = floor
            if self.selected_floor_is_active_supervisor(floor):
                self.publish_control(payload)
                return jsonify({"ok": True, "route": "supervisor", "floor": floor})
            serial_cmd = self.floor_command_to_serial(payload)
            if not serial_cmd:
                return jsonify({
                    "ok": False,
                    "route": "unsupported",
                    "floor": floor,
                    "error": "이 명령은 현재 운용 중인 supervisor 층에서만 적용할 수 있습니다.",
                }), 400
            self.publish_floor_serial(floor, serial_cmd)
            return jsonify({"ok": True, "route": "arduino_bridge", "floor": floor, "serial": serial_cmd})

        @self.app.post("/api/twin_cmd")
        def twin_command():
            payload = request.get_json(force=True, silent=True) or {}
            if not self.publish_twin_control(payload):
                return jsonify({"ok": False, "error": "floor is required"}), 400
            return jsonify({"ok": True, "floor": self.clamp_floor(payload.get("floor", 1))})

    def run_flask(self):
        host = self.get_parameter("host").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.get_logger().info(f"Web control listening on http://{host}:{port}")
        self.app.run(host=host, port=port, threaded=True, use_reloader=False)


def main(args=None):
    lock_file = acquire_singleton_lock("refuge_web_control")
    rclpy.init(args=args)
    node = WebControl()
    thread = threading.Thread(target=node.run_flask, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock_file.close()


if __name__ == "__main__":
    main()
