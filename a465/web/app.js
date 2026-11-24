import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/OrbitControls.js';
import { buildArm, buildTrack, HOUSING_COLOR } from '/static/arm.js';

const $ = (id) => document.getElementById(id);
const fmt = (v) => (v === undefined ? '–' : v.toFixed(1));
let model = null;
let state = null;
let socket = null;
let frame = 'joint';
let rows = [];

// ---------------------------------------------------------------- 3D scene

const renderer = new THREE.WebGLRenderer({ canvas: $('scene'), antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(38, 1, 10, 20000);
camera.up.set(0, 0, 1);                       // the robot's world is Z up
camera.position.set(1500, -1750, 1150);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(150, 0, 480);
controls.enableDamping = true;
controls.minDistance = 500;
controls.maxDistance = 9000;

const sky = new THREE.HemisphereLight(0xffffff, 0x39424e, 1.7);
sky.position.set(0, 0, 1);
scene.add(sky);
const sun = new THREE.DirectionalLight(0xffffff, 2.0);
sun.position.set(900, -1300, 2200);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -1300, right: 1300, top: 1300, bottom: -1300, near: 200, far: 5000 });
scene.add(sun);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const floor = new THREE.Mesh(new THREE.PlaneGeometry(9000, 6000), new THREE.ShadowMaterial({ opacity: 0.35 }));
floor.position.set(1100, 0, -0.5);
floor.receiveShadow = true;
scene.add(floor);

const grid = new THREE.GridHelper(4800, 48, 0x4a5565, 0x252c37);   // 100 mm squares
grid.rotation.x = Math.PI / 2;
grid.position.x = 1100;
scene.add(grid);
for (const [dir, color] of [[[1, 0, 0], 0xff6b6b], [[0, 1, 0], 0x51cf66], [[0, 0, 1], 0x4dabf7]]) {
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(...dir), new THREE.Vector3(0, 0, 1), 300, color, 50, 24));
}

const LIMIT_COLOR = 0xf85149;
let arm = null;
const rig = new THREE.Group();                   // everything that rides the track: carriage and arm
scene.add(rig);
const shown = [0, 0, 0, 0, 0, 0];
let shownTrack = 0;

let framed = false;
function resize() {
  const { clientWidth: w, clientHeight: h } = $('view');
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  if (!framed && camera.aspect < 1.2) {            // narrow screen: start further back so the whole arm fits
    camera.position.sub(controls.target).multiplyScalar(1.2 / camera.aspect).add(controls.target);
  }
  framed = true;
}
new ResizeObserver(resize).observe($('view'));

const ORBIT_RAD_S = 1.8;                         // view rotation at full stick
const ZOOM_PER_S = 1.2;                          // distance changes by e^1.2 (about 3.3x) per second at full stick
function orbit(right, down, zoomIn, seconds) {
  const offset = camera.position.clone().sub(controls.target);
  const radius = THREE.MathUtils.clamp(offset.length() * Math.exp(-zoomIn * ZOOM_PER_S * seconds),
    controls.minDistance, controls.maxDistance);
  const around = Math.atan2(offset.y, offset.x) - right * ORBIT_RAD_S * seconds;   // stick right: look right
  const fromTop = THREE.MathUtils.clamp(Math.acos(offset.z / radius) - down * ORBIT_RAD_S * seconds, 0.08, Math.PI * 0.52);
  camera.position.set(Math.sin(fromTop) * Math.cos(around), Math.sin(fromTop) * Math.sin(around), Math.cos(fromTop))
    .multiplyScalar(radius).add(controls.target);
}

let lastFrame = performance.now();
let lastMessage = 0;
function draw(now) {
  const blend = 1 - Math.exp(-(now - lastFrame) / 45);   // ease towards the latest reported angles
  const seconds = Math.min((now - lastFrame) / 1000, 0.1);
  lastFrame = now;
  const stick = state?.gamepad.camera;
  if (stick && stick.some(Boolean) && now - lastMessage < 300) orbit(...stick, seconds);   // stale data never moves the view
  if (model && state) {
    state.robot.joints.forEach((angle, i) => {
      shown[i] += (angle - shown[i]) * blend;
      arm.pivots[i].rotation[model.joints[i].axis] = model.joints[i].sign * THREE.MathUtils.degToRad(shown[i]);
      arm.housing[i].color.setHex(state.robot.at_limit[i] ? LIMIT_COLOR : HOUSING_COLOR);
    });
    const gap = state.robot.gripper_open ? 34 : 9;
    for (const finger of arm.fingers) finger.position.y += (finger.userData.side * gap - finger.position.y) * blend;
    const moved = (state.robot.track - shownTrack) * blend;
    shownTrack += moved;
    rig.position.x = shownTrack;
    if (!sideView) {                                             // the view rides along with the arm, except the map view
      camera.position.x += moved;
      controls.target.x += moved;
    }
    sky.intensity = state.robot.light ? 2.6 : 1.7;
    arm.toolMark.emissive.setHex(state.robot.tool ? 0xf2a900 : 0x000000);
    track.rail.visible = track.carriage.visible = state.robot.track_fitted;
    drawCubes();
    updateMapLive();
    const busy = state.robot.command?.kind === 'cycle' ? state.robot.command.cell : null;
    for (const mark of marks.children) mark.userData.tile.material.opacity = mark.userData.name === busy ? 0.95 : 0.5;
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(draw);
}

// ---------------------------------------------------------------- link to the server

function connect() {
  socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/ws`);
  socket.onopen = () => setLink(true);
  socket.onclose = () => { setLink(false); setTimeout(connect, 1000); };
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.error) toast(message.error);
    else { state = message; lastMessage = performance.now(); render(); }
  };
}

function send(command) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(command));
  else toast('not connected to the server');
}

function setLink(up) {
  $('link').className = `link ${up ? 'up' : 'down'}`;
  $('link').textContent = up ? 'live' : 'offline';
}

let toastTimer = null;
function toast(text) {
  $('toast').textContent = text;
  $('toast').hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $('toast').hidden = true; }, 2500);
}

// ---------------------------------------------------------------- jog rows

const CARTESIAN = [['X', 'x'], ['Y', 'y'], ['Z', 'z'], ['Roll', 'roll'], ['Pitch', 'pitch'], ['Yaw', 'yaw']];

function buildFrames() {
  $('frames').replaceChildren(...model.frames.map((name) => {
    const tab = document.createElement('button');
    tab.textContent = name;
    tab.classList.toggle('sel', name === frame);
    tab.onclick = () => { stopJog(); frame = name; buildFrames(); buildRows(); render(); };
    return tab;
  }));
}

function buildRows() {
  const makeRow = (index, label, name, limits) => {
    const row = document.createElement('div');
    row.className = 'axis';
    row.innerHTML = `<button aria-label="${label} minus">−</button>
      <div class="mid"><div class="label"><span>${label}${name ? ` · ${name}` : ''}</span><b></b></div>
      ${limits ? '<div class="bar"><i></i></div>' : ''}</div>
      <button aria-label="${label} plus">+</button>`;
    const [minus, plus] = row.querySelectorAll('button');
    holdToJog(minus, jogCommand(index, -1));
    holdToJog(plus, jogCommand(index, 1));
    const entry = { row, value: row.querySelector('b'), marker: row.querySelector('.bar i'), limits };
    return [row, entry];
  };
  const made = frame === 'joint'
    ? model.joints.map((joint, i) => makeRow(i, `J${i + 1}`, joint.name, [joint.min, joint.max]))
    : CARTESIAN.map(([label], i) => makeRow(i, label, '', null));
  rows = made.map(([, entry]) => entry);
  $('jog').replaceChildren(...made.map(([row]) => row));
}

// Track and carousel: the same hold-to-move rows, shown in every frame.
let auxRows = [];
function buildAux() {
  const made = [makeAuxRow(0, 'Track', 'mm', [model.track.min, model.track.max]), makeAuxRow(1, 'Carousel', '°', null)];
  auxRows = made.map(([, entry]) => entry);
  $('aux').replaceChildren(...made.map(([row]) => row));
}

function makeAuxRow(index, label, unit, limits) {
  const row = document.createElement('div');
  row.className = 'axis';
  row.innerHTML = `<button aria-label="${label} minus">−</button>
    <div class="mid"><div class="label"><span>${label} · ${unit}</span><b></b></div>
    ${limits ? '<div class="bar"><i></i></div>' : ''}</div>
    <button aria-label="${label} plus">+</button>`;
  const [minus, plus] = row.querySelectorAll('button');
  for (const [button, direction] of [[minus, -1], [plus, 1]]) {
    holdToJog(button, () => {
      const aux = [0, 0];
      aux[index] = direction;
      return { op: 'jog', frame: 'joint', vec: [0, 0, 0, 0, 0, 0], aux, ttl: 0.3 };
    });
  }
  return [row, { row, value: row.querySelector('b'), marker: row.querySelector('.bar i'), limits }];
}

function jogCommand(index, direction) {
  return () => {
    const vec = [0, 0, 0, 0, 0, 0];
    vec[index] = direction;
    return { op: 'jog', frame, vec, ttl: 0.3 };
  };
}

let jogTimer = null;
let heldButton = null;

function holdToJog(button, command) {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    stopJog();
    button.setPointerCapture(event.pointerId);
    const refresh = () => send(command());
    refresh();
    jogTimer = setInterval(refresh, 100);
    heldButton = button;
    button.classList.add('held');
  });
  for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(type, stopJog);
  button.addEventListener('contextmenu', (event) => event.preventDefault());
}

function stopJog() {
  if (jogTimer === null) return;
  clearInterval(jogTimer);
  jogTimer = null;
  heldButton.classList.remove('held');
  send({ op: 'jog', frame: 'joint', vec: [0, 0, 0, 0, 0, 0], ttl: 0.05 });   // a zero jog brakes at once
}
window.addEventListener('blur', stopJog);
document.addEventListener('visibilitychange', stopJog);

// ---------------------------------------------------------------- panel

$('power').onclick = () => send({ op: 'armpower', on: !state?.robot.arm_power });
$('halt').onclick = () => { stopJog(); send({ op: 'halt' }); };
$('gripper').onclick = () => send({ op: 'gripper', open: !state?.robot.gripper_open });
$('home').onclick = () => send({ op: 'home' });
$('cycle').onclick = () => send({ op: 'cycle' });
$('track-fitted').onclick = () => send({ op: 'track', fitted: !state?.robot.track_fitted });
$('carousel-fitted').onclick = () => send({ op: 'track', carousel: !state?.robot.carousel_fitted });
$('track-moves').onclick = () => send({ op: 'track', moves: !state?.robot.track_moves });
$('light').onclick = () => send({ op: 'light', on: !state?.robot.light });
$('tool').onclick = () => send({ op: 'tool', on: !state?.robot.tool });
for (const button of document.querySelectorAll('[data-goto]')) {
  button.onclick = () => send({ op: 'goto', pose: button.dataset.goto });
}
$('speed').oninput = () => {
  $('speed-out').textContent = `${$('speed').value}%`;
  send({ op: 'speed', percent: Number($('speed').value) });
};

let helpFrame = null;
function buildHelp(padFrame) {
  if (padFrame === helpFrame) return;
  helpFrame = padFrame;
  const help = model.gamepad_help;
  const entries = [...help.before, ...help[padFrame === 'joint' ? 'joint' : 'world'], ...help.after];
  $('pad-help').replaceChildren(...entries.flatMap(([key, text]) => {
    const dt = document.createElement('dt');
    const dd = document.createElement('dd');
    dt.textContent = key;
    dd.textContent = text;
    return [dt, dd];
  }));
}

function chip(element, text, kind) {
  element.textContent = text;
  element.className = `chip ${kind}`;
}

function render() {
  if (!model || !state) return;
  const { robot, gamepad } = state;
  chip($('backend'), robot.backend === 'mock' ? 'MOCK ROBOT' : robot.backend.toUpperCase(), robot.backend === 'mock' ? 'warn' : 'ok');
  if (!robot.arm_power) chip($('motion'), 'ARM POWER OFF', 'dim');
  else chip($('motion'), robot.moving ? 'MOVING' : 'IDLE', robot.moving ? 'ok' : 'dim');
  chip($('homed'), robot.homed ? 'HOMED' : 'NOT HOMED', robot.homed ? 'ok' : 'warn');
  $('source').hidden = !robot.command;
  if (robot.command) {
    const progress = robot.command.kind === 'cycle' ? ` ${robot.command.step}/${robot.command.of} · ${robot.command.label}` : '';
    $('source').textContent = `${robot.command.kind}${progress} · ${robot.command.source}`;
  }
  $('notice').hidden = !robot.notice;
  $('notice').textContent = robot.notice;
  watchJob(robot);

  for (const key of ['x', 'y', 'z', 'yaw', 'pitch', 'roll']) $(`pose-${key}`).textContent = fmt(robot.pose[key]);

  $('power').textContent = `Arm power ${robot.arm_power ? 'ON' : 'OFF'}`;
  $('power').classList.toggle('on', robot.arm_power);
  $('gripper').textContent = `Gripper: ${robot.gripper_open ? 'open' : 'closed'}`;
  $('home').classList.toggle('on', robot.homed);
  $('cycle').classList.toggle('on', robot.command?.kind === 'cycle');
  for (const name of ['light', 'tool']) {
    $(name).textContent = `${name === 'light' ? 'Light' : 'Tool'}: ${robot[name] ? 'on' : 'off'}`;
    $(name).classList.toggle('on', robot[name]);
  }
  auxRows[0].row.hidden = !robot.track_fitted;                     // no track: no jog control or readout
  auxRows[1].row.hidden = !robot.carousel_fitted;
  const idle = !robot.moving && !robot.command;
  for (const [id, text, on, enabled] of [
    ['track-fitted', 'Track fitted', robot.track_fitted, idle],
    ['carousel-fitted', 'Carousel fitted', robot.carousel_fitted, idle],
    ['track-moves', 'Move track in cycle', robot.track_moves, idle && robot.track_fitted]]) {
    $(id).textContent = `${text}: ${on ? 'on' : 'off'}`;
    $(id).classList.toggle('on', on);
    $(id).disabled = !enabled;
  }
  auxRows[0].value.textContent = fmt(robot.track);
  auxRows[0].marker.style.left = `${(robot.track / model.track.max) * 100}%`;
  auxRows[0].row.classList.toggle('limit', robot.track_at_limit);
  auxRows[1].value.textContent = fmt(robot.carousel);
  if (!$('speed').matches(':active')) {
    $('speed').value = robot.speed;
    $('speed-out').textContent = `${robot.speed}%`;
  }

  rows.forEach((entry, i) => {
    const value = frame === 'joint' ? robot.joints[i] : robot.pose[CARTESIAN[i][1]];
    entry.value.textContent = fmt(value);
    if (entry.limits) {
      const [low, high] = entry.limits;
      entry.marker.style.left = `${((value - low) / (high - low)) * 100}%`;
      entry.row.classList.toggle('limit', robot.at_limit[i]);
    }
  });

  $('pad-live').hidden = !gamepad.connected;
  $('pad-name').textContent = gamepad.connected
    ? `${gamepad.name} · ${gamepad.frame} frame${gamepad.message ? ` · ${gamepad.message}` : ''}`
      + (gamepad.watchdog ? '' : ` · no signal watchdog (${gamepad.watchdog_note})`)
    : gamepad.message;
  if (!gamepad.connected) chip($('pad-state'), 'not connected', 'dim');
  else if (gamepad.signal_lost) chip($('pad-state'), 'SIGNAL LOST', 'danger');
  else chip($('pad-state'), gamepad.enabled ? 'ENABLED (L1)' : 'connected', gamepad.enabled ? 'warn' : 'ok');
  if (gamepad.connected) {
    buildHelp(gamepad.frame);
    $('stick-l').style.transform = `translate(${(gamepad.axes.lx || 0) * 20}px, ${(gamepad.axes.ly || 0) * 20}px)`;
    $('stick-r').style.transform = `translate(${(gamepad.axes.rx || 0) * 20}px, ${(gamepad.axes.ry || 0) * 20}px)`;
    $('pad-buttons').replaceChildren(...gamepad.buttons.map((name) => {
      const tag = document.createElement('span');
      tag.textContent = name.replace('dpad_', 'd-pad ');
      return tag;
    }));
  }
}

// ---------------------------------------------------------------- pick and place

const PICK_COLOR = 0xf2a900;
const PLACE_COLOR = 0x58a6ff;
let job = null;                 // the grid, the chosen cells and the planned cycle, as /api/job reports them
let choosing = false;           // a click on a cell adds or removes it
let jobVersion = 0;
let planTimer = null;
const work = new THREE.Group(); // lies on the work plane
const tiles = new THREE.Group();
const marks = new THREE.Group();
const cubes = new THREE.Group();
work.add(tiles, marks);
scene.add(work, cubes);
let pathLine = null;
const hover = new THREE.Mesh(new THREE.PlaneGeometry(1, 1),
  new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.3, depthWrite: false, side: THREE.DoubleSide }));
hover.visible = false;
work.add(hover);

const cellName = (col, row) => `${String.fromCharCode(65 + row)}${col + 1}`;
function cellCentre(name) {
  const g = job.grid;
  return [g.x_min + (Number(name.slice(1)) - 0.5) * g.cell, g.y_min + (name.charCodeAt(0) - 65 + 0.5) * g.cell];
}

async function fetchJob() {
  showJob(await (await fetch('/api/job')).json());
}

async function postJob(change) {
  const reply = await (await fetch('/api/job', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ picks: job.picks, places: job.places, ...change }),
  })).json();
  if (reply.error) toast(reply.error);
  else showJob(reply.job);
}

// Ask for the plan again when the cells change, and when the arm comes to rest somewhere new
// or at a new speed (the plan starts from where the arm is and is timed for the speed set).
let plannedFor = '';
let wasBusy = false;
function watchJob(robot) {
  const busy = robot.moving || robot.command?.kind === 'cycle';
  const key = `${robot.speed}|${robot.gripper_open}`;
  const stale = robot.job_version !== jobVersion || (!busy && (wasBusy || key !== plannedFor));
  wasBusy = busy;
  if (!stale) return;
  jobVersion = robot.job_version;
  plannedFor = key;
  clearTimeout(planTimer);
  planTimer = setTimeout(fetchJob, 250);
}

function tile(size, color, opacity) {
  return new THREE.Mesh(new THREE.PlaneGeometry(size, size),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false, side: THREE.DoubleSide }));
}

function tag(text, color) {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 96;
  const c = canvas.getContext('2d');
  c.fillStyle = 'rgba(13, 17, 23, 0.82)';
  c.beginPath();
  c.roundRect(4, 4, 248, 88, 22);
  c.fill();
  let size = 52;
  c.font = `600 ${size}px system-ui, sans-serif`;
  while (size > 24 && c.measureText(text).width > 228) c.font = `600 ${size -= 2}px system-ui, sans-serif`;     // long text shrinks to fit
  c.textAlign = 'center';
  c.textBaseline = 'middle';
  c.fillStyle = color;
  c.fillText(text, 128, 51);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }));
  sprite.scale.set(170, 64, 1);
  sprite.renderOrder = 20;
  return sprite;
}

function showJob(next) {
  const gridChanged = !job || job.grid.plane_z !== next.grid.plane_z || job.grid.reach.join() !== next.grid.reach.join();
  job = next;
  const g = job.grid;
  work.position.z = g.plane_z + 1;
  if (gridChanged) {                                   // faint tiles on every cell the arm can serve
    tiles.clear();
    const spots = [];
    g.reach.forEach((cells, row) => [...cells].forEach((code, col) => { if (code === 'y') spots.push([col, row]); }));
    const mesh = new THREE.InstancedMesh(new THREE.PlaneGeometry(g.cell - 6, g.cell - 6),
      new THREE.MeshBasicMaterial({ color: 0x9fb3c8, transparent: true, opacity: 0.14, depthWrite: false, side: THREE.DoubleSide }), spots.length);
    const at = new THREE.Matrix4();
    spots.forEach(([col, row], i) => mesh.setMatrixAt(i, at.makeTranslation(g.x_min + (col + 0.5) * g.cell, g.y_min + (row + 0.5) * g.cell, 0)));
    tiles.add(mesh);
    hover.scale.set(g.cell - 6, g.cell - 6, 1);
    if (document.activeElement !== $('plane-z')) $('plane-z').value = g.plane_z;
  }

  for (const mark of marks.children) { mark.userData.label.material.map.dispose(); mark.userData.label.material.dispose(); }
  marks.clear();
  for (const [kind, cells, color, css] of [['pick', job.effective_picks, PICK_COLOR, '#f2a900'], ['place', job.effective_places, PLACE_COLOR, '#58a6ff']]) {
    cells.forEach((name, i) => {
      const mark = new THREE.Group();
      const [x, y] = cellCentre(name);
      mark.position.set(x, y, 0.5);
      const square = tile(g.cell - 6, color, 0.5);
      const label = tag(`${kind} ${i + 1}`, css);
      label.position.z = 95;
      mark.add(square, label);
      mark.userData = { name, tile: square, label };
      marks.add(mark);
    });
  }

  if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); pathLine = null; }
  const plan = job.plan;
  if (plan.path.length) {                              // the fingertips' route through the whole cycle
    pathLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(plan.path.map((p) => new THREE.Vector3(...p))),
      new THREE.LineBasicMaterial({ color: plan.ok ? 0x3fb950 : 0xf85149, transparent: true, opacity: 0.9 }));
    scene.add(pathLine);
  }

  const pairs = Math.max(job.effective_picks.length, job.effective_places.length);
  $('pairs').replaceChildren(...Array.from({ length: pairs }, (_, i) => {
    const item = document.createElement('li');
    const spot = plan.pairs[i];
    const tracks = spot && spot.pick_track !== null && spot.place_track !== null
      ? `track ${spot.pick_track.toFixed(0)} → ${spot.place_track.toFixed(0)} mm` : '';
    item.innerHTML = `<b class="pick">${job.effective_picks[i] ?? '?'}</b> → <b class="place">${job.effective_places[i] ?? '?'}</b><small>${tracks}</small>`;
    return item;
  }));
  showZones();
  drawMap();
  $('plan-problems').replaceChildren(...[...plan.problems, ...plan.notes.filter((n) => /raised|left out/.test(n))].map((text) => {
    const item = document.createElement('li');
    item.textContent = text;
    return item;
  }));
  if (plan.problems.length) chip($('plan-state'), plan.problems.length === 1 ? '1 problem' : `${plan.problems.length} problems`, 'danger');
  else if (plan.ok) chip($('plan-state'), `checked · ${plan.seconds.toFixed(1)} s at ${plan.speed}%`, 'ok');
  else chip($('plan-state'), pairs ? 'needs a place cell' : 'no cells chosen', 'dim');
  $('plan-steps').hidden = !plan.steps.length;
  $('plan-steps').querySelector('summary').textContent = `${plan.steps.length} steps`;
  $('plan-steps').querySelector('ol').replaceChildren(...plan.steps.map((step) => {
    const item = document.createElement('li');
    item.textContent = `${step.label}${step.cell ? ` (${step.cell})` : ''} · ${step.seconds.toFixed(1)} s`;
    return item;
  }));
}

// Zones in the 3D view: keep-outs as translucent red boxes up to their top, areas as outlines on the plane.
const zoneShapes = new THREE.Group();
scene.add(zoneShapes);
function showZones3d() {
  for (const child of [...zoneShapes.children]) {
    child.traverse((o) => { o.geometry?.dispose(); o.material?.map?.dispose(); o.material?.dispose(); });
    zoneShapes.remove(child);
  }
  const plane = job.grid.plane_z;
  job.zones.forEach((zone, i) => {
    const w = zone.x1 - zone.x0, d = zone.y1 - zone.y0, cx = (zone.x0 + zone.x1) / 2, cy = (zone.y0 + zone.y1) / 2;
    const group = new THREE.Group();
    if (zone.kind === 'keepout') {
      const height = zone.top - plane;
      const box = new THREE.Mesh(new THREE.BoxGeometry(w, d, height),
        new THREE.MeshBasicMaterial({ color: 0xf85149, transparent: true, opacity: 0.22, depthWrite: false }));
      box.position.set(cx, cy, plane + height / 2);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry), new THREE.LineBasicMaterial({ color: 0xff7b72 }));
      edges.position.copy(box.position);
      const label = tag(`no-go ${i + 1} · ${zone.top}`, '#ff7b72');
      label.position.set(cx, cy, zone.top + 70);
      group.add(box, edges, label);
    } else {
      const color = zone.kind === 'pick' ? PICK_COLOR : PLACE_COLOR;
      const outline = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([
        [zone.x0, zone.y0], [zone.x1, zone.y0], [zone.x1, zone.y1], [zone.x0, zone.y1]].map(([x, y]) => new THREE.Vector3(x, y, plane + 3))),
      new THREE.LineBasicMaterial({ color }));
      const label = tag(`${zone.kind} area ${i + 1}`, zone.kind === 'pick' ? '#f2a900' : '#58a6ff');
      label.position.set(cx, cy, plane + 60);
      group.add(outline, label);
    }
    zoneShapes.add(group);
  });
}

// The zone form and list.
function showZones() {
  showZones3d();
  $('zones').replaceChildren(...job.zones.map((zone, i) => {
    const item = document.createElement('li');
    const extent = `X ${zone.x0} to ${zone.x1}, Y ${zone.y0} to ${zone.y1} mm`;
    const detail = zone.kind === 'keepout' ? `top at ${zone.top} mm`
      : `${zone.cells.length} ${zone.cells.length === 1 ? 'cell' : 'cells'} (${zone.left_out} left out)`;
    const names = { pick: 'Pick area', place: 'Place area', keepout: 'Keep-out zone' };
    item.innerHTML = `<div class="zrow"><span><b class="${zone.kind}">${names[zone.kind]}</b></span><button type="button" aria-label="Remove zone ${i + 1}">Remove</button></div>
      <small>${extent}</small><small>${detail}</small>`;
    item.querySelector('button').onclick = () => postJob({ zones: zonesToSend().filter((_, k) => k !== i) });
    return item;
  }));
}
const zonesToSend = () => job.zones.map(({ kind, x0, y0, x1, y1, top }) => ({ kind, x0, y0, x1, y1, top }));
$('zone-kind').onchange = () => { $('zone-top-label').hidden = $('zone-kind').value !== 'keepout'; };
$('zone-form').onsubmit = (event) => {
  event.preventDefault();
  const zone = { kind: $('zone-kind').value };
  for (const key of ['x0', 'x1', 'y0', 'y1']) zone[key] = Number($(`zone-${key}`).value);
  if (zone.kind === 'keepout') zone.top = Number($('zone-top').value);
  postJob({ zones: [...zonesToSend(), zone] });
};

// The 2D map: the whole grid from above, X to the right and Y up, as in the 3D map view.
const SVGNS = 'http://www.w3.org/2000/svg';
const MAP = { left: 340, top: 60, right: 70, bottom: 240 };
const mapX = (x) => MAP.left + x - job.grid.x_min;
const mapY = (y) => MAP.top + (job.grid.y_min + job.grid.rows * job.grid.cell) - y;
function svgEl(name, attrs, text) {
  const el = document.createElementNS(SVGNS, name);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  if (text !== undefined) el.textContent = text;
  return el;
}

function drawMap() {
  const g = job.grid;
  const width = g.cols * g.cell, height = g.rows * g.cell;
  const svg = $('map');
  svg.setAttribute('viewBox', `0 0 ${MAP.left + width + MAP.right} ${MAP.top + height + MAP.bottom}`);
  const parts = [];
  g.reach.forEach((cells, row) => [...cells].forEach((code, col) => {
    parts.push(svgEl('rect', { class: `cell c-${code}`, x: mapX(g.x_min + col * g.cell), y: mapY(g.y_min + (row + 1) * g.cell), width: g.cell, height: g.cell }));
  }));
  if (job.track_fitted) {                              // the rail, 220 mm wide, with its end stops
    const travel = model.track.max;
    parts.push(svgEl('rect', { class: 'rail', x: mapX(-235), y: mapY(110), width: travel + 470, height: 220, rx: 20 }));
    parts.push(svgEl('text', { class: 'rail-label', x: mapX(travel / 2), y: mapY(0) }, 'track'));
  }
  for (const zone of job.zones) {
    const attrs = { x: mapX(zone.x0), y: mapY(zone.y1), width: zone.x1 - zone.x0, height: zone.y1 - zone.y0 };
    parts.push(svgEl('rect', { ...attrs, class: zone.kind === 'keepout' ? 'keep' : `area ${zone.kind}` }));
  }
  job.zones.forEach((zone, i) => {
    const colour = { pick: '#f2a900', place: '#58a6ff', keepout: '#ff7b72' }[zone.kind];
    const text = zone.kind === 'keepout' ? `${i + 1}: top ${zone.top}` : `${zone.kind} ${i + 1}`;
    const label = zone.kind === 'keepout'
      ? svgEl('text', { class: 'tag', x: mapX((zone.x0 + zone.x1) / 2), y: mapY((zone.y0 + zone.y1) / 2), fill: colour }, text)
      : svgEl('text', { class: 'tag area-tag', x: mapX(zone.x0), y: mapY(zone.y1) - 46, fill: colour }, text);     // areas are labelled above their corner
    parts.push(label);
  });
  for (const [kind, cells] of [['pick', job.effective_picks], ['place', job.effective_places]]) {
    cells.forEach((name, i) => {
      const [x, y] = cellCentre(name);
      parts.push(svgEl('rect', { class: `mark ${kind}`, x: mapX(x) - 40, y: mapY(y) - 40, width: 80, height: 80, rx: 12 }));
      parts.push(svgEl('text', { class: 'markn', x: mapX(x), y: mapY(y) + 3 }, String(i + 1)));
    });
  }
  for (let row = 0; row < g.rows; row++) {
    parts.push(svgEl('text', { class: 'rowname', x: MAP.left - 22, y: mapY(g.y_min + (row + 0.5) * g.cell) }, String.fromCharCode(65 + row)));
  }
  for (let col = 0; col < g.cols; col++) {
    parts.push(svgEl('text', { class: 'colname', x: mapX(g.x_min + (col + 0.5) * g.cell), y: MAP.top + height + 70 }, String(col + 1)));
  }
  const xMax = g.x_min + width, yMax = g.y_min + g.rows * g.cell;
  for (const x of [g.x_min, 0, 1000, 2000, xMax]) {     // millimetres along the bottom: the extremes, the origin and every metre
    const edge = x === g.x_min ? 'start' : x === xMax ? 'end' : 'middle';
    parts.push(svgEl('text', { class: 'tick', x: mapX(x), y: MAP.top + height + 160, 'text-anchor': edge }, `${x} mm`));
    parts.push(svgEl('line', { x1: mapX(x), x2: mapX(x), y1: MAP.top + height, y2: MAP.top + height + 18, stroke: '#8b949e', 'stroke-width': 5 }));
  }
  for (const y of [g.y_min, 0, yMax]) {
    parts.push(svgEl('text', { class: 'tick', x: 6, y: mapY(y) + (y === yMax ? 50 : y === g.y_min ? -10 : 22) }, `${y} mm`));
  }
  parts.push(svgEl('rect', { id: 'map-drag', x: 0, y: 0, width: 0, height: 0, visibility: 'hidden' }));
  parts.push(svgEl('circle', { id: 'map-base', class: 'base', r: 44, cx: 0, cy: 0 }));
  parts.push(svgEl('line', { id: 'map-reach', class: 'reach', x1: 0, y1: 0, x2: 0, y2: 0 }));
  parts.push(svgEl('circle', { id: 'map-tip', class: 'tip', r: 32, cx: 0, cy: 0 }));
  svg.replaceChildren(...parts);
}

// The arm's base and fingertip, live.
const fingertip = new THREE.Vector3();
function updateMapLive() {
  const base = $('map-base');
  if (!base || !job || !arm) return;
  const hand = arm.fingers[0].parent;
  hand.updateWorldMatrix(true, false);
  fingertip.set(0, 0, model.flange_offset[2] + model.gripper_length).applyMatrix4(hand.matrixWorld);
  const [bx, by, tx, ty] = [mapX(rig.position.x), mapY(0), mapX(fingertip.x), mapY(fingertip.y)];
  base.setAttribute('cx', bx); base.setAttribute('cy', by);
  $('map-tip').setAttribute('cx', tx); $('map-tip').setAttribute('cy', ty);
  const line = $('map-reach');
  line.setAttribute('x1', bx); line.setAttribute('y1', by); line.setAttribute('x2', tx); line.setAttribute('y2', ty);
}

// Pointer on the map: a click is a click on a cell; a drag draws a rectangle that fills in the zone form.
function mapPoint(event) {
  const svg = $('map');
  const at = new DOMPoint(event.clientX, event.clientY).matrixTransform(svg.getScreenCTM().inverse());
  const g = job.grid;
  return [at.x - MAP.left + g.x_min, g.y_min + g.rows * g.cell - (at.y - MAP.top)];     // world mm
}
let mapPress = null;
$('map').addEventListener('pointerdown', (event) => {
  if (!job) return;
  $('map').setPointerCapture(event.pointerId);
  mapPress = { screen: [event.clientX, event.clientY], world: mapPoint(event), dragged: false };
});
$('map').addEventListener('pointermove', (event) => {
  if (!mapPress) return;
  if (Math.hypot(event.clientX - mapPress.screen[0], event.clientY - mapPress.screen[1]) > 6) mapPress.dragged = true;
  if (!mapPress.dragged) return;
  const [x, y] = mapPoint(event);
  const [x0, x1] = [mapPress.world[0], x].sort((a, b) => a - b);
  const [y0, y1] = [mapPress.world[1], y].sort((a, b) => a - b);
  const box = $('map-drag');
  Object.entries({ x: mapX(x0), y: mapY(y1), width: x1 - x0, height: y1 - y0, visibility: 'visible' }).forEach(([k, v]) => box.setAttribute(k, v));
});
$('map').addEventListener('pointerup', (event) => {
  const press = mapPress;
  mapPress = null;
  if (!press || !job) return;
  $('map-drag').setAttribute('visibility', 'hidden');
  const g = job.grid;
  const [x, y] = mapPoint(event);
  if (press.dragged) {
    const snap = (v, lo, hi) => Math.min(hi, Math.max(lo, Math.round(v / 50) * 50));
    const xs = [press.world[0], x].map((v) => snap(v, g.x_min, g.x_min + g.cols * g.cell)).sort((a, b) => a - b);
    const ys = [press.world[1], y].map((v) => snap(v, g.y_min, g.y_min + g.rows * g.cell)).sort((a, b) => a - b);
    [$('zone-x0').value, $('zone-x1').value, $('zone-y0').value, $('zone-y1').value] = [...xs, ...ys];
    return;
  }
  if (!choosing) return;
  const col = Math.floor((x - g.x_min) / g.cell), row = Math.floor((y - g.y_min) / g.cell);
  if (col >= 0 && col < g.cols && row >= 0 && row < g.rows) clickCell({ name: cellName(col, row), code: g.reach[row][col] });
});
$('map').addEventListener('pointercancel', () => { mapPress = null; $('map-drag')?.setAttribute('visibility', 'hidden'); });
$('map-big').onclick = () => {
  const big = $('map-wrap').classList.toggle('big');
  $('map-big').classList.toggle('on', big);
  $('map-big').textContent = big ? 'Shrink map' : 'Enlarge map';
};

// Cubes: free ones sit where the server says; a held one rides in the drawn gripper.
function drawCubes() {
  if (!job) return;
  const parts = state.robot.parts;
  const size = job.grid.part;
  while (cubes.children.length < parts.length) {
    const cube = new THREE.Mesh(new THREE.BoxGeometry(size, size, size),
      new THREE.MeshStandardMaterial({ color: 0xd9822b, roughness: 0.6 }));
    cube.castShadow = true;
    cubes.add(cube);
  }
  while (cubes.children.length > parts.length) cubes.remove(cubes.children[cubes.children.length - 1]);
  const hand = arm.fingers[0].parent;
  parts.forEach((part, i) => {
    const cube = cubes.children[i];
    if (part.held) {
      hand.updateWorldMatrix(true, false);
      cube.position.set(0, 0, model.flange_offset[2] + model.gripper_length + job.grid.grip_height - size / 2).applyMatrix4(hand.matrixWorld);
      hand.getWorldQuaternion(cube.quaternion);
    } else {
      cube.position.set(part.x, part.y, part.z + size / 2);
      cube.quaternion.identity();
    }
  });
}

const raycaster = new THREE.Raycaster();
function cellAt(event) {
  if (!job) return null;
  const box = renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2(((event.clientX - box.left) / box.width) * 2 - 1,
    -((event.clientY - box.top) / box.height) * 2 + 1), camera);
  const g = job.grid;
  const hit = raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -g.plane_z), new THREE.Vector3());
  if (!hit) return null;
  const col = Math.floor((hit.x - g.x_min) / g.cell);
  const row = Math.floor((hit.y - g.y_min) / g.cell);
  if (col < 0 || col >= g.cols || row < 0 || row >= g.rows) return null;
  return { name: cellName(col, row), code: g.reach[row][col] };
}

function clickCell(cell) {
  const picks = [...job.picks];
  const places = [...job.places];
  if (picks.includes(cell.name)) picks.splice(picks.indexOf(cell.name), 1);
  else if (places.includes(cell.name)) places.splice(places.indexOf(cell.name), 1);
  else if (cell.code !== 'y') { toast(`${cell.name} is ${{ t: 'over the track', k: 'inside a keep-out zone' }[cell.code] ?? 'out of reach'}`); return; }
  else (picks.length <= places.length ? picks : places).push(cell.name);     // pick, then its place, then the next pick
  postJob({ picks, places });
}

let pressedAt = null;
$('scene').addEventListener('pointerdown', (event) => { pressedAt = [event.clientX, event.clientY]; });
$('scene').addEventListener('pointerup', (event) => {
  const dragged = !pressedAt || Math.hypot(event.clientX - pressedAt[0], event.clientY - pressedAt[1]) > 6;
  pressedAt = null;
  if (!choosing || dragged) return;                    // a drag turns the view; only a click chooses
  const cell = cellAt(event);
  if (cell) clickCell(cell);
});
$('scene').addEventListener('pointermove', (event) => {
  const cell = choosing ? cellAt(event) : null;
  hover.visible = Boolean(cell);
  if (cell) hover.position.set(...cellCentre(cell.name), 1);
});
$('scene').addEventListener('pointerleave', () => { hover.visible = false; });

$('choose').onclick = () => {
  choosing = !choosing;
  $('choose').classList.toggle('on', choosing);
  $('map').classList.toggle('choosing', choosing);
  $('scene').classList.toggle('choosing', choosing);
  hover.visible = false;
};
$('job-clear').onclick = () => postJob({ picks: [], places: [] });
$('plane-z').onchange = () => postJob({ plane_z: Number($('plane-z').value) });

let sideView = null;
$('top-view').onclick = () => {
  if (sideView) {
    const rode = shownTrack - sideView.track;          // catch up with the arm's travel while in the map view
    camera.position.copy(sideView.position).setX(sideView.position.x + rode);
    controls.target.copy(sideView.target).setX(sideView.target.x + rode);
    sideView = null;
  } else {
    sideView = { position: camera.position.clone(), target: controls.target.clone(), track: shownTrack };
    const g = job.grid;                                // straight down on the whole grid, as a map
    const half = Math.tan(THREE.MathUtils.degToRad(camera.fov / 2));
    const height = Math.min(controls.maxDistance,
      1.06 * Math.max((g.cols * g.cell) / 2 / (half * camera.aspect), (g.rows * g.cell) / 2 / half));
    const x = g.x_min + (g.cols * g.cell) / 2;
    controls.target.set(x, 0, g.plane_z);
    camera.position.set(x, -height / 80, g.plane_z + height);   // a hair off vertical so that Y stays up on the screen
  }
  $('top-view').classList.toggle('on', Boolean(sideView));
};

// ---------------------------------------------------------------- start

model = await (await fetch('/api/model')).json();
arm = buildArm(model);
const track = buildTrack(model.track.max);
scene.add(track.rail);
rig.add(track.carriage, arm.root);
grid.position.z = track.floorZ;
floor.position.z = track.floorZ - 0.5;
buildFrames();
buildRows();
buildAux();
await fetchJob();
resize();
connect();
window.a465 = { camera, controls, cellCentre, job: () => job };   // for inspection from the browser console
requestAnimationFrame(draw);
