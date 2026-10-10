import * as THREE from './vendor/three/three.module.min.js';
import { WORLD_TO_THREE, loadScene, PoseStream } from './scene.js';

const JOINTS = [
  'wrist', 'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip',
  ...['index', 'middle', 'ring', 'pinky'].flatMap((finger) => [
    `${finger}-finger-metacarpal`, `${finger}-finger-phalanx-proximal`,
    `${finger}-finger-phalanx-intermediate`, `${finger}-finger-phalanx-distal`, `${finger}-finger-tip`,
  ]),
];
const STAGES = {
  idle: '待开始', preparing: '准备中', preparation_failed: '准备失败',
  recording: '录制中', paused: '已暂停', blending: '平滑接入中',
  reverting: '回退中', rewind_wait: '等待恢复目标', saving: '保存中',
  aborting: '保留未完成数据', discarding: '结束中', error: '异常',
  binding: '等待稳定跟踪并绑定', rebinding: '冻结重绑定',
  auto_paused: '跟踪丢失，自动暂停', returning_home: '准备区安全回零', home_paused: '回零已暂停',
};
const ui = Object.fromEntries([
  'scene', 'scene-message', 'connection', 'connect', 'task-title', 'task-goal', 'stage',
  'tracking', 'frames', 'control', 'notice', 'error', 'duration', 'enter-vr', 'xr-support',
].map((id) => [id, document.getElementById(id)]));
const commandButtons = [...document.querySelectorAll('[data-key]')];
const scene = new THREE.Scene();
scene.background = new THREE.Color('#182331');
scene.add(new THREE.HemisphereLight(0xe8f3ff, 0x697078, 2.4));
const light = new THREE.DirectionalLight(0xffffff, 2.7);
light.position.set(-3, 6, 4);
scene.add(light);
const rig = new THREE.Group();
scene.add(rig);
const xrCamera = new THREE.PerspectiveCamera(65, 1, 0.02, 150);
rig.add(xrCamera);
const observer = new THREE.PerspectiveCamera(50, 1, 0.02, 150);
const observerTarget = new THREE.Vector3();
const observerOffset = new THREE.Spherical(3, 1.1, 0.7);
let observerHome = null;
let renderer;
let activeScene = null;
let poses = null;
let socket = null;
let sceneRequest = null;
let requestedGeneration = 0;
let readyGeneration = 0;
let clockReady = false;
let clockNonce = null;
let status = {};
let clientError = '';
let connectionText = '正在连接…';
let sceneText = '正在加载真实场景…';
let trackingText = '桌面观察模式 · 不发送头手输入';
let session = null;
let referenceSpace = null;
let sessionStarting = false;
let sessionStopping = false;
let xrSupported = false;
let nextSequence = 0;
let lastInputTime = -Infinity;
let nextSendAt = 0;
let lastStateAt = 0;
let lastUiAt = 0;
let lastHud = '';
let stateStale = false;
let disposed = false;
const point = new THREE.Vector3();
const rotation = new THREE.Quaternion();
const FRAME_MS = 1000 / 60;

function setError(message) {
  clientError = message;
  updateUi();
}

function socketOpen() {
  return socket?.readyState === WebSocket.OPEN;
}

function authorized() {
  return socketOpen() && clockReady && readyGeneration === requestedGeneration && activeScene?.generation === readyGeneration;
}

function send(message) {
  if (!socketOpen()) return false;
  // Never queue stale hand poses behind a slow transport. Closing invalidates
  // server-side authority; reconnect is a deliberate operator action.
  if (socket.bufferedAmount > 64 * 1024) {
    setError('网络发送阻塞，已断开输入；请检查连接后重新连接。');
    clockReady = false;
    socket.close(1000, 'Input transport backpressure');
    return false;
  }
  socket.send(JSON.stringify(message));
  return true;
}

function inputTime() {
  const now = performance.now();
  lastInputTime = Math.max(now, lastInputTime + 0.001);
  return lastInputTime;
}

function invalidateInput(reason) {
  if (authorized()) {
    send({ type: 'tracking', generation: readyGeneration, sequence: ++nextSequence,
      time_ms: inputTime(), head: null, hands: { left: null, right: null } });
  }
  nextSendAt = 0;
  trackingText = reason;
  updateUi();
}

function command(key) {
  if (!authorized()) {
    setError('场景或输入时钟尚未就绪，操作未发送。');
    return;
  }
  if (session && session.visibilityState !== 'visible') {
    setError('头显不可见时不接受操作；请戴好头显后重试。');
    return;
  }
  if (send({ type: 'command', generation: readyGeneration, key })) {
    clientError = '';
    updateUi();
  }
}

function controlText() {
  if (!Number.isInteger(status.control_flags)) return '等待服务报告控制状态';
  const flags = status.control_flags;
  const right = flags & 2 ? '右手等待稳定手指' : flags & 8 ? '右手平滑接入' : '右手就绪';
  const left = flags & 4 ? '左手等待稳定手指' : flags & 16 ? '左手平滑接入' : '左手就绪';
  return `${flags & 1 ? '双臂输入降级；' : ''}${right}；${left}`;
}

function updateUi() {
  const title = status.task_title || activeScene?.task?.title || '等待任务';
  const goal = status.task_goal || activeScene?.task?.goal || '加载真实场景后显示任务目标。';
  const stage = STAGES[status.stage] || status.stage || '等待服务';
  const checkpoint = status.checkpoint_frames ?? '无';
  const frames = `${status.state_frames ?? 0} / ${checkpoint}`;
  const targetDuration = status.target_duration_s ?? activeScene?.task?.target_duration_s;
  const referenceDuration = Number.isFinite(targetDuration) ? `${targetDuration.toFixed(1)} 秒` : '未报告';
  const recordedDuration = Number.isFinite(status.recorded_duration_s) ? `${status.recorded_duration_s.toFixed(1)} 秒` : '—';
  const duration = `参考时长（论文表2均值）：${referenceDuration} · 已录 ${recordedDuration}`;
  const serverError = status.error ? `服务异常：${status.error}` : '';
  const error = [clientError, serverError].filter(Boolean).join('；');
  ui.connection.textContent = connectionText;
  ui['task-title'].textContent = title;
  ui['task-goal'].textContent = goal;
  ui.stage.textContent = stage;
  ui.tracking.textContent = trackingText;
  ui.frames.textContent = frames;
  ui.duration.textContent = duration;
  ui.control.textContent = controlText();
  ui.notice.textContent = status.notice || '';
  ui.error.textContent = error;
  ui.error.hidden = !error;
  ui['scene-message'].textContent = sceneText;
  ui['scene-message'].hidden = !sceneText;
  const canCommand = authorized() && (!session || session.visibilityState === 'visible');
  for (const button of commandButtons) button.disabled = !canCommand;
  ui['enter-vr'].disabled = sessionStarting || (!session && (!xrSupported || !authorized()));
  ui['enter-vr'].textContent = session ? '退出 VR（立即停止头手输入）' : sessionStarting ? '正在进入 VR…' : xrSupported ? '进入 VR · 启用手部追踪' : '此浏览器无法进入沉浸式 VR';
  drawHud({ title, goal, stage, frames, error, duration });
}

const panelCanvas = document.createElement('canvas');
panelCanvas.width = 1400;
panelCanvas.height = 900;
const panelContext = panelCanvas.getContext('2d');
const panelTexture = new THREE.CanvasTexture(panelCanvas);
panelTexture.colorSpace = THREE.SRGBColorSpace;
panelTexture.minFilter = THREE.LinearFilter;
panelTexture.generateMipmaps = false;
const panelMaterial = new THREE.MeshBasicMaterial({
  map: panelTexture, transparent: true, depthTest: false, depthWrite: false, toneMapped: false,
});
const panel = new THREE.Mesh(new THREE.PlaneGeometry(1.04, 0.67), panelMaterial);
panel.position.set(0, -0.42, -1.35);
panel.renderOrder = 10000;
xrCamera.add(panel);

function drawHud(values) {
  const lines = [
    ['SPD 沉浸式采集', '#9cdef2', 32, 1],
    [values.title, '#ffffff', 48, 2],
    [values.goal, '#d5e4f3', 35, 2],
    [`${connectionText}  |  ${values.stage}`, '#9cdef2', 34, 2],
    [trackingText, trackingText.includes('丢失') || stateStale ? '#ffce91' : '#d5e4f3', 32, 2],
    [`轨迹帧 / 手动检查点：${values.frames}  自动：${status.auto_checkpoint_frames ?? '无'}`, '#d5e4f3', 30, 1],
    [values.duration, '#d5e4f3', 29, 1],
    [controlText(), '#d5e4f3', 30, 1],
    [values.error || status.notice || '进入 VR 不会开始运动；保持双手稳定后按 G。', values.error ? '#ffb6a9' : '#ffdb9b', 31, 2],
    ['踏板  R 检查点   S 暂停 / 恢复   D 回退 / 跳过', '#ffffff', 34, 1],
    ['G 开始   F 保存成功   Q 停止；请按服务提示确认', '#ffffff', 32, 1],
  ];
  const signature = JSON.stringify(lines);
  if (signature === lastHud) return;
  lastHud = signature;
  panelContext.clearRect(0, 0, 1400, 900);
  panelContext.fillStyle = '#101b2aee';
  panelContext.fillRect(0, 0, 1400, 900);
  panelContext.strokeStyle = '#5a9bb1';
  panelContext.lineWidth = 3;
  panelContext.strokeRect(2, 2, 1396, 896);
  let y = 50;
  for (const [text, color, size, maxLines] of lines) {
    panelContext.font = `${size}px system-ui, "Noto Sans CJK SC", sans-serif`;
    panelContext.fillStyle = color;
    let row = '';
    let count = 1;
    for (const character of String(text).replace(/\s+/g, ' ')) {
      if (panelContext.measureText(row + character).width > 1310) {
        if (count === maxLines) {
          row = `${row.slice(0, -1)}…`;
          break;
        }
        panelContext.fillText(row, 42, y);
        y += size * 1.25;
        row = '';
        count++;
      }
      row += character;
    }
    panelContext.fillText(row, 42, y);
    y += size * 1.25 + 14;
  }
  panelTexture.needsUpdate = true;
}

function updateObserver() {
  observer.position.setFromSpherical(observerOffset).add(observerTarget);
  observer.lookAt(observerTarget);
}

function frameObserver() {
  const bounds = activeScene.bounds();
  if (bounds.isEmpty()) return;
  bounds.getCenter(observerTarget);
  const radius = Math.max(0.2, bounds.getSize(point).length() / 2);
  observerOffset.set(radius * 2.8, 1.12, 0.7);
  observerHome = { target: observerTarget.clone(), radius: observerOffset.radius };
  updateObserver();
}

async function changeScene(message) {
  if (!Number.isInteger(message.generation) || message.generation <= 0) throw new Error('场景编号无效');
  if (message.generation === requestedGeneration && sceneRequest) return;
  invalidateInput('正在切换真实场景；等待重新同步');
  clockReady = false;
  clockNonce = null;
  readyGeneration = 0;
  requestedGeneration = message.generation;
  sceneRequest?.abort();
  const request = new AbortController();
  sceneRequest = request;
  const sourceSocket = socket;
  status = {};
  lastStateAt = 0;
  stateStale = false;
  if (activeScene) activeScene.root.visible = false;
  sceneText = '正在加载真实 MuJoCo 场景…';
  updateUi();
  const url = new URL(message.url, location.href);
  if (url.origin !== location.origin || url.pathname !== '/scene') throw new Error('场景 URL 必须来自当前采集服务');
  let loaded = null;
  try {
    const response = await fetch(url, { signal: request.signal, cache: 'no-store' });
    if (!response.ok) throw new Error(`场景导出失败（HTTP ${response.status}）`);
    const description = await response.json();
    if (description.generation !== message.generation) throw new Error('场景代次已变化，请重新连接');
    loaded = await loadScene(description, request.signal);
    if (request.signal.aborted || requestedGeneration !== message.generation || sourceSocket !== socket || !socketOpen()) {
      loaded.dispose();
      return;
    }
    activeScene?.dispose();
    activeScene = loaded;
    scene.add(loaded.root);
    poses = new PoseStream(loaded);
    observerHome = null;
    // A fixed tracking origin is shared by view and input. Never attach the
    // robot/world root to the headset or recenter on a head pose.
    rig.position.copy(loaded.viewPosition).applyQuaternion(WORLD_TO_THREE);
    rig.quaternion.copy(WORLD_TO_THREE).multiply(loaded.viewQuaternion);
    rig.updateMatrixWorld(true);
    readyGeneration = loaded.generation;
    sceneText = '真实场景已加载；等待刚体状态与输入时钟…';
    send({ type: 'ready', generation: readyGeneration });
    updateUi();
  } catch (error) {
    if (request.signal.aborted || sourceSocket !== socket) return;
    setError(error.message);
    sceneText = '场景加载失败；未使用替代或占位场景。';
    updateUi();
    socket.close(1000, 'Scene load failed');
  }
}

function receiveMessage(event) {
  try {
    if (event.data instanceof ArrayBuffer) {
      if (!poses || readyGeneration !== requestedGeneration) return;
      if (poses.accept(event.data, performance.now())) {
        lastStateAt = performance.now();
        if (!observerHome) frameObserver();
        stateStale = false;
        sceneText = '';
      }
      return;
    }
    const message = JSON.parse(event.data);
    if (message.type === 'scene_loading') {
      invalidateInput('场景重建中；输入已暂停');
      clockReady = false;
      clockNonce = null;
      readyGeneration = 0;
      requestedGeneration = message.generation;
      lastStateAt = 0;
      sceneRequest?.abort();
      sceneRequest = null;
      if (activeScene) activeScene.root.visible = false;
      sceneText = '工作站正在重建真实场景…';
      updateUi();
    } else if (message.type === 'scene') {
      changeScene(message).catch((error) => {
        setError(error.message);
        invalidateInput('场景协议错误；已停止输入');
        clockReady = false;
        socket?.close(1000, 'Scene protocol failed');
      });
    } else if (message.type === 'status' && message.generation === requestedGeneration) {
      status = message;
      updateUi();
    } else if (message.type === 'clock' && message.generation === readyGeneration && readyGeneration === requestedGeneration) {
      clockReady = false;
      clockNonce = message.nonce;
      const time = inputTime();
      send({ type: 'clock', generation: readyGeneration, nonce: clockNonce, time_ms: time });
    } else if (message.type === 'clock_ready' && message.generation === readyGeneration && clockNonce !== null) {
      clockReady = true;
      clientError = '';
      connectionText = '已连接 · 场景与时钟就绪';
      if (!session) trackingText = '桌面观察模式 · 不发送头手输入';
      updateUi();
    } else if (message.type === 'error') {
      setError(`服务拒绝：${message.error || message.message || '请求无效'}`);
    }
  } catch (error) {
    invalidateInput('场景数据错误；已停止输入');
    clockReady = false;
    setError(error.message);
    socket?.close(1000, 'Invalid scene data');
  }
}

function connect() {
  if (socket && socket.readyState < WebSocket.CLOSING) return;
  sceneRequest?.abort();
  requestedGeneration = 0;
  readyGeneration = 0;
  nextSequence = 0;
  lastInputTime = -Infinity;
  clockReady = false;
  clockNonce = null;
  connectionText = '正在连接…';
  clientError = '';
  ui.connect.hidden = true;
  const connection = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws`);
  socket = connection;
  connection.binaryType = 'arraybuffer';
  connection.onopen = () => {
    if (socket !== connection) return;
    connectionText = '已连接 · 等待真实场景';
    updateUi();
  };
  connection.onmessage = (event) => { if (socket === connection) receiveMessage(event); };
  connection.onerror = () => { if (socket === connection) setError('连接失败；请检查服务地址、HTTPS / ADB 转发，以及是否已有另一个控制页面。'); };
  connection.onclose = (event) => {
    if (socket !== connection) return;
    sceneRequest?.abort();
    clockReady = false;
    connectionText = `连接已断开${event.reason ? `：${event.reason}` : ''}`;
    sceneText = '连接已断开 · 场景停留在最后一帧 · 不发送控制输入';
    trackingText = '连接丢失；头手输入已失效';
    ui.connect.hidden = false;
    if (activeScene && poses?.latest) activeScene.root.visible = true;
    updateUi();
  };
  updateUi();
}

// view maps XR local-floor -> MuJoCo world. The input consumer also requires
// joint/head LOCAL axes in FLU, unlike raw XR right/up/back. Let B map XR to
// FLU and C=B^-1=WORLD_TO_THREE. Thus p_W=t_view+R_view*p_XR and
// R_W,FLU=R_view*R_XR*C. Default R_view=B gives B*R_XR*B^-1 and a neutral
// head has identity orientation / +X forward. Rendering keeps raw XR axes.
function worldPose(pose) {
  if (!pose || pose.emulatedPosition) return null;
  const { position, orientation } = pose.transform;
  point.set(position.x, position.y, position.z).applyQuaternion(activeScene.viewQuaternion).add(activeScene.viewPosition);
  rotation.set(orientation.x, orientation.y, orientation.z, orientation.w)
    .premultiply(activeScene.viewQuaternion).multiply(WORLD_TO_THREE).normalize();
  return [point.x, point.y, point.z, rotation.x, rotation.y, rotation.z, rotation.w];
}

function sendTracking(now, frame) {
  if (!session || sessionStopping || frame.session !== session || session.visibilityState !== 'visible' || !referenceSpace || !authorized()) return;
  if (now < nextSendAt) return;
  // Advance an accumulated deadline rather than resetting it to now. At
  // 72/90 Hz this samples 60 times per second instead of falling to 36/45.
  // Skip missed periods; never send catch-up bursts.
  if (!nextSendAt) nextSendAt = now;
  nextSendAt += (Math.floor((now - nextSendAt) / FRAME_MS) + 1) * FRAME_MS;
  const viewer = frame.getViewerPose(referenceSpace);
  const head = worldPose(viewer);
  const hands = { left: null, right: null };
  if (head) {
    for (const source of session.inputSources) {
      if (!source.hand || !(source.handedness in hands)) continue;
      hands[source.handedness] = JOINTS.map((name) => {
        const space = source.hand.get(name);
        return space ? worldPose(frame.getJointPose(space, referenceSpace)) : null;
      });
    }
  }
  const left = hands.left?.filter(Boolean).length || 0;
  const right = hands.right?.filter(Boolean).length || 0;
  trackingText = !head ? '头部位置跟踪丢失；双手输入无效' :
    `左手 ${left}/25 · 右手 ${right}/25${!hands.left?.[0] || !hands.right?.[0] ? ' · 手腕跟踪丢失' : ' · local-floor'}`;
  send({ type: 'tracking', generation: readyGeneration, sequence: ++nextSequence,
    time_ms: inputTime(), head, hands });
}

function referenceReset() {
  sessionStopping = true;
  invalidateInput('定位原点已重置；请退出并重新进入 VR，再按 S 重新接入');
  setError('头显重定位使原跟踪坐标失效，已结束本次 VR 会话。');
  referenceSpace?.removeEventListener('reset', referenceReset);
  referenceSpace = null;
  session?.end().catch((error) => setError(`结束重定位会话失败：${error.message}`));
}

function sessionVisibility() {
  if (session?.visibilityState !== 'visible') invalidateInput('头显不可见或系统界面遮挡；输入已失效');
  else {
    trackingText = '头显恢复可见；等待真实手部追踪，按 S 重新接入';
    updateUi();
  }
}

function sessionEnded() {
  invalidateInput('VR 会话已结束；头手输入已失效');
  referenceSpace?.removeEventListener('reset', referenceReset);
  referenceSpace = null;
  session?.removeEventListener('visibilitychange', sessionVisibility);
  session = null;
  sessionStopping = false;
  sessionStarting = false;
  updateUi();
}

async function toggleVr() {
  if (session) {
    sessionStopping = true;
    invalidateInput('正在退出 VR；头手输入已失效');
    await session.end();
    return;
  }
  if (!authorized() || !xrSupported || sessionStarting) return;
  sessionStarting = true;
  updateUi();
  let requested = null;
  try {
    // Both features are mandatory. No controller-only, local-space, inline,
    // DOM-overlay, or synthetic-hand fallback masquerades as collection.
    requested = await navigator.xr.requestSession('immersive-vr', {
      requiredFeatures: ['hand-tracking', 'local-floor'],
    });
    if (!authorized()) throw new Error('进入 VR 时连接或场景已变化，请重新连接');
    session = requested;
    sessionStopping = false;
    session.addEventListener('end', sessionEnded, { once: true });
    session.addEventListener('visibilitychange', sessionVisibility);
    renderer.xr.setReferenceSpaceType('local-floor');
    await renderer.xr.setSession(session);
    if (session !== requested) throw new Error('头显会话已结束');
    referenceSpace = renderer.xr.getReferenceSpace();
    if (!referenceSpace) throw new Error('未取得必需的 local-floor 跟踪空间');
    referenceSpace.addEventListener('reset', referenceReset);
    nextSendAt = 0;
    trackingText = 'VR 已进入；等待双手 25 关节跟踪，按 G 开始';
    clientError = '';
  } catch (error) {
    sessionStopping = true;
    invalidateInput('VR 启动失败；没有发送有效头手输入');
    if (requested) await requested.end().catch(() => {});
    setError(`无法启动手部追踪 VR：${error.message}。请启用 Quest 手部追踪并允许权限。`);
  } finally {
    sessionStarting = false;
    updateUi();
  }
}

async function checkXr() {
  if (!window.isSecureContext) {
    ui['xr-support'].textContent = '当前页面不是安全上下文。Quest 请使用 ADB 转发后的 localhost，或受信任的 HTTPS。';
  } else if (!navigator.xr) {
    ui['xr-support'].textContent = '浏览器未提供 WebXR。桌面可观察真实场景；请使用 Quest 浏览器进入 VR。';
  } else {
    try {
      xrSupported = await navigator.xr.isSessionSupported('immersive-vr');
      ui['xr-support'].textContent = xrSupported ? '沉浸式 VR 可用；进入时必须授权手部追踪与地面定位。' : '未检测到沉浸式头显；仍可使用桌面观察视角。';
    } catch (error) {
      ui['xr-support'].textContent = `WebXR 检测失败：${error.message}`;
    }
  }
  updateUi();
}

function resize() {
  if (!renderer || renderer.xr.isPresenting) return;
  const width = ui.scene.clientWidth;
  const height = ui.scene.clientHeight;
  if (!width || !height) return;
  renderer.setSize(width, height, false);
  observer.aspect = width / height;
  observer.updateProjectionMatrix();
}

function render(now, frame) {
  if (disposed) return;
  try {
    // XR animation timestamps can predict display time; received state samples
    // use performance.now(), so interpolate and age them in that same clock.
    const receivedClock = performance.now();
    poses?.apply(receivedClock);
    if (frame) sendTracking(now, frame);
    if (lastStateAt && receivedClock - lastStateAt > 1000 && !stateStale) {
      stateStale = true;
      sceneText = '服务状态流已停滞 · 画面为最后已知状态';
      invalidateInput('场景状态丢失；请检查服务后重新接入');
      clockReady = false;
      socket?.close(1000, 'Scene state timed out');
    }
    if (now - lastUiAt >= 100) {
      lastUiAt = now;
      updateUi();
    }
    panel.visible = Boolean(session);
    renderer.render(scene, session ? xrCamera : observer);
  } catch (error) {
    invalidateInput('渲染或跟踪发生错误；输入已停止');
    clockReady = false;
    setError(`渲染 / 跟踪失败：${error.message}`);
    socket?.close(1000, 'Rendering or tracking failed');
    renderer.setAnimationLoop(null);
  }
}

for (const button of commandButtons) button.addEventListener('click', () => command(button.dataset.key));
ui.connect.addEventListener('click', connect);
ui['enter-vr'].addEventListener('click', () => toggleVr().catch((error) => setError(error.message)));
window.addEventListener('keydown', (event) => {
  if (event.repeat || event.ctrlKey || event.altKey || event.metaKey || event.isComposing) return;
  if (event.target instanceof HTMLElement && (event.target.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(event.target.tagName))) return;
  const key = event.key.toLowerCase();
  if (!['r', 's', 'd', 'q'].includes(key)) return;
  event.preventDefault();
  command(key);
});
let drag = null;
ui.scene.addEventListener('pointerdown', (event) => {
  if (session || event.button !== 0) return;
  drag = { x: event.clientX, y: event.clientY, id: event.pointerId };
  ui.scene.setPointerCapture(event.pointerId);
});
ui.scene.addEventListener('pointermove', (event) => {
  if (!drag || drag.id !== event.pointerId || session) return;
  observerOffset.theta -= (event.clientX - drag.x) * 0.006;
  observerOffset.phi = THREE.MathUtils.clamp(observerOffset.phi + (event.clientY - drag.y) * 0.006, 0.08, Math.PI - 0.08);
  drag.x = event.clientX;
  drag.y = event.clientY;
  updateObserver();
});
ui.scene.addEventListener('pointerup', () => { drag = null; });
ui.scene.addEventListener('pointercancel', () => { drag = null; });
ui.scene.addEventListener('wheel', (event) => {
  if (session) return;
  event.preventDefault();
  observerOffset.radius = THREE.MathUtils.clamp(observerOffset.radius * Math.exp(event.deltaY * 0.001), 0.2, 80);
  updateObserver();
}, { passive: false });
ui.scene.addEventListener('dblclick', () => {
  if (session || !observerHome) return;
  observerTarget.copy(observerHome.target);
  observerOffset.set(observerHome.radius, 1.12, 0.7);
  updateObserver();
});
window.addEventListener('resize', resize);
document.addEventListener('visibilitychange', () => {
  // An immersive session has its own visibility state; browsers may hide the
  // DOM while that session is visible, so do not confuse it with page focus.
  if (document.hidden && (!session || session.visibilityState !== 'visible')) {
    invalidateInput('页面不可见；输入已失效');
  }
});
ui.scene.addEventListener('webglcontextlost', (event) => {
  event.preventDefault();
  invalidateInput('图形上下文丢失；输入已停止');
  clockReady = false;
  socket?.close(1000, 'WebGL context lost');
  renderer?.setAnimationLoop(null);
  session?.end().catch(() => {});
  setError('WebGL 图形上下文已丢失，请重新加载页面。');
});
window.addEventListener('pagehide', () => {
  disposed = true;
  sessionStopping = true;
  invalidateInput('页面已离开；输入已失效');
  clockReady = false;
  session?.end().catch(() => {});
  sceneRequest?.abort();
  socket?.close(1000, 'Page left');
  renderer?.setAnimationLoop(null);
  activeScene?.dispose();
  panel.geometry.dispose();
  panelMaterial.dispose();
  panelTexture.dispose();
  renderer?.dispose();
});
window.addEventListener('pageshow', (event) => {
  if (event.persisted) location.reload();
});

try {
  renderer = new THREE.WebGLRenderer({ canvas: ui.scene, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.xr.enabled = true;
  renderer.xr.addEventListener('sessionend', resize);
  updateObserver();
  resize();
  renderer.setAnimationLoop(render);
  connect();
  checkXr();
} catch (error) {
  sceneText = '真实三维场景无法启动';
  setError(`WebGL 初始化失败：${error.message}`);
}
