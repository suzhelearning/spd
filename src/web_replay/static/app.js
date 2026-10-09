import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { GLTFLoader } from './vendor/GLTFLoader.js';
import { CameraRig } from './camera-rig.js';

const API_ROOT = '/api';
const SOURCE_SAMPLE_RATE = 60;
const MAX_CHUNK_FRAMES = 240;
const MAX_CACHED_CHUNKS = 6;
const PRESENT_INTERVAL_MS = 1000 / 60;

const elements = {
  workspace: document.querySelector('.workspace'),
  catalogPanel: document.getElementById('catalogPanel'),
  applicationTitle: document.getElementById('applicationTitle'),
  headerDescription: document.getElementById('headerDescription'),
  productKicker: document.getElementById('productKicker'),
  directoryForm: document.getElementById('directoryForm'),
  directoryInput: document.getElementById('directoryInput'),
  scanButton: document.getElementById('scanButton'),
  scanStatus: document.getElementById('scanStatus'),
  searchInput: document.getElementById('searchInput'),
  catalogCount: document.getElementById('catalogCount'),
  catalogList: document.getElementById('catalogList'),
  skippedPanel: document.getElementById('skippedPanel'),
  skippedSummary: document.getElementById('skippedSummary'),
  skippedList: document.getElementById('skippedList'),
  viewerModeKicker: document.getElementById('viewerModeKicker'),
  episodeTitle: document.getElementById('episodeTitle'),
  episodePath: document.getElementById('episodePath'),
  sceneStatus: document.getElementById('sceneStatus'),
  canvasStage: document.getElementById('canvasStage'),
  replayCanvas: document.getElementById('replayCanvas'),
  stageMessage: document.getElementById('stageMessage'),
  stageMessageText: document.getElementById('stageMessageText'),
  stageSpinner: document.getElementById('stageSpinner'),
  bufferingIndicator: document.getElementById('bufferingIndicator'),
  cameraPreviewGrid: document.getElementById('cameraPreviewGrid'),
  cameraPreviewSlots: new Map(
    [...document.querySelectorAll('[data-camera-preview]')].map((slot) => [slot.dataset.cameraPreview, slot]),
  ),
  taskValue: document.getElementById('taskValue'),
  frameValue: document.getElementById('frameValue'),
  timeValue: document.getElementById('timeValue'),
  sampleRateValue: document.getElementById('sampleRateValue'),
  fpsValue: document.getElementById('fpsValue'),
  timelineInput: document.getElementById('timelineInput'),
  timelineStart: document.getElementById('timelineStart'),
  timelineEnd: document.getElementById('timelineEnd'),
  restartButton: document.getElementById('restartButton'),
  replayControls: document.getElementById('replayControls'),
  previousButton: document.getElementById('previousButton'),
  playButton: document.getElementById('playButton'),
  nextButton: document.getElementById('nextButton'),
  loopButton: document.getElementById('loopButton'),
  resetCameraButton: document.getElementById('resetCameraButton'),
  cameraRigEditor: document.getElementById('cameraRigEditor'),
  cameraRigStatus: document.getElementById('cameraRigStatus'),
  cameraRigExportButton: document.getElementById('cameraRigExportButton'),
  sceneResetCameraButton: document.getElementById('sceneResetCameraButton'),
};

const publishedReplayState = {
  episodeId: null,
  frame: 0,
  playing: false,
  frames: 0,
  sampleRate: SOURCE_SAMPLE_RATE,
  renderFps: 0,
  buffering: false,
  cameraMounts: [],
  mode: 'replay',
};
window.replayState = publishedReplayState;

const state = {
  catalog: [],
  skipped: [],
  catalogGeneration: 0,
  scanController: null,
  generation: 0,
  selectionController: null,
  episodeId: null,
  selectedSummary: null,
  info: null,
  sceneRoot: null,
  sceneObjects: [],
  bodyNodes: new Map(),
  chunks: new Map(),
  pendingChunks: new Map(),
  cacheStamp: 0,
  frame: 0,
  requestedFrame: 0,
  frames: 0,
  sampleRate: SOURCE_SAMPLE_RATE,
  chunkFrames: MAX_CHUNK_FRAMES,
  poseWidth: 0,
  playing: false,
  buffering: false,
  stopAtEnd: false,
  loop: false,
  playbackAnchorFrame: 0,
  playbackAnchorTime: 0,
  rendererReady: false,
  needsRender: true,
  lastRenderAt: -Infinity,
  fpsWindowStart: 0,
  fpsFrames: 0,
  renderFps: 0,
  lastFpsMeasurement: 0,
  previewViewports: [],
  mode: 'replay',
};

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b1523);
scene.up.set(0, 0, 1);

const camera = new THREE.PerspectiveCamera(46, 1, 0.01, 1000);
camera.up.set(0, 0, 1);
camera.position.set(2.4, -2.4, 1.8);

const fillLight = new THREE.HemisphereLight(0xd6e8ff, 0x182638, 1.65);
scene.add(fillLight);
const keyLight = new THREE.DirectionalLight(0xffffff, 1.35);
keyLight.position.set(3, -4, 5);
scene.add(keyLight);

let renderer = null;
let controls = null;
let gltfLoader = null;
let resizeObserver = null;
let cameraRig = null;
const renderSize = new THREE.Vector2();

function nonEmptyString(value) {
  return typeof value === 'string' && value.trim() ? value.trim() : '';
}

function displayTitle(entry) {
  return nonEmptyString(entry?.title) || nonEmptyString(entry?.relative_path) || nonEmptyString(entry?.id) || '未命名轨迹';
}

function displayTask(entry) {
  return nonEmptyString(entry?.task) || '未标注';
}

function clampFrame(frame) {
  if (state.frames < 1) {
    return 0;
  }
  return Math.min(state.frames - 1, Math.max(0, Math.trunc(frame)));
}

function formatDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) {
    return '—';
  }
  const milliseconds = Math.round(seconds * 1000);
  const minutes = Math.floor(milliseconds / 60000);
  const wholeSeconds = Math.floor((milliseconds % 60000) / 1000);
  const remainder = milliseconds % 1000;
  return `${minutes}:${String(wholeSeconds).padStart(2, '0')}.${String(remainder).padStart(3, '0')}`;
}

function formatSummaryDuration(summary) {
  const duration = Number(summary?.duration);
  if (Number.isFinite(duration) && duration >= 0) {
    return formatDuration(duration);
  }
  const frames = Number(summary?.frames);
  const sampleRate = Number(summary?.sample_rate) || SOURCE_SAMPLE_RATE;
  return Number.isFinite(frames) && frames > 0 ? formatDuration((frames - 1) / sampleRate) : '—';
}

function formatByteSize(bytes) {
  const value = Number(bytes);
  if (!Number.isFinite(value) || value < 0) {
    return '';
  }
  const units = ['B', 'KiB', 'MiB', 'GiB'];
  let unitIndex = 0;
  let amount = value;
  while (amount >= 1024 && unitIndex < units.length - 1) {
    amount /= 1024;
    unitIndex += 1;
  }
  const precision = amount >= 10 || unitIndex === 0 ? 0 : 1;
  return `${amount.toFixed(precision)} ${units[unitIndex]}`;
}

function syncReplayState() {
  publishedReplayState.episodeId = state.episodeId;
  publishedReplayState.frame = state.frame;
  publishedReplayState.playing = state.playing;
  publishedReplayState.frames = state.frames;
  publishedReplayState.sampleRate = state.sampleRate;
  publishedReplayState.renderFps = Math.round(state.renderFps * 10) / 10;
  publishedReplayState.buffering = state.buffering;
  publishedReplayState.mode = state.mode;
}

function setPresentationMode(mode) {
  state.mode = mode;
  const staticScene = mode === 'scene';
  document.title = staticScene ? 'SPD 相机位姿调节' : 'SPD 轨迹回放';
  elements.workspace.classList.toggle('scene-mode', staticScene);
  elements.catalogPanel.hidden = staticScene;
  elements.replayControls.hidden = staticScene;
  elements.sceneResetCameraButton.hidden = !staticScene;
  elements.viewerModeKicker.textContent = staticScene ? '当前场景' : '当前轨迹';
  elements.applicationTitle.textContent = staticScene ? '三维场景相机调节' : '三维轨迹回放';
  elements.productKicker.textContent = staticScene ? 'SPD 静态场景' : 'SPD 数据回放';
  elements.headerDescription.textContent = staticScene
    ? '查看编译后的静态模型并手动调整三台相机'
    : '按源数据 60 Hz 时间轴查看已完成轨迹';
  elements.replayCanvas.setAttribute('aria-label', staticScene ? '静态三维场景' : '三维轨迹场景');
}

function setScanStatus(message, tone = 'neutral') {
  elements.scanStatus.textContent = message;
  elements.scanStatus.dataset.tone = tone;
}

function setSceneStatus(message, tone = 'neutral') {
  elements.sceneStatus.textContent = message;
  elements.sceneStatus.dataset.tone = tone;
}

function showStageMessage(message, mode = 'empty', spinning = false) {
  elements.stageMessage.hidden = false;
  elements.stageMessage.dataset.mode = mode;
  elements.stageMessageText.textContent = message;
  elements.stageSpinner.hidden = !spinning;
  elements.canvasStage.setAttribute('aria-busy', spinning ? 'true' : 'false');
}

function hideStageMessage() {
  elements.stageMessage.hidden = true;
  elements.stageSpinner.hidden = true;
  elements.canvasStage.setAttribute('aria-busy', state.buffering ? 'true' : 'false');
}

function setBuffering(buffering) {
  state.buffering = buffering;
  elements.bufferingIndicator.hidden = !buffering;
  elements.canvasStage.setAttribute('aria-busy', buffering ? 'true' : 'false');
  updateTelemetry();
  syncReplayState();
  updateTransportUI();
}

function updateHeading() {
  const staticScene = state.mode === 'scene';
  if (!state.selectedSummary && !state.info) {
    elements.episodeTitle.textContent = staticScene ? '尚未加载场景' : '尚未选择轨迹';
    elements.episodePath.textContent = staticScene
      ? '正在等待服务端静态模型。'
      : '请从左侧列表选择一条已完成轨迹。';
    return;
  }
  const descriptor = state.info || state.selectedSummary;
  if (staticScene) {
    elements.episodeTitle.textContent = nonEmptyString(descriptor?.title)
      || nonEmptyString(descriptor?.scene)
      || '静态模型';
    const sceneName = nonEmptyString(descriptor?.scene)
      || nonEmptyString(descriptor?.source_path)
      || `场景 ID：${state.episodeId || '—'}`;
    const poseSource = nonEmptyString(descriptor?.pose_source) === 'compiled_qpos0'
      ? '编译模型 qpos0'
      : '静态模型';
    elements.episodePath.textContent = `${sceneName} · ${poseSource}`;
    return;
  }
  elements.episodeTitle.textContent = displayTitle(descriptor);
  elements.episodePath.textContent = nonEmptyString(state.selectedSummary?.relative_path)
    || nonEmptyString(descriptor?.relative_path)
    || `轨迹 ID：${state.episodeId || '—'}`;
}

function updateTelemetry() {
  const descriptor = state.info || state.selectedSummary;
  elements.taskValue.textContent = descriptor ? displayTask(descriptor) : '—';
  elements.sampleRateValue.textContent = `${state.sampleRate} Hz`;

  if (state.frames > 0) {
    const finalFrame = state.frames - 1;
    elements.frameValue.textContent = `${state.frame} / ${finalFrame}`;
    elements.timeValue.textContent = `${formatDuration(state.frame / state.sampleRate)} / ${formatDuration(finalFrame / state.sampleRate)}`;
    elements.timelineStart.textContent = '0:00.000';
    elements.timelineEnd.textContent = formatDuration(finalFrame / state.sampleRate);
    elements.timelineInput.max = String(finalFrame);
    if (!state.buffering) {
      elements.timelineInput.value = String(state.frame);
    }
  } else {
    elements.frameValue.textContent = '—';
    elements.timeValue.textContent = '—';
    elements.timelineStart.textContent = '0:00.000';
    elements.timelineEnd.textContent = '0:00.000';
    elements.timelineInput.max = '0';
    elements.timelineInput.value = '0';
  }

  elements.fpsValue.textContent = state.sceneRoot && state.renderFps > 0
    ? `${state.renderFps.toFixed(1)} fps`
    : '—';
}

function updateTransportUI() {
  const ready = Boolean(state.info && state.sceneRoot && state.frames > 0);
  const finalFrame = Math.max(0, state.frames - 1);
  elements.timelineInput.disabled = !ready;
  elements.restartButton.disabled = !ready;
  elements.previousButton.disabled = !ready || state.frame <= 0;
  elements.nextButton.disabled = !ready || state.frame >= finalFrame;
  elements.playButton.disabled = !ready;
  elements.loopButton.disabled = !ready;
  elements.resetCameraButton.disabled = !ready;
  elements.sceneResetCameraButton.disabled = state.mode !== 'scene' || !state.sceneRoot;
  elements.playButton.textContent = state.playing ? '暂停' : '播放';
  elements.playButton.setAttribute('aria-pressed', String(state.playing));
  elements.loopButton.textContent = `循环：${state.loop ? '开' : '关'}`;
  elements.loopButton.setAttribute('aria-pressed', String(state.loop));
}

function refreshEpisodeUI() {
  updateHeading();
  updateTelemetry();
  updateTransportUI();
}

function renderCatalog() {
  const query = elements.searchInput.value.trim().toLocaleLowerCase();
  const entries = state.catalog.filter((entry) => {
    if (!query) {
      return true;
    }
    return [entry.title, entry.task, entry.scene, entry.relative_path, entry.id]
      .filter((value) => typeof value === 'string')
      .some((value) => value.toLocaleLowerCase().includes(query));
  });

  elements.catalogCount.textContent = `${state.catalog.length} 条`;
  elements.catalogList.replaceChildren();
  if (entries.length === 0) {
    const empty = document.createElement('p');
    empty.className = 'catalog-empty';
    empty.textContent = state.catalog.length === 0
      ? '目录中没有已完成且可回放的轨迹。'
      : '没有与当前筛选条件匹配的轨迹。';
    elements.catalogList.append(empty);
    return;
  }

  for (const entry of entries) {
    const card = document.createElement('button');
    card.type = 'button';
    card.className = 'episode-card';
    card.setAttribute('role', 'option');
    card.setAttribute('aria-selected', String(entry.id === state.episodeId));
    card.addEventListener('click', () => {
      void selectEpisode(entry);
    });

    const title = document.createElement('span');
    title.className = 'episode-title';
    title.textContent = displayTitle(entry);

    const meta = document.createElement('span');
    meta.className = 'episode-meta';
    const frameCount = Number.isFinite(Number(entry.frames)) ? `${entry.frames} 帧` : '帧数未知';
    const size = formatByteSize(entry.size_bytes);
    meta.textContent = [displayTask(entry), frameCount, formatSummaryDuration(entry), size].filter(Boolean).join(' · ');

    const path = document.createElement('span');
    path.className = 'episode-path-small';
    path.textContent = nonEmptyString(entry.relative_path) || entry.id;

    card.append(title, meta, path);
    elements.catalogList.append(card);
  }
}

function renderSkipped() {
  elements.skippedList.replaceChildren();
  const skipped = state.skipped.filter((entry) => entry && typeof entry === 'object');
  elements.skippedPanel.hidden = skipped.length === 0;
  if (skipped.length === 0) {
    return;
  }
  elements.skippedSummary.textContent = `已跳过 ${skipped.length} 个不可用文件`;
  for (const entry of skipped) {
    const item = document.createElement('li');
    const path = document.createElement('span');
    path.className = 'skipped-path';
    path.textContent = nonEmptyString(entry.path) || '未提供路径';
    const error = document.createElement('span');
    error.className = 'skipped-error';
    error.textContent = nonEmptyString(entry.error) || '文件不符合回放要求';
    item.append(path, error);
    elements.skippedList.append(item);
  }
}

async function responseMessage(response) {
  try {
    const payload = await response.json();
    if (payload && typeof payload.error === 'string' && payload.error.trim()) {
      return payload.error;
    }
  } catch {
    // The endpoint contract uses JSON errors, but retain the HTTP status if a proxy does not.
  }
  return `请求失败（HTTP ${response.status}）`;
}

async function requestJson(url, options = {}) {
  const response = await fetch(url, options);
  if (!response.ok) {
    throw new Error(await responseMessage(response));
  }
  try {
    return await response.json();
  } catch {
    throw new Error('服务端返回的数据不是有效 JSON。');
  }
}

async function requestArrayBuffer(url, options = {}) {
  const response = await fetch(url, options);
  if (!response.ok) {
    throw new Error(await responseMessage(response));
  }
  return response.arrayBuffer();
}

function isAbortError(error) {
  return error instanceof DOMException ? error.name === 'AbortError' : error?.name === 'AbortError';
}

function readableError(error) {
  if (error instanceof Error && error.message) {
    return error.message;
  }
  return '发生未知错误。';
}

function normalizeCenter(value) {
  if (Array.isArray(value) && value.length >= 3) {
    const point = value.slice(0, 3).map(Number);
    return point.every(Number.isFinite) ? point : null;
  }
  if (value && typeof value === 'object') {
    if (Number.isFinite(Number(value.x)) && Number.isFinite(Number(value.y)) && Number.isFinite(Number(value.z))) {
      return [Number(value.x), Number(value.y), Number(value.z)];
    }
    for (const key of ['workspace_center', 'center']) {
      const nested = normalizeCenter(value[key]);
      if (nested) {
        return nested;
      }
    }
  }
  return null;
}

function normalizeInfo(raw, expectedId) {
  if (!raw || typeof raw !== 'object') {
    throw new Error('轨迹信息格式无效。');
  }
  const frames = Number(raw.frames);
  const sampleRate = Number(raw.sample_rate);
  if (!Number.isInteger(frames) || frames < 1) {
    throw new Error('轨迹没有可回放的源帧。');
  }
  if (!Number.isFinite(sampleRate) || sampleRate <= 0) {
    throw new Error('轨迹采样率无效。');
  }
  if (!Array.isArray(raw.body_ids) || raw.body_ids.length === 0) {
    throw new Error('轨迹没有可显示的刚体。');
  }
  const bodyIds = raw.body_ids.map(Number);
  if (!bodyIds.every(Number.isInteger) || new Set(bodyIds).size !== bodyIds.length) {
    throw new Error('轨迹刚体索引无效。');
  }
  const chunkFrames = Number(raw.chunk_frames);
  return {
    ...raw,
    id: nonEmptyString(raw.id) || expectedId,
    frames,
    sampleRate,
    bodyIds,
    chunkFrames: Number.isInteger(chunkFrames) && chunkFrames > 0
      ? Math.min(chunkFrames, MAX_CHUNK_FRAMES)
      : MAX_CHUNK_FRAMES,
    center: normalizeCenter(raw.center) || normalizeCenter(raw.workspace_center),
  };
}

function normalizeStaticSceneInfo(raw) {
  if (!raw || typeof raw !== 'object' || raw.mode !== 'scene') {
    throw new Error('静态场景信息格式无效。');
  }
  if (nonEmptyString(raw.id) && raw.id !== 'static-scene') {
    throw new Error('静态场景标识无效。');
  }
  if (!Array.isArray(raw.body_ids) || raw.body_ids.length === 0) {
    throw new Error('静态场景没有可显示的刚体。');
  }
  const bodyIds = raw.body_ids.map(Number);
  if (!bodyIds.every(Number.isInteger) || new Set(bodyIds).size !== bodyIds.length) {
    throw new Error('静态场景刚体索引无效。');
  }
  return {
    ...raw,
    id: 'static-scene',
    mode: 'scene',
    bodyIds,
    center: normalizeCenter(raw.center) || normalizeCenter(raw.workspace_center),
  };
}

function chunkStartForFrame(frame) {
  return Math.floor(frame / state.chunkFrames) * state.chunkFrames;
}

function cachedChunkForFrame(frame) {
  const start = chunkStartForFrame(frame);
  const chunk = state.chunks.get(start);
  if (!chunk || frame < chunk.start || frame >= chunk.start + chunk.count) {
    return null;
  }
  chunk.lastUsed = ++state.cacheStamp;
  return chunk;
}

function trimChunkCache() {
  const protectedStarts = new Set();
  if (state.frames > 0) {
    protectedStarts.add(chunkStartForFrame(state.frame));
    protectedStarts.add(chunkStartForFrame(state.requestedFrame));
  }
  while (state.chunks.size > MAX_CACHED_CHUNKS) {
    let candidate = null;
    for (const chunk of state.chunks.values()) {
      if (protectedStarts.has(chunk.start)) {
        continue;
      }
      if (!candidate || chunk.lastUsed < candidate.lastUsed) {
        candidate = chunk;
      }
    }
    if (!candidate) {
      break;
    }
    state.chunks.delete(candidate.start);
  }
}

function cacheChunk(chunk) {
  const entry = {
    ...chunk,
    lastUsed: ++state.cacheStamp,
  };
  state.chunks.set(entry.start, entry);
  trimChunkCache();
  return entry;
}

async function fetchFrameBlock(episodeId, start, count, info, signal) {
  if (!Number.isInteger(start) || start < 0 || start >= info.frames) {
    throw new Error('请求的帧范围无效。');
  }
  const requestedCount = Math.min(count, info.frames - start, MAX_CHUNK_FRAMES);
  if (!Number.isInteger(requestedCount) || requestedCount < 1) {
    throw new Error('请求的帧数量无效。');
  }

  const parameters = new URLSearchParams({ start: String(start), count: String(requestedCount) });
  const response = await fetch(`${API_ROOT}/episodes/${encodeURIComponent(episodeId)}/frames?${parameters}`, { signal });
  if (!response.ok) {
    throw new Error(await responseMessage(response));
  }

  const responseStartHeader = response.headers.get('X-Frame-Start');
  const responseCountHeader = response.headers.get('X-Frame-Count');
  const responseStart = responseStartHeader === null ? start : Number(responseStartHeader);
  const responseCount = responseCountHeader === null ? NaN : Number(responseCountHeader);
  if (!Number.isInteger(responseStart) || responseStart !== start) {
    throw new Error('服务端返回了不匹配的帧起点。');
  }

  const buffer = await response.arrayBuffer();
  if (buffer.byteLength % Float32Array.BYTES_PER_ELEMENT !== 0) {
    throw new Error('帧数据长度无效。');
  }
  const values = new Float32Array(buffer);
  const poseWidth = info.bodyIds.length * 7;
  const actualCount = Number.isInteger(responseCount) ? responseCount : values.length / poseWidth;
  if (!Number.isInteger(actualCount) || actualCount < 1 || actualCount > requestedCount
      || start + actualCount > info.frames || values.length !== actualCount * poseWidth) {
    throw new Error('服务端返回的帧数据尺寸无效。');
  }
  return { start, count: actualCount, values };
}

function parseGlb(arrayBuffer) {
  return new Promise((resolve, reject) => {
    try {
      gltfLoader.parse(arrayBuffer, '', resolve, reject);
    } catch (error) {
      reject(error);
    }
  });
}

function disposeSceneResources(roots) {
  const rootSet = new Set(roots.filter(Boolean));
  const geometries = new Set();
  const materials = new Set();
  const textures = new Set();

  const collectMaterial = (material) => {
    if (!material || materials.has(material)) {
      return;
    }
    materials.add(material);
    for (const value of Object.values(material)) {
      if (value?.isTexture) {
        textures.add(value);
      }
    }
    if (material.uniforms) {
      for (const uniform of Object.values(material.uniforms)) {
        if (uniform?.value?.isTexture) {
          textures.add(uniform.value);
        }
      }
    }
  };

  for (const root of rootSet) {
    root.traverse((object) => {
      if (object.geometry) {
        geometries.add(object.geometry);
      }
      if (Array.isArray(object.material)) {
        object.material.forEach(collectMaterial);
      } else {
        collectMaterial(object.material);
      }
      if (object.skeleton?.boneTexture) {
        textures.add(object.skeleton.boneTexture);
      }
    });
  }

  for (const geometry of geometries) {
    geometry.dispose();
  }
  for (const material of materials) {
    material.dispose();
  }
  for (const texture of textures) {
    texture.dispose();
  }
  renderer?.renderLists.dispose();
}

function abortEpisodeRequests() {
  if (state.selectionController) {
    state.selectionController.abort();
    state.selectionController = null;
  }
  for (const pending of state.pendingChunks.values()) {
    pending.controller.abort();
  }
  state.pendingChunks.clear();
}

function clearInstalledEpisode() {
  for (const object of new Set(state.sceneObjects)) {
    object.removeFromParent();
  }
  disposeSceneResources(state.sceneObjects);
  cameraRig?.clear();
  state.previewViewports = [];
  publishedReplayState.cameraMounts = [];
  state.sceneRoot = null;
  state.sceneObjects = [];
  state.bodyNodes.clear();
  state.info = null;
  state.chunks.clear();
  state.cacheStamp = 0;
  state.frame = 0;
  state.requestedFrame = 0;
  state.frames = 0;
  state.sampleRate = SOURCE_SAMPLE_RATE;
  state.chunkFrames = MAX_CHUNK_FRAMES;
  state.poseWidth = 0;
  state.playing = false;
  state.buffering = false;
  state.stopAtEnd = false;
  state.playbackAnchorFrame = 0;
  state.playbackAnchorTime = performance.now();
  state.renderFps = 0;
  state.fpsFrames = 0;
  state.fpsWindowStart = 0;
  state.lastFpsMeasurement = 0;
  state.needsRender = true;
  elements.bufferingIndicator.hidden = true;
  syncReplayState();
}

function clearSelection() {
  setPresentationMode('replay');
  state.generation += 1;
  abortEpisodeRequests();
  clearInstalledEpisode();
  state.episodeId = null;
  state.selectedSummary = null;
  showStageMessage('选择一条轨迹后，将加载真实模型与源帧数据。');
  setSceneStatus('等待选择');
  refreshEpisodeUI();
  renderCatalog();
}

function isCurrentSelection(generation, episodeId) {
  return state.generation === generation && state.episodeId === episodeId;
}

function mountModel(gltf, info) {
  const root = gltf?.scene;
  if (!root?.isObject3D) {
    throw new Error('GLB 场景为空。');
  }

  const namedBodies = new Map();
  root.traverse((object) => {
    const match = /^body_(-?\d+)$/.exec(object.name || '');
    if (match && !namedBodies.has(Number(match[1]))) {
      namedBodies.set(Number(match[1]), object);
    }
  });

  const orderedNodes = info.bodyIds.map((bodyId) => namedBodies.get(bodyId));
  if (orderedNodes.some((node) => !node)) {
    const missing = info.bodyIds.filter((bodyId) => !namedBodies.has(bodyId));
    throw new Error(`场景缺少刚体节点：${missing.join(', ')}。`);
  }

  scene.add(root);
  scene.updateMatrixWorld(true);
  try {
    // Replay frames and compiled static scenes use world poses. Detach each body group
    // so later world-pose assignment never multiplies an exported hierarchy.
    for (const node of orderedNodes) {
      scene.attach(node);
      node.matrixAutoUpdate = true;
      node.matrixWorldNeedsUpdate = true;
    }
    scene.updateMatrixWorld(true);
  } catch (error) {
    root.removeFromParent();
    for (const node of orderedNodes) {
      node.removeFromParent();
    }
    disposeSceneResources([root, ...orderedNodes]);
    throw error;
  }

  state.sceneRoot = root;
  state.sceneObjects = [...new Set([root, ...orderedNodes])];
  state.bodyNodes = new Map(info.bodyIds.map((bodyId, index) => [bodyId, orderedNodes[index]]));
  state.needsRender = true;
}

function installCameraRig(info) {
  if (!cameraRig) {
    throw new Error('相机安装编辑器尚未初始化。');
  }
  scene.updateMatrixWorld(true);
  cameraRig.install({
    cameraConfig: info.camera_config,
    cameraMounts: info.camera_mounts,
    bodyNodes: state.bodyNodes,
  });
  updatePreviewViewports();
  state.needsRender = true;
}

function applyFrame(frame) {
  if (state.mode !== 'replay') {
    return false;
  }
  const clamped = clampFrame(frame);
  const chunk = cachedChunkForFrame(clamped);
  if (!chunk) {
    return false;
  }
  const localIndex = clamped - chunk.start;
  const offset = localIndex * state.poseWidth;
  const values = chunk.values;
  for (let index = 0; index < state.info.bodyIds.length; index += 1) {
    const bodyId = state.info.bodyIds[index];
    const node = state.bodyNodes.get(bodyId);
    if (!node) {
      return false;
    }
    const poseOffset = offset + index * 7;
    node.position.set(values[poseOffset], values[poseOffset + 1], values[poseOffset + 2]);
    node.quaternion.set(values[poseOffset + 3], values[poseOffset + 4], values[poseOffset + 5], values[poseOffset + 6]);
    node.matrixAutoUpdate = true;
    node.matrixWorldNeedsUpdate = true;
  }
  scene.updateMatrixWorld(true);
  cameraRig?.update(state.bodyNodes);

  state.frame = clamped;
  state.requestedFrame = clamped;
  state.needsRender = true;
  updateTelemetry();
  updateTransportUI();
  syncReplayState();
  preloadNextChunk(clamped);
  return true;
}

function cancelUnneededChunkRequests(keepStart) {
  for (const [start, pending] of state.pendingChunks) {
    if (start !== keepStart) {
      pending.controller.abort();
      state.pendingChunks.delete(start);
    }
  }
}

function recoverBufferedFrame() {
  if (!state.buffering || !state.info) {
    return;
  }
  const target = clampFrame(state.requestedFrame);
  if (!cachedChunkForFrame(target)) {
    return;
  }
  const shouldStop = state.stopAtEnd;
  state.stopAtEnd = false;
  setBuffering(false);
  if (!applyFrame(target)) {
    return;
  }
  state.playbackAnchorFrame = target;
  state.playbackAnchorTime = performance.now();
  if (shouldStop) {
    pausePlayback({ announce: false });
    setSceneStatus('已到最后一帧', 'success');
  } else {
    setSceneStatus(state.playing ? '正在播放' : '已定位', 'success');
  }
}

function failBufferedFrame(start, error) {
  if (!state.info || chunkStartForFrame(state.requestedFrame) !== start) {
    return;
  }
  state.playing = false;
  state.stopAtEnd = false;
  state.requestedFrame = state.frame;
  setBuffering(false);
  setSceneStatus(`无法读取源帧：${readableError(error)}；可再次定位或重新选择轨迹。`, 'error');
  updateTransportUI();
  syncReplayState();
}

function requestChunk(start, { prefetch = false } = {}) {
  if (state.mode !== 'replay' || !state.info || start < 0 || start >= state.frames) {
    return Promise.resolve(null);
  }
  const cached = state.chunks.get(start);
  if (cached) {
    cached.lastUsed = ++state.cacheStamp;
    return Promise.resolve(cached);
  }

  const existing = state.pendingChunks.get(start);
  if (existing) {
    if (!prefetch) {
      existing.prefetch = false;
    }
    return existing.promise;
  }

  const generation = state.generation;
  const episodeId = state.episodeId;
  const controller = new AbortController();
  const pending = { controller, prefetch, promise: null };
  state.pendingChunks.set(start, pending);
  const count = Math.min(state.chunkFrames, state.frames - start);

  pending.promise = fetchFrameBlock(episodeId, start, count, state.info, controller.signal)
    .then((chunk) => {
      if (!isCurrentSelection(generation, episodeId)) {
        return null;
      }
      const cachedChunk = cacheChunk(chunk);
      recoverBufferedFrame();
      return cachedChunk;
    })
    .catch((error) => {
      if (!isCurrentSelection(generation, episodeId) || isAbortError(error)) {
        return null;
      }
      if (!pending.prefetch) {
        failBufferedFrame(start, error);
      }
      return null;
    })
    .finally(() => {
      if (state.pendingChunks.get(start) === pending) {
        state.pendingChunks.delete(start);
      }
    });

  return pending.promise;
}

function preloadNextChunk(frame) {
  if (state.mode !== 'replay' || !state.info) {
    return;
  }
  const nextStart = chunkStartForFrame(frame) + state.chunkFrames;
  if (nextStart < state.frames) {
    void requestChunk(nextStart, { prefetch: true });
  }
}

function seekTo(frame, { pause = false } = {}) {
  if (state.mode !== 'replay' || !state.info || !state.sceneRoot) {
    return;
  }
  if (pause) {
    pausePlayback({ announce: false, cancelBuffer: true });
  }
  const target = clampFrame(frame);
  state.stopAtEnd = false;
  if (cachedChunkForFrame(target)) {
    setBuffering(false);
    applyFrame(target);
    state.playbackAnchorFrame = target;
    state.playbackAnchorTime = performance.now();
    setSceneStatus(state.playing ? '正在播放' : '已定位', 'success');
    return;
  }

  const start = chunkStartForFrame(target);
  cancelUnneededChunkRequests(start);
  state.requestedFrame = target;
  setBuffering(true);
  elements.timelineInput.value = String(target);
  setSceneStatus(state.playing ? '正在缓冲源帧，时间轴已冻结' : '正在定位并读取源帧', 'loading');
  void requestChunk(start);
}

function pausePlayback({ announce = true, cancelBuffer = false } = {}) {
  state.playing = false;
  state.stopAtEnd = false;
  if (cancelBuffer && state.buffering) {
    const currentStart = state.frames > 0 ? chunkStartForFrame(state.frame) : null;
    cancelUnneededChunkRequests(currentStart);
    state.requestedFrame = state.frame;
    setBuffering(false);
  }
  state.playbackAnchorFrame = state.frame;
  state.playbackAnchorTime = performance.now();
  updateTransportUI();
  syncReplayState();
  if (announce && state.sceneRoot) {
    setSceneStatus('已暂停', 'success');
  }
}

function startPlayback() {
  if (state.mode !== 'replay' || !state.info || !state.sceneRoot) {
    return;
  }
  if (state.frame >= state.frames - 1 && !state.loop && !state.buffering) {
    state.playing = true;
    syncReplayState();
    seekTo(0);
    return;
  }
  state.playing = true;
  state.playbackAnchorFrame = state.frame;
  state.playbackAnchorTime = performance.now();
  updateTransportUI();
  syncReplayState();
  setSceneStatus(state.buffering ? '正在缓冲源帧，时间轴已冻结' : '正在播放', 'loading');
}

function togglePlayback() {
  if (state.playing) {
    pausePlayback({ cancelBuffer: true });
  } else {
    startPlayback();
  }
}

function stepFrame(delta) {
  if (state.mode !== 'replay' || !state.info || !state.sceneRoot) {
    return;
  }
  pausePlayback({ announce: false, cancelBuffer: true });
  const previousFrame = state.frame;
  const target = clampFrame(previousFrame + delta);
  seekTo(target);
  if (target === previousFrame && !state.buffering) {
    setSceneStatus(target === 0 ? '已位于首帧' : '已位于最后一帧', 'success');
  }
}

function advancePlayback(now) {
  if (state.mode !== 'replay' || !state.playing || state.buffering || !state.info || state.frames < 1) {
    return;
  }
  const elapsedFrames = Math.floor(((now - state.playbackAnchorTime) / 1000) * state.sampleRate);
  if (elapsedFrames <= 0) {
    return;
  }

  let target = state.playbackAnchorFrame + elapsedFrames;
  let stopAtEnd = false;
  if (target >= state.frames) {
    if (state.loop) {
      target %= state.frames;
    } else {
      target = state.frames - 1;
      stopAtEnd = true;
    }
  }

  if (target === state.frame) {
    if (stopAtEnd) {
      pausePlayback({ announce: false });
      setSceneStatus('已到最后一帧', 'success');
    }
    return;
  }

  if (!cachedChunkForFrame(target)) {
    const start = chunkStartForFrame(target);
    cancelUnneededChunkRequests(start);
    state.requestedFrame = target;
    state.stopAtEnd = stopAtEnd;
    setBuffering(true);
    setSceneStatus('正在缓冲源帧，时间轴已冻结', 'loading');
    void requestChunk(start);
    return;
  }

  applyFrame(target);
  if (stopAtEnd) {
    pausePlayback({ announce: false });
    setSceneStatus('已到最后一帧', 'success');
  }
}

function resetCamera() {
  if (!state.sceneRoot || !controls) {
    return;
  }
  scene.updateMatrixWorld(true);
  const bounds = new THREE.Box3();
  for (const object of state.sceneObjects) {
    if (object.name === 'body_0') {
      continue;
    }
    bounds.expandByObject(object);
  }
  for (const { marker } of cameraRig.getPreviewEntries()) {
    bounds.expandByObject(marker);
  }

  const sphere = new THREE.Sphere();
  if (!bounds.isEmpty()) {
    bounds.getBoundingSphere(sphere);
  } else {
    sphere.center.set(0, 0, 0);
    sphere.radius = 0.5;
  }

  const target = state.info.center
    ? new THREE.Vector3(...state.info.center)
    : sphere.center.clone();
  const radius = Math.max(0.35, sphere.radius + sphere.center.distanceTo(target));
  const offset = new THREE.Vector3(1.45, -1.45, 1.08).normalize().multiplyScalar(radius * 2.65);
  camera.near = Math.max(0.001, radius / 1200);
  camera.far = Math.max(100, radius * 120);
  camera.position.copy(target).add(offset);
  camera.up.set(0, 0, 1);
  controls.target.copy(target);
  controls.minDistance = Math.max(0.04, radius * 0.08);
  controls.maxDistance = Math.max(10, radius * 45);
  controls.update();
  keyLight.position.copy(target).add(new THREE.Vector3(radius * 2.2, -radius * 2.8, radius * 4.4));
  keyLight.target.position.copy(target);
  scene.add(keyLight.target);
  state.needsRender = true;
}

function configureInfo(info) {
  state.info = info;
  state.frames = info.frames;
  state.sampleRate = info.sampleRate;
  state.chunkFrames = info.chunkFrames;
  state.poseWidth = info.bodyIds.length * 7;
  state.frame = 0;
  state.requestedFrame = 0;
  state.playbackAnchorFrame = 0;
  state.playbackAnchorTime = performance.now();
  refreshEpisodeUI();
  syncReplayState();
}

function configureStaticSceneInfo(info) {
  state.info = info;
  state.frames = 0;
  state.sampleRate = SOURCE_SAMPLE_RATE;
  state.chunkFrames = MAX_CHUNK_FRAMES;
  state.poseWidth = 0;
  state.frame = 0;
  state.requestedFrame = 0;
  state.playing = false;
  state.buffering = false;
  state.stopAtEnd = false;
  state.playbackAnchorFrame = 0;
  state.playbackAnchorTime = performance.now();
  refreshEpisodeUI();
  syncReplayState();
}

async function loadStaticScene(scenePath) {
  const sceneId = 'static-scene';
  setPresentationMode('scene');
  const generation = ++state.generation;
  abortEpisodeRequests();
  clearInstalledEpisode();
  state.episodeId = sceneId;
  state.selectedSummary = {
    id: sceneId,
    title: '正在加载静态场景',
    scene: nonEmptyString(scenePath) || '静态模型',
  };
  updateHeading();
  updateTransportUI();
  syncReplayState();
  showStageMessage('正在加载编译后的静态模型与相机配置…', 'loading', true);
  setSceneStatus('正在加载静态模型', 'loading');

  const controller = new AbortController();
  state.selectionController = controller;
  let parsedScene = null;
  try {
    const [rawInfo, glbBuffer] = await Promise.all([
      requestJson(`${API_ROOT}/scene/info`, { signal: controller.signal }),
      requestArrayBuffer(`${API_ROOT}/scene/scene.glb`, { signal: controller.signal }),
    ]);
    if (!isCurrentSelection(generation, sceneId)) {
      return;
    }

    const info = normalizeStaticSceneInfo(rawInfo);
    configureStaticSceneInfo(info);
    state.episodeId = info.id;
    state.selectedSummary = info;
    updateHeading();
    const parsed = await parseGlb(glbBuffer);
    parsedScene = parsed?.scene || null;
    if (!isCurrentSelection(generation, info.id)) {
      if (parsedScene) {
        disposeSceneResources([parsedScene]);
      }
      return;
    }
    if (!parsedScene) {
      throw new Error('静态场景 GLB 为空。');
    }

    mountModel({ scene: parsedScene }, info);
    parsedScene = null;
    installCameraRig(info);
    state.selectionController = null;
    setBuffering(false);
    resetCamera();
    hideStageMessage();
    setSceneStatus('已加载静态模型，可调整相机位姿', 'success');
    state.needsRender = true;
    refreshEpisodeUI();
    syncReplayState();
  } catch (error) {
    if (!isCurrentSelection(generation, sceneId) || isAbortError(error)) {
      if (parsedScene) {
        disposeSceneResources([parsedScene]);
      }
      return;
    }
    if (parsedScene) {
      disposeSceneResources([parsedScene]);
    }
    state.selectionController = null;
    abortEpisodeRequests();
    clearInstalledEpisode();
    updateHeading();
    refreshEpisodeUI();
    showStageMessage(`无法加载静态模型：${readableError(error)}。请检查场景文件或重新启动服务。`, 'error');
    setSceneStatus('静态模型加载失败', 'error');
  }
}

async function selectEpisode(summary) {
  if (!summary || !nonEmptyString(summary.id)) {
    return;
  }
  setPresentationMode('replay');
  const episodeId = summary.id;
  const generation = ++state.generation;
  abortEpisodeRequests();
  clearInstalledEpisode();
  state.episodeId = episodeId;
  state.selectedSummary = summary;
  updateHeading();
  updateTelemetry();
  updateTransportUI();
  syncReplayState();
  renderCatalog();
  showStageMessage('正在加载真实场景与首段轨迹数据…', 'loading', true);
  setSceneStatus('正在加载轨迹', 'loading');

  const controller = new AbortController();
  state.selectionController = controller;
  let parsedScene = null;
  try {
    const [rawInfo, glbBuffer] = await Promise.all([
      requestJson(`${API_ROOT}/episodes/${encodeURIComponent(episodeId)}/info`, { signal: controller.signal }),
      requestArrayBuffer(`${API_ROOT}/episodes/${encodeURIComponent(episodeId)}/scene.glb`, { signal: controller.signal }),
    ]);
    if (!isCurrentSelection(generation, episodeId)) {
      return;
    }

    const info = normalizeInfo(rawInfo, episodeId);
    configureInfo(info);
    const parsedPromise = parseGlb(glbBuffer);
    const firstBlockPromise = fetchFrameBlock(
      episodeId,
      0,
      Math.min(info.chunkFrames, info.frames),
      info,
      controller.signal,
    );
    const [parsedResult, frameResult] = await Promise.allSettled([parsedPromise, firstBlockPromise]);
    if (parsedResult.status === 'fulfilled') {
      parsedScene = parsedResult.value?.scene || null;
    }
    if (!isCurrentSelection(generation, episodeId)) {
      if (parsedScene) {
        disposeSceneResources([parsedScene]);
      }
      return;
    }
    if (parsedResult.status === 'rejected') {
      throw parsedResult.reason;
    }
    if (frameResult.status === 'rejected') {
      throw frameResult.reason;
    }
    if (!parsedScene) {
      throw new Error('GLB 场景为空。');
    }

    mountModel({ scene: parsedScene }, info);
    parsedScene = null;
    cacheChunk(frameResult.value);
    if (!applyFrame(0)) {
      throw new Error('无法应用首帧姿态。');
    }
    installCameraRig(info);
    state.selectionController = null;
    setBuffering(false);
    resetCamera();
    hideStageMessage();
    setSceneStatus('已加载，可播放', 'success');
    state.needsRender = true;
    refreshEpisodeUI();
    syncReplayState();
  } catch (error) {
    if (!isCurrentSelection(generation, episodeId) || isAbortError(error)) {
      if (parsedScene) {
        disposeSceneResources([parsedScene]);
      }
      return;
    }
    if (parsedScene) {
      disposeSceneResources([parsedScene]);
    }
    state.selectionController = null;
    abortEpisodeRequests();
    clearInstalledEpisode();
    updateHeading();
    refreshEpisodeUI();
    showStageMessage(`无法加载轨迹：${readableError(error)}。请检查目录、文件完整性或重新选择。`, 'error');
    setSceneStatus('加载失败，可重新选择或重新扫描目录', 'error');
  }
}

async function scanCatalog() {
  if (state.mode !== 'replay') {
    return false;
  }
  const directory = elements.directoryInput.value.trim();
  if (!directory) {
    setScanStatus('请输入服务端可访问的轨迹目录。', 'error');
    elements.directoryInput.focus();
    return false;
  }

  const generation = ++state.catalogGeneration;
  state.scanController?.abort();
  const controller = new AbortController();
  state.scanController = controller;
  elements.scanButton.disabled = true;
  setScanStatus('正在扫描目录中的已完成轨迹…', 'loading');

  try {
    const payload = await requestJson(`${API_ROOT}/catalog`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ directory }),
      signal: controller.signal,
    });
    if (generation !== state.catalogGeneration) {
      return false;
    }
    if (!payload || !Array.isArray(payload.episodes)) {
      throw new Error('服务端返回的轨迹目录格式无效。');
    }
    state.catalog = payload.episodes.filter((entry) => entry && typeof entry === 'object' && nonEmptyString(entry.id));
    state.skipped = Array.isArray(payload.skipped) ? payload.skipped : [];
    if (nonEmptyString(payload.directory)) {
      elements.directoryInput.value = payload.directory;
    }

    if (state.episodeId && !state.catalog.some((entry) => entry.id === state.episodeId)) {
      clearSelection();
      setSceneStatus('当前轨迹不在新扫描的目录中', 'neutral');
    } else {
      renderCatalog();
    }
    renderSkipped();
    const skippedText = state.skipped.length ? `；跳过 ${state.skipped.length} 个不可用文件` : '';
    setScanStatus(`扫描完成：找到 ${state.catalog.length} 条可回放轨迹${skippedText}。`, 'success');
    return true;
  } catch (error) {
    if (generation !== state.catalogGeneration || isAbortError(error)) {
      return false;
    }
    setScanStatus(`扫描失败：${readableError(error)}。已保留上一次可用列表。`, 'error');
    return false;
  } finally {
    if (generation === state.catalogGeneration) {
      elements.scanButton.disabled = false;
      state.scanController = null;
    }
  }
}

async function loadConfiguration() {
  try {
    const config = await requestJson(`${API_ROOT}/config`);
    if (config?.mode === 'scene') {
      await loadStaticScene(config.scene_path);
      return;
    }
    setPresentationMode('replay');
    if (!nonEmptyString(config?.directory)) {
      throw new Error('服务端没有提供默认目录。');
    }
    elements.directoryInput.value = config.directory;
    const sampleRate = Number(config.sample_rate);
    if (Number.isFinite(sampleRate) && sampleRate > 0) {
      state.sampleRate = sampleRate;
      updateTelemetry();
      syncReplayState();
    }
    const scanned = await scanCatalog();
    if (!scanned) {
      return;
    }

    const parameters = new URLSearchParams(window.location.search);
    const hasUrlEpisode = parameters.has('episode');
    const requestedEpisode = hasUrlEpisode
      ? nonEmptyString(parameters.get('episode'))
      : nonEmptyString(config.initial_episode);
    if (!requestedEpisode) {
      if (hasUrlEpisode) {
        setScanStatus('URL 的 episode 参数为空，未自动选择其他轨迹。', 'error');
      }
      return;
    }
    const matchedEpisode = state.catalog.find((entry) => (
      entry.id === requestedEpisode || entry.relative_path === requestedEpisode
    ));
    if (!matchedEpisode) {
      setScanStatus(
        `未找到指定轨迹“${requestedEpisode}”（仅接受 ID 或相对路径完全匹配）；未自动选择其他轨迹。`,
        'error',
      );
      return;
    }
    setScanStatus(`已精确匹配指定轨迹：${displayTitle(matchedEpisode)}。正在加载…`, 'loading');
    await selectEpisode(matchedEpisode);
  } catch (error) {
    setScanStatus(`无法读取默认目录：${readableError(error)}。请填写服务端目录后重新扫描。`, 'error');
  }
}

function updatePreviewViewports() {
  state.previewViewports = [];
  if (!renderer || !cameraRig?.isInstalled || elements.cameraPreviewGrid.hidden) {
    return;
  }
  const canvasBounds = elements.replayCanvas.getBoundingClientRect();
  if (canvasBounds.width < 1 || canvasBounds.height < 1) {
    return;
  }
  // Three.js applies the pixel ratio to viewport/scissor coordinates itself.
  renderer.getSize(renderSize);
  const scaleX = renderSize.x / canvasBounds.width;
  const scaleY = renderSize.y / canvasBounds.height;
  for (const entry of cameraRig.getPreviewEntries()) {
    const slot = elements.cameraPreviewSlots.get(entry.name);
    if (!slot) {
      continue;
    }
    const slotBounds = slot.getBoundingClientRect();
    const left = Math.max(canvasBounds.left, slotBounds.left);
    const right = Math.min(canvasBounds.right, slotBounds.right);
    const top = Math.max(canvasBounds.top, slotBounds.top);
    const bottom = Math.min(canvasBounds.bottom, slotBounds.bottom);
    if (right <= left || bottom <= top) {
      continue;
    }
    const x = Math.floor((left - canvasBounds.left) * scaleX);
    const viewportRight = Math.ceil((right - canvasBounds.left) * scaleX);
    const y = Math.floor((canvasBounds.bottom - bottom) * scaleY);
    const viewportTop = Math.ceil((canvasBounds.bottom - top) * scaleY);
    const viewportWidth = viewportRight - x;
    const viewportHeight = viewportTop - y;
    if (viewportWidth > 0 && viewportHeight > 0) {
      state.previewViewports.push({
        entry,
        x,
        y,
        width: viewportWidth,
        height: viewportHeight,
      });
    }
  }
}

function renderSceneWithCameraPreviews() {
  renderer.getSize(renderSize);
  const width = renderSize.x;
  const height = renderSize.y;
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, width, height);
  renderer.clear(true, true, true);
  renderer.render(scene, camera);

  if (!cameraRig?.isInstalled || state.previewViewports.length === 0) {
    return;
  }

  const helpersWereVisible = cameraRig.helpersRoot?.visible ?? true;
  cameraRig.setHelpersVisible(false);
  try {
    renderer.setScissorTest(true);
    for (const viewport of state.previewViewports) {
      const previewCamera = viewport.entry.previewCamera;
      previewCamera.aspect = viewport.width / viewport.height;
      previewCamera.updateProjectionMatrix();
      renderer.setViewport(viewport.x, viewport.y, viewport.width, viewport.height);
      renderer.setScissor(viewport.x, viewport.y, viewport.width, viewport.height);
      renderer.clear(true, true, true);
      renderer.render(scene, previewCamera);
    }
  } finally {
    cameraRig.setHelpersVisible(helpersWereVisible);
    renderer.setScissorTest(false);
    renderer.setViewport(0, 0, width, height);
  }
}

function resizeRenderer() {
  if (!renderer) {
    return;
  }
  const width = Math.floor(elements.canvasStage.clientWidth);
  const height = Math.floor(elements.canvasStage.clientHeight);
  if (width < 1 || height < 1) {
    return;
  }
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 1.5);
  renderer.setPixelRatio(pixelRatio);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  updatePreviewViewports();
  camera.updateProjectionMatrix();
  state.needsRender = true;
}

function initializeRenderer() {
  try {
    renderer = new THREE.WebGLRenderer({
      canvas: elements.replayCanvas,
      antialias: true,
      alpha: false,
      powerPreference: 'high-performance',
    });
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.NoToneMapping;
    renderer.setClearColor(scene.background, 1);
    renderer.autoClear = false;
    renderer.shadowMap.enabled = false;
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = false;
    controls.screenSpacePanning = false;
    controls.target.set(0, 0, 0);
    controls.update();
    controls.addEventListener('change', () => {
      state.needsRender = true;
    });
    gltfLoader = new GLTFLoader();
    state.rendererReady = true;
    resizeRenderer();

    if (typeof ResizeObserver === 'function') {
      resizeObserver = new ResizeObserver(resizeRenderer);
      resizeObserver.observe(elements.canvasStage);
    } else {
      window.addEventListener('resize', resizeRenderer);
    }
    elements.replayCanvas.addEventListener('webglcontextlost', (event) => {
      event.preventDefault();
      state.rendererReady = false;
      pausePlayback({ announce: false, cancelBuffer: true });
      showStageMessage('WebGL 上下文已丢失。请刷新页面后重新加载轨迹。', 'error');
      setSceneStatus('WebGL 上下文已丢失', 'error');
    });
  } catch (error) {
    state.rendererReady = false;
    showStageMessage(`无法初始化三维渲染：${readableError(error)}。请检查浏览器的 WebGL 支持。`, 'error');
    setSceneStatus('三维渲染不可用', 'error');
  }
}

function recordRender(now) {
  if (!state.sceneRoot) {
    return;
  }
  if (state.fpsWindowStart === 0) {
    state.fpsWindowStart = now;
    state.fpsFrames = 0;
  }
  state.fpsFrames += 1;
  const elapsed = now - state.fpsWindowStart;
  if (elapsed >= 500) {
    state.renderFps = (state.fpsFrames * 1000) / elapsed;
    state.lastFpsMeasurement = now;
    state.fpsWindowStart = now;
    state.fpsFrames = 0;
    updateTelemetry();
    syncReplayState();
  }
}

function animationFrame(now) {
  window.requestAnimationFrame(animationFrame);
  if (state.playing && !state.buffering && !document.hidden) {
    advancePlayback(now);
  }

  if (state.renderFps > 0 && !state.playing && now - state.lastFpsMeasurement > 1500) {
    state.renderFps = 0;
    updateTelemetry();
    syncReplayState();
  }

  if (!state.rendererReady || !renderer) {
    return;
  }
  if ((!state.playing || state.buffering) && !state.needsRender) {
    return;
  }
  if (now - state.lastRenderAt < PRESENT_INTERVAL_MS - 0.25) {
    return;
  }

  try {
    renderSceneWithCameraPreviews();
    state.lastRenderAt = now;
    state.needsRender = false;
    recordRender(now);
  } catch (error) {
    state.rendererReady = false;
    pausePlayback({ announce: false, cancelBuffer: true });
    showStageMessage(`三维渲染失败：${readableError(error)}。请刷新页面后重试。`, 'error');
    setSceneStatus('三维渲染失败', 'error');
  }
}

function isEditableTarget(target) {
  if (!(target instanceof Element)) {
    return false;
  }
  return Boolean(target.closest('input, textarea, select, button, [contenteditable="true"]'));
}

function installEvents() {
  elements.directoryForm.addEventListener('submit', (event) => {
    event.preventDefault();
    void scanCatalog();
  });
  elements.searchInput.addEventListener('input', renderCatalog);
  elements.playButton.addEventListener('click', togglePlayback);
  elements.previousButton.addEventListener('click', () => stepFrame(-1));
  elements.nextButton.addEventListener('click', () => stepFrame(1));
  elements.restartButton.addEventListener('click', () => {
    pausePlayback({ announce: false, cancelBuffer: true });
    seekTo(0);
  });
  elements.loopButton.addEventListener('click', () => {
    state.loop = !state.loop;
    updateTransportUI();
  });
  elements.resetCameraButton.addEventListener('click', resetCamera);
  elements.sceneResetCameraButton.addEventListener('click', resetCamera);
  elements.timelineInput.addEventListener('input', () => {
    pausePlayback({ announce: false, cancelBuffer: true });
    seekTo(Number(elements.timelineInput.value));
  });

  document.addEventListener('keydown', (event) => {
    if (event.defaultPrevented || event.isComposing || event.altKey || event.ctrlKey || event.metaKey || isEditableTarget(event.target)) {
      return;
    }
    if (state.mode !== 'replay') {
      return;
    }
    if (event.code === 'Space') {
      event.preventDefault();
      if (!event.repeat) {
        togglePlayback();
      }
    } else if (event.key === 'ArrowLeft') {
      event.preventDefault();
      stepFrame(-1);
    } else if (event.key === 'ArrowRight') {
      event.preventDefault();
      stepFrame(1);
    }
  });

  document.addEventListener('visibilitychange', () => {
    if (document.hidden && state.playing) {
      pausePlayback({ announce: false, cancelBuffer: true });
      setSceneStatus('页面在后台，回放已暂停', 'neutral');
    }
  });

  window.addEventListener('beforeunload', () => {
    state.scanController?.abort();
    abortEpisodeRequests();
    resizeObserver?.disconnect();
    cameraRig?.dispose();
    controls?.dispose();
  });
}

function initializeCameraRig() {
  cameraRig = new CameraRig({
    scene,
    editor: elements.cameraRigEditor,
    status: elements.cameraRigStatus,
    exportButton: elements.cameraRigExportButton,
    previewGrid: elements.cameraPreviewGrid,
    onPoseChanged: () => {
      state.needsRender = true;
    },
    onDiagnostics: (cameraMounts) => {
      publishedReplayState.cameraMounts = cameraMounts;
    },
  });
}

function initialize() {
  initializeCameraRig();
  initializeRenderer();
  installEvents();
  refreshEpisodeUI();
  window.requestAnimationFrame(() => resizeRenderer());
  window.requestAnimationFrame(animationFrame);
  void loadConfiguration();
}

initialize();
