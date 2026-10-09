import * as THREE from 'three';

export const CAMERA_NAMES = Object.freeze(['top', 'left_wrist', 'right_wrist']);

const CAMERA_DETAILS = Object.freeze({
  top: { color: 0x48b8ff, label: '顶部相机' },
  left_wrist: { color: 0x72d89b, label: '左腕相机' },
  right_wrist: { color: 0xffa65c, label: '右腕相机' },
});

const POSITION_FIELDS = Object.freeze([
  { key: 'x', label: 'X', index: 0, unit: 'm', step: 0.001, defaultLimit: 2 },
  { key: 'y', label: 'Y', index: 1, unit: 'm', step: 0.001, defaultLimit: 2 },
  { key: 'z', label: 'Z', index: 2, unit: 'm', step: 0.001, defaultLimit: 2 },
]);

const ROTATION_FIELDS = Object.freeze([
  { key: 'roll', label: 'Roll', index: 0, unit: '°', step: 0.1, defaultLimit: 180 },
  { key: 'pitch', label: 'Pitch', index: 1, unit: '°', step: 0.1, defaultLimit: 180 },
  { key: 'yaw', label: 'Yaw', index: 2, unit: '°', step: 0.1, defaultLimit: 180 },
]);

const X_AXIS = new THREE.Vector3(1, 0, 0);
const Y_AXIS = new THREE.Vector3(0, 1, 0);
const Z_AXIS = new THREE.Vector3(0, 0, 1);
const DEGREE_TO_RADIAN = Math.PI / 180;

function nonEmptyString(value) {
  return typeof value === 'string' && value.trim() ? value.trim() : '';
}

function isObject(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

function finiteNumber(value) {
  if (typeof value !== 'number' && typeof value !== 'string') {
    return null;
  }
  if (value === '' || (typeof value === 'string' && !value.trim())) {
    return null;
  }
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function requiredPositiveNumber(value, label) {
  const number = finiteNumber(value);
  if (number === null || number <= 0) {
    throw new Error(`${label} 必须是正数。`);
  }
  return number;
}

function requiredVector(value, label) {
  if (!Array.isArray(value) || value.length !== 3) {
    throw new Error(`${label} 必须是三个数值组成的数组。`);
  }
  const vector = value.map(finiteNumber);
  if (vector.some((component) => component === null)) {
    throw new Error(`${label} 含有无效数值。`);
  }
  return vector;
}

function formatInputValue(value) {
  return Number(value).toFixed(9).replace(/\.?0+$/, '') || '0';
}

function formatYamlNumber(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) {
    throw new Error('无法导出非有限相机数值。');
  }
  return number.toFixed(9);
}

function localQuaternionFromRpy(quaternion, rpyDeg, temporaryQuaternion) {
  const roll = rpyDeg[0] * DEGREE_TO_RADIAN;
  const pitch = rpyDeg[1] * DEGREE_TO_RADIAN;
  const yaw = rpyDeg[2] * DEGREE_TO_RADIAN;

  // Keep this explicit: local camera rotation is Rz(yaw) * Ry(pitch) * Rx(roll),
  // not Three's default XYZ Euler order.
  quaternion.setFromAxisAngle(Z_AXIS, yaw);
  temporaryQuaternion.setFromAxisAngle(Y_AXIS, pitch);
  quaternion.multiply(temporaryQuaternion);
  temporaryQuaternion.setFromAxisAngle(X_AXIS, roll);
  quaternion.multiply(temporaryQuaternion);
  return quaternion;
}

function createFrustumGeometry(fovyDeg, aspect) {
  const near = 0.026;
  const far = 0.18;
  const tangent = Math.tan((fovyDeg * DEGREE_TO_RADIAN) / 2);
  const nearHalfHeight = near * tangent;
  const farHalfHeight = far * tangent;
  const nearHalfWidth = nearHalfHeight * aspect;
  const farHalfWidth = farHalfHeight * aspect;
  const corners = [
    [-nearHalfWidth, -nearHalfHeight, -near],
    [nearHalfWidth, -nearHalfHeight, -near],
    [nearHalfWidth, nearHalfHeight, -near],
    [-nearHalfWidth, nearHalfHeight, -near],
    [-farHalfWidth, -farHalfHeight, -far],
    [farHalfWidth, -farHalfHeight, -far],
    [farHalfWidth, farHalfHeight, -far],
    [-farHalfWidth, farHalfHeight, -far],
  ];
  const edges = [
    0, 1, 1, 2, 2, 3, 3, 0,
    4, 5, 5, 6, 6, 7, 7, 4,
    0, 4, 1, 5, 2, 6, 3, 7,
  ];
  const positions = new Float32Array(edges.length * 3);
  for (let index = 0; index < edges.length; index += 1) {
    const source = corners[edges[index]];
    positions[index * 3] = source[0];
    positions[index * 3 + 1] = source[1];
    positions[index * 3 + 2] = source[2];
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  return geometry;
}

function createMarker(color, fovyDeg, aspect) {
  const marker = new THREE.Group();
  marker.name = 'camera-rig-marker';

  const positionDot = new THREE.Mesh(
    new THREE.SphereGeometry(0.027, 12, 8),
    new THREE.MeshBasicMaterial({ color, depthTest: false, depthWrite: false }),
  );
  positionDot.renderOrder = 3;
  marker.add(positionDot);

  const direction = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 0.17, color, 0.052, 0.028);
  direction.line.material.depthTest = false;
  direction.line.material.depthWrite = false;
  direction.cone.material.depthTest = false;
  direction.cone.material.depthWrite = false;
  direction.renderOrder = 3;
  marker.add(direction);

  const axes = new THREE.AxesHelper(0.092);
  axes.traverse((object) => {
    if (object.material) {
      object.material.depthTest = false;
      object.material.depthWrite = false;
    }
    object.renderOrder = 3;
  });
  marker.add(axes);

  const frustum = new THREE.LineSegments(
    createFrustumGeometry(fovyDeg, aspect),
    new THREE.LineBasicMaterial({ color, depthTest: false, depthWrite: false }),
  );
  frustum.renderOrder = 3;
  marker.add(frustum);

  return marker;
}

function disposeMarkerResources(root) {
  if (!root) {
    return;
  }
  const geometries = new Set();
  const materials = new Set();
  root.traverse((object) => {
    if (object.geometry) {
      geometries.add(object.geometry);
    }
    if (Array.isArray(object.material)) {
      object.material.forEach((material) => materials.add(material));
    } else if (object.material) {
      materials.add(object.material);
    }
  });
  for (const geometry of geometries) {
    geometry.dispose();
  }
  for (const material of materials) {
    material.dispose();
  }
}

function yamlKey(key) {
  return /^[A-Za-z_][A-Za-z0-9_-]*$/.test(key) ? key : JSON.stringify(key);
}

function yamlScalar(value) {
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) {
      throw new Error('无法导出非有限 YAML 数值。');
    }
    return String(value);
  }
  if (typeof value === 'boolean') {
    return value ? 'true' : 'false';
  }
  if (value === null) {
    return 'null';
  }
  if (typeof value === 'string') {
    return /^[A-Za-z0-9_./:+-]+$/.test(value) && !/^(?:true|false|null|yes|no|on|off)$/i.test(value)
      ? value
      : JSON.stringify(value);
  }
  throw new Error('相机配置包含无法导出的标量。');
}

function isScalar(value) {
  return value === null || ['string', 'number', 'boolean'].includes(typeof value);
}

function inlineArray(value) {
  return `[${value.map(yamlScalar).join(', ')}]`;
}

function writeYamlValue(lines, value, indent) {
  const padding = ' '.repeat(indent);
  if (isObject(value)) {
    for (const [key, nested] of Object.entries(value)) {
      if (isScalar(nested)) {
        lines.push(`${padding}${yamlKey(key)}: ${yamlScalar(nested)}`);
      } else if (Array.isArray(nested) && nested.every(isScalar)) {
        lines.push(`${padding}${yamlKey(key)}: ${inlineArray(nested)}`);
      } else {
        lines.push(`${padding}${yamlKey(key)}:`);
        writeYamlValue(lines, nested, indent + 2);
      }
    }
    return;
  }
  if (Array.isArray(value)) {
    for (const item of value) {
      if (isScalar(item)) {
        lines.push(`${padding}- ${yamlScalar(item)}`);
      } else if (Array.isArray(item) && item.every(isScalar)) {
        lines.push(`${padding}- ${inlineArray(item)}`);
      } else {
        lines.push(`${padding}-`);
        writeYamlValue(lines, item, indent + 2);
      }
    }
    return;
  }
  lines.push(`${padding}${yamlScalar(value)}`);
}

function buildExportConfiguration(cameraConfig, entries) {
  const output = {};
  for (const [key, value] of Object.entries(cameraConfig)) {
    if (key !== 'cameras') {
      output[key] = value;
    }
  }

  const originalCameras = cameraConfig.cameras;
  output.cameras = {};
  for (const entry of entries) {
    const source = isObject(originalCameras[entry.name]) ? originalCameras[entry.name] : {};
    const camera = {
      parent: entry.parent,
      position: entry.pose.position.map(formatYamlNumber),
      rpy_deg: entry.pose.rpyDeg.map(formatYamlNumber),
    };
    for (const [key, value] of Object.entries(source)) {
      if (!(key in camera)) {
        camera[key] = value;
      }
    }
    output.cameras[entry.name] = camera;
  }
  for (const [name, source] of Object.entries(originalCameras)) {
    if (!(name in output.cameras)) {
      output.cameras[name] = source;
    }
  }
  return output;
}

function yamlForExport(cameraConfig, entries) {
  const output = buildExportConfiguration(cameraConfig, entries);
  const lines = [];
  for (const [key, value] of Object.entries(output)) {
    if (key === 'cameras') {
      lines.push('cameras:');
      for (const [name, camera] of Object.entries(value)) {
        lines.push(`  ${yamlKey(name)}:`);
        const position = camera.position.map((component) => typeof component === 'string' ? component : formatYamlNumber(component));
        const rpyDeg = camera.rpy_deg.map((component) => typeof component === 'string' ? component : formatYamlNumber(component));
        lines.push(`    parent: ${yamlScalar(camera.parent)}`);
        lines.push(`    position: [${position.join(', ')}]`);
        lines.push(`    rpy_deg: [${rpyDeg.join(', ')}]`);
        for (const [cameraKey, cameraValue] of Object.entries(camera)) {
          if (!['parent', 'position', 'rpy_deg'].includes(cameraKey)) {
            if (isScalar(cameraValue)) {
              lines.push(`    ${yamlKey(cameraKey)}: ${yamlScalar(cameraValue)}`);
            } else if (Array.isArray(cameraValue) && cameraValue.every(isScalar)) {
              lines.push(`    ${yamlKey(cameraKey)}: ${inlineArray(cameraValue)}`);
            } else {
              lines.push(`    ${yamlKey(cameraKey)}:`);
              writeYamlValue(lines, cameraValue, 6);
            }
          }
        }
      }
    } else if (isScalar(value)) {
      lines.push(`${yamlKey(key)}: ${yamlScalar(value)}`);
    } else if (Array.isArray(value) && value.every(isScalar)) {
      lines.push(`${yamlKey(key)}: ${inlineArray(value)}`);
    } else {
      lines.push(`${yamlKey(key)}:`);
      writeYamlValue(lines, value, 2);
    }
  }
  return `${lines.join('\n')}\n`;
}

function sortedFields() {
  return [
    { section: '位置（相对父体）', group: 'position', fields: POSITION_FIELDS },
    { section: '姿态（ZYX：Yaw → Pitch → Roll）', group: 'rpyDeg', fields: ROTATION_FIELDS },
  ];
}

export class CameraRig {
  constructor({ scene, editor, status, exportButton, previewGrid, onPoseChanged, onDiagnostics }) {
    this.scene = scene;
    this.editor = editor;
    this.status = status;
    this.exportButton = exportButton;
    this.previewGrid = previewGrid;
    this.onPoseChanged = onPoseChanged;
    this.onDiagnostics = onDiagnostics;
    this.entries = [];
    this.helpersRoot = null;
    this.cameraConfig = null;
    this.bodyNodes = null;
    this._parentPosition = new THREE.Vector3();
    this._parentQuaternion = new THREE.Quaternion();
    this._rotationPart = new THREE.Quaternion();
    this._onExportClick = () => this.downloadYaml();
    this.exportButton.addEventListener('click', this._onExportClick);
    this.clear();
  }

  get isInstalled() {
    return this.entries.length > 0;
  }

  setStatus(message, tone = 'neutral') {
    this.status.textContent = message;
    this.status.dataset.tone = tone;
  }

  clear() {
    if (this.helpersRoot) {
      this.helpersRoot.removeFromParent();
      disposeMarkerResources(this.helpersRoot);
    }
    this.entries = [];
    this.helpersRoot = null;
    this.cameraConfig = null;
    this.bodyNodes = null;
    this.previewGrid.hidden = true;
    this.exportButton.disabled = true;
    this.editor.replaceChildren();
    const empty = document.createElement('p');
    empty.className = 'camera-rig-empty';
    empty.textContent = '加载场景后可查看并调整三台相机的安装位姿。';
    this.editor.append(empty);
    this.setStatus('等待相机配置', 'neutral');
    this.onDiagnostics([]);
  }

  dispose() {
    this.clear();
    this.exportButton.removeEventListener('click', this._onExportClick);
  }

  install({ cameraConfig, cameraMounts, bodyNodes }) {
    this.clear();
    if (!isObject(cameraConfig)) {
      throw new Error('场景未提供 v2 相机配置。');
    }
    if (Number(cameraConfig.version) !== 2) {
      throw new Error('相机配置必须为 version: 2。');
    }
    if (!isObject(cameraConfig.cameras)) {
      throw new Error('相机配置缺少 cameras。');
    }
    if (!Array.isArray(cameraMounts)) {
      throw new Error('场景未提供相机安装父体信息。');
    }
    const mountsByName = new Map();
    for (const mount of cameraMounts) {
      const name = nonEmptyString(mount?.name);
      if (name) {
        mountsByName.set(name, mount);
      }
    }

    const fovyDeg = finiteNumber(cameraConfig.fovy_deg);
    if (fovyDeg === null || fovyDeg <= 0 || fovyDeg >= 180) {
      throw new Error('相机配置的 fovy_deg 无效。');
    }
    const width = requiredPositiveNumber(cameraConfig.width, '相机配置 width');
    const height = requiredPositiveNumber(cameraConfig.height, '相机配置 height');
    const near = requiredPositiveNumber(cameraConfig.near_m, '相机配置 near_m');
    const far = requiredPositiveNumber(cameraConfig.far_m, '相机配置 far_m');
    if (far <= near) {
      throw new Error('相机配置的 far_m 必须大于 near_m。');
    }

    const definitions = [];
    for (const name of CAMERA_NAMES) {
      const configured = cameraConfig.cameras[name];
      if (!isObject(configured)) {
        throw new Error(`相机配置缺少 ${name}。`);
      }
      const parent = nonEmptyString(configured.parent);
      if (!parent) {
        throw new Error(`相机 ${name} 缺少安装父体。`);
      }
      const mount = mountsByName.get(name);
      const parentBodyId = Number(mount?.parent_body_id);
      if (!Number.isInteger(parentBodyId) || parentBodyId <= 0) {
        throw new Error(`相机 ${name} 的安装父体索引无效。`);
      }
      if (!bodyNodes.get(parentBodyId)) {
        throw new Error(`场景缺少相机 ${name} 的安装父体 ${parent}。`);
      }
      definitions.push({
        name,
        parent,
        parentBodyId,
        pose: {
          position: requiredVector(configured.position, `相机 ${name} 的 position`),
          rpyDeg: requiredVector(configured.rpy_deg, `相机 ${name} 的 rpy_deg`),
        },
      });
    }

    const helpersRoot = new THREE.Group();
    helpersRoot.name = 'camera-rig-helpers';
    const entries = [];
    try {
      for (const definition of definitions) {
        const { name, parent, parentBodyId, pose } = definition;
        const detail = CAMERA_DETAILS[name];
        const marker = createMarker(detail.color, fovyDeg, width / height);
        marker.name = `camera-rig-marker-${name}`;
        helpersRoot.add(marker);

        const previewCamera = new THREE.PerspectiveCamera(fovyDeg, width / height, near, far);
        previewCamera.name = `camera-rig-preview-${name}`;
        previewCamera.up.set(0, 1, 0);
        entries.push({
          name,
          label: detail.label,
          color: detail.color,
          parent,
          parentBodyId,
          pose,
          initialPose: {
            position: pose.position.slice(),
            rpyDeg: pose.rpyDeg.slice(),
          },
          marker,
          previewCamera,
          localQuaternion: new THREE.Quaternion(),
          worldPosition: new THREE.Vector3(),
          worldQuaternion: new THREE.Quaternion(),
          controls: new Map(),
          bounds: new Map(),
          diagnostic: {
            name,
            parent,
            parent_body_id: parentBodyId,
            local: {
              position: pose.position.slice(),
              rpy_deg: pose.rpyDeg.slice(),
            },
            world: {
              position: [0, 0, 0],
              quaternion_xyzw: [0, 0, 0, 1],
            },
          },
        });
      }
    } catch (error) {
      disposeMarkerResources(helpersRoot);
      throw error;
    }

    this.entries = entries;
    this.helpersRoot = helpersRoot;
    this.cameraConfig = cameraConfig;
    this.bodyNodes = bodyNodes;
    try {
      this.scene.add(helpersRoot);
      this.previewGrid.hidden = false;
      this.renderEditor();
      this.exportButton.disabled = false;
      this.update(bodyNodes);
      this.setStatus('本地调整即时生效；不会写入源场景或源轨迹。', 'success');
    } catch (error) {
      this.clear();
      throw error;
    }
  }

  getPreviewEntries() {
    return this.entries;
  }

  setHelpersVisible(visible) {
    if (this.helpersRoot) {
      this.helpersRoot.visible = visible;
    }
  }

  update(bodyNodes = this.bodyNodes) {
    if (!this.isInstalled || !bodyNodes) {
      return;
    }
    this.bodyNodes = bodyNodes;
    for (const entry of this.entries) {
      const parentNode = bodyNodes.get(entry.parentBodyId);
      if (!parentNode) {
        continue;
      }
      parentNode.getWorldPosition(this._parentPosition);
      parentNode.getWorldQuaternion(this._parentQuaternion);
      localQuaternionFromRpy(entry.localQuaternion, entry.pose.rpyDeg, this._rotationPart);
      entry.worldPosition.fromArray(entry.pose.position).applyQuaternion(this._parentQuaternion).add(this._parentPosition);
      entry.worldQuaternion.copy(this._parentQuaternion).multiply(entry.localQuaternion);
      entry.marker.position.copy(entry.worldPosition);
      entry.marker.quaternion.copy(entry.worldQuaternion);
      entry.marker.updateMatrixWorld(true);
      entry.previewCamera.position.copy(entry.worldPosition);
      entry.previewCamera.quaternion.copy(entry.worldQuaternion);
      entry.previewCamera.updateMatrixWorld(true);

      entry.diagnostic.local.position.splice(0, 3, ...entry.pose.position);
      entry.diagnostic.local.rpy_deg.splice(0, 3, ...entry.pose.rpyDeg);
      entry.diagnostic.world.position.splice(0, 3, entry.worldPosition.x, entry.worldPosition.y, entry.worldPosition.z);
      entry.diagnostic.world.quaternion_xyzw.splice(
        0,
        4,
        entry.worldQuaternion.x,
        entry.worldQuaternion.y,
        entry.worldQuaternion.z,
        entry.worldQuaternion.w,
      );
    }
    this.onDiagnostics(this.entries.map((entry) => entry.diagnostic));
  }

  renderEditor() {
    const fragment = document.createDocumentFragment();
    for (const entry of this.entries) {
      const card = document.createElement('section');
      card.className = 'camera-rig-card';
      card.style.setProperty('--camera-color', `#${entry.color.toString(16).padStart(6, '0')}`);

      const heading = document.createElement('div');
      heading.className = 'camera-rig-card-heading';
      const title = document.createElement('div');
      const name = document.createElement('h3');
      name.textContent = entry.name;
      const description = document.createElement('p');
      description.textContent = entry.label;
      title.append(name, description);
      const reset = document.createElement('button');
      reset.type = 'button';
      reset.className = 'camera-reset-button';
      reset.textContent = '恢复初始位姿';
      reset.addEventListener('click', () => this.resetEntry(entry));
      heading.append(title, reset);
      card.append(heading);

      const parent = document.createElement('p');
      parent.className = 'camera-parent';
      const parentLabel = document.createElement('span');
      parentLabel.textContent = '安装父体';
      const parentName = document.createElement('code');
      parentName.textContent = entry.parent;
      parent.append(parentLabel, parentName);
      card.append(parent);

      for (const section of sortedFields()) {
        const sectionHeading = document.createElement('p');
        sectionHeading.className = 'camera-pose-section';
        sectionHeading.textContent = section.section;
        card.append(sectionHeading);
        const fields = document.createElement('div');
        fields.className = 'camera-pose-fields';
        for (const field of section.fields) {
          fields.append(this.createPoseControl(entry, section.group, field));
        }
        card.append(fields);
      }
      fragment.append(card);
    }
    this.editor.replaceChildren(fragment);
  }

  createPoseControl(entry, group, field) {
    const row = document.createElement('label');
    row.className = 'camera-pose-control';
    const label = document.createElement('span');
    label.textContent = `${field.label} (${field.unit})`;
    const range = document.createElement('input');
    range.type = 'range';
    range.step = 'any';
    range.setAttribute('aria-label', `${entry.name} ${field.label}`);
    const number = document.createElement('input');
    number.type = 'number';
    number.step = String(field.step);
    number.inputMode = 'decimal';
    number.setAttribute('aria-label', `${entry.name} ${field.label} 数值`);

    const controlKey = `${group}:${field.key}`;
    const initialLimit = Math.max(field.defaultLimit, Math.ceil(Math.abs(entry.pose[group][field.index])));
    entry.bounds.set(controlKey, { min: -initialLimit, max: initialLimit, field });
    entry.controls.set(controlKey, { range, number });
    this.syncControl(entry, group, field);

    range.addEventListener('input', () => {
      const value = finiteNumber(range.value);
      if (value !== null) {
        this.setPoseValue(entry, group, field, value, { syncNumber: true });
      }
    });
    number.addEventListener('input', () => {
      const value = finiteNumber(number.value);
      if (value !== null) {
        this.setPoseValue(entry, group, field, value, { syncNumber: false });
      }
    });
    number.addEventListener('change', () => {
      if (finiteNumber(number.value) === null) {
        this.syncControl(entry, group, field);
        this.setStatus(`${entry.name} 的 ${field.label} 必须是有限数值。`, 'error');
      } else {
        this.syncControl(entry, group, field);
      }
    });
    row.append(label, range, number);
    return row;
  }

  syncControl(entry, group, field) {
    const controlKey = `${group}:${field.key}`;
    const bounds = entry.bounds.get(controlKey);
    const controls = entry.controls.get(controlKey);
    const value = entry.pose[group][field.index];
    controls.range.min = String(bounds.min);
    controls.range.max = String(bounds.max);
    controls.range.value = String(value);
    controls.number.value = formatInputValue(value, field.step);
  }

  syncEntry(entry) {
    for (const section of sortedFields()) {
      for (const field of section.fields) {
        this.syncControl(entry, section.group, field);
      }
    }
  }

  setPoseValue(entry, group, field, value, { syncNumber }) {
    const roundedValue = Math.round(value * 1e9) / 1e9;
    value = Number.isFinite(roundedValue) ? roundedValue : value;
    const controlKey = `${group}:${field.key}`;
    const bounds = entry.bounds.get(controlKey);
    let expanded = false;
    if (value < bounds.min || value > bounds.max) {
      const limit = Math.ceil(Math.max(Math.abs(value), field.defaultLimit));
      bounds.min = -limit;
      bounds.max = limit;
      expanded = true;
    }
    entry.pose[group][field.index] = value;
    const controls = entry.controls.get(controlKey);
    controls.range.min = String(bounds.min);
    controls.range.max = String(bounds.max);
    controls.range.value = String(value);
    if (syncNumber) {
      controls.number.value = formatInputValue(value, field.step);
    }
    this.update();
    this.onPoseChanged();
    if (expanded) {
      this.setStatus(`${entry.name} 的 ${field.label} 超出默认范围，滑条已显式扩展至 ±${bounds.max}${field.unit}。`, 'neutral');
    }
  }

  resetEntry(entry) {
    entry.pose.position.splice(0, 3, ...entry.initialPose.position);
    entry.pose.rpyDeg.splice(0, 3, ...entry.initialPose.rpyDeg);
    this.syncEntry(entry);
    this.update();
    this.onPoseChanged();
    this.setStatus(`${entry.name} 已恢复初始本地位姿。`, 'success');
  }

  downloadYaml() {
    if (!this.isInstalled || !this.cameraConfig) {
      return;
    }
    try {
      const yaml = yamlForExport(this.cameraConfig, this.entries);
      const blob = new Blob([yaml], { type: 'application/x-yaml;charset=utf-8' });
      const url = URL.createObjectURL(blob);
      const anchor = document.createElement('a');
      anchor.href = url;
      anchor.download = 'sim_cameras.v2.yaml';
      document.body.append(anchor);
      anchor.click();
      anchor.remove();
      window.setTimeout(() => URL.revokeObjectURL(url), 1000);
      this.setStatus('已下载完整 v2 相机 YAML；源场景与源轨迹均未修改。', 'success');
    } catch (error) {
      const message = error instanceof Error && error.message ? error.message : '未知导出错误';
      this.setStatus(`相机 YAML 导出失败：${message}`, 'error');
    }
  }
}
