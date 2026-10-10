import * as THREE from './vendor/three/three.module.min.js';

// MuJoCo FLU/Z-up -> Three right/up/back: C(x,y,z) = (-y,z,-x).
export const WORLD_TO_THREE = new THREE.Quaternion().setFromRotationMatrix(
  new THREE.Matrix4().set(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1),
);

function imageTexture(url, signal, colorSpace) {
  return new Promise((resolve, reject) => {
    if (!/^data:image\/(png|jpeg|webp);base64,/.test(url)) {
      reject(new Error('场景纹理必须是本地导出的 PNG / JPEG / WebP 图像'));
      return;
    }
    const image = new Image();
    const cleanup = () => {
      signal.removeEventListener('abort', abort);
      image.onload = null;
      image.onerror = null;
    };
    const abort = () => {
      cleanup();
      image.src = '';
      reject(new DOMException('场景加载已取消', 'AbortError'));
    };
    image.onload = () => {
      cleanup();
      const texture = new THREE.Texture(image);
      texture.colorSpace = colorSpace === 'linear' ? THREE.LinearSRGBColorSpace : THREE.SRGBColorSpace;
      // Export preserves MuJoCo tex_data row order; do not apply Three's
      // usual HTML-image Y flip on top of that OpenGL-native ordering.
      texture.flipY = false;
      texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
      texture.needsUpdate = true;
      resolve(texture);
    };
    image.onerror = () => {
      cleanup();
      reject(new Error('场景纹理解码失败'));
    };
    signal.addEventListener('abort', abort, { once: true });
    if (signal.aborted) abort();
    else image.src = url;
  });
}

function primitive(type, size) {
  const [x, y, z] = size;
  switch (type) {
    case 'box': return new THREE.BoxGeometry(2 * x, 2 * y, 2 * z);
    case 'sphere': return new THREE.SphereGeometry(x, 24, 16);
    case 'ellipsoid': return new THREE.SphereGeometry(1, 24, 16).scale(x, y, z);
    case 'capsule': return new THREE.CapsuleGeometry(x, 2 * y, 8, 20).rotateX(Math.PI / 2);
    case 'cylinder': return new THREE.CylinderGeometry(x, x, 2 * y, 24).rotateX(Math.PI / 2);
    // MuJoCo zero-sized planes are infinite. Render a 100 m patch of that
    // actual plane; exclude it from automatic desktop scene framing.
    case 'plane': return new THREE.PlaneGeometry(2 * (x || 50), 2 * (y || 50));
    default: throw new Error(`不支持的真实场景几何：${type}`);
  }
}

export async function loadScene(description, signal) {
  if (description.version !== 1 || !Number.isInteger(description.generation)) {
    throw new Error('不支持的场景协议版本或场景编号');
  }
  const geometries = new Set();
  const materials = new Set();
  const textures = new Set();
  const root = new THREE.Group();
  root.quaternion.copy(WORLD_TO_THREE);
  root.visible = false;
  const bodies = [];
  const byBody = new Map();
  const meshGeometry = new Map();
  const textureById = new Map();
  const materialDescription = new Map(description.materials.map((item) => [item.id, item]));
  const materialCache = new Map();
  const textureCache = new Map();
  const shapeCache = new Map();
  const framedMeshes = [];
  const dispose = () => {
    root.removeFromParent();
    for (const resource of [...geometries, ...materials, ...textures]) resource.dispose();
    geometries.clear();
    materials.clear();
    textures.clear();
  };
  try {
    // Each load owns its textures. Aborted/failed loads never publish objects
    // into the live scene, and all already-created GPU resources are disposed.
    for (const source of description.textures) {
      const texture = await imageTexture(source.data_url, signal, source.color_space);
      textures.add(texture);
      textureById.set(source.id, texture);
    }
    signal.throwIfAborted();
    for (const source of description.meshes) {
      const geometry = new THREE.BufferGeometry();
      geometries.add(geometry);
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(source.positions, 3));
      geometry.setIndex(source.indices);
      if (source.uvs) geometry.setAttribute('uv', new THREE.Float32BufferAttribute(source.uvs, 2));
      if (source.normals) geometry.setAttribute('normal', new THREE.Float32BufferAttribute(source.normals, 3));
      else geometry.computeVertexNormals();
      meshGeometry.set(source.id, geometry);
    }
    for (const source of description.bodies) {
      if (byBody.has(source.id)) throw new Error('场景刚体编号重复');
      const body = new THREE.Group();
      body.name = source.name;
      root.add(body);
      byBody.set(source.id, body);
      bodies.push(body);
    }
    for (const source of description.geoms) {
      const body = byBody.get(source.body_id);
      if (!body) throw new Error(`缺少场景刚体 ${source.body_id}`);
      let geometry;
      if (source.type === 'mesh') {
        geometry = meshGeometry.get(source.mesh_id);
        if (!geometry) throw new Error(`缺少真实网格 ${source.mesh_id}`);
      } else {
        const shapeKey = `${source.type}:${source.size.join(',')}`;
        geometry = shapeCache.get(shapeKey);
        if (!geometry) {
          geometry = primitive(source.type, source.size);
          shapeCache.set(shapeKey, geometry);
          geometries.add(geometry);
        }
      }
      const materialSource = materialDescription.get(source.material_id);
      const rgba = source.rgba;
      const materialKey = `${source.material_id}:${rgba.join(',')}`;
      let material = materialCache.get(materialKey);
      if (!material) {
        let map = null;
        if (materialSource?.texture_id != null) {
          const repeat = materialSource.texrepeat || [1, 1];
          const textureKey = `${materialSource.texture_id}:${repeat.join(',')}`;
          map = textureCache.get(textureKey);
          if (!map) {
            const original = textureById.get(materialSource.texture_id);
            if (!original) throw new Error(`缺少材质纹理 ${materialSource.texture_id}`);
            map = original.clone();
            map.repeat.fromArray(repeat);
            map.needsUpdate = true;
            textures.add(map);
            textureCache.set(textureKey, map);
          }
        }
        material = new THREE.MeshStandardMaterial({
          color: new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.LinearSRGBColorSpace),
          opacity: rgba[3], transparent: rgba[3] < 1, depthWrite: rgba[3] >= 1,
          roughness: 0.7, metalness: 0, map, side: THREE.DoubleSide,
        });
        materials.add(material);
        materialCache.set(materialKey, material);
      }
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.fromArray(source.position);
      mesh.quaternion.fromArray(source.quaternion).normalize();
      mesh.visible = rgba[3] > 0;
      body.add(mesh);
      if (source.type !== 'plane' && mesh.visible) framedMeshes.push(mesh);
    }
    const viewPosition = new THREE.Vector3().fromArray(description.view.position);
    const viewQuaternion = new THREE.Quaternion().fromArray(description.view.quaternion).normalize();
    const result = {
      root, bodies, viewPosition, viewQuaternion, task: description.task,
      generation: description.generation, dispose,
      bounds() {
        root.updateMatrixWorld(true);
        const bounds = new THREE.Box3();
        const local = new THREE.Box3();
        for (const mesh of framedMeshes) {
          if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox();
          local.copy(mesh.geometry.boundingBox).applyMatrix4(mesh.matrixWorld);
          bounds.union(local);
        }
        return bounds;
      },
    };
    return result;
  } catch (error) {
    dispose();
    throw error;
  }
}

export class PoseStream {
  constructor(scene) {
    this.scene = scene;
    this.previous = null;
    this.latest = null;
    this.q0 = new THREE.Quaternion();
    this.q1 = new THREE.Quaternion();
  }

  accept(buffer, receivedAt) {
    const header = new DataView(buffer);
    if (buffer.byteLength < 24 || header.getUint32(0, true) !== 0x53445053) {
      throw new Error('场景状态数据头无效');
    }
    const generation = header.getUint32(4, true);
    const sequence = header.getUint32(8, true);
    const count = header.getUint32(12, true);
    if (generation !== this.scene.generation) return false;
    if (count !== this.scene.bodies.length || buffer.byteLength !== 24 + count * 28) {
      throw new Error('场景状态刚体数量与导出场景不一致');
    }
    const time = header.getFloat64(16, true);
    if (!Number.isFinite(time)) throw new Error('场景仿真时间无效');
    if (this.latest && ((sequence - this.latest.sequence) >>> 0) >= 0x80000000) return false;
    if (this.latest?.sequence === sequence) return false;
    const values = new Float32Array(count * 7);
    for (let i = 0; i < values.length; i++) {
      const value = header.getFloat32(24 + i * 4, true);
      if (!Number.isFinite(value)) throw new Error('场景刚体姿态包含非有限数值');
      values[i] = value;
    }
    this.previous = this.latest;
    this.latest = { values, time, receivedAt, sequence };
    // A rewind or a long gap must never interpolate through a discontinuity.
    if (!this.previous || time < this.previous.time || receivedAt - this.previous.receivedAt > 150) {
      this.previous = this.latest;
    }
    this.apply(receivedAt);
    this.scene.root.visible = true;
    return true;
  }

  apply(now) {
    if (!this.latest) return;
    const older = this.previous;
    const newer = this.latest;
    // One measured packet interval of delay, bounded to 50 ms. Never predict
    // beyond authoritative physics; sparse streams hold their last pose.
    const interval = Math.min(50, Math.max(1, newer.receivedAt - older.receivedAt));
    const alpha = older === newer ? 1 : THREE.MathUtils.clamp((now - newer.receivedAt) / interval, 0, 1);
    for (let i = 0; i < this.scene.bodies.length; i++) {
      const body = this.scene.bodies[i];
      const offset = i * 7;
      const a = older.values;
      const b = newer.values;
      body.position.set(
        THREE.MathUtils.lerp(a[offset], b[offset], alpha),
        THREE.MathUtils.lerp(a[offset + 1], b[offset + 1], alpha),
        THREE.MathUtils.lerp(a[offset + 2], b[offset + 2], alpha),
      );
      this.q0.fromArray(a, offset + 3).normalize();
      this.q1.fromArray(b, offset + 3).normalize();
      body.quaternion.slerpQuaternions(this.q0, this.q1, alpha);
    }
  }
}
