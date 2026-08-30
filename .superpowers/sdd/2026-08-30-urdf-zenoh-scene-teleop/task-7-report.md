# Task 7 报告：URDF-first MuJoCo artifacts

## 交付

- `model_compiler/mjcf.py`：唯一从 `UrdfModel` 递归生成 Full/Arm MJCF；URDF RPY 转 MuJoCo `wxyz`；visual/collision geom 分离；wrist sites；14-DoF arm 投影；完整 contact excludes 与 position actuators。
- `model_compiler/artifacts.py`：消费 Task5 `UrdfModel` 与 Task6 `CollisionArtifact`，生成并 hash 校验五个 artifacts；collision piece 转 MuJoCo 可读 STL；manifest source/output/piece hash 校验；原子 sibling-directory 发布。
- `model_compiler/cli.py`：`spd-model` 编译/验证入口。
- `model_builder.py`：仅转发新 compiler；删除旧 hybrid 产物与 setup data-files 引用。
- `manifest.py`：严格校验 54 joints、14 arm joints、joint/actuator order、ranges、wrist targets 与 output hashes。

## RED/GREEN

初次定向测试在 compiler 模块不存在时收集失败：`ModuleNotFoundError: No module named 'spd_vr.model_compiler.artifacts'`。

实现后运行：

```text
OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_generated_models.py -q
2 passed, 1 warning in 9.55s
```

测试覆盖：Full/Arm MuJoCo 加载与 `nq/nv/nu = 54/14`、wrist sites、24 axis debug visual manifest 记录且不进入 scene、79 adjacent excludes、source mesh hashes、复制 workspace 后相对路径加载、source/XML tamper 拒绝。

## 真实输入质量门

使用 authoritative URDF 运行 CLI 时，Link_Base 经过 Task6 固定约束 `seed=0,max_pieces=16,max_vertices=64` 和碰撞专用 `decimate=True` 后进入 CoACD；按 spec 采用 arm/base `surface_p95_threshold_m=0.003`、hand `0.0015`，阈值写入每条 collision record 并参与 Task6 cache key。未放宽 piece 数/顶点数，未对厂家 visual STL 降采样，也没有 fallback。

真实运行在 Link_Base 生成 16 个 `<=64` 顶点 pieces，但 Task6 的双向表面质量门测得 `surface_p95=0.036785362 m`，超过 arm/base 的 `0.003 m` 阈值，因此 CLI 以 `CollisionError` fail-closed，耗时约 95 秒且未发布半成品目录。该输入不能在不违反固定 piece/vertex/质量约束的前提下生成合法 collision artifact；没有降采样 visual STL、放宽质量门或 fallback。

CLI 真实运行命令：

```bash
OMP_NUM_THREADS=1 pixi run python -m spd_vr.model_compiler.cli \
  --urdf ../assets/tianji_wuji2/tianji_wuji2.urdf \
  --output /tmp/spd-task7-real-v2/generated \
  --cache /tmp/spd-task7-real-v2/collision_cache
```

## Review round 1 修复

- runtime 与 `UnifiedSimulator` 默认消费 `unified_plant.xml` / `model_manifest.yaml`，并通过 `verify_artifacts` 校验 authoritative URDF 与全部发布文件。
- setup 使用递归 `data_files` 安装 generated 下的 XML/YAML、`meshes/` 和 `collision/`；没有 generated 目录时保持可导入；恢复 `validate_pico_sample` 入口。
- verifier 校验每个 `visual_meshes[*].output_file` 的发布字节与 source hash。
- 发布改为拒绝覆盖已存在目录的单次 POSIX `os.replace`，避免公开目录空窗和孤儿 backup。
- 编译入口保存初始 URDF bytes/hash，解析后及发布前检测 TOCTOU；manifest 使用相对 URDF 文件名。
- `arm_joint_order` 必须与 joints 中 `group=arm` 的顺序精确相等且唯一。

修复后定向验证：

```text
OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_generated_models.py -q
2 passed, 1 warning in 9.82s
pixi run python -m py_compile .../setup.py .../artifacts.py .../mjcf.py .../runtime.py .../simulator.py
无输出（通过）
```
