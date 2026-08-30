from pathlib import Path

from setuptools import find_packages, setup

package_name = "spd_vr"

generated_data_files: list[tuple[str, list[str]]] = []
generated_root = Path(__file__).resolve().parent / "generated"
if generated_root.is_dir():
    for source in sorted(path for path in generated_root.rglob("*") if path.is_file()):
        relative_parent = source.parent.relative_to(generated_root)
        destination = Path("share") / package_name / "generated" / relative_parent
        generated_data_files.append((str(destination), [str(source)]))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/sim_cameras.yaml",
            "config/wuji2_pico_left.yaml",
            "config/wuji2_pico_right.yaml",
        ]),
        *generated_data_files,
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "PyYAML",
        "h5py",
        "mujoco",
        "scipy",
        "osqp",
        "eclipse-zenoh",
        "trimesh",
        "coacd",
    ],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "spd-model = spd_vr.model_compiler.cli:main",
            "spd-pxrea-bridge = spd_vr.pxrea_bridge:main",
            "spd-arm-ik = spd_vr.arm_ik:main",
            "spd-viewer = spd_vr.viewer:main",
            "spd-preflight = spd_vr.preflight:main",
            "spd-control = spd_vr.control_cli:main",
            "spd-status = spd_vr.status_cli:main",
            "validate_pico_sample = spd_vr.sample_schema:main",
            "validate_scenes = spd_vr.scenes.validate:main",
            "benchmark_sim = spd_vr.simulator:benchmark_main",
            "validate_episode = spd_vr.episode:validate_main",
            "replay_episode = spd_vr.replay:main",
            "align_30hz = spd_vr.align_30hz:main",
            "filter_contacts = spd_vr.filter_contacts:main",
        ],
    },
)
