from setuptools import find_packages, setup

package_name = "spd_vr"

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
        ("share/" + package_name + "/generated", [
            "generated/tianji_wuji2_spd.xml",
            "generated/joint_manifest.yaml",
            "generated/sim_actuator_calibration.yaml",
        ]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "h5py"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "validate_pico_sample = spd_vr.sample_schema:main",
            "build_model = spd_vr.model_builder:main",
            "validate_scenes = spd_vr.scenes.validate:main",
            "benchmark_sim = spd_vr.simulator:benchmark_main",
            "validate_episode = spd_vr.episode:validate_main",
            "replay_episode = spd_vr.replay:main",
            "align_30hz = spd_vr.align_30hz:main",
            "filter_contacts = spd_vr.filter_contacts:main",
        ],
    },
)
