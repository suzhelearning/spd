from setuptools import find_packages, setup

setup(
    name="spd-vr",
    version="0.1.0",
    description="External joint command driven simulation data collection",
    packages=find_packages(exclude=["test"]),
    install_requires=[
        "numpy",
        "PyYAML",
        "Pillow",
        "h5py",
        "mujoco>=3.12,<3.13",
        "scipy",
        "trimesh",
        "coacd",
        "spd-envs",
    ],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "spd-model = spd_vr.description.model_compiler.cli:main",
            "spd-viewer = spd_vr.simulation.ros_viewer:main",
            "spd-scene = spd_vr.simulation.scene:main",
            "spd-collect-trigger = spd_vr.data_collector.trigger:main",
            "validate_episode = spd_vr.data_collector.episode:validate_main",
            "replay_episode = spd_vr.data_collector.replay:main",
            "align_30hz = spd_vr.data_collector.align_30hz:main",
            "filter_contacts = spd_vr.data_collector.filter_contacts:main",
        ],
    },
)
