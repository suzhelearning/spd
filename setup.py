from setuptools import find_packages, setup

setup(
    name="spd",
    version="0.1.0",
    description="WebXR dexterous simulation collection and replay",
    packages=find_packages("src"),
    package_dir={"": "src"},
    package_data={
        "web_replay": ["static/*", "static/vendor/*"],
        "webxr": ["static/*", "static/vendor/three/*"],
    },
    install_requires=[
        "numpy",
        "PyYAML",
        "Pillow",
        "h5py",
        "mujoco>=3.12,<3.13",
        "glfw>=2.10,<3",
        "scipy",
        "trimesh",
        "coacd",
        "spd-envs",
        "aiohttp>=3.11,<4",
    ],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "spd-model = description.model_compiler.cli:main",
            "spd-scene = simulation.scene:main",
            "spd-collect-trigger = data_collector.trigger:main",
            "validate_episode = data_collector.episode:validate_main",
            "replay_episode = data_collector.replay:main",
            "spd-web = web_replay.cli:main",
        ],
    },
)
