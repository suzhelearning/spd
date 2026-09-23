from setuptools import find_packages, setup

setup(
    name="spd",
    version="0.1.0",
    description="External joint command driven simulation data collection",
    packages=find_packages("src"),
    package_dir={"": "src"},
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
            "spd-model = description.model_compiler.cli:main",
            "spd-scene = simulation.scene:main",
            "spd-collect-trigger = data_collector.trigger:main",
            "spd-render = offline_rendering.cli:main",
            "validate_episode = data_collector.episode:validate_main",
            "replay_episode = data_collector.replay:main",
        ],
    },
)
