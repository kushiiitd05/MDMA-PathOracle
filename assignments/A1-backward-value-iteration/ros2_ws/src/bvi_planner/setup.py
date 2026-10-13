from glob import glob

from setuptools import setup

PACKAGE = "bvi_planner"

setup(
    name=PACKAGE,
    version="1.0.0",
    # bvi_core is a symlink to the shared pure-Python core (single source of truth).
    packages=[PACKAGE, "bvi_core"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/launch", glob("launch/*.py")),
        (f"share/{PACKAGE}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Kush",
    maintainer_email="kush23296@iiitd.ac.in",
    description="Backward Value Iteration planner with RViz2 visualization",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"planner_node = {PACKAGE}.planner_node:main",
            f"robot_animation_node = {PACKAGE}.robot_animation_node:main",
            f"optimality_checker_node = {PACKAGE}.optimality_checker_node:main",
        ],
    },
)
