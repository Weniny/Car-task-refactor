from glob import glob

from setuptools import setup


package_name = "competition_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agilex",
    maintainer_email="agilex@todo.todo",
    description="Wrist-camera flag and traffic-light perception.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wrist_traffic_node = competition_perception.wrist_traffic_node:main",
            "traffic_light_node = competition_perception.traffic_light_node:main",
        ],
    },
)
