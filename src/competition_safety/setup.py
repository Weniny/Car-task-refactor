from setuptools import setup


package_name = "competition_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agilex",
    maintainer_email="agilex@todo.todo",
    description="Independent safety exit for the competition car.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "proximity_stop_node = competition_safety.proximity_stop_node:main",
            "safety_node = competition_safety.safety_node:main",
        ],
    },
)
