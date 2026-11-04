from setuptools import setup


package_name = "competition_control"

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
    description="MPPI trajectory tracking for the competition car.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mppi_control_node = competition_control.mppi_control_node:main",
            "ranger_twist_adapter_node = competition_control.ranger_twist_adapter_node:main",
        ],
    },
)
