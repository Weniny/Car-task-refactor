from setuptools import setup

package_name = "competition_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            "fastlio_anchor_node = competition_localization.fastlio_anchor_node:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agilex",
    maintainer_email="agilex@todo.todo",
    description="Localization adapters for the competition car.",
    license="Apache-2.0",
)
