from setuptools import setup

package_name = "competition_planning"

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
            "offline_global_plan = competition_planning.offline_global_plan:main",
            "offline_footprint_check = competition_planning.footprint_checker:main",
            "offline_optimized_trajectory = competition_planning.offline_optimized_trajectory:main",
            "offline_continuous_trajectory = competition_planning.offline_continuous_trajectory:main",
            "offline_retime_continuous_trajectory = competition_planning.offline_retime_continuous_trajectory:main",
            "semantic_global_path_node = competition_planning.global_path_node:main",
            "local_replanner_node = competition_planning.local_replanner_node:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agilex",
    maintainer_email="agilex@todo.todo",
    description="Semantic-corridor global planning for the competition car.",
    license="Apache-2.0",
)
