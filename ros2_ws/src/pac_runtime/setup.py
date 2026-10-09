from setuptools import setup

package_name = "pac_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/runtime.launch.py", "launch/gazebo_cell.launch.py",
                                             "launch/merged_algorithm.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    entry_points={"console_scripts": ["runtime_node = pac_runtime.ros_node:main",
                                    "gazebo_cell = pac_runtime.gazebo_driver:main"]},
    zip_safe=True,
    maintainer="taehyeon",
    maintainer_email="taehyeon@example.com",
    description="AHEAD runtime stages 1-3, 7-8 and the 1-8 loop",
    license="Proprietary",
)
