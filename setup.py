from setuptools import setup

package_name = "pitop_ros2"
data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
)
data_files.append(
    ("share/" + package_name + "/launch", ["launch/pitop.launch.py"]),
)
data_files.append(
    ("share/" + package_name + "/launch", ["launch/pitop_rplidar.launch.py"]),
)
data_files.append(("share/" + package_name, ["package.xml"]))


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miquel Massot",
    maintainer_email="miquel.massot@gmail.com",
    description="PiTop ROS2 package",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["pitop_node = pitop_ros2.pitop_node:main"],
    },
)
