from setuptools import setup

package_name = "pitop_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
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
