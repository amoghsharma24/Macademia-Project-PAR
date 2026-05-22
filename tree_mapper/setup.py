from setuptools import setup, find_packages
import os
from glob import glob

package_name = "snc_path_tracker"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
    ],
    zip_safe=True,
    maintainer="YOUR_NAME",
    maintainer_email="YOUR_EMAIL",
    description="Search and Navigation Challenge package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"tree_mapper_node = {package_name}.tree_mapper_node:main",
        ],
    },
)
