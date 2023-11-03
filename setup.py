from glob import glob

from setuptools import setup

package_name = "detector_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
        (  # get all .pt files in weights folder
            f"share/{package_name}/weights",
            glob(f"{package_name}/weights/*.pt"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="someone",
    maintainer_email="someone@todo.todo",
    description="Detector package",
    license="AGPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"object_bbox_node = {package_name}.object_bbox_node:main",
        ],
    },
)
