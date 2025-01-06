# setup.py
from setuptools import setup, find_packages

setup(
    name="teleop-tongs",
    version="0.1.0",
    description="Dexterous teleoperation for general purpose manipulators.",
    author="Arpit Chauhan",
    packages=find_packages(),
    install_requires=[
        "scipy",
        "six",
        "jupyter",
        "filelock",
        "pyserial",
        "transforms3d",
        "pyusb",
        "opencv-python",
        "urchin",
        "XLib",
        "ikpy",
        "feetech-servo-sdk"
    ],
    python_requires="==3.10.12",
)
