import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name='arm_controller',
    version='0.0.1',
    author='CSCI 445 Spring 2021 John Deere Group',
    description='A robot arm controller library',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/jsSwizzle/csci-deere',
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent"
    ],
    packages=[
        "arm_controller",
        "arm_controller.arms",
        "arm_controller.chains",
        "arm_controller.solvers",
        "arm_controller.test"
    ],
    install_requires=[
        "numpy>=1.20.0",
        "scipy",
        "adafruit-circuitpython-servokit",
        "matplotlib",
        "ikpy",
        "roboticstoolbox-python"
    ],
    python_requires=">=3.6"
)
