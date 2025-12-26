"""
Setup configuration for BonicBot Bridge package
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="bonicbot_bridge",
    version="0.2.1",
    author="Autobonics Pvt Ltd",
    author_email="support@bonic.ai",
    description="Python SDK for educational robotics programming with BonicBot A2",
    long_description="Python bridge library for controlling BonicBot A2 educational robot via ROS2 rosbridge",
    long_description_content_type="text/markdown",
    url="https://github.com/autobonics/bonicbot-bridge",
    project_urls={
        "Bug Tracker": "https://github.com/autobonics/bonicbot-bridge/issues",
        "Documentation": "https://docs.bonic.ai/",
        "Homepage": "https://bonic.ai/",
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Education",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "roslibpy>=1.6.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "black>=22.0",
            "flake8>=5.0",
        ],
        "jupyter": [
            "jupyter>=1.0",
            "matplotlib>=3.5",
            "numpy>=1.20",
        ],
        "camera": [
            "opencv-python>=4.5",
            "numpy>=1.20",
        ],
    },
    entry_points={
        "console_scripts": [
            "bonicbot-test=bonicbot_bridge.test:main",
        ],
    },
    keywords="robotics, education, ros2, blockly, jupyter, stem",
    include_package_data=True,
    zip_safe=False,
)