#!/usr/bin/env python

"""Installation script for the 'doosan_ik_pick_place' python package."""

from __future__ import absolute_import
from __future__ import print_function

import os

from setuptools import find_packages, setup

root_dir = os.path.dirname(os.path.realpath(__file__))


# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "numpy",
    "torch>=1.13.1",
]

# Installation operation
setup(
    name="doosan_ik_pick_place",
    version="1.0.0",
    author="Isaac Lab Developers",
    maintainer="Isaac Lab Developers",
    url="https://github.com/isaac-sim/IsaacLab",
    description="E0509 robot manipulation environments for Isaac Lab.",
    long_description=open(os.path.join(root_dir, "README.md"), "r").read(),
    long_description_content_type="text/markdown",
    keywords=["robotics", "rl"],
    license="BSD-3-Clause",
    packages=find_packages("."),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Natural Language :: English",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Science/Topic :: Scientific/Engineering",
    ],
    python_requires=">=3.7",
    install_requires=INSTALL_REQUIRES,
    include_package_data=True,
    zip_safe=False,
)