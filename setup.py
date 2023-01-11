import ctre
import wpilib
import rticonnextdds_connector

from setuptools import setup, find_packages

import os
import compileall
compileall.compile_dir("rio")

INSTALL_REQUIREMENTS = [
    "wpilib=2022.4.1.6",
    "rticonnextdds_connector="
]