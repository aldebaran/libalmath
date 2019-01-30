#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" QiGeometry Setup """
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import shutil
import datetime
import setuptools


def find_file(filename):
    """ Search a file by name in previous folder and return its path """
    search_folder = os.path.join(PATH_HERE, "..")
    for directory, _folders, content in os.walk(search_folder):
        if "build" not in directory:
            continue
        if filename not in content:
            continue
        return os.path.join(directory, filename)
    return None


PATH_HERE = os.path.abspath(os.path.dirname(__file__))
SETUP_FILES = {
    "manylinux1-x86_64": {
        "libalmath.so": "linux/libalmath.so",
        "libqigeometry.so": "linux/libqigeometry.so",
        "libgeometry_module.so": "linux/lib/libgeometry_module.so",
        "geometry_module.mod": "linux/share/qi/module/geometry_module.mod",
    },
    "macosx-10.12-intel": {
        "libalmath.dylib": "mac/lib/libalmath.dylib",
        "libqigeometry.dylib": "mac/lib/libqigeometry.dylib",
        "libgeometry_module.dylib": "mac/lib/libgeometry_module.dylib",
        "geometry_module.mod": "mac/share/qi/module/geometry_module.mod",
    },
    "win-amd64": {
        "geometry_module.dll": "win/lib/geometry_module.dll",
        "geometry_module.mod": "win/share/qi/module/geometry_module.mod",
    },
}
SETUP_VERSION = datetime.datetime.now().strftime("%y.%m.%d")
SETUP_CONFIG = """\
[metadata]
license_file = LICENSE.txt

[bdist_wheel]
python-tag = cp27
plat-name = %s
plat-tag = %s
"""
SETUP_PLATFORM = "manylinux1-x86_64"
SETUP_SYSTEM = "Operating System :: POSIX :: Linux"
if "darwin" in sys.platform:
    SETUP_PLATFORM = "macosx-10.12-intel"
    SETUP_SYSTEM = "Operating System :: MacOS :: MacOS X"
elif "win" in sys.platform:
    SETUP_PLATFORM = "win-amd64"
    SETUP_SYSTEM = "Operating System :: Microsoft :: Windows"
# Write the Configuration File
with open(os.path.join(PATH_HERE, "setup.cfg"), "w") as file_config:
    file_config.write(SETUP_CONFIG % (SETUP_PLATFORM, SETUP_PLATFORM))
file_config.close()
# Read the Description
LONG_DESCRIPTION = ""
try:
    with open(os.path.join(PATH_HERE, "README.md")) as file_desc:
        LONG_DESCRIPTION = file_desc.read()
    file_desc.close()
except Exception:
    pass
# Search and copy the Files to Include
PLATFORM_FILES = SETUP_FILES.get(SETUP_PLATFORM)
for filename in PLATFORM_FILES:
    source_path = find_file(filename)
    if source_path:
        dest_path = os.path.join(PATH_HERE, "qigeometry", PLATFORM_FILES.get(filename))
        if not os.path.isdir(os.path.dirname(dest_path)):
            os.makedirs(os.path.dirname(dest_path))
        shutil.copy(source_path, dest_path)
        print(" + File %s added to the Package." % filename)
    else:
        print(" ! File %s is missing !" % filename)
# Create the Wheel Package
setuptools.setup(
    name="qigeometry",
    version=SETUP_VERSION,
    description="QiGeometry Python Module",
    long_description=LONG_DESCRIPTION,
    keywords="naoqi softbank nao pepper romeo robot",
    url="http://doc.aldebaran.com",
    author="SoftBank Robotics",
    author_email="jmillet@softbankrobotics.com",
    platforms=SETUP_PLATFORM,
    python_requires=">=2.6, <3",
    packages=["qigeometry"],
    package_dir={"qigeometry": "qigeometry"},
    package_data={
        "qigeometry": [PLATFORM_FILES.get(filename) for filename in PLATFORM_FILES]
    },
    include_package_data=True,
    install_requires=["qi"],
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Embedded Systems",
        "Framework :: Robot Framework :: Tool",
        "Programming Language :: Python :: 2",
        SETUP_SYSTEM,
    ],
)
# Remove the Setup File
os.remove(os.path.join(PATH_HERE, "setup.cfg"))
# Clean QiGeometry Folder
for name in os.listdir(os.path.join(PATH_HERE, "qigeometry")):
    path = os.path.join(PATH_HERE, "qigeometry", name)
    if os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
