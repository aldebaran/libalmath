#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" ALMath Setup """
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import shutil
import datetime
import setuptools

SETUP_PROJECT = "almath"
SETUP_DESCRIPTION = "ALMath is an optimized mathematic toolbox for robotics."
SETUP_FILES = {
    "manylinux1-x86_64": {
        "almathswig.py": "almathswig.py",
        "libalmath.so": "libalmath.so",
        "_almathswig.so": "_almathswig.so",
    },
    "macosx-10.12-intel": {
        "almathswig.py": "almathswig.py",
        "libalmath.dylib": "libalmath.dylib",
        "_almathswig.so": "python2.7/site-packages/_almathswig.so",
    },
    "win-amd64": {
        "almathswig.py": "almathswig.py",
        "almath.dll": "almath.dll",
        "_almathswig.pyd": "_almathswig.pyd",
    },
}
PATH_HERE = os.path.abspath(os.path.dirname(__file__))


def find_file(filename):
    """ Search a file by name in previous folder and return its path """
    search_folder = os.environ.get("QIPYTHON_BUILD_FOLDER")
    if not search_folder or not os.path.isdir(search_folder):
        search_folder = os.path.join(PATH_HERE, "..")
    for directory, _folders, content in os.walk(search_folder):
        if filename not in content:
            continue
        return os.path.join(directory, filename)
    return None


DEFAULT_VERSION = datetime.datetime.now().strftime("%y.%m.%d")
SETUP_VERSION = os.environ.get("QIPYTHON_BUILD_VERSION", DEFAULT_VERSION)
SETUP_CONFIG = """\
[metadata]\nlicense_file = LICENSE.txt\n
[bdist_wheel]\npython-tag = cp27\nplat-name = %s\nplat-tag = %s
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
        dest_path = os.path.join(PATH_HERE, SETUP_PROJECT, PLATFORM_FILES.get(filename))
        if not os.path.isdir(os.path.dirname(dest_path)):
            os.makedirs(os.path.dirname(dest_path))
        shutil.copy(source_path, dest_path)
        print(" + File %s added to the Package." % filename)
    else:
        print(" ! File %s is missing !" % filename)
# Create the Wheel Package
setuptools.setup(
    name=SETUP_PROJECT,
    version=SETUP_VERSION,
    description=SETUP_DESCRIPTION,
    long_description=LONG_DESCRIPTION,
    keywords="naoqi softbank nao pepper romeo robot",
    url="http://doc.aldebaran.com",
    author="SoftBank Robotics",
    author_email="release@softbankrobotics.com",
    platforms=SETUP_PLATFORM,
    python_requires=">=2.6, <3",
    packages=[SETUP_PROJECT],
    package_dir={SETUP_PROJECT: SETUP_PROJECT},
    package_data={
        SETUP_PROJECT: [PLATFORM_FILES.get(filename) for filename in PLATFORM_FILES]
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
# Clean Project Folder
for name in os.listdir(os.path.join(PATH_HERE, SETUP_PROJECT)):
    if name in PLATFORM_FILES:
        os.remove(os.path.join(PATH_HERE, SETUP_PROJECT, name))
