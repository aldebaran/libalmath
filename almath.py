#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" ALMath """
from __future__ import print_function
from __future__ import absolute_import

import os
import sys
import ctypes

# Get the Absolute Path of the Package
PATH_PACKAGE = os.path.dirname(os.path.realpath(__file__))

if sys.platform.startswith("linux"):
    try:
        ctypes.cdll.LoadLibrary(os.path.join(PATH_PACKAGE, "libalmath.so"))
    except Exception as exc:
        print(exc)
elif "darwin" in sys.platform:
    sys.path.insert(1, os.path.join(PATH_PACKAGE, "python2.7", "site-packages"))

# Update LD_LIBRARY_PATH if needed
path_library = os.environ.get("LD_LIBRARY_PATH", "")
if PATH_PACKAGE not in path_library:
    if path_library:
        path_library += os.path.pathsep
    os.environ["LD_LIBRARY_PATH"] = path_library + PATH_PACKAGE

from .almathswig import *
