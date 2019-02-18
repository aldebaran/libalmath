#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" ALMath """
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import ctypes

# Get the Absolute Path of the Package
path_package = os.path.dirname(os.path.realpath(__file__))

if sys.platform.startswith("linux"):
    ctypes.cdll.LoadLibrary(os.path.join(path_package, "libalmath.so"))

# Update QiAdditionalSdkPrefixes if needed
qisdk_prefixes = os.environ.get("QI_ADDITIONAL_SDK_PREFIXES", "")
if path_package not in qisdk_prefixes:
    if qisdk_prefixes:
        qisdk_prefixes += os.path.pathsep
    os.environ["QI_ADDITIONAL_SDK_PREFIXES"] = qisdk_prefixes + path_package

from .almathswig import *
