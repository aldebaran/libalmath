#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" Upload Generated Wheel Package to PyPi """
from __future__ import absolute_import
from __future__ import unicode_literals
from __future__ import print_function

import os
import getpass

from twine.commands.upload import main as twine_upload

CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))

if __name__ == "__main__":
    if os.environ.get("TWINE_USERNAME") is None:
        os.environ["TWINE_USERNAME"] = "sbr"
    if os.environ.get("TWINE_PASSWORD") is None:
        os.environ["TWINE_PASSWORD"] = getpass.getpass().strip()
    dist_dir = os.path.join(CURRENT_PATH, "dist")
    if not os.listdir(dist_dir):
        exit()
    twine_upload([os.path.join(dist_dir, "*")])
    for filename in os.listdir(dist_dir):
        os.remove(os.path.join(dist_dir, filename))
