#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" Create the Wheel Package """
from __future__ import absolute_import
from __future__ import unicode_literals
from __future__ import print_function

import os
import sys
import stat
import shutil
from setuptools import sandbox
import six

PATH_HERE = os.path.dirname(os.path.realpath(__file__))
PATH_TEMP = os.path.join(PATH_HERE, "Temp")
UNIXLE_EXTENSIONS = [".py", ".xml", ".json", ".pml", ".txt", ".htm", ".html", ".css"]
UNIXLE_EXCLUDED = [".git", ".dll", ".so", ".pyd", ".dylib"]

def unix_line_ending(folder, extensions=None, excluded=None):
    """ Replace Windows Line Endings with Unix Line Endings """
    if extensions is None:
        extensions = UNIXLE_EXTENSIONS
    if excluded is None:
        excluded = UNIXLE_EXCLUDED
    for dname, _dirs, files in os.walk(folder):
        if dname in excluded:
            continue
        for fname in files:
            if fname in excluded:
                continue
            fext = os.path.splitext(fname)[-1]
            if fext not in extensions and fname not in extensions:
                continue
            fpath = os.path.join(dname, fname)
            with open(fpath, "rb") as filer:
                fdata = filer.read()
            filer.close()
            try:
                if "\r\n" not in fdata:
                    continue
            except Exception:
                continue
            with open(fpath, "wb") as filew:
                filew.write(fdata.replace("\r\n", "\n"))
            filew.close()
            print("Unix Line Ending : %s" % fpath)

def folder_cleanup(folder, names=None, extensions=None):
    """ Remove all sub folders and files by name or extension """
    if not isinstance(names, list):
        names = []
    if not isinstance(extensions, list):
        extensions = []
    for dname, dirs, files in os.walk(folder):
        for dirname in dirs:
            if dirname not in names:
                continue
            dirpath = os.path.join(dname, dirname)
            if os.path.isdir(dirpath):
                if not os.access(dirpath, os.W_OK):
                    os.chmod(dirpath, stat.S_IWRITE)
                shutil.rmtree(dirpath, ignore_errors=True)
        for fname in files:
            fpath = os.path.join(dname, fname)
            if fname in names:
                if os.path.isfile(fpath):
                    if not os.access(fpath, os.W_OK):
                        os.chmod(fpath, stat.S_IWRITE)
                    os.remove(fpath)
            fext = os.path.splitext(fname)[-1]
            if fext in extensions or fname in extensions:
                if os.path.isfile(fpath):
                    if not os.access(fpath, os.W_OK):
                        os.chmod(fpath, stat.S_IWRITE)
                    os.remove(fpath)

def clean_access_rights(path):
    """ Allow write access to a path """
    if os.path.isfile(path):
        if not os.access(path, os.W_OK):
            os.chmod(path, stat.S_IWRITE)
            os.chmod(path, stat.S_IWUSR)
    elif os.path.isdir(path):
        if not os.access(path, os.W_OK):
            os.chmod(path, stat.S_IWRITE)
            os.chmod(path, stat.S_IWUSR)
        for dname, dirs, files in os.walk(path):
            for dirname in dirs:
                dirpath = os.path.join(dname, dirname)
                if os.path.isdir(dirpath):
                    if not os.access(dirpath, os.W_OK):
                        os.chmod(dirpath, stat.S_IWRITE)
                        os.chmod(dirpath, stat.S_IWUSR)
            for fname in files:
                fpath = os.path.join(dname, fname)
                if os.path.isfile(fpath):
                    if not os.access(fpath, os.W_OK):
                        os.chmod(fpath, stat.S_IWRITE)
                        os.chmod(fpath, stat.S_IWUSR)

def cleanup_folders(dist=False):
    """ Prepare the Folders for a platform """
    if dist is True:
        path_dist = os.path.join(PATH_HERE, "dist")
        if os.path.isdir(path_dist):
            shutil.rmtree(path_dist, ignore_errors=True)
    path_egg = os.path.join(PATH_HERE, "qigeometry.egg-info")
    if os.path.isdir(path_egg):
        shutil.rmtree(path_egg, ignore_errors=True)
    path_build = os.path.join(PATH_HERE, "build")
    if os.path.isdir(path_build):
        shutil.rmtree(path_build, ignore_errors=True)
    path_temp = os.path.join(PATH_HERE, "temp")
    if os.path.isdir(path_temp):
        shutil.rmtree(path_temp, ignore_errors=True)

def create():
    """ Create a Package """
    print("- Clean Access Rights")
    clean_access_rights(PATH_HERE)
    print("- Remove Building Folders")
    cleanup_folders(dist=True)
    print("- Clean .pyc Files")
    folder_cleanup(PATH_HERE, names=None, extensions=[".pyc"])
    unix_line_ending(PATH_HERE, extensions=None, excluded=None)
    print("- Build Package")
    sandbox.run_setup(os.path.join(PATH_HERE, "setup.py"), ["bdist_wheel"])
    print("- Remove Building Folders")
    cleanup_folders()
    print("- Package Generation Finished")

if __name__ == "__main__":
    create()
