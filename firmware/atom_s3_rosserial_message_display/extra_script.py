#!/usr/bin/env python

import os
import subprocess
import sys

def run_subprocess(cmd, working_directory="."):
    if sys.version.split(".")[0] == "2":
        subprocess.call(cmd, shell=True, cwd=working_directory)
    if sys.version.split(".")[0] == "3":
        subprocess.run(cmd, shell=True, cwd=working_directory)

def get_lib_dir():
    current_dir = os.getcwd()
    parent_dir = os.path.dirname(current_dir)
    lib_dir = os.path.join(parent_dir, 'lib')
    return lib_dir

def generate_roslib():
    run_subprocess("rosrun rosserial_arduino make_libraries.py .", working_directory=get_lib_dir())

generate_roslib()
