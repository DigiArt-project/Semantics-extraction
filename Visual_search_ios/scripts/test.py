#!/usr/bin/env python

import sys
import os
from subprocess import *

print "pwd : "
print os.getcwd()
dir_path = os.path.dirname(os.path.realpath(__file__))
print "Dir_path : " + dir_path
