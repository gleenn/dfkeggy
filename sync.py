#!/usr/bin/python
#

import subprocess
import sys

command = ['git', 'submodule', 'update', '--init']

subprocess.check_call(command)

