#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Path Planning Project
#
# Copyright (C) 2020 Mark Weinreuter <mark.weinreuter@kit.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
import importlib
import unittest
from os.path import join
import sys

import rospkg
from rll_tools.run import run_project_in_background


def add_script_to_python_path(pkg_name, scripts_path_relative_to_pkg):
    pkg_path = rospkg.RosPack().get_path(pkg_name)
    scripts_path = join(pkg_path, scripts_path_relative_to_pkg)
    sys.path.append(scripts_path)


def create_test_class(pkg_dirs_to_add, module_name,
                      execute_func_name, client_module_name, client_name):
    for pkg_name, rel_path in pkg_dirs_to_add:
        add_script_to_python_path(pkg_name, rel_path)

    code_module = importlib.import_module(module_name)
    client_module = importlib.import_module(client_module_name)
    exec_func = getattr(code_module, execute_func_name)

    class DemoScriptRunner(unittest.TestCase):

        def __init__(self, *args, **kwargs):
            super(DemoScriptRunner, self).__init__(*args, **kwargs)

        def runTest(self):  # pylint: disable=invalid-name
            client = getattr(client_module, client_name)(exec_func)
            run_project_in_background(8)
            last_result = client.spin(oneshot=True)
            self.assertTrue(last_result)

    return DemoScriptRunner
