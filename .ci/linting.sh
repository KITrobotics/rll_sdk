#!/bin/bash

set -e

CLANG_TIDY_CMD="run-clang-tidy-7.py -p /root/target_ws/build"
CLANG_TIDY_CHECK_PACKAGES="rll_move rll_move_client rll_kinematics rll_moveit_kinematics_plugin"

for package in $CLANG_TIDY_CHECK_PACKAGES
do
    $CLANG_TIDY_CMD/$package -header-filter=$package/*
done

pip -q install 'pylint<2.0.0'
. /root/target_ws/install/setup.bash
find * -not -path 'docs/*' -iname '*.py' | xargs python -m pylint
