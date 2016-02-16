#!/usr/bin/env python

import sys
from move_to_pose_map import GoToPose

test = GoToPose()

if sys.argv[1] == '1':
	test.move_to_pose(-9.798, -4.671, 135.0)

if sys.argv[1] == '2':
	test.move_to_pose(-10.224, -0.311, 100.0)

if sys.argv[1] == '3':
	test.move_to_pose(3.618, 0.727, 85.0)
