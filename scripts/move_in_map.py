#!/usr/bin/env python

import sys
from move_to_pose_map import GoToPose

test = GoToPose()

if sys.argv[1] == '1':
	test.move_to_pose(-12.1, -4.5, 0.9, 0.4)

if sys.argv[1] == '2':
	test.move_to_pose(-12.3, 0.26, 0.6, 0.8)

if sys.argv[1] == '3':
	test.move_to_pose(1.888, -3.2, -0.74, 0.68)
