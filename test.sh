#!/bin/bash

gzserver -r \
		--iters 300000 \
		--record_path ~/Repositories/gazebo_wfqi/logs/${1} \
		--verbose \
		~/Repositories/gazebo_wfqi/testgerade.world
