#!/bin/bash

xhost +
docker compose up &
python3 ./planner/visualizer.py
