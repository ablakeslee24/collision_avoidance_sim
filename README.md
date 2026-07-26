# Collision Detection and Avoidance Simulator
Work-in-Progress python conflict detection and avoidance simulation

## Current Features
-Class representing each aircraft with ID, start position, destination, speed, and priority
-Predicts future conflicts between paths
-Checks separation between aircraft
-Slows lower priority aircraft to avoid conflict
-Resumes speed once clear of conflict
-Logs slow and resume events

## Future Features
-Full simulation loop using the modules
-Test scenarios to stress test
-Visualizer for trajectory, conflict logs, and transit time
-Written analysis

## Structure

lorenz_audition
  -drone.py
  -conflict_detect.py
  -conflict_res.py
  -README.md
