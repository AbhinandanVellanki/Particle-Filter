# Particle Filter - MATLAB Implementation

This repository contains a **complete particle filter implementation in MATLAB** for robot localization using range measurements from known beacons. The particle filter is a nonparametric Bayesian method that uses a collection of weighted particles to represent the posterior distribution of the robot's pose.

**Core Algorithm:** Sequential Monte Carlo (SMC) estimation with motion model prediction, measurement likelihood weighting, and particle resampling. **Features:** Handles 2D robot localization (x, y, theta) with process and measurement noise, implements adaptive resampling to maintain particle diversity. **Visualization:** Real-time animation shows particle cloud evolution, ground truth trajectory, beacon positions, and estimated robot pose compared to true position. **Output:** Video file demonstrating particle filter convergence and tracking accuracy over time on simulated sensor data.
