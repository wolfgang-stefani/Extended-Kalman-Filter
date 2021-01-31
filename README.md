# Extended Kalman Filter - Tracking Moving Objects

In this project I implemented a Kalman filter in C++ to estimate the state of a nmoving bicycle that travels around a vehicle.

This project involves a simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The simulator provides the script simulated lidar and radar measurements of the bicycle. The script feeds back the measured estimation markers and RMSE (root mean squared error) values from its Kalman filter. All in all, I used a Kalman filter, lidar measurements and radar measurements to track the bicycle's position and velocity:

The gif above shows what the simulator looks like when a C++ script is using its Kalman filter to track the object. Lidar measurements are red circles, radar measurements are blue circles and estimation markers are green triangles.

Check out this video on YouTube to see a real world example of object tracking with lidar:
[![IMAGE ALT TEXT HERE](./readme_data/Thumbnail.png)](https://youtu.be/FMNJPX_sszU)

In this project, I only tracked one object, but the video will give you a better sense for how object tracking with lidar works.
