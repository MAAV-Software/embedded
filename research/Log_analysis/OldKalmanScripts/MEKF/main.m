%main.m 

%% Simulation Setup

sim_setup;

%% Compute the Actual States

comp_act_states;

%% Compute the Estimated States

comp_est_states;

%% Converting the Yaw-Frame Positions and Velocities to the Inertial Frame

convert_frames;

%% Plot the Results

plot_results;

%% Key Notes

%Key Pages, Beard Notes: 11, 20, 31, 34-36.

%Nine System States, See Pages 11, 20, and 34:

%The "yaw frame" is the inertial frame rotated through the yaw angle, psi.

%1) Position, x-direction, yaw frame: px
%2) Position, y-direction, yaw frame: py
%3) Position, z-direction, yaw frame: pz
%4) Velocity, x-direction, yaw frame: dpx/dt
%5) Velocity, y-direction, yaw frame: dpy/dt
%6) Velocity, z-direction, yaw frame: dpz/dt
%7) Roll Euler Angle: phi
%8) Pitch Euler Angle: theta
%9) Yaw Euler Angle: psi

%Four System "Inputs", All Measured by the IMU
%1) Acceleration, z-direction, body frame: az
%2) Angular Velocity, x-direction, body frame: p
%3) Angular Velocity, y-direction, body frame: q
%4) Angular Velocity, z-direction, body frame: r

%Even though the actual inputs are the rotor lift forces, the EKF is setup
%to accept the az, p, q, and r measurements for practicality.
