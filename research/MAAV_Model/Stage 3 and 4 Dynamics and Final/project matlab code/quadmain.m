%Greg Miller 50760004
%Course Project
%Intermediate Dynamics, AEROSP 540
%This m-file solves (integrates) the quad-rotor helicopter's system of 
%ordinary differential equations (the ODE system).

%List of m-files in the directory used to carry out this integration:

% 1) quadmain.m: Is this m-file. This is the m-file in command. It 
% calls upon the other m-files on this list to perform the integration. 
% 2) quadconstants.m: Defines all of the system's constants. 
% 3) quadcross.m: Computes the cross operation on component matrices. 
% 4) quadDCMC1.m: Computes the 1-axis principle direction cosine
% matrix on an angle.
% 5) quadDCMC2.m: Computes the 2-axis principle direction cosine
% matrix on an angle.
% 6)quadDCMC3.m: Computes the 3-axis principle direction cosine
% matrix on an angle.
% 7) quadodesys.m: Defines the ODE system. Is a function m-file.
% 8) quadodesolver.m: Calls upon the ode45() solver to integrate the 
% ODE system. Is a function m-file. 
% 9) quadenergy.m: % Computes the kinetic energy TBwa, the potential 
% energy UBw, and the total energy EBwa of the system for every time step.
% Is a function m-file. 
% 10) quadplots.m: Plots the results of the ode45() solver. Is a 
% function m-file. 

close all
clear all
clc

%Setting up the ODE system integration parameters:
tspan_quad=[0,30]; %The ODE system is integrated over the time span tspan. 

%Initial conditions for the 12 states:

x_init=3; %Initial x-position, m.
y_init=3; %Initial y-position, m.
z_init=3; %Initial z-position, m.
x_dot_init=0; %Initial a1-direction speed, m/s.
y_dot_init=0; %Initial a2-direction speed, m/s.
z_dot_init=0; %Initial a3-direction speed, m/s.
theta_init=0; %Initial pitch angle, rad.
phi_init=0; %Initial roll angle, rad. 
psi_init=0; %Initial yaw angle, rad.
alpha_init=0; %Initial rotors' angular position, rad. 
wbab1_init=0; %The inital value for the b1 component of the wba angular
%velovity vector, rad/s. 
wbab2_init=0; %The inital value for the b2 component of the wba angular
%velovity vector, rad/s. 
wbab3_init=0; %The inital value for the b3 component of the wba angular
%velovity vector, rad/s. 
Xo_quad=[x_init,y_init,z_init,x_dot_init,y_dot_init,z_dot_init,theta_init,phi_init,psi_init,alpha_init,wbab1_init,wbab2_init,wbab3_init]; %The 
%vector of initial conditions for the 12 states.

[t_quad,X_quad]=quadodesolver(tspan_quad,Xo_quad); %Calling upon the 
%function m-file "quadodesolver.m" which contains the code used to 
%integrate the ODE system. 

[EBwa,TBwa,UBw]=quadenergy(t_quad,X_quad); %Calling upon the 
%function m-file "quadenergy.m" which computes the kinetic energy TBwa, the 
%potential energy UBw, and the total energy EBwa of the system as a 
%function of time. 

quadplots(t_quad,X_quad,EBwa,TBwa,UBw) %Calling upon the function m-file 
%"quadplots.m" to plot the results of the ODE solver after integrating 
%the ODE system. 


