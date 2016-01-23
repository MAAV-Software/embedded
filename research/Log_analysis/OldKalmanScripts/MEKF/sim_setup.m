%sim_setup.m

close all
clear all
clc

%% Simulation Parameters

n=5000; %Total number of prediction steps. Is equal to the total number
%of data taken by the "slowest" sensor.

T_predict=0.001; %Prediction time, seconds. Choose this value to be as 
%small as computationally possible. 
T_correct=0.01; %Sensor sampling time, seconds. Needs to be larger than
%T_predict. Should be the sampling time of the "slowest" sensor.
N=round(T_correct/T_predict); %Ratio of T_correct to T_predict. It MUST be 
%an integer so the round() function is used which rounds to the nearest 
%integer.
sim_time=linspace(0,n*T_predict,n+1); %Time vector, seconds.
sim_time(n+1)=[];

%% System-State Parameters 

%Page 31, Beard Notes.

n_states=6; %Total number of system states.
g=9.81; %Acceleration due to gravity, m/s^2.

act_states=zeros(n_states,n); %Actual states.
init_cond=[1;1;1;1;1;1;1;1;1]; %System initial conditions. In reality,
%should all be zero since the q-rotor starts from rest.
act_states(:,1)=init_cond;

act_states(:,1)=[0,0,0,0,0,0]';

est_states=zeros(n_states,n); %Estimated states. Zero initial conditions.

act_inertial_pos=zeros(3,n); %Actual inertial positions of the q-rotor.
act_inertial_vel=zeros(3,n); %Actual inertial velocities of the q-rotor.
est_inertial_pos=zeros(3,n); %Estimated inertial positions of the q-rotor.
est_inertial_vel=zeros(3,n); %Estimated inertial velocities of the q-rotor.

num_correction_steps=floor(n/N); %Total number of correction steps that occur
%during the simulation. The floor() function rounds down to the nearest
%integer.
correction_bool=ones(1,num_correction_steps); %A value of one will update
%the seven states that the camera does not measure (every state but the
%px and py positions and the yaw angle) during a correction step. A value 
%of zero will not.
camera_correction_bool=ones(1,num_correction_steps); %A value of one will
%update the two states that the camera measures (the px and py positions
%and the yaw angle) during a correction step. A value of zero will not. 

%% System-Noise Parameters

%The q-rotors system and sensor noises are assumed to be zero-mean Gaussian
%random processes:

base_sys_noise=randn(n_states,n); %Zero-mean Gaussian random system noise. 
%Has a standard deviation of one. 
base_sen_noise=randn(n_states,n); %Zero-mean Gaussian random sensor noise. 
%Has a standard deviation of one. 

Q=zeros(n_states,n_states); %Covariance matrix of the system noise. Is
%assumed to be diagonal meaning the system noises for the system states
%are uncorrelated. 
G=eye(n_states); %Since the system-noise characteristics cannot be measured, 
%G is set equal to the identity matrix to simplify the analysis.

R=zeros(n_states,n_states); %Covariance matrix of the sensor noise. Is
%assumed to be diagonal meaning the sensor noises for the system states
%are uncorrelated. 
C=eye(n_states); %The output matrix is the identity matrix as there are 
%sufficient sensors to measure all states.

%% Tuning Q and R

%For demonstration purposes, the covariances are chosen to lie within 0 
%and 0.1.

%The q-rotors system and sensor noises are assumed to be zero-mean Gaussian
%random processes.

for i=1:1:n_states
    
   Q(i,i)=0.1*rand(1);
   R(i,i)=0.1*rand(1);
   
end

system_noise=Q*base_sys_noise; %"Applying" the Q covariances to 
%base_sys_noise.
sensor_noise=R*base_sen_noise; %"Applying" the R covariances to 
%base_sen_noise.

%% System "Inputs"

%Place the az, p, q, and r measurements from text files here.

az=sin(pi*sim_time); %Units: m/s^2.
p=sin(2*pi*sim_time); %Units: rad/s.
q=sin(3*pi*sim_time); %Units: rad/s.
r=sin(4*pi*sim_time); %Units: rad/s

