function [m_rotor,alpha_dot,hRca_b,L,A_top,mS,JBc_b,JRq_s,g,C_drag,den_air,f_lift_base]=quadconstants()
%This function m-file defines the system's constants. 

%The system parameters:

len_arm=0.20; %The length of one of the four rotor arms, m.
width_arm=0.05; %The width (and thickness) of the rotor arms, m.
m_arm=1; %The mass of one of the four rotor arms, kg.
rad_ring=0.05; %The radius of the rotor rings, m.
m_ring=0.25; %The mass of one of the four rotor rings, kg.

len_rotor=0.09; %The length of one of the four rotors, m.
width_rotor=0.02; %The width of one of the four rotors, m.
thickness_rotor=0.003; %The thickness of one of the four rotors, m.
m_rotor=0.05; %Mass of one of the four rotors, kg.
alpha_dot=50*(2*pi); %The angular speed of the rotors, rad/s.
hRca_b=[0;0;4*((1/2)*alpha_dot*m_rotor*(len_rotor^2+width_rotor^2))]; %The 
%components of the angular momentum vector of all four rotors relative to the 
%center of mass c with respect to frame Fa resolved in frame Fb.

L=m_arm+m_ring; %Distance between the center of mass of the quad-rotor 
%helicopter body B and the center of one of the four rotor rings, m.
A_top=4*len_arm*width_arm; %The surface area of the top of the quad-rotor
%helicopter, m^2. 
mS=4*m_arm+4*m_ring+4*m_rotor; %Total mass of the quad-rotor helicopter 
%system S, kg.

%The second moment of inertia matrix JBc_b frame Fb components for the
%helicopter body B:
JBc11_b=(1/12)*(2*m_arm)*(2*width_arm^2)+(1/12)*(2*m_arm)*(4*len_arm^2+width_arm^2)+...
    2*(m_ring*rad_ring^2)+2*(m_ring*L^2); %The (1,1) component of the JBc_b 
    %matrix, kg*m^2.
JBc22_b=JBc11_b; %The (2,2) component of the JBc_b matrix, kg*m^2.
JBc33_b=2*(1/12)*(2*m_arm)*(4*len_arm^2+width_arm^2)+4*(m_ring*rad_ring^2)+...
    4*(m_ring*L^2); %The (3,3) component of the JBc_b matrix, kg*m^2.
JBc_b=[JBc11_b,0,0;0,JBc22_b,0;0,0,JBc33_b]; %The second moment of inertia matrix. 

%The second moment of inertia matrix JRq_s frame Fs components for a rotor
%body Ri:
JRq11_s=(1/2)*m_rotor*(len_rotor^2+thickness_rotor^2); %The (1,1) component 
%of the JRq_s matrix, kg*m^2.
JRq22_s=(1/2)*m_rotor*(width_rotor^2+thickness_rotor^2); %The (2,2) component 
%of the JRq_s matrix, kg*m^2.
JRq33_s=(1/2)*m_rotor*(len_rotor^2+width_rotor^2); %The (3,3) component of 
%the JRq_s matrix, kg*m^2.
JRq_s=[JRq11_s,0,0;0,JRq22_s,0;0,0,JRq33_s]; %The second moment of inertia
%matrix. 

%Environment parameters:

g=9.81; %The acceleration due to gravity, m/sec^2.
C_drag=0.1; %The drag coefficient.
den_air=1.225; %The density of the air, kg/m^3.
f_lift_base=(mS*g)/4; %The base lift force needed to be produced by each
%rotor to have the quad-rotor helicopter hover steadily in the air, N.

end
