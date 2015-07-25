function [X_quad_dot]=quadodesys(t_quad,X_quad)
%This function m-file defines the quad-rotor helicopter ODE system. 

[m_rotor,alpha_dot,hRca_b,L,A_top,mS,JBc_b,JRq_s,g,C_drag,den_air,f_lift_base]=quadconstants();
%Retrieves the system's constants from the m-file quadconstants.m".

%The system states for the current time step:

x=X_quad(1); %The x-position at the current time step, m.
y=X_quad(2); %The y-position at the current time step, m.
z=X_quad(3); %The z-position at the current time step, m.
quad_pos_a=[x;y;z]; %The quad-rotor helictop position frame Fa components
%at the current time step, m.

x_dot=X_quad(4); %The a1-direction speed at the current time step, m/s.
y_dot=X_quad(5); %The a2-direction speed at the current time step, m/s.
z_dot=X_quad(6); %The a3-direction speed at the current time step, m/s.
quad_vel_a=[x_dot;y_dot;z_dot]; %The quad-rotor helicopter's velocity
%frame Fa components at the current time step, m/s.

theta=X_quad(7); %The pitch angle at the current time step, rad. 
phi=X_quad(8); %The roll angle at the current time step, rad. 
psi=X_quad(9); %The yaw angle at the current time step, rad. 
quad_ang_pos=[theta;phi;psi]; %The angular orientation of the quad-rotor
%helicopter at the current time step, rad. 

alpha=X_quad(10); %The rotors' angular position at the current time step,
%rad.

wba_b1=X_quad(11); %The value of the b1 component of the wba angular
%velocity vector, rad/s.
wba_b2=X_quad(12); %The value of the b2 component of the wba angular
%velocity vector, rad/s.
wba_b3=X_quad(13); %The value of the b3 component of the wba angular
%velocity vector, rad/s.
wba_b=[wba_b1;wba_b2;wba_b3]; %The wba angular velocity frame Fb components 
%at the current time step, rad/s.

%Computing the principle direction cosine matrices (DCM's) for the current 
%time step:
C1_theta=quadDCMC1(theta);
C2_phi=quadDCMC2(phi);
C3_psi=quadDCMC3(psi);
C3_alpha=quadDCMC3(alpha);
Cba=C3_psi*C2_phi*C1_theta; %The DCM relating frames Fa and Fb at the
%current time step. 

%Computing the Sba_b matrix at the current time step:
Sba_b=[C3_psi*C2_phi*[1;0;0],C3_psi*[0;1;0],[0;0;1]];

%Computing wbax_b for the current time step:
wbax_b=quadcross(wba_b);

%Computing JSc_b for the current time step:
rq1c_b=[L;0;0];
rq1cx_b=quadcross(rq1c_b);
rq2c_b=[0;L;0];
rq2cx_b=quadcross(rq2c_b);
rq3c_b=[-1*L;0;0];
rq3cx_b=quadcross(rq3c_b);
rq4c_b=[0;-1*L;0];
rq4cx_b=quadcross(rq4c_b);
JR1c_b=C3_alpha'*JRq_s*C3_alpha-m_rotor*rq1cx_b*rq1cx_b;
JR2c_b=C3_alpha'*JRq_s*C3_alpha-m_rotor*rq2cx_b*rq2cx_b;
JR3c_b=C3_alpha'*JRq_s*C3_alpha-m_rotor*rq3cx_b*rq3cx_b;
JR4c_b=C3_alpha'*JRq_s*C3_alpha-m_rotor*rq4cx_b*rq4cx_b;
JSc_b=JBc_b+JR1c_b+JR2c_b+JR3c_b+JR4c_b;

%Declaring the rotor lift forces (the inputs), all with units of Newtons:
if t_quad>=0 & t_quad<0.1
    f1=f_lift_base+1;
    f2=f_lift_base;
    f3=f_lift_base;
    f4=f_lift_base+1;
end

% %Simulation #3:
% 
% if t_quad>=0.1
%     f1=f_lift_base;
%     f2=f_lift_base;
%     f3=f_lift_base;
%     f4=f_lift_base;
% end

%Simulation #4 using a Basic Height Controller:
if t_quad>=0.1
    z_desired=50; %Desired quad-rotor helicopter height, m.
    P_Gain=1;
    force=P_Gain*(z_desired-z);
    f1=f_lift_base+force;
    f2=f_lift_base+force;
    f3=f_lift_base+force;
    f4=f_lift_base+force;
end

%Computing the net force acting on the body B in for the time derivative 
%of linear momentum equations:
fS_drag_a=(-1/2)*C_drag*(A_top/3)*den_air*sqrt(x_dot^2+y_dot^2+z_dot^2)*...
    [x_dot;y_dot;z_dot]; %The drag force frame Fa components, Newtons.
fS_a=(Cba'*[0;0;f1+f2+f3+f4]+[0;0;-1*mS*g]+fS_drag_a); %The net force frame
%Fa components, Newtons. 
%fS_a=[0;0;-1*mS*g]; %Zero-force input for Simulations #1 and #2, Newtons.

%Computing the net moment acting on the body B with respect to its center 
%of mass c for the time derivative of angular momentum equations:
mSc_b=[L*f2-L*f4;-1*L*f1+L*f3;0]; %The net moment frame Fb components, N*m.
%mSc_b=[0;0;0]; %Zero-moment input for Simulations #1 and #2, N*m.

%Computing the state derivatives component matrices for the current time
%step:
X_quad_dot123=quad_vel_a;
X_quad_dot456=(1/mS)*fS_a;
X_quad_dot789=inv(Sba_b)*wba_b;
X_quad_dot10=alpha_dot;
X_quad_dot11_12_13=inv(JSc_b)*(mSc_b-wbax_b*(JSc_b*wba_b+hRca_b));
X_quad_dot=[X_quad_dot123;X_quad_dot456;X_quad_dot789;X_quad_dot10;X_quad_dot11_12_13];

end










