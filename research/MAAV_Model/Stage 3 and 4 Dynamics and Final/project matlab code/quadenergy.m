function [EBwa,TBwa,UBw]=quadenergy(t_quad,X_quad)
% This function m-file computes the kinetic energy TBwa, the potential 
% energy UBw, and the total energy EBwa of the quad-rotor helicopter as a 
%function of time.

[m_rotor,alpha_dot,hRca_b,L,A_top,mS,JBc_b,JRq_s,g,C_drag,den_air,f_lift_base]=quadconstants();
%Retrieves the system's constants from the m-file quadconstants.m".

TBwa=zeros(numel(t_quad),1); %The matrix that will contain the kinetic energy 
%values for every time step, J.
UBw=zeros(numel(t_quad),1); %The matrix that will contain the potential energy 
%values for every time step, J.
EBwa=zeros(numel(t_quad),1); %The matrix that will contain the total energy 
%values for every time step, J.

%Computing the kinetic energy TBwa for every time step:

for i=1:1:numel(t_quad)
    x_dot=X_quad(i,4); %The a1-direction speed at the current time step, m/s.
    y_dot=X_quad(i,5); %The a2-direction speed at the current time step, m/s.
    z_dot=X_quad(i,6); %The a3-direction speed at the current time step, m/s.
    quad_vel_a=[x_dot;y_dot;z_dot]; %The quad-rotor helicopter's velocity
    %frame Fa components at the current time step, m/s.
    alpha=X_quad(i,10); %The rotors' angular positions at the current time
    %step, rad.
    wba_b1=X_quad(i,11); %The value of the b1 component of the wba angular
    %velocity vector, rad/s.
    wba_b2=X_quad(i,12); %The value of the b2 component of the wba angular
    %velocity vector, rad/s.
    wba_b3=X_quad(i,13); %The value of the b3 component of the wba angular
    %velocity vector, rad/s.
    wba_b=[wba_b1;wba_b2;wba_b3]; %The wba angular velocity frame Fb components 
    %at the current time step, rad/s.
    wsa_b=wba_b+[0;0;alpha_dot]; %The wsa angular velocity frame Fb components
    %at the current time step, rad/s.
    
    C3_alpha=quadDCMC3(alpha); %The C3(alpha) DCM at the current time step.
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
    
    TBwa(i)=(1/2)*mS*quad_vel_a'*quad_vel_a+(1/2)*wba_b'*JBc_b*wba_b+...
        4*(1/2)*(wsa_b'*JR1c_b*wsa_b+wsa_b'*JR2c_b*wsa_b+wsa_b'*JR3c_b*wsa_b+...
        wsa_b'*JR4c_b*wsa_b);
    %The kinetic energy at the current time step, J. 
end

%Computing the potential energy UBw for every time step:

for i=1:1:numel(t_quad)
    z=X_quad(i,3); %The z-position at the current time step, m.
    UBw(i)=mS*g*z; %The potential energy at the current time step, J.
end

%Computing the total energy EBwa for every time step:
EBwa=TBwa+UBw; %The total energy vector, J. 

end

    
    
    
    
    
    
    
    
    
    
    