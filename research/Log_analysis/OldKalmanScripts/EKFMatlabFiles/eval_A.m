%eval_A.m

%Page 36, Beard Notes.

function A=eval_A(curr_states,az,p,q,r)

g=9.81; %Acceleration due to gravity, m/s^2.

px=curr_states(1); py=curr_states(2); pz=curr_states(3);
px_dot=curr_states(4); py_dot=curr_states(5); pz_dot=curr_states(6);
phi=curr_states(7); theta=curr_states(8); psi=curr_states(9);

A=[0,0,0,1,0,0,0,0,0;
   0,0,0,0,1,0,0,0,0;
   0,0,0,0,0,1,0,0,0;
   0,0,0,0,0,0,-1*sin(phi)*sin(theta)*az,cos(phi)*cos(theta)*az,0;
   0,0,0,0,0,0,-1*cos(phi)*az,0,0;
   0,0,0,0,0,0,-1*sin(phi)*cos(theta)*az, -1*cos(phi)*sin(theta)*az,0;
   0,0,0,0,0,0,q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta),(q*sin(phi)+r*cos(phi))/(cos(theta))^2,0;
   0,0,0,0,0,0,-1*q*sin(phi)-r*cos(phi),0,0;
   0,0,0,0,0,0,(q*cos(phi)-r*sin(phi))/(cos(theta)),-1*(q*sin(phi)+r*cos(phi))*(tan(theta)/cos(theta)),0];

end