%eval_fxu.m

%Page 35, Beard Notes.

function fxu=eval_fxu(curr_states,az,p,q,r)

g=9.81; %Acceleration due to gravity, m/s^2.

px=curr_states(1); py=curr_states(2); pz=curr_states(3);
px_dot=curr_states(4); py_dot=curr_states(5); pz_dot=curr_states(6);
phi=curr_states(7); theta=curr_states(8); psi=curr_states(9);

fxu=[px_dot;
     py_dot;
     pz_dot;
     cos(phi)*sin(theta)*az;
     -1*sin(phi)*az;
     g+cos(phi)*cos(theta)*az;
     p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
     q*cos(phi)-r*sin(phi);
     q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta))];

end