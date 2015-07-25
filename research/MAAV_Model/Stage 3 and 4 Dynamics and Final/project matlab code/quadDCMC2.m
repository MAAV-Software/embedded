function [C2_angle]= quadDCMC2(angle)
%This function m-file computes the 2-axis principle direction cosine
%matrix.

C2_angle=[cos(angle),0,-1*sin(angle);0,1,0;sin(angle),0,cos(angle)];

end
