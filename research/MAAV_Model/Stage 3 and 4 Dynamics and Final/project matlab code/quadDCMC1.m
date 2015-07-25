function [C1_angle]= quadDCMC1(angle)
%This function m-file computes the 1-axis principle direction cosine
%matrix.

C1_angle=[1,0,0;0,cos(angle),sin(angle);0,-1*sin(angle),cos(angle)];

end