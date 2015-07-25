function [C3_angle]= quadDCMC3(angle)
%This function m-file computes the 3-axis principle direction cosine
%matrix.

C3_angle=[cos(angle),sin(angle),0;-1*sin(angle),cos(angle),0;0,0,1];

end

