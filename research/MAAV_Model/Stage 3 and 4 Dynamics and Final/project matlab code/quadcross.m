function [comp_mat_cross] = quadcross(comp_mat)
%This function m-file computes the cross operation of a vector's 
%component matrix. 

comp_mat_cross=[0,-1*comp_mat(3),comp_mat(2);comp_mat(3),0,-1*comp_mat(1);...
    -1*comp_mat(2),comp_mat(1),0]; %The cross operation.

end

