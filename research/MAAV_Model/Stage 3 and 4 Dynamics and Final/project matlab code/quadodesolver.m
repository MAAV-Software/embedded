function [t_quad,X_quad]=quadodesolver(tspan_quad,Xo_quad)
%This function m-file calls upon the Matlab built-in solver for a system
%of differential equations, ode45(), to solve (integrate) the quad-rotor
%helicopter ODE system. 

[t_quad,X_quad]=ode45(@quadodesys,tspan_quad,Xo_quad);

end