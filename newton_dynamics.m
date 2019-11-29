function [omega,alpha, ac, ae, g, centrifugal_c, magnitude] = newton_dynamics(R0i,R,O0i, O, Q ,omeg0,alpha0, ae0 )
syms t
b = R'*O0i; 

omega = R'*omeg0 + b*Q(2);
alpha = R'*alpha0 + b*Q(3) + cross(omega, (b*Q(2)));

d_omega = diff(omega,t);
centrifugal_c = cross(omega, cross(omega, O/2));

ae = R'*ae0 + cross(d_omega, O)+ cross(omega, cross(omega, O));
ac = R'*ae0 + cross(d_omega, O/2) + centrifugal_c; 
g = R0i'*[0;0;-9.81];

magnitude.C = dot(centrifugal_c,O);
magnitude.omega = dot(omega,O);
magnitude.alpha = dot(alpha,O);
magnitude.ae = sqrt(dot(ae,ae));
magnitude.ac = sqrt(dot(ac,ac));
magnitude.g = dot(g,O0i);
end 