function [omega,alpha, ac, ae,g,coriollos,magnitude] = newton_dynamics_p(R0i,R,O0i, O, Q ,omeg0,alpha0, ae0 )
syms t
b = R'*O0i; 

omega = R'*omeg0;
alpha = R'*alpha0;

d_omega = diff(omega,t);
coriollos = cross((2*Q(2)*omega),O0i);
centrifugal_c = cross(omega, cross(omega, O/2));

ae = R'*ae0 + cross(d_omega, O)  + cross(omega, cross(omega, O))+Q(3)*O0i+ cross((2*Q(2)*omega),O0i);
ac = R'*ae0 + cross(d_omega,O/2) + centrifugal_c +Q(3)*O0i+ cross((2*Q(2)*omega),O0i); 
g = R0i'*[0;0;-9.81];

magnitude.coriollos = dot(coriollos,O);
magnitude.C = dot(centrifugal_c,O);
magnitude.omega = dot(omega,O);
magnitude.alpha = dot(alpha,O);
magnitude.ae = sqrt(dot(ae,ae));
magnitude.ac = sqrt(dot(ac,ac));
magnitude.g = dot(g,O0i);

end 