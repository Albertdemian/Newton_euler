function [force, torque, magnitude]= newton_ft(R, O,omegak,alphak, ack,g, m, forcek,torquek,I)

force = R*forcek + m*ack - m*g;

torque = R*torquek - cross(force,O/2) + cross(R*forcek , O/2)+ alphak + cross(omegak,I*omegak);

magnitude.F = dot(force,O);
magnitude.T = dot(torque,O);

end