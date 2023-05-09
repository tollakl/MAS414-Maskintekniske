% function gam = Gamma(q,qd,t)
% Proper notation for the coordinates for each body
r1   = q(1:2,1);
phi1 = q(3,1);
r2   = q(4:5,1);
phi2 = q(6,1);
r1d   = qd(1:2,1);
phi1d = qd(3,1);
r2d   = qd(4:5,1);
phi2d = qd(6,1);

A1 = Arot(phi1);
A2 = Arot(phi2);
B1 = Brot(phi1);
B2 = Brot(phi2);

%PASTE HERE
%------------------------------------------------------


%------------------------------------------------------

gam = zeros(5,1);

% Revolute joint between body 1 and ground
gam(1:2,1) = phi1d^2*A1*s1Cm;

% Revolute joint between body 2 and ground
gam(3:4,1) = phi2d^2*A2*s2Fm;

% Revolute-translational joint between body 1 and 2
d1 = r1 + A1*s1Dm - (r2 + A2*s2Em);
d1d = r1d + phi1d*B1*s1Dm - r2d - phi2d*B2*s2Em;
gam(5,1) = (phi1d^2*A1*n1m)'*d1 - 2*(phi1d*B1*n1m)'*d1d - (A1*n1m)'*(-phi1d^2*A1*s1Dm + phi2d^2*A2*s2Em);
