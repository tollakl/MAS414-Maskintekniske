function Phi = Phi(q,t)

% Proper notation for the coordinates for each body
% Add variable for all bodies, (this example is for two bodies)
r1 = q(1:2,1);
phi1 = q(3,1);
r2 = q(4:5,1);
phi2 = q(6,1);

A1 = Arot(phi1);
A2 = Arot(phi2);

%PASTE HERE
%------------------------------------------------------




%------------------------------------------------------



% This is the position constraints of the joints
% Revolute has two constraint
% Revolute- trans has one constraint

%Step 1: Find all joints

% Initialization of the Phi-vector
Phi = zeros(5,1);

% Joint 1 
Phi(1:2,1) = r1 + A1*s1Cm - s0C;

% Joint 2 
Phi(3:4,1) = r2 + A2*s2Fm - s0F;

% Joint 3
d1 = r1 + A1*s1Dm - (r2 + A2*s2Em);
Phi(5,1) = (A1*n1m)'*d1;
