function Phi = Phi(q,t)

% Proper notation for the coordinates for each body
r1 = q(1:2,1);
phi1 = q(3,1);
r2 = q(4:5,1);
phi2 = q(6,1);

A1 = Arot(phi1);
A2 = Arot(phi2);

% Geometry
L1  = 2.600;
L2  = 0.600;
L3  = 2.800;
L4  = 1.700;
L5  = 3.050;
L6  = 3.500;
L7  = 0.500;
L8  = 1.650;
L9  = 0.750;
L10 = 0.600;
L11 = 0.600;
L12 = 0.500;
L13 = 2.175;
L14 = 2.325;
L_undeformed = 2;

alpha1 = 18*pi/180;

% Constant vectors
s0A = [-L4; L2];
s0C = [-L3; L1];
s0F = [L5; L6];
s1Bm = [L9; -L10];
s1Cm = [-L8; -L7];
s1Dm = [-L11; L12];
s2Em = [-L13; 0];
s2Fm = [L14; 0];
n1m  = [-sin(-alpha1);cos(-alpha1)];

% Initialization of the Phi-vector
Phi = zeros(6,1);
% Revolute joint between body 1 and ground
Phi(1:2,1) = r1 + A1*s1Cm - s0C;
% Revolute joint between body 2 and ground
Phi(3:4,1) = r2 + A2*s2Fm - s0F;

% Revolute-translational joint between body 1 and 2
d1 = r1 + A1*s1Dm - (r2 + A2*s2Em);
Phi(5,1) = (A1*n1m)'*d1;

Phi(6,1) = phi1;