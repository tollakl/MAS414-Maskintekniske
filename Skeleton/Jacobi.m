function J = Jacobi(q)

% Proper notation for the coordinates for each body
% Add for all bodies this example is for two bodies

r1   = q(1:2,1);
phi1 = q(3,1);
r2   = q(4:5,1);
phi2 = q(6,1);

A1 = Arot(phi1);
A2 = Arot(phi2);
B1 = Brot(phi1);
B2 = Brot(phi2);

%PASTE HERE GEOMETRY AND Constant vectors
%------------------------------------------------------




%------------------------------------------------------

J = zeros(5,6);

% eye = inentity matrix

% Joint 1
J(1:2,1:3) = [eye(2), B1*s1Cm];

% Joint 2
J(3:4,4:6) = [eye(2), B2*s2Fm];

% Joint 3
d1 = r1 + A1*s1Dm - (r2 + A2*s2Em);

J(5,1:6) = [(A1*n1m)',   ((B1*n1m)'*d1 + (A1*n1m)'*B1*s1Dm),  -(A1*n1m)',  -(A1*n1m)'*B2*s2Em];


