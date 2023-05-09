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


%---------------------------------------------------------------
% Geometry
L1  = 2.6;                    %|
L2  = 0.6;                    %|
L3  = 2.8;                    %|
L4  = 1.7;                    %|
L5  = 3.05;                    %|
L6  = 3.5;                    %|
L7  = 0.5;                    %|
L8  = 1.65;                    %|
L9  = 0.75;                    %|
L10 = 0.6;                    %|
L11 = 0.6;                    %|
L12 = 0.5;                    %|
L13 = 2.175;                    %|
L14 = 2.325;                    %|

%Spring L0_init
L_undeformed = 3.2;           %|

%Angle to rad               
alpha1 = 18*pi/180;          %|

%Spring damper constant
k1 = 27400; % N/m           %|
c1 = 1100; % Ns/m           %|
k2 = 3500; % Nm/rad         %|
g = -9.81; % m/s^2          %|
    
% Constant vectors
s0A =   [-L4;L2];                %|
s0C =   [-L3;L1];                %|
s0F =   [L5;L6];                %|
s1Bm =  [L9;-L10];                %|
s1Cm =  [-L8;-L7];                %|
s1Dm =  [-L11;L12];                %|
s2Em =  [-L13;0];                %|
s2Fm =  [L14;0];                %|
n1m  =  [-sin(-alpha1);cos(-alpha1)];    %Rememer to make sure of this
%---------------------------------------------------------------------


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


