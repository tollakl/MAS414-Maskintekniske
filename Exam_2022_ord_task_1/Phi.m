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
