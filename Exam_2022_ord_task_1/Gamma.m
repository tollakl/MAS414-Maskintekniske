function gam = Gamma(q,qd,t)
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

gam = zeros(5,1);

% Revolute joint between body 1 and ground
gam(1:2,1) = phi1d^2*A1*s1Cm;

% Revolute joint between body 2 and ground
gam(3:4,1) = phi2d^2*A2*s2Fm;

% Revolute-translational joint between body 1 and 2
d1 = r1 + A1*s1Dm - (r2 + A2*s2Em);
d1d = r1d + phi1d*B1*s1Dm - r2d - phi2d*B2*s2Em;
gam(5,1) = (phi1d^2*A1*n1m)'*d1 - 2*(phi1d*B1*n1m)'*d1d - (A1*n1m)'*(-phi1d^2*A1*s1Dm + phi2d^2*A2*s2Em);
