function yd = fm(t,y,M) %,cdamp,Mdriver)

% Proper notation
q = y(1:6,1); % Position
qd= y(7:12,1); % Velocity

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

% External forces
Fgrav = M[0;g;0;0;g;0];                         %insert

% Spring-damper 1                               %insert
d2 = r1 + A1*s1Bm - s0A;                        %insert
d2d = r1d + phi1d*B1*s1Bm;                      %insert
Lspring = sqrt(d2'*d2);                         %insert
u = 1/Lspring*d2;                               %insert
Fspring1 = (L_undeformed - Lspring)*k1*u;       %insert
Lspringdot = d2'*d2d/Lspring;                   %insert
Fdamper1 = -c1*Lspringdot*u;                    %insert

% Torsional spring
M2spring = -phi2*k2;                            %insert
Fs = [Fspring1; 
    CrossP2D(A1*s1Bm, Fspring1);
    0;
    0;
    0];

Fd = [Fdamper1; 
    CrossP2D(A1*s1Bm, Fdamper1);
    0;
    0;
    0];

M = [0;
    0;
    0;
    0;
    0;
    M2spring]

Fext = Fgrav + Fs + Fd + M; % + [0;0;Mdriver;0;0;0];

RH = [Fext; Gamma(q,qd)];
J = Jacobi(q);

%SKJEKK DENNE
%Equation for motion and and the second derivatives of the constraint equations
Coef = [M, -J'; J, zeros(5,5)];
% pause
% return

% Solving the equations of motion and kinematic constraints
x = Coef\RH;

qdd = x(1:6,1);
lambda = x(7:11,1);

Jtranspose = J';
ReacFor(1:8,1) = 0;
ReacFor(1:2,1) = (  Jtranspose(1:2,1:2)*lambda(1:2,1)  ); % Reaction force on body 1 in translational joint
ReacFor(3:4,1) = (  Jtranspose(4:5,3:4)*lambda(3:4,1)  ); % Reaction force on body 1 in translational joint
ReacFor(5:6,1) = (  Jtranspose(1:2,5)*lambda(5,1)  ); % Reaction force on body 1 in translational joint
ReacFor(7:8,1) = (  Jtranspose(4:5,5)*lambda(5,1)  ); % Reaction force on body 1 in translational joint

% Data back to the integrator
yd = [qd;qdd;ReacFor];