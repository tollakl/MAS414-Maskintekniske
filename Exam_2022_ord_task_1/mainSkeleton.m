clear
close all
clc


tic

%COPY THIS INTO ALL FUNCTIONS
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


% Mass properties
m1 = 1200;                     %|
J_G1 = 2100;                   %|
m2 = 680;                     %|
J_G2 = 980;                   %|
M = diag([m1,m1,J_G1, m2,m2,J_G2]);

% % Initial guess (accurate in this case)
% qini = [-L3 + L8;...
%         L1 + L7;...
%         0;... % Initial posiiton of body 1
%         L5 - L14;...
%         L6;...
%         0;... % Initial posiiton of body 2
%         0; 0; 0; 0; 0; 0] % Initial velocities
% qini = NewtonRaphson('Phi1',qini(1:6,1),0)

  
% Time for the analysis
tmin = 0;                   %|
tmax = 15;                  %|  Impotant to see in the task
dt = 0.0025;                %|  Impotant to see in the task
t = [tmin:dt:tmax]';        
N = length(t);              


%Initial guesses can be given in task
qini = [-1.15;...
      3.1;...
      -0;...                        % Initial posiiton of body 1
      0.725000365677873;...
      3.501303994934132;...
      -0.000560858065879;...        % Initial posiiton of body 2
      0; 0; 0; 0; 0; 0]             % Initial velocities]


MKE = Phi(qini(1:6,1),0);
MKE1 = Jacobi(qini(1:6,1))

MKE2 = Gamma(qini(1:6,1), qini(7:12,1),0)

MKE3 = NumericalJacobian('Phi',qini(1:6,1),0)

%INTEGRATOR
[time,Y,Yd,Ydd,ReacFor] = RK4extraMultiBodyWithExtraData('fm',t,dt,qini,6,8,M);


% % Call to integrator function [tspan,Y,Yd,Ydd,ExtraData] = RK4extraMultiBodyWithExtraData(f,tspan,dt,yini,Nvar,Nextra,varargin)
% [tspan,Y,Yd,Ydd,ExtraStates,ExtraStates_dot,ExtraData] = RK4extraMultiBody_ExtraStates_ExtraData(f,tspan,dt,yini,Nvar,NExtraStates,NextraData,varargin)
% [tspan,Y,Yd,Ydd,ExtraStates,ExtraStates_dot,ExtraData] = RK4extraMultiBody_ExtraStates_ExtraData( 'fm',t,  dt,qini,9,   1,           1,        M);


toc


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%  Plot of position, velocity and acceleration for the solution text  %%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%POSITION
figure(20)        
lw = 0.6;
lw1 = 0.75;
h = 2.6;
b = 14.2;
lc = 1.4;
lt = -90;
wp = 6.4;
hp = 4.9;
% hp = 3.9;
dw = 1.4;
dh = 1.0;
labelyx = -0.85;
labelyy = hp/2;
labelxx = wp/2;
labelxy = -0.4;
FontSize = 9;
fig = gcf;
c = fig.Units;
fig.Units = 'centimeters';
c = fig.Position;
fig.Position = [5 5 15.9 12];
% fig.Position = [5 5 15.9 10];


subplot(2,2,1)
hold on
     plot(t,Y(:,1),'b-','LineWidth',lw)
     plot(t,Y(:,2),'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[dw,2*dh+hp,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Position [m]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('x_1','y_1','Location','east')


subplot(2,2,3)
hold on
     plot(t,Y(:,3),'m-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[dw,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Position [rad]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('\phi_1','Location','northeast')


subplot(2,2,2)
hold on
     plot(t,Y(:,4),'b-','LineWidth',lw)
     plot(t,Y(:,5),'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,2*dh+hp,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Position [m]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('x_2','y_2','Location','east')


subplot(2,2,4)
hold on
     plot(t,Y(:,6),'m-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Position [rad]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('\phi_2','Location','northeast')


%%%%%%%%%%%%%%%%%%%%%%VELOCITY

figure(21)        
lw = 0.6;
lw1 = 0.75;
h = 2.6;
b = 14.2;
lc = 1.4;
lt = -90;
wp = 6.6;
hp = 4.9;
% hp = 3.9;
dw = 1.2;
dh = 1.0;
labelyx = -0.65;
labelyy = hp/2;
labelxx = wp/2;
labelxy = -0.4;
FontSize = 9;
fig = gcf;
c = fig.Units;
fig.Units = 'centimeters';
c = fig.Position;
fig.Position = [5 5 15.9 12];
% fig.Position = [5 5 15.9 10];


subplot(2,2,1)
hold on
     plot(t,Yd(:,1),'b-','LineWidth',lw)
     plot(t,Yd(:,2),'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[dw,2*dh+hp,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Velocity [m/s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('x_{1,d}','y_{1,d}','Location','northeast')


subplot(2,2,3)
hold on
     plot(t,Yd(:,3),'m-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[dw,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Velocity [rad/s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('\phi_{1,d}','Location','northeast')


subplot(2,2,2)
hold on
     plot(t,Yd(:,4),'b-','LineWidth',lw)
     plot(t,Yd(:,5),'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,2*dh+hp,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Velocity [m/s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('x_{2,d}','y_{2,d}','Location','northeast')


subplot(2,2,4)
hold on
     plot(t,Yd(:,6),'m-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Velocity [rad/s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('\phi_{2,d}','Location','northeast')




%%%%%%%%%%%%%%%%%%%ACCELERATION
figure(22)          
lw = 0.6;
lw1 = 0.75;
h = 2.6;
b = 14.2;
lc = 1.4;
lt = -90;
wp = 6.6;
hp = 4.9;
% hp = 3.9;
dw = 1.2;
dh = 1.0;
labelyx = -0.65;
labelyy = hp/2;
labelxx = wp/2;
labelxy = -0.4;
FontSize = 9;
fig = gcf;
c = fig.Units;
fig.Units = 'centimeters';
c = fig.Position;
fig.Position = [5 5 15.9 12];
% fig.Position = [5 5 15.9 10];


subplot(2,2,1)
hold on
     plot(t,Ydd(:,1),'b-','LineWidth',lw)
     plot(t,Ydd(:,2),'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[dw,2*dh+hp,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Acceleration [m/s^2]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('x_{1,dd}','y_{1,dd}','Location','northeast')


subplot(2,2,3)
hold on
     plot(t,Ydd(:,3),'m-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[dw,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(t), max(t), -2e4, 3e4]) %øvre og nedre grænse
ylabel('Acceleration [rad/s^2]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('\phi_{1,dd}','Location','southeast')


subplot(2,2,2)
hold on
     plot(t,Ydd(:,4),'b-','LineWidth',lw)
     plot(t,Ydd(:,5),'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,2*dh+hp,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Acceleration [m/s^2]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('x_{2,dd}','y_{2,dd}','Location','northeast')


subplot(2,2,4)
hold on
     plot(t,Ydd(:,6),'m-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Acceleratino [rad/s^2]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('\phi_{2,dd}','Location','southeast')



% %%%%%% Plot of reactions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(23)
lw = 0.6;
lw1 = 0.75;
% h = 2.6;
% b = 14.2;
lc = 1.4;
lt = -90;
wp = 4;
% wp = 4.1;
hp = 7.8;
% hp = 3.9;
dw = 1.2;
dh = 1.0;
labelyx = -0.68;
labelyy = hp/2;
labelxx = wp/2;
labelxy = -0.5;
FontSize = 9;
fig = gcf;
c = fig.Units;
fig.Units = 'centimeters';
c = fig.Position;
fig.Position = [5 5 15.9 9];
% fig.Position = [5 5 15.9 10];


subplot(1,3,1)
hold on
     plot(t,ReacFor(:,1)/1000,'b-','LineWidth',lw)
     plot(t,ReacFor(:,2)/1000,'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[dw,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(t), max(t), -2e4, 3e4]) %øvre og nedre grænse
ylabel('Reaction force [kN]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('R_{1Cx}','R_{1Cy}','Location','southeast')


subplot(1,3,2)
hold on
     plot(t,ReacFor(:,3)/1000,'b-','LineWidth',lw)
     plot(t,ReacFor(:,4)/1000,'r-','LineWidth',lw)
hold off
grid on
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Reaction force [kN]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('R_{2Fx}','R_{2Fy}','Location','southeast')




subplot(1,3,3)
hold on
     plot(t,ReacFor(:,5)/1000,'b-','LineWidth',lw)
     plot(t,ReacFor(:,6)/1000,'r-','LineWidth',lw)
     plot(t,ReacFor(:,7)/1000,'m-','LineWidth',lw)
     plot(t,ReacFor(:,8)/1000,'k-','LineWidth',lw)
hold off
grid on
% set(gca,'GridLineStyle','--','Units','Centimeter','Position',[2*dw+wp,dh,wp,hp], 'FontName','Times New Roman') % '-' '--' ':' '-.'
set(gca,'GridLineStyle','--','Units','Centimeter','Position',[3*dw+2*wp,dh,wp,hp], 'FontName','Times New Roman','FontSize',FontSize) % '-' '--' ':' '-.'
box on  % Sætter en boks om det enkelte sæt akser
% axis([min(x), max(x), -6000, 4000]) %øvre og nedre grænse
ylabel('Reaction force [kN]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelyx, labelyy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
xlabel('Time t [s]',...
    'Units','Centimeter',...
    'FontSize',FontSize,...       
    'Position',[labelxx, labelxy],... % Placering af label. Koordinaterne er opgivet i de aktuelle akseinddelinger
    'FontName','Times New Roman')
    legend('R_{1Ex}','R_{1Ey}','R_{2Ex}','R_{2Ey}','Location','southeast')
%     legend({'R_{1Ex}','R_{1Ey}','R_{2Ex}','R_{2Ey}'},'Position',[0 0 2 2],'Units','centimeter')
