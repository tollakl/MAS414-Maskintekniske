function [tspan,Y,Yd,Ydd,ExtraData] = RK4extraMultiBodyWithExtraData(f,tspan,dt,yini,Nvar,Nextra,varargin)
% [tspan,Y,Yd,Ydd] = RK4(f,tspan,dt,yini,Nvar,Nlambda,var1,var2...)
% 
% INPUT
% f         -> name of function
% tspan     -> interval of integration linspace(tmin:dt:tmax)
% dt        -> increment
% yini      -> initial values
% Nvar      -> number of state variables
% Nextra    -> number of extra data that is returned to calling file
% var1,var2 -> extra variables that is passed on to the function f
% 
% OUTPUT
% tspan       -> interval of integration linspace(tmin:dt:tmax)
% Y           -> Position
% Yd          -> Velocity
% Ydd         -> Acceleration

n = length(tspan);
m = length(yini);
p = 3*Nvar;
q = 2*Nvar;
r = 2*Nvar+1;
s = Nvar+1;
y = yini;
Y(1:n,1:Nvar) = 0;
Yd(1:n,1:Nvar) = 0;
Ydd(1:n,1:Nvar) = 0;
ExtraData(1:n,1:Nextra) = 0;
Y(1,1:Nvar) = yini(1:Nvar,1)';
Yd(1,1:Nvar) = yini(s:q,1)';


for i=1:(n-1)
    temp1 = feval(f,tspan(i),y,varargin{:});
    Ydd(i,1:Nvar) = temp1(s:q,1)';
    ExtraData(i,1:Nextra) = temp1(r:q+Nextra,1)';
    k1 = dt*temp1(1:q,1);
    temp2 = feval(f,tspan(i)+dt/2,y+k1/2,varargin{:});
    k2 = dt*temp2(1:q,1);
    temp3 = feval(f,tspan(i)+dt/2,y+k2/2,varargin{:});
    k3 = dt*temp3(1:q,1);
    temp4 = feval(f,tspan(i)+dt,y+k3,varargin{:});
    k4 = dt*temp4(1:q,1);
    y = y + k1/6 + k2/3 +k3/3 + k4/6;
    Y(i+1,1:Nvar) = y(1:Nvar,1)';
    Yd(i+1,1:Nvar) = y(s:q,1)';
end
temp1 = feval(f,tspan(n),y,varargin{:});
Ydd(n,1:Nvar) = temp1(s:q,1)';
ExtraData(n,1:Nextra) = temp1(r:q+Nextra,1)';
