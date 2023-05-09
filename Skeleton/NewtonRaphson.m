function b = NewtonRaphson(FunFcn,x,p,tol,relax)
% NewtonRaphson(FunFcn,x,p,tol,relax)
% 
% NewtonRaphson Solves a set of functions (linear or non linear) by means of
% Newton-Raphson iteration. 
% FunFcn is a string holding name of function
% x      is initial guess on variables
% p      is a set of fixed parameters
% tol    is the tolerance to be met in iteration
% relax  is the relaxation factor for the change in the coordinate vector
% 
% Michael Rygaard Hansen
% Aalborg University, 1999
%
% modified by
% Niels L. Pedersen 2003
% 
% Explanation added by
% Morten Kjeld Ebbesen 2005

% Initialization
if nargin < 4 | isempty(tol), tol = 1e-10; end
if nargin < 5 | isempty(relax), relax = 1.0; end

% Get rank of problem
n = length(x);

% Set perturbation
pert = 1e-10;

% Initialize counter
i = 0;

% Initial residual vector computation
b = feval(FunFcn, x, p);
n2 = length(b);

% Do Newton-Raphson iteration procedure. Stop when counter = 100 or
% tolerance met.
while norm(b)>tol & i<100

  % Increment counter
  i = i+1;

  % Generate jacobian
  for j=1:n
    temp=x(j);  
    if x(j)==0  
      x(j) = x(j)+pert;
      delta=pert;
    else
      x(j) = x(j)*(1.0+pert);
      delta = pert*temp;
    end  
    bPert = feval(FunFcn, x, p);
    x(j) = temp;
    for k=1:n
      jacob(k,j)=(bPert(k)-b(k))/delta;
    end
  end  

  % Do correction
  x = x - relax*(jacob\b);
  b = feval(FunFcn, x, p);
  c = norm(b);
end
MKE = jacob
b = x;