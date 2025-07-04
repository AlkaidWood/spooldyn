%% rungeKutta - Solve second-order ODEs using fourth-order Runge-Kutta method
%
% This function implements the classical fourth-order Runge-Kutta method 
% for solving second-order ordinary differential equations (ODEs) of the form:
%   d²y/dt² = f(t, y, dy/dt)
%
%% Syntax
%   [yn1, dyn1] = rungeKutta(fun, tn, yn, dyn)
%   [yn1, dyn1] = rungeKutta(fun, tn, yn, dyn, h)
%
%% Description
% |rungeKutta| advances the solution of a second-order ODE by one time step 
% using the fourth-order Runge-Kutta method. The method provides high accuracy 
% while maintaining numerical stability for a wide range of problems.
%
%% Input Arguments
% * |fun| - Function handle for the second derivative:
%   * Must accept three arguments: fun(t, y, dy/dt)
%   * Returns d²y/dt² = f(t, y, dy/dt)
%
% * |tn| - Current time value [scalar]:
%   * Independent variable at step n
%
% * |yn| - Current solution value [scalar]:
%   * Dependent variable at step n
%
% * |dyn| - Current first derivative [scalar]:
%   * dy/dt at step n
%
% * |h| - (Optional) Step size [scalar]:
%   * Default: 1
%   * Must be > 0
%
%% Output Arguments
% * |yn1| - Solution at next time step [scalar]:
%   * y(tₙ + h)
%
% * |dyn1| - First derivative at next time step [scalar]:
%   * dy/dt(tₙ + h)
%
%% Examples
% % Solve simple harmonic oscillator: d²y/dt² = -y
% fun = @(t,y,dy) -y; % Second derivative function
% 
% % Initial conditions
% t0 = 0; y0 = 1; dy0 = 0; h = 0.1;
% 
% % Single step solution
% [y1, dy1] = rungeKutta(fun, t0, y0, dy0, h);
% 
% % Multi-step solution for 10 steps
% t = t0; y = y0; dy = dy0;
% y_numerical = zeros(1,100);
% t_numerical = zeros(1,100);
% for i = 1:100
%     [y, dy] = rungeKutta(fun, t, y, dy, h);
%     t = t + h;
%     y_numerical(i) = y;
%     t_numerical(i) = t;
% end
% 
% % Compare with analytical solution (cos(t))
% t_vals = 0:h:10;
% y_analytical = cos(t_vals);
% 
% figure
% scatter(t_numerical, y_numerical); hold on
% plot(t_vals, y_analytical)
% legend('RK4', 'Analytical')
%
%% Implementation Notes
% 1. Method Characteristics:
%    * Fourth-order accuracy: O(h⁴) local truncation error
%    * Self-starting: Requires only initial conditions
%    * Fixed step size implementation
% 2. Stability:
%    * Conditionally stable for most problems
%    * Stability region larger than lower-order methods
% 3. Limitations:
%    * Step size h must be constant
%    * Not adaptive (error control not implemented)
%
%% Application Areas
% * Structural dynamics
% * Rotor dynamics simulations
% * Orbital mechanics
% * Electrical circuit analysis
% * Control system simulation
%
%% See Also
% ode45, ode23, ode113, diff, gradient
%
% Copyright (c) 2021-2025 Haopeng Zhang, Northwestern Polytechnical University, Politecnico di Milano
% This code is licensed under the MIT License. See the LICENSE file in the project root for the full text of the license.
%


function [yn1,dyn1] = rungeKutta(fun,tn,yn,dyn,h) 

%Check inputs
if nargin < 5
    h=1;
    if nargin < 4
         error('rungeKutta(): not enough inputs')
    end
end

%%
%input K11~K24
k11 = dyn;
k21 = fun(tn,yn,dyn);
k12 = dyn+h/2*k21;
k22 = fun(tn+h/2, yn+h/2*k11, dyn+h/2*k21);
k13 = dyn+h/2*k22;
k23 = fun(tn+h/2, yn+h/2*k12, dyn+h/2*k22);
k14 = dyn+h*k23;
k24 = fun(tn+h, yn+h*k13, dyn+h*k23);
yn1 = yn + h/6*(k11+ 2*k12+ 2*k13 +k14);
dyn1 = dyn + h/6*(k21 +2*k22 +2*k23 +k24);

end


