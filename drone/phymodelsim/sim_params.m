%% Simulation parameters & constants 
% This file is included in modelsim.m

%% Time
time.start = 0;		% start time
time.end   = 5;	% end time
time.delta = 0.1/1000;	% time step

%% Body state, set initail distrubance here
state.x       = zeros(3,1);
state.d_x     = zeros(3,1);
state.theta   = zeros(3,1);
state.d_theta = zeros(3,1);

% Initial distrubance
% Simulate disturbulance in angular velocity
% Format: "[xd, yd, zd]" or "d"
theta_rand        = 0;
d_theta_devi_rand = 20;				% in degree/second for randomization
d_theta_devi_init = [8.3746; 10.1875; -8.9590];	% in degree/second for non-randomization

%% Controller gains, tune by hand
ctrlparam.Kd    = 2;
ctrlparam.Kp    = 5;
ctrlparam.delay = 12/1000;
ctrlparam.time_delta = 12/1000;
% Controller 

%% Display
dispparam.f_scale    = 0.05;	% force scale for displaying
dispparam.speed      = 1;	% display speed
dispparam.time_delta = 50/1000;	% display time step

%% Physical constants
phyparam.m    = 1.2;	% mass
phyparam.L    = 0.13;	% distance from center of drone to motors
phyparam.g    = 10;	% gravity acceleration
phyparam.b    = 0.05;	% yaw constant, torque = k*omega^2
phyparam.k    = 1;	% thrust constant
phyparam.kd   = 1;	% air friction constant
phyparam.I_xx = 0.2;	% inertia along xx
phyparam.I_yy = 0.2;	% inertia along yy
phyparam.I_zz = 0.4;	% inertia along zz

%% m position in body frame
phyparam.m1ang = 0;	% angular position of four motors, in radians
phyparam.m2ang = pi/2;
phyparam.m3ang = pi;
phyparam.m4ang = 3*pi/2;
