%% Simulation parameters & constants 
% This file is included in modelsim.m

%% Time
time.start = 0;		% start time
time.end = 10;		% end time
time.delta = 0.05;	% time step

%% Body state, set initail distrubance here
x = zeros(3,1);
d_x = zeros(3,1);
theta = zeros(3,1);
d_theta = zeros(3,1);

% Initial distrubance
% Simulate disturbulance in angular velocity
% Format: "[xd, yd, zd]" or "d"
d_theta_devi = 20;	% in degree/second

%% Controller gains, tune by hand
Kd = 2;
Kp = 5;

%% Display
dispparam.f_scale = 0.05; 	% force scale for displaying
dispparam.speed = 1;		% display speed

%% Physical constants
phyparam.m = 1.2;	% mass
phyparam.L = 0.13;	% distance from center of drone to motors
phyparam.g = 10;	% gravity acceleration
phyparam.b = 0.05;	% yaw constant, torque = k*omega^2
phyparam.k = 1;		% thrust constant
phyparam.kd = 1;	% air friction constant
phyparam.I_xx = 0.2;	% inertia along xx
phyparam.I_yy = 0.2;	% inertia along yy
phyparam.I_zz = 0.4;	% inertia along zz

%% m position in body frame
m1ang = 0;	% angular position of four motors, in radians
m2ang = pi/2;
m3ang = pi;
m4ang = 3*pi/2;
