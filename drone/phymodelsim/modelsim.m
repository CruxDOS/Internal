clear;
%% Simulation set up
% Simulation time, in seconds
start_time = 0;
end_time = 10;
d_time = 0.1;

% Initial body state
x = [0; 0; 10];
d_x = zeros(3,1);
theta = zeros(3,1);

% Physical parameters
m = 1.2;
L = 0.13;
g = 10;
b = 0.05;
k = 1;
kd = 1;
I_xx = 1; % Inertia
I_yy = 1;
I_zz = 2;

I = [	I_xx	0	0	;
	0	I_yy	0	;
	0	0	I_zz	];

% Simulate disturbulance in angular velocity, deviation in radians/second
deviation = 100;
d_theta = deg2rad(2*deviation*rand(3,1) - deviation);


%% Simulating
time_seq = start_time:d_time:end_time;
N_time = numel(time_seq);

% Matrices to save data
X_data = zeros(3, N_time);
Theta_data = zeros(3, N_time);

for timecount = 1:N_time
	
	t = time_seq(timecount);
	disp(t);
	i = 2*ones(4,1); %input(t);

	omega = d_theta2omega(d_theta, theta);

	% Compute linear and angular accelerations.
	a = acceleration(i, theta, d_x, m, g, k, kd);
	d_omega = angular_accelaration(i, omega, I, L, b, k);

	omega = omega + d_time*d_omega;
	d_theta = omega2d_theta(omega, theta);
	theta = theta + d_time*d_theta;
	d_x = d_x + d_time*a;
	x = x + d_time*d_x;

	% Save data
	X_data(:,timecount) = x;
	Theta_data(:,timecount) = theta;

end

