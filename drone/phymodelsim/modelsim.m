clear;
%% Simulation set up
% Simulation time, in seconds
start_time = 0;
end_t = 10;
d_t = 0.05;
f_scale = 0.05; 	% force scale for displaying
disp_speed = 1;

% Controller gains, tuned by hand
Kd = 2;
Kp = 5;


% Initial body state
x = [0; 0; 0];
d_x = zeros(3,1);
theta = zeros(3,1);

% Physical parameters
m = 1.2;
L = 0.13;
g = 10;
b = 0.05;
k = 1;
kd = 0.1;
I_xx = 0.2; % Inertia
I_yy = 0.2;
I_zz = 0.4;


% Simulate disturbulance in angular velocity, deviation in radians/second
deviation = 50;



%% Simulating
time_seq = start_time:d_t:end_t;
N_time = numel(time_seq);

% Inertia matrix
I = [	I_xx	0	0	;
	0	I_yy	0	;
	0	0	I_zz	];
state = struct(				...
	'theta',	{0},		...
	'prev_d_theta',	{0}	...
	);
params = struct( 	...
	'd_t', 	{d_t},	...
	'm', 	{m},    ...
	'g', 	{g},    ...
	'k', 	{k},    ...
	'L', 	{L},    ...
	'b', 	{b},    ...
	'I_xx',	{I_xx}, ...
	'I_yy',	{I_yy}, ...
	'I_zz',	{I_zz}  ...
	);

PD_param = struct(	...
	'Kd',	{Kd},	...
	'Kp',	{Kp}	...
	);

% m position in body frame
m1ang = 0;	% angular position of four ms, in radians
m2ang = pi/2;
m3ang = pi;
m4ang = 3*pi/2;
m1pos = [L*cos(m1ang); L*sin(m1ang); 0]; % convert to rectangular coordinates
m2pos = [L*cos(m2ang); L*sin(m2ang); 0];
m3pos = [L*cos(m3ang); L*sin(m3ang); 0];
m4pos = [L*cos(m4ang); L*sin(m4ang); 0];

% Matrices to save data
X_data = zeros(3, N_time);
M1x_data = zeros(3, N_time);
M2x_data = zeros(3, N_time);
M3x_data = zeros(3, N_time);
M4x_data = zeros(3, N_time);
F1x_data = zeros(3, N_time);
F2x_data = zeros(3, N_time);
F3x_data = zeros(3, N_time);
F4x_data = zeros(3, N_time);

Theta_data = zeros(3, N_time);


for tc = 1:N_time 	% time count
	
	% Compute linear and angular accelerations.
	if tc == 1
		d_theta = deg2rad(2*deviation*[rand(2,1); 0.5] - deviation); % apply disturbance at beginning
		state.prev_d_theta = d_theta; % assuming previous d_theta has the same value
	else
		d_theta = omega2d_theta(omega, theta);
	end

	% Controller
	[r, state] = pd_controller(state, params, d_theta, PD_param); % r: motor anglar velocity square

	% Compute linear and angular acceleration
	omega = d_theta2omega(d_theta, theta);
	a = acceleration(r, theta, d_x, m, g, k, kd);
	d_omega = angular_acceleration(r, omega, I, L, b, k);
	% Data for next time slot
	omega = omega + d_t*d_omega;
	theta = theta + d_t*d_theta;
	d_x = d_x + d_t*a;
	x = x + d_t*d_x; 	% drone center coordinates in inertial frame

	% Compute and save data
	R = rotation(theta);
	m1x = R*m1pos + x;	% m coordinates in inertial frame
	m2x = R*m2pos + x;
	m3x = R*m3pos + x;
	m4x = R*m4pos + x;
	f1v = f_scale*[0; 0; r(1)];	% f vectors in body frame
	f2v = f_scale*[0; 0; r(2)];
	f3v = f_scale*[0; 0; r(3)];
	f4v = f_scale*[0; 0; r(4)];
	f1x = R*f1v + m1x; 	% f coordinates in inertial frame
	f2x = R*f2v + m2x;
	f3x = R*f3v + m3x;
	f4x = R*f4v + m4x;

	X_data(:,tc) = x; 	% save drone center x data
	M1x_data(:,tc) = m1x; 	% save motor x data
	M2x_data(:,tc) = m2x;
	M3x_data(:,tc) = m3x;
	M4x_data(:,tc) = m4x;
	F1x_data(:,tc) = f1x; 	% save force x data
	F2x_data(:,tc) = f2x;
	F3x_data(:,tc) = f3x;
	F4x_data(:,tc) = f4x;

	Theta_data(:,tc) = theta;

end

%% Displaying dynamics
d_t_plot = d_t/disp_speed;
t_int = d_t_plot/100; % time interval to check timer, in seconds

% calculate limit of coordinates
axlim = max(max(abs(X_data))) + 2*L;
ax_limit = [-axlim axlim -axlim axlim -axlim axlim];

disp('Initial diviation in angular velocity, in radians/second:');
disp('x:' + d_theta(1));
disp('y:' + d_theta(2));
disp('z:' + d_theta(3));
disp('\n');
disp('Start displaying');
figure('Position', [400 100 800 800])

for tc = 1:N_time
	tic;

	x_plot = X_data(:,tc);
	m1x_plot = M1x_data(:,tc);
	m2x_plot = M2x_data(:,tc);
	m3x_plot = M3x_data(:,tc);
	m4x_plot = M4x_data(:,tc);
	f1x_plot = F1x_data(:,tc);
	f2x_plot = F2x_data(:,tc);
	f3x_plot = F3x_data(:,tc);
	f4x_plot = F4x_data(:,tc);

	% plot drone center
	%plot3(x_plot(1),x_plot(2),x_plot(3),'or','LineWidth',4);
	% plot drone body
	plot3([m1x_plot(1);m3x_plot(1)], [m1x_plot(2);m3x_plot(2)], [m1x_plot(3);m3x_plot(3)], '-b', 'LineWidth', 2);
	hold on;
	plot3([m2x_plot(1);m4x_plot(1)], [m2x_plot(2);m4x_plot(2)], [m2x_plot(3);m4x_plot(3)], '-b', 'LineWidth', 2);
	% plot motor forces
	plot3([m1x_plot(1);f1x_plot(1)], [m1x_plot(2);f1x_plot(2)], [m1x_plot(3);f1x_plot(3)], '-g', 'LineWidth', 2);
	plot3([m2x_plot(1);f2x_plot(1)], [m2x_plot(2);f2x_plot(2)], [m2x_plot(3);f2x_plot(3)], '-g', 'LineWidth', 2);
	plot3([m3x_plot(1);f3x_plot(1)], [m3x_plot(2);f3x_plot(2)], [m3x_plot(3);f3x_plot(3)], '-g', 'LineWidth', 2);
	plot3([m4x_plot(1);f4x_plot(1)], [m4x_plot(2);f4x_plot(2)], [m4x_plot(3);f4x_plot(3)], '-g', 'LineWidth', 2);
	
	% plot projections
	plot3([m1x_plot(1);m3x_plot(1)], [m1x_plot(2);m3x_plot(2)], [-axlim; -axlim], '-k', 'LineWidth', 2);
	plot3([m2x_plot(1);m4x_plot(1)], [m2x_plot(2);m4x_plot(2)], [-axlim; -axlim], '-k', 'LineWidth', 2);
	plot3([m1x_plot(1);m3x_plot(1)], [axlim; axlim], [m1x_plot(3);m3x_plot(3)], '-k', 'LineWidth', 2);
	plot3([m2x_plot(1);m4x_plot(1)], [axlim; axlim], [m2x_plot(3);m4x_plot(3)], '-k', 'LineWidth', 2);
	plot3([axlim; axlim], [m1x_plot(2);m3x_plot(2)], [m1x_plot(3);m3x_plot(3)], '-k', 'LineWidth', 2);
	plot3([axlim; axlim], [m2x_plot(2);m4x_plot(2)], [m2x_plot(3);m4x_plot(3)], '-k', 'LineWidth', 2);

	hold off;
	grid on;
	axis(ax_limit);

	if toc > d_t_plot
		disp('Warning: Plotting time longer than real time');
	end

	while(toc < d_t_plot)
		pause(t_int);
	end
    
end


%% Plot dynamics change

figure(2);
plot(time_seq, rad2deg(Theta_data));
title('Angle vs time');
legend('\phi ~ x', '\theta ~ y', '\gamma ~ z');
xlabel('time (s)'); ylabel('angle (^{o})');


