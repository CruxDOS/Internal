clear;
%% Simulation set up
% Simulation time, in seconds
start_time = 0;
end_time = 1;
d_time = 0.05;
f_scale = 0.05; 	% force scale for displaying

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
I_xx = 1; % Inertia
I_yy = 1;
I_zz = 2;


% Simulate disturbulance in angular velocity, deviation in radians/second
deviation = 10;
d_theta = deg2rad(2*deviation*rand(3,1) - deviation);


%% Simulating
time_seq = start_time:d_time:end_time;
N_time = numel(time_seq);

% Inertia matrix
I = [	I_xx	0	0	;
	0	I_yy	0	;
	0	0	I_zz	];

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


for tc = 1:N_time 	% time count
	
	t = time_seq(tc);
	i = 1.1* m*g/4*ones(4,1); %input(t);

	omega = d_theta2omega(d_theta, theta);

	% Compute linear and angular accelerations.
	a = acceleration(i, theta, d_x, m, g, k, kd);
	d_omega = angular_acceleration(i, omega, I, L, b, k);

	omega = omega + d_time*d_omega;
	d_theta = omega2d_theta(omega, theta);
	theta = theta + d_time*d_theta;
	d_x = d_x + d_time*a;
	x = x + d_time*d_x; 	% drone center coordinates in inertial frame

	% Compute and save data
	R = rotation(theta);
	m1x = R*m1pos + x;	% m coordinates in inertial frame
	m2x = R*m2pos + x;
	m3x = R*m3pos + x;
	m4x = R*m4pos + x;
	f1v = f_scale*[0; 0; i(1)];	% f vectors in body frame
	f2v = f_scale*[0; 0; i(2)];
	f3v = f_scale*[0; 0; i(3)];
	f4v = f_scale*[0; 0; i(4)];
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

end

%% Displaying
display_speed = 0.1;
d_time_plot = d_time/display_speed;
time_int = d_time_plot/100; % time interval to check timer, in seconds

% calculate limit of coordinates
axlim = max(max(abs(X_data)));
ax_limit = [-axlim axlim -axlim axlim -axlim axlim];

disp('Start displaying');
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

	figure(1);
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

	if toc > d_time_plot
		disp('Warning: Plotting time longer than real time');
	end

	while(toc < d_time_plot)
		pause(time_int);
	end
    
end





