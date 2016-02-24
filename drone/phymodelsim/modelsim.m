clear;
%% Simulation set up
sim_params;	% include sim_params.m
sim_setup;	% include sim_setup.m

%% Start simulation
for tc = 1:time.N 	% time count
	
	% Apply initial angular velocity at the begining
	if tc == 1
		d_theta = d_theta_init; 
	end

	% Controller
	[r, state] = pd_controller(state, params, d_theta, PD_param); % r: motor anglar velocity square

	% Compute linear and angular acceleration
	omega = d_theta2omega(d_theta, theta);
	a = acceleration(r, theta, d_x, phyparam.m, phyparam.g, phyparam.k, phyparam.kd);
	d_omega = angular_acceleration(r, omega, phyparam.I, phyparam.L, phyparam.b, phyparam.k);
	
	% Compute and save data
	R = rotation(theta);
	m1x = R*m1pos + x;	% m coordinates in inertial frame
	m2x = R*m2pos + x;
	m3x = R*m3pos + x;
	m4x = R*m4pos + x;
	f1v = R*dispparam.f_scale*[0; 0; r(1)];	% f vectors in body frame
	f2v = R*dispparam.f_scale*[0; 0; r(2)];
	f3v = R*dispparam.f_scale*[0; 0; r(3)];
	f4v = R*dispparam.f_scale*[0; 0; r(4)];
	f1x = f1v + m1x; 	% f coordinates in inertial frame
	f2x = f2v + m2x;
	f3x = f3v + m3x;
	f4x = f4v + m4x;
	fav = (f1v + f2v + f3v + f4v)/4; % average f vector
	fa1x = fav + m1x; 	% f coordinates in inertial frame
	fa2x = fav + m2x;
	fa3x = fav + m3x;
	fa4x = fav + m4x;

	X_data(:,tc) = x; 	% save drone center x data
	M1x_data(:,tc) = m1x; 	% save motor x data
	M2x_data(:,tc) = m2x;
	M3x_data(:,tc) = m3x;
	M4x_data(:,tc) = m4x;
	F1x_data(:,tc) = f1x; 	% save f x data
	F2x_data(:,tc) = f2x;
	F3x_data(:,tc) = f3x;
	F4x_data(:,tc) = f4x;
	Fa1x_data(:,tc) = fa1x; % save average f x data
	Fa2x_data(:,tc) = fa2x;
	Fa3x_data(:,tc) = fa3x;
	Fa4x_data(:,tc) = fa4x;
	R_data(:,tc) = r;
	A_data(:,tc) = a;

	Theta_data(:,tc) = theta;
	D_theta_data(:,tc) = d_theta/time.delta;
	V_data(:,tc) = time.delta*a;
	
	% Data for next time slot
	omega = omega + time.delta*d_omega;
	theta = theta + time.delta*d_theta;
	d_x = d_x + time.delta*a;
	x = x + time.delta*d_x; 	% drone center coordinates in inertial frame
	d_theta = omega2d_theta(omega, theta);
end

%% Displaying dynamics
d_t_plot = time.delta/dispparam.speed;
t_int = d_t_plot/100; % time interval to check timer, in seconds

% calculate limit of coordinates
axlim = max(max(abs(X_data))) + 2*phyparam.L;
ax_limit = [-axlim axlim -axlim axlim -axlim axlim];

disp('Initial diviation in angular velocity, in degrees/second:');
disp(['	x: ', num2str(rad2deg(d_theta_init(1)))]);
disp(['	y: ', num2str(rad2deg(d_theta_init(2)))]);
disp(['	z: ', num2str(rad2deg(d_theta_init(3)))]);
disp('');
disp('Start displaying...');
fig1hd = figure(1);
set(fig1hd, 'Position', [200 100 800 800])
% plot contrast lines
plot3([ 0         ;  0         ], [-axlim     ;  axlim     ], [-axlim     ; -axlim     ], '-k', 'LineWidth', 0.5);
hold on;
plot3([-axlim     ;  axlim     ], [ 0         ;  0         ], [-axlim     ; -axlim     ], '-k', 'LineWidth', 0.5);
plot3([ 0         ;  0         ], [ axlim     ;  axlim     ], [-axlim     ;  axlim     ], '-k', 'LineWidth', 0.5);
plot3([-axlim     ;  axlim     ], [ axlim     ;  axlim     ], [ 0         ;  0         ], '-k', 'LineWidth', 0.5);
plot3([ axlim     ;  axlim     ], [ 0         ;  0         ], [-axlim     ;  axlim     ], '-k', 'LineWidth', 0.5);
plot3([ axlim     ;  axlim     ], [-axlim     ;  axlim     ], [ 0         ;  0         ], '-k', 'LineWidth', 0.5);
axis(ax_limit);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');

for tc = 1:time.N
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
	fa1x_plot = Fa1x_data(:,tc);
	fa2x_plot = Fa2x_data(:,tc);
	fa3x_plot = Fa3x_data(:,tc);
	fa4x_plot = Fa4x_data(:,tc);
	
	handle1 = findall (gca, 'LineWidth', 2);
	delete(handle1);
	if tc ~= 1
		delete(h0);
	end

	% plot projections
	plot3([m1x_plot(1); m3x_plot(1)], [m1x_plot(2); m3x_plot(2)], [-axlim     ; -axlim     ], '-k', 'LineWidth', 2);
	plot3([m2x_plot(1); m4x_plot(1)], [m2x_plot(2); m4x_plot(2)], [-axlim     ; -axlim     ], '-k', 'LineWidth', 2);
	plot3([m1x_plot(1); m3x_plot(1)], [axlim      ; axlim      ], [m1x_plot(3); m3x_plot(3)], '-k', 'LineWidth', 2);
	plot3([m2x_plot(1); m4x_plot(1)], [axlim      ; axlim      ], [m2x_plot(3); m4x_plot(3)], '-k', 'LineWidth', 2);
	plot3([axlim      ; axlim      ], [m1x_plot(2); m3x_plot(2)], [m1x_plot(3); m3x_plot(3)], '-k', 'LineWidth', 2);
	plot3([axlim      ; axlim      ], [m2x_plot(2); m4x_plot(2)], [m2x_plot(3); m4x_plot(3)], '-k', 'LineWidth', 2);

	% plot drone body
	plot3([m1x_plot(1); m3x_plot(1)], [m1x_plot(2); m3x_plot(2)], [m1x_plot(3); m3x_plot(3)], '-b', 'LineWidth', 2);
	plot3([m2x_plot(1); m4x_plot(1)], [m2x_plot(2); m4x_plot(2)], [m2x_plot(3); m4x_plot(3)], '-b', 'LineWidth', 2);

	% plot motor forces 
	plot3([m1x_plot(1); f1x_plot(1)], [m1x_plot(2); f1x_plot(2)], [m1x_plot(3); f1x_plot(3)], '-r', 'LineWidth', 2);
	plot3([m2x_plot(1); f2x_plot(1)], [m2x_plot(2); f2x_plot(2)], [m2x_plot(3); f2x_plot(3)], '-r', 'LineWidth', 2);
	plot3([m3x_plot(1); f3x_plot(1)], [m3x_plot(2); f3x_plot(2)], [m3x_plot(3); f3x_plot(3)], '-r', 'LineWidth', 2);
	plot3([m4x_plot(1); f4x_plot(1)], [m4x_plot(2); f4x_plot(2)], [m4x_plot(3); f4x_plot(3)], '-r', 'LineWidth', 2);

	% plot average force point
	h0 = plot3(	[fa1x_plot(1); fa2x_plot(1); fa3x_plot(1); fa4x_plot(1)], ...
			[fa1x_plot(2); fa2x_plot(2); fa3x_plot(2); fa4x_plot(2)], ...
			[fa1x_plot(3); fa2x_plot(3); fa3x_plot(3); fa4x_plot(3)], ...
			'og', 'MarkerSize', 5);


	% plot drone center
	%plot3(x_plot(1),x_plot(2),x_plot(3),'or','LineWidth',4);


	if toc > d_t_plot
		disp('Warning: Plotting time longer than real time');
	end

	while(toc < d_t_plot)
		pause(t_int);
	end
    
end
hold off;


%% Plot dynamics change

fig2hd = figure(2);
set(fig2hd, 'Position', [1020 100 800 800])
subplot(3,2,1);
plot(time.seq, rad2deg(Theta_data));
title('Angles vs. Time');
legend('\alpha', '\beta', '\gamma');
xlabel('time (s)'); ylabel('angles (^{o})');

subplot(3,2,2);
plot(time.seq, rad2deg(D_theta_data));
title('Angular Velocities vs. Time');
legend('v_{\alpha}', 'v_{\beta}', 'v_{\gamma}');
xlabel('time (s)'); ylabel('angular velocities (^{o}/s)');

subplot(3,2,3);
plot(time.seq, X_data);
title('Coordinates vs. Time');
legend('x', 'y', 'z');
xlabel('time (s)'); ylabel('Coordinates (m)');

subplot(3,2,4);
plot(time.seq, V_data);
title('Velocities vs. Time');
legend('v_{x}', 'v_{y}', 'v_{z}');
xlabel('time (s)'); ylabel('velocities (m/s)');

subplot(3,2,5);
plot(time.seq, R_data);
title('Controller Outputs (Forces) vs. Time');
legend('motor #1', 'motor #2', 'motor #3', 'motor #4');
xlabel('time (s)'); ylabel('Controller Outputs (Forces)');

subplot(3,2,6);
plot(time.seq, A_data);
title('Accelerations vs. Time');
legend('a_{x}', 'a_{y}', 'a_{z}');
xlabel('time (s)'); ylabel('acceleration (m/s^{2})');

disp('Done!')
