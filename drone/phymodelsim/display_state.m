% Function: Compute data for displaying
% input: body sate, rotate velocity
function result = display_state(DATA, dispparam, phyparam)

	d_t_plot = dispparam.time_delta/dispparam.speed;
	t_int = d_t_plot/100; % time interval to check timer, in seconds
	
	% calculate limit of coordinates
	axlim = max(max(abs([DATA(:).X]))) + 2*phyparam.L;
	ax_limit = [-axlim axlim -axlim axlim -axlim axlim];
	
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
	
	tic;
	for tc = 1:dispparam.time_N
	
		x_plot    = DATA(tc).X;
		m1x_plot  = DATA(tc).M1x;
		m2x_plot  = DATA(tc).M2x;
		m3x_plot  = DATA(tc).M3x;
		m4x_plot  = DATA(tc).M4x;
		f1x_plot  = DATA(tc).F1x;
		f2x_plot  = DATA(tc).F2x;
		f3x_plot  = DATA(tc).F3x;
		f4x_plot  = DATA(tc).F4x;
		fa1x_plot = DATA(tc).Fa1x;
		fa2x_plot = DATA(tc).Fa2x;
		fa3x_plot = DATA(tc).Fa3x;
		fa4x_plot = DATA(tc).Fa4x;
		
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
		
		tic;
	    
	end
	hold off;
	
	
	%% Plot dynamics change
	
	fig2hd = figure(2);
	set(fig2hd, 'Position', [1020 100 800 800])
	subplot(3,2,1);
	plot(dispparam.time_seq, rad2deg([DATA(:).Theta]));
	title('Angles vs. Time');
	legend('\alpha', '\beta', '\gamma');
	xlabel('time (s)'); ylabel('angles (^{o})');
	
	subplot(3,2,2);
	plot(dispparam.time_seq, rad2deg([DATA(:).D_theta]));
	title('Angular Velocities vs. Time');
	legend('v_{\alpha}', 'v_{\beta}', 'v_{\gamma}');
	xlabel('time (s)'); ylabel('angular velocities (^{o}/s)');
	
	subplot(3,2,3);
	plot(dispparam.time_seq, [DATA(:).X]);
	title('Coordinates vs. Time');
	legend('x', 'y', 'z');
	xlabel('time (s)'); ylabel('Coordinates (m)');
	
	subplot(3,2,4);
	plot(dispparam.time_seq, [DATA(:).V]);
	title('Velocities vs. Time');
	legend('v_{x}', 'v_{y}', 'v_{z}');
	xlabel('time (s)'); ylabel('velocities (m/s)');
	
	subplot(3,2,5);
	plot(dispparam.time_seq, [DATA(:).Rotate]);
	title('Controller Outputs (Forces) vs. Time');
	legend('motor #1', 'motor #2', 'motor #3', 'motor #4');
	xlabel('time (s)'); ylabel('Controller Outputs (Forces)');
	
	subplot(3,2,6);
	plot(dispparam.time_seq, [DATA(:).A]);
	title('Accelerations vs. Time');
	legend('a_{x}', 'a_{y}', 'a_{z}');
	xlabel('time (s)'); ylabel('acceleration (m/s^{2})');
	
	disp('Done!')

end
