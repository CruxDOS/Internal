clear;
%% Simulation set up
sim_params;	% include sim_params.m
sim_setup;	% include sim_setup.m

%% Start simulation
for t = time.seq 	% time count
	
	time.t = t;
	time.tc = time.tc + 1; % Increment time count

	% Apply initial angular velocity at the begining
	if time.t == time.start
		state.d_theta = d_theta_init; 
	else
		state = state_next;
	end

	% Controller
	[rotate, ctrlstate] = controller(ctrlstate, phyparam, time, state.d_theta, ctrlparam);

	% Compute next state
	[state_next, a] = compute_state(state, rotate, time, phyparam);

	% Compute data and save
	if mod(time.t, dispparam.time_delta) == 0
		dispparam.tc = dispparam.tc + 1;
		DATA(dispparam.tc) = compute_data(state, rotate, a, time, phyparam, dispparam);
	end

end


%% Displaying dynamics
display_state(DATA, dispparam, phyparam);
