clear;
%% Simulation set up
sim_params;	% include sim_params.m
sim_setup;	% include sim_setup.m

%% Start simulation
for tc = 1:time.N 	% time count
	
	% Apply initial angular velocity at the begining
	if tc == 1
		state.d_theta = d_theta_init; 
	else
		state = state_next;
	end

	% Controller
	[rotate, ctrlstate] = pd_controller(ctrlstate, phyparam, time, state.d_theta, PD_param);

	% Compute next state
	[state_next, a] = compute_state(state, rotate, time, phyparam);

	% Compute data and save
	DATA(tc) = compute_data(state, rotate, a, time, phyparam, dispparam);

end


%% Displaying dynamics
display_state(DATA, time, dispparam, phyparam);
