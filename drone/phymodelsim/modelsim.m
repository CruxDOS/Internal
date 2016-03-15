clear;
%% Simulation set up
sim_params;	% include sim_params.m
sim_setup;	% include sim_setup.m

%% Start simulation
for t = time.seq 	% time sequence
	
	time.t = t;
	time.tc = time.tc + 1; % Increment time count

	% Apply initial angular velocity at the begining
	if time.t == time.start
		state.d_theta = d_theta_init; 
	else
		state = state_next;
	end

	% Controller
	if mod(time.t, ctrlparam.time_delta) == 0
		ctrlstate = controller(ctrlstate, phyparam, time, state.d_theta, ctrlparam);
	end

	% Check controller output queue
	size_out_q = size(ctrlstate.out_q);
	if size_out_q(2) ~= 0 && ctrlstate.out_q(1,1) <= time.t
		rotate = ctrlstate.out_q(2:end,1);
		ctrlstate.out_q = ctrlstate.out_q(:,2:end);
	end

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
