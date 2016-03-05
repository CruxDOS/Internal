% Function: compute state
% input: state, rotate, phyparam, time
function [state_next, a] = compute_state(state, rotate, time, phyparam)
	% Compute linear and angular acceleration
	omega = d_theta2omega(state.d_theta, state.theta);
	a = acceleration(rotate, state.theta, state.d_x, phyparam.m, phyparam.g, phyparam.k, phyparam.kd);
	d_omega = angular_acceleration(rotate, omega, phyparam.I, phyparam.L, phyparam.b, phyparam.k);

	% Data for next time slot
	omega = omega + time.delta*d_omega;
	state_next.theta = state.theta + time.delta*state.d_theta;
	state_next.d_x = state.d_x + time.delta*a;
	state_next.x = state.x + time.delta*state.d_x; 	% drone center coordinates in inertial frame
	state_next.d_theta = omega2d_theta(omega, state.theta);

end
