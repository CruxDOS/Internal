% Function: Compute data for displaying
% input: body sate, rotate velocity
function data = compute_data(state, rotate, a, time, phyparam, dispparam)

	R    = rotation(state.theta);
	m1x  = R*phyparam.m1pos + state.x;		% m coordinates in inertial frame
	m2x  = R*phyparam.m2pos + state.x;
	m3x  = R*phyparam.m3pos + state.x;
	m4x  = R*phyparam.m4pos + state.x;
	f1v  = R*dispparam.f_scale*[0; 0; rotate(1)];	% f vectors in body frame
	f2v  = R*dispparam.f_scale*[0; 0; rotate(2)];
	f3v  = R*dispparam.f_scale*[0; 0; rotate(3)];
	f4v  = R*dispparam.f_scale*[0; 0; rotate(4)];
	f1x  = f1v + m1x; 				% f coordinates in inertial frame
	f2x  = f2v + m2x;
	f3x  = f3v + m3x;
	f4x  = f4v + m4x;
	fav  = (f1v + f2v + f3v + f4v)/4; 		% average f vector
	fa1x = fav + m1x; 				% f coordinates in inertial frame
	fa2x = fav + m2x;
	fa3x = fav + m3x;
	fa4x = fav + m4x;

	data.X       = state.x; 			% save drone center x data
	data.M1x     = m1x; 				% save motor x data
	data.M2x     = m2x;
	data.M3x     = m3x;
	data.M4x     = m4x;
	data.F1x     = f1x; 				% save f x data
	data.F2x     = f2x;
	data.F3x     = f3x;
	data.F4x     = f4x;
	data.Fa1x    = fa1x; 				% save average f x data
	data.Fa2x    = fa2x;
	data.Fa3x    = fa3x;
	data.Fa4x    = fa4x;
	data.Rotate  = rotate;				% motor rotate speed
	data.A       = a;				% acceleration
	data.Theta   = state.theta;
	data.D_theta = state.d_theta/time.delta;
	data.V       = time.delta*a;

end
