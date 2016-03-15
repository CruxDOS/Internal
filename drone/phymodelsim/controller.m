% Compute controller output and updated ctrlstate.
function ctrlstate = controller(ctrlstate, phyparam, time, d_theta, ctrlparam)
	
	theta        = ctrlstate.theta;
	prev_d_theta = ctrlstate.prev_d_theta;
	d_t          = ctrlparam.time_delta;

	m    = phyparam.m;
	g    = phyparam.g;
	k    = phyparam.k;
	L    = phyparam.L;
	b    = phyparam.b;
	I_xx = phyparam.I_xx;
	I_yy = phyparam.I_yy;
	I_zz = phyparam.I_zz;

	Kd    = ctrlparam.Kd;
	Kp    = ctrlparam.Kp;
	delay = ctrlparam.delay;
	
	% Update the ctrlstate
	theta = theta + d_t .* (prev_d_theta + d_theta)/2;

	% Compute total thrust
	thrust_ave = m*g/(4*k*(cos(theta(1))*cos(theta(2))));

	% Compute errors
	e = Kd*d_theta + Kp*theta;

	% Solve for the outputs
	r1 = thrust_ave + (- 2*b*e(1)*I_xx - e(3)*I_zz*k*L)/(4*b*k*L);
	r2 = thrust_ave + (- 2*b*e(2)*I_yy + e(3)*I_zz*k*L)/(4*b*k*L);
	r3 = thrust_ave + (  2*b*e(1)*I_xx - e(3)*I_zz*k*L)/(4*b*k*L);
	r4 = thrust_ave + (  2*b*e(2)*I_yy + e(3)*I_zz*k*L)/(4*b*k*L);
	r_int = [r1; r2; r3; r4];
	
	% eliminate negative and extremely large values
	for i = 1:4
		if r_int(i) < 0
			r_int(i) = 0;
		else
			if r_int(i) > 2*m*g/4
				r_int(i) = 2*m*g/4;
			end
		end
	end
	r = r_int;

	ctrlstate.out_q = [ctrlstate.out_q, [time.t+ctrlparam.delay; r]];
	ctrlstate.prev_d_theta = d_theta;
	ctrlstate.theta = theta;
end
