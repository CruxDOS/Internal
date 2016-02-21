% Compute controller output and updated state.
function [r, integral] = pd_controller(integral, params, d_theta, PD_param)
	
	d_t = params.d_t;
	m = params.m;
	g = params.g;
	k = params.k;
	L = params.L;
	b = params.b;
	I_xx = params.I_xx;
	I_yy = params.I_yy;
	I_zz = params.I_zz;

	Kd = PD_param.Kd;
	Kp = PD_param.Kp;
	
	% Update the state
	integral = integral + d_t .* d_theta;

	% Compute total thrust
	thrust_ave = m*g/(4*k*(cos(integral(1))*cos(integral(2))));

	% Compute errors
	e = Kd*d_theta + Kp*integral;

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
		else if r_int(i) > 2*m*g/4
			r_int(i) = 2*m*g/4;
		end
	end
	r = r_int;
end
