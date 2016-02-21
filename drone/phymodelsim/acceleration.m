% Function: return acceleration
% input: r (motor angular velocity square), angles, d_x, m, g, k, d_k
function result = acceleration(r, angles, d_x, m, g, k, kd)
	gravity = [0; 0; -g];
	R = rotation(angles);
	T = R*thrust(r, k);
	F_D = -kd*d_x;
	result = gravity + T/m + F_D/m;
end

