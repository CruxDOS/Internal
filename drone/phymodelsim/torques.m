% Function: return torques
% input: r (motor angular velocity square), length, drag coefficient, thrust coefficient
function result = torques(r, L, b, k)
	result = [
		L*k*(r(1) - r(3));
		L*k*(r(2) - r(4));
		b*(r(1) - r(2) + r(3) - r(4));
	];
end
