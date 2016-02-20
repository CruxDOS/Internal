% Function: return d_theta
% input: omega, theta
function result = omega2d_theta(omega, theta)
	M = [
		1	0		-sin(theta(2))			;
		0	cos(theta(1))	cos(theta(2))*sin(theta(1))	;
		0	-sin(theta(1))	cos(theta(2))*cos(theta(1))	
	];
	
	result = M \ omega;
end
