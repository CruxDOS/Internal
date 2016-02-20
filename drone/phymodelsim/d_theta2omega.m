% Function: return omega
% input: d_theta, theta
function result = d_theta2omega(d_theta, theta)
	M = [
		1	0		-sin(theta(2))			;
		0	cos(theta(1))	cos(theta(2))*sin(theta(1))	;
		0	-sin(theta(1))	cos(theta(2))*cos(theta(1))	
	];
	
	result = M * d_theta;
end
