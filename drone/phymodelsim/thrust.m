% Function: return thrust
% input: r (motor angular velocity square), thrust coefficient
function result = thrust(r, k)
	result = [0; 0; k*sum(r)];
end
