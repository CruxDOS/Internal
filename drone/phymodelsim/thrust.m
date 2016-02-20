% Function: return thrust
% input: current inputs, thrust coefficient
function result = thrust(inputs, k)
	result = [0; 0; k*sum(inputs)];
end
