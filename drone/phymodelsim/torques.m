% Function: return torques
% input: current inputs, length, drag coefficient, thrust coefficient
function result = torques(inputs, L, b, k)
	result = [
		L*k*(inputs(1) - inputs(3))
		L*k*(inputs(2) - inputs(4))
		b*(inputs(1) - inputs(2) + inputs(3) - inputs(4))
	];
end
