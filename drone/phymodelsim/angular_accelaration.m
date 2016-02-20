% Function: return angular accelaration
% input: current inputs, omega, I, L, b, k)
function d_omega = angular_accelaration(inputs, omega, I, L, b, k)
	tau = torques(inputs, L, b, k);
	d_omega = I\(tau - cross(omega, I*omega));
end
