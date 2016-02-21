% Function: return angular accelaration
% input: r (motor angular velocity square), omega, I, L, b, k)
function d_omega = angular_acceleration(r, omega, I, L, b, k)
	tau = torques(r, L, b, k);
	d_omega = I\(tau - cross(omega, I*omega));
end
