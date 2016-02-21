% Function: convert from polar coordinates to rectangular coordinates
function result = pol2rect(r, theta)
	x = r*cos(theta)
	y = r*sin(theta)
	result = [x; y]
end
