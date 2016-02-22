% Function: return rotation matrix
% input: angles
function result = rotation(angles)
	s1 = sin(angles(1));
	c1 = cos(angles(1));
	s2 = sin(angles(2));
	c2 = cos(angles(2));
	s3 = sin(angles(3));
	c3 = cos(angles(3));

	% ZXZ
	%result = [
	%	c1*c3-c2*s1*s3	-c3*s1-c1*c2*s3	s2*s3	;
	%	c2*c3*s1+c1*s3	c1*c2*c3-s1*s3	-c3*s2  ;
	%	s1*s2		c1*s2		c2	
	%];

	% ZYZ
	%result = [
	%	c1*c2*c3-s1*s3	-c3*s1-c1*c2*s3	c1*s2	;
	%	c1*s3+c2*c3*s1	c1*c3-c2*s1*s3	s1*s2   ;
	%	-c3*s2		s2*s3		c2	
	%];
	
	% XYX
	%result = [
	%	c2	s2*s3		c3*s2		;
	%	s1*s2	c1*c3-c2*s1*s3	-c1*s3-c2*c3*s1	;
	%	-c1*s2	c3*s1+c1*c2*s3	c1*c2*c3-s1*s3
	%];

	% YXY
	result = [
		c1*c3-c2*s1*s3	s1*s2	c1*s3+c2*c3*s1	;
		s2*s3		c2	-c3*s2		;
		-c3*s1-c1*c2*s3	c1*s2	c1*c2*c3-s1*s3
	];

	% tait-bryan angles
	% XYZ
	%result = [
	%	c2*c3		-s2	c2*s3		;
	%	s1*s3+c1*c3*s2	c1*c2	c1*s2*s3-c3*s1	;
	%	c3*s1*s2-c1*s3	c2*s1	c1*c3+s1*s2*s3	;
	%];

	% ZYX
	%result = [
	%	c1*c2	c1*s2*s3-c3*s1	s1*s3+c1*c3*s2	;
	%	c2*s1	c1*c3+s1*s2*s3	c3*s1*s2-c1*s3	;
	%	-s2	c2*s3		c2*c3
	%];



end
