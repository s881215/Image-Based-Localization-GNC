1. The MATLAB/Simulink code in this file is used to verify the designed control law (Input–Output Control).

2. performance.m : 
	a. Specify the desired Mp (maximum overshoot, %) and Ts (2% settling time).
	b. Specify the desired ωd (desired angular velocity, rad/s) and vd (desired linear velocity, m/s).
	c. The script computes the controller parameters kx, ky, and kθ.
	d. Update kx, ky, and kθ in one_step.slx.