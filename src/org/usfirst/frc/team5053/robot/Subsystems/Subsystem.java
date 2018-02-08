package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.PIDController;

public interface Subsystem
{
	/*
	 *  Serves as a template that includes all methods a subsystem should have
	 *  Mostly used for PID controller wrapping and debugging output functions
	 */
	public void WriteDashboardData();
	
	//Related to closed loop feedback
	public boolean isClosedLoopControl();
	public boolean enableClosedLoopControl(double target, double speed, double ramp);
	public boolean disableClosedLoopControl();
	public boolean isOnTarget();
	
}
