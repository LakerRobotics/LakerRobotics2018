package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.Talon;
import com.ctre.CANTalon;

/**
 * Maps all of the output controllers on the robot.
 * These include the following:
 * Talons,
 * Victors,
 * Jaguars,
 * Spikes,
 * Solenoids
 */

//Sensors are located and handled by the RobotSensorMap class

public class RobotControllerMap
{
	private final int leftDrivePWM = 0;
	private final int rightDrivePWM = 1;

	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	
	/**
	 * 
	 */
	
	public RobotControllerMap()
	{
		
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		
		m_LeftDrive.setInverted(true);
		m_RightDrive.setInverted(true);
	}
	
	public Talon getLeftDrive()
	{
		return m_LeftDrive;
	}
	public Talon getRightDrive()
	{
		return m_RightDrive;
	}
}