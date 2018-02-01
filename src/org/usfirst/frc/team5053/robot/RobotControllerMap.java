package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
	private final int leftDrive1PWM = 0;
	private final int leftDrive2PWM = 1;
	private final int rightDrive1PWM = 2;
	private final int rightDrive2PWM = 3;

	private Talon m_LeftDrive1;
	private Talon m_LeftDrive2;
	private Talon m_RightDrive1;
	private Talon m_RightDrive2;
	
	private SpeedControllerGroup m_LeftDrive;
	private SpeedControllerGroup m_RightDrive;
	
	/**
	 * 
	 */
	
	public RobotControllerMap()
	{
		
		m_LeftDrive1 = new Talon(leftDrive1PWM);
		m_LeftDrive2 = new Talon(leftDrive2PWM);
		m_LeftDrive = new SpeedControllerGroup(m_LeftDrive1, m_LeftDrive2);
		m_LeftDrive.setInverted(true);
		
		m_RightDrive1 = new Talon(rightDrive1PWM);
		m_RightDrive2 = new Talon(rightDrive2PWM);
		m_RightDrive = new SpeedControllerGroup(m_RightDrive1, m_RightDrive2);
		m_RightDrive.setInverted(true);
	}
	
	public SpeedControllerGroup getLeftDriveGroup()
	{
		return m_LeftDrive;
	}
	public SpeedControllerGroup getRightDriveGroup()
	{
		return m_RightDrive;
	}
}