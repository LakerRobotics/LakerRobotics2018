package org.usfirst.frc.team5053.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;

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
	private final int leftDrive1PWM 	= 7;
	private final int leftDrive2PWM 	= 8;
	private final int rightDrive1PWM 	= 3;
	private final int rightDrive2PWM 	= 4;
	private final int intakeLeftPWM 	= 0;
	private final int intakeRightPWM 	= 1;
	
	private final int elevatorCAN = 1;
	
	private final int catapultLeftExpandSolenoidSlot 	= 0;
	private final int catapultLeftRetractSolenoidSlot 	= 1;
	private final int catapultRightExpandSolenoidSlot 	= 2;
	private final int catapultRightRetractSolenoidSlot 	= 3;
	private final int intakeExpandSolenoidSlot			= 4;
	private final int intakeRetractSolenoidSlot 		= 5;
	

	private Talon m_LeftDrive1;
	private Talon m_LeftDrive2;
	private Talon m_RightDrive1;
	private Talon m_RightDrive2;
	private Talon m_Intake1;
	private Talon m_Intake2;

	private TalonSRX m_Elevator;
	
	private DoubleSolenoid m_CatapultLeft;
	private DoubleSolenoid m_CatapultRight;
	private DoubleSolenoid m_IntakeSolenoid;
	
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
		m_LeftDrive.setInverted(false); // True
		
		m_RightDrive1 = new Talon(rightDrive1PWM);
		m_RightDrive2 = new Talon(rightDrive2PWM);
		m_RightDrive = new SpeedControllerGroup(m_RightDrive1, m_RightDrive2);
		m_RightDrive.setInverted(false); // True
		
		m_Elevator = new TalonSRX(elevatorCAN);
		m_Elevator.setInverted(true);
		
		m_Intake1 = new Talon(intakeLeftPWM);
		m_Intake2 = new Talon(intakeRightPWM);
		
		m_CatapultLeft = new DoubleSolenoid(catapultLeftExpandSolenoidSlot, catapultLeftRetractSolenoidSlot);
		m_CatapultRight = new DoubleSolenoid(catapultRightExpandSolenoidSlot, catapultRightRetractSolenoidSlot);
		m_IntakeSolenoid = new DoubleSolenoid(intakeExpandSolenoidSlot, intakeRetractSolenoidSlot);
	}
	
	public SpeedControllerGroup getLeftDriveGroup()
	{
		return m_LeftDrive;
	}
	public SpeedControllerGroup getRightDriveGroup()
	{
		return m_RightDrive;
	}
	public TalonSRX getElevator() {
		return m_Elevator;
	}
	public Talon getLeftIntake() {
		return m_Intake1;
	}
	public Talon getRightIntake() {
		return m_Intake2;
	}
	
	public DoubleSolenoid getCatapultLeft() {
		return m_CatapultLeft;
	}
	public DoubleSolenoid getCatapultRight() {
		return m_CatapultRight;
	}
	public DoubleSolenoid getIntakeSolenoid() {
		return m_IntakeSolenoid;
	}
}