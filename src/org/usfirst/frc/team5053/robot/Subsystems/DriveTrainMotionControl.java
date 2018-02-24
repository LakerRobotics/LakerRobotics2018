package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.AnglePIDWrapper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionController;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.SwingPIDWrapper;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * 
 *
 */

public class DriveTrainMotionControl extends DifferentialDrive implements Subsystem
{
	/**
	 * Hello There! : I'm the base constructor.
	 */

	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private MotionController m_MotionController;
	
	private ADXRS450_Gyro m_Gyro;
	
	public boolean isPIDRunning = false;
	
	// Normal PID stuff D:
	PIDController m_AnglePID;
	PIDController m_SwingPID;
	AnglePIDWrapper m_AnglePIDWrapper;
	private SwingPIDWrapper m_SwingPIDWrapper;
	private double m_Speed = 0.0;
	private double m_Turn = 0.0;
	private double m_swingTurnValue = 0.0;
	private boolean m_swingTurnRight = false;
	private final double SWING_TOLERANCE = 1.0;
	
	public DriveTrainMotionControl(SpeedControllerGroup leftMotorGroup, SpeedControllerGroup rightMotorGroup, Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro gyro)
	{
		super(leftMotorGroup, rightMotorGroup);
		
		//m_DriveTrain = new DriveTrain(leftMotorGroup, rightMotorGroup, leftEncoder, rightEncoder, gyro);
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_Gyro = gyro;
		
		m_MotionController = new MotionController(this, (PIDSource) m_LeftEncoder, (PIDSource) m_Gyro);
		
		m_AnglePIDWrapper = new AnglePIDWrapper(this);
		m_AnglePID = new PIDController(0.1, 0.0, 0.0, m_AnglePIDWrapper, m_AnglePIDWrapper);
		m_AnglePID.setAbsoluteTolerance(2.5);
		
		m_SwingPIDWrapper = new SwingPIDWrapper(this);
		
		m_SwingPID = new PIDController(0.07, 0.0, 0.0, m_SwingPIDWrapper, m_SwingPIDWrapper);
		m_SwingPID.setOutputRange(-0.75, 0.75);
		m_SwingPID.setAbsoluteTolerance(SWING_TOLERANCE);
	}
	
	
	
	public void setAngle(double angle)
	{
		m_AnglePID.setSetpoint(90);
	}
	public void setTurn(double turn)
	{
		ArcadeDrive(m_Speed, turn);
	}
	public boolean isTurnPIDOnTarget()
	{
		return Math.abs(m_AnglePID.getSetpoint() - GetAngle()) < 2.5;
	}
	public boolean enableTurnPID()
	{
		if(!m_AnglePID.isEnabled())
			m_AnglePID.enable();
		
		return m_AnglePID.isEnabled();
	}
	public boolean isTurnPIDEnabled()
	{
		return m_AnglePID.isEnabled();
	}
	public boolean disableTurnPID()
	{
		if(m_AnglePID.isEnabled())
			m_AnglePID.disable();
		
		return !m_AnglePID.isEnabled();
	}
	
	
	
	public void DriveDistance(double distance, double maxspeed, double ramp)
	{
		if(!isPIDRunning)
		{
			isPIDRunning = 	m_MotionController.ExecuteStraightMotion(distance, maxspeed, ramp);
		}
		
	}
	public void DriveControlledAngle(double distance, double maxspeed, double ramp, double angle)
	{
		if(!isPIDRunning)
			isPIDRunning = m_MotionController.ExecuteControlledAngleDriveMotion(distance, maxspeed, ramp, angle);
	}
	public void TurnToAngle(double turnAngle)
	{
		if(!isPIDRunning)
		{
			isPIDRunning = m_MotionController.ExecuteTurnMotion(turnAngle);
		}
		
	}
	public void DriveInArc(double distance, double maxspeed, double ramp, double radius)
	{
		if(!isPIDRunning)
		{
			isPIDRunning = m_MotionController.ExecuteArcMotion(distance, maxspeed, ramp, radius);
		}
	}
	public boolean isStraightPIDFinished()
	{
		if(m_MotionController.isStraightMotionFinished())
		{
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	public boolean isTurnPIDFinished() 
	{
		if(m_MotionController.isTurnMotionFinished())
		{
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	public boolean isArcPIDFinished()
	{
		if(m_MotionController.isArcMotionFinished())
		{
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	public void DisablePIDControl()
	{
		m_MotionController.DisablePIDControls();
	}
	public double GetRightDistance()
	{
		return m_RightEncoder.getDistance();
	}
	public double GetRightSpeed()
	{
		return m_RightEncoder.getRate();
	}
	public double GetLeftDistance()
	{
		return m_LeftEncoder.getDistance();
	}
	public double GetLeftSpeed()
	{
		return m_LeftEncoder.getRate();
	}
	public void ResetEncoders()
	{
		m_LeftEncoder.reset();
		m_RightEncoder.reset();
	}
	public void ResetGyro() 
	{
		m_Gyro.reset();
	}
	public double GetAverageSpeed()
	{
		return -(-GetLeftSpeed() + GetRightSpeed())/2;
	}
	public double GetAverageDistance()
	{
		return -(-GetLeftDistance() + GetRightDistance())/2;
	}
	public void ArcadeDrive(double speed, double angle)
	{
		this.m_Speed = speed;
		this.m_Turn = angle;
		
		this.arcadeDrive(speed, angle);
	}
	public double GetAngle()
	{
		PIDSourceType gyroType = m_Gyro.getPIDSourceType();
		m_Gyro.setPIDSourceType(PIDSourceType.kDisplacement);
		double angle = m_Gyro.getAngle();
		m_Gyro.setPIDSourceType(gyroType);
		return angle;
	}
	public double getAngularVelocity()
	{
		return m_Gyro.getRate();
	}
	public boolean StartSwingTurn() {
		try {
			if (!m_SwingPID.isEnabled()) {
				m_SwingPID.enable();
			}
			return true;
		} catch (Exception ex) {
			System.out.println(ex.getMessage());
		}
		return false;
		
	}
	public boolean SetSwingParameters(double angle, boolean isRight) {
		try {
			m_SwingPID.setSetpoint(angle);
			m_swingTurnRight = isRight;
			
			return true;
		} catch (Exception ex) {
			System.out.println(ex.getMessage());
		}
		return false;
	}
	public void SwingTurn(double turnSpeed) {
		m_swingTurnValue = turnSpeed;
		
		System.out.println("Swing turn speed: " + turnSpeed);
		
		if (m_swingTurnRight) 
		{
			this.tankDrive(0, turnSpeed);
		} 
		else 
		{
			this.tankDrive(-turnSpeed, 0);
		}
	}
	public boolean SwingAngleOnTarget()
	{
		if(Math.abs(GetSwingPIDSetpoint() - GetAngle()) < SWING_TOLERANCE)
		{
			return true;
		} else return false;
	}
	double GetSwingPIDSetpoint()
	{
		return m_SwingPID.getSetpoint();
	}
	public boolean disableSwingPID()
	{
		if(m_SwingPID.isEnabled())
			m_SwingPID.disable();
		
		return true;
	}
	public HashMap<String, Double> GetDashboardData() 
	{
		return null;
		// TODO Auto-generated method stub
		
	}
	
	public void WriteDashboardData() 
	{
		SmartDashboard.putNumber("Gyro Angle", m_Gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", m_Gyro.getRate());
		SmartDashboard.putNumber("LeftDriveEncoder Rate", m_LeftEncoder.getRate());
		
		// Do not change these names they are used for the DS Dashboard
		SmartDashboard.putNumber("leftDriveEncoder", m_LeftEncoder.getDistance());
		SmartDashboard.putNumber("rightDriveEncoder", m_RightEncoder.getDistance());
	}

}
