package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.RobotSensorMap;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.AnglePIDWrapper;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionController;
import org.usfirst.frc.team5053.robot.Subsystems.Utilities.TalonSRXWrapper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
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

	//private Encoder m_LeftEncoder;
	//private Encoder m_RightEncoder;
	
	private MotionController m_MotionController;
	private WPI_TalonSRX m_rightTalon;
	private WPI_TalonSRX m_leftTalon;
	private TalonSRXWrapper m_talonWrapper;
	
	private ADXRS450_Gyro m_Gyro;
	
	public boolean isPIDRunning = false;
	
	// Normal PID stuff D:
	PIDController m_AnglePID;
	AnglePIDWrapper m_AnglePIDWrapper;
	private double m_Speed = 0.0;
	private double m_Turn = 0.0;
	
	private final int kPidId = 0;
	private final int kTimeOutMs = 0;
	
	public DriveTrainMotionControl(WPI_TalonSRX leftTalon, WPI_TalonSRX rightTalon, ADXRS450_Gyro gyro)
	{
		super(leftTalon, rightTalon);
		m_leftTalon = leftTalon;
		m_leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPidId,
				kTimeOutMs);
		
		m_rightTalon = rightTalon;
		m_rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPidId,
				kTimeOutMs);
		
		//m_DriveTrain = new DriveTrain(leftMotorGroup, rightMotorGroup, leftEncoder, rightEncoder, gyro);
		
		//m_LeftEncoder = leftEncoder;
		//m_RightEncoder = rightEncoder;
		m_talonWrapper = new TalonSRXWrapper(leftTalon);
		
		m_Gyro = gyro;
		
		m_MotionController = new MotionController(this, m_talonWrapper, (PIDSource) m_Gyro);
		
		m_AnglePIDWrapper = new AnglePIDWrapper(this);
		m_AnglePID = new PIDController(0.1, 0.0, 0.0, m_AnglePIDWrapper, m_AnglePIDWrapper);
		m_AnglePID.setAbsoluteTolerance(2.5);
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
		//return m_RightEncoder.getDistance();
		return m_rightTalon.getSelectedSensorPosition(0);
	}
	public double GetRightSpeed()
	{
		return m_rightTalon.getSelectedSensorVelocity(0);
	}
	public double GetLeftDistance()
	{
		return m_leftTalon.getSelectedSensorPosition(0);
	}
	public double GetLeftSpeed()
	{
		return m_leftTalon.getSelectedSensorVelocity(0);
	}
	public void ResetEncoders()
	{
		//m_LeftEncoder.reset();
		//m_RightEncoder.reset();
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
		return m_Gyro.getAngle();
	}
	public double getAngularVelocity()
	{
		return m_Gyro.getRate();
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
		//SmartDashboard.putNumber("LeftDriveEncoder Rate", m_LeftEncoder.getRate());
		SmartDashboard.putNumber("LeftDrive Rate", m_leftTalon.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("LeftDrive Distance", m_leftTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("RightDrive Rate", m_rightTalon.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("RightDrive Distance", m_rightTalon.getSelectedSensorPosition(0));
		// Do not change these names they are used for the DS Dashboard
		//SmartDashboard.putNumber("leftDriveEncoder", m_LeftEncoder.getDistance());
		//SmartDashboard.putNumber("rightDriveEncoder", m_RightEncoder.getDistance());
	}

}
