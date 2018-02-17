package org.usfirst.frc.team5053.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;

public class Elevator implements Subsystem {

	private TalonSRX m_Talon;
	private DigitalInput m_LimitTop;
	private DigitalInput m_LimitBottom;
	
	PIDController m_PID;
	private final double kp = 0.0;
	private final double ki = 0.0;
	private final double kd = 0.0;
	private double m_PositionTarget;
	
	
	
	public Elevator(TalonSRX speedController/*, DigitalInput limitTop, DigitalInput limitBottom*/)
	{
		m_Talon = speedController;
		
		//m_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 4000);
		
		
		//m_LimitTop = limitTop;
		//m_LimitBottom = limitBottom;
		
		//m_PID = new PIDController(kp, ki, kd, m_Encoder, m_Talon);
	}
	public void manualControl(double speed)
	{
		// Runs at half speed for manual control
		m_Talon.set(ControlMode.PercentOutput, speed);
		System.out.println(m_Talon.getDeviceID() + " " + speed);
	}
	
	public void setPosition(double position)
	{
		m_PositionTarget = position;
		//m_Talon.set(ControlMode.Position, position);
	}
	
	public void disablePID()
	{
		//m_Talon.disable();
		//m_Talon.free();
	}
	
	public boolean isPIDEnabled()
	{
		//return m_Talon.isAlive();
		return false;
	}
	public double getPositionTarget() {
		return m_PositionTarget;
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
