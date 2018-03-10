package org.usfirst.frc.team5053.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator implements Subsystem {

	private TalonSRX m_Talon;
	private DigitalInput m_LimitHigh;
	private DigitalInput m_LimitLow;
	
	private final double kp = 0.08;
	private final double ki = 0.0;
	private final double kd = 0.0;
	
	private double m_PositionTarget = 1;
	private final double TOLERANCE = 200;
	
	
	
	
	public Elevator(TalonSRX speedController, DigitalInput limitHigh, DigitalInput limitLow)
	{
		m_Talon = speedController;
		m_LimitHigh = limitHigh;
		m_LimitLow = limitLow;
		
		m_Talon.config_kP(0, kp, 4000);
		m_Talon.config_kI(0, ki, 4000);
		m_Talon.config_kD(0, kd, 4000);
		
		
		m_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 4000);
		m_Talon.setNeutralMode(NeutralMode.Brake);
	}
	public void manualControl(double speed)
	{
		// Runs at half speed for manual control
		if (speed < 0 && !m_LimitHigh.get()) {
			speed = 0;
		} else if (speed > 0 && !m_LimitLow.get()) {
			speed = 0;
		}
		
		System.out.println("Limit High: " + m_LimitHigh.get());
		System.out.println("Limit Low: " + m_LimitLow.get());
		m_Talon.set(ControlMode.PercentOutput, speed);
	}
	public boolean getLimitHigh() {
		return m_LimitHigh.get();
	}
	public boolean getLimitLow() {
		return m_LimitLow.get();
	}
	public void setPosition(double position)
	{
		m_PositionTarget = position;
		m_Talon.set(ControlMode.Position, position);
	}
	
	public void disablePID()
	{
		m_Talon.set(ControlMode.PercentOutput, 0);
	}
	public void resetEncoder()
	{
		m_Talon.setSelectedSensorPosition(0, 0, 4000);
	}
	public boolean isPIDEnabled()
	{
		return m_Talon.getControlMode().equals(ControlMode.Position);
	}
	public double getPositionTarget() 
	{
		return m_PositionTarget;
	}
	public boolean isPIDOnTarget()
	{
		return (Math.abs(getCurrentPosition()) >= (Math.abs(getPositionTarget() - TOLERANCE))); 
	}
	public double getCurrentPosition()
	{
		return m_Talon.getSelectedSensorPosition(0);
	}
	@Override
	public void WriteDashboardData() 
	{
		SmartDashboard.putNumber("Elevator Target", getPositionTarget());
		SmartDashboard.putNumber("Elevator Encoder Position", getCurrentPosition());
		SmartDashboard.putString("SRX State", m_Talon.getControlMode().toString());
	}

}
