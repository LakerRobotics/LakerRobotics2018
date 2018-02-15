package org.usfirst.frc.team5053.robot.Subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Talon;

public class Intake implements Subsystem {

	private WPI_TalonSRX m_LeftIntake;
	private WPI_TalonSRX m_RightIntake;
	
	
	
	public Intake(WPI_TalonSRX leftTalon, WPI_TalonSRX rightTalon)
	{
		m_LeftIntake = leftTalon;
		m_RightIntake = rightTalon;
		
	}
	public void IntakeCube() {
		m_LeftIntake.set(1.0);
		m_RightIntake.set(-1.0);
	}
	public void ReleaseCube() {
		m_LeftIntake.set(-1.0);
		m_RightIntake.set(1.0);
	}
	public void RotateLeft() {
		m_LeftIntake.set(1.0);
		m_RightIntake.set(1.0);
	}
	public void RotateRight() {
		m_LeftIntake.set(-1.0);
		m_RightIntake.set(-1.0);
	}
	public void StopIntake() {
		m_LeftIntake.set(0.0);
		m_RightIntake.set(0.0);
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
