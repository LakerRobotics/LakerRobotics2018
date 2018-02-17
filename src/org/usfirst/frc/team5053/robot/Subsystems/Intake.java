package org.usfirst.frc.team5053.robot.Subsystems;


import edu.wpi.first.wpilibj.Talon;

public class Intake implements Subsystem {

	private Talon m_LeftIntake;
	private Talon m_RightIntake;
	
	
	
	public Intake(Talon leftTalon, Talon rightTalon)
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
