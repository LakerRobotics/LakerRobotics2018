package org.usfirst.frc.team5053.robot.Subsystems;


import edu.wpi.first.wpilibj.Solenoid;

public class Catapult implements Subsystem {

	private Solenoid m_Solenoid;
	
	public Catapult(Solenoid solenoid)
	{
		m_Solenoid = solenoid;
	}
	public void Launch() {
		m_Solenoid.set(true);
	}
	public void Arm() {
		m_Solenoid.set(false);
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
