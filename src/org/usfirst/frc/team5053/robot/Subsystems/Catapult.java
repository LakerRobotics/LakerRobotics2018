package org.usfirst.frc.team5053.robot.Subsystems;


import java.time.Instant;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;

public class Catapult implements Subsystem {

	private DoubleSolenoid m_LeftSolenoid;
	private DoubleSolenoid m_RightSolenoid;
	
	public Catapult(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid)
	{
		m_LeftSolenoid = leftSolenoid;
		m_RightSolenoid = rightSolenoid;
	}
	public void Launch() 
	{
		m_LeftSolenoid.set(Value.kForward);
		m_RightSolenoid.set(Value.kForward);
	}
	/**
	 *  pass in fractions of a seconds to throw with the piston before reversing 
	 *  
	 */
	public void LauchTimeLimited(double secondsTillReverse ) {
		long milliSecondsTillReverse = (long) (secondsTillReverse*1000);
		int nanoSecondTillReverse = (int) (secondsTillReverse*1000*1000000 - milliSecondsTillReverse*1000000);

		//Define the sepearte thread 
        Thread t = new Thread(() -> {
        	try {
        		Instant start = Instant.now();
        		m_LeftSolenoid.set(Value.kForward);
        		m_RightSolenoid.set(Value.kForward);
        		
        		Thread.sleep(milliSecondsTillReverse, nanoSecondTillReverse);
        		
        		m_LeftSolenoid.set(Value.kReverse);
        		m_RightSolenoid.set(Value.kReverse);
        		Instant end = Instant.now();
        		
        		double howLongActualFired = (end.getNano()-start.getNano())/1000000000;
        		
        		System.out.println("Catapult.LaunchTimeLimited thread: Wanted to fire for "+secondsTillReverse+" seconds; Catapult actually Fired for "+howLongActualFired+ " seconds.  Error was " +(howLongActualFired -secondsTillReverse)+ " seconds.");
        		
        	} catch (InterruptedException e) {
        		// TODO Auto-generated catch block
        		e.printStackTrace();
        	}
        });
        
        // start the thread that will run the above code totally indipendently of the normal robot thread
        t.start();

	}
	public void Arm() 
	{
		m_LeftSolenoid.set(Value.kReverse);
		m_RightSolenoid.set(Value.kReverse);
	}
	public Value getState()
	{
		return m_LeftSolenoid.get();
	}
	
	@Override
	public void WriteDashboardData() 
	{
		SmartDashboard.putString("Catapult State", getState().toString());	
	}

}
