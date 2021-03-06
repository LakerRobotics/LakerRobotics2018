package org.usfirst.frc.team5053.robot.record_playback;



import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team5053.robot.RobotControllerMap;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import org.usfirst.frc.team5053.robot.Robot;
import org.usfirst.frc.team5053.robot.RobotConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


/*
*This macro records all the movements you make in teleop and saves them to the file you specify.
*make sure you record every variable you need, if you dont record the value from a motor or a solenoid,
*you won't be able to play it back. It records it in "frames" that contain a value for each output 
* you want to use during teleop
*BE AWARE: write into the same file as you do in the Play macro
*BE AWARE: Only write/read the motors/other things that you actually have fully created in 
*your code. Otherwise you'll lose robot code randomly with no reason
*In main, the try/catch structure catches any IOExceptions or FileNotFoundExceptions. Necessary to play back
*the recorded routine during autonomous
*Dennis Melamed, Melanie Quick
*22 March, 2015
*/


public class BTMacroRecord {
	
	//this object writes values into the file we specify
	FileWriter writer = null;
	
	boolean debug = false;
	
	long startTime;
	
	public BTMacroRecord() throws IOException
	{
			//record the time we started recording
			startTime = System.currentTimeMillis();
			
			try
			{
				//put the filesystem location you are supposed to write to as a string 
				//as the argument in this method, as of 2015 it is /home/lvuser/recordedAuto.csv
				System.out.println("BTMacroRecord.BTMacroRecord() constructor about to create a new writter created for file "+(Robot.autoFile+(Robot.getMaxRecorderFileNumber()+1))+".csv ");
				writer = new FileWriter(Robot.autoFile+Robot.getNextRecorderFileNumber()+".csv");
			}
			catch(Exception e) {
				System.out.println("BTMacroRecord.BTMacroRecord() trying to open writer for"+Robot.autoFile+Robot.getMaxRecorderFileNumber()+".csv error="+e);
			}
	}
	

	public void record(RobotControllerMap theRobotControllerMap) throws IOException
	{
		if(writer == null) {
			System.out.println("in BTMacroRecord.record() but writer is null, so exiting without doing anything");
		}
		else // writer is available so 
		{
			if(RobotConstants.getRobotName()== "lisa") {
				writeLisaMotorValuesToFile(theRobotControllerMap);
			}else {
				writeLilGeekMotorValuesToFile(theRobotControllerMap);
			}
		}
	}


	private void writeLilGeekMotorValuesToFile(RobotControllerMap theRobotControllerMap) throws IOException {
		if(debug) {
			System.out.print("in BTMacroRecord.writeLilGeekMotorValuesToFile() "+(System.currentTimeMillis()-startTime)
				//drive motors		
				+"," + theRobotControllerMap.getLeftDriveGroup().get()
				+"," + theRobotControllerMap.getRightDriveGroup().get()
				//intake motors
				+"," + theRobotControllerMap.getLeftIntake().get()
				+"," + theRobotControllerMap.getRightIntake().get()
				+ "\n"
			);
		}
		//start each "frame" with the elapsed time since we started recording
		writer.append("" + (System.currentTimeMillis()-startTime));

		//in this chunk, use writer.append to add each type of data you want to record to the frame

		//drive motors
		writer.append("," + theRobotControllerMap.getLeftDriveGroup().get());
		writer.append("," + theRobotControllerMap.getRightDriveGroup().get());

		//intake motors
		writer.append("," + theRobotControllerMap.getLeftIntake().get());
		writer.append("," + theRobotControllerMap.getRightIntake().get());

		/*
		 * THE LAST ENTRY OF THINGS YOU RECORD NEEDS TO HAVE A DELIMITER CONCATENATED TO 
		 * THE STRING AT THE END. OTHERWISE GIVES NOSUCHELEMENTEXCEPTION
		 */ 

		//this records a true/false value from a piston
		//		writer.append("," + theRobotControllerMap.getCatapult().get() + "\n");
		writer.append( "\n");
		
		//writer.append("," + storage.robot.getToteClamp().isExtended() + "\n");

		/*
		 * CAREFUL. KEEP THE LAST THING YOU RECORD BETWEEN THESE TWO COMMENTS AS A
		 * REMINDER TO APPEND THE DELIMITER
		 */
	}
	
	private void writeLisaMotorValuesToFile(RobotControllerMap theRobotControllerMap) throws IOException {
		if(debug) {
			System.out.print("in BTMacroRecord.writeLisaMotorValuesToFile() "+(System.currentTimeMillis()-startTime)
				//drive motors		
				+"," + theRobotControllerMap.getLeftDriveGroup().get()
				+"," + theRobotControllerMap.getRightDriveGroup().get()
				//intake motors
				+"," + theRobotControllerMap.getLeftIntake().get()
				+"," + theRobotControllerMap.getRightIntake().get()
				+"," + theRobotControllerMap.getRoller().get()
				//Elevator motor
				+"," + theRobotControllerMap.getElevator().getMotorOutputPercent()
				//Intake Squeeze
				+"," + theRobotControllerMap.getIntakeSolenoid().get()
				//Catapult
				+"," + theRobotControllerMap.getCatapultLeft().get()
				+ "\n"
			);
		}
		//start each "frame" with the elapsed time since we started recording
		writer.append("" + (System.currentTimeMillis()-startTime));

		//in this chunk, use writer.append to add each type of data you want to record to the frame

		//drive motors
		writer.append("," + theRobotControllerMap.getLeftDriveGroup( ).get());
		writer.append("," + theRobotControllerMap.getRightDriveGroup().get());

		//intake motors
		writer.append("," + theRobotControllerMap.getLeftIntake( ).get());
		writer.append("," + theRobotControllerMap.getRightIntake().get());
		writer.append("," + theRobotControllerMap.getRoller(     ).get());

		//Elevator motor
		writer.append("," + theRobotControllerMap.getElevator().getMotorOutputPercent());
		
		// intake squeeze
		boolean squeeze;
		if(theRobotControllerMap.getIntakeSolenoid().get() == Value.kForward) {
			squeeze = true;
		}
		else {
			squeeze = false;
		}
		writer.append("," + squeeze);
		
		//catapult
		boolean catapult;
		if(theRobotControllerMap.getCatapultLeft().get() == Value.kForward) {
			catapult = true;
		}
		else {
			catapult = false;
		}
		writer.append("," + catapult);
	
		/*
		 * THE LAST ENTRY OF THINGS YOU RECORD NEEDS TO HAVE A DELIMITER CONCATENATED TO 
		 * THE STRING AT THE END. OTHERWISE GIVES NOSUCHELEMENTEXCEPTION
		 */ 

		writer.append( "\n");
		
	}
	
	//this method closes the writer and makes sure that all the data you recorded makes it into the file
	public void end() throws IOException
	{
		if(writer !=null)
		{
		writer.flush();
		writer.close();
		}
	}
}
