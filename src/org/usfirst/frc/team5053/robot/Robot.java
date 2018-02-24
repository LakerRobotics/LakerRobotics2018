package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Sensors.LidarLite;
import org.usfirst.frc.team5053.robot.Subsystems.Catapult;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Elevator;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5053.robot.record_playback.BTMacroRecord;
import org.usfirst.frc.team5053.robot.record_playback.BTMacroPlay;


/**
 * 
 * Power Up 2018
 * Laker Robotics 
 * 
 * 
 * Driver Controls
 * Left Stick - Pitch
 * Right Stick - Yaw
 * Left Trigger - 70% Speed
 * Right Trigger - Fire Catapult (If armed)
 * Left Bumper - 100% Speed
 * Right Bumper -
 * Select -
 * Start -
 * A -
 * B -
 * X -
 * Y - 
 * 
 * 
 * Operator Controls
 * Joystick Y - Manual Elevator Height
 * Button 3 - Intake Cube
 * Button 4 - Release Cube
 * Button 5 - Rotate Cube Left
 * Button 6 - Rotate Cube Right
 * ?? - Elevator to Floor Pickup
 * ?? - Elevator to Transfer
 * ?? - Elevator to Switch High
 * ?? - Elevator to Switch Low
 */

public class Robot extends IterativeRobot
{
	/* Declare any and all variables here
	 *  Do not initialize until robotInit()
	 */
	
	//Robot Map Declaration
	private RobotInterfaceMap m_RobotInterface;
	private RobotControllerMap m_RobotControllers;
	private RobotSensorMap m_RobotSensors;
	
	//Robot Subsystem Declaration
	private DriveTrainMotionControl m_DriveTrain;
//	private Elevator m_Elevator;
	private Intake m_Intake;
	private Catapult m_ThePult;
	private LidarLite m_Lidar;
	
	//Vision declaration
	
	//Subsystem constants

	//Vision constants
	
	//Autonomous variables
	private int autonomousRoutine;
	private int autonomousCase;
	private int autonomousWait;
	
	private String autonRoutine;	
	private boolean secondPart;
	private String matchData;
	private int switchTurn;
	private int scaleTurn;
	private char switchChar;
	private char scaleChar;
	
	// Diagnostic variables
	//private NetworkTable m_NetworkTable;
	private double driveTrainDiagnosticPower;
	private double[] diagnosticLeftRate;
	private double[] diagnosticRightRate;
	private double[] diagnosticPowerSent;
	private int arrIndex;
	
	// Record Playback
	BTMacroRecord m_BTMacroRecord;
	BTMacroPlay m_BTMacroPlay;
	
	boolean isRecording = false;
	//autoNumber defines an easy way to change the file you are recording to/playing from, in case you want to make a
	//few different auto programs
		static final int autoNumber = 1; // I guess if you like the recording then for now increment recompile so don't overwrite (obviously we can do better again this is just Proof of Concept)
		//autoFile is a global constant that keeps you from recording into a different file than the one you play from
		public static final String autoFile = new String("/home/lvuser/recordedAuto");
		public static final String recorderFileMaxNumber = new String("/home/lvuser/recorderMaxNumber.csv");
	
	
	//Misc variables
	private final double kFloor = 0.0;
	private final double kTransfer = 0.0;
	private final double kLow = 0.0;
	private final double kHigh = 0.0;
	
	private double m_driveSpeed = 1.0;
	
	private int m_catapultDelay = 0;
	
	@Override
    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	m_RobotInterface = new RobotInterfaceMap(JoystickType.XBOX, JoystickType.JOYSTICK);
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDriveGroup(), m_RobotControllers.getRightDriveGroup(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
//    	m_Elevator = new Elevator(m_RobotControllers.getElevator(), m_RobotSensors.getElevatorEncoder());
    	m_Intake = new Intake(m_RobotControllers.getLeftIntake(), m_RobotControllers.getRightIntake());
    	m_ThePult = new Catapult(m_RobotControllers.getCatapult());
    	// Scaler
    	
    	CameraServer server = CameraServer.getInstance();
    	//server.setQuality(50);
    	server.startAutomaticCapture();
    	
    	// Diagnostic variable initialization
    	//m_NetworkTable =  NetworkTable.getTable("SmartDashboard");
    	driveTrainDiagnosticPower = 0;
    	diagnosticLeftRate = new double[202];
    	diagnosticRightRate = new double[202];
    	diagnosticPowerSent = new double[202];
    	arrIndex = 0;
    	
    }
    
    
    
    
	@Override
	public void teleopInit(){
		isRecording= false;
    	// Record Playback
    	try{
    		m_BTMacroRecord = new BTMacroRecord();
		}
		catch(Exception e){
			System.out.print("Robot.teleopInit() Error creating record-n-playback objects, maybe couldn't create the file. The error is"+e);
		}

	}

	@Override
    public void autonomousInit() 
    {
    	 /**
         * This function is called once when autonomous begins
         */
        // Record Playback
    	try{
			m_BTMacroPlay = new BTMacroPlay(); // note this should initalize the file open to read from
		}
		catch(Exception e){
			System.out.print("Error creating record-n-playback objects, maybe couldn't create the file. The error is"+e);
		}
         
    	
    	// Initialize autonomous variables
    	autonomousCase = 0;
    	autonomousWait = 0; // 20 loops per second
    	
    	// Get information about which autonomous routine to run
    	// TODO Make sure this is defaulted to the correct value when put to production
    	autonRoutine = SmartDashboard.getString("Auton Routine", "Center Scale");		// Start position of the robot from our side of the field
    	secondPart = SmartDashboard.getBoolean("Second Part", false);			// Second part of auton routine
    	matchData = DriverStation.getInstance().getGameSpecificMessage(); 		// Field orientation
    	
    	// Parse matchData  for the switch and scale position in order to determine which way to turn later
//temp    	switchChar = matchData.charAt(0);
//temp    	scaleChar = matchData.charAt(1);
    	switchChar = 'R';
    	scaleChar = 'R';
    	
    	if(switchChar == 'R')
    		switchTurn = 1; // Determines the switch plate we aim at where positive turns right(clockwise)
    	else
    		switchTurn = -1; // And negative turns left (counterclockwise) 
    	
    	if(scaleChar == 'R')
    		scaleTurn = -1; // Final turn is always Counter clockwise when the scale is on the right side
    	else
    		scaleTurn = 1; // And vice versa
    }

    public void autonomousPeriodic()
    {
    	boolean debug_record_playback = true;
    	if(debug_record_playback){
    		m_BTMacroPlay.play(m_RobotControllers);
    	}
    	else
    	{
		
    	/**
         * This function is called periodically during autonomous
         */
    	switch(autonRoutine.toLowerCase())
    	{
    	case "none": // NO AUTON
    		break;
    	case "center": // CENTER AUTON SWITCH FIRST
    		centerSwitch();
			break;
    	case "center scale":
    		scaleCenter();
    		break;
    	case "left": // LEFT AUTON SCALE FIRST
    		scaleSides();
    		break;
    	case "right": // RIGHT AUTON SCALE FIRST
    		scaleSides();
    		break;
    	case "debug": // DEBUG AUTO
			diagnosticTest();
			break;
    	case "test":
    		swingTest();
    		//straightTest();
    		//turnTest();
    		break;
    	case "playback":
    		m_BTMacroPlay.play(m_RobotControllers);
    		break;
		default: // NO AUTON
			break;
    	}
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	autonomousWait++;
    	
    }
    
    public void turnTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.setAngle(90);
    		m_DriveTrain.enableTurnPID();
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isTurnPIDOnTarget())
    		{
    			m_DriveTrain.disableTurnPID();
    			m_DriveTrain.ArcadeDrive(0, 0);
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		break;
    	}
    }
    public void straightTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.DriveDistance(60, 10, 8);
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.ArcadeDrive(0, 0);
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		break;
    	}
    }
    
    public void swingTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.SetSwingParameters(30, true);
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1:
    		//if(m_DriveTrain.SwingAngleOnTarget())
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
        		m_DriveTrain.disableSwingPID();
        		m_DriveTrain./*DriveDistance*/DriveControlledAngle(12*4.5, 8, 5, 30);
    			
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
        		m_DriveTrain.SetSwingParameters(0, false);
        		m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    		}
    		break;
    	}
    }
    
    public void diagnosticTest()
    {
    	m_DriveTrain.arcadeDrive(driveTrainDiagnosticPower/100, 0);
    	
    	switch(autonomousCase)
    	{
    	case 0:
    		if(autonomousWait >= 10)
        	{
        		if(driveTrainDiagnosticPower <= 99)
        		{
            		diagnosticPowerSent[arrIndex] = driveTrainDiagnosticPower/100;
            		diagnosticLeftRate[arrIndex] = m_DriveTrain.GetLeftSpeed();
            		diagnosticRightRate[arrIndex] = m_DriveTrain.GetRightSpeed();
            		
            		//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower++;
            		autonomousWait = 0;
        		}
        		else
        		{
        			//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
        			m_DriveTrain.arcadeDrive(0, 0);
        			driveTrainDiagnosticPower = 0;
        			autonomousWait = 0;
        			autonomousCase++;
        		}
        	}
    		break;
    	case 1:
    		if(autonomousWait >= 10)
        	{
        		if(driveTrainDiagnosticPower >= -99)
        		{
            		diagnosticPowerSent[arrIndex] = driveTrainDiagnosticPower/100;
            		diagnosticLeftRate[arrIndex] = m_DriveTrain.GetLeftSpeed();
            		diagnosticRightRate[arrIndex] = m_DriveTrain.GetRightSpeed();
            		
            		//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower--;
            		autonomousWait = 0;
        		}
        		else
        		{
        			//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
        			m_DriveTrain.arcadeDrive(0, 0);
        			driveTrainDiagnosticPower = 0;
        			autonomousWait = 0;
        			autonomousCase++;
        		}
        	}
    		break;
		default:
			break;
    	}
    	
    }
    
    public void centerSwitch()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Drive to decision point
    		m_DriveTrain.DriveDistance(9.5, 2, 1);
    		autonomousCase++;
    		break;
    	case 1: // Turn based on the position of the switch 
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.TurnToAngle(30*switchTurn);
    			autonomousCase++;
    		}
    		break;
    	case 2: // Drive to the switch
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.DriveDistance(100, 3.5, 1);
    			autonomousCase++;
    		}
    		break;
    	case 3: // Turn to square the robot with the switch
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.TurnToAngle(-30*switchTurn);
    			autonomousCase++;
    		}
    		break;
    	case 4: // Release powercube onto switch plate
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.ArcadeDrive(0, 0);
    			// Release / shoot cube onto the switch plate
    			autonomousCase++;
    		}
    		break;
		default:
			break;
    	}
    }
    
    public void scaleSides()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Path our routine
    		if(scaleChar == autonRoutine.toUpperCase().charAt(0))
				autonomousCase++; // ******Straight ahead
			else
	    		autonomousCase = 2;// Cross the field
    		break;
    	case 1: // ******Drive directly to the scale as we started on the same side as the scale
    			m_DriveTrain.DriveDistance(19*12 + 70.5, 4, 1);
    			autonomousCase = 8; // ******Jump to the end of the routine
    		break;
    	case 2: // Drive to the decision point
    		m_DriveTrain.DriveDistance(19*12, 4, 1);
    		autonomousCase++;
    		break;
    	case 3: // Turn to cross the field
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
        		m_DriveTrain.TurnToAngle(90*-scaleTurn); 
        		// The turn is inverted in order to turn clockwise when scale is on right and counter clockwise when scale is on left 
        		// This is the inverse of the above in the autonomous init
        		autonomousCase++;
    		}
    		break;
    	case 4:
    		if(m_DriveTrain.isTurnPIDFinished()) // Cross field
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.DriveDistance(15*12, 4, 2);
    			autonomousCase++;
    		}
    		break;
    	case 5:
    		if(m_DriveTrain.isStraightPIDFinished()) // Turn to face scale
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.TurnToAngle(90*scaleTurn);
    			autonomousCase++;
    		}
    		break;
    	case 6: // Finish turn PID and meet back up with the other decision path
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			autonomousCase++;
    		}
    		break;
    	case 7: // Drive up to scale from decision point or parallel decision point
    		m_DriveTrain.DriveDistance(70.5, 4, 1);
    		autonomousCase++;
    		break;
    	case 8: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			// TODO what is the actual angle
    			m_DriveTrain.TurnToAngle(45*scaleTurn);
    			autonomousCase++;
    		}
    		break;
    	case 9: // Shoot powercube onto scale plate
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			// Shoot onto scale plate
    		}
    	}
    }
    
    public void scaleCenter()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.SetSwingParameters(45, true);
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
        		m_DriveTrain.DriveControlledAngle(7*12, 8, 5, 45);
        		autonomousCase++;
    		}
    		break;
    	case 2:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.SetSwingParameters(0, false);
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.DriveControlledAngle(5*12, 8, 5, 0);
    			autonomousCase++;
    		}
    		break;
    	case 4:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.SetSwingParameters(30, false);
    			m_DriveTrain.StartSwingTurn();
    		}
    		break;
    	case 5:
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    		}
    		break;
    	}
    }
    
    // Used solely to test the second portion of the scale first auton
    // Starts on case 2 of the scaleFirst routine when the scale position and start position do not match
    // End case matches case 5 of the scaleFirsr routine where the turn PID is ended in order to cleanly meet back up with the other decision path
    public void tournamentWinner()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.TurnToAngle(90*-scaleTurn);
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.DriveDistance(15*12, 4, 2);
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.TurnToAngle(90*scaleTurn);
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			autonomousCase++;
    		}
    		break;
    	}
    }
    
    public void teleopPeriodic()
    {
    	/**
         * This function is called periodically during operator control
         */
    	
    	// For quick testing reset
    	if(autonomousCase != 0)
    	{
    		autonomousCase = 0;
    		m_DriveTrain.DisablePIDControl();
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	arcadeDrive();
//    	elevatorControl();
    	intakeControl();
    	catapultControl();
    	
    	m_DriveTrain.WriteDashboardData();
    	
    	//Shooter methods
    	
    	//Other
      	record4LaterPlayback();
    	
    	//Misc variable updates
    	
    }
    
    public void record4LaterPlayback()
    {
	//Record for record playback
    	//the statement in this "if" checks if a button you designate as your record button 
    	//has been pressed, and stores the fact that it has been pressed in a variable
    	System.out.println("Robot.record4LaterPlayback() m_RobotInterface.GetRecord()="+m_RobotInterface.GetRecord()+"  isRecording="+isRecording);
    	if (m_RobotInterface.GetRecord()) 
		{
    		isRecording = true;
		}  
		//if our record button has been pressed, lets start recording!
		if (isRecording)
		{
   		try
    		{
    			m_BTMacroRecord.record(m_RobotControllers);
			}
			catch (Exception e) 
			{
				e.printStackTrace();
			}
		}
		
  }    
    
public void disabledInit(){
		//once we're done recording, the last thing we'll do is clean up the recording using the end
		//function. more info on the end function is in the record class
    	try 
    	{
    		if(m_BTMacroRecord != null)
    		{
    			m_BTMacroRecord.end();
    		}
		} 
		catch (Exception e) 
		{
			e.printStackTrace();
		}
    	
    }

    public void testPeriodic()
    {  
        /**
         * This function is called periodically during test mode
         */
    }
    
    //Drivetrain methods
    public void arcadeDrive()
    {
    	// Unfortunately both drive motor groups must be inverted in order for the encoders to properly read (On Lil' Geek) which is why the inputs are inverted here
    	
    	
    	if(m_RobotInterface.GetDriverLeftTrigger() && !m_RobotInterface.GetDriverLeftBumper())
    	{
    	 	m_driveSpeed = 0.7;
    	}
    	else if (!m_RobotInterface.GetDriverLeftTrigger() && m_RobotInterface.GetDriverLeftBumper())
    	{
    	 	m_driveSpeed = 1.0;
    	}
    	m_DriveTrain.ArcadeDrive(-m_RobotInterface.GetDriverLeftY()*m_driveSpeed, -m_RobotInterface.GetDriverRightX()*m_driveSpeed);
    	
   }
//    public void elevatorControl() {
//    	
//    	/*if (m_RobotInterface.GetOperatorA() && !(m_Elevator.getPositionTarget() == kFloor)) {
//    		m_Elevator.setPosition(kFloor);
//    	} else if (m_RobotInterface.GetOperatorB() && !(m_Elevator.getPositionTarget() == kTransfer)) {
//    		m_Elevator.setPosition(kTransfer);
//    	} else if (m_RobotInterface.GetOperatorX() && !(m_Elevator.getPositionTarget() == kHigh)) {
//    		m_Elevator.setPosition(kHigh);
//    	} else if (m_RobotInterface.GetOperatorY() && !(m_Elevator.getPositionTarget() == kLow)) {
//    		m_Elevator.setPosition(kLow);
//    	} else */
//    	if (Math.abs(m_RobotInterface.GetOperatorJoystick().getRawAxis(0)) > .05) {
//    		m_Elevator.disablePID();
//    		m_Elevator.manualControl(m_RobotInterface.GetOperatorJoystick().getRawAxis(0)*.5);
//   	}
//    }
    public void intakeControl() {
    	if (m_RobotInterface.GetOperatorButton(3) && !m_RobotInterface.GetOperatorButton(4)) {
    		m_Intake.IntakeCube();
    	} else if (m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3)) {
    		m_Intake.ReleaseCube();
    	} else if (m_RobotInterface.GetOperatorButton(6) && !m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3)){
    		m_Intake.RotateRight();
    	} else if (m_RobotInterface.GetOperatorButton(5) && !m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3)){
    		m_Intake.RotateLeft();
    	} else {
    		m_Intake.StopIntake();
    	}
    }
    public void catapultControl() {
    	
    	//Right Trigger - Fire - Start 2 second delay before rearming
    	
    	if (m_RobotInterface.GetDriverRightTrigger() && m_catapultDelay == 0) {
    		m_ThePult.Launch();
    		m_catapultDelay = 100;
    	} else if (m_catapultDelay <= 0) {
    		m_catapultDelay = 0;
    		m_ThePult.Arm();
    	}
    	
    	if (m_catapultDelay > 0) {
    		m_catapultDelay--;
    	}
    	
    }
    
    
    public void GetDashboardData()
    {
    	//shooterRPM = SmartDashboard.getNumber("shooterRPM", DEFAULT_SHOOTER_RATE);
    	//shooterRPMBF = SmartDashboard.getNumber("shooterRPMBF", DEFAULT_SHOOTER_RATE);
    	//Use this to retrieve values from the Driver Station
    	//e.g Which autonomous to use or processed image information.
    }
    public void WriteDashboardData()
    {
    	m_DriveTrain.WriteDashboardData();
    }
    
    public static int getMaxRecorderFileNumber()
    {
    	int intToReturn = 0;
		//create a scanner to read the file created during BTMacroRecord
		//scanner is able to read out the doubles recorded into recordedAuto.csv (as of 2015)
		File fileMaxNumber = new File(Robot.recorderFileMaxNumber);
		try
		{
			Scanner scannerMaxNumber = new Scanner(fileMaxNumber);
		
			//let scanner know that the numbers are separated by a comma or a newline, as it is a .csv file
			scannerMaxNumber.useDelimiter(",|\\n");
		
			if (scannerMaxNumber.hasNextInt())
			{
				intToReturn = scannerMaxNumber.nextInt();
			}
			else{
				intToReturn = 0;
			}
			scannerMaxNumber.close();
		}
		catch(Exception e)
		{
			System.out.println("Robot.getNextRecorderFileNumber() exception e="+e);
		}
		return intToReturn;
    }
		
	public static int getNextRecorderFileNumber()
	{
		int intToReturn;
		// increment to next unused number
		intToReturn = getMaxRecorderFileNumber() + 1;
		try{
			System.out.print("NextRecorderFileNumber"+intToReturn);
		    boolean OVER_WRITE = false;
//		    boolean APPEND = true;
			FileWriter writerForAutoFileNumber = new FileWriter(Robot.recorderFileMaxNumber, OVER_WRITE);
			writerForAutoFileNumber.append(intToReturn + "\n"); 
			writerForAutoFileNumber.flush();
			writerForAutoFileNumber.close();
		}
		catch(Exception e){
			System.out.println("Robot.getNextRecorderFileNumber() exception e="+e);
		}
			
		return intToReturn;
	}
	
    
}