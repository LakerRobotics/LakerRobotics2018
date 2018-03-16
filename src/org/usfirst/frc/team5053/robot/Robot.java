package org.usfirst.frc.team5053.robot;

import java.io.File;
import java.io.FileWriter;
import java.time.Instant;
import java.time.temporal.ChronoField;
import java.io.FileNotFoundException;
import java.util.Scanner;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Sensors.LidarLite;
import org.usfirst.frc.team5053.robot.Subsystems.Catapult;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Elevator;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
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
 * Trigger	- Retract Intake Solenoid
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
	private Elevator m_Elevator;
	private Intake m_Intake;
	private Catapult m_ThePult;
	private LidarLite m_Lidar;
	private Compressor m_Compressor;
	private Talon m_Roller;
	
	//Vision declaration
	
	//Subsystem constants
	
	// setup for variable control intake the speed of the roller relative to the wheels
	private double speedOfIntake = /*gearRatio*/(24/1)*/*Diameter*/4.00;
	private double speedOfRollers = /*gearRatio*/(5/1)*/*Diameter*/2.5;
	private double rollerMotorDeadZone = 0.15;
	private double speed_relative_to_intake_wheels =  3*(speedOfRollers/speedOfIntake); // RGT 20180303 looks like only 0.13 is not enough to gaurntee the wheels will turn, but add dead zone and 3 times that is
	// private double speed_relative_to_intake_wheels = 0.25; // if the complexity above is not a good guess just start playing with this number
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
	BTMacroPlay m_PlaybackScaleToSwitch;
	
	boolean isRecording = true;// default so it is always recording during teleopp
	//autoNumber defines an easy way to change the file you are recording to/playing from, in case you want to make a
	//few different auto programs
		static final int autoNumber = 1; // I guess if you like the recording then for now increment recompile so don't overwrite (obviously we can do better again this is just Proof of Concept)
		//autoFile is a global constant that keeps you from recording into a different file than the one you play from
		public static final String autoFile = new String("/home/lvuser/recordedAuto");
		public static final String recorderFileMaxNumber = new String("/home/lvuser/recorderMaxNumber.csv");
	
	
	//Misc variables
	// TODO MUST BE INVERTED the SRX encoder is wired incorrectly and we can't invert only the encoder 
	private final double kFloor = 0.0;
	private final double kTransfer = -10000.0; //-13290.0
	private final double kLow = 0.0;
	private final double kHigh = -19569.0;
	private int transferIntakeCase = 0;
	
	private double m_driveSpeed = 1.0;
	
	private int m_catapultDelay = 0;
	private boolean m_switchShotFlag = false;
	
	
	private final int SWITCH_CATAPULT_DELAY = 5;
	private final int SCALE_CATAPULT_DELAY = 100;
	private int m_currentCatapultDelay = SCALE_CATAPULT_DELAY;
	private int m_shortCatapultDelay = 0;
	private boolean isShotFinished = true;
	private boolean hasElevatorEncoderZeroed = false;
	private final String m_robotName = RobotConstants.getRobotName();
	
	@Override
    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		
    	m_RobotInterface = new RobotInterfaceMap(JoystickType.XBOX, JoystickType.JOYSTICK);
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDriveGroup(), m_RobotControllers.getRightDriveGroup(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	if (m_robotName == "lisa") {
    		m_Elevator = new Elevator(m_RobotControllers.getElevator(), m_RobotSensors.getElevatorLimitHigh(), m_RobotSensors.getElevatorLimitLow());
        	m_Intake = new Intake(m_RobotControllers.getLeftIntake(), m_RobotControllers.getRightIntake(), m_RobotControllers.getIntakeSolenoid());
        	m_ThePult = new Catapult(m_RobotControllers.getCatapultLeft(), m_RobotControllers.getCatapultRight());
        	m_Roller = m_RobotControllers.getRoller();
        	
        	
        	
        	
        	m_Compressor = new Compressor(0);

        	//m_Compressor.start();
        	m_Compressor.setClosedLoopControl(true);
    	}
    	
    	// Scaler
    	
    	
    	
    	
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
		isRecording= true;// have it recording all the time
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
         
    	
    	// Initialize autonomous variables
    	autonomousCase = 0;
    	autonomousWait = 0; // 20 loops per second
    	
    	// Get information about which autonomous routine to run
    	// TODO Make sure this is defaulted to the correct value when put to production
    	autonRoutine = SmartDashboard.getString("Auton Selection", "left scale");		// Start position of the robot from our side of the field
    	secondPart = SmartDashboard.getBoolean("Second Step", false);			// Second part of auton routine
    	matchData = DriverStation.getInstance().getGameSpecificMessage(); 		// Field orientation
    	
    	// Parse matchData  for the switch and scale position in order to determine which way to turn later
    	switchChar = matchData.charAt(0);
    	scaleChar = matchData.charAt(1);
    	
    	if(switchChar == 'R')
    		switchTurn = 1; // Determines the switch plate we aim at where positive turns right(clockwise)
    	else
    		switchTurn = -1; // And negative turns left (counterclockwise) 
    	
    	if(scaleChar == 'R')
    		scaleTurn = -1; // Final turn is always Counter clockwise when the scale is on the right side
    	else
    		scaleTurn = 1; // And vice versa
    		
   		// Record Playback
    	int AfterScaleRightShotToRightSwitch = 1;	        
    	int AfterScaleRightShotToLeftSwitch = 1;	        
    	int AfterScaleLeftShotToRightSwitch = 1;	        
    	int AfterScaleLeftShotToLeftSwitch = 1;	        
    	try{
    		// This is playback lates record
    		m_BTMacroPlay = new BTMacroPlay(); // note this should initalize the file open to read from
    				
    	// This sets up the movement from the Scale shot to the switch shot  
    	if(scaleChar == 'R'){
    		if(switchChar == 'R'){
				m_PlaybackScaleToSwitch = new BTMacroPlay(AfterScaleRightShotToRightSwitch);
    	    }
    		else{
				m_PlaybackScaleToSwitch = new BTMacroPlay(AfterScaleRightShotToLeftSwitch);    	
    		}
    	}else{ // was Left scale
    		if(switchChar == 'R'){
				m_PlaybackScaleToSwitch = new BTMacroPlay(AfterScaleLeftShotToRightSwitch);
    	    }
    		else{
				m_PlaybackScaleToSwitch = new BTMacroPlay(AfterScaleLeftShotToLeftSwitch);    	
    		}
    	
    	}
		}
		catch(Exception e){
			System.out.print("Error creating record-n-playback objects, maybe couldn't create the file. The error is"+e);
		}
    		
    	
    	m_DriveTrain.ResetGyro();
    	m_DriveTrain.ResetEncoders();
    	if (m_robotName == "lisa") {
    		m_Elevator.resetEncoder();
    	}
    }

    public void autonomousPeriodic()
    {
    	boolean debug_record_playback = false;
    	if(debug_record_playback){
    		m_BTMacroPlay.play(m_RobotControllers);
    	}
    	else
    	{
		
    	/**
         * This function is called periodically during autonomous
         */
    		/*if (m_robotName == "lisa") {
    			if (!hasElevatorEncoderZeroed) {
    	    		m_Elevator.manualControl(0.30);
    	    	}
    	    	if (!m_Elevator.getLimitLow() && !hasElevatorEncoderZeroed) {
    	    		m_Elevator.resetEncoder();
    	    		hasElevatorEncoderZeroed = true;
    	    	}
    		}*/
    	
    	switch(autonRoutine.toLowerCase())
    	{
    	case "none": // NO AUTON
    		break;
    	case "switch": // CENTER AUTON SWITCH FIRST
    		switchCenter();
			break;
    	case "left scale": // LEFT AUTON SCALE FIRST
    		scaleLeftRight();
    		//worstCaseScenarioScaleLeftRightTest();
    		break;
    	case "center scale": // CENTER AUTON SCALE
    		scaleCenter();
    		break;
    	case "right scale": // RIGHT AUTON SCALE FIRST
    		scaleLeftRight();
    		//worstCaseScenarioScaleLeftRightTest();
    		break;
    	case "diagnostic": // DEBUG AUTO
			diagnosticTest();
			break;
    	case "test":
    		//switchShotTest();
    		//swingTest();
    		straightTest();
    		//controlledAngleTest();
    		//turnTest();
    		//switchCenter();
    		//worstCaseScenarioScaleLeftRightTest();
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
    
    public void controlledAngleTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.DriveControlledAngle(120, 5, 20, 0);
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		break;
    	}
    }
    public void turnTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.TurnToAngle(90);
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isTurnPIDFinished())
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
    public void straightTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.DriveDistance(60, 8, 20);
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
    		SmartDashboard.putBoolean("Swing test complete", false);
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.SetSwingParameters(-45, true);
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.DriveControlledAngle(2*12, 5, 20, -45);
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
        		SmartDashboard.putBoolean("Swing test complete", true);
        		autonomousCase++;
    		}
    		break;
    	case 3:
    		break;
    	}
    }
    public void switchShotTest()
    {
    	if (m_robotName == "lisa") {
    		switch(autonomousCase)
        	{
            case 0: // Launch cube into switch with the short shot
        		m_ThePult.Launch();
        			
    			autonomousWait = 0;
    			autonomousCase++;
    			break;
        	case 1:
        		if(autonomousWait >= SWITCH_CATAPULT_DELAY)
        		{
        			m_ThePult.Arm();
            		autonomousCase++;
        		}
        		break;
        	case 2:
        		break;
        	}
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
    
    // TODO Straight drive length?
    public void switchCenter()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Swing to move toward the desired switch plate
    		if(switchChar == 'L')
    		{
        		m_DriveTrain.SetSwingParameters(-30, false);
    		}
    		else
    		{
    			m_DriveTrain.SetSwingParameters(30, true);
    		}
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1: // Drive toward desired switch plate and arm
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			if(switchChar == 'L')
    			{
            		m_DriveTrain.DriveControlledAngle(-12*3.5, 8, 5, -30);
    			}
    			else
    			{
    				m_DriveTrain.DriveControlledAngle(-12*3.5, 8, 5, 30);
    			}
    			
    			if (m_robotName == "lisa") {
    				m_ThePult.Arm();
    				m_Elevator.setPosition(kHigh);
    			}
    			
        		autonomousCase++;
    		}
    		break;
    	case 2: // Swing to square up with switch plate
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if(switchChar == 'L')
    			{
        			m_DriveTrain.SetSwingParameters(0, true);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			if (m_robotName == "lisa") {
        			m_Roller.set(.80);
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.arcadeDrive(-.5, 0.0);

    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 4: // Launch cube into switch with the short shot
    		m_DriveTrain.arcadeDrive(-.5,  0.0);
    		if(autonomousWait >= 25)
    		{
    			
    			///TODO
    			
    			if (m_robotName == "lisa") 
    			{
    				m_ThePult.Launch();
    				autonomousWait = 0;
    				autonomousCase++;
    			}
    			else
    			{
        			autonomousCase = 6;
    			}
    			
    			
    		}
    		break;
    	case 5:
    		m_DriveTrain.arcadeDrive(-.5,  0.0);
    		if(autonomousWait >= SWITCH_CATAPULT_DELAY)
    		{
    			
    			
    			
    			if (m_robotName == "lisa") {
    				m_ThePult.Arm();
    				m_Roller.set(0.0);
    			}
    			autonomousCase++;
    		}
    		break;
    	case 6:
    		m_DriveTrain.arcadeDrive(0.0, 0.0);
    		break;
    	}
    }
     
    //TODO Find actual distances/angles
    public void scaleLeftRight()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Path our routine
    		if (m_robotName == "lisa") {
        		m_ThePult.Arm();
        		m_Roller.set(.80);
    		}
    		if(scaleChar == autonRoutine.toUpperCase().charAt(0))
    			
				autonomousCase++; // ******Straight ahead
			else
	    		autonomousCase = 2;// Cross the field
    		break;
    	case 1: // ******Drive directly to the scale as we started on the same side as the scale
    			m_DriveTrain.DriveDistance(-(13.5*12/*Decision Point*/ + 60/*Decision point to scale*/), 6, 10);
    			if (m_robotName == "lisa") 
    			{
    				m_Elevator.setPosition(kHigh);
    				m_Roller.set(.80);
    			}
    			autonomousCase = 8;
    		break;
    	case 2: // Drive to the decision point
    		m_DriveTrain.DriveDistance(-13.5*12, 6, 10);
    		
    		autonomousCase++;
    		//autonomousCase = 12;
    		break;
    	case 3: // Turn to cross the field
    		if (m_robotName == "lisa") {
    			m_Roller.set(.80);
    		}
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
        		if(scaleChar == 'R')
        		{
        			m_DriveTrain.SetSwingParameters(90, true);
        		}
        		else
        		{
        			m_DriveTrain.SetSwingParameters(-90, false);
        		}
        		m_DriveTrain.StartSwingTurn();
        		autonomousCase++;
    		}
    		break;
    	case 4: // Cross the field
    		if (m_robotName == "lisa") {
    			m_Roller.set(.80);
    		}
    		if(m_DriveTrain.SwingAngleOnTarget()) 
    		{
    			m_DriveTrain.disableSwingPID();
        		if(scaleChar == 'R')
        		{
        			m_DriveTrain.DriveControlledAngle(-12*12, 8, 10, 90);
        		}
        		else
        		{
        			m_DriveTrain.DriveControlledAngle(-12*12, 8, 10, -90);
        		}
    			autonomousCase++;
    		}
    			
    		break;
    	case 5: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 6: // Finish turn PID and meet back up with the other decision path
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			autonomousCase++;
    		}
    		break;
    	case 7: // Drive up to scale from decision point or parallel decision point
    		m_DriveTrain.DriveDistance(-24, 4, 4);
    		autonomousCase++;
    		break;
    	case 8: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(-90, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(90, true);
    			}
    			if (m_robotName == "lisa") {
    				m_Roller.set(.80);
        			m_Elevator.setPosition(kHigh);
        		}
    			m_DriveTrain.StartSwingTurn();
    			
    			autonomousCase++;
    		}
    		break;
    	case 9: // Back up 3ft for shot
    		if (m_robotName == "lisa") {
    			m_Roller.set(.80);
    		}
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
        		m_DriveTrain.DriveDistance(36, 4, 4);
        		// TODO
        		autonomousCase = 12;
        		//autonomousCase++;
    		}
    		break;
    	case 10: // Shoot powercube onto scale plate
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			
    			//TODO uncomment me!
    			if (m_robotName == "lisa") {
    				m_Roller.set(.80);
    				if (Math.abs(m_Elevator.getPositionTarget()) - Math.abs(m_Elevator.getCurrentPosition()) < 2000) {
    					//m_ThePult.Launch();
        				autonomousCase++;
    				}
    				
    			} else {
    				autonomousCase++;
    			}
    			
        		
    		}
    		break;
    	case 11:// Go actually do the Switch
    		if (m_robotName == "lisa") {
				m_Roller.set(0.0);
				m_Elevator.setPosition(kLow);
				m_Intake.ReleaseCube();
			}
    		if (scaleChar == 'R') {
    			if (switchChar == 'R') {
    				m_DriveTrain.TurnToAngle(20);
    			} else {
    				m_DriveTrain.TurnToAngle(60);
    			}
    		} else {
    			if (switchChar == 'R') {
    				m_DriveTrain.TurnToAngle(-20);
    			} else {
    				m_DriveTrain.TurnToAngle(-60);
    			}
    		}
        	autonomousCase++;
    		break;
    	case 12:
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.ArcadeDrive(0, 0);
    		}
    		break;
    	}
    }
    
    public void scaleCenter()
    {
    	// Scale R 	= 	-1
    	// Scale L 	= 	 1
    	//Switch R 	= 	 1
    	//Switch L 	= 	-1
    	switch(autonomousCase)
    	{
    	case 0: // Swing to move toward the desired scale
    		m_DriveTrain.ResetGyro();
    		if (scaleTurn == -1) 
    		{
    			m_DriveTrain.SetSwingParameters(45, true);
    		} 
    		else 
    		{
    			m_DriveTrain.SetSwingParameters(-45, false);
    		}
    		
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1: // Drive to prepare to run parallel with the wall
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			
        		m_DriveTrain.DriveControlledAngle(-8*12, 8, 5, -45*scaleTurn);
        		
        		if (m_robotName == "lisa") {
        			m_Elevator.setPosition(kHigh);
        		}
        		
        		autonomousCase++;
    		}
    		break;
    	case 2: // Swing to be parallel with the wall
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if (scaleTurn == -1) 
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			} 
    			else 
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			}
    			
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3: // Drive to scale
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.DriveControlledAngle(-6*12, 6, 8, 0);
    			
    			if (m_robotName == "lisa") {
    				m_ThePult.Arm();
    			}
    			
    			
    			autonomousCase++;
    		}
    		break;
    	case 4: // Swing to align for shooting
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			
    			if (scaleTurn == -1) 
    			{
    				m_DriveTrain.SetSwingParameters(-30, false);
    			} 
    			else 
    			{
    				m_DriveTrain.SetSwingParameters(30, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 5: // Use the inferior siege engine
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			if (m_robotName == "lisa") {
    				m_ThePult.Launch();
    			}
    			autonomousCase++;
    		}
    		break;
    	case 6: // Determine if we want to go for the switch
    		if(secondPart)
    		{
    			m_DriveTrain.disableSwingPID();
    			autonomousCase = 99;
    		} 
    		else 
    		{
    			
    			if (scaleTurn == -1) 
    			{
    				if (switchTurn == 1) 
    				{
    					m_DriveTrain.SetSwingParameters(45, false);
    				} 
    				else 
    				{
    					m_DriveTrain.SetSwingParameters(120, false);
    				}
    				
    			} 
    			else 
    			{
    				if (switchTurn == -1) 
    				{
    					m_DriveTrain.SetSwingParameters(-45, true);
    				} 
    				else 
    				{
    					m_DriveTrain.SetSwingParameters(-120, true);
    				}
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    			break;
    		}
    		break;
    	case 7:
    		if (m_DriveTrain.SwingAngleOnTarget()) 
    		{
    			if (m_robotName == "lisa") {
    				m_ThePult.Arm();
    			}
    			
    			m_DriveTrain.DisablePIDControl();
    			
    			if (scaleTurn == -1) 
    			{
    				if (switchTurn == 1) 
    				{
    					m_DriveTrain.DriveControlledAngle(3 * 12, 5, 4, 45);
    				} 
    				else 
    				{
    					m_DriveTrain.DriveControlledAngle(8 * 12, 5, 10, 120);
    				}
    				
    			} 
    			else 
    			{
    				if (switchTurn == -1) 
    				{
    					m_DriveTrain.DriveControlledAngle(3 * 12, 5, 4, -45);
    				} 
    				else 
    				{
    					m_DriveTrain.DriveControlledAngle(8 * 12, 5, 10, -120);
    				}
    			}
    			
    			autonomousCase++;
    		}
    		break;
    		
    	case 8:
    		if (m_DriveTrain.isStraightPIDFinished()) 
    		{
    			m_DriveTrain.DisablePIDControl();
    			if (scaleTurn == -1) 
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			} 
    			else 
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			m_DriveTrain.StartSwingTurn();
    			
    			autonomousCase++;
    		}
    		break;
    	case 9:
    		if (m_DriveTrain.SwingAngleOnTarget()) 
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.DisablePIDControl();
    			autonomousCase++;
    		}
    		break;
    	case 10:
    		m_DriveTrain.disableSwingPID();
			m_DriveTrain.DisablePIDControl();
    		m_DriveTrain.arcadeDrive(0, 0);
    		break;
    		
    	case 99:
    		m_DriveTrain.disableSwingPID();
    		m_DriveTrain.DisablePIDControl();
    		m_DriveTrain.arcadeDrive(0, 0);
    		break;
    	}
    	
    }
    
    public void worstCaseScenarioScaleLeftRightTest()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Turn to cross the field
    		//if(m_DriveTrain.isStraightPIDFinished())
    		//{
        		if(scaleChar == 'R')
        		{
        			m_DriveTrain.SetSwingParameters(90, true);
        		}
        		else
        		{
        			m_DriveTrain.SetSwingParameters(-90, false);
        		}
        		m_DriveTrain.StartSwingTurn();
        		autonomousCase++;
    		//}
    		break;
    	case 1: // Cross the field
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
	    		if(scaleChar == 'R')
	    		{
	    			m_DriveTrain.DriveControlledAngle(-15*12, 8, 5, 90);
	    		}
	    		else
	    		{
	    			m_DriveTrain.DriveControlledAngle(-15*12, 8, 5, -90);
	    		}
				autonomousCase++;
    		}
    		break;
    	case 2: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3: // Finish turn PID and meet back up with the other decision path
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			autonomousCase++;
    		}
    		break;
    	case 4: // Drive up to scale from decision point or parallel decision point
    		m_DriveTrain.DriveDistance(-70.5, 4, 1);
    		autonomousCase++;
    		break;
    	case 5: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			
    			// TODO what is the actual angle
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(-30, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(30, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			
    			autonomousCase++;
    		}
    		break;
    	case 6: // Shoot powercube onto scale plate
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			
    			
    			if (m_robotName == "lisa") {
    				m_ThePult.Launch();
    			}
        		autonomousCase++;
    		}
    		break;
    	case 7:
    		break;
    
    	}
    }
    
    public void teleopPeriodic()
    {
    	/**
         * This function is called periodically during operator control
         */
    	if (m_robotName == "lisa") {
    		m_Compressor.setClosedLoopControl(true);
    	}
    	
    	// For quick testing reset
    	if(autonomousCase != 0)
    	{
    		autonomousCase = 0;
    		m_DriveTrain.DisablePIDControl();
    	}
    	
    	// Subsystem methods
    	arcadeDrive();
    	if (m_robotName == "lisa") {
        	intakeControl();
        	catapultControl();
        	//switchCatapultShot();
        	rollerControl();
    		elevatorControl();// elevator has to come last because transfer button will override other buttons, so specifically the intake and roller
    	}
    	
    	
    	// Other
      	record4LaterPlayback();
      	
    	//Misc variable updates
    	GetDashboardData();
    	WriteDashboardData();
    	
    	
    }
    
    public void record4LaterPlayback()
    {
	//Record for record playback
    	//the statement in this "if" checks if a button you designate as your record button 
    	//has been pressed, and stores the fact that it has been pressed in a variable
    	//System.out.println("Robot.record4LaterPlayback() m_RobotInterface.GetRecord()="+m_RobotInterface.GetDriverButton(11)+"  isRecording="+isRecording);
    	if (m_RobotInterface.GetDriverButton(11)) 
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
				System.out.println("Robot.record4LaterPlayback() excepiton e="+e);	
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
    	// - | +
    	m_DriveTrain.ArcadeDrive(-m_RobotInterface.GetDriverLeftY()*m_driveSpeed, RobotConstants.getArcadeDriveInverted() * m_RobotInterface.GetDriverRightX()*m_driveSpeed );
    	
   }
    public void elevatorControl() {
    	
    	if (m_RobotInterface.GetOperatorButton(7) && !(m_Elevator.getPositionTarget() == kFloor)) 
    	{
    		m_Elevator.setPosition(kFloor);
    	} 
    	//else if (m_RobotInterface.GetOperatorButton(8) && !(m_Elevator.getPositionTarget() == kTransfer)) 
    	//{
    	//	m_Elevator.setPosition(kTransfer);
    	//} 
    	//else if (m_RobotInterface.GetOperatorButton(9) && !(m_Elevator.getPositionTarget() == kHigh)) 
    	//{
    	//	m_Elevator.setPosition(kHigh);
    	//} 
    	else if (m_RobotInterface.GetOperatorButton(2)) 
    	{
    		//m_Elevator.setPosition(kTransfer);
    		//if ((Math.abs(m_Elevator.getCurrentPosition()) - Math.abs(kTransfer)) <= 2000 ) 
    		//{
    			m_Intake.TransferCube();
    			m_Roller.set(.80);
    		//}
    	}  
    	else if (Math.abs(m_RobotInterface.GetOperatorJoystick().getRawAxis(1)) > .05) 
    	{
    		m_Elevator.disablePID();
    		// Elevator power is halved to prevent damage to the elevator when manually controlled
    		m_Elevator.manualControl(m_RobotInterface.GetOperatorJoystick().getRawAxis(1));
    	}	else {
    		m_Elevator.manualControl(0.0);
    	}
    	/*if (m_RobotInterface.GetOperatorButton(8)) 
    	{
    		int transferIntakeVersion  = 2;
    		if (transferIntakeVersion == 1){
    			m_Elevator.setPosition(kTransfer);
    			if ((Math.abs(m_Elevator.getCurrentPosition()) - Math.abs(kTransfer)) <= 1500 ) {
    				m_Intake.IntakeCube();
    			}
    		}else{
    			switch (transferIntakeCase)
    			{
    			case  0:// ensure Cube  is squeezed
    				m_Intake.retractIntake();
    				transferIntakeCase++;
    				break;
    			case  1: // Move Cube up
    	    		m_Elevator.disablePID();
    				m_Elevator.manualControl(0.5);
    				transferIntakeCase++;
        			break;
        		case  2: // Move Cube back into catapult
    				if(m_Elevator.getCurrentPosition()>kTransfer-250) 
    				{
    					m_Elevator.setPosition(kTransfer);// so we start PID control 0.25" below the target
    					//m_Intake.IntakeCube();//RGT 20180304 this is currently way to fast in my opinion at 100%
    					m_Intake.AdjustableSpeed(0.5);
    					m_Roller.set(0.35);
    					transferIntakeCase++;
    				}
        			break;
        		case 3:
    				// Continue to overide any of the other buttons they might be pressing (this one take priority) if they don't like that they will release the button 
					m_Elevator.setPosition(kTransfer);// think normally we would do this at next case but I want this to be fast and not waste a cycle overshoting the transfer height, so we start 0.25" below the target
					m_Intake.AdjustableSpeed(0.5);
					m_Roller.set(0.35);
    				break;
    			}
    		}
    	}else {
    		// See if they just released the button
    		if(transferIntakeCase>0) 
    		{
    			m_Elevator.disablePID();
				m_Intake.ReleaseCube();
				m_Intake.AdjustableSpeed(0);
				m_Roller.set(0);
				transferIntakeCase=0;
    		};
    	}*/
    	
    }
    public void intakeControl() {
    	if (m_RobotInterface.GetOperatorButton(3) && !m_RobotInterface.GetOperatorButton(4)) 
    	{
    		m_Intake.IntakeCube();
    	} 
    	else if (m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3)) 
    	{
    		m_Intake.ReleaseCube();
    	} 
    	else if (m_RobotInterface.GetOperatorButton(6) && !m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3))
    	{
    		m_Intake.RotateRight();
    	} 
    	else if (m_RobotInterface.GetOperatorButton(5) && !m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3))
    	{
    		m_Intake.RotateLeft();
    	} 
    	/*else if (Math.abs(m_RobotInterface.GetOperatorJoystick().getRawAxis(2)) > .05){
    		m_Intake.AdjustableSpeed(m_RobotInterface.GetOperatorJoystick().getRawAxis(2));
    	} */
    	// if joystick left-right more then 5% or twist more then 5% then variable joystick control of intake (note any button above the buttons take precedent and we never get here)
    	else if (  Math.abs(m_RobotInterface.GetOperatorJoystick().getRawAxis(2)) > .15 && m_RobotInterface.GetOperatorButton(8))
    	{
    		// if twist is less then 5% then just straight intake
    		if( Math.abs(m_RobotInterface.GetOperatorJoystick().getRawAxis(3)) < .15) 
    		{
    			
    			m_Intake.AdjustableSpeedWithTwist(0.80, m_RobotInterface.GetOperatorJoystick().getRawAxis(2));
    		// else twist power is more then 5% so apply variable twist power
    		}
    		else 
    		{
    			double speed = m_RobotInterface.GetOperatorJoystick().getRawAxis(3);
    			speed = (speed + 1)/2;
    			
    			m_Intake.AdjustableSpeedWithTwist(speed, m_RobotInterface.GetOperatorJoystick().getRawAxis(2));
    		}
    	}
    	else if (!m_RobotInterface.GetOperatorButton(8))
    	{
    		m_Intake.StopIntake();
    	}
    	
    	SmartDashboard.putNumber("Slider", m_RobotInterface.GetOperatorJoystick().getRawAxis(3));
    	
    	if(m_RobotInterface.GetOperatorButton(1) && m_Intake.getSolenoidState())
    	{
    		m_Intake.retractIntake();
    	}
    	else if(!m_RobotInterface.GetOperatorButton(1) && !m_Intake.getSolenoidState())
    	{
    		m_Intake.expandIntake();
    	}
    	
    }
    public void catapultControl() {
    	
    	//Right Trigger - Fire - Start 2 second delay before rearming
    	
    	if (m_RobotInterface.GetDriverRightTrigger() && m_catapultDelay == 0) 
    	{
    		m_ThePult.Launch();
    		m_catapultDelay = SCALE_CATAPULT_DELAY;
    	} 
    	else if (m_RobotInterface.GetDriverRightBumper() && !m_switchShotFlag) 
    	{
    		m_catapultDelay = SWITCH_CATAPULT_DELAY;
    		m_ThePult.Launch();
    		m_switchShotFlag = true;
    	}
    	else if (m_catapultDelay <= 0) 
    	{
    		m_catapultDelay = 0;
    		m_ThePult.Arm();
    	}
    	
    	if (!m_RobotInterface.GetDriverRightBumper()) {
    		m_switchShotFlag = false;
    	}
    	if (m_catapultDelay > 0) {
    		m_catapultDelay--;
    	}
    	
    }
    public void rollerControl() {
    	if (m_RobotInterface.GetOperatorButton(9)) {
    		m_Roller.set(.80);
    	} else if (m_RobotInterface.GetOperatorButton(10)) {
    		m_Roller.set(-.80);
        // This goes along with the Variable intake
        }/* else if (  Math.abs(m_RobotInterface.GetOperatorJoystick().getRawAxis(2)) > .05) {
        	m_Roller.set(rollerMotorDeadZone+speed_relative_to_intake_wheels*m_RobotInterface.GetOperatorJoystick().getRawAxis(2));
    	} */else if (!m_RobotInterface.GetOperatorButton(8)){
    		m_Roller.set(0.0);
    	}
    }
    
    public void switchCatapultShot()
    {
    	if(m_RobotInterface.GetDriverRightBumper() && isShotFinished)
    	{
    		isShotFinished = false;
    		m_ThePult.Launch();
    		m_shortCatapultDelay = 5;
    	}
    	else if(m_shortCatapultDelay <= 0)
    	{
    		isShotFinished = true;
    		m_shortCatapultDelay = 0;
    		m_ThePult.Arm();
    	}
    	
    	if (m_shortCatapultDelay > 0)
    		m_shortCatapultDelay--;
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
    	SmartDashboard.putNumber("Auton Case", autonomousCase);
    	SmartDashboard.putBoolean("isShotFinished", isShotFinished);
    	SmartDashboard.putNumber("Switch Catapult Delay", m_shortCatapultDelay);
    	
    	m_DriveTrain.WriteDashboardData();
    	if (m_robotName == "lisa") {
    		m_Elevator.WriteDashboardData();
        	m_ThePult.WriteDashboardData();
        	m_Intake.WriteDashboardData();
    	}
    	
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
	
	// Test the CatapultTimeLimited, timing accuracy
	Instant lastThrow = Instant.now();
	int cycleBufferMilli = 100;//Time to wait between cycles
	int cycleCurrentLength = 20; 
	int cycleStartIncrement = 5;
	int cyclesToRun = 50;
	int cycleOnNow = 0;
	boolean timeTestingError = false;
	
	public void testTimingCatapultTimeLimited() {
		if (m_robotName != "lisa") {
			if(timeTestingError==false) {
				try {
					//if(m_ThePult==null) {
					//	m_ThePult = new Catapult(m_RobotControllers.getCatapultLeft(), m_RobotControllers.getCatapultRight());
					//}
					Instant now = Instant.now();
					if (       now.toEpochMilli()-lastThrow.toEpochMilli()
							> cycleCurrentLength+cycleBufferMilli ) {
						double secondsTillReverse = (double)cycleCurrentLength/(double)1000;


						long milliSecondsTillReverse = (long) (secondsTillReverse*1000);
						int nanoSecondTillReverse = (int) (secondsTillReverse*1000*1000000 - milliSecondsTillReverse*1000000);
						
						// called and will print in the Log, and then pull from that log and can see how consistent it is in hitting the requested firing time.
					    Thread t = new Thread(() -> {
					    	try {
					     		Instant start = Instant.now();
								long nanoStart = System.nanoTime();
					     		Thread.sleep(milliSecondsTillReverse, nanoSecondTillReverse);
								long nanoEnd = System.nanoTime();
					     		Instant end = Instant.now();
					     		
//					     		double howLongActualFired = (end.getNano()-start.getNano())/1000;
//					     		double howLongActualFired = (end.getLong(ChronoField.MICRO_OF_SECOND)-start.getLong(ChronoField.MICRO_OF_SECOND));
					     		double howLongActualFired = (end.toEpochMilli()-start.toEpochMilli());
					     		double howLongActualFiredNano = (nanoEnd-nanoStart);

					     		System.out.println("Robot.testTimingCataputlTimeLimited thread: Wanted "+secondsTillReverse*1000+" milli-seconds; actually wait was "
//					     		+howLongActualFired+ " micro-seconds("+end.getNano()+"-"+start.getNano()+").  Error was " +(howLongActualFired -secondsTillReverse*1000000)+ " micro-seconds.");
//					     		+howLongActualFired+ " micro-seconds("+end.getLong(ChronoField.MICRO_OF_SECOND)+"-"+start.getLong(ChronoField.MICRO_OF_SECOND)+").  Error was " +(howLongActualFired -secondsTillReverse*1000000)+ " micro-seconds.");
//					     		+howLongActualFired+ " milli-seconds("+end.toEpochMilli()+"-"+start.toEpochMilli()+").  Error was " +(howLongActualFired -secondsTillReverse*1000)+ " milli-seconds.");
//					     		+(double)howLongActualFiredNano/1000000+ " milli-seconds("+(double) nanoEnd/1000000+"-"+(double)nanoStart/1000000+").  Error was " +((double)howLongActualFiredNano/1000000 -(double)(secondsTillReverse*1000))+ " milli-seconds.");
					     		+(double)howLongActualFiredNano/1000000+ " milli-seconds.  Error was " +((double)howLongActualFiredNano/1000000 -(double)(secondsTillReverse*1000))+ " milli-seconds.");
					     	} catch (InterruptedException e) {
					     		e.printStackTrace();
					     	}
					    });
					        
					    // start the thread that will run the above code totally independently of the normal robot thread
					    t.start();
					        
						//m_ThePult.LauchTimeLimited((double)cycleCurrentLength/(double)1000);
						cycleOnNow++;
						if(cycleOnNow>cyclesToRun) {
							cycleCurrentLength=cycleCurrentLength+cycleStartIncrement;
							cycleOnNow=0;
						}
					}
				}
				catch(Exception e) {
					System.out.println("testTimingCatapultTimeLimited testing stopping becuase exception "+e);
					timeTestingError=true;// stop test
				}
			}
		}
	}
    
}