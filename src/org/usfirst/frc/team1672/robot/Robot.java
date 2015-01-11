package org.usfirst.frc.team1672.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.livewindow.LiveWindow; //liveWindow is the simulation for use in Ubuntu

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive chassis;
	
	
	Joystick driveStick;
	Joystick liftStick;
	
	Talon frontLeft;
	Talon frontRight;
	Talon rearLeft;
	Talon rearRight;
	
	Jaguar lift; //lift motor
	Jaguar test; //not a "real" motor
	
	//sensor constants n' stuff
	//constants are CAPITALIZED so that we know they are constants
	private final double GROUND_SENSOR_DISTANCE = 0.1; //ground level | This value is to give a buffer from hitting the ground
	private final double FIRST_SENSOR_DISTANCE = 12.2; //1st level 
	private final double SECOND_SENSOR_DISTANCE = 24.3; //2nd level
	private final double THIRD_SENSOR_DISTANCE = 36.4; //3rd level
	private final double FOURTH_SENSOR_DISTANCE = 48.5; //4th level
	private final double FIFTH_SENSOR_DISTANCE = 56; //5th level (container)
	
	private double liftHeight; //in inches | totes = 12.1 in, containers = 29 in
	private double desiredHeight; //the height we want the liftHeight to be at 
	private double movementDirection; //the desired direction, in degrees, of the entire robot
	private double movementMagnitude; //the desired magnitude of velocity of the entire robot
	 
	private boolean[] buttonsPressed; //this array keeps track of buttons pressed, there are 11 in total
	private boolean triggerPressed;
	
	/*Button Raw IDs*/
	//public final int FIRE_BUTTON = 1;
	public final int LIFT_TO_ONE = 2; //Right stick only
	public final int LIFT_TO_TWO = 3; //Right stick only
	public final int LIFT_TO_THREE = 4;
	public final int LIFT_TO_FOUR = 5;
	public final int LIFT_TO_FIVE = 10;
	//public final int CAMERA_TOGGLE = 3;
	 
	Ultrasonic liftSensor = new Ultrasonic(6, 6);
	
	//sensitivity constants
	public final double DIRECTIONAL_SENSITIVITY = 0.0; 	//sensitivity to how the joystick's direction changes
														//from 0.0 to 1.0 ONLY. any less or more and there will be problems.
														//0.0==> robot moves on 45deg. angle closest to joystick angle.
														//1.0==> robot moves on exact angle of joystick.
														//any number in between, and the robot moves at angles that are multiples of (DIRECTIONAL_SENSITIVITY * 45deg.)
	public final double MAGNITUDE_SENSITIVITY = 0.0; 	//sensitivity to how the joystick's direction changes
														//from 0.0 to 1.0 ONLY. any less or more and there will be problems.
														//0.0==> motors spin on multiples of 0.25 only.
														//1.0==> motors spin according to exact magnitude of joystick push
														//any number in between, and the motors spin at speeds that are multiples of (MAGNITUDE_SENSITIVITY * 0.25)
	
	int autoLoopCounter;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	chassis = new RobotDrive(1,2,3,4); //four motor drive config using mecanum drive
    	
    	frontLeft = new Talon(1);
    	frontRight = new Talon(2);
    	rearLeft = new Talon(3);
    	rearRight = new Talon(4);
    	
    	lift = new Jaguar(5);
    	test = new Jaguar(11);
    	
    	driveStick = new Joystick(0);
    	liftStick = new Joystick(1);
    	
    	desiredHeight = GROUND_SENSOR_DISTANCE;
    	buttonsPressed = new boolean[11];
    	triggerPressed = false;
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	liftSensor.setAutomaticMode(true);
		liftSensor.setEnabled(true);
		chassis.setSafetyEnabled(false);
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    	//TEST CODE ONLY----
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
    		chassis.mecanumDrive_Polar(0.5, 0.0, 0.0); 	// drive forwards
			autoLoopCounter++;
		} 
    	else 
    	{
			chassis.mecanumDrive_Polar(0.0, 0.0, 0.0); 	// stop robot
		}
    	//------------------
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    }

    /**
     * This function is called periodically during operator control - no need for a loop
     */
    public void teleopPeriodic() {
    	/*
		UPDATE DA SENSOR INFO-------------------
		*/
		liftHeight = liftSensor.getRangeInches();
		//--------------------------------------
		
		/*
		GET DA INPUT WITH THIS LOOP-------------
		*/
		for(int i=0;i<buttonsPressed.length;i++)
		{
			buttonsPressed[i] = liftStick.getRawButton(i);
		}
		triggerPressed = liftStick.getTrigger();
		//---------------------------------------
		
		/*
		NOW DO INPUT ACTIONS--------------------
		*/
		doInputActions();
		//now do the actual lifting
		liftToward(desiredHeight);
		//---------------------------------------
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
    public void liftToward(double desiredHeight){
    	if(liftHeight > desiredHeight)
		{
			lift.set(-0.5);
		}
		if(liftHeight < desiredHeight)
		{
			lift.set(0.5);
		}
		if(liftHeight == desiredHeight)
		{
			//will it ever actually EXACTLY EQUAL the number we want?
			//maybe we should use an 'approximate equals' method instead of ==
			System.out.println("Desired height for lift reached:" + desiredHeight);
			lift.stopMotor();
		}
    }
    public void doInputActions(){
    	
    	//drive mecanum wheels according to joystick and sensitivity-------------
    	double snapIncrement = 45.0 * (1.0 - DIRECTIONAL_SENSITIVITY); //movement direction will snap to multiples of this number
    	if(snapIncrement < 0.1) { movementDirection = driveStick.getDirectionDegrees(); } //catch any floating-point calculation troubles and prevent divide-by-zero
    	else { movementDirection = (int)(driveStick.getDirectionDegrees()/snapIncrement) * snapIncrement; }
    	
    	snapIncrement = 0.10 * (1.0 - MAGNITUDE_SENSITIVITY); //movement magnitude will snap to multiples of this number
    	if(snapIncrement < 0.001) { movementMagnitude = driveStick.getMagnitude(); } //catch any floating-point calculation troubles and prevent divide-by-zero
    	else { movementMagnitude = (int)(driveStick.getMagnitude()/0.25) * 0.25; }
    	chassis.mecanumDrive_Polar(movementMagnitude, movementDirection, driveStick.getTwist());
    	//-----------------------------------------------------------------------
    	
    	//do actions specified by buttons------
    	if(buttonsPressed[LIFT_TO_ONE])
    		desiredHeight = FIRST_SENSOR_DISTANCE;
    	if(buttonsPressed[LIFT_TO_TWO])
    		desiredHeight = SECOND_SENSOR_DISTANCE;
    	if(buttonsPressed[LIFT_TO_THREE])
    		desiredHeight = THIRD_SENSOR_DISTANCE;
    	if(buttonsPressed[LIFT_TO_FOUR])
    		desiredHeight = FOURTH_SENSOR_DISTANCE;
    	if(buttonsPressed[LIFT_TO_FIVE])
    		desiredHeight = FIFTH_SENSOR_DISTANCE;
    	//-------------------------------------
    	
    	//now do actions specified by trigger--
    	if(triggerPressed)
    		desiredHeight = GROUND_SENSOR_DISTANCE;
    	//-------------------------------------
    	
    }
    
}
