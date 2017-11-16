package parkingRobot.hsamr0;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

import lejos.nxt.comm.RConsole;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.027; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.027; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.140; // only rough guess, to be measured exactly and maybe refined by experiments

	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 0.0;
	double angularVelocity = 0.0;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
  
	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensorValue();
		this.lineSensorLeft  		= perception.getLeftLineSensorValue();
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 2);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
		
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensorValue();
		this.lineSensorLeft  		= perception.getLeftLineSensorValue();	
		
		if(this.lineSensorRight	> 100)
			this.lineSensorRight= 100;
		else if(this.lineSensorRight < 0)
			this.lineSensorRight=0;
		
		if(this.lineSensorLeft	> 100)
			this.lineSensorLeft= 100;
		else if(this.lineSensorLeft < 0)
			this.lineSensorLeft=0;
		
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
//		this.drive(this.velocity, this.angularVelocity);
	}
	
    private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
//    static float accu_time = 0;
    static float omega_line_follow;
	private void exec_LINECTRL_ALGO(){
		
		float v_line_follow = 0.14f;
				
		if(lineSensorLeft < 5) {
			if(lineSensorRight < 5) {
				
			}
			else {
				omega_line_follow = v_line_follow*14.8f;
			}
		}
		
		if(lineSensorRight < 5) {
			if(lineSensorLeft<5) {
	
			}
			else {
				omega_line_follow = -v_line_follow*14.8f;
			}
		}
		
		if((lineSensorRight >= 5) && (lineSensorLeft>=5)) {
			omega_line_follow = (lineSensorRight-lineSensorLeft)*0.006f;
		}
		
		drive(v_line_follow,omega_line_follow);
		
//		RConsole.print(this.lineSensorLeft+",");
//		RConsole.println(this.lineSensorRight+";");
		
		
//		if(accu_time > 2.0f)
//			drive(0.1f,0.0f);
//		else
//			drive(-0.1f,0.0f);
//	
//		accu_time += 0.03f;
//		
//		if(accu_time>4.0f)
//			accu_time = 0;
		
		
//		leftMotor.forward();
//		rightMotor.forward();
//		
//		leftMotor.setPower(200);
//		rightMotor.setPower(0);
//		
//		// wait until encoders are updated
//		while((this.encoderLeft.getUpdateFlag()!=true) ||(this.encoderRight.getUpdateFlag()!=true)) {
//			try {
//				Thread.sleep(5);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//		}
//		
//		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
//		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();
//	    float leftAngleSpeed 	= (float)this.angleMeasurementLeft.getAngleSum()  / ((float)this.angleMeasurementLeft.getDeltaT()/1000.0f);  //degree/seconds
//		float rightAngleSpeed 	= (float)this.angleMeasurementRight.getAngleSum() / ((float)this.angleMeasurementRight.getDeltaT()/1000.0f); //degree/seconds
//		
//		
		
		
//		leftMotor.forward();
//		rightMotor.forward();
//		int lowPower = 1;
//		int highPower = 30;
//		
//		// MONITOR (example)
//		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
//		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
//
//        if(this.lineSensorLeft == 2 && (this.lineSensorRight == 1)){
//			
//			// when left sensor is on the line, turn left
//    	    leftMotor.setPower(lowPower);
//			rightMotor.setPower(highPower);
//			
//			// MONITOR (example)
//			monitor.writeControlComment("turn left");
//			
//		} 
//        else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 1)){
//		
//			// when right sensor is on the line, turn right
//			leftMotor.setPower(highPower);
//			rightMotor.setPower(lowPower);
//			
//			// MONITOR (example)
//			monitor.writeControlComment("turn right");
//		}
//		else if(this.lineSensorLeft == 2 && (this.lineSensorRight == 0)){
//			
//			// when left sensor is on the line, turn left
//			leftMotor.setPower(lowPower);
//			rightMotor.setPower(highPower);
//			
//			// MONITOR (example)
//			monitor.writeControlComment("turn left");
//			
//		} 
//		else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 0)){
//		
//			// when right sensor is on the line, turn right
//			leftMotor.setPower(highPower);
//			rightMotor.setPower(lowPower);
//			
//			// MONITOR (example)
//			monitor.writeControlComment("turn right");
//		}
//		else if(this.lineSensorLeft == 1 && this.lineSensorRight == 0) {
//				
//			// when left sensor is on the line, turn left
//			leftMotor.setPower(lowPower);
//			rightMotor.setPower(highPower);
//			
//			// MONITOR (example)
//			monitor.writeControlComment("turn left");
//		} 
//		else if(this.lineSensorRight == 1 && this.lineSensorLeft == 0) {
//			
//			// when right sensor is on the line, turn right
//			leftMotor.setPower(highPower);
//			rightMotor.setPower(lowPower);
//			
//			// MONITOR (example)
//			monitor.writeControlComment("turn right");
//		}
		
	}
	

	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */
	private void drive(float v, float omega){
		//Aufgabe 3.2
		float a11 = (float) (RIGHT_WHEEL_RADIUS/2.0f);
		float a12 = (float) (LEFT_WHEEL_RADIUS/2.0f);
		float a21 = (float) (RIGHT_WHEEL_RADIUS/WHEEL_DISTANCE);
		float a22 = (float) (-LEFT_WHEEL_RADIUS/WHEEL_DISTANCE);
		float inv_det = 1.0f/(a11*a22-a12*a21);
		
		float speed_right = (float) (inv_det*( a22*v-a12*omega)*RIGHT_WHEEL_RADIUS);
		float speed_left  = (float) (inv_det*(-a21*v+a11*omega)*LEFT_WHEEL_RADIUS);
		
		setSpeedAndDrive(speed_left,speed_right);
	}
	
	
	static float i_error_l = 0;
	static float i_error_r = 0;
	
	private void setSpeedAndDrive(float speed_left,float speed_right) {
		
		int power_l;
		int power_r;
		
		// wait until encoders are updated
		while((this.encoderLeft.getUpdateFlag()!=true) ||(this.encoderRight.getUpdateFlag()!=true)) {
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();
		float leftAngleSpeed 	= 0.017453f*(float)this.angleMeasurementLeft.getAngleSum()  / ((float)this.angleMeasurementLeft.getDeltaT()/1000.0f);  //rad/seconds
		float rightAngleSpeed 	= 0.017453f*(float)this.angleMeasurementRight.getAngleSum() / ((float)this.angleMeasurementRight.getDeltaT()/1000.0f); //rad/seconds
		
		float target_omega_l = (float) (speed_left/LEFT_WHEEL_RADIUS);
		float target_omega_r = (float) (speed_right/RIGHT_WHEEL_RADIUS);
		
		float error_omega_l = (float) (target_omega_l - leftAngleSpeed);
		float error_omega_r = (float) (target_omega_r - rightAngleSpeed);
		
		i_error_l += error_omega_l*(float)this.angleMeasurementLeft.getDeltaT()/1000.0f;
		i_error_r += error_omega_r*(float)this.angleMeasurementRight.getDeltaT()/1000.0f;
		
		float omega_to_pwm = 7.80f; //rad per second
		
//		RConsole.print(target_omega_l +",");
//		RConsole.println(leftAngleSpeed +";");
		
//		float u_l = omega_to_pwm*(error_omega_l*0.6f + i_error_l*0.0f);
//		float u_r = omega_to_pwm*(error_omega_r*0.6f + i_error_r*0.0f);
		
		float u_l = omega_to_pwm*(error_omega_l*0.0f + i_error_l*0.0f);
		float u_r = omega_to_pwm*(error_omega_r*0.0f + i_error_r*0.0f);
		
		power_l = (int) (target_omega_l*omega_to_pwm + u_l + 0.5f);
		power_r = (int) (target_omega_r*omega_to_pwm + u_r + 0.5f);
		
		if (power_l >= 0) {
			leftMotor.forward();
			leftMotor.setPower(power_l);
		}
		else {
			leftMotor.backward();
			leftMotor.setPower(-power_l);
		}
			
		if (power_r >= 0) {
			rightMotor.forward();
			rightMotor.setPower(power_r);
		}
		else {
			rightMotor.backward();
			rightMotor.setPower(-power_r);
		}
	}
}