package parkingRobot.hsamr0;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

import lejos.nxt.LCD;
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
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;

	int L_Flag = 0;
	int R_Flag = 0;
	
	float kp = 0;	//Parameter von PID
    float ki = 0;
    float kd = 0;
    int error = 0;
    int lasterror = 0;
    int preerror = 0;
    int errorsum = 0;
    
    double rl = 0.0;
	double rr = 0.0;
	double vl = 0.0;
	double vr = 0.0;
	
	double t = 0.0;
	
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

	double lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
  
	public void T_rechnen(){
		
		double CurrentTime = (double)System.currentTimeMillis();
		double delta_t = ((CurrentTime - lastTime)/1000);
		//LCD.drawString("dt " + delta_t, 0, 5);
		if (lastTime == 0) {
			delta_t = 0;
		}
		lastTime = CurrentTime;
		t = t + delta_t; 
		//RConsole.println(t+ ";");
		}
	
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
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
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
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();		
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity);
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
    double t_LeftTurn = 0;
    double t_RightTurn = 0;
//    float line_follow_v = 2.5f;
	private void exec_LINECTRL_ALGO(){
		double kp = 0.01;	//调试
	    double ki = 0.04;
	    double kd = 0.001;
	    int lasterror = 0;
	    int preerror = 0;
	    int errorsum = 0;
		int left = perception.getLeftLineSensorValue();  
		int right = perception.getRightLineSensorValue();
		int differenz = left-right;
		int sollwert = 0;
		int error = sollwert-differenz;
		
		//double w = (double)(kp*(error - lasterror) + ki*error + kd*errorsum); //PID
	    /*此处可加入分段PID  通过判断 Diiferenz的大小   差值较小 调节小  差值较大 调节大*/
		double w = (double)(kp*(error - lasterror) + ki*error + kd*(error - 2*lasterror + preerror));
	    //double w = (double)(kp*(error - lasterror) + ki*error + kd*(error - 2*lasterror + preerror));
		preerror = lasterror;
		lasterror = error;
		errorsum = errorsum + error;
		
		if(left >=7 && right >=7 ) {
			drive(2.5,w);
			L_Flag = 0;
			R_Flag = 0;
		}
		/*判断左转，LeftsnesorValue =0,  RightsensorValue = 100 
		 * 在转向过程中会出现 左右均为0的情况 因此要加入状态判断条件 
		 * */
		if (left <7 && R_Flag == 0 )
		{   
			L_Flag = 1;	  //标志位置 1 左转
			SpeedControl(0.0,0.3);
		}
			//drive(0,w);   //或者改变算法 此时 应该转 90°  drive(0,90) \速度为0 角度为90  
		if (right <= 7 && L_Flag == 0) { //&& 
			R_Flag = 1;	  //标志位置 1 左转
			SpeedControl(0.3,0.0);
		}
		//if(right <= 5 ) {		//abbiegen nach rechts
		//	SpeedControl(60,0);
		//}
		//RConsole.println(L_Flag +", "+ R_Flag + ";");

	
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
	private void drive(double vm, double omega){
		//Aufgabe 3.2
		 
		float WheelDiameter = 0.056f;  // m
		float PI = (float)Math.PI;
		float DistancePerTurn = PI*WheelDiameter; //m
		float DistancePerDeg = DistancePerTurn/360;//DistancePerDeg = 0.000488692 m/°
		double d = 0.14;  //m 
		if(omega != 0){
				double rm = vm/omega;
				rl = rm - d/2;
				rr = rm + d/2;
				if(rm != 0){				
					vl = rl*vm/rm ;
					vr = rr*vm/rm ;
				
				}
				else{
						vr = (d/2)*omega  ;
						vl = -vr          ;
					}
				}
			else{
				vl = vm  ;
				vr = vm  ;
			}
		
		//vl = (vl / DistancePerDeg)*PI*WheelDiameter/180;  // Einheit °/s --> (2Pi /360)rad /s --> m/s  o.25m/s-->0.5m/s
		//vr = (vr / DistancePerDeg)*PI*WheelDiameter/180; 
		vl = vl / ( DistancePerDeg *100*250);
		vr = vr / ( DistancePerDeg *100*250); // führer 100   2.5 PWM-50%   3--62  
		//vl = vl / ( DistancePerDeg);
		//vr = vr / ( DistancePerDeg);
		//int leftspeed = (int)vl;
		//int rightspeed = (int)vr;
		//RConsole.println(vm +", "+ omega + ";");		
		SpeedControl(vl,vr);
		//RConsole.println(vl +", "+ vr + "," + omega + ";");
	}

 	double L_lasterror = 0;
	double L_errorsum = 0;
	double R_lasterror = 0;
	double R_errorsum = 0;
	
	private void SpeedControl(double L_controlSpeed, double R_controlSpeed) {
		double L_Speed = L_controlSpeed;		// Tastverhaeltnis 80  Geschwindigkeit 0.25m/s 实际速度
		double R_Speed = R_controlSpeed;
				//测速b'b

			while((this.encoderLeft.getUpdateFlag()!=true) || (this.encoderRight.getUpdateFlag() != true)) {
				try {
					Thread.sleep(5);
				}
				catch(InterruptedException e) {
					e.printStackTrace();
				}
			}
			this.angleMeasurementLeft  = this.encoderLeft.getEncoderMeasurement();
			this.angleMeasurementRight  = this.encoderRight.getEncoderMeasurement();
			double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum() / ((double)this.angleMeasurementLeft.getDeltaT()/1000.0);  //degree/seconds
			double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000.0); //degree/seconds			
			double vLeft		= (leftAngleSpeed  * Math.PI * 0.028 ) / 180 ; //velocity of left  wheel in m/s	
			double vRight		= (rightAngleSpeed * Math.PI * 0.028) / 180 ; //velocity of right wheel in m/s		
			double w 		= (vRight - vLeft) / 0.14; //angular velocity of robot in rad/s
			
			double w_Grad = w * 180 / Math.PI;
			
			//double L_Speed_soll  = 0.0042*L_controlSpeed - 0.0725;  // Einheit m/s y = 0.0042x - 0.0725
			//double R_Speed_soll  = 0.0039*R_controlSpeed - 0.0412; // y = 0.0039x - 0.04
			double L_Speed_soll = 237.93 * L_Speed + 10.438;	// 设定速度 --->占空比
			double R_Speed_soll = 256.38 * R_Speed + 10.683;
					
		 		/*LeftMotor*/
			double KL1 = 100;
			double KL2 = 1;
			double L_error = L_Speed - vLeft;
			double LeftSpeed = L_Speed_soll + (KL1*(L_error - L_lasterror) + KL2*L_error);
			L_lasterror = L_error;
			L_errorsum = L_errorsum + L_error;
			leftMotor.forward();
			if(L_controlSpeed == 0) {
				leftMotor.stop();	
				LeftSpeed = 0;
				}
			leftMotor.setPower((int)LeftSpeed);
				/*RightMotor*/
			double KR1 = 100;
			double KR2 = 1;
			double R_error = R_Speed - vRight;
			double RightSpeed = R_Speed_soll + (KR1*(R_error - R_lasterror) + KR2*R_error);
			R_lasterror = R_error;
			R_errorsum = R_errorsum + R_error;
			rightMotor.forward();
			rightMotor.setPower((int)RightSpeed);
			if(R_controlSpeed == 0) {
				rightMotor.stop();
					RightSpeed = 0;
			}
//			RConsole.println(vLeft+", "+ vRight + ", " + w_Grad +";");
			//RConsole.println(w_Grad +";");
		}
}