package parkingRobot.hsamr0;

import lejos.geom.Line;
import lejos.nxt.comm.RConsole;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import parkingRobot.hsamr0.NavigationThread;

import parkingRobot.hsamr0.PF;


/**
 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation{
	
	// my particle filter :)
//	PF pf = new PF();
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
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
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement
	 * between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between
	 * actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensor pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;


	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.140; // only rough guess, to be measured exactly and maybe refined by experiments

	
	/**
	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	Line[] map 								= null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();

	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();	
				
		navThread.setPriority(Thread.MAX_PRIORITY - 2);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	float target_theta[]; // for complementary filter
	public void setMap(Line[] map){
		this.map = map;
		target_theta = new float[map.length];
		// initialize target_theta for the complementary filter
		for(int i=0;i<map.length;i++) {
			target_theta[i] = (float) Math.atan2(map[i].y2 - map[i].y1, map[i].x2-map[i].x1);
		}
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		
		// wait until sensor are updated
		while((this.encoderLeft.getUpdateFlag()!=true) ||(this.encoderRight.getUpdateFlag()!=true)) {
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		// get sensor data
		this.updateSensors();
		
		// compute pose
		this.calculateLocation();
		// particle filter
//		pf.updatePose(0.0f, 0.0f,1.0f,1.0f,1.0f,1.0f,0.05f);
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		
		// MONITOR (example)
//		monitor.writeNavigationComment("Navigation");
	}
	
	
	// Outputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		return null;
	}
	
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		 	
	
	/**
	 * calculates the robot pose from the measurements
	 */    		
	
	private void calculateLocation(){
		float leftAngleSpeed 	= (float)this.angleMeasurementLeft.getAngleSum()  / ((float)this.angleMeasurementLeft.getDeltaT()/1000.0f);  //degree/seconds
		float rightAngleSpeed 	= (float)this.angleMeasurementRight.getAngleSum() / ((float)this.angleMeasurementRight.getDeltaT()/1000.0f); //degree/seconds

		float vLeft		= (float) ((leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180.0f) ; //velocity of left  wheel in m/s
		float vRight    = (float) ((rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180.0f) ; //velocity of right wheel in m/s		
		float omega_encoder = (float) ((vRight - vLeft) / WHEEL_DISTANCE); //angular velocity of robot in rad/s
	
		float xResult 		= 0;
		float yResult 		= 0;
		float angleResult 	= 0;
		
		float dt       = (this.angleMeasurementLeft.getDeltaT())/1000.0f;
		
		float v_encoder    = (vRight + vLeft)/2.0f;
	
		float v = v_encoder;
		float omega = omega_encoder;
		
		int cornerState = updateConnerState(omega,dt);
		
		
		xResult			= (float) (this.pose.getX() + v * Math.cos(this.pose.getHeading()) * dt);
		yResult			= (float) (this.pose.getY() + v * Math.sin(this.pose.getHeading()) * dt);
		angleResult 	= warpToPi(this.pose.getHeading() + omega * dt);
			
		angleResult = angleResult + 0.15f*(warpToPi(target_theta[cornerState]-angleResult));
		angleResult = warpToPi(angleResult);
		
		if((cornerState==0)||(cornerState==2)||(cornerState==4)) {
			yResult = yResult + 0.2f*(map[cornerState].y1*0.01f - yResult);
		}
		else {
			xResult = xResult + 0.2f*(map[cornerState].x1*0.01f - xResult);
		}
		
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);		 
		
		RConsole.println(xResult+","+yResult+","+cornerState+";");
	}
	
	static float cornerTriggerTime = 0;
	static float stateChangedTime = 0;
	static int currentCornerState = 0;
	
	/*
	 * Check which corner the robot reached
	 */
	private int updateConnerState(float omega, float dt) {
		
		if(Math.abs(omega) > 1.2) {
			cornerTriggerTime += dt;
		}
		else {
			cornerTriggerTime = 0.0f;
		}
		
		if((cornerTriggerTime > 0.5)&&(stateChangedTime > 1.0)) {
			stateChangedTime = 0.0f;
			cornerTriggerTime = 0.0f;
			currentCornerState += 1;
			
			if(currentCornerState > (map.length-1))
				currentCornerState = 0;
		}
		
		stateChangedTime += dt;
		
		return currentCornerState;
	}
	
	private float warpToPi(float x) {
		while(x > Math.PI)
			x -=  2.0f*(float)Math.PI;
		while(x < -Math.PI)
			x +=  2.0f*(float)Math.PI;
		
		return x;
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		return; // has to be implemented by students
	}
}
