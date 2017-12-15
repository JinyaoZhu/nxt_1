package parkingRobot.hsamr0;

import java.util.ArrayList;
import java.util.List;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.Sound;
import lejos.nxt.comm.RConsole;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IMonitor;

import parkingRobot.hsamr0.NavigationThread;


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
	boolean is_DEBUG = false;
	
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
	static final float LEFT_WHEEL_RADIUS	= 	0.0275f; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final float RIGHT_WHEEL_RADIUS	= 	0.0275f; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final float WHEEL_DISTANCE		= 	0.1400f; // only rough guess, to be measured exactly and maybe refined by experiments

	
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
	
	// my median filter
	MedianFilter medianfilterIR = new MedianFilter(21);
	MedianFilter medianfilterEncoderL = new MedianFilter(3);
	MedianFilter medianfilterEncoderR = new MedianFilter(3);
	
	float update_deltaT = 0.02f;
	
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
		
		// wait until sensor are updated. Synchronize with perception thread
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
		int corner_state = this.calculateLocation();
		
		this.setDetectionState(true);
		
		if (this.parkingSlotDetectionIsOn)
			if((corner_state==0)||(corner_state==1)||(corner_state==4))
				this.detectParkingSlot(this.pose.getX(),this.pose.getY(),this.pose.getHeading(),
						this.update_deltaT,(float)frontSideSensorDistance*0.001f,current_omega,corner_state);
		
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
		ParkingSlot[] parking_slot_out = new ParkingSlot[parking_slot_list.size()];
		for(int i=0;i<parking_slot_list.size(); i++) {
			parking_slot_out[i] = parking_slot_list.get(i);
		}
		return parking_slot_out;
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

		this.frontSensorDistance	 = Math.max(40,Math.min(perception.getFrontSensorDistance(),300.0));
		this.frontSideSensorDistance = Math.max(40,Math.min(perception.getFrontSideSensorDistance(),300.0));
		this.backSensorDistance		 = Math.max(40,Math.min(perception.getBackSensorDistance(),300.0));
		this.backSideSensorDistance	 = Math.max(40,Math.min(perception.getBackSideSensorDistance(),300.0));
	}
	
	/**
	 * calculates the robot pose from the measurements
	 */    		
	float current_v = 0;
	float current_omega = 0;
	
	private int calculateLocation(){
		
		// raw data from perception module
		float leftAngleSpeed_raw = (float)this.angleMeasurementLeft.getAngleSum()  / ((float)this.angleMeasurementLeft.getDeltaT()/1000.0f);  //degree/seconds
		float rightAngleSpeed_raw = (float)this.angleMeasurementRight.getAngleSum() / ((float)this.angleMeasurementRight.getDeltaT()/1000.0f); //degree/seconds
		
		// apply median filter
		float leftAngleSpeed 	= medianfilterEncoderL.update(leftAngleSpeed_raw);
		float rightAngleSpeed 	= medianfilterEncoderR.update(rightAngleSpeed_raw);

		float vLeft		= (float) ((leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180.0f) ; //velocity of left  wheel in m/s
		float vRight    = (float) ((rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180.0f) ; //velocity of right wheel in m/s		
		
		float v_encoder    = (vRight + vLeft)*0.5f;
		float omega_encoder = (float) ((vRight - vLeft) / WHEEL_DISTANCE); //angular velocity of robot in rad/s
	
		float dt       = (this.angleMeasurementLeft.getDeltaT())/1000.0f;
		update_deltaT = dt;
		float v = current_v = v_encoder;
		float omega = current_omega = omega_encoder;
		
		int cornerState = updateConnerState(omega,dt);
				
		// update pose with kinematic model
		float xResult	  = (float) (this.pose.getX() + v * Math.cos(this.pose.getHeading()) * dt);
		float yResult	  = (float) (this.pose.getY() + v * Math.sin(this.pose.getHeading()) * dt);
		float angleResult = warpToPi(this.pose.getHeading() + omega * dt);
			
		// fusion with the map information
		angleResult = angleResult + 0.15f*(warpToPi(target_theta[cornerState]-angleResult));
		angleResult = warpToPi(angleResult);
		
		if(cornerState%2==0) {
			yResult = yResult + 0.2f*(map[cornerState].y1*0.01f - yResult);
		}
		else {
			xResult = xResult + 0.2f*(map[cornerState].x1*0.01f - xResult);
		}
		
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);		 
		
//		RConsole.println(xResult+","+yResult+","+cornerState+";");
		return cornerState;
	}
	
	float cornerTriggerTime = 0;
	float stateChangedTime = 0;
	int currentCornerState = 0;
	
	/*
	 * Check which corner the robot reached
	 */
	private int updateConnerState(float omega, float dt) {
		final float omega_trigger = 1.2f; // at least how fast the robot turn at the corner
		final float corner_time_trigger = 0.3f; // at least how long the robot turn
		final float state_change_time_threshold = 0.8f; // at least how long since last state-changing
		
		if(Math.abs(omega) > omega_trigger) {
			cornerTriggerTime += dt;
		}
		else {
			cornerTriggerTime = 0.0f;
		}
		
		if((cornerTriggerTime > corner_time_trigger)&&(stateChangedTime > state_change_time_threshold)) {
			stateChangedTime = 0.0f;
			cornerTriggerTime = 0.0f;
			currentCornerState += 1;
			
			if(currentCornerState > (map.length-1))
				currentCornerState = 0;
			
//			if(is_DEBUG)
//				Sound.twoBeeps();
		}
		
		stateChangedTime += dt;
		
		return currentCornerState;
	}
	
	
	// restraint a angle(rad) from +pi to -pi
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
	float last_measure_park_detect  = 0;
	float park_detect_update_time = 0;
	List<ParkingSlot> parking_slot_list = new ArrayList<ParkingSlot>();
	float[] measure_park_detect_FIFO = new float[21];
	int measure_park_detect_FIFO_index = 0;
	float distance_to_wall=0;
	Point parking_start_point,parking_end_point;
	int last_corner_state = 0;
	
	private void detectParkingSlot(float x, float y,float theta, float dt,float raw_measure,float omega, int corner_state){
		
		float measure = medianfilterIR.update(raw_measure);
		final float diff_threshold = 1.0f;
		final float update_interval_threshold = 0.6f;
		final float omega_threshold = 999.5f;
		final float max_diff = 6.0f;
		final float max_distance_to_wall = 20.0f;
		
		float diff = (measure - last_measure_park_detect)/dt;
		diff = Math.max(-max_diff,Math.min(diff,max_diff));//constraint
		
//		RConsole.println(diff+","+measure+";");
		
		measure_park_detect_FIFO[measure_park_detect_FIFO_index++] = measure;
		
		if(measure_park_detect_FIFO_index == measure_park_detect_FIFO.length) 
			measure_park_detect_FIFO_index = 0;
		
		
		if((park_detect_update_time > update_interval_threshold)&&(Math.abs(omega) < omega_threshold)) {
			if((last_measure_park_detect != 0.30f)&&(measure == 0.30f)) {
				distance_to_wall = measure_park_detect_FIFO[measure_park_detect_FIFO_index]+0.05f;//plus offset
				
				parking_start_point = new Point(x+(float)Math.sin(theta)*distance_to_wall,y-(float)Math.cos(theta)*distance_to_wall);
				
//				RConsole.println("Detected start point");
				park_detect_update_time = 0;
				Sound.playNote(Sound.PIANO,2000, 100);
			}
			else if(((last_measure_park_detect == 0.30f)&&(measure != 0.30f))&&(distance_to_wall != 0)&&(distance_to_wall <= max_distance_to_wall)) {
					ParkingSlotStatus status;
					
					parking_end_point = new Point(x+(float)Math.sin(theta)*distance_to_wall,y-(float)Math.cos(theta)*distance_to_wall);
					
					if(distance2(parking_start_point,parking_end_point) > 0.30f)
						status = ParkingSlotStatus.SUITABLE_FOR_PARKING;
					else
						status = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
					
					float Qm = Math.abs(diff)/max_diff*100.0f;//measurement quality
					ParkingSlot new_pks = new ParkingSlot(parking_slot_list.size(),parking_start_point,parking_end_point, status, (int)Qm);
					
					// matching with old parking slots
					int match_index = matchParkingSlot(new_pks,parking_slot_list);
					if(match_index == -1) {// no match found
						if(parking_slot_list.size() < 20) // max number of parking slots = 20
							parking_slot_list.add(new_pks);
						if(is_DEBUG) {
							RConsole.println("add Parking slot ID"+new_pks.getID());
							RConsole.println("start:"+parking_start_point.x+","+parking_start_point.y);
							RConsole.println("end:"+parking_end_point.x+","+parking_end_point.y);
						}
					}
					else {
						ParkingSlot matched_pks = parking_slot_list.get(match_index);
						int ID = matched_pks.getID();
						Point new_f_point = new_pks.getFrontBoundaryPosition();
						Point old_f_point = matched_pks.getFrontBoundaryPosition();
						Point new_b_point = new_pks.getBackBoundaryPosition();
						Point old_b_point = matched_pks.getBackBoundaryPosition();
						float old_Qm = matched_pks.getMeasurementQuality();
						float new_Qm = new_pks.getMeasurementQuality();
						// update (like a normalized product of two 1D Gaussian PDFs)
						Point update_f_point = merge2Point(new_f_point,old_f_point,new_Qm*new_Qm,old_Qm*old_Qm);
						Point update_b_point = merge2Point(new_b_point,old_b_point,new_Qm*new_Qm,old_Qm*old_Qm);
						float update_Q_m_2 = old_Qm*old_Qm + new_Qm*new_Qm;
						float update_Q_m = 1.0f/invSqrt(update_Q_m_2); // possibly exceed 100
						ParkingSlot update_psk = new ParkingSlot(ID,update_b_point,update_f_point,status,(int)update_Q_m);
						
						parking_slot_list.set(match_index,update_psk);
						
						if(is_DEBUG) {
							RConsole.println("merge with ID:"+match_index);
							RConsole.println("front:"+update_f_point.x+","+update_f_point.y);
							RConsole.println("back:"+update_b_point.x+","+update_b_point.y);
							RConsole.println("Qm:"+update_Q_m);
						}
					}
				
//				    Sound.buzz();
					Sound.playNote(Sound.PIANO,2000, 100);
					
					distance_to_wall = 0;
					park_detect_update_time = 0;
			}
		}
		
		// at the corner reset the timer
		if(last_corner_state != corner_state) {
			if(corner_state == 4)
				park_detect_update_time = -1.0f;
			else
				park_detect_update_time = 0;
			distance_to_wall = 0;
		}
		
		last_measure_park_detect = measure;
		park_detect_update_time += dt;
		last_corner_state = corner_state;
	}
	
	/**
	 * @param new parking slot
	 * @param list of old parking slots
	 * @return index of the matched parking slot
	 */
	private int matchParkingSlot(ParkingSlot new_pks,List<ParkingSlot> pks_list) {
		
		float match_threshold = 0.2f;
		float d = 99999.0f;
		int match_index = -1;
		
		if(pks_list.size()<=1) //do not need to match
			return -1;
		
		Point new_p1 = new_pks.getFrontBoundaryPosition();
		Point new_p2 = new_pks.getBackBoundaryPosition();
		
		// find the minimum distance between new and old parking slot
		// return the index of the most similar old parking slot
		for(int i=0; i<pks_list.size(); i++) {
			ParkingSlot old_pks = pks_list.get(i);
			Point old_p1 = old_pks.getFrontBoundaryPosition();
			Point old_p2 = old_pks.getBackBoundaryPosition();
			float d1 = distance2(new_p1,old_p1); // distance between new and old front points
			float d2 = distance2(new_p2,old_p2); // distance between new and old back points
			//TODO: take measurement quality into consideration
			if((d1+d2)<d) {
				d = d1+d2;
				match_index = i;
			}
		}
		
		if(d < match_threshold) {
//			RConsole.println("min distance:"+d);
			return match_index;
		}
		else
			return -1;
	}
	
	// fast sqrt
	private float invSqrt(float x) {
	    float xhalf = 0.5f * x;
	    int i = Float.floatToIntBits(x);
	    i = 0x5f3759df - (i >> 1);
	    x = Float.intBitsToFloat(i);
	    x *= (1.5f - xhalf * x * x);
	    return x;
	}
	
	// distance between two points
	private float distance2(Point p1,Point p2) {
		float x2 = (p1.x-p2.x)*(p1.x-p2.x);
		float y2 = (p1.y-p2.y)*(p1.y-p2.y);
		return 1.0f/invSqrt(x2+y2);
	}
	
	// merge two points with weight
	private Point merge2Point(Point p1,Point p2,float w1, float w2) {
		
		float w_sum = w1+w2;

		w1 /= w_sum; // normalize
		w2 /= w_sum;
		
		float x = p1.x*w1+p2.x*w2;
		float y = p1.y*w1+p2.y*w2;
		
		return new Point(x,y);
	}
}
