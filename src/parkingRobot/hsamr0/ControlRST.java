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
	
	final double [] Einpark_V = {0,0.002390125,0.004759419,0.007106295,0.009429223,0.011726729,0.013997407,0.016239916,0.018452988,0.020635431,0.022786129,0.024904051,0.026988249,0.029037862,0.03105212,0.033030343,0.034971944,0.036876432,0.038743412,0.040572585,0.042363751,0.044116807,0.045831751,0.047508677,0.04914778,0.050749351,0.052313779,0.053841548,0.055333237,0.056789517,0.05821115,0.059598986,0.06095396,0.062277089,0.063569468,0.06483227,0.066066733,0.067274167,0.068455938,0.069613469,0.070748236,0.071861753,0.072955578,0.074031294,0.07509051,0.076134853,0.077165957,0.078185457,0.079194982,0.080196147,0.081190543,0.082179731,0.083165233,0.084148526,0.08513103,0.086114106,0.087099046,0.088087065,0.089079299,0.090076793,0.091080502,0.092091284,0.093109893,0.09413698,0.095173088,0.09621865,0.097273989,0.098339314,0.099414724,0.100500207,0.101595639,0.10270079,0.103815325,0.104938803,0.106070688,0.107210345,0.10835705,0.109509993,0.11066828,0.111830942,0.11299694,0.114165165,0.115334452,0.116503579,0.117671276,0.118836226,0.119997079,0.121152448,0.122300921,0.123441063,0.124571423,0.125690535,0.12679693,0.127889133,0.128965673,0.130025083,0.131065909,0.132086709,0.13308606,0.134062558,0.135014828,0.135941521,0.136841317,0.137712932,0.13855512,0.139366672,0.14014642,0.140893242,0.14160606,0.142283846,0.142925618,0.14353045,0.144097464,0.144625841,0.145114815,0.145563677,0.145971777,0.146338523,0.146663383,0.146945888,0.147185626,0.147382252,0.147535479,0.147645086,0.147710914,0.147732867,0.147710914,0.147645086,0.147535479,0.147382252,0.147185626,0.146945888,0.146663383,0.146338523,0.145971777,0.145563677,0.145114815,0.144625841,0.144097464,0.14353045,0.142925618,0.142283846,0.14160606,0.140893242,0.14014642,0.139366672,0.13855512,0.137712932,0.136841317,0.135941521,0.135014828,0.134062558,0.13308606,0.132086709,0.131065909,0.130025083,0.128965673,0.127889133,0.12679693,0.125690535,0.124571423,0.123441063,0.122300921,0.121152448,0.119997079,0.118836226,0.117671276,0.116503579,0.115334452,0.114165165,0.11299694,0.111830942,0.11066828,0.109509993,0.10835705,0.107210345,0.106070688,0.104938803,0.103815325,0.10270079,0.101595639,0.100500207,0.099414724,0.098339314,0.097273989,0.09621865,0.095173088,0.09413698,0.093109893,0.092091284,0.091080502,0.090076793,0.089079299,0.088087065,0.087099046,0.086114106,0.08513103,0.084148526,0.083165233,0.082179731,0.081190543,0.080196147,0.079194982,0.078185457,0.077165957,0.076134853,0.07509051,0.074031294,0.072955578,0.071861753,0.070748236,0.069613469,0.068455938,0.067274167,0.066066733,0.06483227,0.063569468,0.062277089,0.06095396,0.059598986,0.05821115,0.056789517,0.055333237,0.053841548,0.052313779,0.050749351,0.04914778,0.047508677,0.045831751,0.044116807,0.042363751,0.040572585,0.038743412,0.036876432,0.034971944,0.033030343,0.03105212,0.029037862,0.026988249,0.024904051,0.022786129,0.020635431,0.018452988,0.016239916,0.013997407,0.011726729,0.009429223,0.007106295,0.004759419,0.002390125};
	final double [] Einpark_W = {0,-0.0172131865761656,-0.0343018399004434,-0.0512792824466255,-0.0681583793135491,-0.0849514973383714,-0.101670464833404,-0.118326531375040,-0.134930327131487,-0.151491821272513,-0.168020279060099,-0.184524217274636,-0.201011357688053,-0.217488578353989,-0.233961862546785,-0.250436245246665,-0.266915757138898,-0.283403366170965,-0.299900916794483,-0.316409067108732,-0.332927224220474,-0.349453478240819,-0.365984535454236,-0.382515651317156,-0.399040564073465,-0.415551429910385,-0.432038760719272,-0.448491365669493,-0.464896297946999,-0.481238808148861,-0.497502305956668,-0.513668331830119,-0.529716540561724,-0.545624698607599,-0.561368697151098,-0.576922582857818,-0.592258608234979,-0.607347303407872,-0.622157570964101,-0.636656805286996,-0.650811037498374,-0.664585106755458,-0.677942858197439,-0.690847367317032,-0.703261189948127,-0.715146636422766,-0.726466067773431,-0.737182211158434,-0.747258490990637,-0.756659371577825,-0.765350706463529,-0.773300089118080,-0.780477199198575,-0.786854138299261,-0.792405748972622,-0.797109910833760,-0.800947807777311,-0.803904160740441,-0.805967421032794,-0.807129920010904,-0.807387971779111,-0.806741926622052,-0.805196173980211,-0.802759094929917,-0.799442965280677,-0.795263811513617,-0.790241222815150,-0.784398123374112,-0.777760509878843,-0.770357159750728,-0.762219316068621,-0.753380355368771,-0.743875444550240,-0.733741192986705,-0.723015305658949,-0.711736242700201,-0.699942890214385,-0.687674246612801,-0.674969128045754,-0.661865895809174,-0.648402207907373,-0.634614796273743,-0.620539270509927,-0.606209948415490,-0.591659713055005,-0.576919895654778,-0.562020183240490,-0.546988549620318,-0.531851208083289,-0.516632584015550,-0.501355305532110,-0.486040210171592,-0.470706365699326,-0.455371103102033,-0.440050059928067,-0.424757232223441,-0.409505033429258,-0.394304358734656,-0.379164653515932,-0.364093984632516,-0.349099113490503,-0.334185569921280,-0.319357726054280,-0.304618869487287,-0.289971275173728,-0.275416275553345,-0.260954328550123,-0.246585083149250,-0.232307442343402,-0.218119623308079,-0.204019214726614,-0.190003231238326,-0.176068165028820,-0.162210034620262,-0.148424430952264,-0.134706560871495,-0.121051288170905,-0.107453172338144,-0.0939065051879058,-0.0804053455651183,-0.0669435523155176,-0.0535148157277067,-0.0401126876565853,-0.0267306105424397,-0.0133619455432358,2.33490203117043e-16,0.0133619455432360,0.0267306105424400,0.0401126876565857,0.0535148157277070,0.0669435523155178,0.0804053455651186,0.0939065051879062,0.107453172338144,0.121051288170905,0.134706560871495,0.148424430952265,0.162210034620263,0.176068165028820,0.190003231238326,0.204019214726614,0.218119623308079,0.232307442343402,0.246585083149250,0.260954328550123,0.275416275553346,0.289971275173729,0.304618869487287,0.319357726054280,0.334185569921281,0.349099113490504,0.364093984632516,0.379164653515933,0.394304358734657,0.409505033429258,0.424757232223442,0.440050059928067,0.455371103102034,0.470706365699326,0.486040210171593,0.501355305532110,0.516632584015551,0.531851208083289,0.546988549620319,0.562020183240490,0.576919895654778,0.591659713055005,0.606209948415490,0.620539270509928,0.634614796273744,0.648402207907374,0.661865895809174,0.674969128045754,0.687674246612801,0.699942890214386,0.711736242700202,0.723015305658949,0.733741192986705,0.743875444550241,0.753380355368771,0.762219316068621,0.770357159750728,0.777760509878844,0.784398123374113,0.790241222815150,0.795263811513618,0.799442965280677,0.802759094929917,0.805196173980212,0.806741926622053,0.807387971779111,0.807129920010905,0.805967421032794,0.803904160740441,0.800947807777311,0.797109910833760,0.792405748972622,0.786854138299261,0.780477199198575,0.773300089118080,0.765350706463529,0.756659371577825,0.747258490990637,0.737182211158434,0.726466067773431,0.715146636422765,0.703261189948127,0.690847367317032,0.677942858197439,0.664585106755457,0.650811037498373,0.636656805286995,0.622157570964100,0.607347303407872,0.592258608234978,0.576922582857818,0.561368697151098,0.545624698607598,0.529716540561724,0.513668331830119,0.497502305956667,0.481238808148861,0.464896297946999,0.448491365669493,0.432038760719271,0.415551429910384,0.399040564073464,0.382515651317156,0.365984535454236,0.349453478240819,0.332927224220473,0.316409067108732,0.299900916794482,0.283403366170965,0.266915757138897,0.250436245246664,0.233961862546785,0.217488578353989,0.201011357688053,0.184524217274636,0.168020279060098,0.151491821272513,0.134930327131486,0.118326531375039,0.101670464833403,0.0849514973383710,0.0681583793135483,0.0512792824466251,0.0343018399004425,0.0172131865761651};
	
	//Parameter f眉r T_Rechnen 
	double t = 0.0; 
	double lastTime = 0;
	double CurrentTime = 0;
	
	double ForwardZeit = 1;
	double EinfahrZeit = 5;
	double BackwardZeit = 1;
	double StopZeit = 3;
	double AbfahrZeit = 5;
	double RichtungZeit = 4;
	
	double LastQuerAbweichen = 0.0;
	
	float s = 0;
	float dots = 0;
	
	float x1Now = 0;
	float x2Now = 0;
	
	float l1Now = 0;
	float l2Now = 0;
	
	float m1Now = 0;
	float m2Now = 0;
	
	float n1Now = 0;
	float n2Now = 0;
	
	float x1LastAbweichen = 0;
	float x2LastAbweichen = 0;
	
	int Park_T = 5;
	double t1 = EinfahrZeit;
	double t2 = EinfahrZeit + BackwardZeit;
	double t3 = EinfahrZeit + BackwardZeit + StopZeit;
	double t4 = EinfahrZeit + BackwardZeit + StopZeit + AbfahrZeit;
	
	double t0 = ForwardZeit;
	double t11 = ForwardZeit + EinfahrZeit;
	double t12 = ForwardZeit + EinfahrZeit + BackwardZeit;
	double t13 = ForwardZeit + EinfahrZeit + BackwardZeit + StopZeit;
	double t14 = ForwardZeit + EinfahrZeit + BackwardZeit + StopZeit + AbfahrZeit;
 
	
	float thetai = 0;
	boolean fahr = false;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 0.15;
	double angularVelocity = 1.35;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	float Richtung = 0;
	float PoseRichtung = 0;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

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
	
	
	public ControlMode getControlMode() {
		return this.currentCTRLMODE;
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
		  case PARK_CTRL1	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL1_ALGO();
		  					  break;	
		  case PARK_CTRL2	: update_PARKCTRL_Parameter();
    		  				  exec_PARKCTRL2_ALGO();
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
		T_rechnen(); // 
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
		setPose(navigation.getPose());
		
		//T_rechnen();
		//EinfahrRichtungBestimmen();		
	}
	private void EinfahrRichtungBestimmen(){
		float x = startPosition.getX();
		float y = startPosition.getY();
		if(x < 165 && y < 15){   	// 鑰冭檻璇�宸�
			Richtung = 0;
		}else if(x >= 165){
			Richtung = (float)((90/180)*Math.PI);
		}else if(x < 165 && y >15){
			Richtung = (float)((180/180)*Math.PI);
		}
	}
	  
	public void T_rechnen(){			
		double CurrentTime = (double)System.currentTimeMillis();
		double delta_t = ((CurrentTime - lastTime)/1000);			
		if (lastTime == 0) {
			delta_t = 0;
		}
		lastTime = CurrentTime;
		t = t + delta_t; 
		//RConsole.println(t+ ";");
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
    /*杞﹁韩鏂瑰悜 */
    int RichtungFlag = 0;
    private void RichtungEinstellen(float HeadingZiel){
    	if(navigation.getPose().getHeading()/Math.PI*180 > HeadingZiel/(Math.PI)*180 + 1){
    		SpeedControl(0.08,-0.08); 
        }else if(navigation.getPose().getHeading()/Math.PI*180 < HeadingZiel/(Math.PI)*180 - 1){
        	SpeedControl(-0.08,0.08);
        }else{
        	RichtungFlag = 1;
        }
    }
	
   private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	   float x = navigation.getPose().getX();
	   float y = navigation.getPose().getY();
	   float xziel = 0.3f;		// Ziel 麓Koordination
	   float yziel = 0.0f;
	   float toleranz = 0.02f;
		//float xziel = destination.getX();
		//float yziel = destination.getY();
		float X = 0.3f;
		float Y = 0.0f;
		//float X = destination.getX() - startPosition.getX();
		//float Y = destination.getY() - startPosition.getY();
		double Y_X = Math.atan(Math.abs(Y/X)); 	// rad
		double X_Y = Math.atan(Math.abs(X/Y));
		
		if( X > 0 && Y > 0){
			PoseRichtung = (float)(Y_X);
		}else if(X < 0 && Y < 0){
			PoseRichtung = (float)((Y_X) + (Math.PI));	// 閽堝�逛笉鍚岀殑鍧愭爣绯荤煫姝ｅ姬搴�
		}else if(X < 0 && Y > 0){
			PoseRichtung = (float)((X_Y) + (Math.PI)*0.5);
		}else if(X > 0 && Y < 0){
			PoseRichtung = (float)((X_Y) + (Math.PI*1.5));
		}else if(X > 0 && Y == 0){
			PoseRichtung = 0;
		}else if(X < 0 && Y == 0){
			PoseRichtung = (float)Math.PI;
		}else if(X == 0 && Y > 0){
			PoseRichtung = (float)((Math.PI)*0.5);
		}else if(X == 0 && Y < 0){
			PoseRichtung = (float)((Math.PI)*1.5);
		}else{
			stop();
		}
		
		if(t<4) {
			
			RichtungEinstellen(PoseRichtung);
		}
		else{
    		if (x > xziel-toleranz && x < xziel+toleranz && y > yziel-toleranz && y < yziel+toleranz){
    			stop();
    		}else {
			double k1 = 0.0;
			double k2 = 0;
			double v = 1.5;
			double Querabweichen = perception.getControlOdo().getOdoMeasurement().getUSum()/1000;
			double SPomega =- k1 * Querabweichen - k2 * LastQuerAbweichen; 
			LastQuerAbweichen = Querabweichen;
			drive(v,SPomega);
			//RConsole.println(Querabweichen +";");
    		}
		}
   }
   
   private float s (double t){
   	//Aufgabe 3.4 Polynom f篓鹿r s
   	float s = (float) ((Math.pow((t / Park_T), 2)) * (3 - 2 * t / Park_T));
   	return s;
   }
   private float dots (double t){
   	//Aufgabe 3.4 Polynom f篓鹿r dots
   	float dots = (float) ((6 * t / (Math.pow(Park_T, 2))) * (1 - t / Park_T));
   	return dots;
   }
   private void Backward(){
   	drive(-0.10,0);

   }	
   private void Forward(){
   	drive(0.10,0);
   }
	/**
	 * PARKING along the generated path
	 */
   int Einpark_Counter=0;
   float a3, a2;
   float sx, sy, st, cos_t, sin_t;

   public void setParameter(double a_2, double a_3) {
	   a3 = (float) a_3;
	   a2 = (float) a_2;
   }
   
   public void setStart(double x, double y, double t) {
	   sx = (float) x;
	   sy = (float) y;
	   st = (float)t;
	   cos_t = (float) Math.cos(t);
	   sin_t = (float) Math.sin(t);
   }
   
   float last_d1 = 0;
   float last_e = 0;
   
	private void exec_PARKCTRL1_ALGO(){
		//Aufgabe 3.4
		//RConsole.println( navigation.getPose().getX()+","+navigation.getPose().getY()+";");
//		if(Einpark_Counter < Einpark_V.length)
//		  drive(Einpark_V[Einpark_Counter],Einpark_W[Einpark_Counter]);
//		else {
//			this.currentCTRLMODE = ControlMode.INACTIVE;
//			Einpark_Counter = 0;
//		}
//		Einpark_Counter++;
		float f = 9999;
		int N = 0;
		float _px = this.currentPosition.getX() - sx;
		float _py = this.currentPosition.getY() - sy;
		float px = (float)( cos_t*_px + sin_t*_py);
		float py = (float)(-sin_t*_px + cos_t*_py);
		float x2 = px;
		float x2_2 = 0;
		float x2_3 = 0;
		while((N < 20)&&(Math.abs(f) > 0.001)) {
		  x2_2 = x2*x2;
		  x2_3 = x2_2*x2;
		  f = 2.0f*(x2 - px) + 2.0f*(3.0f*a3*x2_2 + 2.0f*a2*x2)*(a3*x2_3 + a2*x2_2 - py);
		  x2 = x2 - 0.05f*f;
		  N++;
		}
		x2_2 = x2*x2;
		x2_3 = x2_2*x2;
		float y2 = a3*x2_3 + a2*x2_2;
		float d1 = (float)Math.sqrt((px-x2)*(px-x2)+(py-y2)*(py-y2));
		float tx, ty;
		tx = this.destination.getX();
		ty = this.destination.getY();
		float d2 = (float)Math.sqrt((px-tx)*(px-tx)+(py-ty)*(py-ty));
//		RConsole.println(d1+","+d2+";");
		if((d2 < 0.10)||(px >= tx)||(x2 > tx)) {
			float e = st - currentPosition.getHeading();
			
			while(e > Math.PI)
				e -= 2.0*Math.PI;
			while(e < -Math.PI)
			    e += 2.0*Math.PI;
			
			if(Math.abs(e) < 0.02) {
			  this.currentCTRLMODE = ControlMode.INACTIVE;
			  last_d1 = 0;
			}
			else
			  drive(0.05,e*3.0f + (e-last_e)*0.2f);
			
			last_e = e;
		}
		else {
			last_e = st - currentPosition.getHeading();
			float w = d1*20.0f + (d1-last_d1)*10.0f;
			if(py > y2)
			  drive(0.08,-w);
			else
			  drive(0.08, w);
		}
		
		last_d1 = d1;
	}
	
	private void exec_PARKCTRL2_ALGO(){
    	//Aufgabe 3.4
		if(Einpark_Counter < Einpark_V.length)
		  drive(Einpark_V[Einpark_Counter],-Einpark_W[Einpark_Counter]);
		else {
			this.currentCTRLMODE = ControlMode.INACTIVE;
			Einpark_Counter = 0;
		}
		Einpark_Counter++;
    }
	private void park(float x1a,float x2a,float x1b,float x2b,int richtung){
		
		if(richtung == 1){
			thetai = 0;
			//thetai  = navigation.getPose().getHeading() - Richtung;
		}else if(richtung == 0){
			thetai = 0;
			//thetai  = Richtung - navigation.getPose().getHeading();
		}
		float thetaf = 0;
		float ki = 0.5f;
		float kf = 0.5f;
		float k1 = 0;
		float k2 = 0;
		float k3 = 0;
		float k4 = 0;
		
		float a0 = (float) x1a;
		float a1 = (float) (ki * Math.cos(thetai));
		float a2 = (float) (3 * (x1b - x1a) - 2 * ki * Math.cos(thetai) - kf * Math.cos(thetaf));
		float a3 = (float) (2 * (x1a - x1b) + ki * Math.cos(thetai) + kf * Math.cos(thetaf));
		float b0 = (float) x2a;
		float b1 = (float) (ki * Math.sin(thetai));
		float b2 = (float) (3 * (x2b - x2a) - 2 * ki * Math.sin(thetai) - kf * Math.sin(thetaf));
		float b3 = (float) (2 * (x2a - x2b) + ki * Math.sin(thetai) + kf * Math.sin(thetaf));

		float x1 = (float) (a0 + a1 * s + a2 * Math.pow(s, 2) + a3 * Math.pow(s, 3));
		float dx1 = (float) (a1 + 2 * a2 * s + 3 * a3 * Math.pow(s, 2));
		float ddx1 = (float) (2 * a2 + 6 * a3 * s);
		float x2 = (float) (b0 + b1 * s + b2 * Math.pow(s, 2) + b3 * Math.pow(s, 3));
		float dx2 = (float) (b1 + 2 * b2 * s + 3 * b3 * Math.pow(s, 2));
		float ddx2 = (float) (2 * b2 + 6 * b3 * s);
		//double v = dots * Math.sqrt(Math.pow(dx1, 2) + Math.pow(dx2, 2));
		//double w = dots * (ddx2 * dx1 - dx2 * ddx1) / (Math.pow(dx1, 2) + Math.pow(dx2, 2));
		
		float x1Abweichen = x1Now - x1;
		float x2Abweichen = x2Now - x2;
		double ddxx1 = (double)((ddx1 - k1 * x1Abweichen - k2 * (x1Abweichen - x1LastAbweichen)));
		double ddxx2 = (double)((ddx2 - k3 * x2Abweichen - k4 * (x2Abweichen - x2LastAbweichen)));
		x1LastAbweichen = x1Abweichen;
		x2LastAbweichen = x2Abweichen;
		if (richtung == 1){
			double v = dots * Math.sqrt(Math.pow(dx1, 2) + Math.pow(dx2, 2));
			double w = dots * (ddxx2 * dx1 - dx2 * ddxx1) / (Math.pow(dx1, 2) + Math.pow(dx2, 2));
			drive(v,w);
			//RConsole.println(v+",");
		}else if(richtung == 0){
			double v = -dots * Math.sqrt(Math.pow(dx1, 2) + Math.pow(dx2, 2));
			double w = dots * (ddxx2 * dx1 - dx2 * ddxx1) / (Math.pow(dx1, 2) + Math.pow(dx2, 2));
			drive(v,w);
		}
	}
    private void exec_INACTIVE(){
    	drive(0,0);
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
	private void exec_LINECTRL_ALGO(){	
			double kp = 0.00;	//璋冭瘯
		    double ki = 0.002;
		    double kd = 0.0;
		    int lasterror = 0;
		    int preerror = 0;
		    int errorsum = 0;
			int left = perception.getLeftLineSensorValue();  
			int right = perception.getRightLineSensorValue();
			int differenz = left-right;
			int sollwert = 0;
			int error = sollwert-differenz;
			double w = (double)(kp*(error - lasterror) + ki*error + kd*(error - 2*lasterror + preerror));
			preerror = lasterror;
			lasterror = error;
			errorsum = errorsum + error;
			
			if(left >5 && right >5 ) {
				drive(0.15,w);
				L_Flag = 0;
				R_Flag = 0;
			}
			/*鍒ゆ柇宸﹁浆锛孡eftsnesorValue =0,  RightsensorValue = 100 
			 * 鍦ㄨ浆鍚戣繃绋嬩腑浼氬嚭鐜� 宸﹀彸鍧囦负0鐨勬儏鍐� 鍥犳�よ�佸姞鍏ョ姸鎬佸垽鏂�鏉′欢 
			 * */
			if (left <= 5 && R_Flag == 0 )
			{   
				L_Flag = 1;	  //鏍囧織浣嶇疆 1 鍙宠浆
				SpeedControl(0.0,0.30);
			}
			if (right <= 5 && L_Flag == 0) {

				R_Flag = 1;	  //鏍囧織浣嶇疆 1 宸﹁浆
				SpeedControl(0.30,0.0);
			}
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
		float DistancePerDeg = DistancePerTurn/360;//DistancePerDeg = 0.000488692 m/掳
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
		//vl = (vl / DistancePerDeg)*PI*WheelDiameter/180;  // Einheit 掳/s --> (2Pi /360)rad /s --> m/s  o.25m/s-->0.5m/s
		//vr = (vr / DistancePerDeg)*PI*WheelDiameter/180; 
		//vl = vl / ( DistancePerDeg);
		//vr = vr / ( DistancePerDeg); // f眉hrer 100   2.5 PWM-50%   3--62  
		//RConsole.println(vl +", "+ vr +"," + omega + ";");
		//vl = vl /0.8;
		//vr = vr /0.8;
		//int leftspeed = (int)vl;
		//int rightspeed = (int)vr;
		//RConsole.println(vm +", "+ omega + ";");		
		SpeedControl(vl,vr);
		//RConsole.println(vl +", "+ vr + "," + omega + ";");
	}
	
 	double L_lasterror = 0;
	double L_errorsum = 0;
	double L_preerror = 0;
	double R_lasterror = 0;
	double R_errorsum = 0;
	double R_preerror = 0;
	private void SpeedControl(double L_controlSpeed, double R_controlSpeed) {
		double L_Speed = L_controlSpeed;		// Tastverhaeltnis 80  Geschwindigkeit 0.25m/s 瀹為檯閫熷害
		double R_Speed = R_controlSpeed;
				//娴嬮€焍'b
		if(L_Speed<0 ) {
			L_Speed = -L_Speed;
		}
		if(R_Speed<0 ) {
			R_Speed = -R_Speed;
		}
		while((this.encoderLeft.getUpdateFlag()!=true) || (this.encoderRight.getUpdateFlag() != true)) {
			try {					
				Thread.sleep(2);
				}
			catch(InterruptedException e) {
				e.printStackTrace();
			}
		}
			this.angleMeasurementLeft  = this.encoderLeft.getEncoderMeasurement();
			this.angleMeasurementRight  = this.encoderRight.getEncoderMeasurement();
			double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum() / ((double)this.angleMeasurementLeft.getDeltaT()/1000.0);  //degree/seconds
			double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000.0); //degree/seconds			
			double vLeft		= (leftAngleSpeed  * Math.PI * 0.028) / 180 ; //velocity of left  wheel in m/s	
			double vRight		= (rightAngleSpeed * Math.PI * 0.028) / 180 ; //velocity of right wheel in m/s		

			
			//double L_Speed_soll  = 0.0042*L_controlSpeed - 0.0725;  // Einheit m/s y = 0.0042x - 0.0725
			//double R_Speed_soll  = 0.0039*R_controlSpeed - 0.0412; // y = 0.0039x - 0.04
			double L_Speed_soll = L_Speed;//+ 10.438;	// 璁惧畾閫熷害 --->鍗犵┖姣�
			double R_Speed_soll = R_Speed;//+ 10.683;
					
		 		/*LeftMotor*/
			double KLp= 0.5;
			double KLi = 0.1;
			double KLd = 0.0;
			double L_error = L_Speed - vLeft;
			double LeftSpeed = L_Speed_soll +  (KLp*L_error + KLi*L_errorsum + KLd*( L_error - L_lasterror));
			L_lasterror = L_error;
			L_errorsum = L_errorsum + L_error;
			leftMotor.forward();
			if(L_controlSpeed == 0) {
				leftMotor.stop();	
				LeftSpeed = 0;
				}
			if(L_controlSpeed<0) {
				leftMotor.backward();
			}
			leftMotor.setPower((int)(LeftSpeed*255.0f)+5);
				/*RightMotor*/
			double KRp = 0.5;
			double KRi = 0.1;
			double KRd = 0;
			double R_error = R_Speed - vRight;
			double RightSpeed = R_Speed_soll +  (KRp*R_error + KRi*R_errorsum + KRd*( R_error - R_lasterror));
			R_preerror = R_lasterror;
			R_lasterror = R_error;
			R_errorsum = R_errorsum + R_error;
			
			rightMotor.forward();
			if(R_controlSpeed == 0) {
				rightMotor.stop();
					RightSpeed = 0;
			}
			if(R_controlSpeed<0) {
				rightMotor.backward();
			}
	
			rightMotor.setPower((int)(RightSpeed*238.38f)+5);
			//RConsole.println(vLeft+", "+ vRight + ", " + w_Grad +";");
		}
}