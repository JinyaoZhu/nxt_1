package parkingRobot.hsamr0;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.hsamr0.ControlRST;
import parkingRobot.hsamr0.HmiPLT;
import parkingRobot.hsamr0.NavigationAT;
import parkingRobot.hsamr0.PerceptionPMP;

import lejos.nxt.comm.RConsole;
import lejos.util.Matrix;
import lejos.geom.Point;


/**
   Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
   with specialization 'automation, measurement and control'.
   <p>
   Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
   parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
   structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
   <p>
   Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
   state machines (FSM). It may be advantageous to nest more than one FSM.
   <p>
   For the other parts there are interfaces defined and every part has to be realized in one main module class.
   Every class (except guidance) has additionally to start its own thread for background computation.
   <p>
   It is important that data witch is accessed by more than one main module class thread is only handled in a
   synchronized context to avoid inconsistent or corrupt data!
*/
public class GuidanceAT_Nav {

    /**
       states for the main finite state machine. This main states are requirements because they invoke different
       display modes in the human machine interface.
    */
    public enum CurrentStatus {
      /**
         indicates that robot is following the line and maybe detecting parking slots
      */
      DRIVING,
      /**
         indicates that robot is performing an parking maneuver
      */
      INACTIVE,
      /**
         indicates that shutdown of main program has initiated
      */
      EXIT,
      
      EINPARKEN,
      
      AUSPARKEN
    }


    /**
       state in which the main finite state machine is running at the moment
    */
    protected static CurrentStatus currentStatus  = CurrentStatus.INACTIVE;
    /**
       state in which the main finite state machine was running before entering the actual state
    */
    protected static CurrentStatus lastStatus   = CurrentStatus.INACTIVE;
    
    private static double[] pCoeff;


    /**
       one line of the map of the robot course. The course consists of a closed chain of straight lines.
       Thus every next line starts where the last line ends and the last line ends where the first line starts.
       This documentation for line0 hold for all lines.
    */
    static Line line0 = new Line(  0,  0, 180,  0);
    static Line line1 = new Line(180,  0, 180, 60);
    static Line line2 = new Line(180, 60, 150, 60);
    static Line line3 = new Line(150, 60, 150, 30);
    static Line line4 = new Line(150, 30,  30, 30);
    static Line line5 = new Line( 30, 30,  30, 60);
    static Line line6 = new Line( 30, 60,   0, 60);
    static Line line7 = new Line(  0, 60,   0,  0);

    //  static Line line0 = new Line(  0,  0, 75,  0);
    //  static Line line1 = new Line(75,  0, 75, 60);
    //  static Line line2 = new Line(75, 60, 45, 60);
    //  static Line line3 = new Line(45, 60, 45, 30);
    //  static Line line4 = new Line(45, 30,  0, 30);
    //  static Line line5 = new Line( 0, 30,  0, 0); //(cm) **in other routine (m)

    /**
       map of the robot course. The course consists of a closed chain of straight lines.
       Thus every next line starts where the last line ends and the last line ends where the first line starts.
       All above defined lines are bundled in this array and to form the course map.
    */
    static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};
    //  static Line[] map = {line0, line1, line2, line3, line4, line5};

    /**
       main method of project 'ParkingRobot'

       @param args standard string arguments for main method
       @throws Exception exception for thread management
    */
    public static void main(String[] args) throws Exception {
      currentStatus = CurrentStatus.INACTIVE;
      lastStatus    = CurrentStatus.EXIT;
      int parkStatus = 0;
      boolean parking  = false;
      boolean leaving = false;
      float ParkingSlot_L = 0;
      float ParkingSlot_H = 0;
      Point curr_position;
      int cnt = 0;

      // Generate objects

      NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
      NXTMotor rightMotor = new NXTMotor(MotorPort.A);

      //RConsole.openBluetooth(0);

      IMonitor monitor = new Monitor();

      IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
      //    perception.calibrateLineSensors();

      INavigation navigation = new NavigationAT(perception, monitor);
      navigation.setMap(map);
      IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
      INxtHmi   hmi        = new HmiPLT(perception, navigation, control, monitor);

      //    monitor.startLogging();

      Sound.setVolume(100);

      while (true) {
        showData(navigation, perception);
        curr_position = navigation.getPose().getLocation();
        switch ( currentStatus )
        {
          case DRIVING:
            // MONITOR (example)
            //          monitor.writeGuidanceComment("Guidance_Driving");
        	
            //Into action
            if ( lastStatus != CurrentStatus.DRIVING ) {
            	
              if(lastStatus == CurrentStatus.AUSPARKEN)
                cnt = 0;
              else
            	cnt = 14;
              
              control.setCtrlMode(ControlMode.LINE_CTRL);
              navigation.setParkingState(false);
            }

            //While action
            {
              if( cnt > 15) //delay ~1.5 second
                navigation.setDetectionState(true);
              else
            	cnt ++;   
            }

            //State transition check
            lastStatus = currentStatus;
            if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ) {
              currentStatus = CurrentStatus.INACTIVE;
            }
            else if ( Button.ENTER.isDown() ) {
              currentStatus = CurrentStatus.INACTIVE;
              while (Button.ENTER.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if ( Button.ESCAPE.isDown() ) {
              currentStatus = CurrentStatus.EXIT;
              while (Button.ESCAPE.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
              currentStatus = CurrentStatus.EXIT;
            }
            else if ((hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)) {
              currentStatus = CurrentStatus.EINPARKEN;
            }
            //Leave action
            if ( currentStatus != CurrentStatus.DRIVING ) {
              //nothing to do here
            }
            break;
          case INACTIVE:
            //Into action
            if ( lastStatus != CurrentStatus.INACTIVE ) {
              control.setCtrlMode(ControlMode.INACTIVE);
            }

            //While action
            {
              navigation.setDetectionState(false);
              navigation.setParkingState(true);// corner update = off(nav)
            }

            //State transition check
            lastStatus = currentStatus;
            if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ) {
              if ( parkStatus == 0) {
                currentStatus = CurrentStatus.DRIVING;
              }
              else if (parkStatus == 1) {
                currentStatus = CurrentStatus.AUSPARKEN;
              }
            }
            else if ( Button.ENTER.isDown() ) {
              currentStatus = CurrentStatus.DRIVING;
              while (Button.ENTER.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if ( Button.ESCAPE.isDown() ) {
              currentStatus = CurrentStatus.EXIT;
              while (Button.ESCAPE.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
              currentStatus = CurrentStatus.EXIT;
            }
            else if ((hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS)&&(parkStatus == 0)) {
              currentStatus = CurrentStatus.EINPARKEN;
            }

            //Leave action
            if ( currentStatus != CurrentStatus.INACTIVE ) {
              //nothing to do here
            }
            break;
          case EXIT:
            hmi.disconnect();
            /** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
              // monitor.sendOfflineLog();
            */
            monitor.stopLogging();
            System.exit(0);
            break;
                       
          case EINPARKEN:
            // MONITOR (example)
            // monitor.writeGuidanceComment("Guidance_Einparken");

            //Into action

            if ( lastStatus != CurrentStatus.EINPARKEN ) {
              control.setCtrlMode(ControlMode.LINE_CTRL);
            }
            //While action
            {
              navigation.setDetectionState(false);
              
            }
            
            if(parking == false)
            {
            	navigation.setParkingState(false);
            	int index = getParkingSlotIndex(navigation.getParkingSlots(),hmi.getSelectedParkingSlot());
            	ParkingSlot P = navigation.getParkingSlots()[index];
	            Point p2 = P.getFrontBoundaryPosition();
	            Point p1 = P.getBackBoundaryPosition();
	            double d = curr_position.subtract(p1).length();
	            //RConsole.println(d+","+hmi.getSelectedParkingSlot()+",");
	            if (d < 0.30f) {
	              Sound.buzz();
	              control.setCtrlMode(ControlMode.INACTIVE);
	              ParkingSlot_L = p2.subtract(p1).length();
	              ParkingSlot_H = 0.25f;
	              pCoeff =  getPathParameters(new Point(0,0), new Point(ParkingSlot_L/2.0f,-ParkingSlot_H));
	              //RConsole.println(pCoeff[0]+","+pCoeff[1]+","+pCoeff[2]+","+pCoeff[3]+",");
	              //RConsole.println(ParkingSlot_L+","+ParkingSlot_H+",");
	              navigation.setParkingState(true);
	              control.setCtrlMode(ControlMode.PARK_CTRL1);
	              parking = true;
	            }
            }
            
            if(control.getControlMode() == ControlMode.INACTIVE) {
            	parkStatus = 1;
            }
            

            //State transition check
            lastStatus = currentStatus;
            if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ) {
              currentStatus = CurrentStatus.INACTIVE;
            }
            else if ( Button.ENTER.isDown() ) {
              currentStatus = CurrentStatus.INACTIVE;
              while (Button.ENTER.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if ( Button.ESCAPE.isDown() ) {
              currentStatus = CurrentStatus.EXIT;
              while (Button.ESCAPE.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
              currentStatus = CurrentStatus.EXIT;
            }
            else if (parkStatus == 1) {
              control.setCtrlMode(ControlMode.INACTIVE);
              currentStatus = CurrentStatus.INACTIVE;
              parking = false;
            }
            //Leave action
            if ( currentStatus != CurrentStatus.EINPARKEN ) {
              //nothing to do here
            }
            break;

          case AUSPARKEN:
            // MONITOR (example)
            //          monitor.writeGuidanceComment("Guidance_Ausparken");
            //Into action
            if ( lastStatus != CurrentStatus.AUSPARKEN ) {

            }
            //While action
            {
            	navigation.setDetectionState(false);
                navigation.setParkingState(true);
                
                 if(leaving == false)
                 {
     	            pCoeff =  getPathParameters(new Point(0,0), new Point(ParkingSlot_L/2.0f,ParkingSlot_H));
     	            control.setCtrlMode(ControlMode.PARK_CTRL2);
     	            leaving = true;
                 }
            
                if(control.getControlMode() == ControlMode.INACTIVE) {
                	parkStatus = 0;
                }
            }

            //State transition check
            lastStatus = currentStatus;
            if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ) {
              currentStatus = CurrentStatus.INACTIVE;
            }
            else if ( Button.ENTER.isDown() ) {
              currentStatus = CurrentStatus.INACTIVE;
              while (Button.ENTER.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if ( Button.ESCAPE.isDown() ) {
              currentStatus = CurrentStatus.EXIT;
              while (Button.ESCAPE.isDown()) {
                Thread.sleep(1); //wait for button release
              }
            }
            else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
              currentStatus = CurrentStatus.EXIT;
            }
            else
            {
              //currentStatus = CurrentStatus.DRIVING;
            }
            
            if(parkStatus == 0) {
            	navigation.setParkingState(false);
            	control.setCtrlMode(ControlMode.LINE_CTRL);
            	currentStatus = CurrentStatus.DRIVING;
            	leaving = false;
            }
            //Leave action
            if ( currentStatus != CurrentStatus.AUSPARKEN ) {
              //nothing to do here
            }
            break;
            
            default: break;
        }

        Thread.sleep(100);
      }
    }


    /**
       returns the actual state of the main finite state machine as defined by the requirements

       @return actual state of the main finite state machine
    */
    public static CurrentStatus getCurrentStatus() {
      return GuidanceAT_Nav.currentStatus;
    }

    /**
       plots the actual pose on the robots display

       @param navigation reference to the navigation class for getting pose information
    */
    protected static void showData(INavigation navigation, IPerception perception) {
      LCD.clear(0);
      LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);
      LCD.clear(1);
      LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
      LCD.clear(2);
      LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);
      LCD.clear(4);
      switch (getCurrentStatus()) {
        case DRIVING:
          LCD.drawString("STATE:DRIVING", 0, 4);
          break;
        case INACTIVE:
          LCD.drawString("STATE:INACTIVE", 0, 4);
          break;
        case EXIT:
          LCD.drawString("STATE:EXIT", 0, 4);
          break;
        case EINPARKEN:
        	LCD.drawString("STATE:EINPARKEN", 0, 4);
        	break;
        case AUSPARKEN:
        	LCD.drawString("STATE:AUSPARKEN", 0, 4);
        	break;
        default:
          break;
      }
      
      LCD.drawString("IR dist:" + perception.getFrontSideSensorDistance() + "mm", 0, 5);

      //    perception.showSensorData();

      //      if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
      //      LCD.drawString("HMI Mode SCOUT", 0, 3);
      //    }else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
      //      LCD.drawString("HMI Mode PAUSE", 0, 3);
      //    }else{
      //      LCD.drawString("HMI Mode UNKNOWN", 0, 3);
      //    }
    }

    private static int getParkingSlotIndex(ParkingSlot[] P, int ID) {

      for (int i = 0; i < P.length; i++)
        if (P[i].getID() == ID)
          return i;
      return -1;
    }

    private static double[] getPathParameters(Point p1, Point p2) {
      double x_a = p1.x;
      double x_a2 = x_a*x_a;
      double x_a3 = x_a2*x_a;
      double y_a = p1.y;
      double x_e = p2.x;
      double x_e2 = x_e*x_e;
      double x_e3 = x_e2*x_e;
      double y_e = p2.y;
      double [][] A_ = {{1, x_a, x_a2, x_a3}, {0, 1, 2*x_a, 3*x_a2}, {1, x_e, x_e2, x_e3}, {0,1,2*x_e,3*x_e2}};
      Matrix A = new Matrix(A_, 4, 4);
      double [][] b_ = {{y_a}, {0}, {y_e}, {0}};
      Matrix b = new Matrix(b_, 4, 1);
      Matrix A_inv = A.inverse();
      Matrix x = A_inv.times(b);
      return x.getColumnPackedCopy(); //[a0,...,a3]
    }
    
    public static double[] getPathCoeff() {
    	return pCoeff;
    }
}