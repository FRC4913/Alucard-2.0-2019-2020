package org.usfirst.frc.team4913.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.vision.VisionPipeline;
import edu.wpi.first.wpilibj.vision.VisionRunner;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//RobotDrive myRobot = new RobotDrive(0, 1);
	Timer timer = new Timer();
	XboxController controller;
	Spark climbingMotor;
	Spark LED;
	boolean LEDswitch =true;
	RobotDrive myRobot;
	CameraServer server;
	
	
	
	//private final Object imgLock = new Object();
	//private double centerX=0.0;
	
	//Ultrasonic ultra = new Ultrasonic(1,1);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		controller= new XboxController(0);
		climbingMotor=new Spark(0);
		LED=new Spark(2);
		//myRobot =new RobotDrive (climbingMotor,LED);
		
		server = CameraServer.getInstance();
		server.startAutomaticCapture(0);
		//server.startAutomaticCapture();
		
		//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    //camera.setResolution(320, 240);
	    CvSink image = server.getVideo();
	    image.setEnabled(true);
	    Mat source = new Mat();
	    long errorcode = image.grabFrame(source);
	    System.out.println("grabFrame:"+ errorcode );
	    if(errorcode !=0){
	    System.out.println("Mat height:"+ source.height() );
	    System.out.println("Mat width:"+ source.width() );
	    }
	    //GripPipeline.process(source);
	    //VisionThread visionThread;
	    
	    //visionThread a = new VisionThread(camera, Pipeline, pipeline -> {
	    /*visionThread = new VisionThread(camera, Pipeline, pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	            }
	        }
	    });
	    visionThread.start();*/
		//Pipeline.process(source);		
	    
	   // ultra.setAutomaticMode(true);
	    
	        
			
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (timer.get() < 2.0) {
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
		} else {
			myRobot.drive(0.0, 0.0); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//ultrasonic testing
		//double range = ultra.getRangeInches();
		
		//SmartDashboard.putNumber("ultrasonic value: ", range);
		
		//climbing system.
		if(controller.getAButton()){
			LEDswitch= !LEDswitch;
			Timer.delay(0.4);
			}
		else if(controller.getBumper(Hand.kLeft)){
			climbingMotor.set(1);
		}
		else if(controller.getBumper(Hand.kRight)){
				climbingMotor.set(0);}
		double on = LEDswitch? 1:0;
		LED.set(on);
		//myRobot.arcadeDrive(controller);
			/*SmartDashboard.putBoolean("getAbutton: ",controller.getAButton());
			SmartDashboard.putBoolean("getXbutton: ",controller.getXButton());
			SmartDashboard.putBoolean("getYbutton: ",controller.getYButton());
			SmartDashboard.putNumber("getPOV: ",controller.getPOV(0));
			SmartDashboard.putNumber("getPOVcount: ",controller.getPOVCount());
			SmartDashboard.putNumber("LeftTrigerValue: ", controller.getTriggerAxis(Hand.kLeft));
			SmartDashboard.putNumber("RightTrigerValue: ", controller.getTriggerAxis(Hand.kRight));
			SmartDashboard.putBoolean("getBbutton: ",controller.getBButton());
			SmartDashboard.putBoolean("getBackbutton: ",controller.getBackButton());
			SmartDashboard.putBoolean("getLeftBumper: ",controller.getBumper(Hand.kLeft));
			SmartDashboard.putBoolean("getRightBumper: ",controller.getBumper(Hand.kRight));
			SmartDashboard.putString("getName: ",controller.getName() );
			SmartDashboard.putNumber("getXAxis(L) ",Math.round(controller.getX(Hand.kLeft)*100.0)/100.0);
			SmartDashboard.putNumber("getYAxis(L) ",Math.round(controller.getY(Hand.kLeft)*100.0)/100.0);
			SmartDashboard.putNumber("getXAxis(R) ",Math.round(controller.getX(Hand.kRight)*100.0)/100.0);
			SmartDashboard.putNumber("getYAxis(R) ",Math.round(controller.getY(Hand.kRight)*100.0)/100.0);
	*/
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
