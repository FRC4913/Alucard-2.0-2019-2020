package org.usfirst.frc.team4913.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	TalonSRX frontLeftMotor,frontRightMotor,rearLeftMotor,rearRightMotor;
	Servo actuator;
	
	private static final int FRONT_LEFT = 1;
	private static final int REAR_LEFT = 2;
	private static final int FRONT_RIGHT = 4;
	private static final int REAR_RIGHT = 3;
	boolean LEDswitch =true;
	RobotDrive myRobot;
	
	GripPipeline a = new GripPipeline();
	double return0;
	CvSource output;
	ShootingSystem shooting;
	private final Object imgLock = new Object();
	private double centerX1,centerX2,centerX,area,areaInitial;
	private boolean spin;
	private int autoMode;
	private boolean autoModeEnabled;
	private double centerX1Final= 200;
	private double centerX2Final = 100;
	private double areaFinal = 1000;
	private int rotateTime;
	VisionThread visionThread;
	UsbCamera camera;
	private double autoLoopCounter=0;
	Ultrasonic frontSensor;
	Ultrasonic backSensor;
	 final int Mode0 = 0;
	 final int Mode1 = 1;
	 final int Mode2 = 2;
	 final int Mode3 = 3;
	   SendableChooser chooser;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		controller= new XboxController(0);
		climbingMotor=new Spark(1);
		LED=new Spark(0);
		shooting = new ShootingSystem(2,3);		
		camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(320, 240);
	    actuator = new Servo(4);
	    frontSensor = new Ultrasonic(0,1);
	    backSensor = new Ultrasonic(2,3);
	    frontSensor.setEnabled(false);
	    backSensor.setEnabled(false);
	    chooser = new SendableChooser();
        chooser.addDefault("Left Field", Mode0);
        chooser.addObject("Middle Field", Mode1);
        chooser.addObject("Right Field", Mode2);
        chooser.addObject("Left Field (Shooting)", Mode3);
        SmartDashboard.putData("Auto choices", chooser);
	    visionThread = new VisionThread(camera,new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()&&pipeline.filterContoursOutput().size()== 2){
	            spin = false;
	        	Rect r0 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
	            synchronized (imgLock) {
	                centerX1 = r0.x + (r0.width / 2);
	                centerX2 = r1.x + (r1.width/2);
	                centerX = (centerX1+centerX2)/2;
	                area = r0.height*r0.width+r1.height*r1.width;
	            }
	            
	        }
	        else{
            	spin = true;
	        }
	    }); 
	 visionThread.start();
	 frontLeftMotor = new TalonSRX(FRONT_LEFT);
	 frontRightMotor = new TalonSRX(FRONT_RIGHT);
	 rearLeftMotor = new TalonSRX(REAR_LEFT);
	 rearRightMotor = new TalonSRX(REAR_RIGHT);
	 myRobot = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
		autoMode = (int)chooser.getSelected();
		areaInitial = area;
		rotateTime=0;
		return0 =0;
		LED.set(1);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		frontSensor.setEnabled(true);
		
		if (!autoModeEnabled){
			return;
		}
		double centerX,area;
		synchronized (imgLock) {
			centerX = this.centerX;
			area = this.area;
		}
		double rotateSpeed = Math.abs(centerX1Final-centerX)>80? Math.abs(centerX1Final-centerX)*0.005:0.4;
		double rotate = spin? 0.7: rotateSpeed;
		
		
		/*
		 * autoMode == 0 robot in the left of the field
		 * autoMode == 1 robot in the middle of the field
		 * autoMode == 2 robot in the right of the field
		 * autoMode == 3 robot in the left of the field and shoots the fuel
		 */
		if(autoMode == 0){
			if (timer.get()<=1.5){
				myRobot.drive(1.0, 0.0);
			}
			else if(timer.get()>1.5&&spin){
				myRobot.drive(0.0, rotate);
				rotateTime++;
			}
			else if(!spin&&Math.abs(centerX1Final-centerX)>50){
				myRobot.drive(0.0, rotate);
				rotateTime++;
			}
			else if(frontSensor.getRangeMM()>20){
				myRobot.drive(0.8,0.0);
			}
			else{
				actuator.set(0.5);
				Timer.delay(2);
			}
			
		}
		else if(autoMode == 1){
			if(frontSensor.getRangeMM()>20){
				myRobot.drive(0.8,0.0);
			}
			else{
				actuator.set(0.5);
				Timer.delay(2);
			}
		}
		else if (autoMode == 2){
			if(timer.get()<1.5){
				myRobot.drive(1.0, 0.0);
			}
			else if (timer.get()>1.5&&spin){
				myRobot.drive(0.0, -rotate);
				rotateTime++;
			}
			else if(!spin&&Math.abs(centerX1Final-centerX)>50){
				myRobot.drive(0.0, -rotate);
				rotateTime++;
				return0 = frontSensor.getRangeMM();
			}
			else if(frontSensor.getRangeMM()>20){
				myRobot.drive(0.8,0.0);
			}
			else{
				actuator.set(0.5);
				Timer.delay(2);
			}
		}
		else if (autoMode == 3){
			if(frontSensor.getRangeMM()<1000){
				myRobot.drive(1.0,0.0);
			}
			else if (timer.get()<5){
				myRobot.drive(0.0,0.6);
			}
			else{
				shooting.start();
			}
		}
		
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		LED.set(0);
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		if(controller.getBButton()){
			climbingMotor.set(1);
		}
		else if(controller.getXButton()){
			shooting.start();
		}
		else if(controller.getAButton()){
			actuator.set(0.1);
			Timer.delay(1);
		}
		else if(controller.getYButton()){
			actuator.set(1);
			Timer.delay(1);
		}
		else if (controller.getTrigger(Hand.kLeft)){
			frontSensor.setEnabled(!isEnabled());
		}
		else if (controller.getTrigger(Hand.kRight)){
			backSensor.setEnabled(!isEnabled());
		}
		else{
			climbingMotor.set(0);
			shooting.stop();
		}
		String frontSensorSignal = frontSensor.getRangeMM()<30?"stop":"Keep going";
		String backSensorSignal = backSensor.getRangeMM()<30?"stop":"Keep going";
		SmartDashboard.putString("frontSensor",frontSensorSignal );
		SmartDashboard.putString("backSensor",backSensorSignal);
		myRobot.arcadeDrive(controller);
			
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
}
