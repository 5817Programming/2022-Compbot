// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.InaccessibleObjectException;
import java.util.ResourceBundle.Control;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlMode.LedMode;
import edu.wpi.first.cameraserver.CameraServer;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public Controller controller = Controller.getInstance();
  public Drive drive = Drive.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public LimeLight limelight = LimeLight.getInstance();
  public Elevator elevator = Elevator.getInstance();
  public Intake intake = Intake.getInstance();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");

 

  double target = tv.getDouble(0.0);

  double controllerJoystickDeadzone = 0.2;
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  Solenoid intakeSolonoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  Solenoid elevatorSolonoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2); //change ports
  Solenoid elevatorsupportSolonoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0); //change ports
  

  enum DriveState
  {
    Turbo,
    Grandma
  };
  DriveState driveState = DriveState.Turbo;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    shooter.zeroSensor();


    new Thread(() ->  {
      edu.wpi.first.cscore.UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(400, 200);

      edu.wpi.first.cscore.CvSink cvSink = CameraServer.getVideo();
      edu.wpi.first.cscore.CvSource outputStream = CameraServer.putVideo("Blur", 400, 200);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if(cvSink.grabFrame(source) == 0) {
          continue;
        }

        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }


    }).start();
    
    lastTimeStamp = Timer.getFPGATimestamp();

    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5800, "limelight.local", 5800);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  double Angle;
  double a1 = 30.19;
  double heightDiff = 73;
  double distance;
  double shooterPower;
  double desiredRPM;
  double percent;
  double DistanceinMetter;
  double shooterpewpew;
  double shooterpewpewinRPM;

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Right Encoder", drive.RightEncoder() * drive.kTick2feet );
    SmartDashboard.putNumber("Left Encoder", drive.LeftEncoder() * drive.kTick2feet );

    SmartDashboard.putNumber("LEleEncoder", elevator.LeftEncoderElevator());
    SmartDashboard.putNumber("REleEncoder", elevator.RightEncoderElevator());
    SmartDashboard.putNumber("Elev.Speed", elevatorSetSpeed);

    SmartDashboard.putNumber("encoder", shooter.angle());
    SmartDashboard.putNumber("Tx", limelight.getdegRotationToTarget());

    double a2 = ((limelight.getdegVerticalToTarget()));
    double totalAngle = a1 + a2;
    distance = (heightDiff/Math.tan(Math.toRadians((totalAngle))));
    double distanceAlso = Math.toRadians(distance);

    //We get distance the we turn it into meters since in physics only uses meters 
    DistanceinMetter = distance*0.0254;
    
    //this is the equation to get the desired veolcity of the ball that we need it to be shot by 
    shooterpewpew = .3;
    // ((DistanceinMetter/((Math.cos((72.69*3.141592653589)/180)))) * (Math.sqrt(9.81/(2*(DistanceinMetter) *((Math.tan((72.69*3.141592653589)/180)))- 5.48 +1.52 )) ));
// 
    //new first angle 0.297541506743
    //new second angle 3.20865782292

    //first angle 0.
    //second angle 2.85395483798
    //this converts velocity into the RPM that we need to set the motors to 
    shooterpewpewinRPM = shooterpewpew*3.28084/0.012175438596;

    double tickPerRevolution = 2048;
    double TicktoDegree = (tickPerRevolution/14)*155;
    SmartDashboard.putNumber("ShooterPos.", Angle); 
    Angle = Math.round(Math.toDegrees(shooter.angle()/TicktoDegree));

    double cosine=((Math.cos((70*3.141592653589)/180)));

    SmartDashboard.putNumber("cosine", cosine);
    SmartDashboard.putNumber("disinmeeter", DistanceinMetter);
    SmartDashboard.putNumber("pewpewinPM", shooterpewpewinRPM);
    SmartDashboard.putNumber("limelight_Y-valur", limelight.getdegVerticalToTarget());
    SmartDashboard.putNumber("encoder", shooter.angle());
    SmartDashboard.putNumber("Tx", limelight.getdegRotationToTarget());
    SmartDashboard.putBoolean("Tv", limelight.getIsTargetFound());
    SmartDashboard.putNumber("MAbyeDIstance", distanceAlso);
    SmartDashboard.putNumber("DistToTarget", distance);
    SmartDashboard.putNumber("Angle", Math.tan(totalAngle));

    SmartDashboard.putNumber("Shoot Speed", shooterPower); //May work
    SmartDashboard.putBoolean("LeftLeveled?", elevator.leftLeveled);
    SmartDashboard.putBoolean("RightLeveled?", elevator.rightLeveled);

    SmartDashboard.putNumber("LeftELev.Current", elevator.leftSideClimb.getStatorCurrent());
    SmartDashboard.putNumber("RightELev.Current", elevator.rightSideClimb.getStatorCurrent());
  //  SmartDashboard.putNumber("RoboYaw", drive.getYaw());
  
/*
    if(limelight.getIsTargetFound() == false){  //If limelight doesn't detect the target set shooter motor 25%.
    shooterPower = 0.22;
  } else if (distance >= 80 && distance<=98){
    shooterPower = 0.3569;
  }else if (distance >= 99 && distance<=107){
    shooterPower = 0.28;
  }else if (distance >=108 && distance<=115){
    shooterPower = 0.36;
  }else if (distance >=116 && distance<=120){
    shooterPower = 0.39;
  }else if (distance >=121 && distance <= 130){
    shooterPower = 0.42;
  }else if (distance >=131 && distance <= 138){
    shooterPower = 0.53;
  }else if (distance>=139 && distance <= 143){
    shooterPower = 0.58;
  }
*/

if(distance >= 80 && distance <=162){  //If limelight doesn't detect the target set shooter motor 20%.
shooterpewpew = ((DistanceinMetter/((Math.cos((80*3.141592653589)/180)))) * (Math.sqrt(9.81/(2*(DistanceinMetter) *((Math.tan((72.69*3.141592653589)/180)))- 5.48 +1.52 )) ));

//1276
} //subtracted 200 below this
else if(distance > 162){ //If target is between 80in and 95in, set shooter motor 32.3%!
  shooterpewpew = ((DistanceinMetter/((Math.cos((83*3.141592653589)/180)))) * (Math.sqrt(9.81/(2*(DistanceinMetter) *((Math.tan((83*3.141592653589)/180)))- 5.48 +1.52 )) ));

}

else if(distance >= 96 && distance <=105){ //If target is between 96in and 105in, set shooter motor 32.5%!
shooterPower = 0.325;
desiredRPM = 2150.5 - 5;//will change  actually 2200.5 + 35
}else if(distance >= 106 && distance <= 114){ //If target is between 106in and 114in, set shooter motor 34%!
shooterPower = 0.34; //was 106 to 122
desiredRPM = 2169.2 - 45;//maybe change   actually 2219.2
//up here original
} else if(distance >= 115 && distance <= 132){ //If target is between 115in and 132in, set shooter motor 35%!
shooterPower = 0.35; //was 123 to 132
desiredRPM = 2183 - 69;//last 2033, 200 less than original
}else if(distance >= 133 && distance <= 169){ //If target is between 133in and 169in, set shooter motor 37%.
shooterPower = 0.39; //was 0.37
desiredRPM = 2238.20 + 176; //was 2360.60   actualy 2388.2
} else if(distance >= 170 && distance <= 190){ //If target is between 170in and 190in, set shooter motor 43%.
shooterPower = 0.43; 
desiredRPM = 2389.40 + 160; //actually 2429.4
} else if (distance >=191 && distance <=200){// this is right next to safety point(191 in)
  desiredRPM = 2467 + 100; //actually 2565
}else if (distance >= 201 && distance <= 220){
  //desiredRPM = 2850;
  desiredRPM = 2463.98 + 50;
}else if (distance >= 221 && distance <= 241){
  desiredRPM = 3000;
}else if ( distance >= 242 && distance <= 260){
  desiredRPM = 3300;
}else if (distance >= 261){
  desiredRPM = 3690;
}
//down this point is teoridical code
/* }else if(distance > 190 && distance <= 210){  //If target is further than 190in, set shooter motor 53%.
shooterPower = 0.45;
desiredRPM = 2871;

//shooterPower = 0.53;
//desiredRPM = 3600; //was 3381.4, works high once at 3481.4, was 3581.4 at a distance of 255
}else if (distance > 210 && distance <=240){
shooterPower = 0.48;
desiredRPM = 3062.40;
}else if (distance >240 && distance <= 260){
desiredRPM = 3600;
}else if (distance > 260 && distance <= 280){
  shooterPower = 0.58;
  desiredRPM = 3700.4;
} */


//RPM to percent 
percent =.3 ;

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

  double startime;

  @Override
  public void autonomousInit() {
   /* m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: "+ m_autoSelected);

    startime = Timer.getFPGATimestamp();
*/

  drive.zeroSensor();
  errorSum = 0;
  lastTimeStamp = Timer.getFPGATimestamp();
  startime = Timer.getFPGATimestamp();

  limelight.setLEDMode(LedMode.kforceOn);

  }

  
  double lastTimeStamp;
  double dt;


  double kp = -0.02; //was 0.0355
  double ki = -0.05;//maybe negitive? was -.175
  double kD = -0.1;

  final double iLimit = 3;
  double error;
  double errorRate;
  double errorSum = 0;


  double min_command = 0.01;
  double speed = 0.1;


  double heading_error;
  double steering_adjust;

  final double driveiLimit = 1;

  final double drivekP = 0.06;
  final double drivekI = 0.06;

  double drivesetpoint = 0;
  double driveerrorSum = 0;
  double drivelastTimeStamp = 0;

  final double rpmToTicksPer100ms = 2048 / 600 / (69 / 69);
  //rpmToCountsPer100ms = ticksPerRotation / by 600 to get 100 ms / Gear Ratio

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    //limelight.setLEDMode(LedMode.kforceOn);
    double difference = Timer.getFPGATimestamp() - startime;
  
 
    limelight.setLEDMode(LedMode.kforceOn);
    heading_error = (limelight.getdegRotationToTarget() * -1);

   
    //two ball high with new robot
    /*if(difference < 0.5){
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if (difference < 0.8){
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 1.9){
      limelight.setLEDMode(LedMode.kforceOn);
      intakeSolonoid.set(true);
      Timer.delay(0.69);
      intake.BallsGrabber(1);
      intake.towerIntake(.69);
      intake.feeder(0.69);
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if(difference < 2){
      intake.BallsGrabber(0);
      intake.towerIntake(0);
      intake.feeder(0);
      shooter.ShootMotor(0.3);
      shooter.ShootMotor2(0.3);
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 3){
      drive.LeftDrive(0);
      drive.RightDrive(0);
      intake.BallsGrabber(0);
      intake.feeder(0);
      
          
      if(limelight.getIsTargetFound() == true) {

        shooter.Shooter_angle(steering_adjust);  
    
        if(limelight.getdegRotationToTarget() > 0.0){ 
          if(Angle > 79){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error - min_command) - (ki * errorSum) - (kD * errorRate); //+ (ki * errorSum); //KP times X value minus costant
          }
        }else if(limelight.getdegRotationToTarget() < 0.0){
          if(Angle < -69){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error + min_command) + (ki * errorSum) + (kD * errorRate); 
          }
        }
      }else if(limelight.getIsTargetFound() == false){
        shooter.Shooter_angle(speed);
        if(Angle > 69){ //Too far to the right      angle limits were 50
          speed = -0.25;
        }else if (Angle <-50) { //Too far to the left   angle limits were -50
          speed = 0.25;
        }
      }else{
        shooter.Shooter_angle(0);
      }
    }else if (difference > 3 && difference < 15){
      shooter.ShootMotor(percent);
      shooter.ShootMotor2(percent);
      Timer.delay(0.5);
      intake.towerIntake(1);
      intake.feeder(-0.8);
    }else{                                           //turn off if not in time thing
      drive.LeftDrive(0);
      drive.RightDrive(0);
      shooter.ShootMotor(0);
      shooter.ShootMotor2(0);
      intake.towerIntake(0);
      intake.feeder(0);                       
    }*/






    limelight.setLEDMode(LedMode.kforceOn);
    heading_error = (limelight.getdegRotationToTarget() * -1);

    //three ball high with new robot
    /*if(difference < 0.5){  //For half a second, drive forward to drop kickbar
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if (difference < 0.8){
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 1.9){ //for 1.2 seconds, drive up and take an alliance ball
      intakeSolonoid.set(true);
     // Timer.delay(0.69);
      intake.BallsGrabber(1);
      intake.towerIntake(.70);
      intake.feeder(0.69);
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if(difference < 2){ //Stop drive and ramp up shooters
      intake.BallsGrabber(0);
      intake.towerIntake(0);
      intake.feeder(0);
      shooter.ShootMotor(0.3);
      shooter.ShootMotor2(0.3);
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 3){ //For a second, aim for the target
      drive.LeftDrive(0);
      drive.RightDrive(0);
      intake.BallsGrabber(0);
      intake.feeder(0);
      
      if(limelight.getIsTargetFound() == true) {

        shooter.Shooter_angle(steering_adjust);  
    
        if(limelight.getdegRotationToTarget() > 0.0){ 
          if(Angle > 79){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error - min_command) - (ki * errorSum) - (kD * errorRate); //+ (ki * errorSum); //KP times X value minus costant
          }
        }else if(limelight.getdegRotationToTarget() < 0.0){
          if(Angle < -69){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error + min_command) + (ki * errorSum) + (kD * errorRate); 
          }
        }
      }else if(limelight.getIsTargetFound() == false){
        shooter.Shooter_angle(speed);
        if(Angle > 69){ //Too far to the right      angle limits were 50
          speed = -0.25;
        }else if (Angle <-50){ //Too far to the left   angle limits were -50
          speed = 0.25;
        }
      }else{
        shooter.Shooter_angle(0);
            
      }
    }else if(difference > 4 && difference < 6){ //For two seconds, begin to shoot the two big balls
      shooter.Shooter_angle(0);
      shooter.ShootMotor(percent);
      shooter.ShootMotor2(percent);
      Timer.delay(0.5);
      intake.towerIntake(1);
      intake.feeder(-0.8);
//changes made here


    }else if(difference > 6 && difference < 7.5){ //was 7.8
      shooter.ShootMotor(0);
      shooter.ShootMotor2(0);
      intake.towerIntake(0);
      intake.feeder(0);

      if(drive.getYaw() < 50){   //actual angle 56.5    also could be wrong cuz i got kinda confusion
        drive.LeftDrive(-0.16);
        drive.RightDrive(0.16);
      }else{
        drive.LeftDrive(0);
        drive.RightDrive(0);
      }
      
    }else if(difference > 7.5 && difference < 10.3){
      intake.BallsGrabber(1);
      intake.towerIntake(.69);
      intake.feeder(0.69);
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    
    }else if(difference > 10.3 && difference < 14){
      drive.LeftDrive(0);
      drive.RightDrive(0);
      intake.BallsGrabber(0);
      intake.feeder(0);
      intake.BallsGrabber(0);
    
      if(limelight.getIsTargetFound() == true){

        shooter.Shooter_angle(steering_adjust);  
    
        if(limelight.getdegRotationToTarget() > 0.0){ 
          if(Angle > 79){
            steering_adjust= 0;
          }else{
            steering_adjust = (kp*heading_error - min_command) - (ki * errorSum) - (kD * errorRate); //+ (ki * errorSum); //KP times X value minus costant
          }
        }else if(limelight.getdegRotationToTarget() < 0.0){
          if(Angle < -69){
            steering_adjust= 0;
          }else{
            steering_adjust = (kp*heading_error + min_command) + (ki * errorSum) + (kD * errorRate); 
          }
        }
      }else if(limelight.getIsTargetFound() == false){
        shooter.Shooter_angle(speed);
        if(Angle > 79){ //Too far to the right      angle limits were 50
          speed = -0.25;
        }else if (Angle <-69) { //Too far to the left   angle limits were -50
          speed = 0.25;
        }
      }else{
        shooter.Shooter_angle(0);   
      }
    }else if(difference > 14 && difference < 15){
      shooter.Shooter_angle(0);
        shooter.ShootMotor(percent);
        shooter.ShootMotor2(percent);
        Timer.delay(0.5);
        intake.towerIntake(1);
        intake.feeder(-0.8);
    }else{                                           //turn off if not in time thing
      drive.LeftDrive(0);
      drive.RightDrive(0);
      shooter.ShootMotor(0);
      shooter.ShootMotor2(0);
      intake.towerIntake(0);
      intake.feeder(0);                       
    }

*/




    //two ball high with new robot with defense(taking the enemy ball and throwing it into the hanger)
    if(difference < 0.5){  //For half a second, drive forward to drop kickbar
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if (difference < 0.8){
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 1.9){ //for 1.2 seconds, drive up and take an alliance ball
      intakeSolonoid.set(true);
     // Timer.delay(0.69);
      intake.BallsGrabber(1);
      intake.towerIntake(.69);
      intake.feeder(0.69);
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if(difference < 2){ //Stop drive and ramp up shooters
      intake.BallsGrabber(0);
      intake.towerIntake(0);
      intake.feeder(0);
      shooter.ShootMotor(0.25);
      shooter.ShootMotor2(0.25);
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 3){ //For a second, aim for the target
      drive.LeftDrive(0);
      drive.RightDrive(0);
      intake.BallsGrabber(0);
      intake.feeder(0);
      
      if(limelight.getIsTargetFound() == true){

        shooter.Shooter_angle(steering_adjust);  
    
        if(limelight.getdegRotationToTarget() > 0.0){ 
          if(Angle > 79){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error - min_command) - (ki * errorSum) - (kD * errorRate); //+ (ki * errorSum); //KP times X value minus costant
          }
        }else if(limelight.getdegRotationToTarget() < 0.0){
          if(Angle < -69){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error + min_command) + (ki * errorSum) + (kD * errorRate); 
          }
        }
      }else if(limelight.getIsTargetFound() == false){
        shooter.Shooter_angle(speed);
        if(Angle > 69){ //Too far to the right      angle limits were 50
          speed = -0.25;
        }else if (Angle <-50) { //Too far to the left   angle limits were -50
          speed = 0.25;
        }
      }else {
        shooter.Shooter_angle(0);
      }
    }else if(difference > 4 && difference < 6){ //For two seconds, begin to shoot the two big balls
      shooter.Shooter_angle(0);
      shooter.ShootMotor(percent + 0.05);
      shooter.ShootMotor2(percent + 0.05);
      Timer.delay(0.5);
      intake.towerIntake(1);
      intake.feeder(-0.8);
//changes made here
    }else if(difference > 6 && difference < 7.4){ //For 1.8 seconds, turn towards the enemy ball
      shooter.ShootMotor(0);
      shooter.ShootMotor2(0);
      intake.towerIntake(0);
      intake.feeder(0);

        if(drive.getYaw() > -50){   //actual angle 56.5    also could be wrong cuz i got kinda confusion
          drive.LeftDrive(0.15);
          drive.RightDrive(-0.15);
        }else{
          drive.LeftDrive(0);
          drive.RightDrive(0);
        }

    }else if(difference > 7.5 && difference < 8.5){ //For 1seconds, go forward and grab enemy ball
      intake.BallsGrabber(1);
      intake.towerIntake(.69);
      intake.feeder(0.69);
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if(difference > 8.5 && difference < 9.3){ //For 0.5 seconds, turn around towards hanger
      intake.towerIntake(0);
      intake.feeder(0);

      if(drive.getYaw() <= 0){   //actual angle 56.5    also could be wrong cuz i got kinda confusion
        drive.LeftDrive(0.15);
        drive.RightDrive(-0.15);
      }else{
        drive.LeftDrive(0);
        drive.RightDrive(0);
      }
      //changes end here
    }else if(difference > 9.69 && difference < 15){ //For one second, outtake the enemy ball
      shooter.ShootMotor(0.2);
      shooter.ShootMotor2(0.2);
      intake.towerIntake(.69);
      intake.feeder(-0.69);

    }else{                                           //turn off if not in time thing
      drive.LeftDrive(0);
      drive.RightDrive(0);
      shooter.ShootMotor(0);
      shooter.ShootMotor2(0);
      intake.towerIntake(0);
      intake.feeder(0);
      intake.BallsGrabber(0);                       
    } 
 

 //4 Ball Auto: Straight Line, At Terminal Station

    /*if(difference < 0.5){
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if(difference < 0.8){
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 1.9){
      intakeSolonoid.set(true);
      Timer.delay(0.69);
      intake.BallsGrabber(1);
      intake.towerIntake(.69);
      intake.feeder(0.69);
      drive.LeftDrive(.4);
      drive.RightDrive(.4);
    }else if(difference < 2){
      intake.BallsGrabber(0);
      intake.towerIntake(0);
      intake.feeder(0);
      shooter.ShootMotor(0.4);
      shooter.ShootMotor2(0.4);
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference < 3){
      drive.LeftDrive(0);
      drive.RightDrive(0);
      intake.BallsGrabber(0);
      intake.feeder(0);
      
          
      if(limelight.getIsTargetFound() == true) {

        shooter.Shooter_angle(steering_adjust);  
        
        if(limelight.getdegRotationToTarget() > 1.0){ 
          if(Angle > 50){
            steering_adjust= 0;
          }else{
            steering_adjust = (kp*heading_error - min_command); //KP times X value minus costant
          }
        }else if(limelight.getdegRotationToTarget() < 1.0){
          if(Angle <-80){
            steering_adjust= 0;
          }else{
            steering_adjust = kp*heading_error + min_command; 
          }
        }
      }else{
        shooter.Shooter_angle(0);                
      }
    }else if(difference > 4 && difference < 5){
      shooter.Shooter_angle(0);
      shooter.ShootMotor(percent);
      shooter.ShootMotor2(percent);
      Timer.delay(0.5);
      intake.towerIntake(1);
      intake.feeder(-0.8);


    }else if(difference > 5 && difference < 5.3) {
      shooter.ShootMotor(0);
      shooter.ShootMotor2(0);
      intake.towerIntake(0);
      intake.feeder(0);

      if(drive.getYaw() < 15){   //actual angle 56.5    also could be wrong cuz i got kinda confusion
        drive.LeftDrive(0.15);
        drive.RightDrive(-0.15);
      }else{
        drive.LeftDrive(0);
        drive.RightDrive(0);
      } 
    }else if(difference > 5.3 && difference < 8){
      intake.BallsGrabber(1);
      intake.towerIntake(.69);
      intake.feeder(0.69);
      drive.LeftDrive(.5);
      drive.RightDrive(.5);
    }else if(difference > 8 && difference < 8.5){
      intake.BallsGrabber(1);
      intake.towerIntake(.69);
      intake.feeder(0.69);
      drive.LeftDrive(.2);
      drive.RightDrive(.2);
    }else if(difference > 8.5 && difference < 10){
      intake.BallsGrabber(-1);
      intake.towerIntake(-.69);
      intake.feeder(-0.69);
      drive.LeftDrive(0);
      drive.RightDrive(0);
    }else if(difference > 10 && difference < 11){
      intake.BallsGrabber(0);
      intake.feeder(0);
      intake.BallsGrabber(0);
      drive.LeftDrive(-0.6);
      drive.RightDrive(-0.6);
    }else if(difference > 11 && difference < 11.8){
      drive.LeftDrive(0);
      drive.RightDrive(0);
            
      if(limelight.getIsTargetFound()==true){

        shooter.Shooter_angle(steering_adjust);  
      
        if(limelight.getdegRotationToTarget() > 0.0){ 
          if(Angle > 79){
            steering_adjust= 0;
          }else{
            steering_adjust = (kp*heading_error - min_command) - (ki * errorSum) - (kD * errorRate); //+ (ki * errorSum); //KP times X value minus costant
          }
        }else if (limelight.getdegRotationToTarget() < 0.0){
          if(Angle < -69){
            steering_adjust= 0;
          }else{
            steering_adjust = (kp*heading_error + min_command) + (ki * errorSum) + (kD * errorRate); 
          }
        }
      }else if(limelight.getIsTargetFound() == false){
        shooter.Shooter_angle(speed);
        if(Angle > 79){ //Too far to the right      angle limits were 50
          speed = -0.25;
        }else if(Angle <-69){ //Too far to the left   angle limits were -50
          speed = 0.25;
        }
      }else{
        shooter.Shooter_angle(0);        
      }  
    }else if (difference > 11.8 && difference < 15){
      shooter.Shooter_angle(0);
      shooter.ShootMotor(percent);
      shooter.ShootMotor2(percent);
      Timer.delay(0.5);
      intake.towerIntake(1);
      intake.feeder(-0.8);
        
    }else{
      shooter.Shooter_angle(0);
      shooter.ShootMotor(0);
      shooter.ShootMotor2(0);
      intake.towerIntake(0);
      intake.feeder(0);
    } */



    lastTimeStamp = Timer.getFPGATimestamp();
    LastError = error; 

  }




  int counter = 0;
  int elvCounter = 0;
  double elvsupportCounter = 0;
  double LastError; 
  double elevatorSetSpeed;


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    elevator.zeroSensor();
    //shooter.zeroSensor();
    errorSum = 0;
    LastError = 0; 
    lastTimeStamp = Timer.getFPGATimestamp(); 

    driveState = DriveState.Turbo;
    elevatorSetSpeed = 0;
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    //if limelight sees target within interval, CoDrive controll rumbles to tell when ready to shoot
    if(controller.getRightTriggerCoDriver() > 0.69 && limelight.getIsTargetFound() && limelight.getdegRotationToTarget() > -3 && limelight.getdegRotationToTarget() < 3){
      controller.getcoRumble(1);
    } else {
      controller.getcoRumble(0);
    }
  

    

    switch(driveState){

      case Turbo:
        if(controller.getYLeftDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getYLeftDriver()*0.75 - (controller.getXRightDriver())*0.45);
          drive.LeftDrive(controller.getYLeftDriver()*0.75 + (controller.getXRightDriver())*0.45);
        }else if(controller.getYLeftDriver() < controllerJoystickDeadzone * (-1)){
          drive.RightDrive(controller.getYLeftDriver()*0.75 - (controller.getXRightDriver())*0.45);
          drive.LeftDrive(controller.getYLeftDriver()*0.75 + (controller.getXRightDriver())*0.45);
        }else if(controller.getXRightDriver() < controllerJoystickDeadzone *(-1) || controller.getXRightDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getXRightDriver()*(-1));
          drive.LeftDrive(controller.getXRightDriver());
        }else{
          drive.LeftDrive(0); 
          drive.RightDrive(0);
        }
        break;

      case Grandma:
        if(controller.getYLeftDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getYLeftDriver()*0.25 - (controller.getXRightDriver())*0.125);
          drive.LeftDrive(controller.getYLeftDriver()*0.25 + (controller.getXRightDriver())*0.125);
        }else if(controller.getYLeftDriver() < controllerJoystickDeadzone * (-1) ){
          drive.RightDrive(controller.getYLeftDriver()*0.25 - (controller.getXRightDriver())*0.125);
          drive.LeftDrive(controller.getYLeftDriver()*0.25 + (controller.getXRightDriver())*0.125);
        }else if(controller.getXRightDriver() < controllerJoystickDeadzone *(-1) || controller.getXRightDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getXRightDriver()*(-1) * 0.25);
          drive.LeftDrive(controller.getXRightDriver() * 0.25);

        }else{
          drive.LeftDrive(0);
          drive.RightDrive(0);
        }
        break;
    }

    if(controller.getDpadDriver() == 0){
      driveState = DriveState.Turbo;
    }else if(controller.getDpadDriver() == 180){
      driveState = DriveState.Grandma;
    }


    if(controller.getRightTriggerDriver() > 0.2){
      intake.feeder(0.5);
      intake.BallsGrabber(1);
      intake.towerIntake(1);
    } else if(controller.getLeftTriggerDriver() > 0.2){
      intake.BallsGrabber(-1);
      intake.towerIntake(-1);
    } else if(controller.getLeftTriggerCoDriver() > 0.2){
      intake.feeder(-0.8);
      intake.towerIntake(1);
      intake.BallsGrabber(0);
    } else {
      intake.feeder(0);
      intake.BallsGrabber(0);
      intake.towerIntake(0);
    }

    if(controller.getRightBumperDriver()){
      intakeSolonoid.set(true);
    } else if(controller.getLeftBumperDriver()){
      intakeSolonoid.set(false);
    }

    
/*
    if(controller.getButtonYCoDriver() && elevator.LeftEncoderElevator() < 338000){ 
      elevator.elevatorMove(0.5); //was 1, slow is .20

    } else if(controller.getButtonBCoDriver()){
      elevator.elevatorMove(-.69); //was -.69, slow is -.29
      

    } else if(controller.getButtonACoDriver() && elevator.LeftEncoderElevator() > 69000){
      elevator.elevatorMove(-.69); //was funny, slow is unfunny -.29

    } else {
      elevator.elevatorMove(0);
    } */
    









    

  if(controller.getButtonYCoDriver()){ //Button Y
    elevatorSetSpeed = 1;
  } else if(controller.getButtonACoDriver()){ //M 
    elevatorSetSpeed = -0.7;
  } else {
    elevatorSetSpeed = 0;
  }

  if(elevator.LeftEncoderElevator() > 300000 && elevatorSetSpeed > 0.1){ //&& Math.abs(Math.abs(elevatorSetSpeed) - 0.8) < 0.01){
    elevatorSetSpeed = 0;
  } else if(elevator.LeftEncoderElevator() < 30000 && Math.abs(Math.abs(elevatorSetSpeed) - 0.7) < 0.01){
    elevatorSetSpeed = 0;
  }

  if(controller.getButtonXCoDriver()){ //C
    elevatorSetSpeed = 0.2;
  // elevator.zeroSensor();
  }else if(controller.getButtonBCoDriver()){ //A
    elevatorSetSpeed = -0.2;
  }

elevator.elevatorMove(elevatorSetSpeed);

if(controller.getBackButtonCoDriver()){
  elevator.isLeftLeveled(6.9);
  elevator.isRightLeveled(10);
  if(elevator.leftLeveled == false){ //This checks if the Leftside is leveled, with abnormal amperage being over 6
    elevator.leftElevMove(-0.2);
  } else {
    elevator.leftElevMove(0);
  }
  if(elevator.rightLeveled == false){ //This checks if the rightside is leveled, with abnormal amperage being over 6
    elevator.rightElevMove(-0.2);
  } else {
    elevator.rightElevMove(0);
  }

  if(elevator.rightLeveled && elevator.leftLeveled) elevator.zeroSensor();

}





if (controller.getRightBumperCoDriver()&& elvCounter==0){
      elevatorSolonoid.set(true);
      elvCounter = 1;
      Timer.delay(.3); //nice
} else if(controller.getRightBumperCoDriver()&&elvCounter==1){
      elevatorSolonoid.set(false);
      elvCounter = 0;
      Timer.delay(.3); //nice
    }  


   
    /*if (controller.getLeftBumperCoDriver() && elvsupportCounter == 0){

      elevatorsupportSolonoid.set(true);
      elvsupportCounter = 1;
      Timer.delay(.3);

    }else if(controller.getLeftBumperCoDriver()&& elvsupportCounter == 1){
      elevatorsupportSolonoid.set(false);
      elvsupportCounter = 0;
      Timer.delay(.3);
    }*/





    heading_error = (limelight.getdegRotationToTarget() * -1);

   

      //PID CODE

    if(Math.abs(error) < iLimit){
      errorSum += error * dt; // for kI
    }

    //limelight.setLEDMode(LedMode.kforceOn);

    dt = Timer.getFPGATimestamp() - lastTimeStamp;
    errorRate = (error - LastError) / dt; // for kD

    if(controller.getRightTriggerCoDriver()>.8) {
      
      limelight.setLEDMode(LedMode.kforceOn);
      //shooter.ShootMotor2(shooterPower);
      //shooter.ShootMotor(shooterPower);

      shooter.ShootMotor2(percent);
      shooter.ShootMotor(percent );

      if(limelight.getIsTargetFound()==true) {

        shooter.Shooter_angle(steering_adjust);  
    
        if(limelight.getdegRotationToTarget() > 0.0){ 
          if(Angle > 79){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error - min_command) - (ki * errorSum) - (kD * errorRate); //+ (ki * errorSum); //KP times X value minus costant
          }
        }else if (limelight.getdegRotationToTarget() < 0.0){
          if(Angle < -69){
            steering_adjust= 0;
          }else {
            steering_adjust = (kp*heading_error + min_command) + (ki * errorSum) + (kD * errorRate); 
          }
        }
      } else if(controller.getRightTriggerCoDriver()>.8){
        shooter.Shooter_angle(0);
        if(Angle > 15){ //Too far to the right      angle limits were 50
          speed = -0.25;
        }else if (Angle <-15) { //Too far to the left   angle limits were -50
          speed = 0.25;
        }
      }else {
        shooter.Shooter_angle(0);
            
      }
    }else{
      limelight.setLEDMode(LedMode.kforceOn);
      shooter.Shooter_angle(0);
      shooter.ShootMotor2(0.15);
      shooter.ShootMotor(0.15);
    }

    if(controller.getLeftJoystickPressCoDriver()){
      if(Angle > 0){
        shooter.Shooter_angle(-.15); 
      } else if(Angle < 0){
        shooter.Shooter_angle(.15);
      } else {
        shooter.Shooter_angle(0);
      }
    }


    lastTimeStamp = Timer.getFPGATimestamp();
    LastError = error; 

    

   
 
  }


    




  



  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    switch(driveState){

      case Turbo:
        if(controller.getYLeftDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getYLeftDriver()*0.75 - (controller.getXRightDriver())*0.45);
          drive.LeftDrive(controller.getYLeftDriver()*0.75 + (controller.getXRightDriver())*0.45);
        }else if(controller.getYLeftDriver() < controllerJoystickDeadzone * (-1)){
          drive.RightDrive(controller.getYLeftDriver()*0.75 - (controller.getXRightDriver())*0.45);
          drive.LeftDrive(controller.getYLeftDriver()*0.75 + (controller.getXRightDriver())*0.45);
        }else if(controller.getXRightDriver() < controllerJoystickDeadzone *(-1) || controller.getXRightDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getXRightDriver()*(-1));
          drive.LeftDrive(controller.getXRightDriver());
        }else{
          drive.LeftDrive(0); 
          drive.RightDrive(0);
        }
        break;

      case Grandma:
        if(controller.getYLeftDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getYLeftDriver()*0.25 - (controller.getXRightDriver())*0.125);
          drive.LeftDrive(controller.getYLeftDriver()*0.25 + (controller.getXRightDriver())*0.125);
        }else if(controller.getYLeftDriver() < controllerJoystickDeadzone * (-1) ){
          drive.RightDrive(controller.getYLeftDriver()*0.25 - (controller.getXRightDriver())*0.125);
          drive.LeftDrive(controller.getYLeftDriver()*0.25 + (controller.getXRightDriver())*0.125);
        }else if(controller.getXRightDriver() < controllerJoystickDeadzone *(-1) || controller.getXRightDriver() > controllerJoystickDeadzone){
          drive.RightDrive(controller.getXRightDriver()*(-1) * 0.25);
          drive.LeftDrive(controller.getXRightDriver() * 0.25);

        }else{
          drive.LeftDrive(0);
          drive.RightDrive(0);
        }
        break;
    }

    if(controller.getDpadDriver() == 0){
      driveState = DriveState.Turbo;
    }else if(controller.getDpadDriver() == 180){
      driveState = DriveState.Grandma;
    }


    if(controller.getRightTriggerDriver() > 0.8){
      shooter.ShootMotor2(0.5);
      shooter.ShootMotor(0.5);
      // intake.feeder(0.5);
      // intake.BallsGrabber(1);
      // intake.towerIntake(1);
    } else if(controller.getLeftTriggerCoDriver() > 0.2){
      // intake.BallsGrabber(-1);
      // intake.towerIntake(-1);
    } else if(controller.getLeftTriggerDriver() > 0.2){
      intake.feeder(-0.8);
      intake.towerIntake(1);
      intake.BallsGrabber(0);
    } else {
      shooter.ShootMotor2(0);
      shooter.ShootMotor(0);
      intake.feeder(0);
      intake.BallsGrabber(0);
      intake.towerIntake(0);
      // intake.feeder(0);
      // intake.BallsGrabber(0);
      // intake.towerIntake(0);
    }

    if(controller.getRightBumperDriver()){
      intakeSolonoid.set(true);
      Timer.delay(0.3);
      intake.feeder(0.5);
      intake.BallsGrabber(1);
      intake.towerIntake(1);
    } else if(controller.getLeftBumperDriver()){
      intakeSolonoid.set(false);
      intake.BallsGrabber(-1);
      intake.towerIntake(-1);
    }

    if(controller.getRightTriggerCoDriver()>.8) {
      // shooter.ShootMotor2(0.5);
      // shooter.ShootMotor(0.5);
    } else {
      // shooter.ShootMotor2(0);
      // shooter.ShootMotor(0);
      // intake.feeder(0);
      // intake.BallsGrabber(0);
      // intake.towerIntake(0);
    }



  }
}



























































//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69 
//69 69 69 69 69 69 69 69 69 69 69

