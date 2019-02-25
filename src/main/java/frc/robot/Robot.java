// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystem.*;
//import frc.robot.subsystem.Drive;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import edu.wpi.first.wpilibj.Joystick;
import java.util.concurrent.*;
import frc.utility.ThreadScheduler;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  Drive drive = Drive.getInstance();
  CollisionManager collisionManager = CollisionManager.getInstance();
  public static Joystick j = new Joystick(0);
  Turret turret = Turret.getInstance();
  Elevator elevator = Elevator.getInstance();

  ExecutorService executor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();

  

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drive.calibrateGyro();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    scheduler.schedule(drive, executor);
		scheduler.schedule(elevator, executor);
		scheduler.schedule(turret, executor);
		scheduler.schedule(collisionManager, executor);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    //AutoRoutine ar = new AutoRoutine();
    //ar.addComands(new DriveToPooints)
    Path drivePath = new Path(RobotTracker.getInstance().getOdometry().translationMat);
    drivePath.addPoint(new Translation2D(10, 0), 40);
    drivePath.addPoint(new Translation2D(20, 0), 40);
    drivePath.addPoint(new Translation2D(30, 0), 40);
    drivePath.addPoint(new Translation2D(30, -15), 40);


    //drivePath.addPoint(new Translation2D(10, -5), 45);
    //drivePath.addPoint(new Translation2D(20, 0), 40);

    //drivePath.addPoint(new Translation2D(40, -25), 45);
    //drivePath.addPoint(new Translation2D(50, 20), 45);



    drive.setAutoPath(drivePath, false);
    System.out.println("Auto selected: " + m_autoSelected);

    
    //new Thread(rt).start();

    //new Thread(ar).start();
    //new Thread(ar).run();    
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  
  }

  @Override 
  public void teleopInit() {
    drive.stopMovement();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

      drive.arcadeDrive(-j.getRawAxis(1), j.getRawAxis(4));
    

  }

  @Override
  public void testInit() {
  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
