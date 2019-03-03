// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import frc.auton.DriveForward;
import frc.subsystem.*;
import frc.subsystem.Arm.ArmState;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
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
  public static Joystick xbox = new Joystick(0);
  public static Joystick stick = new Joystick(1);
  public static Joystick buttonPanel = new Joystick(2);
  Turret turret = Turret.getInstance();
  Elevator elevator = Elevator.getInstance();
  Manipulator manipulator = Manipulator.getInstance();
  Arm arm = Arm.getInstance();

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
    scheduler.schedule(manipulator, executor);

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
    System.out.println("Auto selected: " + m_autoSelected);
    //new DriveForward().run();
    drive.stopMovement();
    scheduler.resume();
    //turret.setAngle(90);
    elevator.setHeight(10.0);
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
    System.out.println(elevator.getHeight());
  
  }

  @Override 
  public void teleopInit() {
    drive.stopMovement();
    scheduler.resume();
    turret.homeTurret();
    //elevator.elevHome();
    manipulator.setManipulatorIntakeState(Manipulator.ManipulatorIntakeState.OFF);
  }
  /*
  float angle = 0;
  boolean yeet = false;
  long yeetTime = System.currentTimeMillis();
  boolean btn4Edge = false;
  boolean btn3Edge = false;
  boolean prevManipulator = false;
  */

  boolean ballMode = false;
  final double ballElevHigh = 20;
  final double ballElevMid = 10;
  final double ballElevLow = 4.5;
  final double ballElevCargo = 5;

  final double hatchElevHigh = 20;
  final double hatchElevMid = 10;
  final double hatchElevLow = 4.5;
  final double hatchElevCargo = 5;

  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      //turret.setAngle(angle);'
      
      drive.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4));

      if(stick.getRawButton(3)) ballMode = true;
      else if(stick.getRawButton(4)) ballMode = false;

      if(buttonPanel.getRawButton(1)) arm.setState(ArmState.EXTEND);
      if(buttonPanel.getRawButton(2)) arm.setState(ArmState.RETRACT);

      if(buttonPanel.getRawButton(9)) elevator.zero();

      if(ballMode) { //ball mode
        //wheeled intake
        if(stick.getRawButton(0)) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //BALL EJECT
        else if(stick.getRawButton(1)) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.EJECT); //BALL INTAKE
        //elev setpoints
        if(buttonPanel.getRawButton(8)) elevator.setHeight(ballElevHigh);
        if(buttonPanel.getRawButton(7)) elevator.setHeight(ballElevMid);
        if(buttonPanel.getRawButton(6)) elevator.setHeight(ballElevLow);
        if(buttonPanel.getRawButton(5)) elevator.setHeight(ballElevCargo);


      } else { //hatch mode
        //wheeled intake
        if(stick.getRawButton(0)) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.EJECT); //HATCH EJECT
        else if(stick.getRawButton(1)) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //HATCH INTAKE
        //elev setpoints
        if(buttonPanel.getRawButton(8)) elevator.setHeight(hatchElevHigh);
        if(buttonPanel.getRawButton(7)) elevator.setHeight(hatchElevMid);
        if(buttonPanel.getRawButton(6)) elevator.setHeight(hatchElevLow);
        if(buttonPanel.getRawButton(5)) elevator.setHeight(hatchElevCargo);
      }

    /*
      if(j.getRawButton(5)) angle -= 0.6;
      else if(j.getRawButton(6)) angle+=0.6;
      drive.arcadeDrive(-j.getRawAxis(1), j.getRawAxis(4));
      turret.setAngle(angle);
      

      if(j.getRawButton(3) && yeet == false) {
        yeet = true;
        yeetTime =  System.currentTimeMillis();
        Arm.getInstance().setState(ArmState.EXTEND);
      }
      if(yeet) {
        if(System.currentTimeMillis() - yeetTime > 500) {
          Manipulator.getInstance().setManipulatorIntakeState(ManipulatorIntakeState.EJECT);
        } 
        if(System.currentTimeMillis() - yeetTime > 1000) {
          Manipulator.getInstance().setManipulatorIntakeState(ManipulatorIntakeState.OFF);
          yeet = false;
        }
      } else {
        if(j.getRawButton(2)) {
          manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        } else {
          manipulator.setManipulatorIntakeState(ManipulatorIntakeState.OFF);
        }
      }

      if(j.getRawButton(1)) {
        Arm.getInstance().setState(ArmState.RETRACT);
      } 

      if(j.getRawButton(4) && !btn4Edge) {
        System.out.println("switching claw state");
        if(prevManipulator) manipulator.setManipulatorState(Manipulator.ManipulatorState.BALL);
        else manipulator.setManipulatorState(Manipulator.ManipulatorState.HATCH);
        prevManipulator = !prevManipulator;
      }

      
      btn4Edge = j.getRawButton(4);

      if(j.getRawButton(4) == false) {
        btn4Edge = false;
      }

      if(j.getRawButton(8)) elevator.setHeight(4.5);
      if(j.getRawButton(7)) elevator.zero();

      if(j.getRawAxis(2) > 0.1) elevator.setHeight(elevator.getHeight() - 2*Robot.j.getRawAxis(2));
			else if(j.getRawAxis(3) > 0.1)  elevator.setHeight(elevator.getHeight() + 2* Robot.j.getRawAxis(3));
      */
      System.out.println(elevator.getHeight());
     
  }

  @Override
  public void testInit() {
    drive.stopMovement();

    scheduler.resume();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }
}
