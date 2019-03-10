// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import frc.auton.DriveForward;
import frc.subsystem.*;
import frc.subsystem.Arm.ArmState;
import frc.subsystem.HatchIntake.DeployState;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;
import frc.subsystem.Turret.TurretState;
//import frc.robot.subsystem.Drive;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import edu.wpi.first.wpilibj.Joystick;
import java.util.concurrent.*;
import frc.utility.ThreadScheduler;
import frc.utility.JetsonUDP;
import frc.utility.VisionTarget;



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
  HatchIntake groundHatch = HatchIntake.getInstance();
  JetsonUDP jetsonUDP = JetsonUDP.getInstance();
  HatchIntake hatchIntake = HatchIntake.getInstance();

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
    scheduler.schedule(hatchIntake, executor);
    turret.homeTurret();
    elevator.elevHome();
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
    //System.out.println(elevator.getHeight());
  
  }

  @Override 
  public void teleopInit() {
    drive.stopMovement();
    scheduler.resume();
   // turret.homeTurret();
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


  boolean visionMode = false;

  boolean btn1Edge = false;
  boolean btn2Edge = false;  

  int hatchIntakeOption = 0;
  int ballIntakeOption = 0;
  boolean intakeAttempted = false;
  long intakeAttemptedTime = 0;

  boolean yeet = false;
  long yeetTime = 0;

  boolean ballMode = false;
  boolean elevatorManual = false;
  final double ballElevHigh = 55.18;
  final double ballElevMid = 31.02;
  final double ballElevLow = 5.8;
  final double ballElevCargo = 5.8;

  final double hatchElevHigh = 51.6;
  final double hatchElevMid = 26.4;
  final double hatchElevLow = 0;
  final double hatchElevCargo = 4.5;


  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      //System.out.println(hatchIntake.getAngle());
      //set turret to vision vs setpoint
      if(stick.getRawButton(6)) turret.setState(TurretState.VISION);
      else {
        turret.setState(TurretState.SETPOINT);
        //turret.restoreSetpoint();
      }

      //System.out.println("Desired angle: " + desiredAngle + " actual angle " + turret.getAngle());
      //ground hatch 
      /*
      groundHatch.setDeploySpeed(xbox.getRawAxis(3)-xbox.getRawAxis(2));
      if(xbox.getRawButton(1)) groundHatch.setSpeed(1.0);
      else if(xbox.getRawButton(2)) groundHatch.setSpeed(-1.0);
      else groundHatch.setSpeed(0);
      */
      if(xbox.getRawButton(1)) groundHatch.setDeployState(DeployState.HANDOFF);
      else if(xbox.getRawButton(2)) groundHatch.setDeployState(DeployState.INTAKE);
      else groundHatch.setDeployState(DeployState.STOW);


      //Turret control
      if(Math.abs(stick.getY()) > 0.5 || Math.abs(stick.getX()) > 0.5) turret.setDesired(Math.toDegrees((Math.atan2(-stick.getY(), stick.getX()))) - 90, true);
    
      else if(Math.abs(stick.getZ()) >= 0.3) {
         turret.addDesired(-stick.getZ());
      }
     // System.out.println(elevator.getHeight());
      //Drive control
     // drive.arcadeDrive(-xbox.getRawAxis(1) * -xbox.getRawAxis(1) * xbox.getRawAxis(1)/Math.abs(-xbox.getRawAxis(1)), xbox.getRawAxis(4) * xbox.getRawAxis(4) * xbox.getRawAxis(4)/Math.abs(xbox.getRawAxis(4)));
      drive.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4) );
      //Ball vs Turret Mode
      if(stick.getRawButton(3)) ballMode = true;
      else if(stick.getRawButton(4)) ballMode = false;

      //Arm manual override
      if(buttonPanel.getRawButton(1)) arm.setState(ArmState.EXTEND);
      if(buttonPanel.getRawButton(2)) arm.setState(ArmState.RETRACT);

      //Zero elevator and elev manual override
      if(buttonPanel.getRawButton(9)) elevator.elevHome();
      if(buttonPanel.getRawButton(10)) turret.homeTurret();
    
      /*
      if(Math.abs(xbox.getRawAxis(1)) > 0.1) {
        elevatorManual = true;
        elevator.manualControl(-xbox.getRawAxis(1) * 0.6);
        //Elevator.setWonkavator()
      }
      if(elevatorManual == true && xbox.getRawAxis(1) < 0.3) {
        //elevatorManual = false;
        //elevator.setHeight(elevator.getHeight());
      }*/

      if(buttonPanel.getPOV() != -1) {
        elevatorManual = true;
        if(buttonPanel.getPOV() == 90)
          elevator.manualControl(0.2);
        else if (buttonPanel.getPOV() == 270) 
          elevator.manualControl(-0.2);
        //Elevator.setWonkavator()
      }
      if(elevatorManual == true && buttonPanel.getPOV() == -1) {
        elevatorManual = false;
        elevator.setHeight(elevator.getHeight());
      }

      //ball mode
      if(ballMode) { 
        //wheeled intake
        if(stick.getRawButton(1) && yeet == false)  {
          yeet = true;
          yeetTime =  System.currentTimeMillis();
          arm.setState(ArmState.EXTEND);
          //manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //BALL EJECT
        }
        else if(yeet) {
        
          if(System.currentTimeMillis() - yeetTime > 1000) {
            Manipulator.getInstance().setManipulatorIntakeState(ManipulatorIntakeState.OFF);
            yeet = false;
          }
          else if(System.currentTimeMillis() - yeetTime > 500) {
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //MAYBE SHOULD BE EJECT
          } 
           else {
            manipulator.setManipulatorState(ManipulatorState.HATCH);
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.BALL_HOLD);
          }
        }
        else if(stick.getRawButton(2)) {
          
          manipulator.setManipulatorState(ManipulatorState.BALL);
          manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //BALL INTAKE
          arm.setState(ArmState.RETRACT);
        } 
        else {
          manipulator.setManipulatorState(ManipulatorState.HATCH);
          manipulator.setManipulatorIntakeState(ManipulatorIntakeState.BALL_HOLD);
          arm.setState(ArmState.RETRACT);
        } 

        //elev setpoints
        if(buttonPanel.getRawButton(8)) elevator.setHeight(ballElevHigh);
        else if(buttonPanel.getRawButton(7)) elevator.setHeight(ballElevMid);
        else if(buttonPanel.getRawButton(6)) elevator.setHeight(ballElevLow);
        else if(buttonPanel.getRawButton(5)) elevator.setHeight(ballElevCargo);


      } else { //hatch mode
        manipulator.setManipulatorState(ManipulatorState.HATCH);

        //wheeled intake
        if(stick.getRawButton(1)){ //attempting to outake
          arm.setState(ArmState.EXTEND);
          manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
          if(stick.getRawButton(2)) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.EJECT);
        }else if(stick.getRawButton(2)) { //attempting to intake
          manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
          arm.setState(ArmState.EXTEND);
          intakeAttempted = true;
          intakeAttemptedTime = System.currentTimeMillis();
        }  
        else {  //attempting to hold otherwise
          if(intakeAttempted == true) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
          else  manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
          arm.setState(ArmState.RETRACT);
          if(System.currentTimeMillis() - intakeAttemptedTime > 350) {
            intakeAttempted = false;
          }
        }
        //elev setpoints
        if(buttonPanel.getRawButton(8)) elevator.setHeight(hatchElevHigh);
        else if(buttonPanel.getRawButton(7)) elevator.setHeight(hatchElevMid);
        else if(buttonPanel.getRawButton(6)) elevator.setHeight(hatchElevLow);
        else if(buttonPanel.getRawButton(5)) elevator.setHeight(hatchElevCargo);
      }

      btn2Edge = stick.getRawButton(2);
      btn1Edge = stick.getRawButton(1);
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

  @Override
  public void disabledPeriodic() {
    try {
     // System.out.println(JetsonUDP.getInstance().getTargets()[0].x);
     // System.out.println(JetsonUDP.getInstance().getTargets()[0].distance);
    } catch(Exception e) {
      //System.out.println("cant get vision");
    }
  }
}
