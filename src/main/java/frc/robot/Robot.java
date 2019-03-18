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
import frc.auton.Auto;
//import frc.robot.subsystem.Drive;
import frc.utility.math.*;
import frc.utility.control.motion.Path;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.*;
import frc.utility.ThreadScheduler;
import frc.utility.Controller;
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
  public static Controller xbox = new Controller(0);
  //public static Joystick xbox = new Joystick(0);
  public static Joystick stick = new Joystick(1);
  public static Joystick buttonPanel = new Joystick(2);
  Turret turret = Turret.getInstance();
  Elevator elevator = Elevator.getInstance();
  Manipulator manipulator = Manipulator.getInstance();
  Arm arm = Arm.getInstance();
  HatchIntake groundHatch = HatchIntake.getInstance();
  JetsonUDP jetsonUDP = JetsonUDP.getInstance();
  HatchIntake hatchIntake = HatchIntake.getInstance();
  BallIntake ballIntake = BallIntake.getInstance();
  Auto auto;

  ExecutorService executor = Executors.newFixedThreadPool(4);
  ThreadScheduler scheduler = new ThreadScheduler();
  
  boolean firstTeleopRun = true;

  

  private static final String kDefaultAuto = "LeftAuto";
  private static final String kRightAuto = "RightAuto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drive.calibrateGyro();
    m_chooser.setDefaultOption("Left Side Auto", kDefaultAuto);
    m_chooser.addOption("Right Side Auto", kRightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    scheduler.schedule(drive, executor);
		scheduler.schedule(elevator, executor);
		scheduler.schedule(turret, executor);
    scheduler.schedule(collisionManager, executor);
    //scheduler.schedule(manipulator, executor);
    //scheduler.schedule(hatchIntake, executor);
    

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
    auto = new Auto(new Translation2D());
    if (m_autoSelected.equals(kRightAuto)) { 
      auto.targetLeftRocket = false; 
    }
    drive.stopMovement();
    scheduler.resume();
    //elevator.setHeight(10.0); // Just helps show that the code has been run
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    auto.run();
    //System.out.println(elevator.getHeight());
  }

  @Override 
  public void teleopInit() {
    drive.stopMovement();
    scheduler.resume();
    //elevator.setHeight(Constants.HatchElevLow);
   // turret.homeTurret();
    //elevator.elevHome();
    //manipulator.setManipulatorIntakeState(Manipulator.ManipulatorIntakeState.OFF);
    firstTeleopRun = true;
    drive.setTeleop();
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

  boolean openAll = false;
  long openStartTime = 0;

  boolean ballMode = false;
  boolean elevatorManual = false;

  boolean hatchIn = true;
  boolean ballIntakeIn = true;

  
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
      xbox.update();
      
     
      //System.out.println("Desired angle: " + desiredAngle + " actual angle " + turret.getAngle());
      //ground hatch W
    
      //teleopStarttime = Timer.getFPGATimestamp();
      if(xbox.getRisingEdge(1)) {
        ballMode = false;
        if(!hatchIn) collisionManager.handoffHatch();
        else collisionManager.groundHatchIntake(); 
        hatchIn = !hatchIn;
      }

      if(xbox.getRisingEdge(2)) {
        ballMode = true;
        if(!ballIntakeIn) collisionManager.retractBallIntake();
        else collisionManager.extendBallIntake();
        ballIntakeIn = !ballIntakeIn;
      }

      //ballIntake.setSpeed(xbox.getRawAxis(3) - xbox.getRawAxis(2));
      //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 2\n";

      //btn1Edge = xbox.getRawButton(1);
      // if(xbox.getRawButton(1)) collisionManager.handoffHatch();
      // if(xbox.getRawButton(2)) collisionManager.groundHatchIntake();//groundHatch.setDeployState(DeployState.INTAKE);
      //else groundHatch.setDeployState(DeployState.STOW);

      //if(Timer.getFPGATimestamp() - teleopStarttime > 0.01) System.out.println("overrun 1-1: " + (Timer.getFPGATimestamp() - teleopStarttime));
      //teleopStarttime = Timer.getFPGATimestamp();
      //Ball intake test control
      /*if(xbox.getRisingEdge(6)){
        ballIntake.setDeployState(BallIntake.DeployState.DEPLOY);
      } else if(xbox.getRisingEdge(5)) {
        ballIntake.setDeployState(BallIntake.DeployState.STOW);
      }*/
      //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 3\n";
      //System.out.println("hi");

      //teleopStarttime = Timer.getFPGATimestamp();
      if(xbox.getRawAxis(3) > 0.5) {
        ballIntake.setIntakeState(BallIntake.IntakeState.INTAKE);
      } else if(xbox.getRawAxis(2) > 0.5) {
        ballIntake.setIntakeState(BallIntake.IntakeState.EJECT);
      } else {
        ballIntake.setIntakeState(BallIntake.IntakeState.OFF);
      }
      //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 4\n";


      ////teleopStarttime = Timer.getFPGATimestamp();

      //teleopStarttime = Timer.getFPGATimestamp();
      

     // System.out.println(elevator.getHeight());
      //Drive control
      drive.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4) );
      
      if(!collisionManager.isInControl()) {
        //set turret to vision vs setpoint
        if(buttonPanel.getRawButton(4)) turret.setState(TurretState.VISION);
        else {
          turret.setState(TurretState.SETPOINT);
          //turret.restoreSetpoint();
        }

        //Turret control
        if(Math.abs(stick.getY()) > 0.5 || Math.abs(stick.getX()) > 0.5) {
          turret.setDesired(Math.toDegrees((Math.atan2(-stick.getY(), stick.getX()))) - 90, true);
        } else if(Math.abs(stick.getZ()) >= 0.3) {
          turret.addDesired(-stick.getZ()*1.5);
        }
        //Ball vs Turret Mode
        if(stick.getRawButton(3)) ballMode = true;
        else if(stick.getRawButton(4)) ballMode = false;
        //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 1\n";

        //teleopStarttime = Timer.getFPGATimestamp();
        //Arm manual override
        if(buttonPanel.getRawButton(1)) arm.setState(ArmState.EXTEND);
        if(buttonPanel.getRawButton(2)) arm.setState(ArmState.RETRACT);
        //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 7\n";

        //teleopStarttime = Timer.getFPGATimestamp();
        //Zero elevator and elev manual override
        if(buttonPanel.getRawButton(9)) elevator.elevHome();
        //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 8\n";
        
        //teleopStarttime = Timer.getFPGATimestamp();
        if(buttonPanel.getRawButton(10)) turret.homeTurret();
        //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 9\n";
      

        //teleopStarttime = Timer.getFPGATimestamp();
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
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut())
          {}//don't do anything because collision manager is doing things
          else if(stick.getRawButton(1) && yeet == false)  {
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
              //System.out.println("1");
              manipulator.setManipulatorState(ManipulatorState.HATCH);
              manipulator.setManipulatorIntakeState(ManipulatorIntakeState.BALL_HOLD);
            }
          }
          else if(stick.getRawButton(2)) {
            //System.out.println("2");
            manipulator.setManipulatorState(ManipulatorState.BALL);
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //BALL INTAKE
            arm.setState(ArmState.RETRACT);
          } 
          else {
            //System.out.println("3");
            manipulator.setManipulatorState(ManipulatorState.HATCH);
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.BALL_HOLD);
            arm.setState(ArmState.RETRACT);
          } 

          // Comment out in competition robot code. This just makes the robot open up and look nice.
          if(stick.getRawButton(1) && openAll == false)  { // Replace stick.getRawButton with the button that activates this
            openAll = true;
            openStartTime =  System.currentTimeMillis();
            arm.setState(ArmState.EXTEND);
            elevator.setHeightState(Elevator.ElevatorHeight.MIDDLE);
          }
          else if(openAll) {
            if(System.currentTimeMillis() - openStartTime > 2000) {
              Manipulator.getInstance().setManipulatorIntakeState(ManipulatorIntakeState.OFF);
              openAll = false;
            }
            else if(System.currentTimeMillis() - openStartTime > 1000) {
              manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
              ballIntake.setDeployState(BallIntake.DeployState.DEPLOY);
              manipulator.setManipulatorState(Manipulator.ManipulatorState.BALL);
            }
          }
          // End frivilous code

          //elev setpoints
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut())
          {}//don't do anything because collision manager is doing things
          else if(buttonPanel.getRawButton(8)) elevator.setHeight(Constants.BallElevHigh);
          else if(buttonPanel.getRawButton(7)) elevator.setHeight(Constants.BallElevMid);
          else if(buttonPanel.getRawButton(6)) elevator.setHeight(Constants.BallElevLow);
          else if(buttonPanel.getRawButton(5)) elevator.setHeight(Constants.BallElevCargo);


        } else { //hatch mode
          if(!collisionManager.isWorking() && !collisionManager.isBallIntakeOut()) {
            //System.out.println("4");
            manipulator.setManipulatorState(ManipulatorState.HATCH);
          }

          //wheeled intake
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut())
          {}//don't do anything because collision manager is doing things
          else if(stick.getRawButton(1)){ //attempting to outake
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
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut())
          {}//don't do anything because collision manager is doing things
          else if(buttonPanel.getRawButton(8)) elevator.setHeight(Constants.HatchElevHigh);
          else if(buttonPanel.getRawButton(7)) elevator.setHeight(Constants.HatchElevMid);
          else if(buttonPanel.getRawButton(6)) elevator.setHeight(Constants.HatchElevLow);
          else if(buttonPanel.getRawButton(5)) elevator.setHeight(Constants.HatchElevCargo);
        }
      }
      //btn2Edge = xbox.getRawButton(2);
      //btn1Edge = xbox.getRawButton(1);
      firstTeleopRun = false;
  }

  @Override
  public void testInit() {
   // drive.stopMovement();
   // scheduler.resume();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  
  }

  @Override
  public void disabledInit() {
    scheduler.pause();
  }
  
  @Override
  public void disabledPeriodic() {
    //System.out.println(turret.turretHallEffect.get());
    try {
     // System.out.println(JetsonUDP.getInstance().getTargets()[0].x);
     // System.out.println(JetsonUDP.getInstance().getTargets()[0].distance);
    } catch(Exception e) {
      //System.out.println("cant get vision");
    }
  }
}
