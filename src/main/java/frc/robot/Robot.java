// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import frc.auton.*;
import frc.subsystem.*;
import frc.subsystem.Arm.ArmState;
import frc.subsystem.HatchIntake.DeployState;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;
import frc.subsystem.Turret.TurretState;
//import frc.robot.subsystem.Drive;
import frc.utility.math.*;
import frc.utility.telemetry.TelemetryServer;
import frc.utility.control.motion.Path;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

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
  public static Controller stick = new Controller(1);
  public static Controller buttonPanel = new Controller(2);
  TelemetryServer telemetryServer = TelemetryServer.getInstance();
  Turret turret = Turret.getInstance();
  Elevator elevator = Elevator.getInstance();
  Manipulator manipulator = Manipulator.getInstance();
  Arm arm = Arm.getInstance();
  HatchIntake groundHatch = HatchIntake.getInstance();
  JetsonUDP jetsonUDP = JetsonUDP.getInstance();
  HatchIntake hatchIntake = HatchIntake.getInstance();
  BallIntake ballIntake = BallIntake.getInstance();
  RobotTracker robotTracker = RobotTracker.getInstance();
  Climber climber = Climber.getInstance();

  ExecutorService executor = Executors.newFixedThreadPool(4);
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;
  TemplateAuto option;

  boolean firstTeleopRun = true;

  

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<String>();
  private final SendableChooser<String> dir_chooser = new SendableChooser<String>();
  private final SendableChooser<String> goodbad = new SendableChooser<String>();
  private final SendableChooser<String> start_chooser = new SendableChooser<String>();
  private final SendableChooser<String> red_blue = new SendableChooser<String>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drive.calibrateGyro();
    m_chooser.addOption("Cargo F_F", "Cargo F_F");
    m_chooser.addOption("Cargo F_1", "Cargo F_1");
    m_chooser.addOption("Cargo 1_2", "Cargo 1_2");
    m_chooser.setDefaultOption("Rocket Mid", "Rocket Mid");
    
    SmartDashboard.putData("Autonomous Mode", m_chooser);

    dir_chooser.setDefaultOption("Left", "Left");
    dir_chooser.addOption("Right", "Right");
    SmartDashboard.putData("Starting Side", dir_chooser);

    start_chooser.setDefaultOption("Lvl1", "Lvl1");
    start_chooser.addOption("Lvl2", "Lvl2");
    SmartDashboard.putData("Starting Height", start_chooser);

    red_blue.setDefaultOption("Red", "Red");
    red_blue.addOption("Blue", "Blue");
    SmartDashboard.putData("Red and Blue", red_blue);

    goodbad.setDefaultOption("good", "good");
    goodbad.addOption("bad","bad");
    goodbad.addOption("mindBuisness", "mindBuisness");
    goodbad.addOption("mInDbUiSnEsS", "mInDbUiSnEsS");
    SmartDashboard.putData("Good or Bad? To be or Not to Be?", goodbad);


    scheduler.schedule(drive, executor);
		scheduler.schedule(elevator, executor);
		scheduler.schedule(turret, executor);
    scheduler.schedule(collisionManager, executor);
    scheduler.schedule(jetsonUDP, executor);
    scheduler.schedule(robotTracker, executor);
    scheduler.schedule(climber, executor);
    

    turret.homeTurret();
    elevator.elevHome();
    drive.setSimpleDrive(false);
    
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
  boolean autoDone;

  @Override
  public void autonomousInit() {
    climber.setDeploySolenoid(false);
    autoDone = false;
    scheduler.resume();

    int autoDir = 1;
    double startPos = 48+18;

    if(dir_chooser.getSelected().equals("Right")) autoDir = -1;
    else autoDir = 1;

    if(start_chooser.getSelected().equals("Lvl2")) startPos = 18+19-3;//-8;
    else startPos = 48+18;

    if(m_chooser.getSelected().equals("Cargo 1_2")&& red_blue.getSelected().equals("Red")) option = new Ship1_2Red(autoDir, startPos);
    else if(m_chooser.getSelected().equals("Cargo 1_2")&& red_blue.getSelected().equals("Blue")) option = new Ship1_2Blue(autoDir, startPos);
    else if(m_chooser.getSelected().equals("Cargo F_1")) option = new ShipF_1(autoDir, startPos);
    else if(m_chooser.getSelected().equals("Cargo F_F")) option = new ShipF_F(autoDir, startPos);
    else if(m_chooser.getSelected().equals("Rocket Mid") && red_blue.getSelected().equals("Red")) option = new RocketMidRed(autoDir, startPos);
    else option = new RocketMidBlue(autoDir, startPos);
    
    //option = new DriveForward();
    auto = new Thread(option);
    auto.start();
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    buttonPanel.update();
    if(!autoDone) {
    //for(int i = 1; i < 8; i++) {
      if(buttonPanel.getRawButton(2)) {
        autoDone = true;
        teleopInit();
      }
    //}
    }
    if(autoDone) {
      teleopPeriodic();
    }
    
  
  } 

  public void killAuto() {
    if(auto != null) {
      turret.setDesired(0, true);
      turret.restoreSetpoint();
      elevator.setHeight(Math.max(elevator.getHeight(), Constants.HatchElevLow));
      auto.interrupt();
      drive.stopMovement();
      drive.setTeleop();
    }
    if(option != null) {
      option.killSwitch();
    }
  }

  @Override 
  public void teleopInit() {
    climber.setDeploySolenoid(false);
    killAuto();
    System.out.println("teleop init!");
    //drive.stopMovement();

    turret.resetDT();
    elevator.resetDT();
    scheduler.resume();
    //elevator.setHeight(Constants.HatchElevLow);
   // turret.homeTurret();
    //elevator.elevHome();
    //manipulator.setManipulatorIntakeState(Manipulator.ManipulatorIntakeState.OFF);
    firstTeleopRun = true;
    //drive.setTeleop();
    
    
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

  boolean hatchIn = true;
  boolean ballIntakeIn = true;

  boolean hatchOutake = false;

  boolean elevReturn = false;

  boolean autoScoreAllow = true;

  boolean cargoMode = false;

  boolean climberDisable = false;

  double climberPower = 0;
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      //System.out.println("turret " + turret.getAngle());
      //System.out.println(drive.getLeftSpeed() + " right: " + drive.getRightSpeed());
      xbox.update();
      stick.update();
      buttonPanel.update();

      if(stick.getRawButton(9) && stick.getRawButton(10)) {
        climber.setDeploySolenoid(true);
        climberPower = 0.75;
      }

      
      //System.out.println("climber power: " + climberPower);
      /*
      if(!stick.getRawButton(5)) climberPower = 0;
      else if(stick.getRawAxis(3) > 0) climberPower = -0.15;
      else climberPower = 0.75;
      climber.setPower(climberPower);
      */
      
        if(stick.getRawButton(5) && stick.getRawAxis(3) < 0 /*&& !climberDisable*/) climberPower = 0.9;
        else if(stick.getRawButton(5) /*&& !climberDisable*/) climberPower = -0.15;
        if(stick.getRawButton(6)) {
           //climberDisable = true;
           climberPower = 0;
        }
        climber.setPower(climberPower);
        
      

      if(hatchIntake.getCurrent() > 3 || manipulator.getCurrent() > 50 || collisionManager.isManipulatorStalled()) {
        //xbox.setRumble(RumbleType.kLeftRumble, 1.0);
        xbox.setRumble(RumbleType.kRightRumble, 1.0);
      } else {
        //xbox.setRumble(RumbleType.kLeftRumble, 0);
        xbox.setRumble(RumbleType.kRightRumble, 0);
      }
     // System.out.println("manipulator current: " + manipulator.getCurrent());
      //System.out.println("Desired angle: " + desiredAngle + " actual angle " + turret.getAngle());
      //ground hatch W

      if(xbox.getRawButton(4) || xbox.getRawButton(6)) drive.setShiftState(true);
      else drive.setShiftState(false);

      if(stick.getRisingEdge(7)) collisionManager.abortGroundHatch();
    
      //teleopStarttime = Timer.getFPGATimestamp();
      //hatch
      
      /*
      if(xbox.getRisingEdge(1)) {
        ballMode = false;
        
        if(collisionManager.isHatchIntakeOut()) {
          collisionManager.handoffHatch();
          
        }
        else collisionManager.groundHatchIntake(); 
        //hatchIn = !
      } */ //temprary disabled

      //ball
      if(xbox.getRisingEdge(2)) {
        ballMode = true;
        if(collisionManager.isBallIntakeOut()) collisionManager.retractBallIntake();
        else collisionManager.extendBallIntake();
        //ballIntakeIn = !collisionManager.isBallIntakeOut();
      } else if(stick.getRisingEdge(8)) {
        ballMode = true;
        collisionManager.abortBall();

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
      if(collisionManager.isBallIntakeOut() && !collisionManager.isWorking()) {
        if(xbox.getRawAxis(3) > 0.5) { //>0.5
          ballIntake.setIntakeState(BallIntake.IntakeState.INTAKE);
          //ballIntake.setSpeed((-xbox.getRawAxis(3)*0.75));
        } else if(xbox.getRawAxis(2) > 0.5) { //>0.5
          ballIntake.setIntakeState(BallIntake.IntakeState.EJECT);
          //ballIntake.setSpeed((xbox.getRawAxis(2) * 0.75));

        } else {
          ballIntake.setIntakeState(BallIntake.IntakeState.OFF);
        }
      }
      else {
        ballIntake.setIntakeState(BallIntake.IntakeState.OFF);
      }
      
      //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 4\n";


      ////teleopStarttime = Timer.getFPGATimestamp();

      //teleopStarttime = Timer.getFPGATimestamp();
      //System.out.println(autoScoreAllow);

     // System.out.println(elevator.getHeight());
      //Drive control
      //System.out.println("range: " + turret.isInBallRange());
      if(xbox.getRisingEdge(5)) drive.startHold();
      if(xbox.getFallingEdge(5)) drive.endHold();
      if(!xbox.getRawButton(5)) drive.arcadeDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4));

      
      if(buttonPanel.getFallingEdge(1)) turret.resetDistance();;
     // System.out.println(!collisionManager.isInControl());
      if(!collisionManager.isInControl()) {
        //set turret to vision vs setpoint
        if(buttonPanel.getRawButton(4)) turret.setState(TurretState.VISION);
        else if(buttonPanel.getRawButton(1)) {
          turret.setState(TurretState.VISION);

          if(turret.isFinished()) {
            if(elevator.getRequested() == Constants.BallElevCargo) cargoMode = true;
            else cargoMode = false;

            if(ballMode && turret.isInBallRange() == 1 && autoScoreAllow) {
              collisionManager.scoreBall(true, cargoMode);
              autoScoreAllow = false;
            } 
            else if(ballMode && turret.isInBallRange() == 0 && autoScoreAllow) {
              collisionManager.scoreBall(false, cargoMode);
              autoScoreAllow = false;
            } 
            else if((!ballMode && turret.isInRange()) && (autoScoreAllow && turret.getVelocity() < 3)) {
              collisionManager.score();
              autoScoreAllow = false;
            }
          }
        } 
        
        else {
          turret.setState(TurretState.SETPOINT);
          //turret.restoreSetpoint();
        }

        if(!buttonPanel.getRawButton(1)) autoScoreAllow = true;

        //Turret control
        //axis 0 is x, 1 is y, 2 is yaw
        //System.out.println("x: " + stick.getRawAxis(0) + " y: " + stick.getRawAxis(1));
        if(Math.abs(stick.getRawAxis(1)) > 0.5 || Math.abs(stick.getRawAxis(0)) > 0.5) {
          //System.out.println("x: " + stick.getY() + " y: " + stick.getX() + "ang: " + (Math.toDegrees((Math.atan2(-stick.getY(), stick.getX()))) - 90));
          turret.setDesired(Math.toDegrees((Math.atan2(-stick.getRawAxis(1), stick.getRawAxis(0)))) - 90, true);
        } else if(Math.abs(stick.getZ()) >= 0.3) {
          turret.addDesired(-stick.getZ()*Constants.kTurretManual);
        } else if(stick.getPOV() != -1) {
          turret.setDesired(stick.getPOV(), false);
        }
        //Ball vs Turret Mode
        if(stick.getRawButton(3)) ballMode = true;
        else if(stick.getRawButton(4)) ballMode = false;
        //if(firstTeleopRun) toPrint += (Timer.getFPGATimestamp() - teleopStarttime) + " 1\n";

        //teleopStarttime = Timer.getFPGATimestamp();
        //Arm manual override
        //if(buttonPanel.getRawButton(1)) arm.setState(ArmState.EXTEND);
        //if(buttonPanel.getRawButton(2)) arm.setState(ArmState.RETRACT);
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
          if(buttonPanel.getPOV() == 0)
            elevator.setHeight(elevator.getHeight()+Constants.kElevatorManual);
            //elevator.manualControl(0.2);
          else if (buttonPanel.getPOV() == 180) 
            elevator.setHeight(elevator.getHeight()-Constants.kElevatorManual);
            //elevator.manualControl(-0.2);
          //Elevator.setWonkavator()
        }
      
        if(elevatorManual == true && buttonPanel.getPOV() == -1) {
          elevatorManual = false;
          //elevator.setHeight(elevator.getHeight());
        }
        
        //ball mode
        if(ballMode) { 
          //wheeled intake
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut()) {}
          //don't do anything because collision manager is doing things
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
          else if(stick.getRawButton(2) || buttonPanel.getRawButton(3)) {
            //System.out.println("2");
            manipulator.setManipulatorState(ManipulatorState.BALL);
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //BALL INTAKE
            arm.setState(ArmState.RETRACT);
          } else if(buttonPanel.getRawButton(2)) { //test
            manipulator.setManipulatorState(ManipulatorState.HATCH);
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //BALL EJECT
            arm.setState(ArmState.RETRACT);
          }
          else {
            //System.out.println("3");
            manipulator.setManipulatorState(ManipulatorState.HATCH);
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.BALL_HOLD);
            arm.setState(ArmState.RETRACT);
          } 

          //elev setpoints
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut() || elevatorManual)
          {}//don't do anything because collision manager is doing things
          else if(buttonPanel.getRawButton(8)) elevator.setHeight(Constants.BallElevHigh);
          else if(buttonPanel.getRawButton(7)) elevator.setHeight(Constants.BallElevMid);
          else if(buttonPanel.getRawButton(6)) elevator.setHeight(Constants.BallElevLow);
          else if(buttonPanel.getRawButton(5)) {
            elevator.setHeight(Constants.BallElevCargo);
          }

          


        } else { //hatch mode
          if(!collisionManager.isWorking() && !collisionManager.isBallIntakeOut()) {
            //System.out.println("4");
            manipulator.setManipulatorState(ManipulatorState.HATCH);
          }

          //wheeled intake
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut())
          {}//don't do anything because collision manager is doing things
          else if(stick.getRawButton(1)){ //attempting to outake
            hatchOutake = true;
            arm.setState(ArmState.EXTEND);
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
            if(stick.getRawButton(2)) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.EJECT);
          }else if(stick.getRawButton(2)) { //attempting to intake
            if(hatchOutake) {
              arm.setState(ArmState.RETRACT);
              manipulator.setManipulatorIntakeState(ManipulatorIntakeState.EJECT);
            }
            else { //HATCH INTAKE FROM LOADER
              manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
              arm.setState(ArmState.EXTEND);
              if(elevator.getHeight() <= 3) {
                elevator.setHeight(Constants.HatchHP);
                elevReturn = true;
              }
              intakeAttempted = true;
              intakeAttemptedTime = System.currentTimeMillis();
            }
          } 
          else {  //attempting to hold otherwise
           

            if(elevReturn) {
              elevReturn = false;
              
              collisionManager.retrieveHatch();
            }
            if(collisionManager.isRetrieving());
            else if(intakeAttempted == true) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
            else if(buttonPanel.getRawButton(3)) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
            else  manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
            if(!collisionManager.isRetrieving()) arm.setState(ArmState.RETRACT);
            if(System.currentTimeMillis() - intakeAttemptedTime > 350) {
              intakeAttempted = false;
            }
          }
          //elev setpoints
          if(collisionManager.isWorking() || collisionManager.isBallIntakeOut() || elevatorManual)
          {}//don't do anything because collision manager is doing things
          else if(buttonPanel.getRawButton(8)) elevator.setHeight(Constants.HatchElevHigh);
          else if(buttonPanel.getRawButton(7)) elevator.setHeight(Constants.HatchElevMid);
          else if(buttonPanel.getRawButton(6)) elevator.setHeight(Constants.HatchElevLow);
          else if(buttonPanel.getRawButton(5)) elevator.setHeight(Constants.HatchElevLow);
        }
      }
      
      
      //btn2Edge = xbox.getRawButton(2);
      //btn1Edge = xbox.getRawButton(1);
      if(stick.getFallingEdge(2)) hatchOutake = false;
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
    killAuto();
    switch(turret.twistDir) {
      case 1:
        System.out.println("Turret: ccw");
        break;
      case -1:
        System.out.println("Turret: cw");
        break;
    }
    
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
