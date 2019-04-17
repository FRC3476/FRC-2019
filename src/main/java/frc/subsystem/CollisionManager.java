package frc.subsystem;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.utility.Threaded;
import frc.subsystem.*;
import frc.subsystem.Arm.ArmState;
import frc.subsystem.HatchIntake.DeployState;
import frc.subsystem.HatchIntake.IntakeState;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;
import frc.subsystem.Turret.TurretState;

public class CollisionManager extends Threaded { 
    Turret turret = Turret.getInstance();
    Arm arm = Arm.getInstance();
    Elevator elevator = Elevator.getInstance();
    BallIntake ballIntake = BallIntake.getInstance();
    HatchIntake groundHatch = HatchIntake.getInstance();
    Manipulator combinedIntake = Manipulator.getInstance();
    BallIntake.DeployState requestedState;
    boolean waitingOnElevator = false; 
    boolean waitingOnIntake = false;
    long prevTime;

    boolean intakingHatch = false;

    boolean returningHatch = false;
    boolean handoffHatch = false;
    int handoffStage= 0;
    double holdingTime;

    boolean extendingBallIntake = false;
    boolean retractingBallIntake = false;
    int ballIntakeStage = 0;

    boolean ballIntakeOut = false;
    boolean hatchIntakeOut = false;

    boolean retrieveHatch = false;
    int retrieveHatchStage = 0;
    double retractTime = 0;
    private static final CollisionManager cm = new CollisionManager();

    boolean scoring = false;
    double scoreTime = 0;
    int scoreStage = 0;

    boolean scoringBall = false;
    double scoreTimeBall = 0;
    int scoreStageBall = 0;

    boolean abortHatch = false;
    int abortHatchStage = 0;
    double abortHatchTime = 0;

    boolean manipulatorStall = false;
    boolean cargoShipAutoScore = false;

    boolean abortBall = false;
    int abortBallStage = 0;
    public static CollisionManager getInstance() {
        return cm;
    }    
    
    public CollisionManager() {
        setPeriod(Duration.ofMillis(20));
    }

    synchronized public void abortGroundHatch() {
        if(ballIntakeOut) return;//do not do this sequence while the other is out
        intakingHatch = false;
        handoffHatch = false;
        retrieveHatch = false;
        abortHatch = true;
        abortHatchStage = 0;
        abortHatchTime = Timer.getFPGATimestamp();
        groundHatch.setIntakeState(IntakeState.EJECT);
        combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        elevator.setHeight(7);
    }

    synchronized public void groundHatchIntake() {
        //System.out.println("groundHatchIntake");
        if(ballIntakeOut) return;//do not do this sequence while the other is out
        intakingHatch = true;
        hatchIntakeOut = true;
        elevator.setHeight(Constants.ElevClearance);
        arm.setState(ArmState.RETRACT);
        combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
    }

    // synchronized public void returningHatchIntake() {
    //     returningHatch = true;
    //     elevator.setHeight(7);
    // }

    
    synchronized public void handoffHatch() {
        //System.out.println("handoffHatch");
        if(ballIntakeOut) return;//do not do this sequence while the other is out
        handoffHatch = true;
        elevator.setHeight(3);
        handoffStage = 0;
    } 

    synchronized public void abortBall() {
        abortBall = true;
        abortBallStage = 0;
        ballIntakeOut = true;
        arm.setState(ArmState.RETRACT);
        elevator.setHeight(Constants.HatchElevMid);
        turret.setState(TurretState.SETPOINT);
        turret.setDesired(180, false);
    }
    

    synchronized public void setIntakeState(BallIntake.DeployState state) {
        requestedState = state;
        ballIntake.setDeployState(BallIntake.DeployState.DEPLOYING);
        elevator.setHeight(elevator.requested);
        waitingOnElevator = true;
    }

    synchronized public boolean isWorking() {
        return (handoffHatch || intakingHatch) || (extendingBallIntake) || (retractingBallIntake);
    }

    synchronized public boolean isBallIntakeOut() {
        return ballIntakeOut;
    }

    synchronized public boolean isHatchIntakeOut() {
        return hatchIntakeOut;
    }

    synchronized public boolean isInControl() {
        return ballIntakeOut || hatchIntakeOut || scoring || scoringBall;
    }

    synchronized public void extendBallIntake() {
        if(hatchIntakeOut) return;//do not do this sequence while the other is out
        extendingBallIntake = true;
        ballIntakeOut = true;
        arm.setState(ArmState.RETRACT);
        elevator.setHeight(Constants.ElevatorIntakeSafe);
        turret.setState(TurretState.SETPOINT);
        turret.setDesired(180, false);
        ballIntakeStage = 0;
    }

    synchronized public void retractBallIntake() {
        retractingBallIntake = true;
        elevator.setHeight(Constants.ElevatorIntakeSafe);
        turret.setState(TurretState.SETPOINT);
        turret.setDesired(180, false);
        ballIntakeStage = 0;
    }

    synchronized public boolean isRetrieving() {
        return retrieveHatch;
    }

    synchronized public void score() {
        scoring = true;
        scoreStage = 0;
        scoreTime = Timer.getFPGATimestamp();
        arm.setState(ArmState.EXTEND);
        turret.setVisionLimited(true);
        
    }

    synchronized public void scoreBall(boolean extend, boolean cargoShip) {
        cargoShipAutoScore = cargoShip;
        scoringBall = true;
        if(extend) scoreStageBall = 1; 
        else scoreStageBall = 0;
        scoreTimeBall = Timer.getFPGATimestamp();
        //arm.setState(ArmState.RETRACT);
        
    }

    synchronized public boolean isManipulatorStalled() {
        return manipulatorStall;
    }


    synchronized public boolean isScoring() {
        return scoring;
    }

    synchronized public void retrieveHatch() {
        combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        elevator.setHeight(Constants.HatchElevLow + 4);
        //arm.setState(ArmState.RETRACT);
        holdingTime = Timer.getFPGATimestamp();
        retrieveHatch = true;
        retrieveHatchStage = 0;
    }

    @Override
    synchronized public void update() {
        double starttime = Timer.getFPGATimestamp();

        if(abortHatch) {
            switch(abortHatchStage) {
                case 0:
                    if(elevator.isFinished()) {
                        abortHatchStage++;
                        abortHatchTime = starttime;
                    }
                    break;
                case 1: 
                    groundHatch.setDeploySpeed(-0.15);
                    if(starttime - abortHatchTime >= 2) abortHatchStage++;
                    break;
                case 2:
                    groundHatch.setDeploySpeed(0);
                    groundHatch.setEnc(0);
                    abortHatchStage++;
                    break;
                case 3:
                    groundHatch.setDeployState(DeployState.STOW);
                    if(groundHatch.isFinished()) {
                        abortHatch = false;
                        hatchIntakeOut = false;
                        groundHatch.setIntakeState(IntakeState.OFF);
                        elevator.setHeight(Constants.HatchElevLow);
                    }
                    break;
                    
            }
        }

        if(scoring) {
            switch(scoreStage) {
                case 0:
                    arm.setState(ArmState.EXTEND);
                    if(Timer.getFPGATimestamp()-scoreTime > 0.6) {
                        scoreTime = Timer.getFPGATimestamp();
                        scoreStage++;
                    }
                    break;

                case 1:
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.EJECT);
                    arm.setState(ArmState.RETRACT);
                    if(Timer.getFPGATimestamp()-scoreTime > 0.4) {
                        turret.setVisionLimited(false);
                        scoring = false;
                    }
                    break;
                
                
            }
        }
        if(scoringBall) {
            if(cargoShipAutoScore) {
                turret.setState(TurretState.SETPOINT);
                elevator.setHeight(Constants.BallElevCargo-6);
            }
            //System.out.println(scoreStageBall);
            switch(scoreStageBall) {
               
                case 0: //close
                    arm.setState(ArmState.RETRACT);
                    if(Timer.getFPGATimestamp()-scoreTimeBall > 0.0 && !(!elevator.isFinished() && cargoShipAutoScore)) {
                        scoreTimeBall = Timer.getFPGATimestamp();
                        scoreStageBall+=2;
                    }
                    break;
                case 1: //far
                    arm.setState(ArmState.EXTEND);
                    if(Timer.getFPGATimestamp()-scoreTimeBall > 0.5) {
                        scoreTimeBall = Timer.getFPGATimestamp();
                        scoreStageBall++;
                    }
                    break;
                case 2:
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE); //this is whack, this actually spits out because we are in hatch mode
                    arm.setState(ArmState.RETRACT);
                    if(Timer.getFPGATimestamp()-scoreTimeBall > 0.4) scoringBall = false;
                    break;
                
                
            }
        }

        if(retrieveHatch) {
            switch(retrieveHatchStage) {
                /*
                case 0:
                    if(elevator.isFinished()){
                        retrieveHatchStage++;
                        elevator.setHeight(Constants.HatchElevLow);
                    } 
                    break;
                case 1:
                    
                    if(Timer.getFPGATimestamp() - holdingTime >= 1.2) {
                       // holdingTime = 0;
                       
                        combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
                    }
                    if(Timer.getFPGATimestamp() - holdingTime >= 1.2 && elevator.isFinished()) {
                        holdingTime = 0;
                        retrieveHatch = false;
                    }
                    break;
                */
                case 0:
                    if(elevator.isFinished()){
                        retrieveHatchStage++;
                        arm.setState(ArmState.RETRACT);
                        retractTime = Timer.getFPGATimestamp();
                    } 
                    break;
                case 1:
                    if(Timer.getFPGATimestamp() - retractTime >= 0.75) {
                        elevator.setHeight(Constants.HatchElevLow);
                        retrieveHatchStage++;
                    }
                    break;
                case 2:
                    if(Timer.getFPGATimestamp() - holdingTime >= 1.2) {
                    // holdingTime = 0;
                    
                        combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
                    }
                    if(Timer.getFPGATimestamp() - holdingTime >= 1.2 && elevator.isFinished()) {
                        holdingTime = 0;
                        retrieveHatch = false;
                    }
                break;
            } 
        } 
        /*
        if(waitingOnElevator) {
            if(elevator.isSafe()) {
                waitingOnElevator = false;
                prevTime = System.currentTimeMillis();
            } 
            else ;
        }
        if(waitingOnIntake) {
            if(System.currentTimeMillis() - prevTime > Constants.IntakeDeployTime) {
                waitingOnIntake = false;
                ballIntake.setDeployState(requestedState);
                
            }
        } */

        if(extendingBallIntake) {
            switch(ballIntakeStage) {
                case 0: 
                    if(elevator.isFinishedOrHigher()) ballIntakeStage++;
                    break;
                case 1:
                    ballIntake.setDeployState(BallIntake.DeployState.DEPLOY);
                    combinedIntake.setManipulatorState(ManipulatorState.BALL);
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
                    ballIntakeStage++;
                    break;
                case 2:
                    if(ballIntake.isFinished()) ballIntakeStage++;
                    break;                
                case 3:
                    elevator.setHeight(Constants.BallElevCargoGroundIntake);
                    //turret.setDesired(0, true);
                    //turret.restoreSetpoint();
                    extendingBallIntake = false;
                    System.out.println("extended ball intake");
                    break;
            }
        } 
        else if(retractingBallIntake) {
            manipulatorStall = false;
            switch(ballIntakeStage) {
                case 0:
                    if(elevator.isFinished()) ballIntakeStage++;
                    break;
                case 1:
                    ballIntake.setDeployState(BallIntake.DeployState.STOW);
                    ballIntakeStage++;
                    break;
                case 2:
                    if(ballIntake.isFinished()) {
                        elevator.setHeight(Constants.BallElevLow);
                        //reset turret to field centric
                        turret.setDesired(180, true);
                        turret.restoreSetpoint();
                        ballIntakeOut = false;
                        retractingBallIntake = false;
                        System.out.println("Retracted ball intake");
                    }
                    break;
            }
        }
        else if(abortBall) {
            switch(abortBallStage) {
                case 0: 
                    if(elevator.isFinished()) abortBallStage++;
                    break;
                case 1:
                    ballIntake.setDeployState(BallIntake.DeployState.DEPLOY);
                    abortBallStage++;
                    break;
                case 2:
                    if(ballIntake.isFinished()) abortBallStage++;
                    break;                
                case 3:
                    
                    combinedIntake.setManipulatorState(ManipulatorState.BALL);
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
                    //turret.setDesired(0, true);
                    //turret.restoreSetpoint();
                    extendingBallIntake = false;
                    abortBall = false;
                    System.out.println("extended ball intake");
                    break;
            }
        } 
        
        
        if(isBallIntakeOut() && extendingBallIntake != true) {
        
            if(combinedIntake.getCurrent() > 20) {
                combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.BALL_HOLD);    
                combinedIntake.setManipulatorState(ManipulatorState.HATCH);
                manipulatorStall = true;
            }
        }
        //System.out.println("combined intake current: " + combinedIntake.getCurrent());
        
        if(intakingHatch) {
            turret.setDesired(0, false);
            if(elevator.isFinished()) {
                groundHatch.setDeployState(DeployState.INTAKE);
                
                if(groundHatch.isFinished()) {
                    intakingHatch = false;
                    elevator.setHeight(3);
                    //turret.setDesired(0, true);
                    //turret.restoreSetpoint();
                    groundHatch.setIntakeState(IntakeState.INTAKE);
                } 
            }
        } /*else if(returningHatch) {
            turret.setDesired(0, false);
            if(elevator.isFinished()) {
                groundHatch.setDeployState(DeployState.STOW);
                
                if(groundHatch.isFinished()) {
                    returningHatch = false;
                    elevator.setHeight(0);
                    turret.setDesired(0, true);
                    turret.restoreSetpoint();
                } 
            }    
        } */else if(handoffHatch) {
            //int snap = handoffStage;
            switch(handoffStage) {
                case 0:
                    if(elevator.isFinished()) handoffStage++;//wait for elevator to finish moving to handoff height
                    break;
                case 1:
                    groundHatch.setIntakeState(IntakeState.OFF); 
                    combinedIntake.setManipulatorState(ManipulatorState.HATCH);
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
                    groundHatch.setDeployState(DeployState.HANDOFF);
                    if(groundHatch.isFinished()) handoffStage++;//wait for groundhatch to finish moving to handoff position
                    break;
                case 2: 
                    //combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
                    elevator.setHeight(7);
                    if(elevator.isFinished()) {//wait for elevator to move up to remove the hatch from the intake
                        holdingTime = Timer.getFPGATimestamp();//start timing the time the manipulator wheels are running
                        handoffStage++;
                    }
                    break;
                case 3:
                    groundHatch.setDeployState(DeployState.STOW);
                    if(groundHatch.isFinished()) handoffStage++;//wait for the intake to stow
                    break;
                case 4:
                    elevator.setHeight(Constants.HatchElevLow);
                    //wait for a specified time of intaking the hatch
                    if((Timer.getFPGATimestamp() - holdingTime) > Constants.HandoffHoldTime) handoffStage++;
                    break;
                default:
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
                    //reset turret to field centric
                    turret.setDesired(0, true);
                    turret.restoreSetpoint();
                    hatchIntakeOut = false;
                    handoffHatch = false;
                    break;
            }
            //System.out.println("handoffStage " + snap);

        }
        if(Timer.getFPGATimestamp() - starttime > getPeriod())
		{
			//System.out.println("colman time: " + (Timer.getFPGATimestamp() - starttime));
		}
    }


}
