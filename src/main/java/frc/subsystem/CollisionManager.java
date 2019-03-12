package frc.subsystem;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.utility.Threaded;
import frc.subsystem.*;
import frc.subsystem.HatchIntake.DeployState;
import frc.subsystem.HatchIntake.IntakeState;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;

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

    private static final CollisionManager cm = new CollisionManager();

    public static CollisionManager getInstance() {
        return cm;
    }    
    
    public CollisionManager() {
        setPeriod(Duration.ofMillis(20));
    }

    synchronized public void groundHatchIntake() {
        //System.out.println("groundHatchIntake");
        intakingHatch = true;
        elevator.setHeight(7);
    }

    // synchronized public void returningHatchIntake() {
    //     returningHatch = true;
    //     elevator.setHeight(7);
    // }

    
    synchronized public void handoffHatch() {
        //System.out.println("handoffHatch");
        handoffHatch = true;
        elevator.setHeight(3);
        handoffStage = 0;
    } 
    

    synchronized public void setIntakeState(BallIntake.DeployState state) {
        requestedState = state;
        ballIntake.setDeployState(BallIntake.DeployState.DEPLOYING);
        elevator.setHeight(elevator.requested);
        waitingOnElevator = true;
    }

    synchronized public boolean isWorking() {
        return handoffHatch || intakingHatch;
    }

    @Override
    synchronized public void update() {
        double starttime = Timer.getFPGATimestamp();
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
