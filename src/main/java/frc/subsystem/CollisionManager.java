package frc.subsystem;

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

    private static final CollisionManager cm = new CollisionManager();

    public static CollisionManager getInstance() {
        return cm;
    }    
    
    public CollisionManager() {

    }

    public void groundHatchIntake() {
        intakingHatch = true;
        elevator.setHeight(7);
    }

    public void returningHatchIntake() {
        returningHatch = true;
        elevator.setHeight(7);
    }

    
     public void handoffHatch() {
        handoffHatch = true;
        elevator.setHeight(3);
        handoffStage = 0;
     } 
    

    public void setIntakeState(BallIntake.DeployState state) {
        requestedState = state;
        ballIntake.setDeployState(BallIntake.DeployState.DEPLOYING);
        elevator.setHeight(elevator.requested);
        waitingOnElevator = true;
    }



    @Override
    public void update() {
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
        } else if(returningHatch) {
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
        } else if(handoffHatch) {
            switch(handoffStage) {
                case 0:
                    if(elevator.isFinished()) handoffStage++;
                    break;
                case 1:
                    groundHatch.setIntakeState(IntakeState.OFF); 
                    combinedIntake.setManipulatorState(ManipulatorState.HATCH);
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
                    groundHatch.setDeployState(DeployState.HANDOFF);
                    if(groundHatch.isFinished()) handoffStage++;
                    break;
                case 2: 
                    //combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
                    elevator.setHeight(7);
                    if(elevator.isFinished()) handoffStage++;
                    break;
                case 3:
                    
                    groundHatch.setDeployState(DeployState.STOW);
                    if(groundHatch.isFinished()) handoffStage++;
                    break;
                default:
                    combinedIntake.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
                    elevator.setHeight(0);
                    turret.setDesired(0, true);
                    turret.restoreSetpoint();
                    handoffHatch = false;
                    break;
            }
        }

    }


}
