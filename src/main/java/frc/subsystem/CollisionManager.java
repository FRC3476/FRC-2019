package frc.subsystem;

import frc.robot.Constants;
import frc.utility.Threaded;

public class CollisionManager extends Threaded { 
	Turret turret = Turret.getInstance();
	Arm arm = Arm.getInstance();
	Elevator elevator = Elevator.getInstance();
	BallIntake ballIntake = BallIntake.getInstance();
	
	BallIntake.DeployState requestedState;
	boolean waitingOnElevator = false; 
	boolean waitingOnIntake = false;
	long prevTime;
	
	private static final CollisionManager cm = new CollisionManager();
	
	public static CollisionManager getInstance() {
		return cm;
	}    
	
	public CollisionManager() {
		
	}
	
	public void setIntakeState(BallIntake.DeployState state) {
		requestedState = state;
		ballIntake.setDeployState(BallIntake.DeployState.DEPLOYING);
		elevator.setHeight(elevator.requested);
		waitingOnElevator = true;
	}
	
	@Override
	public void update() {
		if (waitingOnElevator) {
			if (elevator.isSafe()) {
				waitingOnElevator = false;
				prevTime = System.currentTimeMillis();
			} 
			else ;
		}
		if (waitingOnIntake) {
			if (System.currentTimeMillis() - prevTime > Constants.IntakeDeployTime) {
				waitingOnIntake = false;
				ballIntake.setDeployState(requestedState);
			}
		}
	}
}
