// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.utility.telemetry.TelemetryServer;

import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class HatchIntake extends Threaded {

	public enum DeployState {
		STOW, HANDOFF, INTAKE
	}

	public enum IntakeState {
		OFF, INTAKE, EJECT
	}
	
	private static final HatchIntake instance = new HatchIntake();
	
	public static HatchIntake getInstance() {
		return instance;
	}
	
	private TelemetryServer telemetryServer = TelemetryServer.getInstance();
	private LazyTalonSRX deployMotor;
	private LazyTalonSRX intakeMotor;
	private DeployState deployState = DeployState.STOW;
	private IntakeState intakeState = IntakeState.OFF;
	
	private HatchIntake() {
		deployMotor = new LazyTalonSRX(Constants.HatchIntakeDeployMotorId);
		intakeMotor = new LazyTalonSRX(Constants.HatchIntakeMotorId);
	}

	public DeployState getDeployState() {
		return deployState;
	}

	public IntakeState getIntakeState() {
		return intakeState;
	}
	
	// Set the deployment state of the intake
	public void setDeployState(DeployState deployState) {
		synchronized (this) {
			this.deployState = deployState;
		}
	}
	
	// Set the state of the intake
	public void setIntakeState(IntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
		}
	}
	
	// Gets the current draw
	public double getCurrent() {
		return intakeMotor.getOutputCurrent();
	}
	
	public boolean isFinished() {
		return true;
  }
  
  public void setAngle(double angle){
		deployMotor.set(ControlMode.Position, angle * Constants.EncoderTicksPerDegree);
	}
	
	@Override
	public void update() {
		IntakeState intake;
		DeployState deploy;
		synchronized (this) {
			intake = intakeState;
			deploy = deployState;
		}
		
		switch (intake) {
			case INTAKE:
				intakeMotor.set(ControlMode.PercentOutput, Constants.HatchIntakeMotorPower);
				telemetryServer.sendString("sIH2", "intake");
				break;
			case EJECT:
				intakeMotor.set(ControlMode.PercentOutput, -Constants.HatchIntakeMotorPower);
				telemetryServer.sendString("sIH2", "eject");
				break;
			case OFF:
				intakeMotor.set(ControlMode.PercentOutput, 0);
				telemetryServer.sendString("sIH2", "off");
				break;
		}

		switch (deploy) {
			case STOW:
				setAngle(Constants.HatchStowAngle);
				telemetryServer.sendString("sIH1", "stow");
				break;
			case HANDOFF:
				setAngle(Constants.HatchHandoffAngle);
				telemetryServer.sendString("sIH1", "handoff");
				break;
			case INTAKE:
				setAngle(Constants.HatchIntakeAngle);
				telemetryServer.sendString("sIH1", "intake");
				break;
		}
	}
}