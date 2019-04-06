// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.utility.telemetry.TelemetryServer;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class BallIntake extends Threaded {

	public enum DeployState {
		DEPLOY, STOW, DEPLOYING
	}
	
	public enum IntakeState {
		OFF, INTAKE, EJECT
	}
	
	private static final BallIntake instance = new BallIntake();
	
	public static BallIntake getInstance() {
		return instance;
	}
	
	private TelemetryServer telemetryServer = TelemetryServer.getInstance();
	private Solenoid deploySolenoid;
	private LazyTalonSRX intakeMotor;
	private DeployState deployState = DeployState.STOW;
	private IntakeState intakeState = IntakeState.OFF;

	private double lastDeployCommandTime;
	
	private BallIntake() {
		deploySolenoid = new Solenoid(Constants.BallIntakeSolenoidId);
		intakeMotor = new LazyTalonSRX(Constants.BallIntakeMasterId);
		intakeMotor.setInverted(true);

		intakeMotor.configPeakCurrentLimit(0);
		intakeMotor.configPeakCurrentDuration(0);
		intakeMotor.configContinuousCurrentLimit(25);
		intakeMotor.enableCurrentLimit(true);
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
			lastDeployCommandTime = Timer.getFPGATimestamp();
		}

		switch (deployState) {
			case DEPLOY:
				deploySolenoid.set(true);
				//telemetryServer.sendString("sIB1", "deploy");
				break;
			case STOW:
				deploySolenoid.set(false);
				//telemetryServer.sendString("sIB1", "stow");
				break;
		}
	}
	
	// Set the state of the intake
	public void setIntakeState(IntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
		}

		switch (intakeState) {
			case INTAKE:
				intakeMotor.set(ControlMode.PercentOutput, -Constants.IntakeMotorPowerIntake);
				//telemetryServer.sendString("sIB2", "intake");
				break;
			case EJECT:
				intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeMotorPowerEject);
				//telemetryServer.sendString("sIB2", "eject");
				break;
			case OFF:
				intakeMotor.set(ControlMode.PercentOutput, 0);
				//telemetryServer.sendString("sIB2", "off");
				break;
		}
	}

	public void setSpeed(double speed) {
		intakeMotor.set(ControlMode.PercentOutput, speed);
	}
	// Gets the current draw
	public double getCurrent() {
		return intakeMotor.getOutputCurrent();
	}
	
	public boolean isFinished() {
		if(Timer.getFPGATimestamp() - lastDeployCommandTime >= Constants.BallIntakeDeployTime) return true;
		return false;
	}
	
	@Override
	public void update() {

	}
}