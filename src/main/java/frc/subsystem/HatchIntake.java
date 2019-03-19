// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.utility.telemetry.TelemetryServer;

import edu.wpi.first.wpilibj.Solenoid;

import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxFrames.SetpointOut;

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

	private double requested;
	
	private HatchIntake() {
		deployMotor = new LazyTalonSRX(Constants.HatchIntakeDeployMotorId);
		intakeMotor = new LazyTalonSRX(Constants.HatchIntakeMotorId);
		deployMotor.setSelectedSensorPosition(0, 0, 10);
		deployMotor.config_kP(0, Constants.kHatchP, Constants.TimeoutMs);
		deployMotor.config_kI(0, Constants.kHatchI, Constants.TimeoutMs);
		deployMotor.config_kD(0, Constants.kHatchD, Constants.TimeoutMs);
		intakeMotor.setInverted(true);
		//deployMotor.configClosedLoopPeakOutput(slotIdx, percentOut)
		setPeriod(Duration.ofMillis(20));
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
			switch (deployState) {
				case STOW:
					setAngle(Constants.HatchStowAngle);
					break;
				case HANDOFF:
					setAngle(Constants.HatchHandoffAngle);
					break;
				case INTAKE:
					setAngle(Constants.HatchIntakeAngle);
					break;
			} 
		}

		switch (deployState) {
			case STOW:
				//telemetryServer.sendString("sIH1", "stow");
				break;
			case HANDOFF:
				//telemetryServer.sendString("sIH1", "handoff");
				break;
			case INTAKE:
				//telemetryServer.sendString("sIH1", "intake");
				break;
		}


	}
	
	// Set the state of the intake
	public void setIntakeState(IntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
			
			switch (intakeState) {
				case INTAKE:
					intakeMotor.set(ControlMode.PercentOutput, Constants.HatchIntakeMotorPower);
					break;
				case EJECT:
					intakeMotor.set(ControlMode.PercentOutput, -Constants.HatchIntakeMotorPower);
					break;
				case OFF:
					intakeMotor.set(ControlMode.PercentOutput, 0);
					break;
			}
		}
		switch (intakeState) {
			case INTAKE:
				//telemetryServer.sendString("sIH2", "intake");
				break;
			case EJECT:
				//telemetryServer.sendString("sIH2", "eject");
				break;
			case OFF:
				//telemetryServer.sendString("sIH2", "off");
				break;
		}
		
	}
	
	// Gets the current draw
	public double getCurrent() {
		return intakeMotor.getOutputCurrent();
	}
	
	synchronized public boolean isFinished() {
		if(Math.abs(getAngle() - requested) < Constants.HatchTargetError) return true;
		else return false;
  	}
  
  	public void setAngle(double angle){
		requested = angle;
		deployMotor.set(ControlMode.Position, angle * Constants.EncoderTicksPerDegree);
	}

	public void setDeploySpeed(double speed) {
		deployMotor.set(ControlMode.PercentOutput, speed);
	}

	public void setSpeed(double speed) {
		intakeMotor.set(ControlMode.PercentOutput, speed);

	}
	public double getAngle() {
		return deployMotor.getSelectedSensorPosition() * Constants.DegreesPerEncoderTick;
	}

	public void setEnc(int pos) {
		deployMotor.setSelectedSensorPosition(pos, 0, 10);
	}
	
	@Override
	public void update() { 
		
	
/*
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
		} */
		//System.out.println(getAngle());
	}
}