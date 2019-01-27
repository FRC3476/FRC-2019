// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Arm extends Threaded {

	private LazyTalonSRX armTalon;
	protected long homeStartTime;

	private static final Arm instance = new Arm();

	public static Arm getInstance() {
		return instance;
	}

	private Arm() {
		armTalon = new LazyTalonSRX(Constants.ArmId);
		armTalon.setSensorPhase(false);
		armTalon.setInverted(false);
  	}

	public void setPercentOutput(double output) {
		armTalon.set(ControlMode.PercentOutput, output);
	}

	protected void setAngle(double angle) {
		armTalon.set(ControlMode.Position, angle * (1d / 360) *
			Constants.SensorTicksPerMotorRotation);
	}

	public void setSpeed(double speed) {
		armTalon.set(ControlMode.Velocity, speed * (1d / 360) *
			Constants.SensorTicksPerMotorRotation);
	}

	public double getSpeed() {
		return armTalon.getSelectedSensorVelocity(0) * 360 * 
			(1d / Constants.SensorTicksPerMotorRotation);
	}

	public double getAngle() {
		return armTalon.getSelectedSensorPosition(0) * 360 * 
			(1d / Constants.SensorTicksPerMotorRotation);
	}

	public double getTargetAngle() {
		return armTalon.getSetpoint() * 360 * 
			(1d / Constants.SensorTicksPerMotorRotation);
	}

	public double getOutputCurrent() {
		return armTalon.getOutputCurrent();
	}

	public void armHome(){
		while(getOutputCurrent()<Constants.HighArmAmps){
			armTalon.set(ControlMode.PercentOutput, Constants.ArmHomingSpeed);
		}
	}

	@Override 
	public void update () {}
}