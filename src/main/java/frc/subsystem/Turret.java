// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Turret extends Threaded {
	private LazyTalonSRX turretTalon;
	private static final Turret instance = new Turret();

	public static Turret getInstance () {
		return instance;
	}

	private Turret () {
		turretTalon = new LazyTalonSRX (Constants.TurretMotorId);
		turretTalon.setSensorPhase (false);
		turretTalon.setInverted (false);
  }

	public void setPercentOutput (double output) {
		turretTalon.set (ControlMode.PercentOutput, output);
	}

	protected void setAngle (double angle) {
		turretTalon.set (ControlMode.Position, angle * Constants.AngleConversionRate2);
	}

	public void setSpeed (double speed) {
		turretTalon.set (ControlMode.Velocity, speed * Constants.AngleConversionRate2);
	}

	public double getSpeed () {
		return turretTalon.getSelectedSensorVelocity (Constants.TurretMotorId) * Constants.AngleConversionRate;
	}

	public double getAngle () {
		return turretTalon.getSelectedSensorPosition (Constants.TurretMotorId) * Constants.AngleConversionRate;
	}

	public double getTargetAngle () {
		return turretTalon.getSetpoint () * Constants.AngleConversionRate;
	}

	public double getOutputCurrent () {
		return turretTalon.getOutputCurrent ();
	}

	@Override
	public void update () {}
}
