// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Turret extends Threaded {
	private LazyTalonSRX turrentTalon = new LazyTalonSRX(Constants.TurretMotorId);
	

	private static final Turret instance = new Turret();

	public static Turret getInstance() {
		return instance;
	}

	private Turret() {
  }

	@Override
	public void update() {
  }
}
