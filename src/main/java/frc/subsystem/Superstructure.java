// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Superstructure extends Threaded {

	private static final Superstructure instance = new Superstructure();

	public static Superstructure getInstance() {
		return instance;
	}

	private Superstructure() {
  }

	@Override
	public void update() {
  }
}
