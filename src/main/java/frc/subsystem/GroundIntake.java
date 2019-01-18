// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class GroundIntake extends Threaded {

	private static final GroundIntake instance = new GroundIntake();

	public static GroundIntake getInstance() {
		return instance;
	}

	private GroundIntake() {
  }

	@Override
	public void update() {
  }
}
