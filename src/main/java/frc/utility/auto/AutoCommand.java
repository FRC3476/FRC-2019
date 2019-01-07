// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.auto;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class AutoCommand {

	/**
	 * Should never block.
	 */
	public abstract void start();

	public abstract boolean isFinished();

	private boolean isBlocking = false;

	public void run() {
		start();
		if (isBlocking) {
			while (!isFinished() && DriverStation.getInstance().isAutonomous()) {

			}
		}
	}

	public void setBlocking(boolean isBlocking) {
		this.isBlocking = isBlocking;
	}
}
