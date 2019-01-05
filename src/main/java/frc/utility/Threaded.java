package org.usfirst.frc.team3476.utility;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;

/**
 * Classes that want to be threaded using ThreadScheduler need to implement this
 * class
 */
public abstract class Threaded implements Runnable {

	private boolean isUpdated = true;
	private boolean isPaused = false;
	private double lastRuntime = 0;
	private long period = Duration.ofMillis(5).toNanos();

	@Override
	public void run() {
		boolean snapPaused;
		synchronized (this) {
			snapPaused = isPaused;
		}
		if (!snapPaused) {
			synchronized (this) {
				isUpdated = false;
			}
			double start = Timer.getFPGATimestamp();
			update();
			synchronized (this) {
				lastRuntime = Timer.getFPGATimestamp() - start;
				isUpdated = true;
			}
		}
	}

	public abstract void update();

	synchronized public boolean isUpdated() {
		return isUpdated;
	}

	synchronized public double getLastRuntime() {
		return lastRuntime;
	}

	synchronized public double getPeriod() {
		return period;
	}

	synchronized public void setPeriod(Duration duration) {
		this.period = duration.getNano();
	}

	synchronized public void pause() {
		isPaused = true;
		isUpdated = false;
	}

	synchronized public void resume() {
		isPaused = false;
		isUpdated = true;
	}
}
