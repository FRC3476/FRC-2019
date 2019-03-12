// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.control;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class TakeBackHalfController implements Runnable {

	private double oldErr, Err, gain, outVal, oldOutVal, setpoint, currRPM;
	private PIDSource source;
	private PIDOutput output;
	private boolean enabled, running;

	public TakeBackHalfController(double _gain, PIDSource _input, PIDOutput _output) {
		source = _input;
		output = _output;
		oldErr = 0;
		Err = 0;
		gain = _gain;
		outVal = 0;
		oldOutVal = 0;
		setpoint = 0;
		currRPM = 0;
		enabled = true;
		running = true;
	}

	public void disable() {
		enabled = false;
	}

	public void enable() {
		enabled = true;
	}

	public double getOldOutput() {
		return oldOutVal;
	}

	public double getOutput() {
		return outVal;
	}

	@Override
	public void run() {
		while (running) {
			update();
		}
	}

	public void setGain(double _gain) {
		gain = _gain;
	}

	public void setSetpoint(double _setpoint) {
		if (_setpoint > setpoint) {
			outVal = 1;
		} else {
			outVal = 0;
		}
		setpoint = _setpoint;
		oldOutVal = 1 - (2 * (1 - (_setpoint / 5000)));

	}

	public void stop() {
		running = false;
	}

	public void update() {
		if (enabled) {
			currRPM = source.pidGet();
			oldErr = Err;
			Err = setpoint - currRPM;
			outVal += gain * Err;
			if (outVal > 1) {
				outVal = 1;
			} else if (outVal < 0) {
				outVal = 0;
			}
			if (oldErr < 0 != Err < 0) {
				outVal = (outVal + oldOutVal) / 2;
				oldOutVal = outVal;
			}
			output.pidWrite(outVal);
		}
		//System.out.println(outVal);
	}

}
