// Copyright 2019 FRC Team 3476 Code Orange
// Adapted from:
// robodashboard - Node.js web dashboard for displaying data from and controlling teleoperated robots
// Copyright 2018 jackw01. Released under the MIT License (see LICENSE for details).
// robodashboard FRC interface v0.1.0

/**
 * Storage class for data points
 */
public class TelemetryDataPoint {
	public String key;
	public double[] values;

	public TelemetryDataPoint(String k, double... v) {
		key = k;
		values = v;
	}
}
