// Copyright 2019 FRC Team 3476 Code Orange
// Adapted from:
// robodashboard - Node.js web dashboard for displaying data from and controlling teleoperated robots
// Copyright 2018 jackw01. Released under the MIT License (see LICENSE for details).
// robodashboard FRC interface v0.1.0

/**
 * Storage class for data strings
 */
public class TelemetryLogEntry<T> {
	public String key;
	public String value;

	public TelemetryLogEntry(String k, String v) {
		key = k;
		value = v;
	}
}
