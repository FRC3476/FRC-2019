// Copyright 2019 FRC Team 3476 Code Orange
// Adapted from:
// robodashboard - Node.js web dashboard for displaying data from and controlling teleoperated robots
// Copyright 2018 jackw01. Released under the MIT License (see LICENSE for details).
// robodashboard FRC interface v0.1.0

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Handles rx/tx of telemetry data on robot side
 */
public class TelemetryServer {

	private static final TelemetryServer instance = new TelemetryServer(5801);

	public static TelemetryServer getInstance() {
		return instance;
	}

	private DatagramSocket socket;
	
	private TelemetryServer(int port) {
		try {
			socket = new DatagramSocket(port);
		} catch (SocketException e) {
			e.printStackTrace();
		}
	}

	/**
	 *
	 * @param message
	 * 		Message to log and send to dashboard
	 */
	public void sendString(String message) {
		sendString(new TelemetryLogEntry("log ", message));
	}

	/**
	 *
	 * @param dataPoint
	 * 		Data point to send
	 */
	public void sendData(TelemetryDataPoint dataPoint) {
		ByteBuffer sendBuffer;
		long timestamp = System.currentTimeMillis();
		sendBuffer = ByteBuffer.allocate(6 + 4 + 8 * dataPoint.values.length).order(ByteOrder.LITTLE_ENDIAN);
		sendBuffer.putLong(timestamp).position(6);
		sendBuffer.put(dataPoint.key.getBytes());
		for (double value : dataPoint.values) {
			sendBuffer.putDouble(value);
		}
		sendRawData(sendBuffer);
	}

	/**
	 *
	 * @param stringValue
	 * 		String to send
	 */
	public void sendString(TelemetryLogEntry stringValue) {
		ByteBuffer sendBuffer;
		long timestamp = System.currentTimeMillis();
		sendBuffer = ByteBuffer.allocate(6 + 4 + stringValue.value.length() + 1).order(ByteOrder.LITTLE_ENDIAN);
		sendBuffer.putLong(timestamp).position(6);
		sendBuffer.put(stringValue.key.getBytes());
		sendBuffer.put(stringValue.value.getBytes());
		sendRawData(sendBuffer);
	}

	/**
	 *
	 * @param sendBuffer
	 * 		Buffer to send
	 */
	private void sendRawData(ByteBuffer sendBuffer) {
		// bytes 0-5: timestamp
		// bytes 6-9: key string
		// bytes 10+: value
		try {
			DatagramPacket msg = new DatagramPacket(sendBuffer.array(), sendBuffer.position(),
			InetAddress.getByName("127.0.0.1"), 5800);
			socket.send(msg);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
