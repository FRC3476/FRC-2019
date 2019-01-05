package frc.utility;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import frc.utility.control.motion.BezierCurve;

public class TCP extends Threaded {
	private class ConnectionHandler extends Threaded {

		Socket clientSocket;
		InputStream inStream;

		public ConnectionHandler(Socket clientSocket) {
			this.clientSocket = clientSocket;
			try {
				inStream = clientSocket.getInputStream();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		@Override
		public void update() {
			while (clientSocket.isConnected()) {
				byte[] buffer = new byte[2048];
				String rawMessage = "";
				try {
					inStream.read(buffer);
					rawMessage = new String(buffer, "UTF-8");
					JSONObject message = (JSONObject) JSONValue.parse(rawMessage);
					BezierCurve parsedCurve = BezierCurve.parseJson(message);

				} catch (UnsupportedEncodingException e) {
					e.printStackTrace();
				} catch (IOException e) {
					e.printStackTrace();
				}
				JSONObject message = (JSONObject) JSONValue.parse(rawMessage);
				// Do whatever with message
			}
			try {
				inStream.close();
				connections.remove(clientSocket.getInetAddress().getHostName() + clientSocket.getPort());
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	private static final TCP instance = new TCP();

	public static TCP getInstance() {
		return TCP.instance;
	}

	private ExecutorService workers;

	private ServerSocket listener;
	private HashMap<String, Socket> connections;

	private TCP() {
		try {
			listener = new ServerSocket(5800);
		} catch (SocketException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		workers = Executors.newCachedThreadPool();
	}

	@Override
	public void update() {
		try {
			Socket clientSocket = listener.accept();
			connections.put(clientSocket.getInetAddress().getHostName() + clientSocket.getPort(), clientSocket);
			workers.execute(new ConnectionHandler(clientSocket));
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 *
	 * @param addr
	 *            Address to send message to
	 * @param message
	 *            Contents of TCP packets
	 * @param port
	 *            Port to send message over
	 */
	public void send(String addr, String message, int port) {
		if (!connections.containsKey(addr + port)) {
			try {
				Socket conn = new Socket(addr, port);
				connections.put(addr + port, conn);
				workers.execute(new ConnectionHandler(conn));
			} catch (SocketException e) {
				e.printStackTrace();
			} catch (UnknownHostException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		try {
			Socket conn = connections.get(addr + port);
			OutputStream outStream = conn.getOutputStream();
			outStream.write(message.getBytes());
		} catch (UnknownHostException e) {
			e.printStackTrace();
			System.out.println("Host:" + addr + " not found!");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
