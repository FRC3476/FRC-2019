package frc.utility;

import java.io.IOException; 
import java.net.DatagramPacket; 
import java.net.DatagramSocket; 
import java.net.InetAddress;
import java.util.Arrays;
import java.util.Scanner;

import frc.robot.Constants;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class JetsonUDP extends Thread {

	private static final JetsonUDP instance = new JetsonUDP();

  private DatagramSocket socket;
  private InetAddress address;

  private VisionTarget[] target = new VisionTarget[12];

  public static JetsonUDP getInstance() {
    return instance;
  }

  public JetsonUDP() {
    super("bleh");
    try {
      socket = new DatagramSocket(Constants.JetsonPort);
      address = InetAddress.getByName(Constants.JetsonIPv4);
    } catch(Exception e) {
      System.out.println("Failed to intialize UDP socket with Jetson");
    }
    start();
  }

  synchronized public VisionTarget[] getTargets() {
    return target;
  }

  private void recieve() {
    byte[] b = new byte[256];
    DatagramPacket packet = new DatagramPacket(b, b.length);
    try {
    socket.receive(packet);
    } catch(Exception e) {
      System.out.println("Failed to recieve a packet");
    }
    //System.out.println(packet.getLength());
    //String received = new String(packet.getData(), 0, packet.getLength());
    synchronized(this) {
      target = new VisionTarget[(int)(packet.getLength()/16)];
    
    for(int i = 0; i < packet.getLength(); i+=16) 
    {
      byte[] to_be_parsed1 = Arrays.copyOfRange(packet.getData(), i, i+4);
      byte[] to_be_parsed2 = Arrays.copyOfRange(packet.getData(), i+4, i+8);
      byte[] to_be_parsed3 = Arrays.copyOfRange(packet.getData(), i+8, i+12);
      byte[] to_be_parsed4 = Arrays.copyOfRange(packet.getData(), i+12, i+16);

      float f1 = ByteBuffer.wrap(to_be_parsed1).order(ByteOrder.LITTLE_ENDIAN).getFloat();
      float f2 = ByteBuffer.wrap(to_be_parsed2).order(ByteOrder.LITTLE_ENDIAN).getFloat();
      float f3 = ByteBuffer.wrap(to_be_parsed3).order(ByteOrder.LITTLE_ENDIAN).getFloat();
      float f4 = ByteBuffer.wrap(to_be_parsed4).order(ByteOrder.LITTLE_ENDIAN).getFloat();

      target[(int)(i/12)] = new VisionTarget(f1, f2, f3, f4);
     // System.out.println(f1);
    }
  }
  }

    @Override
	public void run() {
      while(true) {

      //System.out.println("receieving");
      recieve();  
      }
      
  }
}
