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
import java.time.Duration;

public class JetsonUDP extends Threaded {

	private static final JetsonUDP instance = new JetsonUDP();

  private DatagramSocket socket;
  private InetAddress address;

  private final int packetSize = 16;

  private VisionTarget[] target = null;

  public static JetsonUDP getInstance() {
    return instance;
  }

  public JetsonUDP() {
    //super("bleh");
    try {
      socket = new DatagramSocket(Constants.JetsonPort);
      address = InetAddress.getByName(Constants.JetsonIPv4);
    } catch(Exception e) {
      System.out.println("Failed to intialize UDP socket with Jetson");
    }
    //start();
    setPeriod(Duration.ofMillis(10));
  }

  synchronized public VisionTarget[] getTargets() {
    VisionTarget[] t = target;
    popTargets();
    return t;
  }

  synchronized public void popTargets() {
    target = null;
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
      target = new VisionTarget[(int)(packet.getLength()/packetSize)];
    
    for(int i = 0; i < packet.getLength(); i+=packetSize) 
    {
      byte[] to_be_parsed1 = Arrays.copyOfRange(packet.getData(), i, i+4);
      byte[] to_be_parsed2 = Arrays.copyOfRange(packet.getData(), i+4, i+8);
      byte[] to_be_parsed3 = Arrays.copyOfRange(packet.getData(), i+8, i+12);
      byte[] to_be_parsed4 = Arrays.copyOfRange(packet.getData(), i+12, i+16);

      float f1 = ByteBuffer.wrap(to_be_parsed1).order(ByteOrder.LITTLE_ENDIAN).getFloat();
      float f2 = ByteBuffer.wrap(to_be_parsed2).order(ByteOrder.LITTLE_ENDIAN).getFloat();
      float f3 = ByteBuffer.wrap(to_be_parsed3).order(ByteOrder.LITTLE_ENDIAN).getFloat();
      float f4 = ByteBuffer.wrap(to_be_parsed4).order(ByteOrder.LITTLE_ENDIAN).getFloat();

      target[(int)(i/packetSize)] = new VisionTarget(f1, f2, f3, f4);
     // System.out.println(f1);
    }
  }
  }

  @Override
	public void update() {

      recieve();  
      

  }

  /*
    @Override
	public void run() {
      while(true) {

      //System.out.println("receieving");
      recieve();  
      }

  } */
}
