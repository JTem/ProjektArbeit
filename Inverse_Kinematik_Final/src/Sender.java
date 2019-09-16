import processing.core.PApplet;
import processing.serial.*;

public class Sender {

  Serial myPort;
  String msg;
  PApplet p;
  float[] rq = new float[6];
  public Sender(PApplet papplet) {
    String portName = "COM3";
    p = papplet;
    p.printArray(Serial.list());
    //String portName = Serial.list()[1];
    myPort = new Serial(papplet, portName, 2000000);
  }

  public byte[] normalizeAngleData(float v1, float v2, float v3, float v4, float v5, float v6) {
    String dataS = "S-" + Math.round((Math.toDegrees(v1)*100)+50000) + "-" + Math.round((Math.toDegrees(v2)*100)+50000) + "-" +Math.round((Math.toDegrees(v3)*100)+50000)
      + "-" + Math.round((Math.toDegrees(v4)*100)+50000) + "-" +Math.round((Math.toDegrees(v5)*100)+50000) + "-" + Math.round((Math.toDegrees(v6)*10)+50000) + "-E";
    byte[] data = dataS.getBytes();
    //p.println(dataS);
    return data;
  }
  void printSerialList() {
	   p.printArray(Serial.list());
  }
  void sendData(float v1, float v2, float v3, float v4, float v5, float v6) {
    myPort.write(normalizeAngleData(v1, v2, v3, v4, v5, v6));
    //myPort.write('H');
  }
  void readData() {
    while ( myPort.available() > 0) {  // If data is available,
      char c = myPort.readChar();  // read it and store it in val
      msg+=c;
     // p.println(msg);
      if (msg.length() == 39 && msg.charAt(0) == 'A' && msg.charAt(38) == 'O') {
        
        String sq1 = msg.substring(2, 7);
        String sq2 = msg.substring(8, 13);
        String sq3 = msg.substring(14, 19);
        String sq4 = msg.substring(20, 25);
        String sq5 = msg.substring(26, 31);
        String sq6 = msg.substring(32, 37);

        rq[0] = (Integer.parseInt(sq1) - 50000) * 0.01f;
        rq[1] = (Integer.parseInt(sq2) - 50000) * 0.01f;
        rq[2] = (Integer.parseInt(sq3) - 50000) * 0.01f;
        rq[3] = (Integer.parseInt(sq4) - 50000) * 0.01f;
        rq[4] = (Integer.parseInt(sq5) - 50000) * 0.01f; 
        rq[5] = (Integer.parseInt(sq6) - 50000) * 0.01f;
     
        //p.println(rq[0]);
        msg = "";
      }
      if( msg.length()>= 39){
     
      msg = "";
      }
    }
  }
}