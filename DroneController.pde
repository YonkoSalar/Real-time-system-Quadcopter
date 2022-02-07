import org.gamecontrolplus.gui.*;
import org.gamecontrolplus.*;
import net.java.games.input.*;
import processing.serial.*;

ControlIO control;
ControlDevice stick;
float rx, ry, lx, ly;
boolean button_x;
float m1, m2, m3, m4;
float throttle = 0;
float yaw = 0;
Serial myPort;
boolean active = false;

public void setup() {
  size(400, 400);
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
  delay(1000);
  frameRate(30);

  surface.setTitle("Drone Controller");
  // Initialise the ControlIO
  control = ControlIO.getInstance(this);
  // Find a joystick that matches the configuration file. To match with any 
  // connected device remove the call to filter.

  println(control.deviceListToText(", "));

  stick = control.filter(~GCP.STICK).getMatchedDevice("DroneController");
  if (stick == null) {
    println("No suitable device configured");
    System.exit(-1); // End the program NOW!
  }
  println("Name is " + stick.getName());
  stick.getButton("X_BUTTON").plug(this, "buttonToggle", ControlIO.ON_PRESS);
}


// Poll for user input called from the draw() method.
public void getUserInput() {
  rx = map(stick.getSlider("RX").getValue(), -1, 1, 0, 65535);
  ry = map(stick.getSlider("RY").getValue(), 1, -1, 65535, 0);
  lx = map(stick.getSlider("LX").getValue(), -1, 1, 0, 65535);
  ly = map(stick.getSlider("LY").getValue(), -1, 1, 1, -1);
}

public void buttonToggle() {
  active = !active;
}

//Throttle Direction
public void Throttle() {
  if (ly <= 0.05 && ly >= -0.05) {
    ly = 0;
  }

  throttle += ly*1024;

  if (throttle > 65535) throttle = 65535;
  else if (throttle < 0) throttle = 0;
}



public void draw() {
  getUserInput(); // Polling the input device
  Throttle();

  if (active) {
    myPort.write(byte(170));
    myPort.write(byte((int)throttle >> 8));
    myPort.write(byte(throttle));
    myPort.write(byte((int)lx >> 8));
    myPort.write(byte(lx));
    myPort.write(byte((int)rx >> 8));
    myPort.write(byte(rx));
    myPort.write(byte((int)ry >> 8));
    myPort.write(byte(ry));

    println("Throttle:", throttle);
    println("Yaw: ", lx);
    println("Roll: ", rx);
    println("Pitch: ", ry);
  }

  background(255, 255, 255, 255);
  stroke(255, 0, 0);
  line(0, height/2, width, height/2);
  stroke(0, 255, 0);
  line(width/2, 0, width/2, height);
  noStroke();
  fill(255, 64, 64, 128);
  ellipse(rx*width/65535, ry*height/65535, 20, 20);
  fill(64, 64, 255, 128);
  ellipse(lx*width/65535, -throttle*height/65535 + height, 20, 20);

  if (!active) {
    textSize(20);
    fill(0, 0, 0);
    for (int x = -1; x < 2; x++) {
      text("Sender ikke!", 20+x, 30);
      text("Sender ikke!", 20, 30+x);
    }
    fill(255, 0, 0);
    text("Sender ikke!", 20, 30);
  } else {
    textSize(20);
    fill(0, 0, 0);
    for (int x = -1; x < 2; x++) {
      text("Sender!", 20+x, 30);
      text("Sender!", 20, 30+x);
    }
    fill(0, 255, 0);
    text("Sender!", 20, 30);
  }
}
