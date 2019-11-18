import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.*;

public class LineFollower {
/** The EV3 brick we're controlling */
private static EV3 brick;

/** The motor on the left side of the robot */
private static UnregulatedMotor motorB;

/** The motor on the right side of the robot */
private static UnregulatedMotor motorC;

/** The raw EV3 Color Sensor object */
private static EV3ColorSensor colorSensor;
	        
/** The raw EV3 Ultrasonic Sensor object */
private static EV3UltrasonicSensor ultrasonicSensor;

private static SampleProvider calibrated; // use this to fetch samples
	        
public static void main(String[] args) {
 motorB = new UnregulatedMotor(MotorPort.B);
 motorC = new UnregulatedMotor(MotorPort.C);
 colorSensor = new EV3ColorSensor(SensorPort.S3);
 ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
 move(); // start it running
}
private static void move() {
// only calibrate once, at the start
calibrate();

// so long as we can keep finding the line...
while (findLine()) { followLine(); }
}

private static void calibrate() {

}

private void turn(float degrees) {
...
}
	        
private void findInitialLine() {

	                // Use the "calibrated" variable as your sample source
	                // to call fetchSample() on

	                // Turn using turn(), and take samples while its turning

	                // When you've finished turning, turn back to the location
	                // where you saw the best reading.
	                //
	                // Hint: see the following methods that your motors support:
	                // http://web.suffieldacademy.org/cs/lejos_ev3_api/lejos/robotics/Encoder.html#getTachoCount--
	                // http://web.suffieldacademy.org/cs/lejos_ev3_api/lejos/robotics/Encoder.html#resetTachoCount--
}

private static void followLine(){
 double Kp = 500;
 double Ki = 375; 
 double Kd = 25; 
 double integral = 0;
 double derivative = 0;
 double lastError = 0;
 double dt = ...;
 //double correction = 0; 
 double cTurn;
 double bTurn;
 double offset = 45; 
 double Tp = 50; 
 while(colorSensor.getColorID() == 1){
  double lightValue = colorSensor.getFloodlight();
  double error = lightValue - offset; 
  integral = integral + error * dt; 
  derivative = (error - lastError)/dt; 
  double turn = Kp * offset + Ki * integral + Kd * derivative;
  bTurn = Tp + turn; 
  cTurn = Tp - turn;
  lastError = error; 
  motorB.setPower(new Double(bTurn).intValue());
  motorB.forward();
  motorC.setPower(new Double(cTurn).intValue());
  motorC.forward();
  }
}

private static boolean findLine() {
	float[] sample = new float[calibrated.sampleSize()];
	sample[0] = -1;
	while (sample[0] > 0.5) { calibrated.fetchSample(sample, 0);}
	int i = 0; 
	while(colorSensor.getColorID() != 1) { turn(sample[i]); i++; }
	return true;
}
