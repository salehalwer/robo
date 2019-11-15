

public class LineFollower {


            /** The EV3 brick we're controlling */

       private static EV3 brick;


       /** The motor on the left side of the robot */

       private static RegulatedMotor motorB;


       /** The motor on the right side of the robot */

       private static RegulatedMotor motorC;


       /** The raw EV3 Color Sensor object */

       private static EV3ColorSensor colorSensor;

       


       /** The raw EV3 Ultrasonic Sensor object */

       private static EV3UltrasonicSensor ultrasonicSensor;


       private static SampleProvider calibrated; // use this to fetch samples

       

       public static void main(String[] args) {

               motorB = new UnregulatedMotor(SerialPort.);

               motorC = new UnregulatedMotor(SerialPort.);

               colorSensor = new EV3ColorSensor(SerialPort.);

               ultrasonicSensor = new EV3UltrasonicSenaor(SerialPort.)

               move(); // start it running

       }

       private static void go() {

               // only calibrate once, at the start

               calibrate();


               // so long as we can keep finding the line

               while (findLine()) {

                       // go forward until the line is lost

                       followLine();

               }

       }


       private void calibrate() {

               // clear the screen

               brick.getTextLCD().clear();


               brick.getTextLCD().drawString("Calibrate:", 0, 0);

               brick.getTextLCD().drawString("Place the robot", 0, 1);

               brick.getTextLCD().drawString("on or near the", 0, 2);

               brick.getTextLCD().drawString("line start.", 0, 3);

               brick.getTextLCD().drawString("Then press the", 0, 5);

               brick.getTextLCD().drawString("Enter key", 0, 6);


               brick.getKey("Enter").waitForPressAndRelease();


               // put the sensor in "red" mode to shine a light and

               // read back a brightness value

               //

               // The two numbers are the min and max scaled values

               // you want to receive (you can make them whatever you want

               // but the default is to return numbers between 0 and 1).

               //

               // The boolean determines if the filter "clamps" values out

               // of range to the minimum and maximum provided.  If set to

               // true, you will always get values in the range you specify.

               // If false, a poor calibration may result in readings coming

               // back outside of range.

               NormalizationFilter nf = new NormalizationFilter

                               (colorSensor.getRedMode(), 0, 1, true);


               calibrated = nf;


               // signal the filter to start the calibration process

               nf.startCalibration();


               findInitialLine();


               // end the collection of calibration data

               nf.stopCalibration();


               brick.getTextLCD().clear();

       }


       private static void turn(int degrees) {

               motorB.turn(degrees);

               motorC.turn(degrees);

       }

       

       private static void findInitialLine() {

               // You will need to complete this method in order to

               // calibrate the sensor and get your robot in the correct

               // starting position


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

           double Kp = …;


	

 double Ki = …; 


	

 double Kd = …; 


	

 int error = 0;


	

 int integral = 0;


	

 int derivative = 0;


	

int lastError = 0;


	

double correction = 0;



	

int color;


	

double cTurn;


	

double bTurn;

//threshold = (LFPIDController.lvh.getBlack() + LFPIDController.lvh


	

.getWhite()) / 2;


while(colorSensor.getColorID() == 1){

color = colorSensor.getFloodLight(); / getAmbientMode(); 

     error = color - threshold;


	

integral = error + integral;


	

derivative = error - lastError;


	



	

correction = kp * error + ki * integral + kd * derivative;


	



	

bTurn = 20 - correction;


	

cTurn = 20 + correction;

mB.setPower(new Double(bTurn).intValue());


	

mB.forward();


	

mC.setPower(new Double(cTurn).intValue());


	

mC.forward();


	



	

lastError = error;



}


       }




       private static boolean findLine() {

               float[] sample = new float[calibrated.sampleSize()];

               sample[0] = -1;

               while (sample[0] > 0.5) {

                   calibrated.fetchSample(sample, 0);

               }

               int i = 0; 

while(colorSensor.getColorID() != 1) {

turn(sample[i]); i++; 

                }


       }



}


}
