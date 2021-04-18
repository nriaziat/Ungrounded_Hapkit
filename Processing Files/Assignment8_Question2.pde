// ME327 Spring 2020
// HW8_P2 Starter code
// Brandon Ritter

PImage racket;
PImage tennisBall;
PImage net;
PImage cone;
import processing.serial.*;

Serial myPort;        // The serial port

//initialize all variables
float inByte = 0; //current value of the first variable in the string
float lastByte = 0; //previous value of the first variable in the string
float inByte2 = 0; //current value of the second variable in the string
float lastByte2 = 0; //previous value of the second variable in the string
float wall = 0; //wall position
float xMass = 0; //mass position
float user = 0; //user position

void setup () {
  // set the window size:
  size(1000, 800); 
  //serena=loadImage("serena.jpg");

  // List all the available serial ports
  //println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  //note you may need to change port number, it is 9 for me
  myPort = new Serial(this, Serial.list()[2], 9600);  // also make sure baud rate matches Arduino
  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  //background(0);      // set inital background:
  
  // Import Picture of Tennis Racket 
  
  racket = loadImage("tennis racket.PNG");
  tennisBall = loadImage("tennis ball.PNG");
  net = loadImage("tennis net.PNG");
  cone = loadImage("cone.PNG");


}

void draw () {
  // everything happens in the serialEvent()
  background(255); //uncomment if you want to control a ball
 
  
  //START EDITING HERE
  //imageMode(CENTER);
  //image(serena,200,200);
  stroke(127,34,255);     //stroke color
  strokeWeight(4);        //stroke wider
  
  
  // Mass Spring Damper
  
  //draw the wall
  //line(wall, 0, wall, height);
  float netWidth = 1276;
  float netHeight = 800;
  float netD = 1;
  image(net, wall-netWidth/2.18, -20, netWidth/netD, netHeight/netD);
  
  //draw a line from the wall to the xMass
  //line(wall, height/2, xMass, height/2);

  float racketWidth = 1276;
  float racketHeight = 800;
  float racketD = 1;
  float coneWidth = 1179;
  float coneHeight = 407;
  float coneD = 1.5;
  
  float tBallY = 130;

  //draw an ellipse to represent the user
  //ellipse(user, height/2, 50, 50);
  image(racket, user-racketWidth/2, 0, racketWidth/racketD, racketHeight/racketD); 
  //image(cone, user-coneWidth/2, tBallY+60, coneWidth/coneD, coneHeight/coneD); 

  
  float tBallW = 1276;
  float tBallH = 800;
  float tBallD = 9;
  //draw an ellipse to represent the mass of the spring-mass-damper
  //ellipse(xMass, height/2, 50, 50);
  image(tennisBall, xMass-tBallW/11, tBallY, tBallW/tBallD, tBallH/tBallD);
  
  

}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  
  //For Mass Spring Damper
  // make sure to match variable names with what has been declared above in lines 10-13 (or change the original variable names if you wish)
  
  // read the first part of the input string
  String String_1 = myPort.readStringUntil(' ');
  
  // trim and convert string to a number
  // if: the number is NaN, set current value to previous value
  if (String_1 != null){
    lastByte = float (String_1);
    inByte = float (String_1);
    println(String_1);
  }
  // otherwise: map the new value to the screen width
  //           & update previous value variable
  else {
    inByte= lastByte ;
  }
  
  // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels 
   

  // repeat for second part of the input string
  String String_2 = myPort.readStringUntil('\n');
  
  if (String_2 != null){
    lastByte2 = float(String_2);
    inByte2 = float (String_2);
    println(String_2);
    
  }
  else {
     inByte2 = lastByte2;
  }
  
  // map the user position
  user = map(inByte, -0.058, 0.042, 0, width);
 
  
  // map the wall position
  wall = map(0.005, -0.058, 0.042, 0, width);
  
  // map the mass position
  xMass = map(inByte2, -0.058, 0.042, 0, width);
  
  //STOP EDITING HERE
  
}
