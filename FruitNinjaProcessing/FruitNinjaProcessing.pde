import processing.serial.*;
Serial myPort;        // The serial port

final int WINDOW_SIZE = 600;
final int MAX_FRUITS = 1;
final float CANVAS_WIDTH = 10;
final int BAUD_RATE = 115200;

int D = 120;
int d = 10;

float xCursor = 0;
float xCursorLast = 0;
float yCursor = 0;
float yCursorLast = 0;
int numIntersectedFruits = 0;
int numIntersectedFruitsLast = 0;
float[] xVect = new float[MAX_FRUITS];
float[] yVect = new float[MAX_FRUITS];
float[] xVectLast = new float[MAX_FRUITS];
float[] yVectLast = new float[MAX_FRUITS];
int[] isCut = new int[MAX_FRUITS];

void settings(){
  // Set the window size
  size(WINDOW_SIZE,WINDOW_SIZE);
}

void setup(){     
  // List all the available serial ports
  printArray(Serial.list());

  myPort = new Serial(this, Serial.list()[2], BAUD_RATE);
  myPort.clear();

  myPort.bufferUntil('\n');
  background(0);
  textSize(25);
}

void draw(){
  background(0);
  stroke(0,0,0);
  strokeWeight(3);
  for(int i=0; i<MAX_FRUITS; i++){
    // Maps so coordinate system of graphics window is in bottom left of window
    float xFruitCanvasPos = map(xVect[i],0.0,CANVAS_WIDTH,0.0,WINDOW_SIZE);
    float yFruitCanvasPos = WINDOW_SIZE-map(yVect[i],0.0,CANVAS_WIDTH,-D/2,WINDOW_SIZE);
    float xCursorCanvasPos = map(xCursor,0.0,CANVAS_WIDTH,0.0,WINDOW_SIZE);
    float yCursorCanvasPos = WINDOW_SIZE-map(yCursor,0.0,CANVAS_WIDTH,-D/2,WINDOW_SIZE);
    // Detect intersection between cursor and fruit
    float xDiff = xCursorCanvasPos - xFruitCanvasPos;
    float yDiff = yCursorCanvasPos - yFruitCanvasPos;
    float cursorDist = sqrt(pow(xDiff,2.0)+pow(yDiff,2.0));
    if(cursorDist < D/2){
      isCut[i] = 1;
    }
    // Reset cut when the fruit exits the window
    if(yFruitCanvasPos>=WINDOW_SIZE+D/4){
      isCut[i] = 0;
    }
    // Only draw the fruit when it is not at the origin
    if(xFruitCanvasPos != 0 && yFruitCanvasPos != 0){
      circle(xFruitCanvasPos,yFruitCanvasPos,D);
      if(isCut[i] == 1){
        line(xFruitCanvasPos-D/2,yFruitCanvasPos+D/2,xFruitCanvasPos+D/2,yFruitCanvasPos-D/2);
        line(xFruitCanvasPos-D/2,yFruitCanvasPos-D/2,xFruitCanvasPos+D/2,yFruitCanvasPos+D/2);
      }
    }
    // Draw cursor
    circle(xCursorCanvasPos,yCursorCanvasPos,d);
    // Draw intersection count
    text(numIntersectedFruits,WINDOW_SIZE-40,WINDOW_SIZE-40);
  }
}

void serialEvent(Serial myPort){
  String inString = myPort.readStringUntil('\n');
  if(inString != null){
    inString = trim(inString);
    // String elements are the x positions of all fruits then the y positions of all fruits
    String stringElements[] = splitTokens(inString,",");
    for(int i = 0; i < MAX_FRUITS*2+3; i = i+1) {
      if(i<MAX_FRUITS){
        xVect[i] = float(stringElements[i]);
        if(Float.isNaN(xVect[i])){
          xVect[i] = xVectLast[i];
        }else{
          xVectLast[i] = xVect[i];
        }
      }else if(i < MAX_FRUITS*2){
        yVect[i-MAX_FRUITS] = float(stringElements[i]);
        if(Float.isNaN(yVect[i-MAX_FRUITS])){
          yVect[i-MAX_FRUITS] = yVectLast[i-MAX_FRUITS];
        }else{
          yVectLast[i-MAX_FRUITS] = yVect[i-MAX_FRUITS];
        }
      }else{
        if(i == MAX_FRUITS*2){
          xCursor = float(stringElements[i]);
          if(Float.isNaN(xCursor)){
            xCursor = xCursorLast;
          }else{
            xCursorLast = xCursor;
          }
        }
        if(i == MAX_FRUITS*2+1){
          yCursor = float(stringElements[i]);
          if(Float.isNaN(yCursor)){
            yCursor = yCursorLast;
          }else{
            yCursorLast = yCursor;
          }
        }
        if(i == MAX_FRUITS*2+2){
          numIntersectedFruits = int(stringElements[i]);
          if(numIntersectedFruits >= 0 && numIntersectedFruits <= MAX_FRUITS){
            numIntersectedFruitsLast = numIntersectedFruits;
          }else{
            numIntersectedFruits = numIntersectedFruitsLast;
          }
        }
      }
    }
  }
}
