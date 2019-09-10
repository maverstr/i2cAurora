#include "MotionSensorAcq.h"

PS2Mouse mouse(6, 5); //clk, data
PS2Mouse mouse2(3, 2); //clk data for mouse 2

//void toVector(int x, int y, int posz);

float posx = 0;
float posy = 0;
float posy2 = 0;
float posz = 0;

float previousx = posx;
float previousy = posy;
float previousy2 = posy2;
float previousz = posz;

float vector[2];

int x, y, x2, z;

bool rotation = true;
bool polar = false;

Coordinates polarCoord = Coordinates();


void motionSensorSetup() {
  Serial.print("Motion Sensor Setup...");
  mouse.begin();
  mouse2.begin();
  Serial.println("complete!");
}

void motionSensorAcq(int *motionArray) {


  uint8_t stat, stat2;
  mouse.getPosition(stat, x, z);
  mouse2.getPosition(stat2, y, x2);

  previousx = posx;
  previousy = posy;
  previousz = posz;

  posz = previousz + (float)z;
  /*
    if (rotation) {
      toVector(x, y, posz); //rotation of x and y depending on z reference
      x = vector[0];
      y = vector[1];
    }*/
  posx = previousx + (float)x;
  posy = previousy + (float)y;

  Serial.print(posx);
  Serial.print(" ");
  Serial.print(-posy);
  Serial.print(" ");
  Serial.print(posz);
  Serial.print(" ");
  Serial.println("end");

  motionArray[0] = posx;
  motionArray[1] = -posy;
  motionArray[2] = posz;
  /*
    Serial.print("distance parcourue : ");
    Serial.println(PITCH*posx);
  */

  if (polar) {
    //go from cart to polar coordinates
    polarCoord.fromCartesian(x, y);
    /*
      Serial.print("polar coord : ");
      Serial.print(polarCoord.getR());
      Serial.print(" ");
      Serial.println(polarCoord.getAngle() * (360 / (2 * PI)));
      }
    */
  }
}

void toVector(float x, float y, float z) {
  //rotates the vector around the z axis to follow the path
  float rotatedX = (float) (x * cos((z / 13000) * 2 * PI) - y * sin((z / 13000) * 2 * PI)); //one full ball rotation = 13000
  float rotatedY = (float) (x * sin((z / 13000) * 2 * PI) + y * cos((z / 13000) * 2 * PI));
  float rotatedVector[2];
  vector[0] = rotatedX;
  vector[1] = rotatedY;
}


