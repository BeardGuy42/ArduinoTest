/*
  Test for my nano device. I think I will compose a small library of APIs for my components. This will be the beginning of my API for the LED Matrix.
*/
#include <string.h>

#include "GY521.h"

#define MATRIX_DIMENSION  8 // 8x8 LED Matrix
#define GRAVITY_NOISE_THRESHOLD 0.1

class LEDMatrix
{
  public:
  
  const unsigned char CLOCK_PIN_ID = 6;
  const unsigned char DIN_PIN_ID = 4;
  const unsigned char CS_PIN_ID = 5;
  const unsigned char DOUT_PIN_ID = 7;
  
  class LEDState
  {
    public:
    const unsigned char NOOP = 0x00;
    const unsigned char ROW0 = 0x01;
    const unsigned char ROW1 = 0x02;
    const unsigned char ROW2 = 0x03;
    const unsigned char ROW3 = 0x04;
    const unsigned char ROW4 = 0x05;
    const unsigned char ROW5 = 0x06;
    const unsigned char ROW6 = 0x07;
    const unsigned char ROW7 = 0x08;
    const unsigned char DECODE_MODE = 0x09;
    const unsigned char INTENSITY = 0x0a;
    const unsigned char SCAN_LIMIT = 0x0b;
    const unsigned char SHUTDOWN = 0x0c;
    const unsigned char DISPLAY_TEST = 0x0F;

    class LEDRow
    {
      public:
        unsigned char address;
        unsigned char data;

        void Write(LEDMatrix* matrix)
        {
          digitalWrite(matrix->CS_PIN_ID, LOW);

          // Serial.println("START MESSAGE");
          // Serial.println("ADDRESS: " + String(address));
          write(matrix, address);
          // Serial.println("DATA: " + String(data));
          write(matrix, data);
          // Serial.println("END MESSAGE");
          digitalWrite(matrix->CS_PIN_ID, HIGH);
        }
        
      private:

      void write(LEDMatrix* matrix, unsigned char byte)
        {
          
          for(unsigned char i = 0; i < 8; i++)
          {
            digitalWrite(matrix->CLOCK_PIN_ID, LOW);
            digitalWrite(matrix->DIN_PIN_ID, ((byte << i) & 0x80) / 0x80);
            digitalWrite(matrix->CLOCK_PIN_ID, HIGH);
            // Serial.println(String(i) + " of " + String(8) + " == " + String(((byte << i) & 0x80) / 0x80));
          }
        }
    };

    LEDRow rows[0xF+1];

    LEDState()
    {
      // Serial.println("Setting state...");

      rows[NOOP] = { NOOP, B11111111 };
      rows[ROW0] = { ROW0, B11100000 };
      rows[ROW1] = { ROW1, B10010000 };
      rows[ROW2] = { ROW2, B10001000 };
      rows[ROW3] = { ROW3, B10000000 };
      rows[ROW4] = { ROW4, B10000000 };
      rows[ROW5] = { ROW5, B10000000 };
      rows[ROW6] = { ROW6, B10000000 };
      rows[ROW7] = { ROW7, B10000000 };
      rows[DECODE_MODE] = { DECODE_MODE, 0x00 };
      rows[INTENSITY] = { INTENSITY, 0x0F };
      rows[SCAN_LIMIT] = { SCAN_LIMIT, 0x07 };
      rows[SHUTDOWN] = { SHUTDOWN, 0x01 };
      rows[0x0d] = { 0x0d, 0x00 };
      rows[0x0e] = { 0x0e, 0x00 };
      rows[DISPLAY_TEST] = { DISPLAY_TEST, 0x01 };

      // Serial.println("State set.");
    }

    void On(unsigned char x, unsigned char y)
    {
      if(x > 7 || y > 7) return;

      unsigned char yi = (7 - y) + 1;
      unsigned char xi = 7 - x;

      rows[yi].data |= (1 << xi);

      Serial.println("ON: " + String(xi) + ", " + String(yi) + " FROM " + String(x) + ", " + String(y));
    }

    void Off(unsigned char x, unsigned char y)
    {
      if(x > 7 || y > 7) return;
      
      unsigned char yi = (7 - y) + 1;
      unsigned char xi = 7 - x;

      rows[yi].data &= ~(rows[yi].data & (1 << xi));
    }

    void WriteRows(LEDMatrix* matrix)
    {
      unsigned int startTime = millis();
      // Serial.println("Writing state...");

      for(unsigned char i = ROW0 ; i <= ROW7; i++)
      {
        rows[i].Write(matrix);
      }

      // Serial.println("State written: " + String(millis() - startTime) + " ms\n");
    }

    void WriteState(LEDMatrix* matrix)
    {
      for(unsigned char i = DECODE_MODE ; i <= SHUTDOWN; i++)
      {
        rows[i].Write(matrix);
      }
    }

    void WriteBrightness(LEDMatrix* matrix, unsigned char brightness)
    {
      rows[INTENSITY].data = brightness%0x0F;

      // Serial.println("SETTING BRIGHTNESS: " + String(rows[INTENSITY].data));
      rows[INTENSITY].Write(matrix);
    }

    void WriteTest(LEDMatrix* matrix)
    {
      // Serial.println("DISPLAY TEST MESSAGE : " + String(DISPLAY_TEST));
      rows[DISPLAY_TEST].Write(matrix);
    }

    void ClearRows()
    {
      for(unsigned char i = ROW0 ; i <= ROW7; i++)
      {
        rows[i].data = 0x00;
      }
    }

  };
  
  LEDState state;

  LEDMatrix() 
  {
    pinMode(CLOCK_PIN_ID, OUTPUT);
    pinMode(DIN_PIN_ID, OUTPUT);
    pinMode(CS_PIN_ID, OUTPUT);
    pinMode(DOUT_PIN_ID, INPUT);

    delay(50);
    state.WriteState(this);
  }


  // x%MATRIX_DIMENSION, y%MATRIX_DIMENSION
  // flips a bit to 1 in the LEDState
  // on the arduino: the y coordinate is the digit or row. the x coordinate is the x'th bit in the y'th row.
  void On(unsigned char x, unsigned char y)
  {
    state.On(x, y);
  }

// x%MATRIX_DIMENSION, y%MATRIX_DIMENSION
// flips a bit to 0 in the LEDState
  void Off(unsigned char x, unsigned char y)
  {
    state.Off(x, y);
  }

  void Clear()
  {
    state.ClearRows();
  }

  void Brightness(unsigned char brightness)
  {
    state.WriteBrightness(this, brightness);
  }

  void Test()
  {
    state.WriteTest(this);
  }

  void Show()
  {
    //Serial.println("START SHOW ROWS\n");
    state.WriteRows(this);
    //Serial.println("END SHOW ROWS\n");
  }
};

class Vector2
{
public:
  unsigned char x;
  unsigned char y;

  bool operator==(Vector2 a)
  {
    return ((a.x == this->x) && (a.y == this->y));
  }

  bool operator!=(Vector2 a)
  {
    return ((a.x != this->x) || (a.y != this->y));
  }

};

class PixelVector
{
  public:
    static Vector2 Up = { 0, 1 };
    static Vector2 UpRight = { 1, 1 };
    static Vector2 Right = { 1, 0 };
    static Vector2 DownRight = { 1, -1 };
    static Vector2 Down = { 0, -1 };
    static Vector2 DownLeft = { -1, -1 };
    static Vector2 Left = { -1, 0 };
    static Vector2 UpLeft = { -1, 1 };

    static Vector2 Zero = { 0, 0 };
    
    // Note: Order is important here
    static Vector2 Directions[8] = {
      {0, 1},
      {1, 1},
      {1, 0},
      {1, -1},
      {0, -1},
      {-1, -1},
      {-1, 0},
      {-1, 1}
    };

    static Vector2 ToPixelVector(Vector2 g)
    {
      // Normalize g.
      float mag = sqrt(pow(g.x, 2) + pow(g.y, 2));

      if(mag < GRAVITY_NOISE_THRESHOLD) return PixelVector::Zero; 

      g.x /= mag; // TODO: double check that this casts as intended and that the right results come out (if it arises as a problem)
      g.y /= mag;

      if(g.x >= cos(3*3.14/8))
      {
        g.x = 1;
      }
      else if(g.x >= cos(5*3.14/8))
      {
        g.x = 0;
      }
      else
      {
        g.x = -1;
      }

      if(g.y >= sin(3.14/8))
      {
        g.y = 1;
      }
      else if(g.y >= sin(9*3.14/8))
      {
        g.y = 0;
      }
      else
      {
        g.y = -1;
      }

      return g; // TODO: Return g as a pixel vector
    }

    static Vector2* GetDirectionPixelVectors(Vector2 pixelVector)
    {
      // #define LEFT 0
      // #define CENTER 1
      // #define RIGHT 2
      
      Vector2 dirPixelVectors[3] = { 
          PixelVector::Zero,
          PixelVector::Zero,
          PixelVector::Zero
      };

      if(pixelVector != PixelVector::Zero)
      {
        //  Find index in All, then pick (i-1)%8, i, (i+1)%8
        for(int i = 0 ; i < 8; i++)
        {
          if(Directions[i] == pixelVector)
          {
            dirPixelVectors[0] = Directions[(i - 1)%8];
            dirPixelVectors[1] = Directions[i];
            dirPixelVectors[2] = Directions[(i + 1)%8];
          }
        }
      }

      return dirPixelVectors; 
    }
};

// returns accelx and accely value in a vector2.
class GravitySensor2
{
  private:
    GY521 gyroscope;
    Vector2 value;

  public:
    GravitySensor2()
    {
      gyroscope = GY521(0x68);
      while(gyroscope.wakeup() == false)
      {
        Serial.print(millis());
        Serial.println("\tCould not connect to GY521");
        delay(1000);
      }

      gyroscope.setAccelSensitivity(0); // 2g
      gyroscope.setGyroSensitivity(0); // 250 deg / s
      gyroscope.setThrottle();
      gyroscope.axe = 0;
      gyroscope.aye = 0;
      gyroscope.aze = 0;
      gyroscope.gxe = 0;
      gyroscope.gye = 0;
      gyroscope.gze = 0;
    }

    Vector2 Value()
    {
      gyroscope.read();
      value.x = gyroscope.getAccelX();
      value.y = gyroscope.getAccelY();

      return value;
    }
};


class Sand
{
private:

    unsigned char GetBit(unsigned char row, unsigned char bit)
    {
      return ((buffer[row] << bit) & 0x80) / 0x80;
    }

    bool Hot(unsigned char row, unsigned char bit)
    {
      return (bool)(((destinationTempBitmap[row] << bit) & 0x80) / 0x80);
    }

    void On(unsigned char* byte, unsigned char row, unsigned char bit)
    {
      *byte |= (1 << bit);      
    }

    void Off(unsigned char* byte, unsigned char row, unsigned char bit)
    {
      *byte &= ~(*byte & (1 << bit));
    }


  // TODO: Apply rule to 'bit' in 'row'.
    void ApplyRule(Vector2 gravity, unsigned char row, unsigned char bit)
    {
        if(Hot(row, bit)) return;   // if we just did something with this data then dont do something immediately again.

        unsigned char data = GetBit(row, bit);
        
        switch(data)
        {
          case 0:
          // do nothing.
          break;
          case 1:
          Vector2 g = PixelVector::ToPixelVector(gravity);

          if(g == PixelVector::Zero)
          {
            // don't do anything
            return;
          }
          
          Vector2* destinations = PixelVector::GetDirectionPixelVectors(g);

          // check if we can copy directly towards g.
          
          unsigned char destinationRow = row + destinations[1].y;
          unsigned char destinationBit = bit + destinations[1].x;

          // Bounds check destinationRow and destinationBit
          if(destinationRow >= MATRIX_DIMENSION || destinationBit >= MATRIX_DIMENSION) 
            return; // don't do anything, we have an unreachable destination.

          unsigned char destinationData = GetBit(destinationRow, destinationBit);

          switch(destinationData)
          {
            case 0:
            Off(&buffer[row], row, bit);
            On(&buffer[destinationRow], destinationRow, destinationBit);
            On(&destinationTempBitmap[destinationRow], destinationRow, destinationBit);
            
            break;
            case 1:
            // TODO: Try left, then right, otherwise stay
            unsigned char leftDestinationRow = row + destinations[0].y;
            unsigned char leftDestinationBit = bit + destinations[0].x;
            unsigned char rightDestinationRow = row + destinations[2].y;
            unsigned char rightDestinationBit = bit + destinations[2].x;

            if(leftDestinationRow < MATRIX_DIMENSION && leftDestinationBit < MATRIX_DIMENSION)
            {
              unsigned char leftDestinationData = GetBit(leftDestinationRow, leftDestinationBit);
              switch(leftDestinationData)
              {
                case 0:
                  Off(&buffer[row], row, bit);
                  On(&buffer[leftDestinationRow], leftDestinationRow, leftDestinationBit);
                  On(&destinationTempBitmap[leftDestinationRow], leftDestinationRow, leftDestinationBit);
                  
                return;
                break;
                case 1:
                // dont do anything
                break;
              }

            } 

            if(rightDestinationRow < MATRIX_DIMENSION && rightDestinationBit < MATRIX_DIMENSION)
            {
              unsigned char leftDestinationData = GetBit(leftDestinationRow, leftDestinationBit);
              switch(leftDestinationData)
              {
                case 0:
                  Off(&buffer[row], row, bit);
                  On(&buffer[rightDestinationRow], rightDestinationRow, rightDestinationBit);
                  On(&destinationTempBitmap[rightDestinationRow], rightDestinationRow, rightDestinationBit);
                return;
                break;
                case 1:
                // dont do anything
                break;
              }
            } 
            break;
          }
          break;
        }
    }

  public:
  #define MAX_SAND_COUNT 16

    // bitmap tracks data that was just copied in to a destination (1 is hot: data just copied in, 0 is cold: data has been here past a single tick)
    unsigned char destinationTempBitmap[8]; 
    unsigned char buffer[8];
    unsigned char sandCount;

    class SandSpawnTimer
    {
      private:
        int lastSpawnTime;
        int spawnPeriod = 1000000; // every 1 million microseconds (1 per second)

      public:

      void Reset()
      {
        lastSpawnTime = micros();
      }

      bool CanSpawn()
      {
        return (micros - lastSpawnTime) >= spawnPeriod;
      }
    };

    SandSpawnTimer timer;

    Sand()
    {

    }

    void Spawn()
    {
        // TODO: flip 5,7 on based on timer, and if there is a bit already on.
      if(GetBit(7, 5))
      {
        return;
      }

      if(timer.CanSpawn())
      {
        On(&buffer[7], 7, 5);
        On(&destinationTempBitmap[7], 7, 5);
        
        timer.Reset();
      }
    }

    // TODO: walk buffer; apply rules
    void Tick(Vector2 gravity)
    {
      // TODO: clear destinationtempbitmap

      // TODO: walk buffer and apply rule to every bit
      
    }
};

GravitySensor2 gravity;
LEDMatrix led_matrix;
Sand sand;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  led_matrix.Brightness(0x0D);
  led_matrix.Show(); 
}

// the loop routine runs over and over again forever:
void loop() {
  static unsigned char timestep = 0;
  static int lastTime = 0;

  float dt = ((micros() - lastTime) / 1000000);
  lastTime = micros();

  led_matrix.Clear();
  timestep += 1;
  sand.Tick(gravity.Value());

  led_matrix.Show();
}
