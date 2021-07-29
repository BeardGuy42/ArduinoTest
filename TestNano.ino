/*
  Test for my nano device. I think I will compose a small library of APIs for my components. This will be the beginning of my API for the LED Matrix.
*/
#include <string.h>

#include "GY521.h"

#define USING_SERIAL_MONITOR 0

class LEDMatrix
{
  public:
  const unsigned char MATRIX_DIMENSION = 8; // 8x8 LED Matrix
  
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

  void Draw()
  {
    //Serial.println("START SHOW ROWS\n");
    state.WriteRows(this);
    //Serial.println("END SHOW ROWS\n");
  }


};

class Ball
{
  public:
  float x, y;
  float vx, vy;

  Ball()
  {
    x = 4;
    y = 4;
  }  

  void Tick(float dt)
  {
    x = vx*dt;
    y = vy*dt;

    if(x > 7)
    {
      x = 7;
      vx = 0;
    }
    else if(x < 0)
    {
      x = 0;
      vx = 0;
    }

    if(y > 7)
    {
      y = 7;
      vy = 0;
    } 
    else if(y < 0)
    {
      y = 0;
      vy = 0;
    }
  }

  void Draw(LEDMatrix* matrix)
  {
    Serial.println(String(this->x) + ", " + String(this->y));
    matrix->On((unsigned char)floor(this->x), (unsigned char)floor(this->y));
  }
};

LEDMatrix* led_matrix;
GY521* gyroscope;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  led_matrix = new LEDMatrix();
  led_matrix->Brightness(0x0D);
  led_matrix->Draw();

  gyroscope = new GY521(0x68);
  while(gyroscope->wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521");
    delay(1000);
  }

  gyroscope->setAccelSensitivity(0); // 2g
  gyroscope->setGyroSensitivity(0); // 250 deg / s
  gyroscope->setThrottle();

  gyroscope->axe = 0;
  gyroscope->aye = 0;
  gyroscope->aze = 0;
  gyroscope->gxe = 0;
  gyroscope->gye = 0;
  gyroscope->gze = 0;
}

// the loop routine runs over and over again forever:
void loop() {
  static unsigned char timestep = 0;
  static int lastTime = 0;
  static Ball ball;

  gyroscope->read();
  float ax = gyroscope->getAccelX();
  float ay = gyroscope->getAccelY();
  float t = gyroscope->getTemperature();

  // if (timestep % 100 == 0)
  // {
  led_matrix->Clear();

  Serial.println("\n\tACCELEROMETER\tTEMPERATURE\tVELOCITY");
  Serial.println("\tax\tay\tT\tvx\tvy");

  ball.vx += ax;
  ball.vy += ay;

  if(ball.vx > 2.5)
  {
    ball.vx = 2.5;
  }
  
  if(ball.vy > 2.5)
  {
    ball.vy = 2.5;
  }

  Serial.print(timestep);
  Serial.print('\t');
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.print(t);
  Serial.print('\t');
  Serial.print(ball.vx);
  Serial.print('\t');
  Serial.print(ball.vy);
  Serial.println();
  ball.Tick(((micros() - lastTime) / 1000000));
  lastTime = micros();
  ball.Draw(led_matrix);
  // }


  timestep += 1;

  led_matrix->Draw();

  //  static unsigned char counter = 0;
  // led_matrix->Brightness(counter++);

  // while(counter < 16)
  // {
  //   digitalWrite(led_matrix->CLOCK_PIN_ID, LOW);
  //   delay(1000);
  //   digitalWrite(led_matrix->CLOCK_PIN_ID, HIGH);
  //   Serial.println(String(digitalRead(led_matrix->DOUT_PIN_ID)));
  //   counter++;
  // }


}
