/*******************************************************************************
  Copyright (c) 2016, ROBOTIS CO., LTD.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

* * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

* * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

* * Neither the name of ROBOTIS nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Kei */

#include <CAN.h>
/*#include "messageList.h"*/

#define SEND_INTERVAL_MS 1000

uint32_t t_time, id, i;
can_message_t tx_msg, rx_msg;
/*
    typedef struct
    {
      uint32_t id      : Identifier of received message
      uint32_t length  : Length of received message data
      uint8_t  data[8] : Data of received message
      uint8_t  format  : Type of ID
    } can_message_t;

   BAUDRATE :
     CAN_BAUD_125K
     CAN_BAUD_250K
     CAN_BAUD_500K
     CAN_BAUD_1000K

   FORMAT :
     CAN_STD_FORMAT
     CAN_EXT_FORMAT
*/
float tarAcc = 0;
float tarSpe = 0;
float tarPos = 0;

void setup()
{
  Serial.begin(115200);

  Serial.println("============================");
  Serial.println("=== Motor Control Start! ===");
  delay(1000);
  while (CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT) == false)
  {
    Serial.println("CAN open fail!!");
    Serial.println("Init CAN BUS again");
    delay(100);
  }
  Serial.println("CAN BUS init ok!");

  //Initialize pins as necessary
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);

  digitalWrite(BDPIN_PUSH_SW_1, LOW);
  digitalWrite(BDPIN_PUSH_SW_2, LOW);

  id = 0x00;
  CanBus.configFilter(id, 0, CAN_STD_FORMAT);
}

float* ploting()
{
  if (CanBus.avaliableMessage())
  {
    if (CanBus.readMessage(&rx_msg))
    {
      Serial.print("ID : ");
      Serial.print(rx_msg.id, HEX);
      Serial.print(", Length : ");
      Serial.print(rx_msg.length);
      Serial.print(", Data : ");
      for (i = 0; i < rx_msg.length; i++)
      {
        Serial.print(rx_msg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      /*-----------------------------------------------------------------------------*/
      if (rx_msg.data[1] == 0x01)
      {
        Serial.print("time : ");
        Serial.print(t_time);
        Serial.println();
        Serial.print("Actual Torque : ");
        if (rx_msg.data[2] == 0x00)
        {
          Serial.print("+");
        }
        else if (rx_msg.data[2] == 0x01)
        {
          Serial.print("-");
        }
        float actTor = ((float) rx_msg.data[3]) / 32768 * 360;
        Serial.print(actTor);
        Serial.println();
        Serial.print("Actual Speed : ");
        if (rx_msg.data[2] == 0x00)
        {
          Serial.print("+");
        }
        else if (rx_msg.data[2] == 0x01)
        {
          Serial.print("-");
        }
        float actSpe = ((float) rx_msg.data[4] + 256 * rx_msg.data[5]) / 32768 * 360;
        if (actSpe == 720)
        {
          float actSpe = 0;
        }
        Serial.print(actSpe);
        Serial.println();
        Serial.print("Actual Position : ");
        float actPos = ((float) (((unsigned char) rx_msg.data[6]) + 256 * ((unsigned char)rx_msg.data[7]))) / 32768 * 360;
        if (actPos == 360)
        {
          float actPos = 0;
        }
        Serial.print(actPos);
        Serial.println();
        Serial.println();

        float Actuals[3] = {actTor, actSpe, actPos};
        return Actuals;
      }
    }
  }
}

void controling(float tarAcc, float tarSpe, float tarPos)
{
  signed char tarAcc16 = (signed char) ((float) tarAcc / 360 * 32768);
  signed char tarSpe16 = (signed char) ((float) tarSpe / 360 * 32768);
  signed char tarPos16a = (signed char) (((int) tarPos / 360 * 32768) % 256);
  signed char tarPos16b = (signed char) (((float) tarPos / 360 * 32768) / 256);

  tx_msg.id = 0x01;
  tx_msg.format = CAN_STD_FORMAT;
  tx_msg.data[0] = 0xFF;
  tx_msg.data[1] = 0x01;
  tx_msg.data[2] = 0x00;
  tx_msg.data[3] = tarAcc16;
  tx_msg.data[4] = 0x00;
  tx_msg.data[5] = tarSpe16;
  tx_msg.data[6] = 0x00;
  tx_msg.data[7] = 0x00;
  if (digitalRead(BDPIN_PUSH_SW_1) == HIGH)
  {
    tx_msg.data[7] = tx_msg.data[7] + 1;
    if (tx_msg.data[7] == 0xFF)
    {
      tx_msg.data[7] = 0xFE;
      //tx_msg.data[6]=0x00;
    }
  }
  if (digitalRead(BDPIN_PUSH_SW_2) == HIGH)
  {
    tx_msg.data[7] = tx_msg.data[7] - 1;
    if (tx_msg.data[7] == 0x00)
    {
      tx_msg.data[7] = 0x01;
      //tx_msg.data[6]=0xA0;
    }
  }

  tx_msg.length = 0x08;
  CanBus.writeMessage(&tx_msg);
}

void loop()
{
  t_time = (float) millis() / 1000;
  float Start = t_time;
  char len = 0;

  float* Actuals = ploting();
  float actTor = Actuals[0];
  float actSpe = Actuals[1];
  float actPos = Actuals[2];

  float tarAcc = 36;
  float tarSpe = 100;

  controling(tarAcc, tarSpe, tarPos);

  float Final = t_time;
}
