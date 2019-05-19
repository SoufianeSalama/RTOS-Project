#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <SoftwareSerial.h>
QueueHandle_t queue;

/// VARIABELES
short int distanceSensor1Echo = 4;
//short int distanceSensor1Trig = 5;
short int distanceSensor2Echo = 6;
//short int distanceSensor2Trig = 7;
short int distanceWarningThreshold = 15; 
short int distanceAlarmThreshold = 10; 
short int gasWarningThreshold = 130; 
short int gasAlarmThreshold = 150;
short int Motor1 = 8;
struct alarmMessage{
  int sensor; //1=distancesensor1, 2=distancesensor2, 3=gassensor
  int statusmessage;// 1=ok, 2=alarm
};

SoftwareSerial EEBlue(12, 13); // RX | TX
/// END VARIABELES

/// TASKS
void TaskDistanceSensor(void *pvParameters);
void TaskGasSensor(void *pvParameters);
void TaskMotors(void *pvParameters);
/// END TASKS

void setup() {
  Serial.begin(9600);
  EEBlue.begin(9600); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  
  queue = xQueueCreate(1,sizeof(alarmMessage));
  if(queue == NULL){
    Serial.println("Error creating the queue");
  }

  /// CREATE TASKS
  xTaskCreate(
    TaskDistanceSensor
    ,  (const portCHAR *)"DistanceSensor1"  
    ,  90//128  
    ,  (void*)&distanceSensor1Echo
    ,  2  
    ,  NULL
    );
    
  xTaskCreate(
    TaskDistanceSensor
    ,  (const portCHAR *)"DistanceSensor2"  
    ,  90//128  
    ,  (void*)&distanceSensor2Echo
    ,  2  
    ,  NULL
    );

    xTaskCreate(
    TaskGasSensor
    ,  (const portCHAR *)"GasSensor"  
    ,  90//128  
    ,  NULL
    ,  1  
    ,  NULL
    );
    
  xTaskCreate(
    TaskMotors
    ,  (const portCHAR *)"Motors"  
    ,  105//128  
    ,  (void*)&Motor1
    ,  2  
    ,  NULL
    );
}

void loop() {
  // LEEG
}
void TaskMotors(void *pvParameters){
  (void) pvParameters;
  short int motor1 = (*((short int*)pvParameters));
  short int motor2 = motor1 + 1;
  short int motor3 = motor1 + 2;
  short int motor4 = motor1 + 3;
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT); 
  pinMode(motor4, OUTPUT);
  int blinkduration = 1000; //1sec
  short int distance1Flag;
  short int distance2Flag = 0;
  
  alarmMessage element;
  for (;;)
  {
    xQueueReceive(queue, &element, portMAX_DELAY);
    /*Serial.print("Message from sensor: ");
    Serial.print(element.sensor);
    Serial.print(" with message: ");
    Serial.print(element.statusmessage);
    Serial.println();*/
      if (element.sensor == 1 && element.statusmessage==2){
        // Distance Sensor 1 heeft object gedetecteerd ALARM
        // Vlieg andere richting -> Motor 3 en 4 op HIGH Speed
        distance1Flag = 1;
        /*Serial.print("Distance flag 1 is HOOG");
        Serial.println();*/
      }
      if (element.sensor == 2 && element.statusmessage==2){
        // Distance Sensor 2 heeft object gedetecteerd
        // Vlieg andere richting -> Motor 1 en 2 op HIGH Speed
        distance2Flag = 1;
        /*Serial.print("Distance flag 2 is HOOG");
        Serial.println();*/
      }
      if (element.sensor == 1 && element.statusmessage == 1){
        distance1Flag = 0;
        /*Serial.print("Distance flag 1 is LAAG");
        Serial.println();*/
      }
      if (element.sensor == 2 && element.statusmessage == 1){
        distance2Flag = 0;
        /*Serial.print("Distance flag 2 is LAAG");
        Serial.println();*/
      }

      if (distance1Flag == 0 && distance2Flag == 0){
        digitalWrite(motor1, HIGH);
        digitalWrite(motor2, HIGH);
        digitalWrite(motor3, HIGH);
        digitalWrite(motor4, HIGH);
        vTaskDelay( 250 / portTICK_PERIOD_MS );
        digitalWrite(motor1, LOW);
        digitalWrite(motor2, LOW);
        digitalWrite(motor3, LOW);
        digitalWrite(motor4, LOW);
        vTaskDelay( 250 / portTICK_PERIOD_MS );
      }
      else if(distance1Flag == 1 && distance2Flag == 0){
        digitalWrite(motor1, LOW);
        digitalWrite(motor2, LOW);
        digitalWrite(motor3, HIGH);
        digitalWrite(motor4, HIGH);
      }
      else if(distance2Flag == 1 && distance1Flag == 0){
        digitalWrite(motor1, HIGH);
        digitalWrite(motor2, HIGH);
        digitalWrite(motor3, LOW);
        digitalWrite(motor4, LOW);
      }
      else if (distance1Flag == 1 && distance2Flag == 1){
        digitalWrite(motor1, HIGH);
        digitalWrite(motor2, HIGH);
        digitalWrite(motor3, HIGH);
        digitalWrite(motor4, HIGH);
        vTaskDelay( 100 / portTICK_PERIOD_MS );
        digitalWrite(motor1, LOW);
        digitalWrite(motor2, LOW);
        digitalWrite(motor3, LOW);
        digitalWrite(motor4, LOW);
        vTaskDelay( 100 / portTICK_PERIOD_MS );
      }
  }
  
}

void TaskDistanceSensor(void *pvParameters){
  (void) pvParameters;
  
  long duration, distance;
  short int distanceSensorEcho = (*((short int*)pvParameters)); // ECHO PIN zit op 4 (Sensor1) of 6 (Sensor2)
  short int distanceSensorTrig = distanceSensorEcho + 1;      // TRIG PIN zit op 5 (Sensor1) of 7 (Sensor2)
  pinMode(distanceSensorTrig, OUTPUT);
  pinMode(distanceSensorEcho, INPUT);
  
  for (;;)
  {
    digitalWrite(distanceSensorTrig, LOW);  // Added this line
    //vTaskDelay( (2/1000) / portTICK_PERIOD_MS ); // wait for one second
    delayMicroseconds(2); // Added this line
    digitalWrite(distanceSensorTrig, HIGH);
    //vTaskDelay( (10/1000) / portTICK_PERIOD_MS ); // wait for one second
    delayMicroseconds(10); // Added this line
    digitalWrite(distanceSensorTrig, LOW);
    duration = pulseIn(distanceSensorEcho, HIGH);
    distance = (duration/2) / 29.1;

    if(distanceSensorEcho == 4){
      // Dit is sensor 1
      /*Serial.print("Dit is Distance Sensor 1");
      Serial.print("--->De afstand(cm) is: ");
      Serial.println(distance);*/
      /*if (distance<distanceWarningThreshold && distance>distanceAlarmThreshold){
        Serial.print("Distance Sensor 1 -> WARNING");
        Serial.println();
        alarmMessage am = { 1, 2};
        xQueueSend(queue, &am, portMAX_DELAY);
      }*/
      if(distance<=distanceAlarmThreshold){
        Serial.print("Distance Sensor 1 -> ALARM");
        Serial.println();
        alarmMessage am = { 1, 2};
        xQueueSend(queue, &am, portMAX_DELAY);
      }
      else if(distance>distanceAlarmThreshold){
        // Distance ok
        alarmMessage am = { 1, 1};
        xQueueSend(queue, &am, portMAX_DELAY);
      }
    }
    else if(distanceSensorEcho == 6){
      // Dit is sensor 2
      /*Serial.print("Dit is Distance Sensor 2");
      Serial.print("--->De afstand(cm) is: ");
      Serial.println(distance);*/
      /*if (distance<distanceWarningThreshold && distance>distanceAlarmThreshold){
        Serial.print("Distance Sensor 2 -> WARNING");
        Serial.println();
        alarmMessage am = { 2, 2};
        xQueueSend(queue, &am, portMAX_DELAY);
      }*/
      if(distance<distanceAlarmThreshold){
        Serial.print("Distance Sensor 2 -> ALARM");
        Serial.println();
        alarmMessage am = { 2, 2};
        xQueueSend(queue, &am, portMAX_DELAY);
      }
      else if(distance>distanceAlarmThreshold){
        // Distance ok
        alarmMessage am = { 2, 1};
        xQueueSend(queue, &am, portMAX_DELAY);
      }
    }
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }

}

void TaskGasSensor(void *pvParameters)  
{
  (void) pvParameters;
  for (;;)
  {
    int gasValue = analogRead(A0);
    /*Serial.print("Dit is Gas Sensor ");
    Serial.print("--->De waarde is: ");
    Serial.println(gasValue);*/
    if (gasValue>gasWarningThreshold && gasValue<gasAlarmThreshold){
      Serial.print("Gas Sensor -> WARNING");
      Serial.println();
    }
    else if(gasValue>gasAlarmThreshold){
      Serial.print("Gas Sensor -> ALARM");
      Serial.println();

      EEBlue.write("Gas detected\n");
    }
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}
