#include <Arduino_FreeRTOS.h>
#include <queue.h>

/// VARIABELES
short int distanceSensor1Echo = 4;
//short int distanceSensor1Trig = 5;
short int distanceSensor2Echo = 6;
//short int distanceSensor2Trig = 7;
/// END VARIABELES

/// TASKS
void TaskDistanceSensor(void *pvParameters);

/// END TASKS

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  /// CREATE TASKS
  xTaskCreate(
    TaskDistanceSensor
    ,  (const portCHAR *)"DistanceSensor1"  
    ,  128  
    ,  (void*)&distanceSensor1Echo
    ,  2  
    ,  NULL
    );
    
  xTaskCreate(
    TaskDistanceSensor
    ,  (const portCHAR *)"DistanceSensor2"  
    ,  128  
    ,  (void*)&distanceSensor2Echo
    ,  2  
    ,  NULL
    );
}

void loop() {
  // LEEG
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

    /*Serial.print(" De Trig pin: ");
    Serial.print(distanceSensorTrig);
    Serial.print("  De afstand is : ");
    Serial.println(distance);*/
    
    if(distanceSensorEcho == 4){
      // Dit is sensor 1
      Serial.print("Dit is Distance Sensor 1:");
      Serial.println();
      Serial.print("--->De afstand(cm) is: ");
      Serial.println(distance);
    }
    else if(distanceSensorEcho == 6){
      // Dit is sensor 2
      Serial.print("Dit is Distance Sensor 2:");
      Serial.println();
      Serial.print("--->De afstand(cm) is: ");
      Serial.println(distance);
    }
    
 
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }

}
