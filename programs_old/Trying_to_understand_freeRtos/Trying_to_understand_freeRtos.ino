struct knobState {
  int rot;
  int num_of_detent;
};

QueueHandle_t structQueue;
//QueueHandle_t Task2queue;

TaskHandle_t task1;
TaskHandle_t task2;

 
void setup() {

 
  //Serial.begin(115200);

  structQueue = xQueueCreate(10, // Queue length
                              sizeof(struct knobState));
 
  //queue = xQueueCreate( 10, sizeof( int ) );
 
  if(structQueue != NULL){

    /*
    xTaskCreate(Task1, // Task function
                "task1", // A name just for humans
                1000,  // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL, 
                2, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL);

    xTaskCreate(Task2, // Task function
                "task2", // A name just for humans
                1000,  // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL, 
                1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL);
   */

   xTaskCreatePinnedToCore(Task1,"Task1",10000,NULL,1,&task1,0); 
   xTaskCreatePinnedToCore(Task2,"Task2",10000,NULL,1,&task2,1);  

  }
 
}
 
void loop() {
}

void Task1(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    for (int i = 0; i< 100; i++) {
      struct knobState currentState;
      currentState.rot = i;
      currentState.num_of_detent = 0;

      xQueueSend(structQueue, &currentState, portMAX_DELAY);
      vTaskDelay(25);
    }
  }
  vTaskDelete(task1);
  //Serial.println("test1):
  //delay(1000);
}

void Task2(void *pvParameters) {
  (void) pvParameters;

  Serial.begin(115200);

   while (!Serial) {
    vTaskDelay(1);
  }

  for(;;) {
    struct knobState currentState;

    if (xQueueReceive(structQueue, &currentState, portMAX_DELAY) == pdPASS) {
      Serial.println(currentState.rot);
      vTaskDelay(100);
      delay(100);
    }
  }
  vTaskDelete(task2);
  
  //Serial.println("test2):
  //delay(1000);
}
