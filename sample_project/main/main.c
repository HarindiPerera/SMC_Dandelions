
/*
IN SUMMARY 
Task do not work the way that I thought they did.
They switch rapidly between eachother not sequentially. 
You can treat each task as if it is the only thing running on the ESP32. 
Queues pass data between the tasks, if a queue is empty it will give control back to the scheduler
untill the queue has atleast one item in it. 
Tasks must not be allowed to return from therir implementing funtion in any way.
They must not contain a return

There is documentation somewhere about FreeRTOS that is absolutely GOATED with the sauce. Need to find that again.


NOTES: 

note that th esp32 has the networking run on core 0 and the sketches run on core 1

you can treat each task as if it would run alone on the esp32
because we can think of each task as it's own task on the esp32 we can use the a non-blocking delay 
vTaskDelay() -indicates to the scheduler that this time can be used for other things. 
task yeild gives controll back to the scheduler 

Quese-> pass data between task. 
they are usual FIO with a defined max length 
xQueueReceive() will vTaskDelay forever if the queue is always empty

you can fill a queue using xQueusSend() -> waits when it is full. 
do not have to worry about queue managment. 

what if you want to send multiple points of data between tasks? 
You use structures that will sit in shared memory that you can pass pointers from the queuea

Structure behaives as one variable and can contain as many variables as you want. 
you can read and write the internal data using the mystruct.myinfo format

protection of shared resources. 
-> semaphors and mutexes mutual exclusive.
-> only need to do this if you have shared ites. 

if you use RTOS in Interupt service rutines you have to use slightly different commands. 
remember the xQueueSendFromISR() and the xQueueReceiveFromISR() commands. 

Protection of critical sections. 
portENTER_CRITICAL(&timerMux);
portEXIT_CRITICAL(&timerMux);
or 
taskENRER_CRITICAL(&timerMux);
taskEXIT_CRITICAL(&timerMux);
on ESP32 it doesnt mater which one you pick

-> disable the scheudler and disable the interupts for a given section. 
-> these are called spinlocks
-> keep protected sections short because they block EVERYTHING




TOP LEVEL TASKS 
Tasks can either be running or not running.

// had to go into the sdkconfig file to disable the software watchdog timer 
line 508 onwards 


the ESP RTOS is a slight modification of the OG RTOS
There will be a few things that are special for the ESP32 

So what is happening today? 



We have agreed on a two state machine. 
Wait on user input -> untill key is pressed q = quit/restart, r = run the experiment. 

motor forwar 3. 5 revolutions. 
LC functionality whatever is req    uie to get 1 reading of the maximum deflection data 
Motor back3.5 revolutions. 
LC functionality 
Sg Fdunctionalit 


ok. lets take some notes on queues 
ques hold a finite number of items both length and size of each item are set when the que is created. 
Ques are normallyt FIFO
written to the tail and read from the head. 
They allow tasks to communicate 
task A sends the value of a local variable to the tail of the queue, 
it then changes the value and writes it to the back of the queue. 
there are now two items in the queue. 
Task B reads in the head of the queue
and everthing get shifted towards the queue's head. 
FreRTOS usies the queue by copy method. This means that they will exist on the queue even when their original sack area is cleaned/deleted 

Queues are objects in their own right that can be aceesed bby asn task or ISR that knows tof theire existance. Any number of tasks can write the same que 
and any number of tasks can read from the same queue. 
It is common for a queue to have multiple writers but it is uncommon for a queue to have multiple readers. 

when a task attempts to read from a queue, it can optionally specify a blcok time,.. this is the time that the task will be kept inm the blcoked state.
This allows time for data to become avaliable in the queue if it already empty. 
A task that is in the blocked state, waiting for data to become avalible. is auutomatically moved 
to the ready state when another task or itnerupt places data into the queue. T
The task will also be automatically moved from the blocked tate to the ready state id the specified block time expires before data becomes avaliable.

Quese can have multiple readers, so it is possibble for a single queue to have more than one task blocked on it waiting for data.  
When this is the case. only was task will get unblocked when the data becomes avaliable. 
The task that is unblocked will alwasy be the higher prioority task that is waiting for the data reading from that queue. 
If the blocked taks shave equal priority thgan th task that has been waiting longest will be unblocked. 

Block on Queue Writes. 
Just as when ready frm a queue a task can optional specity a blcok time when writing to a queue. 
in this case the block time is the maximum time that the task should be held in the blocked state waiting for space to become available on the queue. 

Ques can have multiple writers so it is possiblefor a que to have more than one task waiting for it to become avaliable.
The waiting tasks will have the same unblocking priority as the tasks tasks blocked on a read function. 

Queues can be grouped into sets. 
Tasks can be in the blocked state waiting for any one of the queues in the set to become availiable. 

>>>> so what i am thinking is that there will be a run experiment task. 
This task will be blocked untill the r-key is pressed and it adds something to the experiment queue. 
After completion it will put itself back into the wating state. it should have the same or higher priority as the WD pulse task. 
this will make sure that the hardware wd always gets pulsed appropreately. 


========
queues must be create before that can be used.  
quese are referenced by handles which are variable of type queueHandle_t. 
the xQueueCreate function makes a que and returns a handle that references the queue it creates. 


*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"

QueueHandle_t experimentQueue = NULL;

// Standard function to drive the motor in direction dir for ticks number of pulses. 
int RunMotor(bool dir, int ticks){
    esp_err_t err = ESP_OK;                      // create an error variable to check the stuff. 
    if(gpio_set_level(MOTDIR,dir)!=ESP_OK){
        printf("ERROR: Error in setting the motor direction");
        return HW_FAULT;                        // return a HW fault condition if unable to set direction
    }
    for(int i =0; i<=ticks; i++){

        err = gpio_set_level(MOTSTEP,1);
        vTaskDelay(5/portTICK_PERIOD_MS);
        err = gpio_set_level(MOTSTEP,0);
        vTaskDelay(5/portTICK_PERIOD_MS);

        if(err!=ESP_OK){
            printf("ERROR: Error in setting toggeling the MOTSTEP pin. Breaking from loop.\n");
            return HW_FAULT;        // return fault if unable to toggle MOTSTEP pin
        }  
    }
    return SUCCESS;                 // ELSE return success after motor has been pulsed 'ticks' number of times. 
}

// Task to feed the watchdog every 5 seconds. 
void hwWDPulseTask(void* pvParamemters){
   
    for(;;){
        printf("Watchdog fed\n");
        gpio_set_level(WDI,1);                  // set the hardware watchdog high
        vTaskDelay(10/portTICK_PERIOD_MS);      // settling time
        gpio_set_level(WDI,0);                  // Set the HW_watchdog low
        vTaskDelay(5000/portTICK_PERIOD_MS);    // give controll back to the scheduler for 5 seconds
    }

    vTaskDelete(NULL);                                                          // delete task if it breaks for some reason
    printf("CRITICAL ERROR: hwWDPulseTask broke from loop. Task Deleted\n");    // error msg
}

void keyBoardTask(void * pvParameters){
    char c  = 'p';
    BaseType_t xStatus; // error checking 

    for(;;){
        c = getchar();
        if(c == 'q'){
            // [TODO]-> command all the computational power here when entering restart condition
            // ..... make this protected
            printf("RESTARTING ESP... \n");
            vTaskDelay(2000/portTICK_PERIOD_MS);
            esp_restart();
            // ... make above protected.

        }else if (c == 'r'){
            xStatus = xQueueSendToBack(experimentQueue, &c, portMAX_DELAY);   // send a pointer to the value of c to experiment queue
            if(xStatus != pdPASS){
                printf("IDLE:   Could not send to experiment queue\n");
            }
        }
    }
}



void experimentTask(void*pvParameters){
    BaseType_t xStatus;
    char c  = 'p';   // receive value of the que information. 
    for(;;){
        printf("the Experiment task is blocked after the line\n");    // debug
        xStatus = xQueueReceive(experimentQueue, &c, portMAX_DELAY);    // This is one of the blocking functions
        printf("ther experiment task is there\n");                      // debug

        if (xStatus == pdPASS){ // [TODO] -> figure out what the flop pdPASS is
            // data was sucsessfullt received from the queue
            printf("RUN:    Data received from the queue is : %c\n",c);
        /*    
            // this is where we would run the experiment. 
            printf("RUN:    Actuating up\n");  
            RunMotor(1,TICKS_PER_REV*3.5);  // Actuate the motor forwards
            // Take measurement
            printf("RUN:    Actuating down\n");
            RunMotor(0,TICKS_PER_REV*3.5);  // Actuate the motor backwards. 
            // take measurement.  
        */


        }else{
            printf("RUN:    ERROR data could not be recieved from the quque\n");
        }

        printf("RUN:    Returning to idel state. \n");
        printf("IDLE:   Press 'q' to restart and 'r' to run the experiment\n"); 
    }

}


// this is the entry point for the state machine
void app_main(void)
{
    if(setupHW()!= ESP_OK){
        printf("SETUP ERROR: Restarting...\n");
        vTaskDelay(2000/portTICK_PERIOD_MS);
        esp_restart();
    }

     // [TODO] queue and base task setup

    experimentQueue = xQueueCreate( 1,  1); 

    xTaskCreatePinnedToCore(hwWDPulseTask, /*Task Name*/ "HWWATCHDOG", /*stackdepth*/ 1024, /*pvParameters*/ NULL,  /*Priority*/ 1, /*ret handel*/NULL, /*core*/0);                
    xTaskCreatePinnedToCore(keyBoardTask, "CHECK_q",1024,NULL,1,NULL,0);
    xTaskCreatePinnedToCore(experimentTask,"Experiment",4048,NULL,1,NULL,0);

    printf("IDLE:   Press 'q' to restart and 'r' to run the experiment\n");    
    for(;;) 
    {  
        vTaskDelay(1000/portTICK_PERIOD_MS); // do nothing in the main loop 
    }
}
