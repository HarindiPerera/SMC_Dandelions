
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"
#include <stdio.h>
#include <state.h>

void state_machine() {
    state_t current_state = BOOT;
    char input;

    while (current_state != SHUTDOWN) {
        switch (current_state) {
            case BOOT:
                printf("System is in boot mode.\n");

                printf("System waits for 2 seconds.\n");
                //wait 2 seconds and moves to idle
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                current_state = IDLE;
                break;

            case IDLE:
                printf("System is in idle mode.\n");

                input = getchar();
                if (input == 'e') {
                    current_state = SHUTDOWN;
                    printf("Shutdown\n");
                } else if (input == 't') {
                    printf("Timeout\n");
                } else if (input == 'g') {
                    current_state = EXPERIMENT;
                    printf("Present state: EXPERIMENT\n");
                } else if (input == 'd') {
                    current_state = COMMS;
                    printf("Present state: COMMS\n");
                }
                break;

            case EXPERIMENT:
                printf("System is in experiment mode.\n");
                input = getchar();
                if (input == 'i') {
                    current_state = LOG;
                    printf("Present state: LOG\n");
                } else if (input == 's') {
                    current_state = LOG;
                    printf("Present state: LOG\n");
                } else if (input == 'c') {
                    current_state = NEUTRAL;
                    printf("Present state: NEUTRAL\n");
                } else if (input == 'e') {
                    current_state = SHUTDOWN;
                    printf("Present state: SHUTDOWN\n");
                } else {
                    //wait 3 seconds
                    current_state = NEUTRAL;
                    printf("Present state: NEUTRAL\n");
                }
                break;

            case NEUTRAL:
                printf("System is in neutral mode.\n");
                input = getchar();
                if (input == 'i' || 's' || 'c') {
                    current_state = LOG;
                    printf("Present state: LOG\n");
                } else if (input == 'e') {
                    current_state = SHUTDOWN;
                    printf("Present state: SHUTDOWN\n");
                } else {
                    //wait 3 seconds
                    current_state = LOG;
                    printf("Present state: LOG\n");
                }
                break;

            case LOG:
                printf("System is in neutral mode.\n");
                input = getchar();
                if (input == 'p') {
                    current_state = LOG;
                    printf("Present state: LOG\n");
                } else if (input == 'e') {
                    current_state = SHUTDOWN;
                    printf("Present state: SHUTDOWN\n");
                } else {
                    //wait 3 seconds
                    current_state = IDLE;
                    printf("Present state: IDLE\n");
                }
                break;

            case COMMS:
                printf("System is in comms mode.\n");
                input = getchar();
                if (input == 'e') {
                    current_state = SHUTDOWN;
                    printf("Present state: SHUTDOWN\n");
                } else {
                    //wait 3 seconds
                    current_state = SHUTDOWN;
                }
                break;

            default:
                printf("Invalid state\n");
                break;
        }
    }

    printf("Program ended\n");
}

