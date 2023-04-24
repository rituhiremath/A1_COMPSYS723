#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sys/alt_irq.h"
#include "system.h"
#include "io.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/timers.h"
#include "FreeRTOS/semphr.h"

#define CRITICAL_PERIOD_POLL 500
#define SWITCH_POLLING_PERIOD_MS 100
#define QUEUE_SWITCH_LENGTH 10
#define QUEUE_LOADS_LENGTH 6
#define NUM_LOADS 5
//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)
#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0
#define FREQ_THRESHOLD_INC_AMOUNT 0.5
#define ROC_THRESHOLD_INC_AMOUNT 1

// Handlers
TaskHandle_t VGAdisplay;
TaskHandle_t keyboard_update;
TaskHandle_t freq_update;
TimerHandle_t timer;

// Semaphores
SemaphoreHandle_t sem_freqroc;
SemaphoreHandle_t sem_thresh;
SemaphoreHandle_t sem_block;

// global
static QueueHandle_t Q_freq_data;
static QueueHandle_t Q_time_stamp;
static QueueHandle_t Q_keyb_data;

volatile unsigned char timer_expired = 0;
int stability_status = 0;      // 0 = stable, 1 = unstable
double freq_threshold = 49.5;
double roc_threshold = 10.0;
double freq[100];
double dfreq[100];
int freq_value = 99;
int freq_value_new = 98;
TickType_t time_stamps[100] = {0};

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

// used to delete a task
TaskHandle_t xHandle;

// Timers
TickType_t timerOneStart;
TickType_t timerOneEnd;
TickType_t timerTwoEnd;
TickType_t currentTime;
TimerHandle_t timerTwo;

// Queue
QueueHandle_t switch_queue;
QueueHandle_t loadsShedQueue;
//static QueueHandle_t timer_queue;

// Definition of Semaphore
SemaphoreHandle_t stabilityStatus;
SemaphoreHandle_t maintenanceModeSemaphore;
SemaphoreHandle_t xQueueSwitchSemaphore;

// global variables
volatile unsigned int uiSwitchValue;
volatile unsigned int uiSwitchValuePrevious;
volatile int loadsShed = 0;
enum state {stableState, unstableState, monitorNetworkState, maintenanceState};
typedef enum state State;
static volatile State currentState = stableState;
static volatile State nextState = stableState;

// Local Function Prototypes
void vLoadSheddingTask(void *pvParameters);
void switch_ISR(void* context, alt_u32 id);
void vSwitchPollingTask(void *pvParameters);
void button_ISR(void* context, alt_u32 id);

void vLoadSheddingTask(void *pvParameters)
{
   if ((freq[freq_value_new] < freq_threshold) || (fabs(dfreq[freq_value_new]) > roc_threshold)) {
   	stability_status = 1;
   }
   else{
   	stability_status = 0;
   }
   while(1) {
        switch (currentState) {
            case stableState:
                if (!stability_status) {
                    timerOneStart = xTaskGetTickCount();
                    xTimerStart(timerTwo, timerOneStart);
                    nextState = unstableState;
                } else {
                    nextState = stableState;
                }
                break;
            case unstableState:
                // Determine which load to reconnect based on priority and whether it was previously shed.
                for (int i = 0; i < NUM_LOADS; i++) {
                    // find the first load that is ON
                    if (IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) & (1 << i)) {
                        // switch it off by turning red OFF
                        IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) & ~(1 << i));
                        // switch on green LED
                        IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE) | (1 << i));
                        // Send the shed load index to the load shed queue
                        if (!xQueuePeek(loadsShedQueue, &i, 0)) {
                            xQueueSendToFront(loadsShedQueue, &i, 0);
                        }
                        break;
                    } else {
                        // Load is OFF
                        if (!xQueuePeek(loadsShedQueue, &i, 0)) {
                            xQueueSendToFront(loadsShedQueue, &i, 0);
                        }
                    }
                    timerOneEnd = xTaskGetTickCount();
                }
                loadsShed++;
                nextState = monitorNetworkState;
                break;
            case monitorNetworkState:
                currentTime = xTaskGetTickCount();
                // Check if 500ms has elapsed since TimerOne started
                if (xTimerIsTimerActive(timerTwo) && (currentTime - timerOneStart >= pdMS_TO_TICKS(500))) {
                    if (stabilityStatus) {
                        // Reconnect any previously shed loads
                        while (xQueueReceive(loadsShedQueue, &loadsShed, 0) == pdTRUE) {
                            // switch on the previously shed load
                            IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) | (1 << loadsShed));
                            // switch off the green LED
                            IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE) & ~(1 << loadsShed));
                        }
                        // Reset TimerTwo and update the start time of TimerOne
                        xTimerReset(timerTwo, 0);
                        timerOneStart = currentTime;
                        // Transition to stable state
                        nextState = stableState;
                        // if unstable for 500ms
                        } else {
                            // Reset TimerTwo and update the start time of TimerOne
                            xTimerReset(timerTwo, 0);
                            timerOneStart = currentTime;
                            // Transition to unstable state
                            nextState = unstableState;
                        }
                }
                break;

            case maintenanceState:
                // turn off all the green LEDs
                IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE) & 0xFF);
                break;
        }
        currentState = nextState;
    }
}

void vSwitchPollingTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xDelay = pdMS_TO_TICKS(SWITCH_POLLING_PERIOD_MS); // poll every 100ms
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (xSemaphoreTake(xQueueSwitchSemaphore, portMAX_DELAY) == pdTRUE) {
            taskENTER_CRITICAL();
            vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
            uint32_t uiSwitchValueRead = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
            uint32_t uiSwitchValueWrite = uiSwitchValueRead; // default to read value
            // do some processing on the switch values if necessary
            if (uiSwitchValueRead & 0x01) {
                uiSwitchValueWrite |= 0x10; // set bit 4 if bit 0 is set
            } else {
                uiSwitchValueWrite &= 0xEF; // clear bit 4 if bit 0 is cleared
            }
            IOWR_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE, uiSwitchValueWrite); // write the new switch value
            uiSwitchValue = uiSwitchValueRead;
            vTaskPrioritySet(NULL, tskIDLE_PRIORITY);
            taskEXIT_CRITICAL();
            // ...
            xSemaphoreGive(xQueueSwitchSemaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

void button_ISR(void* context, alt_u32 id)
{
    int* temp = (int*) context;
    (*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);
    // checks if any of the buttons have been pressed
    if ((*temp) & 0x7) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (maintenanceMode == 0) {
            xSemaphoreTakeFromISR(maintenanceModeSemaphore, &xHigherPriorityTaskWoken);
            maintenanceMode = 1;
            currentState = maintenanceState;
            xSemaphoreGiveFromISR(maintenanceModeSemaphore, &xHigherPriorityTaskWoken);
        } else {
            xSemaphoreTakeFromISR(maintenanceModeSemaphore, &xHigherPriorityTaskWoken);
            maintenanceMode = 0;
            currentState = stableState;
            xSemaphoreGiveFromISR(maintenanceModeSemaphore, &xHigherPriorityTaskWoken);
        }
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    // clears the edge capture register
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}


void timer_500ms_isr(xTimerHandle t_timer){
	//timer_expired = 1;

	//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//vTaskNotifyGiveFromISR(load_mngr_handle, &xHigherPriorityTaskWoken);
}

// inst_freq_isr - Fetches frequency values from the memory and pushes to the queue
void inst_freq_isr(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	TickType_t start_time = xTaskGetTickCountFromISR();
	xQueueSendToBackFromISR(Q_freq_data, &temp, pdFALSE);
	xQueueSendToBackFromISR(Q_time_stamp, &start_time, pdFALSE);
	return;
}


// keyboard_isr - Fetches value from the keyboard and pushes to the queue
void keyboard_isr(void* keyboard, alt_u32 id){
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;

	status = decode_scancode(keyboard, &decode_mode , &key , &ascii);

	if ( status == 0 ) //success
	{
		// print out the result
		switch ( decode_mode )
		{
		case KB_ASCII_MAKE_CODE :
			xQueueSendToBackFromISR(Q_keyb_data, &key, pdFALSE);
			break ;

		default :
			break ;
		}
	}
}

//freq_calculate_task - Receives frequency and timestamp from queue and calculates roc
void freq_calculate_task()
{
	while(1)
	{
		xQueueReceive(Q_freq_data, freq+freq_value, portMAX_DELAY);
		xQueueReceive(Q_time_stamp, time_stamps+freq_value, (TickType_t) 2);
		xSemaphoreTake(sem_freqroc, portMAX_DELAY);

		//calculate roc
		if(freq_value == 0){
			dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
		} else {
			dfreq[freq_value] = (freq[freq_value]-freq[freq_value-1]) * 2.0 * freq[freq_value]* freq[freq_value-1] / (freq[freq_value]+freq[freq_value-1]);
		}

		if (dfreq[freq_value] > 100.0){
			dfreq[freq_value] = 100.0;
		}

		freq_value = (++freq_value) % 100;  // point to the next data (oldest) to be overwritten
		freq_value_new = (++freq_value_new) % 100;
		xSemaphoreGive(sem_freqroc);

		//check stability
			if ((freq[freq_value_new] < freq_threshold) || (fabs(dfreq[freq_value_new]) > roc_threshold)) {
				stability_status = 1;
			}
			else{
				stability_status = 0;
			}
	}
}


//keyboard_update_task - Fetches keyboard data from queue and updates frequency thresholds
void keyboard_update_task(){
	unsigned char key;
	unsigned char key_pressed = 0;

	while(1){
		xQueueReceive(Q_keyb_data, &key, portMAX_DELAY);

		switch (key)
		{
		case 0x72:  // DOWN
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				//printf("Freq decrease\n");

				if (xSemaphoreTake(sem_thresh, (TickType_t ) 10)){
					freq_threshold -= FREQ_THRESHOLD_INC_AMOUNT;

					if (freq_threshold < 0.0){
						freq_threshold = 0;
					}
					xSemaphoreGive(sem_thresh);
				}
			}
			break;

		case 0x75:  // UP
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				//printf("Freq increase\n");

				if (xSemaphoreTake(sem_thresh, (TickType_t ) 10)){
					freq_threshold += FREQ_THRESHOLD_INC_AMOUNT;

					if (freq_threshold < 0.0){
						freq_threshold = 0;
					}

					xSemaphoreGive(sem_thresh);
				}
			}
			break;

		case 0x6b:  // LEFT
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				//printf("ROC decrease\n");

				if (xSemaphoreTake(sem_thresh, (TickType_t ) 10)){
					roc_threshold -= ROC_THRESHOLD_INC_AMOUNT;

					if (roc_threshold < 0.0){
						roc_threshold = 0;
					}

					xSemaphoreGive(sem_thresh);
				}
			}
			break;

		case 0x74:  // RIGHT
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				//printf("ROC increase\n");

				if (xSemaphoreTake(sem_thresh, (TickType_t ) 10)){
					roc_threshold += ROC_THRESHOLD_INC_AMOUNT;

					if (roc_threshold < 0.0){
						roc_threshold = 0;
					}

					xSemaphoreGive(sem_thresh);
				}
			}
			break;

		default:
			//printf("key = %c\n", &key);
			break;
		}

	}
}

void VGAdisplay_Task(void *pvParameters ){
	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);


	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

	alt_up_char_buffer_string(char_buf, "Run Time                 = ", 12, 40);
	alt_up_char_buffer_string(char_buf, "Frequency Threshold (Hz) =       (Up/Down arrows)", 12, 42);
	alt_up_char_buffer_string(char_buf, "RoC Threshold (Hz/s)     =       (Left/Right arrows)", 12, 44);
	int j = 0;
	Line line_freq, line_roc;

	char temp_buf[6];
	unsigned int milisec;

	const TickType_t task_period = 100;  // 30 Hz
	TickType_t last_wake_time;

	while(1){
		last_wake_time = xTaskGetTickCount();
		vTaskDelayUntil(&last_wake_time, task_period);

		milisec = xTaskGetTickCount();

		sprintf(temp_buf, "%02d:%02d:%02d.%1d", (milisec/3600000) % 24, (milisec/60000) % 60, (milisec/1000) % 60, (milisec/100) % 10);
		alt_up_char_buffer_string(char_buf, temp_buf, 40, 40);

		// Read thresholds and print to screen
		sprintf(temp_buf, "%2.1f", freq_threshold);
		alt_up_char_buffer_string(char_buf, temp_buf, 40, 42);
		sprintf(temp_buf, "%2.1f ", roc_threshold);
		alt_up_char_buffer_string(char_buf, temp_buf, 40, 44);

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(freq_value+j)%100]) > MIN_FREQ) && ((int)(freq[(freq_value+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freq_value+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freq_value+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freq_value+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freq_value+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
	}
}

int main() {
	// FreeRTOS initialisation
	 	maintenanceModeSemaphore = xSemaphoreCreateMutex();
	 	xQueueSwitchSemaphore = xSemaphoreCreateBinary();
		xSemaphoreGive(switchValueSemaphore);
	 	switch_queue = xQueueCreate(QUEUE_SWITCH_LENGTH, sizeof(int));
	   	loadsShedQueue = xQueueCreate(QUEUE_LOADS_LENGTH, sizeof(int));
		// Initialise queues
		Q_freq_data = xQueueCreate( 100, sizeof(double) );
		Q_time_stamp = xQueueCreate(100, sizeof(TickType_t));
		Q_keyb_data = xQueueCreate(5, sizeof(unsigned char));

		// Initialise semaphores
		sem_freqroc = xSemaphoreCreateMutex();
		sem_thresh = xSemaphoreCreateMutex();
		sem_block = xSemaphoreCreateBinary();

		xSemaphoreGive(sem_freqroc);
		xSemaphoreGive(sem_thresh);
		xSemaphoreGive(sem_block);

		// Hardware initialisation
		// Initialise ps2 device
		alt_up_ps2_dev * keyboard = alt_up_ps2_open_dev(PS2_NAME);

		// Reigster Interrupts
		alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, inst_freq_isr);
		alt_irq_register(PS2_IRQ, keyboard, keyboard_isr);
		alt_up_ps2_enable_read_interrupt(keyboard);

		// Tasks
		xTaskCreate(VGAdisplay_Task, "DrawTsk", configMINIMAL_STACK_SIZE, NULL, 1 , &VGAdisplay);
		xTaskCreate(keyboard_update_task, "keyboard_update_task", configMINIMAL_STACK_SIZE, NULL, 4 , &keyboard_update);
		xTaskCreate(freq_calculate_task, "freq_calculate_task", configMINIMAL_STACK_SIZE, NULL, 5 , &freq_update);
		xTaskCreate(vLoadSheddingTask, "LoadSheddingTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
		xTaskCreate(button_ISR, "ButtonISR", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
		xTaskCreate(switch_ISR, "SwitchISR", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
		xTaskCreate(vSwitchPollingTask, "SwitchPollingTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
		timerTwo = xTimerCreate("500Timer", CRITICAL_PERIOD_POLL / portTICK_PERIOD_MS, pdFALSE, 0, timer_500ms_isr);

	    // enable interrupts for all buttons
	    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	    // Enable the switch button interrupt
	    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(SLIDE_SWITCH_BASE, 0x7);
	    // register the ISR
	    alt_irq_register(SLIDE_SWITCH_IRQ, NULL, switch_ISR);
	    alt_irq_register(PUSH_BUTTON_IRQ,(void*)&button_ISR, NULL);

		vTaskStartScheduler();

        while (1);
    return 0;
}
