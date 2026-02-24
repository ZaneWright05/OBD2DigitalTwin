#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "CAN/CAN_DEV_Config.h"
#include "CAN/MCP2515.h"

#define RPM_VEH_SPEED_PERIOD_MS 333
#define THRTL_INTK_PERIOD_MS 500
#define MAF_CTRL_VOLT_PERIOD_MS 1000
#define COOLNT_FUEL_PRESS_PERIOD_MS 2000
#define RUNTIME_SINCE_ENG_START_PERIOD_MS 4000

StaticTask_t RPM_VEH_SPEED_task_p;
StackType_t RPM_VEH_SPEED_stack[256];

StaticTask_t THRTL_INTK_task_p;
StackType_t THRTL_INTK_stack[256];

StaticTask_t MAF_CTRL_VOLT_task_p;
StackType_t MAF_CTRL_VOLT_stack[256];

StaticTask_t COOLNT_FUEL_PRESS_task_p;
StackType_t COOLNT_FUEL_PRESS_stack[256];

StaticTask_t RUNTIME_SINCE_ENG_START_task_p;
StackType_t RUNTIME_SINCE_ENG_START_stack[256];

StaticTask_t xIdleTaskTCB;
StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = xIdleStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

StaticTask_t xTimerTaskTCB;
StackType_t  xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = xTimerStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

int extract_Data_By_PID(uint8_t pid, uint8_t * replyFrame, int pos){
    int dataSize = 0;
    switch (pid)
    {
    case 0x0C:
        printf("PID 0x%02X: Data %02X %02X (RPM)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        break;
    case 0x42:
        printf("PID 0x%02X: Data %02X %02X (Voltage)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        break;
    case 0x23:
        printf("PID 0x%02X: Data %02X %02X (Fuel Rail Pressure)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        break;
    case 0x10:
        printf("PID 0x%02X: Data %02X %02X (MAF)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        break;
    case 0x1F:
        printf("PID 0x%02X: Data %02X %02X (Run time since engine start)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        break;
    // TODO: handle "unknown PID" case
    default: // case for PIDs with 1 data byte
        printf("PID 0x%02X: Data 0x%02X\n", pid, replyFrame[pos]);
        dataSize = 1;
        break;
    }
   // printf("Returning data size: %d\n", dataSize);
    return dataSize;
}


uint32_t canID = 0x7DF;
// this blocks for the duration of the CAN transmission, max timeout should be 200ms 
// in the recieve function but protocol should max out at 60ms 
void CAN_transmission(uint8_t* request_frame, uint8_t* response_frame){
    vTaskSuspendAll(); // enter the critical region
    MCP2515_Send(canID, request_frame, 8);
    MCP2515_Receive(0x7E8, response_frame, 1); // worst case should be 60ms
    xTaskResumeAll(); // exit the critical region
}

void decode_Reply_Frame(uint8_t * replyFrame){
    uint8_t length = replyFrame[0];
    if(length < 0x00 || length > 0x07){ // frame length check
        return;
    }
    if(replyFrame[1] != 0x41){ // support responses to mode 1
        return;
    }
    int i = 0;
    int offset = 2; // len and mode bytes
    while(i < length - 1){ // loop and collect related data for each PID
        uint8_t pid = replyFrame[offset + i];
        i += (extract_Data_By_PID(pid, replyFrame, offset + i + 1) + 1); // offset + i + 1 first byte after PID 
        //printf("i: %d\n", i);
    }
}

void rpm_veh_task(void *params)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(RPM_VEH_SPEED_PERIOD_MS); // task period 5s
    // const uint LED_PIN = 16;
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);
    uint8_t rpmSpeedFrame[8] = {0x03, 0x01, 0x0C, 0x0D,0x00, 0x00, 0x00, 0x00};
    TickType_t nextRelease = xTaskGetTickCount();
    while (1)
    {
        uint8_t rpmSpeedReply[8] = {0};
        
        CAN_transmission(rpmSpeedFrame, rpmSpeedReply);

        decode_Reply_Frame(rpmSpeedReply);

        vTaskDelayUntil(&nextRelease, period_ticks); // wait until next release
    }
}

void thrttl_intk_task(void *params)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(THRTL_INTK_PERIOD_MS); // task period 5s
    // const uint LED_PIN = 16;
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);
    uint8_t thrtlIntkLdFrame[8] = {0x04, 0x01, 0x11, 0x0F, 0x04, 0x00, 0x00, 0x00};// throttle, intake temp, engine load
    TickType_t nextRelease = xTaskGetTickCount();
    for (;;) 
    {
        uint8_t thrtlIntkLdReply[8] = {0};

        CAN_transmission(thrtlIntkLdFrame, thrtlIntkLdReply);

        decode_Reply_Frame(thrtlIntkLdReply);

        vTaskDelayUntil(&nextRelease, period_ticks);
    }
}

int main()
{
    stdio_init_all();
    while(!stdio_usb_connected());
    CAN_DEV_Module_Init();
    MCP2515_Init();

    xTaskCreateStatic(rpm_veh_task,
                "RPM & Vehicle Speed Task",
                256,
                NULL,
                1,
                RPM_VEH_SPEED_stack,
            &RPM_VEH_SPEED_task_p);

    xTaskCreateStatic(thrttl_intk_task,
                "Throttle & Intake Task",
                256,
                NULL,
                1,
                THRTL_INTK_stack,
            &THRTL_INTK_task_p);

    
    vTaskStartScheduler();

    while (1);
}
