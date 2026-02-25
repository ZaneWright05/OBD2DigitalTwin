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

// struct to hold data size for iteration and the returned value, may need to convert data types later
typedef struct{
    int dataSize;
    float dataValue;
} PIDValue;


/*
simple, should be replaced with look up tables and along with better error/decoding logic
also the data types should be fixed
*/
PIDValue extract_Data_By_PID(uint8_t pid, uint8_t * replyFrame, int pos){
    PIDValue pidValue = {0, 0.0f};
    float value = 0.0f;
    int dataSize = 0;
    switch (pid)
    {
    case 0x0C:
        //printf("PID 0x%02X: Data %02X %02X (RPM)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) / 4; // each bit is 1/4 rpm
        break;
    case 0x42:
        //printf("PID 0x%02X: Data %02X %02X (Voltage)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) / 1000.0f; // each bit is 0.001 volts
        break;
    case 0x23:
        //printf("PID 0x%02X: Data %02X %02X (Fuel Rail Pressure)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) * 10; // each bit is 10 kPa
        break;
    case 0x10:
        //printf("PID 0x%02X: Data %02X %02X (MAF)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) / 100.0f; // each bit is 0.01 g/s
        break;
    case 0x1F:
        //printf("PID 0x%02X: Data %02X %02X (Run time since engine start)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
        dataSize = 2;
        value = (replyFrame[pos] << 8) + replyFrame[pos + 1]; // each bit is 1 second
        break;
    case 0x0D:
        //printf("PID 0x%02X: Data 0x%02X (Vehicle Speed)\n", pid, replyFrame[pos]);
        dataSize = 1;
        value = replyFrame[pos]; // 1km/h = 1 bit  
        break;
    case 0x11:
        //printf("PID 0x%02X: Data 0x%02X (Throttle Position)\n", pid, replyFrame[pos]);
        dataSize = 1;
        value = replyFrame[pos] * 100 / 256; // % value
        break;
     case 0x0F:
        //printf("PID 0x%02X: Data 0x%02X (Intake Air Temperature)\n", pid, replyFrame[pos]);
        dataSize = 1;
        value = replyFrame[pos] - 40; // temp val offset by 40 celcius
        break;
    case 0x05:
        //printf("PID 0x%02X: Data 0x%02X (Engine Coolant Temperature)\n", pid, replyFrame[pos]);
        dataSize = 1;
        value = replyFrame[pos] - 40; // temp val offset by 40 celcius
        break;
    case 0x04:
        //printf("PID 0x%02X: Data 0x%02X (Calculated Engine Load)\n", pid, replyFrame[pos]);
        dataSize = 1;
        value = replyFrame[pos] * 100 / 256; // % value
        break;
    // this is a catch all assuming one data byte
    default:
        printf("PID 0x%02X: Data 0x%02X\n", pid, replyFrame[pos]);
        dataSize = 1;
        value = replyFrame[pos];
        break;
    }

    pidValue.dataSize = dataSize;
    pidValue.dataValue = value;
   // printf("Returning data size: %d\n", dataSize);
    return pidValue;
}


uint32_t canID = 0x7DF;
/* this blocks for the duration of the CAN transmission, max timeout should be 200ms 
 in the recieve function but protocol should max out at 60ms
 also need to check the other functional responses 0x7E8-0x7EF (8 ECUs)
*/
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
        PIDValue pidVal = extract_Data_By_PID(pid, replyFrame, offset + i + 1); // offset + i + 1 first byte after PID 
        i += pidVal.dataSize + 1;

        printf("Data Value for %02X is %f\n", pid, pidVal.dataValue);
    }
}

void rpm_veh_task(void *params)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(RPM_VEH_SPEED_PERIOD_MS);
    const uint task_PIN = 0;
    gpio_init(task_PIN);
    gpio_set_dir(task_PIN, GPIO_OUT);
    uint8_t rpmSpeedFrame[8] = {0x03, 0x01, 0x0C, 0x0D,0x00, 0x00, 0x00, 0x00};
    TickType_t nextRelease = xTaskGetTickCount();
    while (1)
    {
        gpio_put(task_PIN, 1);
        uint8_t rpmSpeedReply[8] = {0};
        CAN_transmission(rpmSpeedFrame, rpmSpeedReply);
        decode_Reply_Frame(rpmSpeedReply);
        gpio_put(task_PIN, 0);
        vTaskDelayUntil(&nextRelease, period_ticks); // wait until next release
    }
}

void thrttl_intk_task(void *params)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(THRTL_INTK_PERIOD_MS); // task period 5s
    const uint task_PIN = 1;
    gpio_init(task_PIN);
    gpio_set_dir(task_PIN, GPIO_OUT);
    uint8_t thrtlIntkLdFrame[8] = {0x04, 0x01, 0x11, 0x0F, 0x04, 0x00, 0x00, 0x00};// throttle, intake temp, engine load
    TickType_t nextRelease = xTaskGetTickCount();
    for (;;) 
    {
        gpio_put(task_PIN, 1);
        uint8_t thrtlIntkLdReply[8] = {0};
        CAN_transmission(thrtlIntkLdFrame, thrtlIntkLdReply);
        decode_Reply_Frame(thrtlIntkLdReply);
        gpio_put(task_PIN, 0);
        vTaskDelayUntil(&nextRelease, period_ticks);
    }
}

void maf_ctrl_volt_task(void* params){
    const TickType_t period_ticks = pdMS_TO_TICKS(MAF_CTRL_VOLT_PERIOD_MS); // task period 5s
    const uint task_PIN = 2;
    gpio_init(task_PIN);
    gpio_set_dir(task_PIN, GPIO_OUT);
    uint8_t MAFVoltFrame[8] = {0x03, 0x01, 0x10, 0x42,0x00, 0x00, 0x00, 0x00};
    TickType_t nextRelease = xTaskGetTickCount();
    for(;;){
        gpio_put(task_PIN, 1);
        uint8_t MAFVoltReply[8] = {0};
        CAN_transmission(MAFVoltFrame, MAFVoltReply);
        decode_Reply_Frame(MAFVoltReply);
        gpio_put(task_PIN, 0);
        vTaskDelayUntil(&nextRelease, period_ticks);
    }
}

void cllnt_fuel_press_task(void* params){
    const TickType_t period_ticks = pdMS_TO_TICKS(COOLNT_FUEL_PRESS_PERIOD_MS); // task period 5s
    const uint task_PIN = 4;
    gpio_init(task_PIN);
    gpio_set_dir(task_PIN, GPIO_OUT);
    uint8_t cllntFuelRlFrame[8] = {0x03, 0x01, 0x05, 0x23,0x00, 0x00, 0x00, 0x00};
    TickType_t nextRelease = xTaskGetTickCount();
    for(;;){
        gpio_put(task_PIN, 1);
        uint8_t cllntFuelRlReply[8] = {0};
        CAN_transmission(cllntFuelRlFrame, cllntFuelRlReply);
        decode_Reply_Frame(cllntFuelRlReply);
        gpio_put(task_PIN, 0);
        vTaskDelayUntil(&nextRelease, period_ticks);
    }
}

void rt_since_strt_task(void* params){
    const TickType_t period_ticks = pdMS_TO_TICKS(RUNTIME_SINCE_ENG_START_PERIOD_MS); // task period 5s
    const uint task_PIN = 6;
    gpio_init(task_PIN);
    gpio_set_dir(task_PIN, GPIO_OUT);
    uint8_t rtStrtFrame [8] = {0x02, 0x01, 0x1F, 0x00,0x00, 0x00, 0x00, 0x00};// run time since engine start
    TickType_t nextRelease = xTaskGetTickCount();
    for(;;){
        gpio_put(task_PIN, 1);
        uint8_t rtStrtReply [8] = {0};
        CAN_transmission(rtStrtFrame, rtStrtReply);
        decode_Reply_Frame(rtStrtReply);
        gpio_put(task_PIN, 0);
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
                5,
                RPM_VEH_SPEED_stack,
            &RPM_VEH_SPEED_task_p);

    xTaskCreateStatic(thrttl_intk_task,
                "Throttle & Intake Task",
                256,
                NULL,
                4,
                THRTL_INTK_stack,
            &THRTL_INTK_task_p);

    xTaskCreateStatic(maf_ctrl_volt_task,
                "MAF & Voltage Task",
                256,
                NULL,
                3,
                MAF_CTRL_VOLT_stack,
            &MAF_CTRL_VOLT_task_p);
    
    xTaskCreateStatic(cllnt_fuel_press_task,
                "Coolant & Fuel Pressure Task",
                256,
                NULL,
                2,
                COOLNT_FUEL_PRESS_stack,
            &COOLNT_FUEL_PRESS_task_p);

    xTaskCreateStatic(rt_since_strt_task,
                "Run Time Since Engine Start Task",
                256,
                NULL,
                1,
                RUNTIME_SINCE_ENG_START_stack,
            &RUNTIME_SINCE_ENG_START_task_p);
    
    vTaskStartScheduler();

    while (1);
}
