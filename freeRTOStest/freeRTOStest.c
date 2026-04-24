    #include <stdio.h>
    #include "pico/stdlib.h"
    #include "pico/multicore.h"
    #include "pico/util/queue.h"
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "CAN/CAN_DEV_Config.h"
    #include "CAN/MCP2515.h"
    #include "hardware/watchdog.h"

    #define RPM_VEH_SPEED_PERIOD_MS 333
    #define THRTL_COOLANT_LOAD_PERIOD_MS 500
    #define MAF_CTRL_VOLT_PERIOD_MS 1000
    #define INTAKE_FUEL_PRESS_PERIOD_MS 2000
    // #define RUNTIME_SINCE_ENG_START_PERIOD_MS 4000
    #define START_SIGNAL 'S'
    #define STOP_SIGNAL 'X'
    #define VALID_VEHICLE_SIGNAL 'V'
    #define INVALID_VEHICLE_SIGNAL 'I'

    StaticTask_t RPM_VEH_SPEED_task_p;
    StackType_t RPM_VEH_SPEED_stack[256];

    StaticTask_t THRTL_COOLANT_LOAD_task_p;
    StackType_t THRTL_COOLANT_LOAD_stack[256];

    StaticTask_t MAF_CTRL_VOLT_task_p;
    StackType_t MAF_CTRL_VOLT_stack[256];

    StaticTask_t INTAKE_FUEL_PRESS_task_p;
    StackType_t INTAKE_FUEL_PRESS_stack[256];

    // StaticTask_t RUNTIME_SINCE_ENG_START_task_p;
    // StackType_t RUNTIME_SINCE_ENG_START_stack[256];

    // static StaticQueue_t xStaticQueue;
    // QueueHandle_t dataQueue;

    uint8_t pid00Mask[4] = {0x18, 0x1B, 0x80, 0x01}; // speed, eng speed, eng load, maf ,coolant temp, throttle pos, int temp and next set
    uint8_t pid20Mask[4] = {0x20, 0x00, 0x00, 0x01}; // fuel press and next set
    uint8_t pid40Mask[4] = {0x40, 0x00, 0x00, 0x00}; // look for volt

    typedef struct __attribute__((packed)) { // esure struct is exactly 7 bytes
        uint8_t pid;
        uint32_t seqNum;
        uint8_t data[2];
    } QueueEntry;

    queue_t dataQueue;
    volatile bool hostConnected = false;
    volatile bool tripStarted = false;


    // uint8_t ucQueueStorageArea[ 256 * sizeof(QueueEntry) ];

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
        uint8_t dataValue[2];
    } PIDValue;


    /*
    simple, should be replaced with look up tables and along with better error/decoding logic
    also the data types should be fixed
    */
    PIDValue extract_Data_By_PID(uint8_t pid, uint8_t * replyFrame, int pos){
        PIDValue pidValue = {0, {0, 0}};
        //float value = 0.0f;
        //int dataSize = 0;
        switch (pid)
        {
        case 0x0C:
            //printf("PID 0x%02X: Data %02X %02X (RPM)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
            //dataSize = 2;
            //value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) / 4; // each bit is 1/4 rpm
            pidValue.dataSize = 2;
            pidValue.dataValue[0] = replyFrame[pos];
            pidValue.dataValue[1] = replyFrame[pos + 1];
            break;
        case 0x42:
            //printf("PID 0x%02X: Data %02X %02X (Voltage)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
            pidValue.dataSize = 2;
            pidValue.dataValue[0] = replyFrame[pos];
            pidValue.dataValue[1] = replyFrame[pos + 1];
            //value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) / 1000.0f; // each bit is 0.001 volts
            break;
        case 0x23:
            //printf("PID 0x%02X: Data %02X %02X (Fuel Rail Pressure)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
            pidValue.dataSize = 2;
            pidValue.dataValue[0] = replyFrame[pos];
            pidValue.dataValue[1] = replyFrame[pos + 1];
            //value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) * 10; // each bit is 10 kPa
            break;
        case 0x10:
            //printf("PID 0x%02X: Data %02X %02X (MAF)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
            pidValue.dataSize = 2;
            pidValue.dataValue[0] = replyFrame[pos];
            pidValue.dataValue[1] = replyFrame[pos + 1];
            //value = ((replyFrame[pos] << 8) + replyFrame[pos + 1]) / 100.0f; // each bit is 0.01 g/s
            break;
        case 0x1F:
            //printf("PID 0x%02X: Data %02X %02X (Run time since engine start)\n", pid, replyFrame[pos], replyFrame[pos + 1]);
            pidValue.dataSize = 2;
            pidValue.dataValue[0] = replyFrame[pos];
            pidValue.dataValue[1] = replyFrame[pos + 1];
            //value = (replyFrame[pos] << 8) + replyFrame[pos + 1]; // each bit is 1 second
            break;
        case 0x0D:
            //printf("PID 0x%02X: Data 0x%02X (Vehicle Speed)\n", pid, replyFrame[pos]);
            pidValue.dataSize = 1;
            pidValue.dataValue[0] = replyFrame[pos];
            //value = replyFrame[pos]; // 1km/h = 1 bit  
            break;
        case 0x11:
            //printf("PID 0x%02X: Data 0x%02X (Throttle Position)\n", pid, replyFrame[pos]);
            pidValue.dataSize = 1;
            pidValue.dataValue[0] = replyFrame[pos];
            //value = replyFrame[pos] * 100 / 256; // % value
            break;
        case 0x0F:
            //printf("PID 0x%02X: Data 0x%02X (Intake Air Temperature)\n", pid, replyFrame[pos]);
            pidValue.dataSize = 1;
            pidValue.dataValue[0] = replyFrame[pos];
            //value = replyFrame[pos] - 40; // temp val offset by 40 celcius
            break;
        case 0x05:
            //printf("PID 0x%02X: Data 0x%02X (Engine Coolant Temperature)\n", pid, replyFrame[pos]);
            pidValue.dataSize = 1;
            pidValue.dataValue[0] = replyFrame[pos];
            //value = replyFrame[pos] - 40; // temp val offset by 40 celcius
            break;
        case 0x04:
            //printf("PID 0x%02X: Data 0x%02X (Calculated Engine Load)\n", pid, replyFrame[pos]);
            pidValue.dataSize = 1;
            pidValue.dataValue[0] = replyFrame[pos];
            //value = replyFrame[pos] * 100 / 256; // % value
            break;
        // this is a catch all assuming one data byte
        default:
            printf("PID 0x%02X: Data 0x%02X\n", pid, replyFrame[pos]);  
            pidValue.dataSize = 100;
            pidValue.dataValue[0] = replyFrame[pos];
            //value = replyFrame[pos];
            break;
        }
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
        MCP2515_Receive(0x7E8, response_frame, 1); // worst case should be 50ms
        xTaskResumeAll(); // exit the critical region
    }

    void add_To_Queue(uint8_t pid, PIDValue pidValue, uint32_t seqNum){
        QueueEntry entry;
        entry.pid = pid;
        entry.seqNum = seqNum;
        entry.data[0] = pidValue.dataValue[0];
        entry.data[1] = pidValue.dataValue[1];
        queue_add_blocking(&dataQueue, &entry); 
    }

    void clear_Queue(){
        QueueEntry entry = {0};
        while(queue_get_level(&dataQueue) > 0){
            queue_remove_blocking(&dataQueue, &entry);
            printf("0x%02X,0x%02X,0x%02X,%u\n", entry.pid, entry.data[0], entry.data[1], entry.seqNum);
        }
    }

    int getMaxPeriod(){
        int max = RPM_VEH_SPEED_PERIOD_MS;
        if(THRTL_COOLANT_LOAD_PERIOD_MS > max){
            max = THRTL_COOLANT_LOAD_PERIOD_MS;
        }
        if(MAF_CTRL_VOLT_PERIOD_MS > max){
            max = MAF_CTRL_VOLT_PERIOD_MS;
        }
        if(INTAKE_FUEL_PRESS_PERIOD_MS > max){
            max = INTAKE_FUEL_PRESS_PERIOD_MS;
        }
        return max;
    }

    void MCU_reset(){
        tripStarted = false;

        sleep_ms(getMaxPeriod()); // wait for longest task so all tasks finish

        clear_Queue(); // clear the queue to avoid old data being printed when trip starts again

        watchdog_reboot(0, 0, 0); // reset the MCU

        while(1){
            tight_loop_contents();
        }
    }

    void pop_From_Queue(){
    // int cntr = 0;
        QueueEntry entry = {0};
        while(1){
            int ch = getchar_timeout_us(0);
            if(ch == STOP_SIGNAL){
                MCU_reset();
            }

            if(queue_get_level(&dataQueue) > 0){
                queue_remove_blocking(&dataQueue, &entry);
                printf("0x%02X,0x%02X,0x%02X,%u\n", entry.pid, entry.data[0], entry.data[1], entry.seqNum);
                sleep_ms(25);
            }
        }
    }

    void decode_Reply_Frame(uint8_t * replyFrame, uint32_t sequenceNum){
        uint8_t length = replyFrame[0];
        if(length <= 0x00 || length > 0x07){ // frame length check
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
            add_To_Queue(pid, pidVal, sequenceNum);
            if(pidVal.dataSize == 100){ // catch all for unsupported PIDs, just print the value
            printf("Invalid PID, printing whole frame: ");
                for(int j = 0; j < 8; j++){
                    printf("0x%02X ", replyFrame[j]);
                }
                printf("\n");
            }
            i += pidVal.dataSize + 1;
            // printf("Data Value for %02X is %f\n", pid, pidVal.dataValue[0]);
        }
    }

    void rpm_veh_task(void *params)
    {
        const TickType_t period_ticks = pdMS_TO_TICKS(RPM_VEH_SPEED_PERIOD_MS);
        const uint task_PIN = 14;

        uint32_t seqNum = 0;
        gpio_init(task_PIN);
        gpio_set_dir(task_PIN, GPIO_OUT);
        uint8_t rpmSpeedFrame[8] = {0x03, 0x01, 0x0C, 0x0D,0x00, 0x00, 0x00, 0x00};
        TickType_t nextRelease = xTaskGetTickCount();
        while (1)
        {
            if (!tripStarted) {
                gpio_put(task_PIN, 0);
                vTaskDelayUntil(&nextRelease, period_ticks);
                continue;
            }
            gpio_put(task_PIN, 1);
            absolute_time_t start = get_absolute_time();
            uint8_t rpmSpeedReply[8] = {0};
            CAN_transmission(rpmSpeedFrame, rpmSpeedReply);
            decode_Reply_Frame(rpmSpeedReply, seqNum);
            seqNum++;
            gpio_put(task_PIN, 0);
            // printf("RPM & Speed Task Duration: %d ms\n", absolute_time_diff_us(start, get_absolute_time()) / 1000);
            vTaskDelayUntil(&nextRelease, period_ticks); // wait until next release
        }
    }

    void thrttl_coolant_load_task(void *params)
    {
        const TickType_t period_ticks = pdMS_TO_TICKS(THRTL_COOLANT_LOAD_PERIOD_MS); // task period 5s
        const uint task_PIN = 15;

        uint32_t seqNum = 0;
        gpio_init(task_PIN);
        gpio_set_dir(task_PIN, GPIO_OUT);
        uint8_t thrtlIntkLdFrame[8] = {0x04, 0x01, 0x11, 0x05, 0x04, 0x00, 0x00, 0x00};// throttle, coolant temp, engine load
        TickType_t nextRelease = xTaskGetTickCount();
        for (;;) 
        {
            if (!tripStarted) {
                gpio_put(task_PIN, 0);
                vTaskDelayUntil(&nextRelease, period_ticks);
                continue;
            }
            gpio_put(task_PIN, 1);
            uint8_t thrtlIntkLdReply[8] = {0};
            CAN_transmission(thrtlIntkLdFrame, thrtlIntkLdReply);
            decode_Reply_Frame(thrtlIntkLdReply, seqNum);
            seqNum++;
            gpio_put(task_PIN, 0);
            vTaskDelayUntil(&nextRelease, period_ticks);
        }
    }

    void maf_ctrl_volt_task(void* params){
        const TickType_t period_ticks = pdMS_TO_TICKS(MAF_CTRL_VOLT_PERIOD_MS); // task period 5s
        const uint task_PIN = 2;
        uint32_t seqNum = 0;
        gpio_init(task_PIN);
        gpio_set_dir(task_PIN, GPIO_OUT);
        uint8_t MAFVoltFrame[8] = {0x03, 0x01, 0x10, 0x42,0x00, 0x00, 0x00, 0x00};
        TickType_t nextRelease = xTaskGetTickCount();
        for(;;){
            if (!tripStarted) {
                gpio_put(task_PIN, 0);
                vTaskDelayUntil(&nextRelease, period_ticks);
                continue;
            }
            gpio_put(task_PIN, 1);
            uint8_t MAFVoltReply[8] = {0};
            CAN_transmission(MAFVoltFrame, MAFVoltReply);
            decode_Reply_Frame(MAFVoltReply, seqNum);
            seqNum++;
            gpio_put(task_PIN, 0);
            vTaskDelayUntil(&nextRelease, period_ticks);
        }
    }

    void intake_fuel_press_task(void* params){
        const TickType_t period_ticks = pdMS_TO_TICKS(INTAKE_FUEL_PRESS_PERIOD_MS); // task period 5s
        const uint task_PIN = 4;

        uint32_t seqNum = 0;
        gpio_init(task_PIN);
        gpio_set_dir(task_PIN, GPIO_OUT);
        uint8_t intakeFuelRlFrame[8] = {0x03, 0x01, 0x0F, 0x23,0x00, 0x00, 0x00, 0x00};
        TickType_t nextRelease = xTaskGetTickCount();
        for(;;){
            if (!tripStarted) {
                gpio_put(task_PIN, 0);
                vTaskDelayUntil(&nextRelease, period_ticks);
                continue;
            }
            gpio_put(task_PIN, 1);
            uint8_t intakeFuelRlReply[8] = {0};
            CAN_transmission(intakeFuelRlFrame, intakeFuelRlReply);
            decode_Reply_Frame(intakeFuelRlReply, seqNum);
            seqNum++;
            gpio_put(task_PIN, 0);
            vTaskDelayUntil(&nextRelease, period_ticks);
        }
    }

    bool check_pid_support(const uint8_t response[4], const uint8_t required[4]) {
        for (int i = 0; i < 4; i++) {
            if ((response[i] & required[i]) != required[i]) {
                return false; 
            }
        }
        return true;
    }

    bool check_all_PIDS_supported(){
        uint8_t request[8] = {0x02, 0x01, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // 00-20
        uint8_t response[8] = {0};
        MCP2515_Send(canID, request, 8);
        MCP2515_Receive(0x7E8, response, 1);
        uint8_t respData[4] = {response[3], response[4], response[5], response[6]}; 
        if(response[1] == 0x41 && response[2] == 0x00 && check_pid_support(respData, pid00Mask)){
            uint8_t request2[8] = {0x02, 0x01, 0x20, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // 20-40
            uint8_t response2[8] = {0};
            MCP2515_Send(canID, request2, 8);
            MCP2515_Receive(0x7E8, response2, 1);
            uint8_t respData2[4] = {response2[3], response2[4], response2[5], response2[6]};

            if(response2[1] == 0x41 && response2[2] == 0x20 && check_pid_support(respData2, pid20Mask)){
                uint8_t request3[8] = {0x02, 0x01, 0x40, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // 40-60
                uint8_t response3[8] = {0};
                MCP2515_Send(canID, request3, 8);
                MCP2515_Receive(0x7E8, response3, 1);
                uint8_t respData3[4] = {response3[3], response3[4], response3[5], response3[6]};
                if(response3[1] == 0x41 && response3[2] == 0x40 && check_pid_support(respData3, pid40Mask)){ // all req PIDS supported
                    return true;
                }
            }
        }
        return false;
    }

    bool find_valid_baud_rate(){
        enum RATEBPS isoRates[] = {KBPS250, KBPS500};
        uint8_t request[8] = {0x02, 0x01, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // 00-20
        uint8_t response[8] = {0};
        for(int i = 0; i < sizeof(isoRates)/sizeof(isoRates[0]); i++){
            MCP2515_Init(isoRates[i]);
            MCP2515_Send(canID, request, 8);
            MCP2515_Receive(0x7E8, response, 1); // worst case should be 60ms
            if(response[1] == 0x41 && response[2] == 0x00){ // valid response received
                printf("Valid baud rate found: %d kbps\n", (isoRates[i] == KBPS250) ? 250 : 500);
                return true;
            }
        }
        return false;
    }

    int main()
    {
        stdio_init_all();
        CAN_DEV_Module_Init();
        bool baudRateFound = find_valid_baud_rate();
        bool validVehicle = check_all_PIDS_supported();
        // while(!stdio_usb_connected()){
        //     tight_loop_contents();
        // }
        // check vehicle connection and supp PIDS, else send err msg and wait on connection

        // wait for host start signal
        while(1){
            if(!stdio_usb_connected()){
                continue;
            }
            if(baudRateFound){
                if(validVehicle){
                    printf("%c\n",VALID_VEHICLE_SIGNAL);
                    int input = getchar_timeout_us(100000); // wait for 100ms for input
                    if(input == START_SIGNAL){
                        tripStarted = true;
                        break;
                    }
                }
                else{
                    printf("%c\n",INVALID_VEHICLE_SIGNAL);
                    validVehicle = check_all_PIDS_supported();
                    sleep_ms(500);
                }
            }
            else{
                printf("%c\n",INVALID_VEHICLE_SIGNAL);
                baudRateFound = find_valid_baud_rate();
                sleep_ms(500);
            }
        }

        // tusb_init();
        //dataQueue = xQueueCreateStatic(256, sizeof(QueueEntry), ucQueueStorageArea, &xStaticQueue);
        queue_init(&dataQueue, sizeof(QueueEntry),256);

        xTaskCreateStatic(rpm_veh_task,
                    "RPM & Vehicle Speed Task",
                    256,
                    NULL,
                    5,
                    RPM_VEH_SPEED_stack,
                &RPM_VEH_SPEED_task_p);

        xTaskCreateStatic(thrttl_coolant_load_task,
                    "Throttle & Coolant and load Task",
                    256,
                    NULL,
                    4,
                    THRTL_COOLANT_LOAD_stack,
                &THRTL_COOLANT_LOAD_task_p);

        xTaskCreateStatic(maf_ctrl_volt_task,
                    "MAF & Voltage Task",
                    256,
                    NULL,
                    3,
                    MAF_CTRL_VOLT_stack,
                &MAF_CTRL_VOLT_task_p);
        
        xTaskCreateStatic(intake_fuel_press_task,
                    "Intake & Fuel Pressure Task",
                    256,
                    NULL,
                    2,
                    INTAKE_FUEL_PRESS_stack,
                &INTAKE_FUEL_PRESS_task_p);

        // xTaskCreateStatic(rt_since_strt_task,
        //             "Run Time Since Engine Start Task",
        //             256,
        //             NULL,
        //             1,
        //             RUNTIME_SINCE_ENG_START_stack,
        //         &RUNTIME_SINCE_ENG_START_task_p);
        
        multicore_launch_core1(pop_From_Queue); // start the queue pop task on the second core
        
        vTaskStartScheduler();

        while (1);
    }
