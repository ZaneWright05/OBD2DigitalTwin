#include <stdlib.h>
#include <stdio.h>
#include "lib/CAN/CAN_DEV_Config.h"
#include "lib/CAN/MCP2515.h"

int query_CAN(uint8_t num){
    uint8_t frame[8] = {0x03, 0x01, num, 0x00,0x00, 0x00, 0x00, 0x00};// frame to send, 2 bytes, mode 1 and the PID
    uint32_t canID = 0x7DF; // broadcast ID
    uint8_t reply[8] = {0};
    MCP2515_Send(canID, frame, 8); // send the frame
    MCP2515_Receive(0x7E8, reply, 1); // wait for the reply (no debug and 200ms timeout)
    // decode_Reply(num, reply); // decode and print the reply
    // printf("Received: ");
    // for(int i = 0; i < 8; i++){
    //     printf("%02X ", reply[i]);
    // }
    // printf("\n");
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

int query_CAN_test(uint8_t* data){
    uint32_t canID = 0x7DF; // broadcast ID
    uint8_t rpmSpeedFrame[8] = {0x03, 0x01, 0x0C, 0x0D,0x00, 0x00, 0x00, 0x00};// rpm and speed
    uint8_t rpmSpeedReply[8] = {0};
    MCP2515_Send(canID, rpmSpeedFrame, 8);
    // expected format len, 0x41, pid, A, B, pid, A....
    MCP2515_Receive(0x7E8, rpmSpeedReply, 0);
    decode_Reply_Frame(rpmSpeedReply);

    uint8_t thrtlIntkLdFrame[8] = {0x04, 0x01, 0x11, 0x0F, 0x04, 0x00, 0x00, 0x00};// throttle, intake temp, engine load
    uint8_t thrtlIntkLdReply[8] = {0};
    MCP2515_Send(canID, thrtlIntkLdFrame, 8);
    MCP2515_Receive(0x7E8, thrtlIntkLdReply, 0);
    decode_Reply_Frame(thrtlIntkLdReply);

    uint8_t MAFVoltFrame[8] = {0x03, 0x01, 0x10, 0x42,0x00, 0x00, 0x00, 0x00};// MAF and voltage
    uint8_t MAFVoltReply[8] = {0};
    MCP2515_Send(canID, MAFVoltFrame, 8);
    MCP2515_Receive(0x7E8, MAFVoltReply, 0);
    decode_Reply_Frame(MAFVoltReply);

    uint8_t cllntFuelRlFrame[8] = {0x03, 0x01, 0x05, 0x23,0x00, 0x00, 0x00, 0x00};// coolant temp and fuel rail pressure
    uint8_t cllntFuelRlReply[8] = {0};
    MCP2515_Send(canID, cllntFuelRlFrame, 8);
    MCP2515_Receive(0x7E8, cllntFuelRlReply, 0);
    decode_Reply_Frame(cllntFuelRlReply);

    uint8_t rtStrtFrame [8] = {0x02, 0x01, 0x1F, 0x00,0x00, 0x00, 0x00, 0x00};// run time since engine start
    uint8_t rtStrtReply [8] = {0};
    MCP2515_Send(canID, rtStrtFrame, 8);
    MCP2515_Receive(0x7E8, rtStrtReply, 0);
    decode_Reply_Frame(rtStrtReply);
}

void init_all(){
    stdio_init_all();
    while(!stdio_usb_connected());
    CAN_DEV_Module_Init();
    MCP2515_Init();
}

int main(void) {
    init_all();
    uint8_t speed_PID = 0x0D; // PID for vehicle speed
    absolute_time_t next_time = get_absolute_time();
    // printf("Test start\n");

    // uint8_t data [15] = {0};
    // query_CAN_test(data);
    // for(int i = 0; i < 15; i++){
    //     printf("%02X,", data[i]);
    // }
    // printf("\n");
    while(1){
        //printf("Sending speed request \n");

        // query_CAN(speed_PID);
        uint8_t data [15] = {0};
        query_CAN_test(data);
        // for(int i = 0; i < 15; i++){
        //     printf("%02X,", data[i]);
        // }
        //printf("\n");
        next_time = delayed_by_ms(next_time, 500);
        sleep_until(next_time);
    }
}