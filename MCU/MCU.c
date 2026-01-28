#include <stdlib.h>
#include <stdio.h>
#include "lib/CAN/CAN_DEV_Config.h"
#include "lib/CAN/MCP2515.h"


void decode_Reply(uint8_t pid, uint8_t* reply){
    if(reply[0] == 0x04 && reply[1] == 0x41 && reply[2] == pid){
        if(pid == 0x0D){ // vehicle speed
            uint8_t speed = reply[3]; // speed in km/h
            printf("Vehicle Speed: %d km/h\n", speed);
        }
        // Add more PID decodings as needed
    } else {
        printf("Invalid reply for PID %02X\n", pid);
    }
}

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

int query_CAN_test(uint8_t* data){
    uint32_t canID = 0x7DF; // broadcast ID

    uint8_t frame[8] = {0x02, 0x01, 0x0C, 0x00,0x00, 0x00, 0x00, 0x00};// rpm
    uint8_t reply[8] = {0};
    // printf("Sending RPM request \n");
    MCP2515_Send(canID, frame, 8); // send the frame
    MCP2515_Receive(0x7E8, reply, 0); // wait for the reply (no debug and 200ms timeout)
    data[0] = reply[3];
    data[1] = reply[4];

    uint8_t frame2[8] = {0x02, 0x01, 0x0D, 0x00,0x00, 0x00, 0x00, 0x00};// speed
    uint8_t reply2[8] = {0};
    // printf("Sending Speed request \n");
    MCP2515_Send(canID, frame2, 8); // send the frame
    MCP2515_Receive(0x7E8, reply2, 0); // wait for the reply (no debug and 200ms timeout)
    data[2] = reply2[3];

    uint8_t frame3[8] = {0x02, 0x01, 0x11, 0x00,0x00, 0x00, 0x00, 0x00};// throttle
    uint8_t reply3[8] = {0};
    MCP2515_Send(canID, frame3, 8); // send the frame
    MCP2515_Receive(0x7E8, reply3, 0); // wait for the reply (no debug and 200ms timeout)
    data[3] = reply3[3];

    uint8_t frame4[8] = {0x02, 0x01, 0x05, 0x00,0x00, 0x00, 0x00, 0x00};// coolant temp
    uint8_t reply4[8] = {0};
    MCP2515_Send(canID, frame4, 8); // send the frame
    MCP2515_Receive(0x7E8, reply4, 0); // wait for the reply (no debug and 200ms timeout)
    data[4] = reply4[3];

    uint8_t frame5[8] = {0x02, 0x01, 0x42, 0x00,0x00, 0x00, 0x00, 0x00};// control module voltage
    uint8_t reply5[8] = {0};
    MCP2515_Send(canID, frame5, 8); // send the frame
    MCP2515_Receive(0x7E8, reply5, 0); // wait for the reply (no debug and 200ms timeout)
    data[5] = reply5[3];
    data[6] = reply5[4];

    uint8_t frame6[8] = {0x02, 0x01, 0x04, 0x00,0x00, 0x00, 0x00, 0x00};// engine load
    uint8_t reply6[8] = {0};
    MCP2515_Send(canID, frame6, 8); // send the frame
    MCP2515_Receive(0x7E8, reply6, 0); // wait for the reply (no debug and 200ms timeout)
    data[7] = reply6[3];

    uint8_t frame7[8] = {0x02, 0x01, 0x23, 0x00,0x00, 0x00, 0x00, 0x00};// fuel rail pressure
    uint8_t reply7[8] = {0};
    MCP2515_Send(canID, frame7, 8); // send the frame
    MCP2515_Receive(0x7E8, reply7, 0); // wait for the reply (no debug and 200ms timeout)
    data[8] = reply7[3];
    data[9] = reply7[4];

    uint8_t frame8[8] = {0x02, 0x01, 0x10, 0x00,0x00, 0x00, 0x00, 0x00};// MAF
    uint8_t reply8[8] = {0};
    MCP2515_Send(canID, frame8, 8); // send the frame
    MCP2515_Receive(0x7E8, reply8, 0); // wait for the reply (no debug and 200ms timeout)
    data[10] = reply8[3];
    data[11] = reply8[4];

    uint8_t frame9[8] = {0x02, 0x01, 0x1F, 0x00,0x00, 0x00, 0x00, 0x00};// rt since engine start
    uint8_t reply9[8] = {0};
    MCP2515_Send(canID, frame9, 8); // send the frame
    MCP2515_Receive(0x7E8, reply9, 0); // wait for the reply (no debug and 200ms timeout)
    data[12] = reply9[3];
    data[13] = reply9[4];

    uint8_t frame10[8] = {0x02, 0x01, 0x0F, 0x00,0x00, 0x00, 0x00, 0x00};// air intake temp
    uint8_t reply10[8] = {0};
    MCP2515_Send(canID, frame10, 8); // send the frame
    MCP2515_Receive(0x7E8, reply10, 0); // wait for the reply (no debug and 200ms timeout)
    data[14] = reply10[3];
    // printf("Received: ");
    // for(int i = 0; i < 8; i++){
    //     printf("%02X ", reply[i]);
    // }
    // printf("\n");
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
    printf("Test start\n");
    while(1){
        //printf("Sending speed request \n");

        // query_CAN(speed_PID);
        uint8_t data [15] = {0};
        query_CAN_test(data);
        for(int i = 0; i < 15; i++){
            printf("%02X,", data[i]);
        }
        printf("\n");
        next_time = delayed_by_ms(next_time, 500);
        sleep_until(next_time);
    }
}