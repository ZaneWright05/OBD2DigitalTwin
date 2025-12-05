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
    uint32_t canID = 0x7E8; // broadcast ID
    uint8_t reply[8] = {0};
    MCP2515_Send(canID, frame, 8); // send the frame
    MCP2515_Receive(0x7E8, reply, 0); // wait for the reply (no debug and 200ms timeout)
    decode_Reply(num, reply); // decode and print the reply
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
    while(1){
        // printf("Sending speed request \n");
        query_CAN(speed_PID);
        sleep_ms(100);
    }
}