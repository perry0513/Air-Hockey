/* WiFi Example
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "mbed.h"
#include "TCPSocket.h"
#include "TCPServer.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"

#define WIFI_IDW0XX1    2

Serial pc(USBTX, USBRX);


#if (defined(TARGET_DISCO_L475VG_IOT01A) || defined(TARGET_DISCO_F413ZH))
// #include "ISM43362Interface.h"
// ISM43362Interface wifi(MBED_CONF_APP_WIFI_SPI_MOSI, MBED_CONF_APP_WIFI_SPI_MISO, MBED_CONF_APP_WIFI_SPI_SCLK, MBED_CONF_APP_WIFI_SPI_NSS, MBED_CONF_APP_WIFI_RESET, MBED_CONF_APP_WIFI_DATAREADY, MBED_CONF_APP_WIFI_WAKEUP, false);

#else // External WiFi modules

#if MBED_CONF_APP_WIFI_SHIELD == WIFI_IDW0XX1
#include "SpwfSAInterface.h"
SpwfSAInterface wifi(MBED_CONF_APP_WIFI_TX, MBED_CONF_APP_WIFI_RX);
#endif // MBED_CONF_APP_WIFI_SHIELD == WIFI_IDW0XX1

#endif

#define SCALE_MULTIPLIER    0.045
#define TIMESTEP            0.0005

float angle[3] = {};
float velocity[3] = {};
float position[3] = {};

float GyroAccumulate[3] = {};
float AccAccumulate[3] = {};


// const char *sec2str(nsapi_security_t sec)
// {
//     switch (sec) {
//         case NSAPI_SECURITY_NONE:
//             return "None";
//         case NSAPI_SECURITY_WEP:
//             return "WEP";
//         case NSAPI_SECURITY_WPA:
//             return "WPA";
//         case NSAPI_SECURITY_WPA2:
//             return "WPA2";
//         case NSAPI_SECURITY_WPA_WPA2:
//             return "WPA/WPA2";
//         case NSAPI_SECURITY_UNKNOWN:
//         default:
//             return "Unknown";
//     }
// }

// int scan_demo(WiFiInterface *wifi)
// {
//     WiFiAccessPoint *ap;

//     printf("Scan:\n");

//     int count = wifi->scan(NULL,0);
//     printf("%d networks available.\n", count);

//     /* Limit number of network arbitrary to 15 */
//     count = count < 15 ? count : 15;

//     ap = new WiFiAccessPoint[count];
//     count = wifi->scan(ap, count);
//     for (int i = 0; i < count; i++)
//     {
//         printf("Network: %s secured: %s BSSID: %hhX:%hhX:%hhX:%hhx:%hhx:%hhx RSSI: %hhd Ch: %hhd\n", ap[i].get_ssid(),
//                sec2str(ap[i].get_security()), ap[i].get_bssid()[0], ap[i].get_bssid()[1], ap[i].get_bssid()[2],
//                ap[i].get_bssid()[3], ap[i].get_bssid()[4], ap[i].get_bssid()[5], ap[i].get_rssi(), ap[i].get_channel());
//     }

//     delete[] ap;
//     return count;
// }

void calculate(float * pGyroDataXYZ, int16_t * pAccDataXYZ) {
    for (int i = 0; i < 3; ++i) {
        if (abs(pGyroDataXYZ[i]) > 15)
            angle[i] += (pGyroDataXYZ[i] + GyroAccumulate[i]) / 2 * TIMESTEP * SCALE_MULTIPLIER;
        if (abs(pAccDataXYZ[i]) > 10) {
            velocity[i] += (pAccDataXYZ[i] + AccAccumulate[i]) / 2 * TIMESTEP;
            position[i] += (pAccDataXYZ[i] + AccAccumulate[i]) / 2 * TIMESTEP * TIMESTEP / 2;
        } else velocity[i] = 0;

        // if (abs(pAccDataXYZ[i] - AccAccumulate[i]) < 2)
        //     velocity[i] = 0;
        
        position[i] += velocity[i] * TIMESTEP;
    }

    for (int i = 0; i < 3; ++i) {
        GyroAccumulate[i] = pGyroDataXYZ[i];
        AccAccumulate[i] = pAccDataXYZ[i];
    }


}


// void acc_server(NetworkInterface *net)
void acc_server()
{
    /* 
    TCPServer socket;
    TCPSocket* client;*/
    // TCPSocket socket;
    // SocketAddress addr("192.168.43.176",65431);
    // nsapi_error_t response;

    // int16_t pDataXYZ[3] = {0};
    // char recv_buffer[9];
    // char acc_json[64];
    // int sample_num = 0;

    

    // // Open a socket on the network interface, and create a TCP connection to addr
    // response = socket.open(net);
    // if (0 != response){
    //     printf("Error opening: %d\n", response);
    // }
    // response = socket.connect(addr);
    
    // if (0 != response){
    //     printf("Error connecting: %d\n", response);
    // }


    // socket.set_blocking(1);
    int sample_num = 0;
    int16_t pAccDataXYZ[3] = {0};
    float pGyroDataXYZ[3] = {0};

    int   AccOffset[3] = {};
    float GyroOffset[3] = {};

    while (sample_num < 2000) {
        sample_num++;
        BSP_GYRO_GetXYZ(pGyroDataXYZ);
        BSP_ACCELERO_AccGetXYZ(pAccDataXYZ);
        for (int i = 0; i < 3; ++i) {
            GyroOffset[i] += pGyroDataXYZ[i];
            AccOffset[i] += pAccDataXYZ[i];
        }

        wait(TIMESTEP);
    }

    for (int i = 0; i < 3; ++i)
        printf("%d ", AccOffset[i]);
    printf("\n");

    for (int i = 0; i < 3; ++i) {
        GyroOffset[i] /= sample_num;
        AccOffset[i] /= sample_num;
    }

    for (int i = 0; i < 3; ++i)
        printf("%d ", AccOffset[i]);
    printf("\n");

    sample_num = 0;

    while (1){
        ++sample_num;

        BSP_GYRO_GetXYZ(pGyroDataXYZ);
        BSP_ACCELERO_AccGetXYZ(pAccDataXYZ);
        for (int i = 0; i < 3; ++i) {
            pGyroDataXYZ[i] = (pGyroDataXYZ[i] - GyroOffset[i]) * SCALE_MULTIPLIER;
            pAccDataXYZ[i] = pAccDataXYZ[i] - AccOffset[i];
        }

        // if (0) {
        if (sample_num % 100 == 0) {
            // printf("\nGYRO_X = %.2f\n", pGyroDataXYZ[0]);
            // printf("GYRO_Y = %.2f\n", pGyroDataXYZ[1]);
            // printf("GYRO_Z = %.2f\n", pGyroDataXYZ[2]);

            // printf("\nANGLE_X = %f\n", angle[0]);
            // printf("ANGLE_Y = %f\n", angle[1]);
            printf("ANGLE_Z = %f\n", angle[2]);

            // printf("\nACCELERO_X = %d\n", pAccDataXYZ[0]);
            // printf("ACCELERO_Y = %d\n", pAccDataXYZ[1]);
            // printf("ACCELERO_Z = %d\n", pAccDataXYZ[2]);

            // printf("\nVELOCITY_X = %f\n", velocity[0]);
            // printf("VELOCITY_Y = %f\n", velocity[1]);
            // printf("VELOCITY_Z = %f\n", velocity[2]);

            // printf("\nPOSITION_X = %f\n", position[0]);
            // printf("POSITION_Y = %f\n", position[1]);
            // printf("POSITION_Z = %f\n", position[2]);
        }

        if (sample_num % 100 == 0) {
            if (abs(velocity[0]) > 1) {
                if (velocity[0] > 0)
                    printf("%5s ", "left");
                else printf("%5s ", "right");
            } else printf("still ");

            if (abs(velocity[1]) > 1) {
                if (velocity[1] > 0)
                    printf("%5s\n", "back");
                else printf("%5s\n", "front");
            } else printf("still\n");

            printf("\n");
        }
        

        // BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        // float x = pDataXYZ[0]*SCALE_MULTIPLIER, y = pDataXYZ[1]*SCALE_MULTIPLIER, z = pDataXYZ[2]*SCALE_MULTIPLIER;
        // pc.printf("(%f, %f, %f)\n", x, y, z);
        // int len = sprintf(acc_json,"{\"x\":%f,\"y\":%f,\"z\":%f,\"s\":%d}",(float)((int)(x*10000))/10000,
        //                                 (float)((int)(y*10000))/10000, (float)((int)(z*10000))/10000, sample_num);

            
        // response = socket.send(acc_json,len);
        // if (0 >= response){
        //     printf("Error seding: %d\n", response);
        // }
        wait(TIMESTEP);
    
        calculate(pGyroDataXYZ, pAccDataXYZ);
        
    }

 
    // socket.close();
}

int main()
{
    pc.baud(115200);

    // printf("\nConnecting to %s...\n", MBED_CONF_APP_WIFI_SSID);
    //wifi.set_network("192.168.130.105","255.255.255.0","192.168.130.254");
    // int ret = wifi.connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    // if (ret != 0) {
    //     printf("\nConnection error\n");
    //     return -1;
    // }

    // printf("Success\n\n");
    // printf("MAC: %s\n", wifi.get_mac_address());
    // printf("IP: %s\n", wifi.get_ip_address());
    // printf("Netmask: %s\n", wifi.get_netmask());
    // printf("Gateway: %s\n", wifi.get_gateway());
    // printf("RSSI: %d\n\n", wifi.get_rssi());


    pc.printf("Program start\n");
    BSP_ACCELERO_Init();    
    BSP_GYRO_Init();

    // acc_server(&wifi);
    acc_server();


}
