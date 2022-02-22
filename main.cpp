#include<stdio.h>
#include<iostream>
#include<signal.h>
#include <unistd.h>

#include "config.h"
#include <rplidar.h>
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}


using namespace rp::standalone::rplidar;

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char** argv){
    // Trap Ctrl-C
    signal(SIGINT, ctrlc);

    std::cout << "LIDAR OpenCV Display for Slamtec RPLIDAR Device\n" << std::endl;
    // create the driver instance
    IChannel* _channel;
    sl_result     op_result;

    ILidarDriver * drv = *createLidarDriver();

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    _channel = (*createSerialPortChannel(PORT, BAUDRATE));
    
    if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result)) {
            connectSuccess = true;
        }
        else {
            delete drv;
            drv = NULL;
        }   
    }

    if(!connectSuccess){
        std::cout << "connect status: " << connectSuccess << std::endl;
        std::cerr << "CANNOT CONNECT PORT!!\n" << std::endl;
        exit(2);
    }

    // scan data
    if (checkSLAMTECLIDARHealth(drv)){
        drv->setMotorSpeed();   
        // Start scanning 
        printf("Start Scan LIDAR Now\n");
        drv->startScan(0, 1);

        while (1) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t   count = _countof(nodes);

            op_result = drv->grabScanDataHq(nodes, count);

            if (SL_IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
                for (int pos = 0; pos < (int)count ; ++pos) {
                    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                        (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                        (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                        nodes[pos].dist_mm_q2/4.0f,
                        nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                }
            }
            if (ctrl_c_pressed){ 
                break;
            }
        }
        
        // Shutdown
        drv->stop();
        drv->setMotorSpeed(0);
    }

    // free 
    if(drv)
        delete drv;
    drv = NULL;
    return 0;
}
