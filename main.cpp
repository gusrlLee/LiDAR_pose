#include<stdio.h>
#include<iostream>
#include<signal.h>
#include <unistd.h>

#include "opencv2/opencv.hpp"
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

void transformTheta(const float theta, double* output_array){
    double x_theta = 0;
    double y_theta = 0;

    if(0 <= theta && theta < 90){
        x_theta = sin(theta*(PI/180));
        y_theta = -1 * (cos(theta*(PI/180)));

    } else if(90 <= theta && theta < 180){
        x_theta = cos((theta - 90) * (PI/180));
        y_theta = sin((theta - 90) * (PI/180));

    } else if(180 <= theta && theta < 270) {
        x_theta = -1 * sin((theta - 180) * (PI/180));
        y_theta = cos((theta - 180.f) * (PI/180));

    } else {
        x_theta = -1 * cos((theta - 270) * (PI/180));
        y_theta = -1 * sin((theta - 270) * (PI/180));
    }

    output_array[0] = x_theta;
    output_array[1] = y_theta;

    return;
}

int main(int argc, char** argv){
    // Trap Ctrl-C
    signal(SIGINT, ctrlc);
    cv::Mat display_image;
    double theta_array[2] = {0};
    float temp = 0;

    std::cout << "LIDAR OpenCV Display_image for Slamtec RPLIDAR Device\n" << std::endl;
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
            display_image = cv::Mat::zeros(800, 800, CV_8UC3);  

            op_result = drv->grabScanDataHq(nodes, count);

            if (SL_IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
                for (int pos = 0; pos < (int)count ; ++pos) {
                    printf("theta: %03.2f Dist: %08.2f\n", (nodes[pos].angle_z_q14 * 90.f) / 16384.f, nodes[pos].dist_mm_q2/4.0f);
                    temp = nodes[pos].angle_z_q14*90.f/16384.f;
                    transformTheta(temp, theta_array);
                    
                    int x = int((nodes[pos].dist_mm_q2/4.0f) * theta_array[0] * PIXEL_RATIO);
                    int y = int((nodes[pos].dist_mm_q2/4.0f) * theta_array[1] * PIXEL_RATIO);
                    // std::cout << "x : " << x << " y : " << y << std::endl;
                    std::cout << "theta_array " << theta_array[0] << " " << theta_array[1] << "\n";
                    
                    cv::line(display_image, cv::Point(400, 400), cv::Point(x + 400, y + 400), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
                    cv::circle(display_image, cv::Point(x + 400, y + 400), 1, cv::Scalar(0, 0, 255), -1, cv::FILLED);
                }
            }

            cv::imshow("LIDAR view", display_image);
            if (ctrl_c_pressed){ 
                break;
            }
            int key = cv::waitKey(1);
            if(key == 27){
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
