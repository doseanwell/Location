#include "Location.h"
#include "TestXgboostDetector.h"
#include "XgboostDetector.h"
#include "config/Config.h"
#include "math/Optimizer.h"
#include "math/Quaternions.h"
#include "sensor/GPS.h"
#include "tcp_interface.h"
#include "test/TestCalibration.h"
#include "test/TestLocation.h"
#include "test/utils/DataFormat.h"
#include <Eigen/Dense>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace Eigen;
using namespace std;

#define SERVER_IP "219.137.207.71"
// #define SERVER_IP "172.16.23.106"
#define MULTI_NAV_BUFF_NUM 1 * 1024 * 1024  // socket接收和发送缓存大小

enum {
    ALGOR_DATA_INDEX_ENUM = 0,  // 组合导航整合数据服务端，输出给算法
    ALGOR_BLH_INDEX_ENUM  = 1,  // 组合导航解算数据服务端，算法输出给服务端，以转发给手持端
} TCP_SERVER_TYPE_ENUM;

typedef struct {
    // uint32_t data_len;  //循环缓存的字节大小
    uint32_t head;
    uint32_t tail;
    uint8_t  data_buf[MULTI_NAV_BUFF_NUM];
    bool     writing; /*1为在向databuf中写数据，在写其它地方不能在写*/
} circle_buff_type;   /*循环部分*/

typedef struct {
    bool flag;  //设置标志 false 未设置 true 已设置
    // int32_t index;    //创建的服务器偏移,初始值记录，正常与index_s值一致
    // int32_t index_s;  //创建的服务器偏移
    int32_t index_c[TCP_CLIENT_NUM_MAX];  //连接客户端的偏移，支持多连接

    circle_buff_type circle_buff_recv[TCP_CLIENT_NUM_MAX];
    circle_buff_type circle_buff_send[TCP_CLIENT_NUM_MAX];
} tcp_client_index_type;

uint32_t nav_data_len          = 0;
uint32_t sol_data_len          = 0;
uint32_t nav_data_client_index = 0;
uint32_t sol_data_client_index = 0;

// tcp_client_index_type tcp_client_imu_data = { 0 };
// tcp_client_index_type tcp_client_blh_data = { 0 };

tcp_client_index_type tcp_client_index = { 0 };

void service_handler(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len)
{

    // printf("index_s is:%d;index_c is:%d\n", index_s, index_c);
    // printf("dat is:%s\n", dat);

    // if (index_s < 0) {  //小于0标志为客户端连接服务器接收服务
    //     if (index_c == tcp_client_imu_data.index_c) {
    //         nav_data_len += len;
    //         printf("nav_data_len is:%d\n", nav_data_len);
    //     }
    //     else if (index_c == tcp_client_blh_data.index_c) {
    //         sol_data_len += len;
    //         printf("sol_data_len is:%d\n", sol_data_len);
    //     }
    // }
    // else {
    //     printf("tcp recv error!\n");
    // }

    uint32_t i;
    printf("index_s is:%d;index_c is:%d\n", index_s, index_c);
    printf("dat is:%s\n", dat);
    // printf("tcp_client_index.index_c[index_c]:%d\n", tcp_client_index.index_c[index_c]);
    if (tcp_client_index.index_c[index_c] >= 0) {
        for (i = 0; i < len; i++) {
            tcp_client_index.circle_buff_recv[index_c].data_buf[tcp_client_index.circle_buff_recv[index_c].head++] =
                dat[i];
            if (tcp_client_index.circle_buff_recv[index_c].head >= MULTI_NAV_BUFF_NUM) {
                tcp_client_index.circle_buff_recv[index_c].head = 0;
            }
        }
        // if (index_s == tcp_server_index[IMU_INDEX_ENUM].index_s) {
        //     // printf("dat is:%s\n", dat);
        //     multi_nav_send_data(tcp_server_index[ALGOR_DATA_INDEX_ENUM].index_s,
        //                         tcp_server_index[ALGOR_DATA_INDEX_ENUM].index_c[0], dat,
        //                         len);  //转发数据到算法 ,当前只发送第一个连接的客户端
        // }
        // else {
        //     multi_nav_send_data(index_s, index_c, (uint8_t *)"I got it!\n", sizeof("I got it!\n"));
        // }
    }
}
void multi_nav_send_data(int32_t index_s, int32_t index_c, const uint8_t *dat, int32_t len)
{
    while (tcp_client_index.circle_buff_send[index_c].writing == 1)  //等待前面的写入完成
    {
        usleep(1000);  // 1ms延时
    }
    tcp_client_index.circle_buff_send[index_c].writing = 1;
    //  memset(tcp_client->client_send->buff, 0, ONECMD_MAX_LENTH); //清空
    if (tcp_client_index.circle_buff_send[index_c].head + len < MULTI_NAV_BUFF_NUM) {
        memcpy(&tcp_client_index.circle_buff_send[index_c].data_buf[tcp_client_index.circle_buff_send[index_c].head],
               dat, len);
        tcp_client_index.circle_buff_send[index_c].head += len;
    }
    else {
        memcpy(&tcp_client_index.circle_buff_send[index_c].data_buf[tcp_client_index.circle_buff_send[index_c].head],
               dat, MULTI_NAV_BUFF_NUM - tcp_client_index.circle_buff_send[index_c].head);
        memcpy(&tcp_client_index.circle_buff_send[index_c].data_buf[0],
               &dat[MULTI_NAV_BUFF_NUM - tcp_client_index.circle_buff_send[index_c].head],
               len - (MULTI_NAV_BUFF_NUM - tcp_client_index.circle_buff_send[index_c].head));
        tcp_client_index.circle_buff_send[index_c].head =
            len - (MULTI_NAV_BUFF_NUM - tcp_client_index.circle_buff_send[index_c].head);
    }
    tcp_client_index.circle_buff_send[index_c].writing = 0;
}
static void *tcp_send_thread(void *arg)
{
    int32_t  i, j;
    int32_t  index_s                      = *(int32_t *)arg;
    uint32_t tail_tmp[TCP_CLIENT_NUM_MAX] = { 0 };

    bool     data_flag  = false;  //数据标志，有收到数据置位
    uint32_t time_count = 0;
    uint32_t head_bak;

    *(int32_t *)arg = -1;

    while (1) {
        if (data_flag) {
            usleep(1000);  //有数据那就1ms延时
        }
        else {
            usleep(100000);  //没有数据那就100ms延时
        }

        for (j = 0; j < TCP_CLIENT_NUM_MAX; j++) {
            // if (tail_tmp[i][j] != tcp_client_index.circle_buff_send[j].head) {
            //     tcp_send_data(i,j,tcp_client_index.circle_buff_send[j].data_buf,);
            // }

            if (tcp_client_index.circle_buff_send[j].writing == 0) {

                if (tcp_client_index.circle_buff_send[j].tail != tcp_client_index.circle_buff_send[j].head) {
                    data_flag = true;
                    head_bak  = tcp_client_index.circle_buff_send[j].head;
                    if (tcp_client_index.circle_buff_send[j].tail < head_bak) {
                        if (tcp_send_data(-1, tcp_client_index.index_c[j],
                                          &tcp_client_index.circle_buff_send[j]
                                               .data_buf[tcp_client_index.circle_buff_send[j].tail],
                                          head_bak - tcp_client_index.circle_buff_send[j].tail)
                            < 0) {
                            printf("PID %d Send tcp data error!\r\n", getpid());
                            break;
                        }
                    }
                    else {
                        if (tcp_send_data(-1, tcp_client_index.index_c[j],
                                          &tcp_client_index.circle_buff_send[j]
                                               .data_buf[tcp_client_index.circle_buff_send[j].tail],
                                          MULTI_NAV_BUFF_NUM - tcp_client_index.circle_buff_send[j].tail)
                            < 0) {
                            printf("PID %d Send tcp data error!\r\n", getpid());
                            break;
                        }
                        usleep(100);  //延时100us
                        if (tcp_send_data(-1, tcp_client_index.index_c[j],
                                          &tcp_client_index.circle_buff_send[j].data_buf[0], head_bak)
                            < 0) {
                            printf("PID %d Send tcp data error!\r\n", getpid());
                            break;
                        }
                    }
                    tcp_client_index.circle_buff_send[j].tail = head_bak;
                }
                else {
                    if (time_count++ > 20)  //当运行20次后还是没有读到数据，那么设置data_flag标志为ｆａｌｓｅ
                    {
                        time_count = 0;
                        data_flag  = false;
                    }
                }
            }
        }
    }
}
static int32_t create_tcp_send_thread(int32_t index_s)
{
    pthread_t tid;
    int32_t   index_tmp = index_s;
    if (pthread_create(&tid, NULL, tcp_send_thread, &index_tmp) != 0) {
        printf("Create tcp send thread error!\r\n");
        return -1;
    }
    else {
        printf("Create tcp send thread Success!\r\n");
    }
    while (index_tmp != -1) {
        usleep(10000);
    }  //等待线程启动完成

    return 0;
}
int main()
{
    ifstream   infile;
    string     sensor_data;
    DataFormat dataFormat;
    Location   location;
    Vector3d   gyro_data_v(3), acc_data_v(3), mag_data_v(3), g_data_v(3), ornt_data_v(3), road_data(3);
    VectorXd   gps_data_v(7);
    string     file_path = "/home/sean/workspace/project/multi-nav/location/test/data/test_data.csv";

    int32_t index_tmp = -1;

    index_tmp = tcp_client_init((uint8_t *)SERVER_IP, 10002, NULL);
    if (index_tmp >= 0) {
        tcp_client_index.index_c[ALGOR_DATA_INDEX_ENUM] = index_tmp;
    }
    else {
        printf("client connect error!\n");
    }

    index_tmp = tcp_client_init((uint8_t *)SERVER_IP, 10003, service_handler);
    if (index_tmp >= 0) {
        tcp_client_index.index_c[ALGOR_BLH_INDEX_ENUM] = index_tmp;
    }
    else {
        printf("client connect error!\n");
    }

    create_tcp_send_thread(0);

    // cout.flags(ios::fixed);
    // cout.precision(8);  // 设置输出精度

    // // cout << "read file: " << file_path << endl;

    // // 数据读取
    // infile.open(file_path, ios::in);
    // assert(infile.is_open());

    // while (getline(infile, sensor_data)) {

    //     // 数据解析赋值
    //     dataFormat.readCSVOne(sensor_data, gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v,
    //                           road_data);

    //     g_data_v[0] = 0.0;
    //     g_data_v[1] = 0.0;
    //     g_data_v[2] = 0.0;

    //     mag_data_v[0] = 0.0;
    //     mag_data_v[1] = 0.0;
    //     mag_data_v[2] = 0.0;

    //     ornt_data_v[0] = 0.0;
    //     ornt_data_v[1] = 0.0;
    //     ornt_data_v[2] = 0.0;

    //     road_data[0] = 0.0;
    //     road_data[1] = 0.0;
    //     road_data[2] = 0.0;
    //     // 惯导计算
    //     location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v,
    //                                     road_data);
    //     // 结果输出
    //     cout << "Current predict result: lng " << location.GetGNSSINS().lng << ", lat " << location.GetGNSSINS().lat
    //          << endl;
    // }
    while (1) {
        multi_nav_send_data(-1, tcp_client_index.index_c[ALGOR_BLH_INDEX_ENUM],
                            (uint8_t *)"23.14982784,113.32067760\r\n", sizeof("23.14982784,113.32067760\r\n"));
        sleep(1);
    }

    return 0;
}