#include "Location.h"
#include "TestXgboostDetector.h"
#include "XgboostDetector.h"
#include "config/Config.h"
#include "math/Optimizer.h"
#include "math/Quaternions.h"
#include "sensor/GPS.h"
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

using namespace Eigen;
using namespace std;

// int main()
// {

//     Location location;
//     Vector3d gyro_data_v(0.004263, 0.019169, -0.001014);
//     Vector3d mag_data_v(-2.313675, -82.446960, -366.183838);
//     Vector3d acc_data_v(0.105081, 0.108075, 9.774973);
//     VectorXd gps_data_v(7);
//     gps_data_v << 114.174118, 22.283789, 0.0, 0.0, 24.0, 0.0, 1554348968704.665039;
//     Vector3d g_data_v(0.094139, 0.107857, 9.808955);
//     Vector3d ornt_data_v(-0.549866, 0.629957, -0.069398);
//     // Vector3d ornt_data_v(-0.0, 0.0, -0.0);
//     Vector3d road_data(0.0, 0.0, 0);
//     location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v,
//     road_data);
//     cout << location.GetGNSSINS().lng << " " << location.GetGNSSINS().lat << endl;
//     return 0;
// }

int main()
{
    ifstream   infile;
    string     sensor_data;
    DataFormat dataFormat;
    Location   location;
    Vector3d   gyro_data_v(3), acc_data_v(3), mag_data_v(3), g_data_v(3), ornt_data_v(3), road_data(3);
    VectorXd   gps_data_v(7);
    string     file_path = "/home/sean/workspace/project/multi-nav/location/test/data/test_data.csv";

    cout.flags(ios::fixed);
    cout.precision(8);  // 设置输出精度

    // cout << "read file: " << file_path << endl;

    // 数据读取
    infile.open(file_path, ios::in);
    assert(infile.is_open());

    while (getline(infile, sensor_data)) {

        // 数据解析赋值
        dataFormat.readCSVOne(sensor_data, gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v,
                              road_data);

        g_data_v[0] = 0.0;
        g_data_v[1] = 0.0;
        g_data_v[2] = 0.0;

        mag_data_v[0] = 0.0;
        mag_data_v[1] = 0.0;
        mag_data_v[2] = 0.0;

        ornt_data_v[0] = 0.0;
        ornt_data_v[1] = 0.0;
        ornt_data_v[2] = 0.0;

        road_data[0] = 0.0;
        road_data[1] = 0.0;
        road_data[2] = 0.0;
        // 惯导计算
        location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v,
                                        road_data);
        // 结果输出
        cout << "Current predict result: lng " << location.GetGNSSINS().lng << ", lat " << location.GetGNSSINS().lat
             << endl;
    }
    return 0;
}