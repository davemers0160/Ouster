
#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#include <win_network_fcns.h>
#include <cstdint>

#elif defined(__linux__) | defined(__APPLE__)
#include <linux_network_fcns.h>
#include <pthread.h>

#include <cstdint>
typedef int32_t SOCKET;

#endif

#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <thread>
#include <algorithm>
#include <numeric>

// OpenCV Includes
#include <opencv2/core/core.hpp>           
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>     
#include <opencv2/imgproc/imgproc.hpp> 

// Custum Includes
#include "os1_packet.h"
//#include "os1_991827000195.h"
#include "os1_991838000603.h"
#include "time_median.h"
#include "sleep_ms.h"
#include "num2string.h"
#include "file_parser.h"
#include "get_current_time.h"
#include "write_binary_image.h"
#include "modulo.h"

namespace OS1 = ouster::OS1;

//-------------------------------GLOBALS---------------------------------------
bool running = false;
const uint32_t packet_size = 12608;
const uint32_t lidar_width = 2048;
const uint32_t lidar_height = OS1::pixels_per_column;

cv::Mat range_px = cv::Mat(lidar_height, lidar_width, CV_32SC1, cv::Scalar::all(0));
cv::Mat range_data = cv::Mat(lidar_height, lidar_width, CV_32FC1, cv::Scalar::all(0));
cv::Mat reflect_px = cv::Mat(lidar_height, lidar_width, CV_32SC1, cv::Scalar::all(0));

//cv::Mat_<uint32_t> lidar_range_map = cv::Mat_<uint32_t>(lidar_height, lidar_width, (const uint32_t)0);
//cv::Mat lidar_azimuth = cv::Mat(lidar_height, 1, CV_32FC1, cv::Scalar::all(0));

std::vector<uint32_t> index_tracker(lidar_width);
std::vector<ouster::OS1::trig_table_entry> trig_table;
//-----------------------------------------------------------------------------

bool config_lidar(std::string lidar_ip_address, uint32_t config_port, std::string lidar_port, std::string imu_port, std::string ip_address, std::vector<std::string> &lidar_info, std::string &error_msg)
{
    uint32_t result;
    std::string operation;
    std::string value;
    std::string parameter;
    std::string message;
    std::string rx_message;
    bool success = true;

    lidar_info.clear();
    error_msg = "";

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
    SOCKET os1_cfg_socket;
    result = init_tcp_socket(lidar_ip_address, config_port, os1_cfg_socket, error_msg);
    if (result != 0)
    {
        //std::cout << "Error Code (" << result << "): " << error_msg << std::endl;
        error_msg = "Error Code " + num2str(result, "(%03d): ") + error_msg + "\n";
        return false;
    }

    // get the basic info about the lidar unit
    operation = "get_sensor_info";
    result = send_message(os1_cfg_socket, operation, message);
    receive_message(os1_cfg_socket, 312, rx_message);
    if (rx_message.length() > 0)
    {
        std::vector<std::string> params, params2;
        parseCSVLine(rx_message, params);
        for (uint32_t idx = 0; idx < params.size()-1; ++idx)
        {
            parse_line(params[idx], ':', params2);
            std::string info = params2[1];
            lidar_info.push_back(info.substr(1, info.length() - 2));
        }
    }

    // begin configuring the lidar
    // This is set up now for firmware v1.9+

    // setup the lidar resolution
    operation = "set_config_param";
    parameter = "lidar_mode";
    value = "2048x10";

    result = send_message(os1_cfg_socket, (operation + " " + parameter + " " + value), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + operation + " " + parameter + " did not match\n";
    }

    // set the UDP IP address
    parameter = "udp_ip";
    result = send_message(os1_cfg_socket, (operation + " " + parameter + " " + ip_address), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + operation + " " + parameter + " did not match\n";
    }    
    
    // set the LIDAR Port
    parameter = "udp_port_lidar";
    result = send_message(os1_cfg_socket, (operation + " " + parameter + " " + lidar_port), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + operation + " " + parameter + " did not match\n";
    }

    // set the IMU Port
    parameter = "udp_port_imu";
    result = send_message(os1_cfg_socket, (operation + " " + parameter + " " + imu_port), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + operation + " " + parameter + " did not match\n";
    }
   
    // set the window_rejection_enable NARROW or STANDARD -> default STNADARD
    // removed in firmware update 1.10.0
    //parameter = "pulse_mode";
    //result = send_message(os1_cfg_socket, (operation + " " + parameter + " " + "STANDARD"), message);
    //receive_message(os1_cfg_socket, 64, rx_message);
    //if (rx_message != operation)
    //{
    //    success &= false;
    //    error_msg = error_msg + operation + " " + parameter + " did not match\n";
    //}

    // set the window_rejection_enable -> defalut "1"
    parameter = "window_rejection_enable";
    result = send_message(os1_cfg_socket, (operation + " " + parameter + " " + "0"), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + operation + " " + parameter + " did not match\n";
    }

    // write the config txt
    operation = "write_config_txt";
    result = send_message(os1_cfg_socket, (operation), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + operation + " did not match\n";
    }

    // reinitialize the LIDAR unit witht eh changes
    operation = "reinitialize";
    result = send_message(os1_cfg_socket, (operation), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + operation + " did not match\n";
    }

    result = close_connection(os1_cfg_socket, error_msg);

#else
    // linux code
    SOCKET os1_cfg_socket;
    result = init_tcp_socket(lidar_ip_address, config_port, os1_cfg_socket, error_msg);
    if (result != 0)
    {
        error_msg = "Error Initializing TCP Socket: " + num2str(result, "(%03d): ") + error_msg + "\n";
        return false;
    }
    
    // get the basic info about the lidar unit
    operation = "get_sensor_info";
    result = send_message(os1_cfg_socket, operation, message);
    receive_message(os1_cfg_socket, 305, rx_message);
    if (rx_message.length() > 0)
    {
        std::vector<std::string> params, params2;
        parseCSVLine(rx_message, params);
        for (uint32_t idx = 0; idx < params.size()-1; ++idx)
        {
            parse_line(params[idx], ':', params2);
            std::string info = params2[1];
            lidar_info.push_back(info.substr(1, info.length() - 2));
        }
    }

    // begin configuring the lidar
    operation = "set_udp_port_lidar";
    result = send_message(os1_cfg_socket, (operation + " " + lidar_port), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + "set_udp_port_lidar did not match\n";
    }

    operation = "set_udp_port_imu";
    result = send_message(os1_cfg_socket, (operation + " " + imu_port), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + "set_udp_port_imu did not match\n";
    }

    operation = "set_udp_ip";
    result = send_message(os1_cfg_socket, (operation + " " + ip_address), message);
    receive_message(os1_cfg_socket, 64, rx_message);
    if (rx_message != operation)
    {
        success &= false;
        error_msg = error_msg + "set_udp_ip did not match\n";
    }

    result = close_connection(os1_cfg_socket, error_msg);    
#endif

    return success;

}   // end of config_lidar

//-----------------------------------------------------------------------------

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
void get_lidar_packet(SOCKET s)
{
#else
void *get_lidar_packet(void *input)
{

    SOCKET *s;
    s = (SOCKET *)input;    
#endif

    uint8_t lidar_packet[packet_size+1];
    int32_t result, error, measurement_id;
    uint32_t encoder_count, reflect;
    uint32_t index = 0;
    uint32_t col_idx = 0;
    double range = 0,  x = 0.0, y = 0.0, z = 0.0;
    double theta = 0.0;

    const double pi_2_encoder = (2.0*M_PI) / ((double)OS1::encoder_ticks_per_rev);

    //uint32_t encoder;

    do 
    {
#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
        result = recvfrom(s, (char *)lidar_packet, packet_size, 0, NULL, NULL);
        if (result == SOCKET_ERROR) {
            error = WSAGetLastError();
            std::cout << "Receive failed. Error: " << error << std::endl;
            continue;
        }
#else
        result = recvfrom(*s, (char *)lidar_packet, packet_size, 0, NULL, NULL);
        if (result == -1) {
            std::cout << "Receive failed... " << std::endl;
            continue;
        }    
#endif

        for (uint32_t col = 0; col < 16; ++col)
        {
            col_idx = col * OS1::column_bytes;
            measurement_id = (int32_t)OS1::get_measurement_id(&lidar_packet[col_idx]);
            encoder_count = OS1::get_encoder_count(&lidar_packet[col_idx]);

            index_tracker[measurement_id] = 1;
            for (uint32_t row = 0; row < lidar_height; ++row)
            {
                index = col_idx + (row * OS1::pixel_bytes) + 16;
                range = (double)OS1::px_range(&lidar_packet[index]);
                reflect = OS1::px_reflectivity(&lidar_packet[index]);
                
                //theta = (pi_2_encoder*(double)encoder_count) + trig_table[row].h_offs;
                //x = range * cos(theta) * trig_table[row].sin_v_angle;
                //y = -range * sin(theta) * trig_table[row].cos_v_angle;
                //z = range * trig_table[row].sin_v_angle;
                //range_data.at<float>(row, (measurement_id + beam_azimuth_index[row] + 2048) % 2048) = (float)abs(x);

                range_px.at<int32_t>(row, (int32_t)mod((measurement_id + beam_azimuth_index[row]), 2048)) = (int32_t)range;
                reflect_px.at<int32_t>(row, (int32_t)mod((measurement_id + beam_azimuth_index[row]), 2048)) = (int32_t)reflect;
                //lidar_range_map.at<int32_t>(row, (measurement_id)) = (int32_t)range;
            }

        }

        sleep_ms(3);

    } while (running == true);

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#else
    pthread_exit(NULL);
#endif

}   // end of get_lidar_packet

//-----------------------------------------------------------------------------

int main(int argc, char *argv[]) 
{
    uint32_t idx;
    int32_t result;
    std::string error_msg;
    uint32_t capture_num = 11;
    uint32_t count = 0;
    std::string rng_capture_name = "lidar_rng_";
    std::string ref_capture_name = "lidar_ref_";
    std::string sdate, stime;
    std::ofstream DataLogStream;
    std::string log_filename = "lidar_capture_log_";
    std::vector<std::string> lidar_info;

    FILE *FP1, *FP2;

    cv::Mat lidar_combined_map = cv::Mat(2 * lidar_height, lidar_width, CV_32SC1, cv::Scalar::all(0));

    std::fill(index_tracker.begin(), index_tracker.end(), 0);

    const std::string params =
        "{help h ?    | | Help message }"
        "{rx_address  | 10.127.1.101 | IP Address for the lidar to send data to }"
        "{os1_address | 10.127.1.175 | IP address for the lidar }"
        "{cfg_port    | 7501 | TCP/IP Port for lidar configuration }"
        "{lidar_port  | 7502 | UDP Port to receive lidar Stream }"
        "{imu_port    | 7503 | UDP Port for the IMU data }"
        "{cfg_file    |  | Alternate input method to supply all parameters, all parameters must be included in the file }"
        "{avg         | 11 | Number of full lidar captures to average }"
        "{output      | ../results/ | Output directory to save lidar images }"
        ;

    // use opencv's command line parser
    cv::CommandLineParser parser(argc, argv, params);

    if (parser.has("help"))
    {
        parser.printMessage();
        std::cout << "Press enter to continue..." << std::endl;
        std::cin.ignore();
        return 0;
    }

    std::string rx_address;
    std::string os1_address;
    int32_t config_port, lidar_port, imu_port;
    std::string save_path;

    // if the input is a config file use this over all other input parameters
    if (parser.has("cfg_file"))
    {
        // input config file should contain all required inputs
        std::string cfg_filename = parser.get<std::string>("cfg_file");
        std::vector<std::vector<std::string>> cfg_params;
        parseCSVFile(cfg_filename, cfg_params);

        if (cfg_params.size() == 5)
        {
            os1_address = cfg_params[0][0];
            config_port = std::stoi(cfg_params[1][0]);
            lidar_port = std::stoi(cfg_params[1][1]);
            imu_port = std::stoi(cfg_params[1][2]);
            rx_address = cfg_params[2][0];
            capture_num = std::stoi(cfg_params[3][0]);
            save_path = cfg_params[4][0];
        }
        else
        {
            std::cout << "The number of supplied parameters in the file does not meet the required criteria: N = " << cfg_params.size() << std::endl;
            std::cin.ignore();
            return -1;
        }
    }
    else
    {
        rx_address = parser.get<std::string>("rx_address");
        os1_address = parser.get<std::string>("os1_address");
        config_port = parser.get<int32_t>("cfg_port");
        lidar_port = parser.get<int32_t>("lidar_port");
        imu_port = parser.get<int32_t>("imu_port");
        capture_num = parser.get<uint32_t>("avg");
        save_path = parser.get<std::string>("output");
    }

    path_check(save_path);

    get_current_time(sdate, stime);
    log_filename = log_filename + sdate + "_" + stime + ".txt";

    std::cout << "Log File: " << (save_path + log_filename) << std::endl << std::endl;
    DataLogStream.open((save_path + log_filename), ios::out | ios::app);

    // Add the date and time to the start of the log file
    DataLogStream << "------------------------------------------------------------------" << std::endl;
    DataLogStream << "Version: 1.0    Date: " << sdate << "    Time: " << stime << std::endl;
    DataLogStream << "------------------------------------------------------------------" << std::endl;


    std::cout << "Running lidar capture using the following parameters:" << std::endl;
    std::cout << "Lidar IP Address:       " << os1_address << std::endl;
    std::cout << "Config Port:            " << config_port << std::endl;
    std::cout << "Data Port:              " << lidar_port << std::endl;
    std::cout << "IMU Port:               " << imu_port << std::endl;
    std::cout << std::endl;
    std::cout << "Receiving IP Address:   " << rx_address << std::endl;
    std::cout << "Average Capture Number: " << capture_num << std::endl;
    std::cout << "Save Path:              " << save_path << std::endl;
    std::cout << std::endl;

    DataLogStream << "Running lidar capture using the following parameters:" << std::endl;
    DataLogStream << "Lidar IP Address:       " << os1_address << std::endl;
    DataLogStream << "Config Port:            " << config_port << std::endl;
    DataLogStream << "Data Port:              " << lidar_port << std::endl;
    DataLogStream << "IMU Port:               " << imu_port << std::endl;
    DataLogStream << std::endl;
    DataLogStream << "Receiving IP Address:   " << rx_address << std::endl;
    DataLogStream << "Average Capture Number: " << capture_num << std::endl;
    DataLogStream << "Save Path:              " << save_path << std::endl;

//-----------------------------------------------------------------------------
//  Main Program
//-----------------------------------------------------------------------------

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
    // LIDAR data
    SOCKET os1_data_socket;
    SOCKADDR_IN lidar_data_sock_add = {};
    lidar_data_sock_add.sin_family = AF_INET;
    lidar_data_sock_add.sin_addr.s_addr = INADDR_ANY;
    lidar_data_sock_add.sin_port = htons(lidar_port);

    // IMU data
    //SOCKET os1_imu_socket;
    //SOCKADDR_IN lidar_imu_sock_add = {};
    //lidar_imu_sock_add.sin_family = AF_INET;
    //lidar_imu_sock_add.sin_addr.s_addr = INADDR_ANY;
    //lidar_imu_sock_add.sin_port = htons(imu_port);
#else
    SOCKET os1_data_socket;
    SOCKET os1_imu_socket;

    pthread_t receiving;
    
#endif

    try
    {

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
        // config the lidar to start sending lidar packets
        std::cout << "Configuring LIDAR: ";
        bool success = config_lidar(os1_address, config_port, std::to_string(lidar_port), std::to_string(imu_port), rx_address, lidar_info, error_msg);
        if (success == false)
        {
            std::cout << "Failed!" << std::endl;
            std::cout << error_msg << std::endl;
            DataLogStream << error_msg << std::endl;
            DataLogStream.close();
            std::cin.ignore();
            return -1;
        }
        std::cout <<  "Successful!" << std::endl;
        
        if (lidar_info.size() > 3)
        {
            std::cout << "Connected to lidar sn: " << lidar_info[2] << std::endl;
            DataLogStream << "Lidar SN:               " << lidar_info[2] << std::endl;
        }
        DataLogStream << "------------------------------------------------------------------" << std::endl;

        std::cout << std::endl;

        // initialize the trig table
        init_trig_table(trig_table);


        // initialize the lidar socket stream
        std::cout << "Initializing Socket: ";
        result = init_udp_socket(lidar_port, lidar_data_sock_add, os1_data_socket, error_msg);
        if (result != 0)
        {
            std::cout << "Failed!" << std::endl;
            std::cout << "Error Code: " << result << " - " << error_msg << std::endl;
            DataLogStream << error_msg << std::endl;
            DataLogStream.close(); 
            std::cin.ignore();
            return -1;
        }

        std::cout << "Successful!" << std::endl << std::endl;

        running = true;
        std::thread receiving(get_lidar_packet, os1_data_socket);
        receiving.detach();

#else
        // config the lidar to start sending lidar packets
        bool success = config_lidar(os1_address, config_port, std::to_string(lidar_port), std::to_string(imu_port), rx_address, lidar_info, error_msg);
        if (success == false)
        {
            std::cout << "Lidar configuration failed..." << std::endl;
            std::cout << error_msg << std::endl;
            DataLogStream << error_msg << std::endl;
            DataLogStream.close();
            std::cin.ignore();
            return -1;
        }
        std::cout << "Configuring lidar successful!" << std::endl;
        if (lidar_info.size() > 3)
        {
            std::cout << "Connected to lidar sn: " << lidar_info[2] << std::endl;
            DataLogStream << "Lidar SN:               " << lidar_info[2] << std::endl;
        }
        DataLogStream << "------------------------------------------------------------------" << std::endl;

        std::cout << std::endl;

        // initialize the lidar socket stream
        result = init_udp_socket(lidar_port, os1_data_socket, error_msg);
        if (result != 0)
        {
            std::cout << "Error Code: " << result << " - " << error_msg << std::endl;
            DataLogStream << error_msg << std::endl;
            DataLogStream.close();
            std::cin.ignore();
            return -1;
        }

        std::cout << "Socket initialization successful." << std::endl << std::endl;

        running = true;
        int rx_thread = pthread_create(&receiving, NULL, get_lidar_packet, (void*)(&os1_data_socket));
        if(rx_thread == -1)
        {
            std::cout << "Error creating LIDAR data thread." << std::endl;
        }
        pthread_detach(receiving);
        //std::thread receiving(get_lidar_packet, os1_data_socket);
        //receiving.detach();

#endif
        double ref_scale = 1.0 / 20.0;
        double rng_scale = 1.0 / 30.0;
        char key;

        std::string depthmapWindow = "Lidar Reflectivity/Range Map";
        cv::namedWindow(depthmapWindow, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);

        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "Press the following keys to perform actions:" << std::endl;
        std::cout << "  s - Save an image" << std::endl;
        std::cout << "  q - Quit" << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        do
        {
            cv::Mat cm_reflect, cm_range;
            cv::Rect ROI_Rect = cv::Rect(cv::Point(905, 6), cv::Size(233,56));

            reflect_px.convertTo(cm_reflect, CV_8UC1, ref_scale, 0);
            range_px.convertTo(cm_range, CV_8UC1, rng_scale, 0);
            //range_data.convertTo(cm_range, CV_8UC1, rng_scale, 0);

            cv::vconcat(cm_reflect, cm_range, lidar_combined_map);
            cv::applyColorMap(lidar_combined_map, lidar_combined_map, cv::COLORMAP_JET);

            //cm_range(ROI_Rect).copyTo(cm_range);
            //cv::applyColorMap(cm_range, lidar_combined_map, cv::COLORMAP_JET);

            cv::imshow(depthmapWindow, lidar_combined_map);    //18000
            key = cv::waitKey(1);


            // check to save the image
            if (key == 's')
            {
                cv::Mat sum_image = cv::Mat(cv::Size(lidar_width, lidar_height), CV_32SC1, cv::Scalar::all(0));
                //cv::Mat sum_image = cv::Mat(cv::Size(lidar_width, lidar_height), CV_32FC1, cv::Scalar::all(0));

                std::vector<cv::Mat> rng_tm, ref_tm;
                get_current_time(sdate, stime);
                std::string rng_save_name = save_path + rng_capture_name + num2str(count, "%05d_") + sdate + "_" + stime + ".bin";
                std::string ref_save_name = save_path + ref_capture_name + num2str(count, "%05d_") + sdate + "_" + stime + ".bin";
                std::cout << "Saving range data to: " << rng_save_name << std::endl;
                std::cout << "Saving reflectivity data to: " << ref_save_name << std::endl;
                DataLogStream << rng_save_name << std::endl;
                DataLogStream << ref_save_name << std::endl;
                DataLogStream << "------------------------------------------------------------------" << std::endl;

                FP1 = fopen(rng_save_name.c_str(), "wb");
                FP2 = fopen(ref_save_name.c_str(), "wb");

                std::cout << ":";
                for (idx = 0; idx < capture_num; ++idx)
                {
                    std::cout << ".";
                    reflect_px.convertTo(cm_reflect, CV_8UC1, ref_scale, 0);
                    range_px.convertTo(cm_range, CV_8UC1, rng_scale, 0);
                    //range_data.convertTo(cm_range, CV_8UC1, rng_scale, 0);

                    cv::vconcat(cm_reflect, cm_range, lidar_combined_map);

                    //cv::medianBlur(cm_lidar, cm_lidar, 3);
                    cv::applyColorMap(lidar_combined_map, lidar_combined_map, cv::COLORMAP_JET);

                    cv::imshow(depthmapWindow, lidar_combined_map);    //18000
                    cv::waitKey(1);

                    int32_t tracker_sum = std::accumulate(index_tracker.begin(), index_tracker.end(), (int32_t)0);
                    // wait for a collect of all lidar azimuth points
                    while (tracker_sum < lidar_width)
                    {
                        tracker_sum = std::accumulate(index_tracker.begin(), index_tracker.end(), (int32_t)0);
                    }

                    //rng_tm.push_back(range_px.clone());
                    //rng_tm.push_back(range_data.clone());
                    ref_tm.push_back(reflect_px.clone());

                    cv::add(sum_image, range_px, sum_image, cv::Mat(), CV_32SC1);
                    //cv::add(sum_image, range_data, sum_image, cv::Mat(), CV_32FC1);

                    // reset the tracker
                    std::fill(index_tracker.begin(), index_tracker.end(), 0);

                }

                cv::Mat rng_tm2, ref_tm2;

                std::cout << ":" << std::endl;
                std::cout << "------------------------------------------------------------------" << std::endl;
                
                sum_image.convertTo(rng_tm2, CV_32SC1, (1 / (double)capture_num));
                //sum_image.convertTo(rng_tm2, CV_32FC1, (1 / (double)capture_num));
                //cv::medianBlur(sum_image, sum_image, 3);

                //time_median_cv(rng_tm, rng_tm2);
                time_median_cv(ref_tm, ref_tm2);

                // write the size of the data
                fwrite(&rng_tm2.rows, sizeof(uint32_t), 1, FP1);
                fwrite(&rng_tm2.cols, sizeof(uint32_t), 1, FP1);
                fwrite(&ref_tm2.rows, sizeof(uint32_t), 1, FP2);
                fwrite(&ref_tm2.cols, sizeof(uint32_t), 1, FP2);

                // write the data
                fwrite(rng_tm2.data, sizeof(int32_t), rng_tm2.rows*rng_tm2.cols, FP1);
                //fwrite(rng_tm2.data, sizeof(float), rng_tm2.rows*rng_tm2.cols, FP1);
                fwrite(ref_tm2.data, sizeof(int32_t), ref_tm2.rows*ref_tm2.cols, FP2);

                fclose(FP1);
                fclose(FP2);

                //write_binary_image(rng_save_name.c_str(), rng_tm2);

                ++count;
                int bp = 0;
            }

        } while (key != 'q');

        running = false;

        sleep_ms(1000);         // delay for a second to let the lidar thread come to a complete stop

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
        receiving.~thread();
#else
        pthread_join(receiving, NULL);
#endif

    }
    catch (std::exception e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        DataLogStream << "Error: " << e.what() << std::endl;

        std::cin.ignore();
    }

    DataLogStream.close();

    cv::destroyAllWindows();
    std::cout << "Program Complete!" << std::endl;
    return 0;
}   // end of main


//int main(int argc, char** argv)
//{
//    struct sockaddr_in si_other;
//    int s, slen = sizeof(si_other);
//    char buf[BUFLEN];
//    char message[BUFLEN];
//    WSADATA wsa;
//
//
//    // Validate the parameters
//    if (argc != 3) 
//    {
//        printf("usage: %s server-name port\n", argv[0]);
//        return 1;
//    }
//
//    std::string address = argv[1];
//    int port = std::stoi(argv[2]);
//
//    //Initialise winsock
//    printf("\nInitialising Winsock...");
//    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
//    {
//        printf("Failed. Error Code : %d", WSAGetLastError());
//        exit(EXIT_FAILURE);
//    }
//    printf("Initialised.\n");
//
//    //create socket
//    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
//    {
//        printf("socket() failed with error code : %d", WSAGetLastError());
//        exit(EXIT_FAILURE);
//    }
//
//    //setup address structure
//    memset((char *)&si_other, 0, sizeof(si_other));
//    si_other.sin_family = AF_INET;
//    si_other.sin_port = htons(port);
//    si_other.sin_addr.S_un.S_addr = inet_addr(address.c_str());
//
//
//    result = ::bind(s, (SOCKADDR*)&srcaddr, sizeof(srcaddr));
//    if (result == SOCKET_ERROR) {
//        error = WSAGetLastError();
//        std::cout << "Socket bind failed. Error: " << error << std::endl;
//        exit(1);
//    }
//
//
//
//    //start communication
//    while (1)
//    {
//
//        printf("Enter message : ");
//        /*
//        fgets(message, 512, stdin);
//
//        //send the message
//        if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
//        {
//            printf("sendto() failed with error code : %d", WSAGetLastError());
//            exit(EXIT_FAILURE);
//        }
//        */
//        //receive a reply and print it
//        //clear the buffer by filling null, it might have previously received data
//        memset(buf, '\0', BUFLEN);
//        //try to receive some data, this is a blocking call
//        if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) == SOCKET_ERROR)
//        {
//            printf("recvfrom() failed with error code : %d", WSAGetLastError());
//            exit(EXIT_FAILURE);
//        }
//
//        puts(buf);
//    }
//
//    closesocket(s);
//    WSACleanup();
//
//    return 0;
//}

//#define _CRT_SECURE_NO_WARNINGS

//#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
//
//#include <winsock2.h>
//#include <ws2tcpip.h>
//#include <windows.h>
//
//// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
//#pragma comment (lib, "Ws2_32.lib")
//#pragma comment (lib, "Mswsock.lib")
//#pragma comment (lib, "AdvApi32.lib")
//
//#endif
//
//#include <cstdint>
//#include <cstdlib>
//#include <cstdio>
//
//#define DEFAULT_BUFLEN 512
//#define DEFAULT_PORT "8888"
//
//int main(int argc, char** argv)
//{
//    
//    WSADATA wsaData;
//    SOCKET ConnectSocket = INVALID_SOCKET;
//    struct addrinfo *result = NULL, *ptr = NULL, hints;
//    char *sendbuf = "this is a test";
//    char recvbuf[DEFAULT_BUFLEN];
//    int iResult;
//    int recvbuflen = DEFAULT_BUFLEN;
//
//    // Validate the parameters
//    if (argc != 2) 
//    {
//        printf("usage: %s server-name\n", argv[0]);
//        return 1;
//    }
//
//    // Initialize Winsock
//    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
//    if (iResult != 0) 
//    {
//        printf("WSAStartup failed with error: %d\n", iResult);
//        return 1;
//    }
//    
//    ZeroMemory(&hints, sizeof(hints));
//    hints.ai_family = AF_UNSPEC;
//    hints.ai_socktype = SOCK_STREAM;
//    hints.ai_protocol = IPPROTO_TCP;
//
//    // Resolve the server address and port
//    iResult = getaddrinfo(argv[1], DEFAULT_PORT, &hints, &result);
//    if (iResult != 0) {
//        printf("getaddrinfo failed with error: %d\n", iResult);
//        WSACleanup();
//        return 1;
//    }
//
//    // Attempt to connect to an address until one succeeds
//    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
//
//        // Create a SOCKET for connecting to server
//        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
//            ptr->ai_protocol);
//        if (ConnectSocket == INVALID_SOCKET) {
//            printf("socket failed with error: %ld\n", WSAGetLastError());
//            WSACleanup();
//            return 1;
//        }
//
//        // Connect to server.
//        iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
//        if (iResult == SOCKET_ERROR) {
//            closesocket(ConnectSocket);
//            ConnectSocket = INVALID_SOCKET;
//            continue;
//        }
//        break;
//    }
//
//    freeaddrinfo(result);
//
//    if (ConnectSocket == INVALID_SOCKET) {
//        printf("Unable to connect to server!\n");
//        WSACleanup();
//        return 1;
//    }
//
//    // Send an initial buffer
//    iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
//    if (iResult == SOCKET_ERROR) {
//        printf("send failed with error: %d\n", WSAGetLastError());
//        closesocket(ConnectSocket);
//        WSACleanup();
//        return 1;
//    }
//
//    printf("Bytes Sent: %ld\n", iResult);
//
//    // shutdown the connection since no more data will be sent
//    iResult = shutdown(ConnectSocket, SD_SEND);
//    if (iResult == SOCKET_ERROR) {
//        printf("shutdown failed with error: %d\n", WSAGetLastError());
//        closesocket(ConnectSocket);
//        WSACleanup();
//        return 1;
//    }
//
//    // Receive until the peer closes the connection
//    do {
//
//        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
//        if (iResult > 0)
//            printf("Bytes received: %d\n", iResult);
//        else if (iResult == 0)
//            printf("Connection closed\n");
//        else
//            printf("recv failed with error: %d\n", WSAGetLastError());
//
//    } while (iResult > 0);
//
//    // cleanup
//    closesocket(ConnectSocket);
//    WSACleanup();
//
//    return 0;
//
//}   // end of main

