/*
 * VL53MKAC.cpp
 * Author: Zhigang Wu
 * Date: 2018-01-12
*/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include "LIM/lim.h"
#include "VL53MKAC/VL53MKAC.h"
#include "console_bridge/console.h"
#include <string>

VL53MKAC::VL53MKAC()
: connected_(false)
, loaded_config_(false)
{
}

VL53MKAC::~VL53MKAC()
{
}

void VL53MKAC::setStructure(double height, double width)
{
  height_ = height;
  width_ = width;
}

void VL53MKAC::connect(std::string host_ip, int port)
{
  if (!connected_)
  {
    // Tạo socket
    std::cout <<"Creating socket." << std::endl;
    int socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ == -1)
    {
      std::cout << "Can not creat socket" << std::endl;
    }
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {
      // Kết nối tới server
      struct sockaddr_in server_address;
      server_address.sin_family = AF_INET;
      server_address.sin_port = htons(port);
      server_address.sin_addr.s_addr = inet_addr(host_ip.c_str()); // Địa chỉ IP của ESP32
      std::cout << "Connecting socket to laser."  << std::endl;
      int ret = ::connect(socket_fd_, (struct sockaddr *)&server_address, sizeof(server_address)) ;

      if (ret == 0)
      {
        connected_ = true;
        std::cout << "Connected succeeded." << std::endl;
      }

    }
  }


}

bool VL53MKAC::initializedLaserConfig()
{
  return loaded_config_;
}
void VL53MKAC::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool VL53MKAC::isConnected()
{
  return connected_;
}

void VL53MKAC::startMeas()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_START_LMD, NULL);
  // std::cout << "meas LIM_Pack finish" << std::endl;
  ssize_t temp;
  temp = write(socket_fd_, lim, lim->nLIMLen);
  // std::cout << "meas write finish" << std::endl;
  LIM_Release(lim);
}

void VL53MKAC::sendHB()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_HB, NULL);
  // std::cout << "sendHB LIM_Pack finish" << std::endl;
  ssize_t temp;
  temp = write(socket_fd_, lim, lim->nLIMLen);
  // std::cout << "sendHB write finish" << std::endl;
  LIM_Release(lim);
}


bool VL53MKAC::getConfig()
{
  int _cid = 1;
  LIM_HEAD *lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_GET_LDBCONFIG, NULL);
  // std::cout << "LIM_Pack finish" << std::endl;
  ssize_t temp;
  temp = write(socket_fd_, lim, lim->nLIMLen);
  // std::cout << "write finish" << std::endl;
  LIM_Release(lim);
  std::cout << "getConfig ok" << std::endl;
  return true;
}


bool VL53MKAC::GetData()
{
  // Nhận dữ liệu mảng từ server

  memset(data, 0, sizeof(data));
  const char* hello = "Hello from client#";
  char recv_buffer[10];

  int total_bytes = 0;
  send(socket_fd_, hello, strlen(hello), 0);
  printf("Hello message sent\n");
  // Xóa bộ đệm của hàm recv() trước khi nsaswwhận dữ liệu mới
  int bytes_received = 0;
  do
  {
    bytes_received = recv(socket_fd_, recv_buffer, 1, MSG_DONTWAIT);
    if (bytes_received <= 0)
    {
      continue;
    }
    std::cout << "Read " << bytes_received << " bytes, value is " << int(recv_buffer[0]) << std::endl;
  } while (recv_buffer[0] != '@');

  // Nhận dữ liệu mới
  while (total_bytes < sizeof(data))
  {

    int bytes_to_receive = sizeof(data) - total_bytes;
    int bytes_received = recv(socket_fd_, (uint16_t *)data + total_bytes, bytes_to_receive, 0);
    if (bytes_received <= 0)
    {
      std::cout << "Lỗi khi nhận dữ liệu" << std::endl;
      return false;
    }

    total_bytes += bytes_received;
    std::cout << "Read " << bytes_received << " bytes from fd, total lenght is " << total_bytes << std::endl;
  }

  std::cout << "Đã nhận dữ liệu mảng từ server" << std::endl;

  // Hiển thị dữ liệu mảng
  for (int i = 0; i < 64; i++)
  {
    std::cout << data[i] << " ";
  }
  std::cout << std::endl;

  return true;
}

// Convert range data to point clound msg
bool VL53MKAC::MsgDecoding(sensor_msgs::PointCloud2 *clound_data)
{

  std::cout << "Decode data to point clound" << std::endl;
  return true;
}

void VL53MKAC::stopMeas()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_STOP_LMD, NULL);

  ssize_t temp;
  temp = write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include <stdio.h>
// #include "vl53l5cx_api.h"
// #include "math.h"


// /* USER CODE BEGIN PV */
// int status;
// volatile int IntCount;
// uint8_t p_data_ready;
// uint8_t resolution, isAlive;
// uint16_t idx;

// const double VL53L5_Zone_Pitch8x8[64] = {
//     59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00,
//     64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
//     67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
//     70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
//     70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
//     67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
//     64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
//     59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00};

// const double VL53L5_Zone_Yaw8x8[64] = {
//     135.00, 125.40, 113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
//     144.60, 135.00, 120.96, 101.31, 78.69, 59.04, 45.00, 35.40,
//     156.80, 149.04, 135.00, 108.45, 71.55, 45.00, 30.96, 23.20,
//     171.87, 168.69, 161.55, 135.00, 45.00, 18.45, 11.31, 8.13,
//     188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
//     203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
//     203.20, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
//     225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00};

// PlaneEquation_t PlaneEquation;
// XYZ_ZoneCoordinates_t XYZ_ZoneCoordinates;

// double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];
// /* USER CODE END PV */

// /* Private function prototypes -----------------------------------------------*/
// /* USER CODE BEGIN PFP */

// uint8_t ComputeSinCosTables(void)
// {
//   // This function will save the math processing time of the code.  If the user wishes to not
//   // perform this function, these tables can be generated and saved as a constant.
//   uint8_t ZoneNum;
//   for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
//   {
//     SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum]) * Pi / 180);
//     CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum]) * Pi / 180);
//     SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum] * Pi / 180);
//     CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum] * Pi / 180);
//   }

//   return 0;
// }

// uint8_t ConvertDist2XYZCoords8x8(VL53L5CX_ResultsData *ResultsData, XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
// {
//   uint8_t ZoneNum;
//   float Hyp;
//   for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
//   {
//     if ((ResultsData->nb_target_detected[ZoneNum] > 0) && (ResultsData->distance_mm[ZoneNum] > 0) && ((ResultsData->target_status[ZoneNum] == 5) || (ResultsData->target_status[ZoneNum] == 6) || (ResultsData->target_status[ZoneNum] == 9)))
//     {
//       Hyp = ResultsData->distance_mm[ZoneNum] / SinOfPitch[ZoneNum];
//       XYZ_ZoneCoordinates->Xpos[ZoneNum] = CosOfYaw[ZoneNum] * CosOfPitch[ZoneNum] * Hyp;
//       XYZ_ZoneCoordinates->Ypos[ZoneNum] = SinOfYaw[ZoneNum] * CosOfPitch[ZoneNum] * Hyp;
//       XYZ_ZoneCoordinates->Zpos[ZoneNum] = ResultsData->distance_mm[ZoneNum];
//     }
//     else
//     {
//       XYZ_ZoneCoordinates->Xpos[ZoneNum] = 0;
//       XYZ_ZoneCoordinates->Ypos[ZoneNum] = 0;
//       XYZ_ZoneCoordinates->Zpos[ZoneNum] = 0;
//     }
//   }
//   return 0;
// }

// uint8_t PrintXYZCoords(XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
// {
//   uint8_t i, j;
//   printf("XYZ Coordinates for the target in each zone\n");
//   for (i = 0; i < 8; i++)
//   {
//     for (j = 0; j < 8; j++)
//     {
//       idx = (i * 8 + j);
//       printf("%5.0f, %5.0f, %5.0f |", XYZ_ZoneCoordinates->Xpos[idx], XYZ_ZoneCoordinates->Ypos[idx], XYZ_ZoneCoordinates->Zpos[idx]);
//     }
//     printf("\n");
//   }
//   printf("\n");

//   return 0;
// }