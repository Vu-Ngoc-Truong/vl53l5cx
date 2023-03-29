/*
 * VL53MKAC.h
*/

#ifndef VL53MKAC_H_
#define VL53MKAC_H_

#include <LIM/lim_buffer.h>
#include <string>
#include <stdint.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"


class VL53MKAC
{
public:
  VL53MKAC();
  virtual ~VL53MKAC();

  // configure the height and width of the point cloud
  void setStructure(double, double);

  // connect laser at certain port
  void connect(std::string host_ip, int port);

  // disconnect laser
  void disconnect();

  // load laser config
  bool initializedLaserConfig();

  // check laser connection
  bool isConnected();

  // start continous laser scan message
  void startMeas();

  // send laser Heart Beat
  void sendHB();

  // get laser config
  bool getConfig();

  // stop continous laser scan message
  void stopMeas();

  // Get laser data
  bool GetData();

  // laser lim struct: LMD laser scan message decode
  bool MsgDecoding(sensor_msgs::PointCloud2 *clound_data);

protected:
  bool connected_;
  bool loaded_config_;
  char buffer_[1024];
  uint16_t data[64];
  int socket_fd_;
  double height_;
  double width_;
};

#endif
