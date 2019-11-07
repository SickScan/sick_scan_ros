/**
* \file
* \brief Laser Scanner Main Handling
* Copyright (C) 2013,     Osnabrück University
* Copyright (C) 2017,2018 Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017,2018 SICK AG, Waldkirch
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrück University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  Last modified: 6th Nov 2019
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*
*
*/

// #include <sick_scan/sick_generic_DeviceConfig.h>
// #include <sick_scan/sick_generic_laser.h>


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define _USE_MATH_DEFINES

#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "SickDeviceConfig.h"


#include "SSBL.h"

using namespace std;
using namespace ssbl;
using namespace DevTiM5xxSkeleton;

static std::string versionInfo = "0.1";

ros::Publisher cloud_pub_;

const double DEG2RAD = 0.017453292;

double getDegrees(double angleInRad)
{
  const double rad2deg = 180.0 / M_PI;
  return angleInRad * rad2deg;
};


class Point
{
private:
  double x_, y_, z_, intensity_;  // coordinates of the point

public:
  Point()
  {
    X(0.0);
    Y(0.0);
    Z(0.0);
    Intensity(0.0);
  }

  Point(double x, double y)
  {
    Point();
    X(x);
    Y(y);
  }

  double X()
  { return x_; }

  double Y()
  { return y_; }

  double Z()
  { return z_; }

  double Intensity()
  { return intensity_; }

  void X(double x)
  { x_ = x; }

  void Y(double y)
  { y_ = y; }

  void Z(double z)
  { z_ = z; }

  void Intensity(double intensity)
  { intensity_ = intensity; }

  static void testbed()
  {
    Point p;
    p.X(10.0);
    p.Y(20.0);
    p.Z(30.0);
    p.Intensity(40.0);
    printf("Test: %8.3lf,%8.3lf,%8.3lf\n", p.X(), p.Y(), p.Z());
  }
};

void setVersionInfo(std::string _versionInfo)
{
  versionInfo = _versionInfo;
}

std::string getVersionInfo()
{

  return (versionInfo);
}

enum NodeRunState
{
  scanner_init, scanner_run, scanner_finalize
};

NodeRunState runState = scanner_init;  //


void signalRecv_handler(int signalRecv)
{
  ROS_INFO("Caught signal %d\n", signalRecv);
  ROS_INFO("good bye");
  ROS_INFO("You are leaving the following version of this node:");
  ROS_INFO("%s", getVersionInfo().c_str());
  runState = scanner_finalize;
}


// Callback function which will be triggered when
// scan data arrives
void OnScan(uint64_t *pEventData)
{
  ScanData_TiM5xxSkeleton_Var *pVar;
  SsblEventContainer *pEvent =
      reinterpret_cast<SsblEventContainer *>(pEventData);

  printf("Received Event\n");

  pVar = dynamic_cast<ScanData_TiM5xxSkeleton_Var *>(pEvent->pComObj);

  int32_t startAngle =
      pVar->Value_.aDataChannel16[0].DataChannelHdr.diStartAngle;

  uint32_t resolution =
      pVar->Value_.aDataChannel16[0].DataChannelHdr.uiAngleRes;

  size_t pointNum = pVar->Value_.aDataChannel16[0].uiLengthaData;


  std::vector<Point> pointVec;
  pointVec.resize(pointNum);

  float scaleFactor_001 =
      0.001F * pVar->Value_.aDataChannel16[0].DataChannelHdr.dScaleFactor;
  float scaleFactorOffset_001 =
      0.001F * pVar->Value_.aDataChannel16[0]
          .DataChannelHdr.dScaleOffset;  // Offset in [m]

  // convert to Cartesian coordinates in [m]
  for (uint16_t dIdx = 0; dIdx < pointNum; dIdx++)
  {
    double dist = pVar->Value_.aDataChannel16[0].aData[dIdx];

    dist = dist * scaleFactor_001 +
           scaleFactorOffset_001;  // Convert from distance units to [m]
    double angle = startAngle / 10000.0;
    angle = angle * DEG2RAD;
    angle -=
        M_PI /
        2.0;  // Working range diagram shows 90 degrees in direction of X-Axis
    pointVec[dIdx].X(dist * cos(angle));
    pointVec[dIdx].Y(dist * sin(angle));
    startAngle = startAngle + resolution;
  }


  int numChannels = 4;
  sensor_msgs::PointCloud2 cloud_;
  cloud_.header.stamp = ros::Time::now();
  cloud_.header.frame_id = "map";
  cloud_.header.seq = 0;
  cloud_.height = 1;
  cloud_.width = pointNum;
  cloud_.is_bigendian = false;
  cloud_.is_dense = true;
  cloud_.point_step = numChannels * sizeof(float);
  cloud_.row_step = cloud_.point_step * cloud_.width;
  cloud_.fields.resize(numChannels);

  for (int i = 0; i < numChannels; i++)
  {
    std::string channelId[] = {"x", "y", "z", "intensity"};
    cloud_.fields[i].name = channelId[i];
    cloud_.fields[i].offset = i * sizeof(float);
    cloud_.fields[i].count = 1;
    cloud_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud_.data.resize(cloud_.row_step * cloud_.height);

  unsigned char *cloudDataPtr = &(cloud_.data[0]);


  for (size_t i = 0; i < pointNum; i++)
  {
    float x = pointVec[i].X();
    float y = pointVec[i].Y();
    float z = pointVec[i].Z();

    enum enum_index_descr
    {
      idx_x,
      idx_y,
      idx_z,
      idx_intensity,
      idx_num
    };

    long adroff = i * (numChannels * (int) sizeof(float));
    float *fptr = (float *) (cloudDataPtr + adroff);

    fptr[idx_x] = x;
    fptr[idx_y] = y;
    fptr[idx_z] = z;

    fptr[idx_intensity] = 0.0;

  }
  // Publish


  delete pEventData;

  cloud_pub_.publish(cloud_);


  ros::spinOnce();
}


/*!
\brief Internal Startup routine.
\param argc: Number of Arguments
\param argv: Argument variable
\return exit-code
\sa main
*/
int main(int argc, char **argv)
{

  mStartMeasure_TiM5xxSkeleton_Func
      startFunction;  // Function which puts the Lidar into measurement mode
  Run_TiM5xxSkeleton_Func
      runFunction;  // Function which puts the Lidar into run mode
  DataOutputRange_TiM5xxSkeleton_Var
      orVariable;  // Variable which contains start angle, stop angle and angle
  // resolution
  ScanDataConfig_TiM5xxSkeleton_Var
      sdcVariable;  // Variable which contains the scan output configuration
  ScanData_TiM5xxSkeleton_Var scanDataVariable;  // Scan data variable
  std::vector<std::string> keys;


  char **argv_tmp; // argv_tmp[0][0] argv_tmp[0] identisch ist zu (*argv_tmp)
  int argc_tmp;
  argc_tmp = argc;
  argv_tmp = argv;

  const int MAX_STR_LEN = 1024;
  char nameTagVal[MAX_STR_LEN] = {0};
  char logTagVal[MAX_STR_LEN] = {0};
  char internalDebugTagVal[MAX_STR_LEN] = {0};
  char sensorEmulVal[MAX_STR_LEN] = {0};
  char nameId[] = "__name:=";
  char nameVal[MAX_STR_LEN] = {0};

  if (argc == 1) // just for testing without calling by roslaunch
  {
    // recommended call for internal debugging as an example: __name:=sick_rms_320 __internalDebug:=1
    // strcpy(nameTagVal, "__name:=sick_rms_3xx");  // sick_rms_320 -> radar
    strcpy(nameTagVal, "__name:=sick_tim_5xx");  // sick_rms_320 -> radar
    strcpy(logTagVal, "__log:=/tmp/tmp.log");
    strcpy(internalDebugTagVal, "__internalDebug:=1");
    // strcpy(sensorEmulVal, "__emulSensor:=1");
    strcpy(sensorEmulVal, "__emulSensor:=0");
    argc_tmp = 5;
    argv_tmp = (char **) malloc(sizeof(char *) * argc_tmp);

    argv_tmp[0] = argv[0];
    argv_tmp[1] = nameTagVal;
    argv_tmp[2] = logTagVal;
    argv_tmp[3] = internalDebugTagVal;
    argv_tmp[4] = sensorEmulVal;

  }

  bool dutInitialized = false;
  std::string nodeName = "sick_tim_5xx";
  ROS_INFO("%s", versionInfo.c_str());
  for (int i = 0; i < argc_tmp; i++)
  {
    if (strstr(argv_tmp[i], nameId) == argv_tmp[i])
    {
      strcpy(nameVal, argv_tmp[i] + strlen(nameId));
      nodeName = nameVal;
    }
    ROS_INFO("Program arguments: %s", argv_tmp[i]);
  }

  ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);  // scannerName holds the node-name
  signal(SIGINT, signalRecv_handler);

  ros::NodeHandle nhPriv("~");


  cloud_pub_ = nhPriv.advertise<sensor_msgs::PointCloud2>("cloud", 100);

  std::string scannerName = "sick_tim_5xx";

// check for TCP - use if ~hostname is set.
  std::string hostname;
  std::string port;

  nhPriv.getParam("hostname", hostname);

  nhPriv.param<std::string>("port", port, "2112");

  int timelimit;
  nhPriv.param("timelimit", timelimit, 5);

  bool subscribe_datagram;
  int device_number;
  nhPriv.param("subscribe_datagram", subscribe_datagram, false);
  nhPriv.param("device_number", device_number, 0);

  sick_scan::SickDeviceConfig *DeviceConfig = new sick_scan::SickDeviceConfig(scannerName);

  double param;

  if (nhPriv.getParam("max_ang", param))
  {
    DeviceConfig->getCurrentParamPtr()->setMaxAng(param);
  }
  if (nhPriv.getParam("min_ang", param))
  {
    DeviceConfig->getCurrentParamPtr()->setMinAng(param);
  }
  if (nhPriv.getParam("time_increment", param))
  {
    DeviceConfig->set_time_increment(param);
  }

  /*
   *  Check, if parameter for protocol type is set
   */
  bool use_binary_protocol = true;

  if (true == nhPriv.getParam("use_binary_protocol", use_binary_protocol))
  {
    ROS_INFO("Found sopas_protocol_type param overwriting default protocol:");
    if (use_binary_protocol == true)
    {
      ROS_INFO("Binary protocol activated");
    }
    else
    {
      if (DeviceConfig->getCurrentParamPtr()->getNumberOfLayers() > 4)
      {
        nhPriv.setParam("sopas_protocol_type", true);
        use_binary_protocol = true;
        ROS_WARN("This scanner type does not support ASCII communication.\n"
                 "Binary communication has been activated.\n"
                 "The parameter \"sopas_protocol_type\" has been set to \"True\".");
      }
      else
      {
        ROS_INFO("ASCII protocol activated");
      }
    }
    DeviceConfig->getCurrentParamPtr()->setUseBinaryProtocol(use_binary_protocol);
  }

  ROS_INFO("Start initialising scanner [Ip: %s] [Port: %s]", hostname.c_str(), port.c_str());



//===============================================================================
  // Step 1) Create a Lidar Skeleton (change the IP to your needs)
  //===============================================================================
  auto DUT = new TiM5xxSkeleton(hostname.c_str());
  dutInitialized = true;
  //===============================================================================
  // Step 2) Connect to the Lidar
  //===============================================================================
  if (SSBL_SUCCESS == DUT->Connect())
  {
    //===============================================================================
    // Step 3) Read the output range currently employed by the device
    // Further details on PAGE 56!
    //===============================================================================
    if (SSBL_SUCCESS == DUT->ReadVariable(orVariable))
    {
      // Now let us change start and stop angle to get a scan from 0-90�.
      // Note that the Lidar specific coordinate system is used in here.
      // Note angle resolution can not be changed for TiM5xx
      int32_t start_ang = (int32_t) round((getDegrees(DeviceConfig->getCurrentParamPtr()->getMinAng()) + 90) *
                                          10000);//Angel value is set90 deg rotated frame in Deg*10000 as int32
      int32_t stop_ang = (int32_t) round((getDegrees(DeviceConfig->getCurrentParamPtr()->getMaxAng()) + 90) * 10000);
      orVariable.Value_.aRange[0].diStartAngle = start_ang;
      orVariable.Value_.aRange[0].diStopAngle = stop_ang;
      printf("Angle resolution: %8.3lf degrees\n",
             (double) orVariable.Value_.aRange[0].udiAngleRes / 10000.00);

      printf("Start angle: %8.3lf degrees\n",
             (double) orVariable.Value_.aRange[0].diStartAngle / 10000.00);
      printf("Stop angle: %8.3lf degrees\n",
             (double) orVariable.Value_.aRange[0].diStopAngle / 10000.00);


      if (SSBL_SUCCESS == DUT->WriteVariable(orVariable))
      {
        // Set the variable to some different value to observe that it has
        // really been written from the device
        orVariable.Value_.aRange[0].diStartAngle = 1;
        orVariable.Value_.aRange[0].diStopAngle = 1;
        if ((SSBL_SUCCESS != DUT->ReadVariable(orVariable)) ||
            (start_ang !=
             orVariable.Value_.aRange[0].diStartAngle) ||
            (stop_ang != orVariable.Value_.aRange[0].diStopAngle))
        {
          printf("Error reading outputRange from the device");
          goto exit;
        }
        else
        {
          // Now you could store this configuration. Note that the embedded
          // flash might have limited number of write cycles. Therefore, one
          // should only store parameters if it's required.
        }
      }
      else
      {
        if (SSBL_SUCCESS != DUT->ReadVariable(orVariable))
        {
          printf("Error while setting the scan range");
          goto exit;
        }
      }
    }

    //===============================================================================
    // Step 4) Read the scan output configuration from the device
    // Further details on PAGE 55!
    //===============================================================================
    if (SSBL_SUCCESS == DUT->ReadVariable(sdcVariable))
    {
      sdcVariable.Value_.DistDataConfig[0] = 1;      // Enable output channel 1
      sdcVariable.Value_.DistDataConfig[1] = 0;      // (distance values)
      sdcVariable.Value_.RemDataConfig.bEnable = 1;  // Enable Remission / RSSI
      sdcVariable.Value_.RemDataConfig.eDataType = 0;  // 8Bit Remission values
      sdcVariable.Value_.RemDataConfig.eContentType =
          0;                                         // Digits there is no other
      sdcVariable.Value_.bEnableCommentBlock = 0;    // Comments? Pff
      sdcVariable.Value_.bEnableDeviceName = 0;      // No device name
      sdcVariable.Value_.bEnablePositionBlock = 1;   // include position data
      sdcVariable.Value_.bEnableTimeBlock = 1;       // include device timestamp
      sdcVariable.Value_.EnableEncoderBlock[0] = 0;  // no Encoder for TiM5xx
      sdcVariable.Value_.EnableEncoderBlock[1] = 0;  // no Encoder for TiM5xx

      sdcVariable.Value_.uiOutputInterval =
          1;  // no downsampling we want all scans
      if (SSBL_SUCCESS != DUT->WriteVariable(sdcVariable))
      {
        printf("Error while setting the scan output configuration");
        goto exit;
      }
    }

    //===============================================================================
    // Step 5.1) Put the Lidar to measurement mode
    // Further details on PAGE 32!
    //===============================================================================
    if (SSBL_SUCCESS != DUT->CallFunction(startFunction))
    {
      printf("Error when trying to put the Lidar into measurement mode");
      goto exit;
    }

    //===============================================================================
    // Step 5.2) Put the Lidar to run mode
    // Further details on PAGE 53!
    //===============================================================================
    if (SSBL_SUCCESS != DUT->CallFunction(runFunction))
    {
      printf("Error when trying to put the Lidar into run mode");
      goto exit;
    }

    //===============================================================================
    // Step 6) Register to scan events, OnScan is a callback function that will
    // be called as soon as the event arrives. You can add 32 bits user data
    // which shall be available when the callback is triggered. For now, we
    // simply add the scan config. Further details on PAGE 63!
    //===============================================================================
    if (SSBL_SUCCESS !=
        DUT->RegisterEvent("ScanData", OnScan,
                           NULL))
    {
      printf("Error when trying to register to scan events");
      goto exit;
    }

    runState = scanner_run;
    while (ros::ok())
    {
      //Infinity loop to do nothin when programm is running
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
    //===============================================================================
    // Disconnect from the Lidar
    //===============================================================================
    exit:
    if (dutInitialized)
    {
      DUT->Disconnect();
    }
  }

  delete DUT;

  return 0;


}


#if 0

//===============================================================================
  // Step 1) Create a Lidar Skeleton (change the IP to your needs)
  //===============================================================================
  auto DUT = new TiM5xxSkeleton(ipAddress);
  dutInitialized = true;
  //===============================================================================
  // Step 2) Connect to the Lidar
  //===============================================================================
  if (SSBL_SUCCESS == DUT->Connect()) {
    //===============================================================================
    // Step 3) Read the output range currently employed by the device
    // Further details on PAGE 56!
    //===============================================================================
    if (SSBL_SUCCESS == DUT->ReadVariable(orVariable)) {
      printf("Angle resolution: %8.3lf degrees\n",
             (double)orVariable.Value_.aRange[0].udiAngleRes / 10000.00);

      printf("Start angle: %8.3lf degrees\n",
             (double)orVariable.Value_.aRange[0].diStartAngle / 10000.00);
      printf("Stop angle: %8.3lf degrees\n",
             (double)orVariable.Value_.aRange[0].diStopAngle / 10000.00);

      // Now let us change start and stop angle to get a scan from 0-90�.
      // Note that the Lidar specific coordinate system is used in here.
      // Note angle resolution can not be changed for TiM5xx
      orVariable.Value_.aRange[0].diStartAngle = gScanConfig.startAngle;
      orVariable.Value_.aRange[0].diStopAngle = gScanConfig.stopAngle;
      if (SSBL_SUCCESS == DUT->WriteVariable(orVariable)) {
        // Set the variable to some different value to observe that it has
        // really been written from the device
        orVariable.Value_.aRange[0].diStartAngle = 1;
        orVariable.Value_.aRange[0].diStopAngle = 1;
        if ((SSBL_SUCCESS != DUT->ReadVariable(orVariable)) ||
            (gScanConfig.startAngle !=
             orVariable.Value_.aRange[0].diStartAngle) ||
            (gScanConfig.stopAngle != orVariable.Value_.aRange[0].diStopAngle))

        {
          printf("Error reading outputRange from the device");
          goto exit;
        } else {
          // Now you could store this configuration. Note that the embedded
          // flash might have limited number of write cycles. Therefore, one
          // should only store parameters if it's required.
        }
      } else {
        if (SSBL_SUCCESS != DUT->ReadVariable(orVariable)) {
          printf("Error while setting the scan range");
          goto exit;
        }
      }
    }

    //===============================================================================
    // Step 4) Read the scan output configuration from the device
    // Further details on PAGE 55!
    //===============================================================================
    if (SSBL_SUCCESS == DUT->ReadVariable(sdcVariable)) {
      sdcVariable.Value_.DistDataConfig[0] = 1;      // Enable output channel 1
      sdcVariable.Value_.DistDataConfig[1] = 0;      // (distance values)
      sdcVariable.Value_.RemDataConfig.bEnable = 1;  // Enable Remission / RSSI
      sdcVariable.Value_.RemDataConfig.eDataType = 0;  // 8Bit Remission values
      sdcVariable.Value_.RemDataConfig.eContentType =
          0;                                         // Digits there is no other
      sdcVariable.Value_.bEnableCommentBlock = 0;    // Comments? Pff
      sdcVariable.Value_.bEnableDeviceName = 0;      // No device name
      sdcVariable.Value_.bEnablePositionBlock = 1;   // include position data
      sdcVariable.Value_.bEnableTimeBlock = 1;       // include device timestamp
      sdcVariable.Value_.EnableEncoderBlock[0] = 0;  // no Encoder for TiM5xx
      sdcVariable.Value_.EnableEncoderBlock[1] = 0;  // no Encoder for TiM5xx

      sdcVariable.Value_.uiOutputInterval =
          1;  // no downsampling we want all scans
      if (SSBL_SUCCESS != DUT->WriteVariable(sdcVariable)) {
        printf("Error while setting the scan output configuration");
        goto exit;
      }
    }

    //===============================================================================
    // Step 5.1) Put the Lidar to measurement mode
    // Further details on PAGE 32!
    //===============================================================================
    if (SSBL_SUCCESS != DUT->CallFunction(startFunction)) {
      printf("Error when trying to put the Lidar into measurement mode");
      goto exit;
    }

    //===============================================================================
    // Step 5.2) Put the Lidar to run mode
    // Further details on PAGE 53!
    //===============================================================================
    if (SSBL_SUCCESS != DUT->CallFunction(runFunction)) {
      printf("Error when trying to put the Lidar into run mode");
      goto exit;
    }

    //===============================================================================
    // Step 6) Register to scan events, OnScan is a callback function that will
    // be called as soon as the event arrives. You can add 32 bits user data
    // which shall be available when the callback is triggered. For now, we
    // simply add the scan config. Further details on PAGE 63!
    //===============================================================================
    if (SSBL_SUCCESS !=
        DUT->RegisterEvent("ScanData", OnScan,
                           reinterpret_cast<uint64_t>(&gScanConfig))) {
      printf("Error when trying to register to scan events");
      goto exit;
    }

    //===============================================================================
    // Lets loop around to observe some scan events
    //===============================================================================
    SSBL_Sleep(5000);

    if (SSBL_SUCCESS == DUT->DeregisterEvent("ScanData")) {
      // Sleep a while to see that there are no more incoming scans
      SSBL_Sleep(5000);
    }
    //===============================================================================
    // Disconnect from the Lidar
    //===============================================================================
  exit:
    if (dutInitialized) {
      DUT->Disconnect();
    }
  }

  delete DUT;

  return 0;
}

// Callback function which will be triggered when
// scan data arrives
void OnScan(uint64_t *pEventData) {
  ScanData_TiM5xxSkeleton_Var *pVar;
  SsblEventContainer *pEvent =
      reinterpret_cast<SsblEventContainer *>(pEventData);
  myScanConfig_t *pCfg =
      reinterpret_cast<myScanConfig_t *>(pEvent->CallbackParameter);

  printf("Received Event\n");
  printf("Scan start angle: %.2lf degrees\n",
         (double)pCfg->startAngle / 10000.00);
  printf("Scan stop angle: %.2lf degrees\n",
         (double)pCfg->stopAngle / 10000.00);

  pVar = dynamic_cast<ScanData_TiM5xxSkeleton_Var *>(pEvent->pComObj);

  int32_t startAngle =
      pVar->Value_.aDataChannel16[0].DataChannelHdr.diStartAngle;

  uint32_t resolution =
      pVar->Value_.aDataChannel16[0].DataChannelHdr.uiAngleRes;

  std::vector<Point> pointVec;
  int32_t pointNum = pVar->Value_.aDataChannel16[0].uiLengthaData;
  pointVec.resize(pointNum);

  /*
  memcpy(&scaleFactor, receiveBuffer + parseOff + 5, 4);
  memcpy(&scaleFactorOffset, receiveBuffer + parseOff + 9, 4);
  memcpy(&startAngleDiv10000, receiveBuffer + parseOff + 13, 4);
  memcpy(&sizeOfSingleAngularStepDiv10000, receiveBuffer + parseOff + 17, 2);
  memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);

  rangePtr[idx] = (float) data[i] *  scaleFactor_001 + scaleFactorOffset;
  */

  float scaleFactor_001 =
      0.001F * pVar->Value_.aDataChannel16[0].DataChannelHdr.dScaleFactor;
  float scaleFactorOffset_001 =
      0.001F * pVar->Value_.aDataChannel16[0]
                   .DataChannelHdr.dScaleOffset;  // Offset in [m]

  // convert to Cartesian coordinates in [m]
  for (uint16_t dIdx = 0; dIdx < pointNum; dIdx++) {
    double dist = pVar->Value_.aDataChannel16[0].aData[dIdx];

    dist = dist * scaleFactor_001 +
           scaleFactorOffset_001;  // Convert from distance units to [m]
    double angle = startAngle / 10000.0;
    angle = angle * DEG2RAD;
    angle -=
        M_PI /
        2.0;  // Working range diagram shows 90 degrees in direction of X-Axis
    pointVec[dIdx].X(dist * cos(angle));
    pointVec[dIdx].Y(dist * sin(angle));
    startAngle = startAngle + resolution;
  }

  DataExporter dataExporter;
  dataExporter.writeJpeg(
      &pointVec);  // write example jpeg file with top view of scan
  dataExporter.writeCSV(&pointVec);  // write csv file with shot information

  // please delete the data after processing
  delete pEventData;
}
#endif