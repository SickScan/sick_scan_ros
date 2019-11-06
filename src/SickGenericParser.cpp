/**
* \file
* \brief Laser Scanner Parser
* Copyright (C) 2013, Osnabrück University
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
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
*  Last modified: 05th Nov 2019
*
*      Authors:
*              Michael Lehning <michael.lehning@lehning.de>
*              Jochen Sprickerhof <jochen@sprickerhof.de>
*              Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*/

#ifdef _MSC_VER
#pragma warning(disable: 4267)
#endif

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES
#include <math.h>

#include "SickGenericParser.h"
// #include <sick_scan/sick_scan_common.h>
#include <ros/ros.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

namespace sick_scan
{
  using namespace std;

  /*!
  \brief Setting name (type) of scanner

  \param _s name of scanner
  \sa getScannerName
  */
  void ScannerBasicParam::setScannerName(std::string _s)
  {
    scannerName = _s;
  }

  /*!
  \brief Getting name (type) of scanner

  \return Name of scanner
  \sa setScannerName
  */
  std::string ScannerBasicParam::getScannerName()
  {
    return(scannerName);
  }


  /*!
  \brief Setting number of scanner layers (depending of scanner type/family)

  \param _layerNum of scanner layers (e.g. 1 for TiM5xx and 24 for MRS6124
  \sa getNumberOfLayers
  */
  void ScannerBasicParam::setNumberOfLayers(int _layerNum)
  {
    numberOfLayers = _layerNum;
  }

  /*!
  \brief Getting number of scanner layers

  \return Number of scanners layer (e.g. 1 for TiM5xx and 24 for MRS6124)
  \sa setNumberOfLayers
  */
  int ScannerBasicParam::getNumberOfLayers(void)
  {
    return(numberOfLayers);

  }

  /*!
  \brief Set number of shots per scan

  \param _shots of shots per scan (for one layer)
  \sa getNumberOfLayers
  */
  void ScannerBasicParam::setNumberOfShots(int _shots)
  {
    numberOfShots = _shots;
  }

  /*!
  \brief Get number of shots per scan

  \return Number of shots per scan (for one layer)
  \sa getNumberOfLayers
  */
  int ScannerBasicParam::getNumberOfShots(void)
  {
    return(numberOfShots);
  }

  /*!
  \brief Set number of maximum echoes for this laser scanner type

  \param _maxEchos of max echoes
  \sa getNumberOfMaximumEchos
  */
  void ScannerBasicParam::setNumberOfMaximumEchos(int _maxEchos)
  {
    this->numberOfMaximumEchos = _maxEchos;
  }


  /*!
  \brief Get number of maximum echoes for this laser scanner type

  \return Number of max echoes
  \sa setNumberOfMaximumEchos
  */
  int ScannerBasicParam::getNumberOfMaximumEchos(void)
  {
    return(numberOfMaximumEchos);
  }

  /*!
  \brief Set pointer to corresponding parameter object to the parser

  \param _ptr to parameter object
  \sa getCurrentParamPtr
  */
  void SickGenericParser::setCurrentParamPtr(ScannerBasicParam* _ptr)
  {
    currentParamSet = _ptr;
  }


  /*!
  \brief Set angular resolution in degrees
  \param _res resolution in degress (NOT rad) between each shot
  \sa getAngularDegreeResolution
  */
  void ScannerBasicParam::setAngularDegreeResolution(double _res)
  {
    angleDegressResolution = _res;
  }

  /*!
  \brief Get angular resolution in degress

  \return angle resolution in degress (NOT rad) between each shot
  */
  double ScannerBasicParam::getAngularDegreeResolution(void)
  {
    return(angleDegressResolution);
  }

  /*!
  \brief set expected scan frequency
  \param _freq scan frequency in [Hz]
  \sa getExpectedFrequency
  */
  void ScannerBasicParam::setExpectedFrequency(double _freq)
  {
    expectedFrequency = _freq;
  }

  /*!
  \brief get expected scan frequency

  \return expected scan frequency in [Hz]
  \sa setExpectedFrequency
  */
  double ScannerBasicParam::getExpectedFrequency()
  {
    return(expectedFrequency);
  }


  /*!
  \brief set angular resolution in VERTICAL direction for multilayer scanner
  \param _elevRes resolution in degree
  \sa getElevationDegreeResolution
  */
  void ScannerBasicParam::setElevationDegreeResolution(double _elevRes)
  {
    this->elevationDegreeResolution = _elevRes;
  }


  /*!
  \brief get angular resolution in VERTICAL direction for multilayer scanner
  \return elevation resolution in degree
  \sa setElevationDegreeResolution
  */
  double ScannerBasicParam::getElevationDegreeResolution()
  {
    return(this->elevationDegreeResolution);
  }

  /*!
  \brief flag to decide between usage of ASCII-sopas or BINARY-sopas
  \param _useBinary: True for binary, False for ASCII
  \sa getUseBinaryProtocol
  */
  void ScannerBasicParam::setUseBinaryProtocol(bool _useBinary)
  {
    this->useBinaryProtocol = _useBinary;
  }

  /*!
  \brief flag to mark the device as radar (instead of laser scanner)
  \param _deviceIsRadar: false for laserscanner, true for radar (like rms_3xx)
  \sa getDeviceIsRadar
  */
  void ScannerBasicParam::setDeviceIsRadar(bool _deviceIsRadar)
  {
    deviceIsRadar = _deviceIsRadar;
  }

  /*!
  \brief flag to mark the device as radar (instead of laser scanner)
  \param _deviceIsRadar: false for laserscanner, true for radar (like rms_3xx)
  \sa getDeviceIsRadar
  */
  bool ScannerBasicParam::getDeviceIsRadar(void)
  {
    return(deviceIsRadar);
  }

  /*!
  \brief flag to decide between usage of ASCII-sopas or BINARY-sopas
  \return _useBinary: True for binary, False for ASCII
  \sa getUseBinaryProtocol
  */
  bool ScannerBasicParam::getUseBinaryProtocol(void)
  {
    return(this->useBinaryProtocol);
  }
  /*!
  \brief Set the RSSI Value length
  \param _useBinary: Boolean value: True=16 Bit False=8Bit
  \sa getUseBinaryProtocol
  */
  void ScannerBasicParam::setIntensityResolutionIs16Bit(bool _IntensityResolutionIs16Bit)
  {
    this->IntensityResolutionIs16Bit = _IntensityResolutionIs16Bit;
  }

  /*!
  \brief Get the RSSI Value length
  \return Boolean value: True=16 Bit False=8Bit
  \sa setUseBinaryProtocol
  */
  bool ScannerBasicParam::getIntensityResolutionIs16Bit(void)
  {
    return(IntensityResolutionIs16Bit);
  }
  /*!
  \brief flag to mark the device uses the safety scanner password
  \param  _useSafetyPasWD: false for normal scanners true for safety scanners
  \sa setUseSafetyPasWD
  */
  void ScannerBasicParam::setUseSafetyPasWD(bool _useSafetyPasWD)
  {
    this->useSafetyPasWD = _useSafetyPasWD;
  }
  /*!
  \brief flag to mark the device uses the safety scanner password
  \reutrn Bool true for safety password false for normal password
  \sa getUseSafetyPasWD
  */
  bool ScannerBasicParam::getUseSafetyPasWD()
  {
    return(useSafetyPasWD);
  }
  /*!
  \brief Construction of parameter object

  */
  ScannerBasicParam::ScannerBasicParam()
  {
    this->elevationDegreeResolution = 0.0;
    this->setUseBinaryProtocol(false);
  }

  /*!
\brief Prama for encoder mode
\param _EncoderMode: -1 Use for Scanners WO Encoder 00 disabled 01 single increment 02 direction recognition phase 03 direction recognition level
\sa setEncoderMode
*/
  void ScannerBasicParam::setEncoderMode(int8_t _encoderMode)
  {
    this->encoderMode = _encoderMode;
  }
  /*!
\brief Getter-Method for encoder mode
\return EncoderMode:-1 Use for Scanners WO Encoder  00 disabled 01 single increment 02 direction recognition phase 03 direction recognition level
\sa setEncoderMode
  */
  int8_t ScannerBasicParam::getEncoderMode()
  {
    return(encoderMode);
  }

  /*!
  \brief Construction of parser object
   \param _scanType Type of the Laserscanner

  */
  SickGenericParser::SickGenericParser(std::string _scanType) :
      override_range_min_((float)0.05),
      override_range_max_((float)100.0),
      override_time_increment_((float)-1.0)
  {
    setScannerType(_scanType);
    allowedScannerNames.push_back(SICK_SCANNER_MRS_1XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_5XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_7XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_TIM_7XXS_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_5XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_1XX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_1XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_MRS_6XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_LMS_4XXX_NAME);
    allowedScannerNames.push_back(SICK_SCANNER_RMS_3XX_NAME); // Radar scanner
    basicParams.resize(allowedScannerNames.size()); // resize to number of supported scanner types
    for (int i = 0; i < (int)basicParams.size(); i++) // set specific parameter for each scanner type - scanner type is identified by name
    {
      basicParams[i].setDeviceIsRadar(false); // Default
      basicParams[i].setScannerName(allowedScannerNames[i]);  // set scanner type for this parameter object

      if (basicParams[i].getScannerName().compare(SICK_SCANNER_MRS_1XXX_NAME) == 0)  // MRS1000 - 4 layer, 1101 shots per scan
      {
        basicParams[i].setNumberOfMaximumEchos(3);
        basicParams[i].setNumberOfLayers(4);
        basicParams[i].setNumberOfShots(1101);
        basicParams[i].setAngularDegreeResolution(0.25);
        basicParams[i].setElevationDegreeResolution(2.5); // in [degree]
        basicParams[i].setExpectedFrequency(50.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_1XXX_NAME) == 0)  // LMS1000 - 4 layer, 1101 shots per scan
      {
        basicParams[i].setNumberOfMaximumEchos(3);
        basicParams[i].setNumberOfLayers(4);
        basicParams[i].setNumberOfShots(1101);
        basicParams[i].setAngularDegreeResolution(1.00);  // 0.25° wurde nicht unterstuetzt. (SFA 4)
        basicParams[i].setElevationDegreeResolution(0.0); // in [degree]
        basicParams[i].setExpectedFrequency(50.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_5XX_NAME) == 0) // TIM_5xx - 1 Layer, max. 811 shots per scan
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_4XXX_NAME) == 0) // LMS_4xxx - 1 Layer, 600 Hz
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(841);//
        basicParams[i].setAngularDegreeResolution(0.0833);//
        basicParams[i].setExpectedFrequency(600.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_7XX_NAME) == 0) // TIM_7xx - 1 Layer Scanner
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_TIM_7XXS_NAME) == 0) // TIM_7xxS - 1 layer Safety Scanner
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(811);
        basicParams[i].setAngularDegreeResolution(0.3333);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(true); // Safety scanner
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_5XX_NAME) == 0) // LMS_5xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(381);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(15.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_LMS_1XX_NAME) == 0) // LMS_1xx - 1 Layer
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(1);
        basicParams[i].setNumberOfShots(541);
        basicParams[i].setAngularDegreeResolution(0.5);
        basicParams[i].setExpectedFrequency(25.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }
      if (basicParams[i].getScannerName().compare(SICK_SCANNER_MRS_6XXX_NAME) == 0) //
      {
        basicParams[i].setNumberOfMaximumEchos(5);
        basicParams[i].setNumberOfLayers(24);
        basicParams[i].setNumberOfShots(925);
        basicParams[i].setAngularDegreeResolution(0.13);
        basicParams[i].setElevationDegreeResolution(1.25); // in [degree]
        basicParams[i].setExpectedFrequency(50.0);
        basicParams[i].setUseBinaryProtocol(true);
        basicParams[i].setDeviceIsRadar(false); // Default
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default
      }

      if (basicParams[i].getScannerName().compare(SICK_SCANNER_RMS_3XX_NAME) == 0) // Radar
      {
        basicParams[i].setNumberOfMaximumEchos(1);
        basicParams[i].setNumberOfLayers(0); // for radar scanner
        basicParams[i].setNumberOfShots(65);
        basicParams[i].setAngularDegreeResolution(0.00);
        basicParams[i].setElevationDegreeResolution(0.00); // in [degree]
        basicParams[i].setExpectedFrequency(0.00);
        basicParams[i].setUseBinaryProtocol(false); // use ASCII-Protocol
        basicParams[i].setDeviceIsRadar(true); // Device is a radar
        basicParams[i].setUseSafetyPasWD(false); // Default
        basicParams[i].setEncoderMode(-1); // Default

      }

    }

    int scannerIdx = lookUpForAllowedScanner(scannerType);
    if (scannerIdx == -1)  // find index of parameter set - derived from scanner type name
    {
      ROS_ERROR("Scanner not supported.\n");
      throw new std::string("scanner type " + scannerType + " not supported.");
    }
    else
    {
      currentParamSet = &(basicParams[scannerIdx]);
    }
  }

  /*!
  \brief Gets Pointer to parameter object
  \return Pointer to parameter object holding information about protocol usage and scanner type specific parameter
  */
  ScannerBasicParam *SickGenericParser::getCurrentParamPtr()
  {
    return(currentParamSet);
  }

  /*!
  \brief checks the given scannerName/scannerType of validity
  \param scannerName as string (e.g. "tim_5xx")
  \return index of found scanner. -1 corresponds to "not found"
  */
  int SickGenericParser::lookUpForAllowedScanner(std::string scannerName)
  {
    int iRet = -1;
    for (int i = 0; i < (int)allowedScannerNames.size(); i++)
    {
      if (allowedScannerNames[i].compare(scannerName) == 0)
      {
        return(i);
      }
    }

    return(iRet);
  }

  /*!
  \brief Destructor of parser
  \sa Constructor SickGenericParser
  */
  SickGenericParser::~SickGenericParser()
  {
  }

  /*!
  \brief check for DIST and RSSI-entries in the datagram. Helper routine for parser

  \param fields: String entries holding the information
  \param expected_number_of_data: Warning, if the number of found entries does not correspond to this entries
  \param distNum: Number of found DIST-entries
  \param rssiNum: Number of found RSSI-entries
  \param distVal: parsed istance values
  \param rssiVal: parsed RSSI-values
  \param distMask: Bit-Masking holds the information of found DIST-entries (e.g. DIST1 -> Bit 0, DIST2 -> BIT 1 and so on)
  \return Errorcode
  \sa parse_datagram
  */
  int SickGenericParser::checkForDistAndRSSI(std::vector<char *>& fields, int expected_number_of_data, int& distNum, int& rssiNum, std::vector<float>& distVal, std::vector<float>& rssiVal, int& distMask)
  {
    int iRet = 0;
    distNum = 0;
    rssiNum = 0;
    int baseOffset = 20;

    distMask = 0;
    // More in depth checks: check data length and RSSI availability
    // 25: Number of data (<= 10F)
    unsigned short int number_of_data = 0;
    if (strstr(fields[baseOffset], "DIST") != fields[baseOffset]) // First initial check
    {
      ROS_WARN("Field 20 of received data does not start with DIST (is: %s). Unexpected data, ignoring scan", fields[20]);
      return 0;
    }

    int offset = 20;
    do
    {
      bool distFnd = false;
      bool rssiFnd = false;
      if (strlen(fields[offset]) == 5)
      {
        if (strstr(fields[offset], "DIST") == fields[offset])
        {
          distFnd = true;
          distNum++;
          int distId = -1;
          if (1 == sscanf(fields[offset], "DIST%d", &distId))
          {
            distMask |= (1 << (distId - 1)); // set bit regarding to id
          }
        }
        if (strstr(fields[offset], "RSSI") == fields[offset])
        {
          rssiNum++;
          rssiFnd = true;
        }
      }
      if (rssiFnd || distFnd)
      {
        offset += 5;
        if (offset >= (int)fields.size())
        {
          ROS_WARN("Missing RSSI or DIST data");
          return 0;
        }
        number_of_data = 0;
        sscanf(fields[offset], "%hx", &number_of_data);
        if (number_of_data != expected_number_of_data)
        {
          ROS_WARN("number of dist or rssi values mismatching.");
          return 0;
        }
        offset++;
        // Here is the first value
        for (int i = 0; i < number_of_data; i++)
        {
          if (distFnd)
          {
            unsigned short iRange;
            float range;
            sscanf(fields[offset + i], "%hx", &iRange);
            range = iRange / 1000.0;
            distVal.push_back(range);
          }
          else
          {
            unsigned short iRSSI;
            sscanf(fields[offset + i], "%hx", &iRSSI);
            rssiVal.push_back((float)iRSSI);
          }
        }
        offset += number_of_data;
      }
      else
      {
        offset++; // necessary????
      }
    } while (offset < (int)fields.size());

    return(iRet);
  }


  void SickGenericParser::checkScanTiming(float time_increment, float scan_time, float angle_increment, float tol)
  {
    if (this->getCurrentParamPtr()->getNumberOfLayers() > 1)
    {
      return;
    }

    float expected_time_increment = this->getCurrentParamPtr()->getNumberOfLayers() * scan_time * angle_increment / (2.0 * M_PI);
    if (fabs(expected_time_increment - time_increment) > 0.00001)
    {
      ROS_WARN_THROTTLE(60, "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
                            "Expected time_increment: %.9f, reported time_increment: %.9f. "
                            "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
                        expected_time_increment, time_increment);
    }
  };




  /*!
  \brief Setting minimum range
  \param min range in [m]
  \sa set_range_max
  */
  void SickGenericParser::set_range_min(float min)
  {
    override_range_min_ = min;
  }

  /*!
  \brief Setting maximum range
  \param max range in [m]
  \sa set_range_min
  */
  void SickGenericParser::set_range_max(float max)
  {
    override_range_max_ = max;
  }


  /*!
   \brief Getting maximum range
   \return range in [m]
   \sa set_range_max
   */
  float SickGenericParser::get_range_max(void)
  {
    return(override_range_max_);
  }

  /*!
  \brief Getting minimum range
  \return range in [m]
  \sa set_range_min
  */
  float SickGenericParser::get_range_min(void)
  {
    return(override_range_min_);
  }

  /*!
\brief setting time increment between shots

\param time increment
*/
  void SickGenericParser::set_time_increment(float time)
  {
    override_time_increment_ = time;
  }

  /*!
  \brief setting scannertype

  \param _scannerType
  \sa getScannerType
  */
  void SickGenericParser::setScannerType(std::string _scannerType)
  {
    scannerType = _scannerType;
  }

  /*!
  \brief getting scannertype

  \return scannerType
  */
  std::string SickGenericParser::getScannerType(void) {
    return(scannerType);

  }

} /* namespace sick_scan */
