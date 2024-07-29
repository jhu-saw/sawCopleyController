/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

  This component provides an interface to a Copley controller, using a serial port.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsCopleyController_h
#define _mtsCopleyController_h

#include <string>
#include <vector>

#include <cisstCommon/cmnPath.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSerialPort.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmOperatingState.h>

#include <sawCopleyController/sawCopleyControllerConfig.h>

// Always include last
#include <sawCopleyController/sawCopleyControllerExport.h>

class CISST_EXPORT mtsCopleyController : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_LOD_RUN_ERROR)

 public:

    mtsCopleyController(const std::string &name);
    mtsCopleyController(const std::string &name, unsigned int sizeStateTable, bool newThread = true);
    mtsCopleyController(const mtsTaskContinuousConstructorArg &arg);

    // This constructor is used to set the serial port name and baud rate, rather than specifying them
    // in JSON configuration file.
    mtsCopleyController(const std::string &name, const std::string &port_name, unsigned long baud_rate);

    ~mtsCopleyController();

    // cisstMultiTask functions
    void Configure(const std::string &fileName) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

    // Returns first axis label
    // This is available after calling:
    //   1) The constructor that specifies the port_name and baud_rate, or
    //   2) The Configure method (with JSON file) after other constructors
    std::string GetAxisLabel(void) const { return mAxisLabel[0]; }

    bool IsConfigured(void) const { return configOK; }

protected:

#ifdef SIMULATION
    long sim24;
#endif

    // Path to configuration files
    cmnPath mConfigPath;

    sawCopleyControllerConfig::controller m_config;
    bool configOK;                          // Whether Configure successful
    unsigned int mNumAxes;                  // Number of axes

    osaSerialPort mSerialPort;
    mtsInterfaceProvided *mInterface;       // Provided interface

    char cmdBuf[64];   // Buffer for sending commands
    char msgBuf[128];  // Buffer for sending messages
    char axisStr[32];  // Either "%s" or "%s on axis %d"

    unsigned int mTicks;                    // Counts number of loops

    vctLongVec mPosRaw;
    vctDoubleVec mPos;
    vctLongVec mStatus;                     // Drive status
    vctLongVec mFault;                      // Fault status
    prmConfigurationJoint m_config_j;       // Joint configuration
    prmStateJoint m_measured_js;            // Measured joint state (CRTK)
    prmStateJoint m_setpoint_js;            // Setpoint joint state (CRTK)
    prmOperatingState m_op_state;           // Operating state (CRTK)
    vctDoubleVec mDispScale;                // Display scale
    std::vector<std::string> mDispUnits;    // Display units
    std::vector<std::string> mAxisLabel;    // Axis label on drive (parameter 0x92)

    vctDoubleVec mSpeed;                    // Max speed for position move
    vctDoubleVec mAccel;                    // Max accel for position move
    vctDoubleVec mDecel;                    // Max decel for position move

    vctUIntVec mState;                      // Internal state machine
    vctBoolVec mIsHomed;                    // true if axis homed

    mtsFunctionWrite operating_state;       // Event generator

    void Init();
    bool InitSerialPort(const std::string &port_name, unsigned long baud_rate);
    void Close();

    void SetupInterfaces();
    bool LoadCCX(const std::string &fileName);

    // Read until CR ('\r') or timeout
    int ReadUntilCR(char *respBuf, size_t respSize, double timeout_s = 0.1);

    // Send the command to the drive; returns 0 on success
    // For a read command, result returned in value
    int SendCommand(const char *cmd, int len, long *value = 0, unsigned int num = 1);

    int ParameterSet(unsigned int addr, long value, unsigned int axis = 0, bool inRAM = true);
    int ParameterGet(unsigned int addr, long &value, unsigned int axis = 0, bool inRAM = true);

    int ParameterSetArray(unsigned int addr, long *value, unsigned int num, unsigned int axis = 0, bool inRAM = true);
    int ParameterGetArray(unsigned int addr, long *value, unsigned int num, unsigned int axis = 0, bool inRAM = true);

    // Note that default is to read from flash (not RAM)
    int ParameterGetString(unsigned int addr, std::string &value, unsigned int axis = 0, bool inRAM = false);

    // Checks whether axis label from drive (parameter 0x92) matches JSON file
    bool CheckAxisLabel(unsigned int axis) const;

    // Performs some common checks, such as whether vector size matches mNumAxes (if vsize != 0)
    // and whether configOK and copleyOK are true.
    bool CheckCommand(const std::string &cmdName, size_t vsize = 0) const;

    // Methods for provided interface
    void GetConfigured(bool &val) const
    { val = configOK; }
    void GetConnected(bool &val) const;
    void SendCommandRet(const std::string& cmdString, std::string &retString);

    // Get joint configuration
    void GetConfig_js(prmConfigurationJoint &cfg_j) const
    { cfg_j = m_config_j; }

    void move_jp(const prmPositionJointSet &goal);
    void move_jr(const prmPositionJointSet &goal);

    void move_common(const char *cmdName, const vctDoubleVec &goal, unsigned int profile_type);

    // Set speed, acceleration and deceleration
    void SetSpeed(const vctDoubleVec &spd);
    void SetAccel(const vctDoubleVec &accel);
    void SetDecel(const vctDoubleVec &decel);

    // Enable motor power
    void EnableMotorPower(void);
    // Disable motor power
    void DisableMotorPower(void);

    // Home: all axes
    void HomeAll();
    // Home: mask indicates which axes to home
    void Home(const vctBoolVec &mask);

    // Clear fault
    void ClearFault();

    // Get axis label
    void GetAxisLabel(std::vector<std::string> &label) const
    { label = mAxisLabel; }

    // Check all axis labels
    void CheckAxisLabels(void);

    // Load ccx file (that was specified in JSON file)
    void CommandLoadCCX(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsCopleyController)

#endif
