/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <fstream>

#include <cisstCommon/cmnPortability.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <sawCopleyController/mtsCopleyController.h>

// Scale factors
const double CurrentBitsToAmps = 0.01;
const double VelocityBitsToCps = 0.1;        // counts/second
const double AccelBitsToCps2 = 10.0;         // counts/s^2
const double AccelLimitBitsToCps2 = 1000.0;  // counts/s^2

enum COPLEY_STATES { ST_IDLE, ST_HOMING, ST_MOVING };

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsCopleyController, mtsTaskContinuous, mtsStdString)

mtsCopleyController::mtsCopleyController(const std::string &name) : mtsTaskContinuous(name, 1024, true)
{
    Init();
}

mtsCopleyController::mtsCopleyController(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread)
{
    Init();
}

mtsCopleyController::mtsCopleyController(const mtsTaskContinuousConstructorArg & arg) : mtsTaskContinuous(arg)
{
    Init();
}

mtsCopleyController::mtsCopleyController(const std::string &name, const std::string &port_name, unsigned long baud_rate) :
    mtsTaskContinuous(name, 1024, true)
{
    Init();
#ifndef SIMULATION
    if (InitSerialPort(port_name, baud_rate)) {
        std::string label;
        if (ParameterGetString(0x92, label, 0) == 0) {
            mAxisLabel[0] = label;
        }
    }
#else
    if (port_name == "COM4")
        mAxisLabel[0] = "Lower Rotate";
    else if (port_name == "COM5")
        mAxisLabel[0] = "Upper Slide";
    else if (port_name == "COM6")
        mAxisLabel[0] = "Upper Rotate";
    else if (port_name == "COM7")
        mAxisLabel[0] = "Lower Slide";
#endif
}

mtsCopleyController::~mtsCopleyController()
{
    Close();
}

void mtsCopleyController::Init(void)
{
    configOK = false;
    mTicks = 0;
    mNumAxes = 1;
    mAxisLabel.resize(mNumAxes);
    mAxisLabel[0].clear();
}

bool mtsCopleyController::InitSerialPort(const std::string &port_name, unsigned long baud_rate)
{
    mSerialPort.SetPortName(port_name);
    // Default constructor sets port to 9600 baud, N,8,1
    if (!mSerialPort.Open()) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": failed to open serial port: "
                                 << mSerialPort.GetPortName() << std::endl;
        return false;
    }
    mSerialPort.Configure();

    // Write a break, to make sure serial port is at 9600 baud
    const double breakTime = 0.5 * cmn_s;
    mSerialPort.WriteBreak(breakTime);
    // Wait for length of break and a bit more
    osaSleep(breakTime + 0.5 * cmn_s);

    if (baud_rate != 9600) {
        // If desired baud rate is not 9600
        osaSerialPort::BaudRateType osaBaudRate;
        switch (baud_rate) {
        case 19200:  osaBaudRate = osaSerialPort::BaudRate19200;
                     break;
        case 38400:  osaBaudRate = osaSerialPort::BaudRate38400;
                     break;
        case 57600:  osaBaudRate = osaSerialPort::BaudRate57600;
                     break;
        case 115200: osaBaudRate = osaSerialPort::BaudRate115200;
                     break;
        default:     CMN_LOG_CLASS_INIT_ERROR << "Unsupported baud rate " << baud_rate << std::endl;
                     return false;
        }
        CMN_LOG_CLASS_INIT_VERBOSE << GetName() <<  ": setting baud rate to " << baud_rate << std::endl;

        sprintf(cmdBuf, "s r0x90 %ld\r", baud_rate);
        int nBytes = static_cast<int>(strlen(cmdBuf));
        int nSent = mSerialPort.Write(cmdBuf, nBytes);
        if (nSent != nBytes) {
            CMN_LOG_CLASS_INIT_ERROR << "Failed to set baud rate to " << baud_rate << std::endl;
            return false;
        }
        // Wait at least 100 msec after setting baud rate
        osaSleep(0.1);
        mSerialPort.SetBaudRate(osaBaudRate);
        if (!mSerialPort.Configure()) {
            CMN_LOG_CLASS_INIT_ERROR << "Failed to configure serial port" << std::endl;
            return false;
        }
    }
    return true;
}

void mtsCopleyController::SetupInterfaces(void)
{
    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Stats
        mInterface->AddCommandReadState(StateTable, mTicks, "GetTicks");
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");

        mInterface->AddCommandReadState(this->StateTable, mPosRaw, "GetPositionRaw");

        // Standard CRTK interfaces
        mInterface->AddCommandReadState(this->StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandReadState(this->StateTable, m_op_state, "operating_state");
        mInterface->AddCommandWrite(&mtsCopleyController::move_jp, this, "move_jp");
        mInterface->AddCommandWrite(&mtsCopleyController::move_jr, this, "move_jr");
        mInterface->AddCommandRead(&mtsCopleyController::GetConfig_js, this, "configuration_js");
        mInterface->AddEventWrite(operating_state, "operating_state", prmOperatingState());

        mInterface->AddCommandVoid(&mtsCopleyController::EnableMotorPower, this, "EnableMotorPower");
        mInterface->AddCommandVoid(&mtsCopleyController::DisableMotorPower, this, "DisableMotorPower");

        mInterface->AddCommandRead(&mtsCopleyController::GetConfigured, this, "GetConfigured");
        mInterface->AddCommandRead(&mtsCopleyController::GetConnected, this, "GetConnected");
        mInterface->AddCommandRead(&mtsCopleyController::GetVersion, this, "GetVersion");
        mInterface->AddCommandWriteReturn(&mtsCopleyController::SendCommandRet, this, "SendCommandRet");
        mInterface->AddCommandReadState(this->StateTable, mStatus, "GetStatus");
        mInterface->AddCommandReadState(this->StateTable, mFault,  "GetFault");
        mInterface->AddCommandReadState(this->StateTable, mSpeed, "GetSpeed");
        mInterface->AddCommandReadState(this->StateTable, mAccel, "GetAccel");
        mInterface->AddCommandReadState(this->StateTable, mDecel, "GetDecel");
        mInterface->AddCommandWrite(&mtsCopleyController::SetSpeed, this, "SetSpeed");
        mInterface->AddCommandWrite(&mtsCopleyController::SetAccel, this, "SetAccel");
        mInterface->AddCommandWrite(&mtsCopleyController::SetDecel, this, "SetDecel");
        mInterface->AddCommandVoid(&mtsCopleyController::HomeAll, this, "Home");
        mInterface->AddCommandWrite(&mtsCopleyController::Home, this, "Home");
        mInterface->AddCommandVoid(&mtsCopleyController::ClearFault, this, "ClearFault");
        mInterface->AddCommandRead(&mtsCopleyController::GetAxisLabel, this, "GetAxisLabel");
        mInterface->AddCommandVoid(&mtsCopleyController::CommandLoadCCX, this, "LoadCCX");
        mInterface->AddCommandVoid(&mtsCopleyController::CheckAxisLabels, this, "CheckAxisLabels");
    }
}

void mtsCopleyController::Close()
{
#ifndef SIMULATION
    if (mSerialPort.IsOpened())
        mSerialPort.Close();
#endif
}

void mtsCopleyController::Configure(const std::string& fileName)
{
    unsigned int axis;

    mConfigPath.Set(cmnPath::GetWorkingDirectory());
    std::string fullname = mConfigPath.Find(fileName);
    std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting mConfigPath to " << configDir
                               << " for file " << fileName << std::endl;
    mConfigPath.Add(configDir, cmnPath::HEAD);

    configOK = false;
    std::ifstream jsonStream;
    jsonStream.open(fileName.c_str());
    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse " << fileName << " for Copley config" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        // Leave configOK false
        return;
    }
    try {
        m_config.DeSerializeTextJSON(jsonConfig);
    } catch (std::exception & std_exception) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: " << fileName << ": " << std_exception.what() << std::endl;
        // Leave configOK false
        return;
    }   

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed file " << fileName << std::endl
                               << "Loaded configuration:" << std::endl
                               << m_config << std::endl;
    
    // Size of array determines number of axes
    mNumAxes = static_cast<unsigned int>(m_config.axes.size());
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found " << mNumAxes << " axes" << std::endl;

    // Following is used as a control string for sprintf
    if (mNumAxes == 1)
        strcpy(axisStr, "%s");
    else
        strcpy(axisStr, "%s on axis %d");

    // Now, set the data sizes
    m_config_j.Name().SetSize(mNumAxes);
    m_config_j.Type().SetSize(mNumAxes);
    m_config_j.PositionMin().SetSize(mNumAxes);
    m_config_j.PositionMax().SetSize(mNumAxes);
    // Raw encoder position
    mPosRaw.SetSize(mNumAxes);
    mPosRaw.SetAll(0);
    mPosOffset.SetSize(mNumAxes);
    mPosOffset.SetAll(0.0);
    mHomeOffsetRaw.SetSize(mNumAxes);
    mHomeOffsetRaw.SetAll(0);
    // We have position for measured_js and setpoint_js
    m_measured_js.Name().SetSize(mNumAxes);
    m_measured_js.Position().SetSize(mNumAxes);
    m_measured_js.Position().SetAll(0.0);
    m_measured_js.Effort().SetSize(mNumAxes);
    m_measured_js.Effort().SetAll(0.0);
    m_setpoint_js.Name().SetSize(mNumAxes);
    m_setpoint_js.Position().SetSize(mNumAxes);
    m_setpoint_js.Position().SetAll(0.0);
    // Status
    mStatus.SetSize(mNumAxes);
    mStatus.SetAll(0);
    mFault.SetSize(mNumAxes);
    mFault.SetAll(0);
    // Display scale and units
    mDispScale.SetSize(mNumAxes);
    mDispScale.SetAll(1.0);
    mDispUnits.resize(mNumAxes);
    mAxisLabel.resize(mNumAxes);
    // Max speed, accel, decel for position move (will be queried after loading ccx file)
    mSpeed.SetSize(mNumAxes);
    mAccel.SetSize(mNumAxes);
    mDecel.SetSize(mNumAxes);
    // Internal state
    mState.SetSize(mNumAxes);
    mState.SetAll(ST_IDLE);
    mIsHomed.SetSize(mNumAxes);
    mIsHomed.SetAll(false);

    for (axis = 0; axis < mNumAxes; axis++) {
        sawCopleyControllerConfig::axis &axisData = m_config.axes[axis];
        m_measured_js.Name()[axis].assign(1, 'A'+axis);
        m_setpoint_js.Name()[axis].assign(1, 'A'+axis);
        m_config_j.Name()[axis].assign(1, 'A'+axis);
        m_config_j.Type()[axis] = axisData.type;
        m_config_j.PositionMin()[axis] = axisData.position_limits.lower;
        m_config_j.PositionMax()[axis] = axisData.position_limits.upper;
        if (axisData.type == CMN_JOINT_PRISMATIC) {
            mDispScale[axis] = 1000.0;     // meters --> millimeters
            mDispUnits[axis].assign("mm");
        }
        else if (axisData.type == CMN_JOINT_REVOLUTE) {
            mDispScale[axis] = cmn180_PI;  // radians --> degrees
            mDispUnits[axis].assign("deg");
        }
        else {
            CMN_LOG_CLASS_INIT_WARNING << "mtsCopleyClient: joint " << axis << " is unknown type ("
                                       << axisData.type << ")" << std::endl;
            mDispScale[axis] = 1.0;
            mDispUnits[axis].assign("cnts");
        }
        // mPosOffset is the position offset, in SI units
        mPosOffset[axis] = m_config.axes[axis].position_bits_to_SI.offset/mDispScale[axis];
        // Compute home position in bits. Note that we have to subtract the offset, then convert from
        // Display Units (mm or deg) to SI units (m or rad), from SI units to bits, and then negate.
        mHomeOffsetRaw[axis] = -static_cast<long>((m_config.axes[axis].home_pos - m_config.axes[axis].position_bits_to_SI.offset)
                                                  * m_config.axes[axis].position_bits_to_SI.scale/mDispScale[axis]);
    }

    StateTable.AddData(mTicks, "ticks");
    StateTable.AddData(mPosRaw, "position_raw");
    StateTable.AddData(m_measured_js, "measured_js");
    m_op_state.SetValid(true);
    StateTable.AddData(m_op_state, "op_state");
    StateTable.AddData(mStatus, "status");
    StateTable.AddData(mFault, "fault");
    StateTable.AddData(mSpeed, "speed");
    StateTable.AddData(mAccel, "accel");
    StateTable.AddData(mDecel, "decel");

    // Call SetupInterfaces after Configure because we need to know the correct sizes of
    // the dynamic vectors, which are based on the number of configured axes.
    // These sizes should be set before calling StateTable.AddData and AddCommandReadState;
    // in the latter case, this ensures that the argument prototype has the correct size.
    SetupInterfaces();

#ifndef SIMULATION
    if (!mSerialPort.IsOpened()) {
        if (!m_config.port_name.empty()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Initializing serial port from " << fileName << std::endl;
            if (!InitSerialPort(m_config.port_name, m_config.baud_rate))
                return;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Serial port must be specified in constructor or in " << fileName << std::endl;
            return;
        }
    }
    else {
        if (!m_config.port_name.empty()) {
            CMN_LOG_CLASS_INIT_WARNING << "Serial port already initialized; ignoring port_name in " << fileName << std::endl;
        }
    }
#else
    sim24 = 21;
#endif
    configOK = true;
}

// Query parameters from Copley controller. This is a separate function so that it can
// be called from Startup and from CommandLoadCCX.
void mtsCopleyController::QueryDrive()
{
    bool isAllHomed = true;
    unsigned int axis;
    for (axis = 0; axis < mNumAxes; axis++) {
#ifndef SIMULATION
        // Query the default speed, accel and decel.
        // Could also add the jerk (0xce).
        long speedRaw;
        if (ParameterGet(0xcb, speedRaw, axis) == 0) {
            mSpeed[axis] = (speedRaw*VelocityBitsToCps)/m_config.axes[axis].position_bits_to_SI.scale;
            CMN_LOG_CLASS_INIT_VERBOSE << "Default speed[" << axis << "]: " << (mSpeed[axis]*mDispScale[axis])
                                       << " " << mDispUnits[axis] << "/s (" << speedRaw << ")" << std::endl;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get speed" << std::endl;
        }
        long accelRaw;
        if (ParameterGet(0xcc, accelRaw, axis) == 0) {
            mAccel[axis] = (accelRaw*AccelBitsToCps2)/m_config.axes[axis].position_bits_to_SI.scale;
            CMN_LOG_CLASS_INIT_VERBOSE << "Default accel[" << axis << "]: " << (mAccel[axis]*mDispScale[axis])
                                       << " " << mDispUnits[axis] << "/s^2 (" << accelRaw << ")" << std::endl;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get acceleration" << std::endl;
        }
        long decelRaw;
        if (ParameterGet(0xcd, decelRaw, axis) == 0) {
            mDecel[axis] = (decelRaw*AccelBitsToCps2)/m_config.axes[axis].position_bits_to_SI.scale;
            CMN_LOG_CLASS_INIT_VERBOSE << "Default decel[" << axis << "]: " << (mDecel[axis]*mDispScale[axis])
                                       << " " << mDispUnits[axis] << "/s^2 (" << decelRaw << ")" << std::endl;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get deceleration" << std::endl;
        }
#endif
        // Check whether axis is homed
        long trajStatus;
        if (ParameterGet(0xc9, trajStatus, axis) == 0) {
            mIsHomed[axis] = trajStatus & (1<<12);
        }
        if (!mIsHomed[axis])
            isAllHomed = false;
    }
    m_op_state.SetIsHomed(isAllHomed);
}

void mtsCopleyController::Startup()
{
    if (!configOK) {
        CMN_LOG_CLASS_INIT_WARNING << GetName() << ": Startup called for invalid configuration" << std::endl;
    }
    if (m_config.load_ccx && !m_config.ccx_file.empty()) {
        std::string fullPath = mConfigPath.Find(m_config.ccx_file);
        if (!fullPath.empty()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Loading drive data from " << m_config.ccx_file << std::endl;
            LoadCCX(fullPath);
        }
    }
#ifndef SIMULATION
    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        // Query the axis label (string).
        std::string label;
        if (ParameterGetString(0x92, label, axis) == 0) {
            mAxisLabel[axis].assign(label);
            if (!CheckAxisLabel(axis)) {
                configOK = false;
            }
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get axis label" << std::endl;
        }
    }
#endif
    // Read parameters (speed, accel, decel, home) from drive
    QueryDrive();
}

void mtsCopleyController::Run()
{
    unsigned int axis;
    bool copleyOK;
    mTicks++;
    GetConnected(copleyOK);
    if (configOK && copleyOK) {
        long value;
        bool isAnyDisabled = false;
        bool isAnyFault = false;
        bool isAnyMoving = false;
        bool isAllHomed = true;
        for (axis = 0; axis < mNumAxes; axis++) {
            if (ParameterGet(0x32, value, axis) == 0) {   // measured position
                mPosRaw[axis] = value;
                m_measured_js.Position()[axis] = (mPosRaw[axis]/m_config.axes[axis].position_bits_to_SI.scale)
                                                 + mPosOffset[axis];
            }
            if (ParameterGet(0x0c, value, axis) == 0) {  // measured current
                // Units of 0.01 Amps, so divide by 100 to get Amps
                m_measured_js.Effort()[axis] = static_cast<double>(value)*CurrentBitsToAmps;
            }
            mFault[axis] = 0;    // Updated later if any fault detected
            if (ParameterGet(0xa0, value, axis) == 0) {   // drive status
                mStatus[axis] = value;
                if (value & (1<<12))
                    isAnyDisabled = true;
                if (value & (1<<22))
                    isAnyFault = true;
                if (value & (1<<27))
                    isAnyMoving = true;
            }
            if (!mIsHomed[axis])
                isAllHomed = false;
        }
        prmOperatingState::StateType newState = prmOperatingState::ENABLED;
        if (isAnyFault) {
            newState = prmOperatingState::FAULT;
            for (axis = 0; axis < mNumAxes; axis++) {
                if (ParameterGet(0xa4, value, axis) == 0)   // fault status
                    mFault[axis] = value;
            }
        }
        else if (isAnyDisabled) {
            newState = prmOperatingState::DISABLED;
        }
        if ((newState != m_op_state.State()) ||
            (isAnyMoving != m_op_state.IsBusy()) ||
            (isAllHomed != m_op_state.IsHomed())) {
            m_op_state.SetState(newState);
            m_op_state.SetIsBusy(isAnyMoving);
            m_op_state.SetIsHomed(isAllHomed);
            // Trigger event
            operating_state(m_op_state);
        }
    }
    else {
        // If not configured or connected, sleep for 0.1 seconds to limit CPU time
        osaSleep(0.1);
    }

    // Advance the state table now, so that any connected components can get
    // the latest data.
    StateTable.Advance();

    // Call any connected components
    RunEvent();

    ProcessQueuedCommands();

    for (axis = 0; axis < mNumAxes; axis++) {

        if (mState[axis] != ST_IDLE) {

            long trajStatus;
            if (ParameterGet(0xc9, trajStatus, axis) == 0) {
                if (mState[axis] == ST_HOMING) {
                    if (trajStatus & (1<<11)) {
                        sprintf(msgBuf, axisStr, ": Home error", axis);
                        mInterface->SendError(GetName()+msgBuf);
                        mState[axis] = ST_IDLE;
                    }
                    else if (trajStatus & (1<<12)) {
                        mIsHomed[axis] = true;
                        sprintf(msgBuf, axisStr, ": Home finished", axis);
                        mInterface->SendStatus(GetName()+msgBuf);
                        mState[axis] = ST_IDLE;
                    }
                    else if (!(trajStatus & (1<<13))) {
                        sprintf(msgBuf, axisStr, ": Home not active", axis);
                        mInterface->SendWarning(GetName()+msgBuf);
                    }
                }
                else if (mState[axis] == ST_MOVING) {
                    if (trajStatus & (1<<14)) {
                        sprintf(msgBuf, axisStr, ": Motion aborted", axis);
                        mInterface->SendWarning(GetName()+msgBuf);
                        mState[axis] = ST_IDLE;
                    }
                    else if (!(trajStatus & (1<<15))) {
                        sprintf(msgBuf, axisStr, ": Motion completed", axis);
                        mInterface->SendStatus(GetName()+msgBuf);
                        mState[axis] = ST_IDLE;
                    }
                }
            }
        }
    }

}

void mtsCopleyController::Cleanup(){
    Close();
}

int mtsCopleyController::ReadUntilCR(char *respBuf, size_t respSize, double timeout_s)
{
    bool done = false;
    int nRecv = 0;
    double start_time = osaGetTime();
    int nIter;
    for (nIter = 0; !done; nIter++) {
        int n = mSerialPort.Read(respBuf+nRecv, static_cast<int>(respSize)-nRecv);
        if (n > 0) {
            nRecv += n;
            if (respBuf[nRecv-1] == '\r') {
                respBuf[nRecv-1] = 0;
                nRecv--;
                done = true;
            }
        }
        else if (n < 0) {
            nRecv = n;
            done = true;
        }
        if (!done) {
            if ((osaGetTime()-start_time) > timeout_s) {
                mInterface->SendWarning(GetName()+": timeout reading from serial port");
                nRecv = -1;
                done = true;
            }
            else {
                // Wait 0.005 seconds
                osaSleep(0.005);
            }
        }
    }
#if 0
    // Following useful for timing measurements
    if (nIter > 1) {
        double dt = osaGetTime()-start_time;
        CMN_LOG_RUN_VERBOSE << "Number of iterations: " << nIter << ", DT = " << dt
                            << ", nRecv = " << nRecv << std::endl;
    }
#endif
    return nRecv;
}

int mtsCopleyController::SendCommand(const char *cmd, int len, long *value, unsigned int num)
{
    int rc = -1;
    msgBuf[0] = 0;
#ifndef SIMULATION
    if (mSerialPort.IsOpened()) {
        int nSent = mSerialPort.Write(cmd, len);
        // Add CR ('\r') if not already in cmd
        if (cmd[len-1] != '\r') {
            char cr = '\r';
            len++;
            nSent += mSerialPort.Write(&cr, 1);
        }
        if (nSent == len) {
            if (cmd[0] == 'r') {
                // No response to reset command
                mInterface->SendStatus(GetName()+": SendCommand reset");
                return 0;
            }
            // Wait a little for the response (typically takes 0.01-0.02 seconds)
            osaSleep(0.01);
            char respBuf[256];
            int nRecv = ReadUntilCR(respBuf, sizeof(respBuf));
            if (nRecv > 0) {
                respBuf[nRecv] = 0;  // May not be needed
                if (strncmp(respBuf, "ok", 2) == 0) {
                    rc = 0;
                }
                else if (strncmp(respBuf, "e ", 2) == 0) {
                    sscanf(respBuf+2, "%d", &rc);
                    sprintf(msgBuf, "SendCommand %s: error %d", cmd, rc);
                }
                else if ((strncmp(respBuf, "v ", 2) == 0) ||
                         (strncmp(respBuf, "r ", 2) == 0)) {
                    rc = 0;
                    if (value) {
                        unsigned int nchars;
                        char *p = respBuf+2;
                        for (unsigned int i = 0; i < num; i++) {
                            if (sscanf(p, "%ld%n", value+i, &nchars) != 1) {
                                sprintf(msgBuf, "SendCommand %s: failed to parse response %s", cmd, respBuf);
                                rc = -1;
                                break;
                            }
                            p += nchars;
                        }
                    }
                    else {
                        sprintf(msgBuf, "SendCommand %s: ignoring response %s", cmd, respBuf);
                    }
                }
                else {
                    sprintf(msgBuf, "SendCommand %s: unexpected response %s", cmd, respBuf);
                }
            }
        }
        else {
            sprintf(msgBuf, "SendCommand %s: failed to write command (nSent = %d, len = %d)",
                    cmd, nSent, len);
        }
    }
    else {
        sprintf(msgBuf, "SendCommand: serial port not open");
    }
#else
    // sprintf(msgBuf, "SendCommand: no serial port in SIMULATION");
    osaSleep(0.01);
    rc = 0;
#endif

    if (msgBuf[0])
        mInterface->SendError(GetName()+": "+msgBuf);

    return rc;
}

void mtsCopleyController::GetConnected(bool &val) const
{
#ifndef SIMULATION
    val = mSerialPort.IsOpened();
#else
    val = true;
#endif
}

int mtsCopleyController::ParameterSet(unsigned int addr, long value, unsigned int axis, bool inRAM)
{
#ifndef SIMULATION
    char *p = cmdBuf;
    if (mNumAxes != 1) {
        sprintf(p, ".%c ", 'A'+axis);
        p += 3;
    }
    char bank = inRAM ? 'r' : 'f';
    sprintf(p, "s %c0x%x %ld\r", bank, addr, value);
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)));
#else
    if (addr == 0x24)
        sim24 = value;
    osaSleep(0.01);
    return 0;
#endif
}

int mtsCopleyController::ParameterGet(unsigned int addr, long &value, unsigned int axis, bool inRAM)
{
#ifndef SIMULATION
    char *p = cmdBuf;
    if (mNumAxes != 1) {
        sprintf(p, ".%c ", 'A'+axis);
        p += 3;
    }
    char bank = inRAM ? 'r' : 'f';
    sprintf(p, "g %c0x%x\r", bank, addr);
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)), &value);
#else
    switch (addr) {
    case 0x24:    // desired state
        value = sim24;
        break;
    case 0x32:    // position
        value = mPosRaw[axis];
        break;
    case 0xa0:
        value = (sim24 == 0) ? (1<<12) : 0;
        break;
    default:
        value = 0;
    }
    osaSleep(0.01);
    return 0;
#endif
}

int mtsCopleyController::ParameterSetArray(unsigned int addr, long *value, unsigned int num,
                                           unsigned int axis, bool inRAM)
{
    char *p = cmdBuf;
    if (mNumAxes != 1) {
        sprintf(p, ".%c ", 'A'+axis);
        p += 3;
    }
    char bank = inRAM ? 'r' : 'f';
    sprintf(p, "s %c0x%x", bank, addr);
    p += strlen(p);
    for (unsigned int i = 0; i < num; i++) {
        sprintf(p, " %ld", value[i]);
        p += strlen(p);
    }
    sprintf(p, "\r");
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)));
}

int mtsCopleyController::ParameterGetArray(unsigned int addr, long *value, unsigned int num,
                                           unsigned int axis, bool inRAM)
{
    char *p = cmdBuf;
    if (mNumAxes != 1) {
        sprintf(p, ".%c ", 'A'+axis);
        p += 3;
    }
    char bank = inRAM ? 'r' : 'f';
    sprintf(p, "g %c0x%x\r", bank, addr);
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)), value, num);
}

int mtsCopleyController::ParameterGetString(unsigned int addr, std::string &value, unsigned int axis, bool inRAM)
{
    char *p = cmdBuf;
    if (mNumAxes != 1) {
        sprintf(p, ".%c ", 'A'+axis);
        p += 3;
    }
    char bank = inRAM ? 'r' : 'f';
    sprintf(p, "g %c0x%x\r", bank, addr);
    std::string resp;
    SendCommandRet(std::string(cmdBuf), resp);
    int rc = -1;
    if ((!resp.empty()) && (resp[0] == 'v')) {
        value = resp.substr(2);
        rc = 0;
    }
    return rc;
}

void mtsCopleyController::SendCommandRet(const std::string &cmdString, std::string &retString)
{
#ifndef SIMULATION
    if (mSerialPort.IsOpened()) {
        int len = static_cast<int>(cmdString.size());
        int nSent = mSerialPort.Write(cmdString);
        // Add CR ('\r') if not already in cmdString
        if (cmdString[len-1] != '\r') {
            char cr = '\r';
            len++;
            nSent += mSerialPort.Write(&cr, 1);
        }
        if (nSent != len) {
            CMN_LOG_CLASS_RUN_ERROR << "Failed to write " << cmdString << std::endl;
            retString.assign("Failed to write command");
            return;
        }
        // Wait a little for the response (typically takes 0.01-0.02 seconds)
        osaSleep(0.01);
        char respBuf[256];
        int nRecv = ReadUntilCR(respBuf, sizeof(respBuf));
        if (nRecv > 0) {
            respBuf[nRecv] = 0;   // May not be needed
            retString.assign(respBuf);
        }
        else {
            retString.assign("");
        }
    }
#else
    retString.assign("v SIMULATION");
    osaSleep(0.01);
#endif
}

bool mtsCopleyController::CheckAxisLabel(unsigned int axis) const
{
    bool ret = true;
    if (!m_config.axes[axis].axis_label.empty()) {
        // If axis_label specified in JSON file, look for a match (case-insensitive)
#if (CISST_OS == CISST_WINDOWS)
        if (_stricmp(m_config.axes[axis].axis_label.c_str(), mAxisLabel[axis].c_str()) == 0)
#else
        if (strcasecmp(m_config.axes[axis].axis_label.c_str(), mAxisLabel[axis].c_str()) == 0)
#endif
        {
            mInterface->SendStatus(GetName() + ", " + mSerialPort.GetPortName() +
                                   ": axis label " + mAxisLabel[axis]);
        }
        else {
            ret = false;
            mInterface->SendError(GetName() + ", " + mSerialPort.GetPortName() +
                                  ": inconsistent axis label, json=["
                                  + m_config.axes[axis].axis_label + "], drive=["
                                  + mAxisLabel[axis] + "]");
        }
    }
    else {
        mInterface->SendStatus(GetName() + ", " + mSerialPort.GetPortName() +
                               ": axis label not specified");
    }
    return ret;
}

bool mtsCopleyController::CheckCommand(const std::string &cmdName, size_t vsize) const
{
    if ((vsize != 0) && (vsize != mNumAxes)) {
        mInterface->SendError(GetName() + ": size mismatch in " + std::string(cmdName));
        return false;
    }
    if (!configOK) {
        mInterface->SendError(GetName() + ": configuration invalid");
        return false;
    }
    bool copleyOK;
    GetConnected(copleyOK);
    if (!copleyOK) {
        mInterface->SendError(GetName() + ": not connected to drive");
        return false;
    }
    return true;
}

void mtsCopleyController::move_jp(const prmPositionJointSet &goal)
{
    // profile_type 0 means absolute move, trapezoidal profile
    move_common("move_jp", goal.Goal(), 0);
}

void mtsCopleyController::move_jr(const prmPositionJointSet &goal)
{
    // profile_type 256 means relative move, trapezoidal profile
    move_common("move_jr", goal.Goal(), 256);
}

void mtsCopleyController::move_common(const char *cmdName, const vctDoubleVec &goal,
                                      unsigned int profile_type)
{
    if (!CheckCommand(cmdName, goal.size()))
        return;

    unsigned int axis;
    for (axis = 0; axis < mNumAxes; axis++) {
        if (ParameterSet(0xc8, profile_type, axis) != 0) {
            sprintf(msgBuf, ": %s: failed to set profile_type to %d for axis %d", cmdName, profile_type, axis);
            mInterface->SendError(GetName()+msgBuf);
            return;
        }
        long goalCnts = static_cast<long>((goal[axis]-mPosOffset[axis])*m_config.axes[axis].position_bits_to_SI.scale);
#ifndef SIMULATION
        if (ParameterSet(0xca, goalCnts, axis) != 0) {
            sprintf(msgBuf, ": %s: failed to set position goal to %ld for axis %d", cmdName, goalCnts, axis);
            mInterface->SendError(GetName()+msgBuf);
            return;
        }
        long desiredState = -1;
        ParameterGet(0x24, desiredState, axis);
        if (desiredState != 21) {
            sprintf(msgBuf, ": %s: changing state for axis %d from %ld to 21", cmdName, axis, desiredState);
            mInterface->SendStatus(GetName()+msgBuf);
            ParameterSet(0x24, 21, axis);
        }
#else
        if (profile_type == 0)
            mPosRaw[axis] = goalCnts;
        else if (profile_type == 256)
            mPosRaw[axis] += goalCnts;
        osaSleep(0.02);
#endif
    }
    // Note that newer multi-axis drives allow the axes to be specified in the command code;
    // the following code could be updated to use that feature.
    for (axis = 0; axis < mNumAxes; axis++) {
        int rc;
        if (mNumAxes == 1) {
            rc = SendCommand("t 1\r", 4);
        }
        else {
            sprintf(cmdBuf, ".%c t 1\r", 'A'+axis);
            rc = SendCommand(cmdBuf, 7);
        }
        if (rc == 0) {
            sprintf(msgBuf, axisStr, ": Starting motion", axis);
            mInterface->SendStatus(GetName()+msgBuf);
#ifndef SIMULATION
            mState[axis] = ST_MOVING;
#endif
        }
        else {
            sprintf(msgBuf, ": %s: axis %d, error %d", cmdName, axis, rc);
            mInterface->SendError(GetName()+msgBuf);
        }
    }
}

void mtsCopleyController::SetSpeed(const vctDoubleVec &spd)
{
    if (!CheckCommand("SetSpeed", spd.size()))
        return;

    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        long speedRaw = static_cast<long>((spd[axis]*m_config.axes[axis].position_bits_to_SI.scale)/VelocityBitsToCps);
        if (ParameterSet(0xcb, speedRaw, axis) == 0)
            mSpeed[axis] = spd[axis];
        else
            CMN_LOG_CLASS_RUN_ERROR << "SetSpeed failed for axis " << axis << std::endl;
    }
}

void mtsCopleyController::SetAccel(const vctDoubleVec &accel)
{
    if (!CheckCommand("SetAccel", accel.size()))
        return;

    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        long accelRaw = static_cast<long>((accel[axis]*m_config.axes[axis].position_bits_to_SI.scale)/AccelBitsToCps2);
        if (ParameterSet(0xcc, accelRaw, axis) == 0)
            mAccel[axis] = accel[axis];
        else
            CMN_LOG_CLASS_RUN_ERROR << "SetAccel failed for axis " << axis << std::endl;
    }
}

void mtsCopleyController::SetDecel(const vctDoubleVec &decel)
{
    if (!CheckCommand("SetDecel", decel.size()))
        return;

    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        long decelRaw = static_cast<long>((decel[axis]*m_config.axes[axis].position_bits_to_SI.scale)/AccelBitsToCps2);
        if (ParameterSet(0xcd, decelRaw, axis) == 0)
            mDecel[axis] = decel[axis];
        else
            CMN_LOG_CLASS_RUN_ERROR << "SetDecel failed for axis " << axis << std::endl;
    }
}

// Enable motor power
void mtsCopleyController::EnableMotorPower(void)
{
    if (!CheckCommand("EnableMotorPower"))
        return;

    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        ParameterSet(0x24, 21, axis);
    }
}

// Disable motor power
void mtsCopleyController::DisableMotorPower(void)
{
    if (!CheckCommand("DisableMotorPower"))
        return;

    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        if (ParameterSet(0x24, 0, axis) == 0)
            mState[axis] = ST_IDLE;
        else
            mInterface->SendError(GetName()+": DisableMotorPower failed");
    }
}

void mtsCopleyController::HomeAll()
{
    if (!CheckCommand("HomeAll"))
        return;

    vctBoolVec mask(mNumAxes, true);
    Home(mask);
}

void mtsCopleyController::Home(const vctBoolVec &mask)
{
    if (!CheckCommand("Home", mask.size()))
        return;

    unsigned int axis;
    // First loop sets up axes for homing. It assumes that all homing parameters, except home offset,
    // are set by the ccx file.
    for (axis = 0; axis < mNumAxes; axis++) {
        if (mask[axis]) {
            // Set home offset
            ParameterSet(0xc6, mHomeOffsetRaw[axis], axis);
            long desiredState = -1;
            ParameterGet(0x24, desiredState, axis);
            if (desiredState != 21) {
                sprintf(msgBuf, ": Home: changing state for axis %d from %ld to 21", axis, desiredState);
                mInterface->SendStatus(GetName()+msgBuf);
                ParameterSet(0x24, 21, axis);
            }
        }
    }
    // Second loop starts homing on each masked axis
    // Note that newer multi-axis drives allow the axes to be specified in the command code;
    // the following code could be updated to use that feature.
    for (axis = 0; axis < mNumAxes; axis++) {
        if (mask[axis]) {
            int rc;
            if (mNumAxes == 1) {
                rc = SendCommand("t 2\r", 4);
            }
            else {
                sprintf(cmdBuf, ".%c t 2\r", 'A'+axis);
                rc = SendCommand(cmdBuf, 7);
            }
            if (rc == 0) {
                sprintf(msgBuf, axisStr, ": Starting Home", axis);
                mInterface->SendStatus(GetName()+msgBuf);
#ifndef SIMULATION
                mIsHomed[axis] = false;
                mState[axis] = ST_HOMING;
#else
                mIsHomed[axis] = true;;
                mPosRaw[axis] = 0;
#endif
            }
            else {
                sprintf(msgBuf, "Home: axis %d, error %d", axis, rc);
                mInterface->SendError(msgBuf);
            }
        }
    }
}

void mtsCopleyController::ClearFault()
{
    if (!CheckCommand("ClearFault"))
        return;

    unsigned int axis;
    for (axis = 0; axis < mNumAxes; axis++) {
        if (mFault[axis] != 0) {
            if (ParameterSet(0xa4, mFault[axis], axis) == 0) {
                mInterface->SendStatus(GetName()+": Clearing fault");
            }
            else {
                mInterface->SendError(GetName()+": ClearFault failed");
            }
        }
    }
}

void mtsCopleyController::CheckAxisLabels(void)
{
    unsigned int axis;
    for (axis = 0; axis < mNumAxes; axis++) {
        CheckAxisLabel(axis);
    }
}

unsigned int FLAGS_DEFAULT   = 0x0;  // single decimal integer, update if different
unsigned int FLAGS_NO_UPDATE = 0x1;  // single decimal integer, do not update
unsigned int FLAGS_ARRAY3H   = 0x2;  // array of 3 hex values
unsigned int FLAGS_FILTER    = 0x4;  // filter parameters (7 or 9 values)

typedef std::map<unsigned int, unsigned int> ParameterMap;
ParameterMap parms = {   // Programmed Position Mode Parameters
                         { 0xc8, FLAGS_DEFAULT  },    // Profile mode
                         { 0xca, FLAGS_DEFAULT  },    // Position command
                         { 0xcb, FLAGS_DEFAULT  },    // Max velocity, units 0.1 cnts/s
                         { 0xcc, FLAGS_DEFAULT  },    // Max accel, units 10 cnts/s^2
                         { 0xcd, FLAGS_DEFAULT  },    // Max decel, units 10 cnts/s^2
                         { 0xce, FLAGS_DEFAULT  },    // Max jerk, units 100 cnts/s^3
                         { 0xcf, FLAGS_DEFAULT  },    // Abort decel, units 10 cnts/s^2
                         // Homing Mode Parameters
                         { 0xc2, FLAGS_DEFAULT },     // Home configuration
                         { 0xc3, FLAGS_DEFAULT  },    // Home velocity fast, units cnts/s
                         { 0xc4, FLAGS_DEFAULT  },    // Home velocity slow, units cnts/s
                         { 0xc5, FLAGS_DEFAULT  },    // Home accel/decel, units 10 cnts/s^2
                         { 0xc6, FLAGS_NO_UPDATE },   // Home offset
                         { 0xc7, FLAGS_DEFAULT  },    // Home current, units 0.01 A
                         { 0xb8, FLAGS_DEFAULT  },    // Positive software limit
                         { 0xb9, FLAGS_DEFAULT  },    // Negative software limit
                         { 0xbf, FLAGS_DEFAULT  },    // Home current delay time, units ms
                         // Current Loop Limits Parameters
                         { 0x21, FLAGS_DEFAULT  },    // Peak current, units 0.01 A
                         { 0x22, FLAGS_DEFAULT  },    // Continuous current, units 0.01 A
                         { 0x23, FLAGS_DEFAULT  },    // Peak current time limit, units ms
                         { 0xae, FLAGS_DEFAULT  },    // Current offset, units 0.01 A
                         // Current Loop Gains Parameters
                         { 0x00, FLAGS_DEFAULT },     // Cp
                         { 0x01, FLAGS_DEFAULT },     // Ci
                         { 0x02, FLAGS_DEFAULT },     // Programmed current, units 0.01 A
                         { 0x6a, FLAGS_DEFAULT },     // Commanded current ramp, units mA/s
                         // Velocity Loop Limits Parameters
                         { 0x2f, FLAGS_DEFAULT },     // Programmed velocity, units 0.1 cnts/s
                         { 0x36, FLAGS_DEFAULT },     // Accel limit, units 1000 cnts/s^2
                         { 0x37, FLAGS_DEFAULT },     // Decel limit, units 1000 cnts/s^2
                         { 0x39, FLAGS_DEFAULT },     // Fast stop ramp, units 1000 cnts/s^2
                         { 0x3a, FLAGS_DEFAULT },     // Velocity limit, units 0.1 cnts/s
                         // Velocity Loop Gains Parameters
                         { 0x27, FLAGS_DEFAULT },     // Vp
                         { 0x28, FLAGS_DEFAULT },     // Vi
                         // Position Loop Gains Parameters
                         { 0x30, FLAGS_DEFAULT },     // Pp
                         { 0x33, FLAGS_DEFAULT },     // Vel ff
                         { 0x34, FLAGS_DEFAULT },     // Accel ff
                         { 0xe3, FLAGS_DEFAULT },     // Gain mult, units 0.01
                         // Status and state
                         { 0x24, FLAGS_NO_UPDATE },   // Desired state
                         { 0xa7, FLAGS_DEFAULT },     // Fault mask
                         // Filters
                         // 7 values (2 hex and 5 float) for Plus, or 9 decimal values
                         { 0x5f, FLAGS_FILTER },      // Velocity loop output filter
                         { 0x6b, FLAGS_FILTER },      // Velocity loop command filter
                         // Output configuration
                         { 0x70, FLAGS_ARRAY3H },     // Output 1 config
                         { 0x71, FLAGS_ARRAY3H },     // Output 2 config
                         { 0x72, FLAGS_ARRAY3H },     // Output 3 config
                         { 0x73, FLAGS_ARRAY3H },     // Output 4 config
                         { 0x74, FLAGS_ARRAY3H },     // Output 5 config
                         { 0x75, FLAGS_ARRAY3H },     // Output 6 config
                         { 0x76, FLAGS_ARRAY3H },     // Output 7 config
                         { 0x77, FLAGS_ARRAY3H },     // Output 8 config
                         // Input configuration
                         { 0x78, FLAGS_DEFAULT},      // Input 1 config
                         { 0x79, FLAGS_DEFAULT},      // Input 2 config
                         { 0x7a, FLAGS_DEFAULT},      // Input 3 config
                         { 0x7b, FLAGS_DEFAULT},      // Input 4 config
                         { 0x7c, FLAGS_DEFAULT},      // Input 5 config
                         { 0x7d, FLAGS_DEFAULT},      // Input 6 config
                         { 0x7e, FLAGS_DEFAULT},      // Input 7 config
                         { 0x7f, FLAGS_DEFAULT},      // Input 8 config
                         { 0xd0, FLAGS_DEFAULT},      // Input 9 config
                         { 0xd1, FLAGS_DEFAULT},      // Input 10 config
                         { 0xd2, FLAGS_DEFAULT},      // Input 11 config
                         { 0xd3, FLAGS_DEFAULT},      // Input 12 config
                         { 0xd4, FLAGS_DEFAULT},      // Input 13 config
                         { 0xd5, FLAGS_DEFAULT},      // Input 14 config
                         { 0xd6, FLAGS_DEFAULT},      // Input 15 config
                         { 0xd7, FLAGS_DEFAULT},      // Input 16 config
                         { 0xa5, FLAGS_DEFAULT}       // Input configuration register
                        };

bool mtsCopleyController::LoadCCX(const std::string &fileName)
{
    // CCX File Format
    // First line:  version number (14)
    // Second line:  number of axes (1)
    // Subsequent lines are data, of format
    //    param-id (hex), axis-num (0), name, value
    // In most cases, value is a decimal integer, but there are some special cases:
    //    - strings
    //    - multiple values, which are separated by colon, e.g., 0:1:2
    // Special cases:
    //    - param-ids 70-77 (programmable outputs) require 3 values, in hex
    //    - param-id 95 is host config state; used by host software
    char buf[128];
    char mbuf[128];
    std::ifstream ccxFile(fileName.c_str());
    if (!ccxFile.is_open()) {
        mInterface->SendError(std::string("LoadCCX: failed to open ") + fileName);
        return false;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "LoadCCX: parsing file " << fileName << std::endl;
    ccxFile.getline(buf, sizeof(buf));
    unsigned int fileVer;
    if (sscanf(buf, "%d", &fileVer) != 1) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: failed to parse file version from [" << buf << "]" << std::endl;
        return false;
    }
    if ((fileVer != 13) && (fileVer != 14)) {
        CMN_LOG_CLASS_INIT_WARNING << "LoadCCX: unsupported file version " << fileVer << std::endl;
    }
    ccxFile.getline(buf, sizeof(buf));
    unsigned int numAxes;
    if (sscanf(buf, "%d", &numAxes) != 1) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: failed to parse number of axes from [" << buf << "]" << std::endl;
        return false;
    }
    if (numAxes != mNumAxes) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: inconsistent number of axes (json " << mNumAxes
                                 << ", ccx " << numAxes << ")" << std::endl;
        return false;
    }
    while (ccxFile.good()) {
        ccxFile.getline(buf, sizeof(buf));
        if (ccxFile.eof()) break;
        unsigned int param, axis, nchars;
        if (sscanf(buf, "%x,%d,%n", &param, &axis, &nchars) != 2) {
            CMN_LOG_CLASS_INIT_WARNING << "LoadCCX: failed to parse line [" << buf << "]" << std::endl;
            continue;
        }
        char *name = strtok(buf+nchars, ",\n");
        if (!name) {
            CMN_LOG_CLASS_INIT_WARNING << "LoadCCX: failed to parse parameter name, param " << std::hex
                                       << param << std::dec << std::endl;
            continue;
        }
        char *valueStr = strtok(0, ",\n");
        ParameterMap::const_iterator it;
        it = parms.find(param);
        if (it != parms.end()) {
            sprintf(mbuf, "Parameter %x, axis %d, %s: ", param, axis, name);
            if ((it->second == FLAGS_DEFAULT) || (it->second == FLAGS_NO_UPDATE)) {
                long value;
                if (sscanf(valueStr, "%ld", &value) != 1) {
                    CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: failed to parse parameter value, param " << std::hex
                                             << param << " from [" << valueStr << "]" << std::dec << std::endl;
                    continue;
                }
#ifndef SIMULATION
                long curValue;
                ParameterGet(param, curValue, axis);
                if (value != curValue) {
                    if (it->second == FLAGS_DEFAULT) {
                        CMN_LOG_CLASS_INIT_VERBOSE << mbuf << "updating from " << curValue
                                                   << " to " << value << std::endl;
                        ParameterSet(param, value, axis);
                    }
                    else {
                        CMN_LOG_CLASS_INIT_VERBOSE << mbuf << "current value is " << curValue
                                                   << ", not updating to " << value << std::endl;
                    }
                }
                else {
                    // CMN_LOG_CLASS_INIT_VERBOSE << mbuf << "already set to " << value << std::endl;
                }
#else
                switch (param) {
                case 0xcb:  // Max velocity
                    mSpeed[axis] = (value*VelocityBitsToCps)/m_config.axes[axis].position_bits_to_SI.scale;
                    break;

                case 0xcc:  // Max accel
                    mAccel[axis] = (value*AccelBitsToCps2)/m_config.axes[axis].position_bits_to_SI.scale;
                    break;

                case 0xcd:  // Max decel
                    mDecel[axis] = (value*AccelBitsToCps2)/m_config.axes[axis].position_bits_to_SI.scale;
                    break;
                }
#endif
            }
#ifndef SIMULATION
            else if (it->second == FLAGS_ARRAY3H) {
                long values[3];
                if (sscanf(valueStr, "%lx:%lx:%lx", &values[0], &values[1], &values[2]) != 3) {
                    CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: failed to parse parameter values (3H), param " << std::hex
                                             << param << " from [" << valueStr << "]" << std::dec << std::endl;
                    continue;
                }
                // Note: The Copley Xenus Plus appears to use only the first two values
                long curValues[2];
                if (ParameterGetArray(param, curValues, 2, axis) == 0) {
                    if ((values[0] != curValues[0]) || (values[1] != curValues[1])) {
                        CMN_LOG_CLASS_INIT_VERBOSE << mbuf << "updating from " << std::hex << curValues[0] << ", "
                                                   << curValues[1] << " to " << values[0] << ", " << values[1]
                                                   << std::dec << std::endl;
                        ParameterSetArray(param, values, 2, axis);
                    }
                    else {
                        // CMN_LOG_CLASS_INIT_VERBOSE << mbuf << "already set to " << std::hex << values[0]
                        //                            << ", " << values[1] << std::dec << std::endl;
                    }
                }
                else {
                    CMN_LOG_CLASS_INIT_WARNING << mbuf << "failed to read current values, not updating" << std::endl;
                }
            }
            else if (it->second == FLAGS_FILTER) {
                if (m_config.is_plus) {
                    // When reading, first two numbers are hex (start with 0x) and last five numbers
                    // are float (potentially in scientific notation)
                    CMN_LOG_CLASS_INIT_WARNING << mbuf << "Filter not yet supported for Plus controller" << std::endl;
                }
                else {
                    // Following code (for non-Plus controller) not tested
                    long curValues[9];
                    if (ParameterGetArray(param, curValues, 9, axis) == 0) {
                        std::cout << mbuf << "Current values:";
                        for (unsigned int i = 0; i < 9; i++)
                            std::cout << " " << curValues[i];
                        std::cout << ", TODO: Update values" << std::endl;
                    }
                    else {
                        CMN_LOG_CLASS_INIT_WARNING << mbuf << "failed to read current values" << std::endl;
                    }
                }
            }
            else {
                // Should not happen
                CMN_LOG_CLASS_INIT_WARNING << mbuf << "unsupported parameter" << std::endl;
            }
#endif
        }
    }
    return true;
}

void mtsCopleyController::CommandLoadCCX(void)
{
    if (!m_config.ccx_file.empty()) {
        mInterface->SendStatus("Loading drive data from " + m_config.ccx_file);
        std::string fullPath = mConfigPath.Find(m_config.ccx_file);
        if (!fullPath.empty()) {
            if (LoadCCX(fullPath)) {
                mInterface->SendStatus("Finished loading drive data");
            }
            else {
                mInterface->SendError("Failed to load drive data (check log)");
            }
        }
        else {
            mInterface->SendError("Could not find file in mConfigPath (check log)");
        }
    }
    else {
        mInterface->SendWarning("No ccx file specified in json configuration file");
    }
}
