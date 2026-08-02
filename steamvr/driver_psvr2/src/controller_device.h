// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "openvr_driver.h"

class Psvr2HmdDriver;

class Psvr2ControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
    Psvr2ControllerDriver(bool left_hand, std::string event_path, Psvr2HmdDriver *hmd);
    ~Psvr2ControllerDriver();

    vr::EVRInitError Activate(uint32_t object_id) override;
    void Deactivate() override;
    void EnterStandby() override;
    void *GetComponent(const char *component_name_and_version) override;
    void DebugRequest(const char *request, char *response, uint32_t response_size) override;
    vr::DriverPose_t GetPose() override;

    const std::string &GetSerialNumber() const { return serial_number_; }
    bool IsLeft() const { return left_hand_; }

private:
    void InputThread();
    void HandleEvent(uint16_t type, uint16_t code, int32_t value);
    void UpdateBoolean(vr::VRInputComponentHandle_t handle, bool value);
    void UpdateScalar(vr::VRInputComponentHandle_t handle, float value);
    void SubmitPose();
    vr::DriverPose_t BuildHeadRelativePose() const;

    bool left_hand_ = false;
    std::string event_path_;
    std::string serial_number_;
    Psvr2HmdDriver *hmd_ = nullptr;

    std::atomic<bool> active_{false};
    std::atomic<uint32_t> device_index_{vr::k_unTrackedDeviceIndexInvalid};
    std::thread input_thread_;
    int event_fd_ = -1;

    vr::VRInputComponentHandle_t trigger_click_ = 0;
    vr::VRInputComponentHandle_t trigger_value_ = 0;
    vr::VRInputComponentHandle_t grip_click_ = 0;
    vr::VRInputComponentHandle_t grip_value_ = 0;
    vr::VRInputComponentHandle_t joystick_x_ = 0;
    vr::VRInputComponentHandle_t joystick_y_ = 0;
    vr::VRInputComponentHandle_t joystick_click_ = 0;
    vr::VRInputComponentHandle_t primary_click_ = 0;
    vr::VRInputComponentHandle_t secondary_click_ = 0;
    vr::VRInputComponentHandle_t system_click_ = 0;
    vr::VRInputComponentHandle_t menu_click_ = 0;
};
