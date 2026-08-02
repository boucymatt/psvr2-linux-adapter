// SPDX-License-Identifier: GPL-2.0
#include "device_provider.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <linux/input.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#include "driverlog.h"

namespace
{
std::vector<std::string> FindSenseEventDevices()
{
    std::vector<std::string> devices;
    const std::filesystem::path input_dir("/dev/input");
    std::error_code ec;
    if (!std::filesystem::exists(input_dir, ec))
        return devices;

    for (const auto &entry : std::filesystem::directory_iterator(input_dir, ec))
    {
        if (ec || !entry.is_character_file(ec))
            continue;
        const std::string filename = entry.path().filename().string();
        if (filename.rfind("event", 0) != 0)
            continue;

        const int fd = open(entry.path().c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            continue;

        input_id id{};
        std::array<char, 256> name{};
        const bool have_id = ioctl(fd, EVIOCGID, &id) == 0;
        const bool have_name = ioctl(fd, EVIOCGNAME(name.size()), name.data()) >= 0;
        close(fd);

        const std::string device_name = have_name ? name.data() : "";
        const bool sony_sense = have_id && id.vendor == 0x054c && id.product == 0x0e45;
        const bool named_sense = device_name.find("VR2 Sense") != std::string::npos ||
                                 device_name.find("PS VR2 Sense") != std::string::npos;
        if (sony_sense || named_sense)
            devices.push_back(entry.path().string());
    }

    std::sort(devices.begin(), devices.end());
    devices.erase(std::unique(devices.begin(), devices.end()), devices.end());
    return devices;
}

std::string ReadEventPathSetting(const char *key)
{
    char path[512]{};
    vr::EVRSettingsError error = vr::VRSettingsError_None;
    vr::VRSettings()->GetString("driver_psvr2", key, path, sizeof(path), &error);
    if (error == vr::VRSettingsError_None && path[0])
        return path;
    return {};
}
} // namespace

vr::EVRInitError Psvr2DeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    hmd_ = std::make_unique<Psvr2HmdDriver>();
    if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
            hmd_->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, hmd_.get()))
    {
        DriverLog("psvr2: failed to add HMD device");
        return vr::VRInitError_Driver_Unknown;
    }

    std::string left_path = ReadEventPathSetting("left_event_device");
    std::string right_path = ReadEventPathSetting("right_event_device");
    const auto discovered = FindSenseEventDevices();

    if (left_path.empty() && !discovered.empty())
        left_path = discovered[0];
    if (right_path.empty() && discovered.size() > 1)
        right_path = discovered[1];

    if (!left_path.empty())
    {
        left_controller_ = std::make_unique<Psvr2ControllerDriver>(true, left_path, hmd_.get());
        if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
                left_controller_->GetSerialNumber().c_str(),
                vr::TrackedDeviceClass_Controller, left_controller_.get()))
        {
            DriverLog("psvr2: failed to add left Sense controller");
            left_controller_.reset();
        }
    }
    else
    {
        DriverLog("psvr2: no left Sense evdev device found; set left_event_device in driver settings");
    }

    if (!right_path.empty())
    {
        right_controller_ = std::make_unique<Psvr2ControllerDriver>(false, right_path, hmd_.get());
        if (!vr::VRServerDriverHost()->TrackedDeviceAdded(
                right_controller_->GetSerialNumber().c_str(),
                vr::TrackedDeviceClass_Controller, right_controller_.get()))
        {
            DriverLog("psvr2: failed to add right Sense controller");
            right_controller_.reset();
        }
    }
    else
    {
        DriverLog("psvr2: no right Sense evdev device found; set right_event_device in driver settings");
    }

    DriverLog("psvr2: controller evdev paths: left='%s' right='%s'",
              left_path.c_str(), right_path.c_str());
    return vr::VRInitError_None;
}

void Psvr2DeviceProvider::Cleanup()
{
    right_controller_.reset();
    left_controller_.reset();
    hmd_.reset();
}

const char *const *Psvr2DeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void Psvr2DeviceProvider::RunFrame()
{
    vr::VREvent_t event{};
    while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event)))
    {
    }
}

bool Psvr2DeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

void Psvr2DeviceProvider::EnterStandby() {}
void Psvr2DeviceProvider::LeaveStandby() {}
