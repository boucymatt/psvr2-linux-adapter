// SPDX-License-Identifier: GPL-2.0
#include "controller_device.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "driverlog.h"
#include "hmd_device_driver.h"

namespace
{
float NormalizeAxis(int32_t value, int32_t minimum, int32_t maximum)
{
    if (maximum <= minimum)
        return 0.0f;
    const float t = static_cast<float>(value - minimum) /
                    static_cast<float>(maximum - minimum);
    return std::clamp(t * 2.0f - 1.0f, -1.0f, 1.0f);
}

float NormalizeTrigger(int32_t value, int32_t maximum = 255)
{
    if (maximum <= 0)
        return 0.0f;
    return std::clamp(static_cast<float>(value) / static_cast<float>(maximum), 0.0f, 1.0f);
}

vr::HmdQuaternion_t Multiply(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b)
{
    vr::HmdQuaternion_t q{};
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return q;
}

vr::HmdQuaternion_t AxisAngle(double x, double y, double z, double radians)
{
    const double half = radians * 0.5;
    const double s = std::sin(half);
    vr::HmdQuaternion_t q{};
    q.w = std::cos(half);
    q.x = x * s;
    q.y = y * s;
    q.z = z * s;
    return q;
}

void RotateVector(const vr::HmdQuaternion_t &q, double x, double y, double z,
                  double &out_x, double &out_y, double &out_z)
{
    const double tx = 2.0 * (q.y * z - q.z * y);
    const double ty = 2.0 * (q.z * x - q.x * z);
    const double tz = 2.0 * (q.x * y - q.y * x);
    out_x = x + q.w * tx + (q.y * tz - q.z * ty);
    out_y = y + q.w * ty + (q.z * tx - q.x * tz);
    out_z = z + q.w * tz + (q.x * ty - q.y * tx);
}
} // namespace

Psvr2ControllerDriver::Psvr2ControllerDriver(bool left_hand, std::string event_path,
                                             Psvr2HmdDriver *hmd)
    : left_hand_(left_hand), event_path_(std::move(event_path)), hmd_(hmd)
{
    serial_number_ = left_hand_ ? "PSVR2-SENSE-L" : "PSVR2-SENSE-R";
}

Psvr2ControllerDriver::~Psvr2ControllerDriver()
{
    Deactivate();
}

vr::EVRInitError Psvr2ControllerDriver::Activate(uint32_t object_id)
{
    device_index_ = object_id;
    const auto container = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id);

    vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "Sony");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String,
                                           left_hand_ ? "PS VR2 Sense Controller (L)"
                                                      : "PS VR2 Sense Controller (R)");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_RenderModelName_String,
                                           "vr_controller_vive_1_5");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
                                           "{psvr2}/input/psvr2_sense_profile.json");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ControllerType_String,
                                           "psvr2_sense");
    vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32,
        left_hand_ ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand);
    vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceIsWireless_Bool, true);
    vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);

    auto *input = vr::VRDriverInput();
    input->CreateBooleanComponent(container, "/input/trigger/click", &trigger_click_);
    input->CreateScalarComponent(container, "/input/trigger/value", &trigger_value_,
                                 vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    input->CreateBooleanComponent(container, "/input/squeeze/click", &grip_click_);
    input->CreateScalarComponent(container, "/input/squeeze/value", &grip_value_,
                                 vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    input->CreateScalarComponent(container, "/input/thumbstick/x", &joystick_x_,
                                 vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    input->CreateScalarComponent(container, "/input/thumbstick/y", &joystick_y_,
                                 vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    input->CreateBooleanComponent(container, "/input/thumbstick/click", &joystick_click_);
    input->CreateBooleanComponent(container, "/input/a/click", &primary_click_);
    input->CreateBooleanComponent(container, "/input/b/click", &secondary_click_);
    input->CreateBooleanComponent(container, "/input/system/click", &system_click_);
    input->CreateBooleanComponent(container, "/input/application_menu/click", &menu_click_);

    event_fd_ = open(event_path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (event_fd_ < 0)
    {
        DriverLog("psvr2: failed to open %s for %s Sense controller: %s",
                  event_path_.c_str(), left_hand_ ? "left" : "right", std::strerror(errno));
        return vr::VRInitError_Driver_Failed;
    }

    active_ = true;
    input_thread_ = std::thread(&Psvr2ControllerDriver::InputThread, this);
    SubmitPose();
    DriverLog("psvr2: opened %s for %s Sense controller input with head-relative pose fallback",
              event_path_.c_str(), left_hand_ ? "left" : "right");
    return vr::VRInitError_None;
}

void Psvr2ControllerDriver::Deactivate()
{
    active_ = false;
    if (input_thread_.joinable())
        input_thread_.join();
    if (event_fd_ >= 0)
    {
        close(event_fd_);
        event_fd_ = -1;
    }
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void Psvr2ControllerDriver::EnterStandby() {}
void *Psvr2ControllerDriver::GetComponent(const char *) { return nullptr; }

void Psvr2ControllerDriver::DebugRequest(const char *, char *response, uint32_t response_size)
{
    if (response_size)
        response[0] = '\0';
}

vr::DriverPose_t Psvr2ControllerDriver::BuildHeadRelativePose() const
{
    vr::DriverPose_t pose{};
    pose.qWorldFromDriverRotation.w = 1.0;
    pose.qDriverFromHeadRotation.w = 1.0;
    pose.qRotation.w = 1.0;
    pose.deviceIsConnected = event_fd_ >= 0;

    if (!hmd_ || !pose.deviceIsConnected)
    {
        pose.poseIsValid = false;
        pose.result = vr::TrackingResult_Uninitialized;
        return pose;
    }

    const vr::DriverPose_t head = hmd_->GetPose();
    if (!head.poseIsValid)
    {
        pose.poseIsValid = false;
        pose.result = vr::TrackingResult_Running_OutOfRange;
        return pose;
    }

    const double side = left_hand_ ? -0.23 : 0.23;
    const double down = -0.24;
    const double forward = -0.38;
    double ox = 0.0, oy = 0.0, oz = 0.0;
    RotateVector(head.qRotation, side, down, forward, ox, oy, oz);

    pose.vecPosition[0] = head.vecPosition[0] + ox;
    pose.vecPosition[1] = head.vecPosition[1] + oy;
    pose.vecPosition[2] = head.vecPosition[2] + oz;

    constexpr double kPi = 3.14159265358979323846;
    const auto pitch = AxisAngle(1.0, 0.0, 0.0, -22.0 * kPi / 180.0);
    const auto yaw = AxisAngle(0.0, 1.0, 0.0, (left_hand_ ? -8.0 : 8.0) * kPi / 180.0);
    pose.qRotation = Multiply(Multiply(head.qRotation, yaw), pitch);

    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.shouldApplyHeadModel = false;
    pose.willDriftInYaw = false;
    return pose;
}

vr::DriverPose_t Psvr2ControllerDriver::GetPose()
{
    return BuildHeadRelativePose();
}

void Psvr2ControllerDriver::SubmitPose()
{
    const uint32_t index = device_index_.load();
    if (index == vr::k_unTrackedDeviceIndexInvalid)
        return;
    const vr::DriverPose_t pose = BuildHeadRelativePose();
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(index, pose, sizeof(pose));
}

void Psvr2ControllerDriver::UpdateBoolean(vr::VRInputComponentHandle_t handle, bool value)
{
    if (handle)
        vr::VRDriverInput()->UpdateBooleanComponent(handle, value, 0.0);
}

void Psvr2ControllerDriver::UpdateScalar(vr::VRInputComponentHandle_t handle, float value)
{
    if (handle)
        vr::VRDriverInput()->UpdateScalarComponent(handle, value, 0.0);
}

void Psvr2ControllerDriver::HandleEvent(uint16_t type, uint16_t code, int32_t value)
{
    if (type == EV_ABS)
    {
        input_absinfo info{};
        if (ioctl(event_fd_, EVIOCGABS(code), &info) < 0)
        {
            info.minimum = 0;
            info.maximum = 255;
        }

        switch (code)
        {
        case ABS_X:
            UpdateScalar(joystick_x_, NormalizeAxis(value, info.minimum, info.maximum));
            break;
        case ABS_Y:
            UpdateScalar(joystick_y_, -NormalizeAxis(value, info.minimum, info.maximum));
            break;
        case ABS_Z:
        case ABS_RZ:
            UpdateScalar(trigger_value_, NormalizeTrigger(value, info.maximum));
            UpdateBoolean(trigger_click_, value > (info.maximum * 3 / 4));
            break;
        case ABS_RX:
        case ABS_RY:
            UpdateScalar(grip_value_, NormalizeTrigger(value, info.maximum));
            UpdateBoolean(grip_click_, value > (info.maximum / 2));
            break;
        default:
            break;
        }
        return;
    }

    if (type != EV_KEY)
        return;

    const bool pressed = value != 0;
    switch (code)
    {
    case BTN_THUMBL:
    case BTN_THUMBR:
        UpdateBoolean(joystick_click_, pressed);
        break;
    case BTN_SOUTH:
    case BTN_WEST:
        UpdateBoolean(primary_click_, pressed);
        break;
    case BTN_EAST:
    case BTN_NORTH:
        UpdateBoolean(secondary_click_, pressed);
        break;
    case BTN_MODE:
        UpdateBoolean(system_click_, pressed);
        break;
    case BTN_SELECT:
    case BTN_START:
        UpdateBoolean(menu_click_, pressed);
        break;
    case BTN_TL:
    case BTN_TR:
    case 319:
        UpdateBoolean(grip_click_, pressed);
        UpdateScalar(grip_value_, pressed ? 1.0f : 0.0f);
        break;
    case BTN_TL2:
    case BTN_TR2:
        UpdateBoolean(trigger_click_, pressed);
        break;
    default:
        break;
    }
}

void Psvr2ControllerDriver::InputThread()
{
    pollfd pfd{event_fd_, POLLIN, 0};
    while (active_)
    {
        const int ready = poll(&pfd, 1, 10);
        if (ready < 0)
        {
            if (errno == EINTR)
                continue;
            break;
        }

        if (ready > 0 && (pfd.revents & POLLIN))
        {
            input_event events[32]{};
            const ssize_t count = read(event_fd_, events, sizeof(events));
            if (count > 0)
            {
                const size_t n = static_cast<size_t>(count) / sizeof(input_event);
                for (size_t i = 0; i < n; ++i)
                    HandleEvent(events[i].type, events[i].code, events[i].value);
            }
            else if (count < 0 && errno != EAGAIN && errno != EINTR)
            {
                break;
            }
        }

        SubmitPose();
    }
}
