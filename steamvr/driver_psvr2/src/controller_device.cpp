// SPDX-License-Identifier: GPL-2.0
#include "controller_device.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "driverlog.h"
#include "hmd_device_driver.h"

namespace
{
float NormalizeAxis(int32_t value, int32_t minimum, int32_t maximum)
{
    if (maximum <= minimum) return 0.0f;
    const float t = static_cast<float>(value - minimum) / static_cast<float>(maximum - minimum);
    return std::clamp(t * 2.0f - 1.0f, -1.0f, 1.0f);
}
float NormalizeTrigger(int32_t value, int32_t maximum = 255)
{
    if (maximum <= 0) return 0.0f;
    return std::clamp(static_cast<float>(value) / static_cast<float>(maximum), 0.0f, 1.0f);
}
vr::HmdQuaternion_t Multiply(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b)
{
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
vr::HmdQuaternion_t AxisAngle(double x,double y,double z,double r)
{
    const double h=r*0.5,s=std::sin(h); return {std::cos(h),x*s,y*s,z*s};
}
void RotateVector(const vr::HmdQuaternion_t &q,double x,double y,double z,double &ox,double &oy,double &oz)
{
    const double tx=2*(q.y*z-q.z*y),ty=2*(q.z*x-q.x*z),tz=2*(q.x*y-q.y*x);
    ox=x+q.w*tx+(q.y*tz-q.z*ty); oy=y+q.w*ty+(q.z*tx-q.x*tz); oz=z+q.w*tz+(q.x*ty-q.y*tx);
}
}

Psvr2ControllerDriver::Psvr2ControllerDriver(bool left,std::string path,Psvr2HmdDriver *hmd)
    : left_hand_(left),event_path_(std::move(path)),hmd_(hmd)
{ serial_number_=left_hand_?"PSVR2-SENSE-L":"PSVR2-SENSE-R"; }
Psvr2ControllerDriver::~Psvr2ControllerDriver(){ Deactivate(); }

bool Psvr2ControllerDriver::OpenPoseSocket()
{
    pose_fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(pose_fd_<0) return false;
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    a.sin_port=htons(left_hand_?psvr2_bridge::kLeftPort:psvr2_bridge::kRightPort);
    if(bind(pose_fd_,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0){ close(pose_fd_); pose_fd_=-1; return false; }
    return true;
}

vr::EVRInitError Psvr2ControllerDriver::Activate(uint32_t id)
{
    device_index_=id; const auto c=vr::VRProperties()->TrackedDeviceToPropertyContainer(id);
    vr::VRProperties()->SetStringProperty(c,vr::Prop_ManufacturerName_String,"Sony");
    vr::VRProperties()->SetStringProperty(c,vr::Prop_ModelNumber_String,left_hand_?"PS VR2 Sense Controller (L)":"PS VR2 Sense Controller (R)");
    vr::VRProperties()->SetStringProperty(c,vr::Prop_RenderModelName_String,left_hand_?"{psvr2}/rendermodels/psvr2_sense_left":"{psvr2}/rendermodels/psvr2_sense_right");
    vr::VRProperties()->SetStringProperty(c,vr::Prop_InputProfilePath_String,"{psvr2}/input/psvr2_sense_profile.json");
    vr::VRProperties()->SetStringProperty(c,vr::Prop_ControllerType_String,"psvr2_sense");
    vr::VRProperties()->SetInt32Property(c,vr::Prop_ControllerRoleHint_Int32,left_hand_?vr::TrackedControllerRole_LeftHand:vr::TrackedControllerRole_RightHand);
    vr::VRProperties()->SetBoolProperty(c,vr::Prop_DeviceIsWireless_Bool,true);

    auto *in=vr::VRDriverInput();
    in->CreateBooleanComponent(c,"/input/trigger/click",&trigger_click_);
    in->CreateScalarComponent(c,"/input/trigger/value",&trigger_value_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedOneSided);
    in->CreateBooleanComponent(c,"/input/squeeze/click",&grip_click_);
    in->CreateScalarComponent(c,"/input/squeeze/value",&grip_value_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedOneSided);
    in->CreateScalarComponent(c,"/input/thumbstick/x",&joystick_x_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedTwoSided);
    in->CreateScalarComponent(c,"/input/thumbstick/y",&joystick_y_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedTwoSided);
    in->CreateBooleanComponent(c,"/input/thumbstick/click",&joystick_click_);
    in->CreateBooleanComponent(c,"/input/a/click",&primary_click_);
    in->CreateBooleanComponent(c,"/input/b/click",&secondary_click_);
    in->CreateBooleanComponent(c,"/input/system/click",&system_click_);
    in->CreateBooleanComponent(c,"/input/application_menu/click",&menu_click_);

    event_fd_=open(event_path_.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
    if(event_fd_<0) return vr::VRInitError_Driver_Failed;
    OpenPoseSocket();
    active_=true; input_thread_=std::thread(&Psvr2ControllerDriver::InputThread,this); SubmitPose();
    DriverLog("psvr2: %s controller input %s; bridge UDP port %u",left_hand_?"left":"right",event_path_.c_str(),left_hand_?psvr2_bridge::kLeftPort:psvr2_bridge::kRightPort);
    return vr::VRInitError_None;
}

void Psvr2ControllerDriver::Deactivate()
{
    active_=false; if(input_thread_.joinable()) input_thread_.join();
    if(event_fd_>=0){close(event_fd_);event_fd_=-1;} if(pose_fd_>=0){close(pose_fd_);pose_fd_=-1;}
    device_index_=vr::k_unTrackedDeviceIndexInvalid;
}
void Psvr2ControllerDriver::EnterStandby(){}
void *Psvr2ControllerDriver::GetComponent(const char*){return nullptr;}
void Psvr2ControllerDriver::DebugRequest(const char*,char *r,uint32_t n){if(n)r[0]='\0';}

void Psvr2ControllerDriver::ReceivePosePackets()
{
    if(pose_fd_<0) return;
    psvr2_bridge::PosePacket p{};
    while(recv(pose_fd_,&p,sizeof(p),0)==sizeof(p))
    {
        if(p.magic!=psvr2_bridge::kMagic||p.version!=psvr2_bridge::kVersion||p.hand!=(left_hand_?0:1)) continue;
        std::lock_guard<std::mutex> lock(pose_mutex_); bridge_pose_=p; bridge_pose_time_=std::chrono::steady_clock::now();
    }
}

vr::DriverPose_t Psvr2ControllerDriver::BuildPose() const
{
    { std::lock_guard<std::mutex> lock(pose_mutex_);
      if(bridge_pose_.valid && std::chrono::steady_clock::now()-bridge_pose_time_<std::chrono::milliseconds(250))
      {
          vr::DriverPose_t p{}; p.qWorldFromDriverRotation.w=1; p.qDriverFromHeadRotation.w=1;
          for(int i=0;i<3;i++){p.vecPosition[i]=bridge_pose_.position[i];p.vecVelocity[i]=bridge_pose_.linear_velocity[i];p.vecAngularVelocity[i]=bridge_pose_.angular_velocity[i];}
          p.qRotation={bridge_pose_.orientation[0],bridge_pose_.orientation[1],bridge_pose_.orientation[2],bridge_pose_.orientation[3]};
          p.deviceIsConnected=true;p.poseIsValid=true;p.result=vr::TrackingResult_Running_OK; return p;
      }}
    return BuildHeadRelativePose();
}

vr::DriverPose_t Psvr2ControllerDriver::BuildHeadRelativePose() const
{
    vr::DriverPose_t p{};p.qWorldFromDriverRotation.w=1;p.qDriverFromHeadRotation.w=1;p.qRotation.w=1;p.deviceIsConnected=event_fd_>=0;
    if(!hmd_||!p.deviceIsConnected){p.result=vr::TrackingResult_Uninitialized;return p;}
    const auto h=hmd_->GetPose(); if(!h.poseIsValid){p.result=vr::TrackingResult_Running_OutOfRange;return p;}
    double ox,oy,oz;RotateVector(h.qRotation,left_hand_?-0.23:0.23,-0.24,-0.38,ox,oy,oz);
    p.vecPosition[0]=h.vecPosition[0]+ox;p.vecPosition[1]=h.vecPosition[1]+oy;p.vecPosition[2]=h.vecPosition[2]+oz;
    constexpr double pi=3.141592653589793; p.qRotation=Multiply(Multiply(h.qRotation,AxisAngle(0,1,0,(left_hand_?-8:8)*pi/180)),AxisAngle(1,0,0,-22*pi/180));
    p.poseIsValid=true;p.result=vr::TrackingResult_Running_OK;return p;
}
vr::DriverPose_t Psvr2ControllerDriver::GetPose(){return BuildPose();}
void Psvr2ControllerDriver::SubmitPose(){auto i=device_index_.load();if(i!=vr::k_unTrackedDeviceIndexInvalid){auto p=BuildPose();vr::VRServerDriverHost()->TrackedDevicePoseUpdated(i,p,sizeof(p));}}
void Psvr2ControllerDriver::UpdateBoolean(vr::VRInputComponentHandle_t h,bool v){if(h)vr::VRDriverInput()->UpdateBooleanComponent(h,v,0);}
void Psvr2ControllerDriver::UpdateScalar(vr::VRInputComponentHandle_t h,float v){if(h)vr::VRDriverInput()->UpdateScalarComponent(h,v,0);}

void Psvr2ControllerDriver::HandleEvent(uint16_t t,uint16_t c,int32_t v)
{
    if(t==EV_ABS){input_absinfo a{};if(ioctl(event_fd_,EVIOCGABS(c),&a)<0){a.minimum=0;a.maximum=255;}
        if(c==ABS_X)UpdateScalar(joystick_x_,NormalizeAxis(v,a.minimum,a.maximum));
        else if(c==ABS_Y)UpdateScalar(joystick_y_,-NormalizeAxis(v,a.minimum,a.maximum));
        else if(c==ABS_Z||c==ABS_RZ){UpdateScalar(trigger_value_,NormalizeTrigger(v,a.maximum));UpdateBoolean(trigger_click_,v>a.maximum*3/4);}
        else if(c==ABS_RX||c==ABS_RY){UpdateScalar(grip_value_,NormalizeTrigger(v,a.maximum));UpdateBoolean(grip_click_,v>a.maximum/2);} return;}
    if(t!=EV_KEY)return;const bool on=v!=0;
    if(c==BTN_THUMBL||c==BTN_THUMBR)UpdateBoolean(joystick_click_,on);
    else if(c==BTN_SOUTH||c==BTN_WEST)UpdateBoolean(primary_click_,on);
    else if(c==BTN_EAST||c==BTN_NORTH)UpdateBoolean(secondary_click_,on);
    else if(c==BTN_MODE)UpdateBoolean(system_click_,on);
    else if(c==BTN_SELECT||c==BTN_START)UpdateBoolean(menu_click_,on);
    else if(c==BTN_TL||c==BTN_TR||c==319){UpdateBoolean(grip_click_,on);UpdateScalar(grip_value_,on?1.f:0.f);}
    else if(c==BTN_TL2||c==BTN_TR2)UpdateBoolean(trigger_click_,on);
}

void Psvr2ControllerDriver::InputThread()
{
    pollfd fds[2]={{event_fd_,POLLIN,0},{pose_fd_,POLLIN,0}};
    while(active_){int n=poll(fds,pose_fd_>=0?2:1,10);if(n<0&&errno!=EINTR)break;
        if(fds[0].revents&POLLIN){input_event e[32]{};ssize_t b=read(event_fd_,e,sizeof(e));for(size_t i=0;b>0&&i<static_cast<size_t>(b)/sizeof(input_event);++i)HandleEvent(e[i].type,e[i].code,e[i].value);}
        if(pose_fd_>=0&&(fds[1].revents&POLLIN))ReceivePosePackets(); SubmitPose();}
}
