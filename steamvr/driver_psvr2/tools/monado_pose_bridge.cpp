// SPDX-License-Identifier: GPL-2.0
#include <openxr/openxr.h>

#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <thread>
#include <unistd.h>

#include "../src/pose_bridge_protocol.h"

static bool ok(XrResult r,const char *what){if(XR_FAILED(r)){std::fprintf(stderr,"%s failed: %d\n",what,r);return false;}return true;}
static void send_pose(int fd,uint16_t port,uint8_t hand,uint64_t seq,const XrSpaceLocation &loc)
{
    psvr2_bridge::PosePacket p{};p.hand=hand;p.sequence=seq;
    p.valid=(loc.locationFlags&(XR_SPACE_LOCATION_POSITION_VALID_BIT|XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))==(XR_SPACE_LOCATION_POSITION_VALID_BIT|XR_SPACE_LOCATION_ORIENTATION_VALID_BIT);
    p.position[0]=loc.pose.position.x;p.position[1]=loc.pose.position.y;p.position[2]=loc.pose.position.z;
    p.orientation[0]=loc.pose.orientation.w;p.orientation[1]=loc.pose.orientation.x;p.orientation[2]=loc.pose.orientation.y;p.orientation[3]=loc.pose.orientation.z;
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(port);
    sendto(fd,&p,sizeof(p),0,reinterpret_cast<sockaddr*>(&a),sizeof(a));
}
int main()
{
    const char *exts[]={"XR_MND_headless"};
    XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};std::strcpy(ici.applicationInfo.applicationName,"psvr2-monado-pose-bridge");ici.applicationInfo.apiVersion=XR_CURRENT_API_VERSION;ici.enabledExtensionCount=1;ici.enabledExtensionNames=exts;
    XrInstance inst=XR_NULL_HANDLE;if(!ok(xrCreateInstance(&ici,&inst),"xrCreateInstance"))return 1;
    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};sgi.formFactor=XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;XrSystemId system;if(!ok(xrGetSystem(inst,&sgi,&system),"xrGetSystem"))return 1;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};sci.systemId=system;XrSession session;if(!ok(xrCreateSession(inst,&sci,&session),"xrCreateSession"))return 1;
    XrReferenceSpaceCreateInfo rs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};rs.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_LOCAL;rs.poseInReferenceSpace.orientation.w=1;XrSpace base;if(!ok(xrCreateReferenceSpace(session,&rs,&base),"xrCreateReferenceSpace"))return 1;
    XrPath left,right;xrStringToPath(inst,"/user/hand/left",&left);xrStringToPath(inst,"/user/hand/right",&right);XrPath hands[]={left,right};
    XrActionSetCreateInfo asci{XR_TYPE_ACTION_SET_CREATE_INFO};std::strcpy(asci.actionSetName,"sense");std::strcpy(asci.localizedActionSetName,"Sense");XrActionSet aset;xrCreateActionSet(inst,&asci,&aset);
    XrActionCreateInfo aci{XR_TYPE_ACTION_CREATE_INFO};aci.actionType=XR_ACTION_TYPE_POSE_INPUT;std::strcpy(aci.actionName,"grip_pose");std::strcpy(aci.localizedActionName,"Grip pose");aci.countSubactionPaths=2;aci.subactionPaths=hands;XrAction action;xrCreateAction(aset,&aci,&action);
    XrPath profile,lgrip,rgrip;xrStringToPath(inst,"/interaction_profiles/oculus/touch_controller",&profile);xrStringToPath(inst,"/user/hand/left/input/grip/pose",&lgrip);xrStringToPath(inst,"/user/hand/right/input/grip/pose",&rgrip);XrActionSuggestedBinding binds[]={{action,lgrip},{action,rgrip}};XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};sb.interactionProfile=profile;sb.countSuggestedBindings=2;sb.suggestedBindings=binds;xrSuggestInteractionProfileBindings(inst,&sb);
    XrSessionActionSetsAttachInfo ai{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};ai.countActionSets=1;ai.actionSets=&aset;xrAttachSessionActionSets(session,&ai);
    XrActionSpaceCreateInfo sp{XR_TYPE_ACTION_SPACE_CREATE_INFO};sp.action=action;sp.poseInActionSpace.orientation.w=1;XrSpace spaces[2];sp.subactionPath=left;xrCreateActionSpace(session,&sp,&spaces[0]);sp.subactionPath=right;xrCreateActionSpace(session,&sp,&spaces[1]);
    XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};bi.primaryViewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;xrBeginSession(session,&bi);
    int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);uint64_t seq=0;
    std::puts("PSVR2 Monado pose bridge running. Keep monado-service active.");
    for(;;){XrActiveActionSet active{aset,XR_NULL_PATH};XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};sync.countActiveActionSets=1;sync.activeActionSets=&active;xrSyncActions(session,&sync);
        XrTime now=0;XrFrameWaitInfo wi{XR_TYPE_FRAME_WAIT_INFO};XrFrameState fs{XR_TYPE_FRAME_STATE};if(XR_SUCCEEDED(xrWaitFrame(session,&wi,&fs)))now=fs.predictedDisplayTime;
        for(int i=0;i<2;i++){XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};xrLocateSpace(spaces[i],base,now,&loc);send_pose(fd,i?psvr2_bridge::kRightPort:psvr2_bridge::kLeftPort,i,seq,loc);}++seq;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
}
