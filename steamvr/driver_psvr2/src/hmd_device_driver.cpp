// SPDX-License-Identifier: GPL-2.0
#include "hmd_device_driver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include "driverlog.h"

static const char *kSettingsSection = "driver_psvr2";

static constexpr int32_t kEdidVendorId = 0x4DD9;
static constexpr int32_t kEdidProductId = 0xA205;

namespace
{
float ReadFloatSetting( const char *key, float fallback )
{
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	const float value = vr::VRSettings()->GetFloat( kSettingsSection, key, &err );
	return err == vr::VRSettingsError_None ? value : fallback;
}

float ClampUv( float value )
{
	return std::clamp( value, 0.0f, 1.0f );
}

void DistortChannel( float u, float v, float center_x, float center_y,
                     float k1, float k2, float k3, float chroma_scale,
                     float out[2] )
{
	// Work in an aspect-corrected eye coordinate system so the radial profile is
	// circular in angular space rather than stretched by the nearly-square panel.
	constexpr float eye_aspect = 2000.0f / 2040.0f;
	float x = ( u - center_x ) * 2.0f * eye_aspect;
	float y = ( v - center_y ) * 2.0f;
	const float r2 = x * x + y * y;
	const float radial = 1.0f + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
	const float scale = radial * chroma_scale;
	x *= scale;
	y *= scale;
	out[0] = ClampUv( center_x + x / ( 2.0f * eye_aspect ) );
	out[1] = ClampUv( center_y + y / 2.0f );
}
} // namespace

Psvr2HmdDriver::Psvr2HmdDriver()
{
	char buf[256] = { 0 };
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSettingsSection, "model_number", buf, sizeof( buf ), &err );
	model_number_ = ( err == vr::VRSettingsError_None && buf[0] ) ? buf : "PSVR2";

	buf[0] = 0;
	err = vr::VRSettingsError_None;
	vr::VRSettings()->GetString( kSettingsSection, "serial_number", buf, sizeof( buf ), &err );
	serial_number_ = ( err == vr::VRSettingsError_None && buf[0] ) ? buf : "PSVR2-0001";

	display_frequency_ = ReadFloatSetting( "display_frequency", 90.0f );
	user_ipd_meters_ = ReadFloatSetting( "default_ipd_meters", 0.064f );
	if ( user_ipd_meters_ < 0.055f || user_ipd_meters_ > 0.075f )
		user_ipd_meters_ = 0.064f;

	Psvr2DisplayConfig cfg{};
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_x" ) ) cfg.window_x = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_y" ) ) cfg.window_y = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_width" ) ) cfg.window_width = v;
	if ( int32_t v = vr::VRSettings()->GetInt32( kSettingsSection, "window_height" ) ) cfg.window_height = v;
	cfg.render_width = cfg.window_width / 2;
	cfg.render_height = cfg.window_height;

	cfg.left_eye_left_tan = ReadFloatSetting( "left_eye_left_tan", cfg.left_eye_left_tan );
	cfg.left_eye_right_tan = ReadFloatSetting( "left_eye_right_tan", cfg.left_eye_right_tan );
	cfg.right_eye_left_tan = ReadFloatSetting( "right_eye_left_tan", cfg.right_eye_left_tan );
	cfg.right_eye_right_tan = ReadFloatSetting( "right_eye_right_tan", cfg.right_eye_right_tan );
	cfg.top_tan = ReadFloatSetting( "top_tan", cfg.top_tan );
	cfg.bottom_tan = ReadFloatSetting( "bottom_tan", cfg.bottom_tan );

	cfg.distortion_k1 = ReadFloatSetting( "distortion_k1", cfg.distortion_k1 );
	cfg.distortion_k2 = ReadFloatSetting( "distortion_k2", cfg.distortion_k2 );
	cfg.distortion_k3 = ReadFloatSetting( "distortion_k3", cfg.distortion_k3 );
	cfg.chroma_red_scale = ReadFloatSetting( "chroma_red_scale", cfg.chroma_red_scale );
	cfg.chroma_blue_scale = ReadFloatSetting( "chroma_blue_scale", cfg.chroma_blue_scale );
	cfg.left_lens_center_x = ReadFloatSetting( "left_lens_center_x", cfg.left_lens_center_x );
	cfg.right_lens_center_x = ReadFloatSetting( "right_lens_center_x", cfg.right_lens_center_x );
	cfg.lens_center_y = ReadFloatSetting( "lens_center_y", cfg.lens_center_y );

	vr::EVRSettingsError dm_err = vr::VRSettingsError_None;
	const bool direct_mode = vr::VRSettings()->GetBool( kSettingsSection, "direct_mode", &dm_err );
	cfg.direct_mode = ( dm_err == vr::VRSettingsError_None ) ? direct_mode : true;
	direct_mode_ = cfg.direct_mode;

	display_ = std::make_unique<Psvr2DisplayComponent>( cfg );
	pose_source_ = std::make_unique<PoseSource>();

	if ( !pose_source_->HasPose() )
		DriverLog( "psvr2: /dev/psvr2-pose not found — HMD will not track (is the module loaded?)" );
}

vr::EVRInitError Psvr2HmdDriver::Activate( uint32_t unObjectId )
{
	device_index_ = unObjectId;

	vr::PropertyContainerHandle_t c = vr::VRProperties()->TrackedDeviceToPropertyContainer( unObjectId );
	vr::VRProperties()->SetStringProperty( c, vr::Prop_ModelNumber_String, model_number_.c_str() );
	vr::VRProperties()->SetStringProperty( c, vr::Prop_ManufacturerName_String, "Sony" );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_UserIpdMeters_Float, user_ipd_meters_ );

	vr::VRProperties()->SetFloatProperty( c, vr::Prop_DisplayFrequency_Float, display_frequency_ );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.012f );
	vr::VRProperties()->SetFloatProperty( c, vr::Prop_SecondsFromVsyncToPhotons_Float, 0.011f );

	vr::VRProperties()->SetInt32Property( c, vr::Prop_EdidVendorID_Int32, kEdidVendorId );
	vr::VRProperties()->SetInt32Property( c, vr::Prop_EdidProductID_Int32, kEdidProductId );
	vr::VRProperties()->SetBoolProperty( c, vr::Prop_IsOnDesktop_Bool, !direct_mode_ );

	DriverLog( "psvr2: display mode = %s (EDID %04X:%04X, IPD %.1f mm)",
	           direct_mode_ ? "direct (DRM lease)" : "extended desktop",
	           kEdidVendorId, kEdidProductId, user_ipd_meters_ * 1000.0f );

	active_ = true;
	pose_thread_ = std::thread( &Psvr2HmdDriver::PoseThread, this );
	return vr::VRInitError_None;
}

void Psvr2HmdDriver::Deactivate()
{
	if ( active_.exchange( false ) && pose_thread_.joinable() )
		pose_thread_.join();
	device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void Psvr2HmdDriver::EnterStandby() {}

void *Psvr2HmdDriver::GetComponent( const char *pchComponentNameAndVersion )
{
	if ( std::strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) == 0 )
		return display_.get();
	return nullptr;
}

void Psvr2HmdDriver::DebugRequest( const char *, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
	if ( unResponseBufferSize >= 1 )
		pchResponseBuffer[0] = 0;
}

vr::DriverPose_t Psvr2HmdDriver::GetPose()
{
	return last_pose_;
}

void Psvr2HmdDriver::PoseThread()
{
	while ( active_ )
	{
		vr::DriverPose_t pose{};
		if ( pose_source_->ReadPose( pose ) )
		{
			last_pose_ = pose;
			if ( device_index_ != vr::k_unTrackedDeviceIndexInvalid )
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated( device_index_, pose, sizeof( pose ) );
		}
		else
		{
			std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
		}
	}
}

Psvr2DisplayComponent::Psvr2DisplayComponent( const Psvr2DisplayConfig &config )
	: config_( config )
{
}

bool Psvr2DisplayComponent::IsDisplayOnDesktop()
{
	return !config_.direct_mode;
}

bool Psvr2DisplayComponent::IsDisplayRealDisplay()
{
	return true;
}

void Psvr2DisplayComponent::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnWidth = config_.render_width;
	*pnHeight = config_.render_height;
}

void Psvr2DisplayComponent::GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnY = 0;
	*pnWidth = config_.window_width / 2;
	*pnHeight = config_.window_height;
	*pnX = ( eEye == vr::Eye_Left ) ? 0 : config_.window_width / 2;
}

void Psvr2DisplayComponent::GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
{
	if ( eEye == vr::Eye_Left )
	{
		*pfLeft = config_.left_eye_left_tan;
		*pfRight = config_.left_eye_right_tan;
	}
	else
	{
		*pfLeft = config_.right_eye_left_tan;
		*pfRight = config_.right_eye_right_tan;
	}
	*pfTop = config_.top_tan;
	*pfBottom = config_.bottom_tan;
}

vr::DistortionCoordinates_t Psvr2DisplayComponent::ComputeDistortion( vr::EVREye eEye, float fU, float fV )
{
	const float center_x = eEye == vr::Eye_Left ? config_.left_lens_center_x : config_.right_lens_center_x;
	const float center_y = config_.lens_center_y;

	vr::DistortionCoordinates_t c{};
	DistortChannel( fU, fV, center_x, center_y,
	                config_.distortion_k1, config_.distortion_k2, config_.distortion_k3,
	                config_.chroma_red_scale, c.rfRed );
	DistortChannel( fU, fV, center_x, center_y,
	                config_.distortion_k1, config_.distortion_k2, config_.distortion_k3,
	                1.0f, c.rfGreen );
	DistortChannel( fU, fV, center_x, center_y,
	                config_.distortion_k1, config_.distortion_k2, config_.distortion_k3,
	                config_.chroma_blue_scale, c.rfBlue );
	return c;
}

void Psvr2DisplayComponent::GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnX = config_.window_x;
	*pnY = config_.window_y;
	*pnWidth = config_.window_width;
	*pnHeight = config_.window_height;
}

bool Psvr2DisplayComponent::ComputeInverseDistortion( vr::HmdVector2_t *, vr::EVREye, uint32_t, float, float )
{
	return false;
}
