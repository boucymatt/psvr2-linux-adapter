// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "openvr_driver.h"
#include "pose_source.h"

// Geometry of the PSVR2 panel as the compositor needs to see it. Defaults are
// the native panel (4000x2040, two 2000x2040 eyes); values can be overridden
// from resources/settings/default.vrsettings while the optical calibration is
// being refined on real hardware.
struct Psvr2DisplayConfig
{
	int32_t window_x = 0;
	int32_t window_y = 0;
	int32_t window_width = 4000;
	int32_t window_height = 2040;
	int32_t render_width = 2000;
	int32_t render_height = 2040;

	// Per-eye asymmetric projection tangents. OpenVR expects left/top to be
	// negative and right/bottom to be positive. PSVR2 lenses are offset from the
	// centre of each half-panel, so a symmetric projection causes poor stereo
	// fusion and the "two copies of the room" effect.
	float left_eye_left_tan = -1.42f;
	float left_eye_right_tan = 1.10f;
	float right_eye_left_tan = -1.10f;
	float right_eye_right_tan = 1.42f;
	float top_tan = -1.24f;
	float bottom_tan = 1.24f;

	// Configurable radial lens correction. ComputeDistortion maps an output
	// location back into the rendered eye texture. These defaults are a mild,
	// usable starting profile rather than an identity transform; they remain
	// user-adjustable until a measured PSVR2 mesh is available.
	float distortion_k1 = 0.18f;
	float distortion_k2 = 0.05f;
	float distortion_k3 = 0.00f;
	float chroma_red_scale = 1.002f;
	float chroma_blue_scale = 0.998f;

	// Optical centre in normalized coordinates within each eye viewport.
	float left_lens_center_x = 0.53f;
	float right_lens_center_x = 0.47f;
	float lens_center_y = 0.50f;

	bool direct_mode = true;
};

class Psvr2DisplayComponent : public vr::IVRDisplayComponent
{
public:
	explicit Psvr2DisplayComponent( const Psvr2DisplayConfig &config );

	bool IsDisplayOnDesktop() override;
	bool IsDisplayRealDisplay() override;
	void GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight ) override;
	void GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight ) override;
	void GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom ) override;
	vr::DistortionCoordinates_t ComputeDistortion( vr::EVREye eEye, float fU, float fV ) override;
	void GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight ) override;
	bool ComputeInverseDistortion( vr::HmdVector2_t *pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV ) override;

private:
	Psvr2DisplayConfig config_;
};

class Psvr2HmdDriver : public vr::ITrackedDeviceServerDriver
{
public:
	Psvr2HmdDriver();

	vr::EVRInitError Activate( uint32_t unObjectId ) override;
	void Deactivate() override;
	void EnterStandby() override;
	void *GetComponent( const char *pchComponentNameAndVersion ) override;
	void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize ) override;
	vr::DriverPose_t GetPose() override;

	const std::string &GetSerialNumber() const { return serial_number_; }

private:
	void PoseThread();

	std::unique_ptr<Psvr2DisplayComponent> display_;
	std::unique_ptr<PoseSource> pose_source_;

	std::string model_number_;
	std::string serial_number_;
	float display_frequency_ = 90.0f;
	float user_ipd_meters_ = 0.064f;
	bool direct_mode_ = true;

	std::atomic<bool> active_{ false };
	std::atomic<uint32_t> device_index_{ vr::k_unTrackedDeviceIndexInvalid };
	vr::DriverPose_t last_pose_{};
	std::thread pose_thread_;
};
