/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ParamManager.hpp
 *
 * Defines basic functionality of UAVCAN parameter management class
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>

#include <uavcan/_register/Name_1_0.h>
#include <uavcan/_register/Value_1_0.h>

static constexpr uint16_t CANARD_PORT_ID_UNSET = 65535U;
static constexpr uint16_t CANARD_PORT_ID_MAX   = 32767U;

static bool px4_param_to_uavcan_port_id(param_t &in, uavcan_register_Value_1_0 &out)
{
	if (param_type(in) == PARAM_TYPE_INT32) {
		int32_t out_val {};
		const int res = param_get(in, &out_val);

		if (res != PX4_OK) {
			// Parameter not found / internal error
			return false;
		}

		if (out_val >= 0 && out_val <= CANARD_PORT_ID_MAX) {
			out.natural16.value.elements[0] = (uint16_t)out_val;

		} else {
			// "Invalid" value -- set to "UNSET"
			out.natural16.value.elements[0] = CANARD_PORT_ID_UNSET;
		}

		out.natural16.value.count = 1;
		uavcan_register_Value_1_0_select_natural16_(&out);
		return true;
	}

	return false;
};

static bool uavcan_port_id_to_px4_param(const uavcan_register_Value_1_0 &in, param_t &out)
{
	if (uavcan_register_Value_1_0_is_natural16_(&in) && in.natural16.value.count == 1) {
		if (param_type(out) == PARAM_TYPE_INT32) {
			int32_t in_val = in.natural16.value.elements[0];
			param_set(out, &in_val);
			return true;
		}
	}

	return false;
};

using param_2_reg_t = bool(*)(param_t &in, uavcan_register_Value_1_0 &out);
using reg_2_param_t = bool(*)(const uavcan_register_Value_1_0 &in, param_t &out);


typedef struct {
	const char *uavcan_name;
	const char *px4_name;
	param_2_reg_t px4_param_to_register_value;
	reg_2_param_t register_value_to_px4_param;
	bool is_mutable {true};
	bool is_persistent {true};
} UavcanParamBinder;

typedef struct {
	const char *name;
	const char *value;
} CyphalTypeRegister;

class UavcanParamManager
{
public:

	bool GetParamByName(const char *param_name, uavcan_register_Value_1_0 &value);
	bool GetParamByName(const uavcan_register_Name_1_0 &name, uavcan_register_Value_1_0 &value);
	bool GetParamName(uint32_t id, uavcan_register_Name_1_0 &name);
	bool SetParamByName(const uavcan_register_Name_1_0 &name, const uavcan_register_Value_1_0 &value);

private:


	const UavcanParamBinder _uavcan_params[36] {
		{"uavcan.pub.udral.esc.0.id",                	"UCAN1_ESC_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.udral.readiness.0.id",            	"UCAN1_READ_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.udral.servo.0.id",              	"UCAN1_SERVO_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.udral.rgbled.0.id",              	"UCAN1_RGBLED_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.udral.gps.0.id",                	"UCAN1_GPS_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.udral.actuator_outputs.0.id",   	"UCAN1_ACTR_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.accel.0.id",                	"UCAN1_ACCEL_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.aspd.dpres.0.id",           	"UCAN1_DPRES0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.aspd.dpres.1.id",           	"UCAN1_DPRES1_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.aspd.tas.0.id",		"UCAN1_ATAS0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.aspd.cas.0.id",		"UCAN1_ACAS0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.aspd.temperature.0.id",	"UCAN1_ATEM0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.baro.pressure.0.id",		"UCAN1_BAROP0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.baro.temperature.0.id",	"UCAN1_BAROT0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.esc.0.id",                	"UCAN1_ESC0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.gps.0.id",                	"UCAN1_GPS0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.gps.cov.0.id",		"UCAN1_GPSCO0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.gps.1.id",                	"UCAN1_GPS1_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.gyro.0.id",                	"UCAN1_GYRO_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.imu.0.id",                	"UCAN1_IMU_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.mag.0.id",               	"UCAN1_MAG0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.range.0.id",               	"UCAN1_RANGE0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.energy_source.0.id",      	"UCAN1_BMS_ES_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.battery_status.0.id",     	"UCAN1_BMS_BS_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.battery_parameters.0.id",	"UCAN1_BMS_BP_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.udral.legacy_bms.0.id",         	"UCAN1_LG_BMS_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.uorb.sensor_gps.0.id",    		"UCAN1_UORB_GPS",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.uorb.sensor_gps.0.id",    		"UCAN1_UORB_GPS_P",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.0.id",            	"UCAN1_FB0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.1.id",            	"UCAN1_FB1_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.2.id",            	"UCAN1_FB2_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.3.id",            	"UCAN1_FB3_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.4.id",            	"UCAN1_FB4_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.5.id",            	"UCAN1_FB5_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.6.id",            	"UCAN1_FB6_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.zubax.feedback.7.id",            	"UCAN1_FB7_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		//{"uavcan.sub.bms.0.id",   "UCAN1_BMS0_SUB"}, //FIXME instancing
		//{"uavcan.sub.bms.1.id",   "UCAN1_BMS1_SUB"},
	};

	CyphalTypeRegister _type_registers[25] {
		{"uavcan.pub.udral.esc.0.type", 		"reg.udral.service.actuator.common.sp.Vector31"},
		{"uavcan.pub.udral.readiness.0.type", 		"reg.udral.service.common.Readiness.0.1"},
		{"uavcan.pub.udral.rgbled.0.type", 		"reg.udral.physics.optics.HighColor.0.1"},
		{"uavcan.sub.zubax.feedback.0.type", 		"zubax.telega.CompactFeedback.0.1"},
		{"uavcan.sub.zubax.feedback.1.type", 		"zubax.telega.CompactFeedback.0.1"},
		{"uavcan.sub.zubax.feedback.2.type", 		"zubax.telega.CompactFeedback.0.1"},
		{"uavcan.sub.zubax.feedback.3.type", 		"zubax.telega.CompactFeedback.0.1"},
		{"uavcan.sub.zubax.feedback.4.type", 		"zubax.telega.CompactFeedback.0.1"},
		{"uavcan.sub.zubax.feedback.5.type", 		"zubax.telega.CompactFeedback.0.1"},
		{"uavcan.sub.zubax.feedback.6.type", 		"zubax.telega.CompactFeedback.0.1"},
		{"uavcan.sub.zubax.feedback.7.type", 		"zubax.telega.CompactFeedback.0.1"},

		{"uavcan.sub.udral.gps.0.type", 		"reg.udral.physics.kinematics.geodetic.PointStateVarTs.0.1"},
		{"uavcan.sub.udral.gps.cov.0.type", 		"ds015.service.gnss.Covariance.0.1"},

		{"uavcan.sub.udral.baro.pressure.0.type", 	"uavcan.si.sample.pressure.Scalar.1.0"},
		{"uavcan.sub.udral.baro.temperature.0.type", 	"uavcan.si.sample.temperature.Scalar.1.0"},

		{"uavcan.sub.udral.mag.0.type", 		"uavcan.si.sample.magnetic_field_strength.Vector3.1.1"},

		{"uavcan.sub.udral.range.0.type", 		"uavcan.si.sample.length.Scalar.1.0"},
		{"uavcan.sub.udral.accel.0.type", 		"uavcan.si.sample.acceleration.Vector3.1.0"},

		{"uavcan.sub.udral.aspd.dpres.0.type", 		"uavcan.si.sample.pressure.Scalar.1.0"},
		{"uavcan.sub.udral.aspd.dpres.1.type", 		"uavcan.si.sample.pressure.Scalar.1.0"},
		{"uavcan.sub.udral.aspd.cas.0.type", 		"uavcan.si.sample.velocity.Scalar.1.0"},
		{"uavcan.sub.udral.aspd.tas.0.type", 		"uavcan.si.sample.velocity.Scalar.1.0"},
		{"uavcan.sub.udral.aspd.temperature.0.type", 	"uavcan.si.sample.temperature.Scalar.1.0"},

		{"uavcan.sub.udral.gyro.0.type", 		"uavcan.si.sample.angular_velocity.Vector3.1.0"},
		{"uavcan.sub.udral.imu.0.type", 		"uavcan.primitive.array.Real16.1.0"},
	};
};
