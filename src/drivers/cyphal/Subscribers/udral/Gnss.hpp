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
 * @file Gnss.hpp
 *
 * Defines basic functionality of Cyphal GNSS subscription
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

// UDRAL Specification Messages
#include <reg/udral/physics/kinematics/geodetic/PointStateVarTs_0_1.h>
#include <uavcan/primitive/scalar/Integer16_1_0.h>
#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <uORB/topics/sensor_gps.h>

#include "../DynamicPortSubscriber.hpp"
#include "ds015/service/gnss/Gnss_0_1.h"


class UavcanGnssSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanGnssSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "gps", instance)
	{
		_subj_sub.next = &_cov_sub;

		_cov_sub._subject_name = "gps.cov";
		_cov_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
		_cov_sub._canard_sub.user_reference = this;
		_cov_sub.next = nullptr;

		// eph and epv should be -1 when unknown
		_report.eph = -1.0F;
		_report.epv = -1.0F;

		_report.s_variance_m_s = -1.0F;
		_report.c_variance_rad = -1.0F;

		_report.heading = NAN;
		_report.heading_offset = NAN;
		_report.heading_accuracy = NAN;
	};

	void subscribe() override
	{
		if (_subj_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _subj_sub._canard_sub.port_id,
						   ds015_service_gnss_Gnss_0_1_EXTENT_BYTES_,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_subj_sub._canard_sub);
		}

		if (_cov_sub._canard_sub.port_id != CANARD_PORT_ID_UNSET) {
			_canard_handle.RxSubscribe(CanardTransferKindMessage,
						   _cov_sub._canard_sub.port_id,
						   24,
						   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						   &_cov_sub._canard_sub);
		}
	};

	void callback(const CanardRxTransfer &receive) override
	{
		if (receive.metadata.port_id == _subj_sub._canard_sub.port_id) {
			parsePoint(receive);
			publishUorb();

		} else if (receive.metadata.port_id == _cov_sub._canard_sub.port_id) {
			parseCovariance(receive);

		}
	};

	void parsePoint(const CanardRxTransfer &receive)
	{
		ds015_service_gnss_Gnss_0_1 msg {};
		auto payload = (const uint8_t *)receive.payload;
		size_t msg_size_in_bytes = receive.payload_size;

		if (0 != ds015_service_gnss_Gnss_0_1_deserialize_(&msg, payload, &msg_size_in_bytes)) {
			return;
		}

		_report.timestamp = hrt_absolute_time();
		_report.latitude_deg = M_RAD_TO_DEG * msg.point.latitude;
		_report.longitude_deg = M_RAD_TO_DEG * msg.point.longitude;
		_report.altitude_msl_m = msg.point.altitude.meter;
		_report.altitude_ellipsoid_m = _report.altitude_msl_m;

		_report.vel_n_m_s = msg.velocity.meter_per_second[0];
		_report.vel_e_m_s = msg.velocity.meter_per_second[1];
		_report.vel_d_m_s = msg.velocity.meter_per_second[2];
		_report.vel_m_s = sqrtf(_report.vel_n_m_s * _report.vel_n_m_s +
					_report.vel_e_m_s * _report.vel_e_m_s +
					_report.vel_d_m_s * _report.vel_d_m_s);
		_report.cog_rad = atan2f(_report.vel_e_m_s, _report.vel_n_m_s);
		_report.vel_ned_valid = true;

		_report.satellites_used = msg.num_sats;
		_report.fix_type = msg.status.status;
		_report.hdop = msg.hdop;
		_report.vdop = msg.vdop;

		_report.eph = msg.horizontal_accuracy;
		_report.epv = msg.vertical_accuracy;
		_report.s_variance_m_s = msg.speed_accuracy;
		_report.c_variance_rad = msg.yaw_accuracy.radian;
	}

	void parseCovariance(const CanardRxTransfer &receive)
	{
		if (receive.payload_size != 24) {
			return;
		}

		const uint8_t* buffer = (const uint8_t *)receive.payload;

		float pos_cov_nn = nunavutGetF16(buffer, 24, 16 * 0);
		float pos_cov_ee = nunavutGetF16(buffer, 24, 16 * 3);
		float pos_cov_dd = nunavutGetF16(buffer, 24, 16 * 5);
		const float horizontal_pos_variance = math::max(pos_cov_nn, pos_cov_ee);
		_report.eph = (horizontal_pos_variance > 0) ? sqrtf(horizontal_pos_variance) : -1.0F;
		_report.epv = (pos_cov_dd > 0) ? sqrtf(pos_cov_dd) : -1.0F;

		float vel_cov_nn = nunavutGetF16(buffer, 24, 16 * (6 + 0));
		float vel_cov_ne = nunavutGetF16(buffer, 24, 16 * (6 + 1));
		float vel_cov_ee = nunavutGetF16(buffer, 24, 16 * (6 + 3));
		float vel_cov_dd = nunavutGetF16(buffer, 24, 16 * (6 + 5));

		float vel_n = _report.vel_n_m_s;
		float vel_e = _report.vel_e_m_s;
		float vel_n_sq = vel_n * vel_n;
		float vel_e_sq = vel_e * vel_e;

		_report.s_variance_m_s = math::max(vel_cov_nn, vel_cov_ee, vel_cov_dd);
		if (vel_n_sq < 0.001f && vel_e_sq < 0.001f) {
			_report.c_variance_rad = -1.0f;
		} else {
			_report.c_variance_rad =
				(vel_e_sq * vel_cov_nn +
				-2 * vel_n * vel_e * vel_cov_ne +
				vel_n_sq * vel_cov_ee) / ((vel_n_sq + vel_e_sq) * (vel_n_sq + vel_e_sq));
		}
	}

	void publishUorb()
	{
		if (_orb_advert == nullptr) {
			_orb_advert = orb_advertise_multi(ORB_TOPIC, &_report, &_instance);

		} else {
			(void)orb_publish(ORB_TOPIC, _orb_advert, &_report);
		}
	}

private:
	int16_t parseInteger16(const CanardRxTransfer &receive)
	{
		uavcan_primitive_scalar_Integer16_1_0 msg;
		size_t size_in_bytes = receive.payload_size;

		if (0 != uavcan_primitive_scalar_Integer16_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&size_in_bytes)) {
			return 0;
		}

		return msg.value;
	}

	float parseReal32(const CanardRxTransfer &receive)
	{
		uavcan_primitive_scalar_Real32_1_0 msg;
		size_t size_in_bytes = receive.payload_size;

		if (0 != uavcan_primitive_scalar_Real32_1_0_deserialize_(&msg,
				(const uint8_t *)receive.payload,
				&size_in_bytes)) {
			return 0;
		}

		return msg.value;
	}

	const orb_id_t ORB_TOPIC = ORB_ID(sensor_gps);
	orb_advert_t _orb_advert{nullptr};

	int _instance = 0;
	sensor_gps_s _report{};

	SubjectSubscription _cov_sub;
};
