
#include <algorithm>
#include <climits>
#include <iomanip>
#include <ctime>

#include "messages.hpp"
#include "argos_scheduler.hpp"
#include "rtc.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"
#include "bitpack.hpp"
#include "timeutils.hpp"
#include "bch.hpp"
#include "crc8.hpp"
#include "binascii.hpp"

extern "C" {
	#include "previpass.h"
}

#define INVALID_SCHEDULE    (std::time_t)-1

#define FIXTYPE_3D			3

#define HOURS_PER_DAY       24
#define SECONDS_PER_MINUTE	60
#define SECONDS_PER_HOUR    3600
#define SECONDS_PER_DAY     (SECONDS_PER_HOUR * HOURS_PER_DAY)
#define MM_PER_METER		1000
#define MM_PER_KM   		1000000
#define MV_PER_UNIT			20
#define MS_PER_SEC			1000
#define METRES_PER_UNIT     40
#define DEGREES_PER_UNIT	(1.0f/1.42f)
#define BITS_PER_BYTE		8
#define MIN_ALTITUDE		0
#define MAX_ALTITUDE		254
#define INVALID_ALTITUDE	255

#define LON_LAT_RESOLUTION  10000

#define MAX_GPS_ENTRIES_IN_PACKET	4

#define SHORT_PACKET_HEADER_BYTES   7
#define SHORT_PACKET_PAYLOAD_BITS   99
#define SHORT_PACKET_BYTES			22
#define SHORT_PACKET_MSG_LENGTH		6
#define SHORT_PACKET_BITFIELD       0x11

#define LONG_PACKET_HEADER_BYTES    7
#define LONG_PACKET_PAYLOAD_BITS    216
#define LONG_PACKET_BYTES			38
#define LONG_PACKET_MSG_LENGTH		15
#define LONG_PACKET_BITFIELD        0x8B

#define PACKET_SYNC					0xFFFE2F

#define ARGOS_TX_MARGIN_SECS        12

extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC       *rtc;
extern Logger    *sensor_log;


ArgosScheduler::ArgosScheduler() {
	m_is_running = false;
	m_switch_state = false;
	m_earliest_tx = INVALID_SCHEDULE;
	m_last_longitude = INVALID_GEODESIC;
	m_last_latitude = INVALID_GEODESIC;
}

void ArgosScheduler::reschedule() {
	std::time_t schedule = INVALID_SCHEDULE;

	// Obtain fresh copy of configuration as it may have changed
	configuration_store->get_argos_configuration(m_argos_config);

	if (m_argos_config.mode == BaseArgosMode::OFF) {
		DEBUG_WARN("ArgosScheduler: mode is OFF -- not scheduling");
		return;
	} else if (!rtc->is_set()) {
		DEBUG_WARN("ArgosScheduler: RTC is not yet set -- not scheduling");
		return;
	} else if (m_argos_config.time_sync_burst_en && !m_time_sync_burst_sent && m_num_gps_entries) {
		// Schedule an immediate time sync burst
		DEBUG_TRACE("ArgosScheduler: scheduling immediate time sync burst");
		schedule = 0;
	} else if (m_argos_config.mode == BaseArgosMode::LEGACY) {
		// In legacy mode we schedule every hour aligned to UTC
		schedule = next_duty_cycle(0xFFFFFFU);
	} else if (m_argos_config.mode == BaseArgosMode::DUTY_CYCLE) {
		schedule = next_duty_cycle(m_argos_config.duty_cycle);
	} else if (m_argos_config.mode == BaseArgosMode::PASS_PREDICTION) {
		schedule = next_prepass();
	}

	if (INVALID_SCHEDULE != schedule) {
		DEBUG_INFO("ArgosScheduler: schedule in: %llu secs", schedule);
		deschedule();
		m_argos_task = system_scheduler->post_task_prio(std::bind(&ArgosScheduler::process_schedule, this),
				"ArgosSchedulerProcessSchedule",
				Scheduler::DEFAULT_PRIORITY, MS_PER_SEC * schedule);
	} else {
		DEBUG_INFO("ArgosScheduler: not rescheduling");
	}
}

static inline bool is_in_duty_cycle(std::time_t time, unsigned int duty_cycle)
{
	unsigned int seconds_of_day = (time % SECONDS_PER_DAY);
	unsigned int hour_of_day = seconds_of_day / SECONDS_PER_HOUR;
	return (duty_cycle & (0x800000 >> hour_of_day));
}

std::time_t ArgosScheduler::next_duty_cycle(unsigned int duty_cycle)
{
	// Do not proceed unless duty cycle is non-zero to save time
	if (duty_cycle == 0)
	{
		DEBUG_INFO("ArgosScheduler::next_duty_cycle: no schedule found as duty cycle is zero!");
		m_tr_nom_schedule = INVALID_SCHEDULE;
		return INVALID_SCHEDULE;
	}

	// Find epoch time for start of the "current" day
	std::time_t now = rtc->gettime();
	unsigned int start_of_day = now - (now % SECONDS_PER_DAY);

	// If we have already a future schedule then don't recompute
	if (m_tr_nom_schedule != INVALID_SCHEDULE && m_tr_nom_schedule >= now &&
		m_tr_nom_schedule > m_last_transmission_schedule)
	{
		if (m_is_deferred) {
			DEBUG_INFO("ArgosScheduler::next_duty_cycle: existing schedule deferred: in %llu seconds", m_tr_nom_schedule - now);
			m_is_deferred = false;
			return m_tr_nom_schedule - now;
		}

		DEBUG_INFO("ArgosScheduler::next_duty_cycle: use existing schedule: in %llu seconds", m_tr_nom_schedule - now);
		return INVALID_SCHEDULE;
	}

	// If earliest TX is inside the duty cycle window, then use that since we were deferred by saltwater switch
	if (m_is_deferred && m_earliest_tx != INVALID_SCHEDULE && m_earliest_tx > now && is_in_duty_cycle(m_earliest_tx, duty_cycle))
	{
		DEBUG_INFO("ArgosScheduler::next_duty_cycle: using earliest TX: %llu", m_earliest_tx);
		m_is_deferred = false;
		return m_earliest_tx - now;
	}

	// Set schedule to be earliest possible next TR_NOM.  If there was no last schedule or
	// the last schedule was over 24 hours ago, then set our starting point to current time.
	// Otherwise increment the last schedule by TR_NOM for our starting point.
	if (m_tr_nom_schedule == INVALID_SCHEDULE || (now - m_tr_nom_schedule) > HOURS_PER_DAY)
		m_tr_nom_schedule = start_of_day;
	else
		m_tr_nom_schedule += m_argos_config.tr_nom;

	// Compute the seconds of day and hours of day for candidate m_tr_nom_schedule
	unsigned int seconds_of_day = (m_tr_nom_schedule % SECONDS_PER_DAY);
	unsigned int hour_of_day = (seconds_of_day / SECONDS_PER_HOUR);
	unsigned int terminal_hours = hour_of_day + (2*HOURS_PER_DAY);

	// Note that duty cycle is a bit-field comprising 24 bits as follows:
	// 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00  bit
	// 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 21 21 22 23  hour (UTC)
	// The range of TR_NOM is 45 seconds to 20 hours
	//
	// We iterate forwards from the candidate m_tr_nom_schedule until we find a TR_NOM that
	// falls inside a permitted hour of transmission.  The maximum span we search is 24 hours.
	while (hour_of_day < terminal_hours) {
		//DEBUG_TRACE("ArgosScheduler::next_duty_cycle: now: %lu candidate schedule: %lu hour_of_day: %u", now, m_tr_nom_schedule, hour_of_day);
		if ((duty_cycle & (0x800000 >> (hour_of_day % HOURS_PER_DAY))) && m_tr_nom_schedule >= now) {
			DEBUG_INFO("ArgosScheduler::next_duty_cycle: found schedule: %lu", m_tr_nom_schedule);
			return m_tr_nom_schedule - now;
		} else {
			m_tr_nom_schedule += m_argos_config.tr_nom;
			seconds_of_day += m_argos_config.tr_nom;
			hour_of_day = (seconds_of_day / SECONDS_PER_HOUR);
		}
	}

	DEBUG_WARN("ArgosScheduler::next_duty_cycle: no schedule found!");

	m_tr_nom_schedule = INVALID_SCHEDULE;

	return INVALID_SCHEDULE;
}

std::time_t ArgosScheduler::next_prepass() {

	DEBUG_TRACE("ArgosScheduler::next_prepass");

	// We must have a previous GPS location to proceed
	if (m_last_latitude == INVALID_GEODESIC) {
		DEBUG_WARN("ArgosScheduler::next_prepass: current GPS location is not presently known - aborting");
		return INVALID_SCHEDULE;
	}

	std::time_t curr_time = rtc->gettime();

	// If we have an existing prepass schedule and we were interrupted by saltwater switch, then check
	// to see if the current prepass schedule is still valid to use.  We allow up to ARGOS_TX_MARGIN_SECS
	// before the end of the window
	if (m_next_prepass != INVALID_SCHEDULE &&
		m_is_deferred &&
		m_earliest_tx != INVALID_SCHEDULE &&
		m_earliest_tx >= curr_time &&
		m_earliest_tx < (m_next_prepass + m_prepass_duration - ARGOS_TX_MARGIN_SECS))
	{
		DEBUG_INFO("ArgosScheduler::next_prepass: rescheduling after SWS interruption earliest TX=%llu epoch=%llu", m_earliest_tx, m_next_prepass);
		m_is_deferred = false;
		return std::max(m_earliest_tx, m_next_prepass) - curr_time;
	}

	if (m_next_prepass != INVALID_SCHEDULE &&
		curr_time < (m_next_prepass + m_prepass_duration - ARGOS_TX_MARGIN_SECS)) {
		DEBUG_WARN("ArgosScheduler::next_prepass: existing prepass schedule for epoch=%llu secs from now is still valid", m_next_prepass - curr_time);
		return INVALID_SCHEDULE;
	}

	// Ensure start window is advanced from the previous transmission by the TR_NOM (repetition period)
	std::time_t start_time;
	if (m_last_transmission_schedule != INVALID_SCHEDULE)
		start_time = std::max(curr_time, (m_last_transmission_schedule + m_argos_config.tr_nom - ARGOS_TX_MARGIN_SECS));
	else
		start_time = curr_time;

	std::time_t stop_time = start_time + (std::time_t)(24 * SECONDS_PER_HOUR);
	struct tm *p_tm = std::gmtime(&start_time);
	struct tm tm_start = *p_tm;
	p_tm = std::gmtime(&stop_time);
	struct tm tm_stop = *p_tm;

	DEBUG_INFO("ArgosScheduler::next_prepass: now=%llu start=%llu stop=%llu", curr_time, start_time, stop_time);

	BasePassPredict& pass_predict = configuration_store->read_pass_predict();
	PredictionPassConfiguration_t config = {
		(float)m_last_latitude,
		(float)m_last_longitude,
		{ (uint16_t)(1900 + tm_start.tm_year), (uint8_t)(tm_start.tm_mon + 1), (uint8_t)tm_start.tm_mday, (uint8_t)tm_start.tm_hour, (uint8_t)tm_start.tm_min, (uint8_t)tm_start.tm_sec },
		{ (uint16_t)(1900 + tm_stop.tm_year), (uint8_t)(tm_stop.tm_mon + 1), (uint8_t)tm_stop.tm_mday, (uint8_t)tm_stop.tm_hour, (uint8_t)tm_stop.tm_min, (uint8_t)tm_stop.tm_sec },
        (float)m_argos_config.prepass_min_elevation,        //< Minimum elevation of passes [0, 90]
		(float)m_argos_config.prepass_max_elevation,        //< Maximum elevation of passes  [maxElevation >= < minElevation]
		(float)m_argos_config.prepass_min_duration / 60.0f,  //< Minimum duration (minutes)
		m_argos_config.prepass_max_passes,                  //< Maximum number of passes per satellite (#)
		(float)m_argos_config.prepass_linear_margin / 60.0f, //< Linear time margin (in minutes/6months)
		m_argos_config.prepass_comp_step                    //< Computation step (seconds)
	};
	SatelliteNextPassPrediction_t next_pass;

	while (PREVIPASS_compute_next_pass(
    		&config,
			pass_predict.records,
			pass_predict.num_records,
			&next_pass)) {

		// Computed schedule is advanced by ARGOS_TX_MARGIN_SECS so that Artic R2 programming delay is not included in the window
		std::time_t now = rtc->gettime();
		std::time_t schedule = (std::time_t)(next_pass.epoch - ARGOS_TX_MARGIN_SECS) < now ? now : ((std::time_t)next_pass.epoch - ARGOS_TX_MARGIN_SECS);
		// Ensure the schedule is at least TR_NOM away from previous transmission
		if (m_last_transmission_schedule != INVALID_SCHEDULE)
			schedule = std::max(schedule, (m_last_transmission_schedule + m_argos_config.tr_nom - ARGOS_TX_MARGIN_SECS));

		DEBUG_INFO("ArgosScheduler::next_prepass: hex_id=%01x last=%llu now=%llu s=%u c=%llu e=%u",
					next_pass.satHexId, (m_last_transmission_schedule == INVALID_SCHEDULE) ? 0 : m_last_transmission_schedule, curr_time, (unsigned int)next_pass.epoch, schedule, (unsigned int)next_pass.epoch + next_pass.duration);

		// Check we fit inside the prepass window
		if ((schedule + ARGOS_TX_MARGIN_SECS) < ((std::time_t)next_pass.epoch + next_pass.duration)) {
			// Compute delay until epoch arrives for scheduling and select Argos transmission mode
			DEBUG_INFO("ArgosScheduler::next_prepass: scheduled for %llu seconds from now", schedule - now);
			m_next_prepass = schedule;
			m_next_mode = next_pass.uplinkStatus >= SAT_UPLK_ON_WITH_A3 ? ArgosMode::ARGOS_3 : ArgosMode::ARGOS_2;
			m_prepass_duration = next_pass.duration;
			return schedule - now;
		} else {
			DEBUG_TRACE("ArgosScheduler::next_prepass: computed prepass window is too late", next_pass.epoch, next_pass.duration);
			start_time = (std::time_t)next_pass.epoch + next_pass.duration;
			p_tm = std::gmtime(&start_time);
			tm_start = *p_tm;
			config.start = { (uint16_t)(1900 + tm_start.tm_year), (uint8_t)(tm_start.tm_mon + 1), (uint8_t)tm_start.tm_mday, (uint8_t)tm_start.tm_hour, (uint8_t)tm_start.tm_min, (uint8_t)tm_start.tm_sec };
			DEBUG_INFO("ArgosScheduler::next_prepass: now=%llu start=%llu stop=%llu", curr_time, start_time, stop_time);
		}
	}

	// No passes reported
	DEBUG_ERROR("ArgosScheduler::next_prepass: PREVIPASS_compute_next_pass returned no passes");
	return INVALID_SCHEDULE;
}

void ArgosScheduler::process_schedule() {

	std::time_t now = rtc->gettime();
	if (!m_switch_state && (m_earliest_tx == INVALID_SCHEDULE || m_earliest_tx <= now)) {
		if (m_argos_config.time_sync_burst_en && !m_time_sync_burst_sent && m_num_gps_entries && rtc->is_set()) {
			time_sync_burst_algorithm();
		} else if (m_argos_config.mode == BaseArgosMode::PASS_PREDICTION) {
			pass_prediction_algorithm();
		} else {
			periodic_algorithm();
		}
	} else {
		DEBUG_INFO("ArgosScheduler::process_schedule: sws=%u t=%llu earliest_tx=%llu deferring transmission", m_switch_state, now, m_earliest_tx);
		m_is_deferred = true;
	}

	// After each transmission attempt to reschedule
	reschedule();
}

void ArgosScheduler::pass_prediction_algorithm() {
	unsigned int max_index = (((unsigned int)m_argos_config.depth_pile + MAX_GPS_ENTRIES_IN_PACKET-1) / MAX_GPS_ENTRIES_IN_PACKET);
	unsigned int index = 0;
	unsigned int max_msg_index;
	ArgosPacket packet;

	// Get latest configuration
	configuration_store->get_argos_configuration(m_argos_config);

	DEBUG_INFO("ArgosScheduler::pass_prediction_algorithm: m_msg_index=%u max_index=%u", m_msg_index, max_index);

	// Find first eligible slot for transmission
	max_msg_index = m_msg_index + max_index;
	while (m_msg_index < max_msg_index) {
		index = m_msg_index % max_index;
		//DEBUG_TRACE("ArgosScheduler::pass_prediction_algorithm: m_msg_index=%u index=%u gps_entries=%u", m_msg_index, index, m_gps_entries[index].size());
		if (m_gps_entries[index].size())
		{
			DEBUG_TRACE("ArgosScheduler::pass_prediction_algorithm: eligible slot=%u", index);
			break;
		}
		m_msg_index++;
	}

	// No further action if no slot is eligible for transmission
	if (m_msg_index == max_msg_index) {
		DEBUG_WARN("ArgosScheduler::pass_prediction_algorithm: no eligible slot found");
		m_last_transmission_schedule = m_next_prepass;
		m_next_prepass = INVALID_SCHEDULE;
		return;
	}

	if (m_gps_entries[index].size() == 1) {
		build_short_packet(m_gps_entries[index][0], packet);
		handle_packet(packet, SHORT_PACKET_BYTES * BITS_PER_BYTE, m_next_mode);
	} else {
		build_long_packet(m_gps_entries[index], packet);
		handle_packet(packet, LONG_PACKET_BYTES * BITS_PER_BYTE, m_next_mode);
	}

	m_msg_burst_counter[index]--;
	m_msg_index++;
	m_last_transmission_schedule = rtc->gettime();
	m_next_prepass = INVALID_SCHEDULE;

	// Check if message burst count has reached zero and perform clean-up of this slot
	if (m_msg_burst_counter[index] == 0) {
		m_msg_burst_counter[index] = (m_argos_config.ntry_per_message == 0) ? UINT_MAX : m_argos_config.ntry_per_message;
		m_gps_entries[index].clear();
	}
}

void ArgosScheduler::notify_sensor_log_update() {
	DEBUG_TRACE("ArgosScheduler::notify_sensor_log_update");

	if (m_is_running) {

		GPSLogEntry gps_entry;

		// Read the most recent GPS entry out of the sensor log
		unsigned int idx = sensor_log->num_entries() - 1;  // Most recent entry in log
		sensor_log->read(&gps_entry, idx);
		// Update last known position if the GPS entry is valid (otherwise we preserve the last one)
		if (gps_entry.info.valid) {
			DEBUG_TRACE("ArgosScheduler::notify_sensor_log_update: updated last known GPS position; is_rtc_set=%u",
					rtc->is_set());
			m_last_longitude = gps_entry.info.lon;
			m_last_latitude = gps_entry.info.lat;
		}

		if (m_argos_config.mode == BaseArgosMode::PASS_PREDICTION) {
			unsigned int msg_index;
			unsigned int max_index = (((unsigned int)m_argos_config.depth_pile + MAX_GPS_ENTRIES_IN_PACKET-1) / MAX_GPS_ENTRIES_IN_PACKET);
			unsigned int span = std::min((unsigned int)MAX_GPS_ENTRIES_IN_PACKET, (unsigned int)m_argos_config.depth_pile);

			DEBUG_TRACE("ArgosScheduler::notify_sensor_log_update: PASS_PREDICTION mode: max_index=%u span=%u", max_index, span);

			// Find the first slot whose vector has less then depth pile entries in it
			for (msg_index = 0; msg_index < max_index; msg_index++) {
				if (m_gps_entries[msg_index].size() < span)
					break;
			}

			// If a slot is found then append most recent sensor log entry to it
			if (msg_index < max_index) {
				m_gps_entries[msg_index].push_back(gps_entry);
				DEBUG_TRACE("ArgosScheduler::notify_sensor_log_update: PASS_PREDICTION mode: appending entry=%u to msg_slot=%u", idx, msg_index);
			} else {
				DEBUG_TRACE("ArgosScheduler::notify_sensor_log_update: PASS_PREDICTION mode: no free slots");
			}
		}

		// Keep a running track of number of GPS entries even if we are not in periodic mode
		{
			// Update the GPS map based on the most recent entry
			m_gps_entry_burst_counter[m_num_gps_entries] = (m_argos_config.ntry_per_message == 0) ? UINT_MAX : m_argos_config.ntry_per_message;

			// Update our local count of available GPS entries (also acts as a map key and guaranteed to be unique)
			m_num_gps_entries++;

			// If the number of GPS entries exceeds the depth pile then delete the oldest entry from the GPS map so it
			// doesn't keep growing in size
			if (m_num_gps_entries > (unsigned int)m_argos_config.depth_pile) {
				unsigned int index = m_num_gps_entries - (unsigned int)m_argos_config.depth_pile - 1;
				DEBUG_TRACE("ArgosScheduler::notify_sensor_log_update: erasing entry %u from depth pile", index);
				m_gps_entry_burst_counter.erase(index);
			}

			DEBUG_TRACE("ArgosScheduler::notify_sensor_log_update: m_gps_entry_burst_counter has %u/%u entries with total seen %u", m_gps_entry_burst_counter.size(), (unsigned int)m_argos_config.depth_pile, m_num_gps_entries);
		}
		reschedule();
	}
}

unsigned int ArgosScheduler::convert_latitude(double x) {
	if (x >= 0)
		return x * LON_LAT_RESOLUTION;
	else
		return ((unsigned int)((x - 0.00005) * -LON_LAT_RESOLUTION)) | 1<<20; // -ve: bit 20 is sign
}

unsigned int ArgosScheduler::convert_longitude(double x) {
	if (x >= 0)
		return x * LON_LAT_RESOLUTION;
	else
		return ((unsigned int)((x - 0.00005) * -LON_LAT_RESOLUTION)) | 1<<21; // -ve: bit 21 is sign
}

void ArgosScheduler::build_short_packet(GPSLogEntry const& gps_entry, ArgosPacket& packet) {

	DEBUG_TRACE("ArgosScheduler::build_short_packet");

#ifndef ARGOS_TEST_PACKET
	unsigned int base_pos = 0;

	// Reserve required number of bytes
	packet.assign(SHORT_PACKET_BYTES, 0);

	// Header bytes
	PACK_BITS(PACKET_SYNC, packet, base_pos, 24);
	PACK_BITS(SHORT_PACKET_MSG_LENGTH, packet, base_pos, 4);
	PACK_BITS(m_argos_config.argos_id>>8, packet, base_pos, 20);
	PACK_BITS(m_argos_config.argos_id, packet, base_pos, 8);

	// Payload bytes
#ifdef ARGOS_USE_CRC8
	PACK_BITS(0, packet, base_pos, 8);  // Zero CRC field (computed later)
#else
	PACK_BITS(SHORT_PACKET_BITFIELD, packet, base_pos, 8);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: bitfield=%u", SHORT_PACKET_BITFIELD);
#endif
	PACK_BITS(gps_entry.info.day, packet, base_pos, 5);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: day=%u", gps_entry.info.day);
	PACK_BITS(gps_entry.info.hour, packet, base_pos, 5);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: hour=%u", gps_entry.info.hour);
	PACK_BITS(gps_entry.info.min, packet, base_pos, 6);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: min=%u", gps_entry.info.min);

	if (gps_entry.info.valid) {
		PACK_BITS(convert_latitude(gps_entry.info.lat), packet, base_pos, 21);
		DEBUG_TRACE("ArgosScheduler::build_short_packet: lat=%u (%lf)", convert_latitude(gps_entry.info.lat), gps_entry.info.lat);
		PACK_BITS(convert_longitude(gps_entry.info.lon), packet, base_pos, 22);
		DEBUG_TRACE("ArgosScheduler::build_short_packet: lon=%u (%lf)", convert_longitude(gps_entry.info.lon), gps_entry.info.lon);
		double gspeed = (SECONDS_PER_HOUR * gps_entry.info.gSpeed) / (2*MM_PER_KM);
		PACK_BITS((unsigned int)gspeed, packet, base_pos, 7);
		DEBUG_TRACE("ArgosScheduler::build_short_packet: speed=%u", (unsigned int)gspeed);

		// OUTOFZONE_FLAG
		PACK_BITS(m_argos_config.is_out_of_zone, packet, base_pos, 1);
		DEBUG_TRACE("ArgosScheduler::build_short_packet: is_out_of_zone=%u", m_argos_config.is_out_of_zone);

		PACK_BITS(gps_entry.info.headMot * DEGREES_PER_UNIT, packet, base_pos, 8);
		DEBUG_TRACE("ArgosScheduler::build_short_packet: heading=%u", (unsigned int)(gps_entry.info.headMot * DEGREES_PER_UNIT));
		if (gps_entry.info.fixType == FIXTYPE_3D) {
			int32_t altitude = gps_entry.info.hMSL / (MM_PER_METER * METRES_PER_UNIT);
			if (altitude > MAX_ALTITUDE) {
				DEBUG_WARN("ArgosScheduler::build_short_packet: altitude %d (x 40m) exceeds maximum - truncating", altitude);
				altitude = MAX_ALTITUDE;
			} else if (altitude < MIN_ALTITUDE) {
				DEBUG_WARN("ArgosScheduler::build_short_packet: altitude %d (x 40m) below minimum - truncating", altitude);
				altitude = MIN_ALTITUDE;
			}
			DEBUG_TRACE("ArgosScheduler::build_short_packet: altitude=%d (x 40m)", altitude);
			PACK_BITS(altitude, packet, base_pos, 8);
		} else {
			DEBUG_WARN("ArgosScheduler::build_short_packet: altitude not available without 3D fix");
			PACK_BITS(INVALID_ALTITUDE, packet, base_pos, 8);
		}
	} else {
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		PACK_BITS(0xFF, packet, base_pos, 7);
		PACK_BITS(m_argos_config.is_out_of_zone, packet, base_pos, 1);
		DEBUG_TRACE("ArgosScheduler::build_short_packet: is_out_of_zone=%u", m_argos_config.is_out_of_zone);
		PACK_BITS(0xFF, packet, base_pos, 8);
		PACK_BITS(0xFF, packet, base_pos, 8);
	}

	unsigned int batt = std::min(127, std::max((int)gps_entry.info.batt_voltage - 2700, (int)0) / MV_PER_UNIT);
	PACK_BITS(batt, packet, base_pos, 7);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: voltage=%u (%u)", batt, gps_entry.info.batt_voltage);

	// LOWBATERY_FLAG
	PACK_BITS(m_argos_config.is_lb, packet, base_pos, 1);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: is_lb=%u", m_argos_config.is_lb);

	// Calculate CRC8
#ifdef ARGOS_USE_CRC8
	unsigned char crc8 = CRC8::checksum(packet.substr(SHORT_PACKET_HEADER_BYTES+1), SHORT_PACKET_PAYLOAD_BITS - 8);
	unsigned int crc_offset = 8*SHORT_PACKET_HEADER_BYTES;
	PACK_BITS(crc8, packet, crc_offset, 8);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: crc8=%02x", crc8);
#endif

	// BCH code B127_106_3
	BCHCodeWord code_word = BCHEncoder::encode(
			BCHEncoder::B127_106_3,
			sizeof(BCHEncoder::B127_106_3),
			packet.substr(SHORT_PACKET_HEADER_BYTES), SHORT_PACKET_PAYLOAD_BITS);
	DEBUG_TRACE("ArgosScheduler::build_short_packet: bch=%06x", code_word);

	// Append BCH code
	PACK_BITS(code_word, packet, base_pos, BCHEncoder::B127_106_3_CODE_LEN);
#else

	// Send a nail-up test packet
	packet = std::string("\xFF\xFF\xFF\x64\xE7\xB5\x6A\xC1\x47\xCA\x6B\x48\x17\xC7\x65\xDC\x8A\x2A\x9D\xA1\xE2\x18"s);

#endif
}

void ArgosScheduler::adjust_logtime_for_gps_ontime(GPSLogEntry const& a, uint8_t& day, uint8_t& hour, uint8_t& minute)
{
	uint16_t year;
	uint8_t  month;
	uint8_t  second;
	std::time_t t;
	t = convert_epochtime(a.header.year, a.header.month, a.header.day, a.header.hours, a.header.minutes, a.header.seconds);
	t -= ((unsigned int)a.info.onTime / 1000);
	convert_datetime_to_epoch(t, year, month, day, hour, minute, second);
}

void ArgosScheduler::build_long_packet(std::vector<GPSLogEntry> const& gps_entries, ArgosPacket& packet)
{
	unsigned int base_pos = 0;

	DEBUG_TRACE("ArgosScheduler::build_long_packet: gps_entries: %u", gps_entries.size());

	// Must be at least two GPS entries in the vector to use long packet format
	if (gps_entries.size() < 2) {
		DEBUG_ERROR("ArgosScheduler::build_long_packet: requires at least 2 GPS entries in vector (only %u)", gps_entries.size());
		return;
	}

	// Reserve required number of bytes
	packet.assign(LONG_PACKET_BYTES, 0);

	// Header bytes
	PACK_BITS(PACKET_SYNC, packet, base_pos, 24);
	PACK_BITS(LONG_PACKET_MSG_LENGTH, packet, base_pos, 4);
	PACK_BITS(m_argos_config.argos_id>>8, packet, base_pos, 20);
	PACK_BITS(m_argos_config.argos_id, packet, base_pos, 8);

	// Payload bytes
#ifdef ARGOS_USE_CRC8
	PACK_BITS(0, packet, base_pos, 8);  // Zero CRC field (computed later)
#else
	PACK_BITS(LONG_PACKET_BITFIELD, packet, base_pos, 8);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: bitfield=%u", LONG_PACKET_BITFIELD);
#endif

	// This will adjust the log time for the GPS on time since we want the time
	// of when the GPS was scheduled and not the log time
	uint8_t day, hour, minute;
	adjust_logtime_for_gps_ontime(gps_entries[0], day, hour, minute);

	PACK_BITS(day, packet, base_pos, 5);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: day=%u", (unsigned int)day);
	PACK_BITS(hour, packet, base_pos, 5);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: hour=%u", (unsigned int)hour);
	PACK_BITS(minute, packet, base_pos, 6);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: min=%u", (unsigned int)minute);

	// First GPS entry
	if (gps_entries[0].info.valid) {
		PACK_BITS(convert_latitude(gps_entries[0].info.lat), packet, base_pos, 21);
		DEBUG_TRACE("ArgosScheduler::build_long_packet: lat=%u (%lf)", convert_latitude(gps_entries[0].info.lat), gps_entries[0].info.lat);
		PACK_BITS(convert_longitude(gps_entries[0].info.lon), packet, base_pos, 22);
		DEBUG_TRACE("ArgosScheduler::build_long_packet: lon=%u (%lf)", convert_longitude(gps_entries[0].info.lon), gps_entries[0].info.lon);
		double gspeed = (SECONDS_PER_HOUR * gps_entries[0].info.gSpeed) / (2*MM_PER_KM);
		PACK_BITS((unsigned int)gspeed, packet, base_pos, 7);
		DEBUG_TRACE("ArgosScheduler::build_long_packet: speed=%u", (unsigned int)gspeed);
	} else {
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
		PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		PACK_BITS(0xFF, packet, base_pos, 7);
	}

	// OUTOFZONE_FLAG
	PACK_BITS(m_argos_config.is_out_of_zone, packet, base_pos, 1);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: is_out_of_zone=%u", m_argos_config.is_out_of_zone);

	unsigned int batt = std::min(127, std::max((int)gps_entries[0].info.batt_voltage - 2700, (int)0) / MV_PER_UNIT);
	PACK_BITS(batt, packet, base_pos, 7);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: voltage=%u (%u)", batt, gps_entries[0].info.batt_voltage);

	// LOWBATERY_FLAG
	PACK_BITS(m_argos_config.is_lb, packet, base_pos, 1);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: is_lb=%u", m_argos_config.is_lb);

	// Delta time loc
	PACK_BITS((unsigned int)m_argos_config.delta_time_loc, packet, base_pos, 4);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: delta_time_loc=%u", (unsigned int)m_argos_config.delta_time_loc);

	// Subsequent GPS entries
	for (unsigned int i = 1; i < MAX_GPS_ENTRIES_IN_PACKET; i++) {
		DEBUG_TRACE("ArgosScheduler::build_long_packet: gps_valid[%u]=%u", i, gps_entries[i].info.valid);
		if (gps_entries.size() <= i || 0 == gps_entries[i].info.valid) {
			PACK_BITS(0xFFFFFFFF, packet, base_pos, 21);
			PACK_BITS(0xFFFFFFFF, packet, base_pos, 22);
		} else {
			PACK_BITS(convert_latitude(gps_entries[i].info.lat), packet, base_pos, 21);
			DEBUG_TRACE("ArgosScheduler::build_long_packet: lat[%u]=%u (%lf)", i, convert_latitude(gps_entries[i].info.lat), gps_entries[i].info.lat);
			PACK_BITS(convert_longitude(gps_entries[i].info.lon), packet, base_pos, 22);
			DEBUG_TRACE("ArgosScheduler::build_long_packet: lon[%u]=%u (%lf)", i, convert_longitude(gps_entries[i].info.lon), gps_entries[i].info.lon);
		}
	}

	// Calculate CRC8
#ifdef ARGOS_USE_CRC8
	unsigned char crc8 = CRC8::checksum(packet.substr(LONG_PACKET_HEADER_BYTES+1), LONG_PACKET_PAYLOAD_BITS - 8);
	unsigned int crc_offset = 8*LONG_PACKET_HEADER_BYTES;
	PACK_BITS(crc8, packet, crc_offset, 8);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: crc8=%02x", crc8);
#endif

	// BCH code B255_223_4
	BCHCodeWord code_word = BCHEncoder::encode(
			BCHEncoder::B255_223_4,
			sizeof(BCHEncoder::B255_223_4),
			packet.substr(LONG_PACKET_HEADER_BYTES), LONG_PACKET_PAYLOAD_BITS);
	DEBUG_TRACE("ArgosScheduler::build_long_packet: bch=%08x", code_word);

	// Append BCH code
	PACK_BITS(code_word, packet, base_pos, BCHEncoder::B255_223_4_CODE_LEN);
}

void ArgosScheduler::handle_packet(ArgosPacket const& packet, unsigned int total_bits, const ArgosMode mode) {
	DEBUG_TRACE("ArgosScheduler::handle_packet: bytes=%u total_bits=%u freq=%lf power=%u mode=%u",
			packet.size(), total_bits, m_argos_config.frequency, m_argos_config.power, (unsigned int)mode);
	DEBUG_TRACE("ArgosScheduler::handle_packet: data=%s", Binascii::hexlify(packet).c_str());
	power_on();
	set_frequency(m_argos_config.frequency);
	set_tx_power(m_argos_config.power);
	if (m_data_notification_callback) {
    	ServiceEvent e;
    	e.event_type = ServiceEventType::ARGOS_TX_START;
    	e.event_data = false;
        m_data_notification_callback(e);
	}
	send_packet(packet, total_bits, mode);
	if (m_data_notification_callback) {
    	ServiceEvent e;
    	e.event_type = ServiceEventType::ARGOS_TX_END;
    	e.event_data = false;
        m_data_notification_callback(e);
	}
	power_off();

	// Update the LAST_TX in the configuration store
	std::time_t last_tx = rtc->gettime();
	configuration_store->write_param(ParamID::LAST_TX, last_tx);

	// Increment TX counter
	configuration_store->increment_tx_counter();

	// Save configuration params
	configuration_store->save_params();
}

void ArgosScheduler::time_sync_burst_algorithm() {
	unsigned int num_entries = sensor_log->num_entries();

	DEBUG_TRACE("ArgosScheduler::time_sync_burst_algorithm: num_gps_entries=%u log_size=%u", m_num_gps_entries, num_entries);

	if (m_num_gps_entries && num_entries) {
		ArgosPacket packet;
		GPSLogEntry gps_entry;
		unsigned int index = num_entries - 1;

		// Read most recent log entry but don't decrement any counters
		sensor_log->read(&gps_entry, index);

		build_short_packet(gps_entry, packet);
		handle_packet(packet, SHORT_PACKET_BYTES * BITS_PER_BYTE, ArgosMode::ARGOS_2);

	} else {
		DEBUG_ERROR("ArgosScheduler::time_sync_burst_algorithm: sensor log state is invalid, can't transmit");
	}

	// Even on an error we mark the time sync burst as sent because otherwise we would enter an infinite rescheduling scenario
	m_time_sync_burst_sent = true;
}

void ArgosScheduler::periodic_algorithm() {
	unsigned int max_index = (((unsigned int)m_argos_config.depth_pile + MAX_GPS_ENTRIES_IN_PACKET-1) / MAX_GPS_ENTRIES_IN_PACKET);
	unsigned int index = m_msg_index % max_index;
	unsigned int num_entries = sensor_log->num_entries();
	unsigned int span = std::min((unsigned int)MAX_GPS_ENTRIES_IN_PACKET, (unsigned int)m_argos_config.depth_pile);
	unsigned int eligible_gps_count = 0;
	unsigned int first_eligible_gps_index = -1;
	unsigned int max_msg_index;
	ArgosPacket packet;
	GPSLogEntry gps_entry;

	// Get latest configuration
	configuration_store->get_argos_configuration(m_argos_config);

	DEBUG_TRACE("ArgosScheduler::periodic_algorithm: msg_index=%u/%u span=%u num_log_entries=%u", m_msg_index % max_index, max_index, span, m_num_gps_entries);

	// Mark last schedule attempt
	m_last_transmission_schedule = m_tr_nom_schedule;

	// Wait for at least span entries
	if (m_num_gps_entries < span) {
		DEBUG_TRACE("ArgosScheduler::periodic_algorithm: insufficient GPS entries %u available", m_num_gps_entries);
		return;
	}

	// Find first eligible slot for transmission
	max_msg_index = m_msg_index + max_index;
	while (m_msg_index < max_msg_index) {
		index = m_msg_index % max_index;
		// Check to see if any GPS entry has a non-zero burst counter
		for (unsigned int k = 0; k < span; k++) {
			unsigned int idx = m_num_gps_entries - (span * (index+1)) + k;
			if (m_gps_entry_burst_counter.count(idx)) {
				if (m_gps_entry_burst_counter.at(idx)) {
					eligible_gps_count++;
					if (first_eligible_gps_index == (unsigned int)-1)
						first_eligible_gps_index = idx;
				}
			} else
				break;
		}

		// Ensure the entire slot has elib
		if (eligible_gps_count) {
			DEBUG_TRACE("ArgosScheduler::periodic_algorithm: found %u eligible GPS entries in slot %u", eligible_gps_count, index);
			break;
		}

		m_msg_index++;
	}

	// No further action if no slot is eligible for transmission
	if (m_msg_index == max_msg_index) {
		DEBUG_WARN("ArgosScheduler::periodic_algorithm: no eligible slot found");
		return;
	}

	// Handle short packet case
	if (eligible_gps_count == 1) {
		// If only a single eligible GPS, then use first_eligible_gps_index
		DEBUG_TRACE("ArgosScheduler::periodic_algorithm: using short packet for log entry %u bursts remaining %u",
				first_eligible_gps_index, m_gps_entry_burst_counter.at(first_eligible_gps_index));
		sensor_log->read(&gps_entry, first_eligible_gps_index);

		// Decrement GPS entry burst counter
		m_gps_entry_burst_counter.at(first_eligible_gps_index) = std::max((int)0, (int)m_gps_entry_burst_counter.at(first_eligible_gps_index) - 1);

		build_short_packet(gps_entry, packet);
		handle_packet(packet, SHORT_PACKET_BYTES * BITS_PER_BYTE, ArgosMode::ARGOS_2);
	} else {

		DEBUG_TRACE("ArgosScheduler::periodic_algorithm: using long packet");

		std::vector<GPSLogEntry> gps_entries;
		for (unsigned int k = 0; k < span; k++) {
			unsigned int idx = num_entries - (span * (index+1)) + k;
			unsigned int idx2 = m_num_gps_entries - (span * (index+1)) + k;
			DEBUG_TRACE("read gps_entry=%u bursts remaining %u", idx, m_gps_entry_burst_counter.at(idx2));
			sensor_log->read(&gps_entry, idx);
			gps_entries.push_back(gps_entry);

			// Decrement GPS entry burst counter
			m_gps_entry_burst_counter.at(idx2) = std::max((int)0, (int)m_gps_entry_burst_counter.at(idx2) - 1);
		}
		build_long_packet(gps_entries, packet);
		handle_packet(packet, LONG_PACKET_BYTES * BITS_PER_BYTE, ArgosMode::ARGOS_2);
	}

	// Increment for next message slot index
	m_msg_index++;
}

void ArgosScheduler::start(std::function<void(ServiceEvent&)> data_notification_callback) {
	DEBUG_INFO("ArgosScheduler::start");
	m_data_notification_callback = data_notification_callback;
	m_is_running = true;
	m_is_deferred = false;
	m_time_sync_burst_sent = false;
	m_msg_index = 0;
	m_num_gps_entries = 0;
	m_next_prepass = INVALID_SCHEDULE;
	m_tr_nom_schedule = INVALID_SCHEDULE;
	m_last_transmission_schedule = INVALID_SCHEDULE;
	m_gps_entry_burst_counter.clear();
	configuration_store->get_argos_configuration(m_argos_config);
	for (unsigned int i = 0; i < MAX_MSG_INDEX; i++) {
		m_msg_burst_counter[i] = (m_argos_config.ntry_per_message == 0) ? UINT_MAX : m_argos_config.ntry_per_message;
		m_gps_entries[i].clear();
	}
}

void ArgosScheduler::stop() {
	DEBUG_INFO("ArgosScheduler::stop");
	deschedule();
	m_is_running = false;

	power_off();
}

void ArgosScheduler::deschedule() {
	system_scheduler->cancel_task(m_argos_task);
}

void ArgosScheduler::notify_saltwater_switch_state(bool state) {
	DEBUG_TRACE("ArgosScheduler::notify_saltwater_switch_state");
	if (m_is_running && m_argos_config.underwater_en) {
		m_switch_state = state;
		if (!m_switch_state) {
			DEBUG_TRACE("ArgosScheduler::notify_saltwater_switch_state: state=0: rescheduling");
			m_earliest_tx = rtc->gettime() + m_argos_config.dry_time_before_tx;
			reschedule();
		} else {
			DEBUG_TRACE("ArgosScheduler::notify_saltwater_switch_state: state=1: deferring schedule");
			deschedule();
			m_is_deferred = true;
		}
	}
}
