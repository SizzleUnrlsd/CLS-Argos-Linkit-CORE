#include "argos_rx_service.hpp"
#include "config_store.hpp"
#include "binascii.hpp"
#include "dte_protocol.hpp"

extern ConfigurationStore *configuration_store;

#define SECONDS_PER_HOUR   3600

ArgosRxService::ArgosRxService(ArticDevice& device) : Service(ServiceIdentifier::ARGOS_RX, "ARGOSRX"), m_artic(device) {
}

void ArgosRxService::service_init() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	m_artic.set_frequency(argos_config.frequency);
	m_artic.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);
	m_artic.subscribe(*this);
}

void ArgosRxService::service_term() {
	m_artic.unsubscribe(*this);
}

bool ArgosRxService::service_is_enabled() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	return (argos_config.argos_rx_en && argos_config.mode == BaseArgosMode::PASS_PREDICTION && !argos_config.cert_tx_enable);
}

unsigned int ArgosRxService::service_next_schedule_in_ms() {
	if (!m_is_last_location_known) {
		DEBUG_TRACE("ArgosRxService::service_next_schedule_in_ms: can't schedule as last location is not known");
		return Service::SCHEDULE_DISABLED;
	}

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	std::time_t start_time = service_current_time();
	std::time_t stop_time = start_time + (std::time_t)(24 * SECONDS_PER_HOUR);
	struct tm *p_tm = std::gmtime(&start_time);
	struct tm tm_start = *p_tm;
	p_tm = std::gmtime(&stop_time);
	struct tm tm_stop = *p_tm;

	DEBUG_INFO("ArgosRxService::service_next_schedule_in_ms: searching window start=%llu stop=%llu", start_time, stop_time);

	BasePassPredict& pass_predict = configuration_store->read_pass_predict();
	PredictionPassConfiguration_t config = {
		(float)m_last_latitude,
		(float)m_last_longitude,
		{ (uint16_t)(1900 + tm_start.tm_year), (uint8_t)(tm_start.tm_mon + 1), (uint8_t)tm_start.tm_mday, (uint8_t)tm_start.tm_hour, (uint8_t)tm_start.tm_min, (uint8_t)tm_start.tm_sec },
		{ (uint16_t)(1900 + tm_stop.tm_year), (uint8_t)(tm_stop.tm_mon + 1), (uint8_t)tm_stop.tm_mday, (uint8_t)tm_stop.tm_hour, (uint8_t)tm_stop.tm_min, (uint8_t)tm_stop.tm_sec },
        (float)argos_config.prepass_min_elevation,        //< Minimum elevation of passes [0, 90]
		(float)argos_config.prepass_max_elevation,        //< Maximum elevation of passes  [maxElevation >= < minElevation]
		(float)argos_config.prepass_min_duration / 60.0f,  //< Minimum duration (minutes)
		argos_config.prepass_max_passes,                  //< Maximum number of passes per satellite (#)
		(float)argos_config.prepass_linear_margin / 60.0f, //< Linear time margin (in minutes/6months)
		argos_config.prepass_comp_step                    //< Computation step (seconds)
	};
	SatelliteNextPassPrediction_t next_pass;

	if (PREVIPASS_compute_next_pass_with_status(
    	&config,
		pass_predict.records,
		pass_predict.num_records,
		SAT_DNLK_ON_WITH_A3,
		SAT_UPLK_OFF,
		&next_pass)) {
		m_next_timeout = 1000 * next_pass.duration;
		unsigned int offset_ms = 1000 * (next_pass.epoch - start_time);
		DEBUG_INFO("ArgosRxService::service_next_schedule_in_ms: new DL RX window in %u ms duration %u ms", offset_ms, m_next_timeout);
		return offset_ms;
	}

	DEBUG_WARN("ArgosRxService::service_next_schedule_in_ms: failed to find DL RX window");

	return Service::SCHEDULE_DISABLED;
}

void ArgosRxService::service_initiate() {
	m_artic.start_receive(ArticMode::A3);
}

bool ArgosRxService::service_cancel() {
	m_artic.stop_receive();
	unsigned int t = m_artic.get_cumulative_receive_time();
	if (t) {
		configuration_store->increment_rx_time(t);
		configuration_store->save_params();
		return true;
	}
	return false;
}

unsigned int ArgosRxService::service_next_timeout() {
	return m_next_timeout;
}

bool ArgosRxService::service_is_triggered_on_surfaced() {
	return false;
}

bool ArgosRxService::service_is_usable_underwater() {
	return false;
}

void ArgosRxService::notify_peer_event(ServiceEvent& e) {

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
		e.event_type == ServiceEventType::SENSOR_LOG_UPDATED)
	{
		// Update location information if we got a valid fix
		GPSLogEntry& gps = std::get<GPSLogEntry>(e.event_data);
		if (gps.info.valid) {
			DEBUG_TRACE("ArgosRxService::notify_peer_event: updated GPS location");
			bool is_first_location = !m_is_last_location_known;
			m_is_last_location_known = true;
			m_last_longitude = gps.info.lon;
			m_last_latitude = gps.info.lat;
			if (is_first_location)
				service_reschedule();
		}
	}
}

bool ArgosRxService::service_is_triggered_on_event(ServiceEvent&) {
	return false;
}

void ArgosRxService::react(ArticEventRxPacket const& e) {
	DEBUG_INFO("ArgosScheduler::handle_rx_packet: packet=%s length=%u", Binascii::hexlify(e.packet).c_str(), e.size_bits);

	// Increment RX counter
	configuration_store->increment_rx_counter();

	// Save configuration params
	configuration_store->save_params();

	// Attempt to decode the queue of packets
	BasePassPredict pass_predict;
	PassPredictCodec::decode(m_orbit_params_map, m_constellation_status_map, e.packet, pass_predict);

	// Check to see if any new AOP records were found
	if (pass_predict.num_records)
		update_pass_predict(pass_predict);
}

void ArgosRxService::react(ArticEventDeviceError const&) {
	service_cancel();
	service_complete();
}

void ArgosRxService::update_pass_predict(BasePassPredict& new_pass_predict) {

	BasePassPredict existing_pass_predict;
	unsigned int num_updated_records = 0;

	// Read in the existing pass predict database
	existing_pass_predict = configuration_store->read_pass_predict();

	// Iterate over new candidate records
	for (unsigned int i = 0; i < new_pass_predict.num_records; i++) {
		unsigned int j = 0;

		for (; j < existing_pass_predict.num_records; j++) {
			// Check for existing hex ID match
			if (new_pass_predict.records[i].satHexId == existing_pass_predict.records[j].satHexId) {
				DEBUG_TRACE("ArgosRxService::update_pass_predict: hexid=%01x dl=%u ul=%u aop=%u",
						(unsigned int)new_pass_predict.records[i].satHexId,
						(unsigned int)new_pass_predict.records[i].downlinkStatus,
						(unsigned int)new_pass_predict.records[i].uplinkStatus,
						(unsigned int)new_pass_predict.records[i].bulletin.year ? 1: 0
						);
				if ((new_pass_predict.records[i].downlinkStatus || new_pass_predict.records[i].uplinkStatus) &&
						new_pass_predict.records[i].bulletin.year) {
					num_updated_records++;
					existing_pass_predict.records[j] = new_pass_predict.records[i];
				} else if (!new_pass_predict.records[i].downlinkStatus && !new_pass_predict.records[i].uplinkStatus) {
					num_updated_records++;
					existing_pass_predict.records[j].downlinkStatus = new_pass_predict.records[i].downlinkStatus;
					existing_pass_predict.records[j].uplinkStatus = new_pass_predict.records[i].uplinkStatus;
				}
				break;
			}
		}

		// If we reached the end of the existing database then this is a new hex ID, so
		// add it to the end of the existing database
		if (j == existing_pass_predict.num_records &&
			existing_pass_predict.num_records < MAX_AOP_SATELLITE_ENTRIES) {
			if ((new_pass_predict.records[i].downlinkStatus || new_pass_predict.records[i].uplinkStatus) &&
					new_pass_predict.records[i].bulletin.year) {
				existing_pass_predict.records[j] = new_pass_predict.records[i];
				existing_pass_predict.num_records++;
				num_updated_records++;
			} else if (!new_pass_predict.records[i].downlinkStatus && !new_pass_predict.records[i].uplinkStatus) {
				existing_pass_predict.records[j].downlinkStatus = new_pass_predict.records[i].downlinkStatus;
				existing_pass_predict.records[j].uplinkStatus = new_pass_predict.records[i].uplinkStatus;
				existing_pass_predict.num_records++;
				num_updated_records++;
			}
		}
	}

	DEBUG_TRACE("ArgosRxService::update_pass_predict: received=%u required=%u", num_updated_records, existing_pass_predict.num_records);

	// Check if we received a sufficient number of records
	if (num_updated_records == new_pass_predict.num_records && num_updated_records >= existing_pass_predict.num_records) {
		DEBUG_INFO("ArgosRxService::update_pass_predict: RX_OFF: committing %u AOP records", num_updated_records);
		configuration_store->write_pass_predict(existing_pass_predict);
		std::time_t new_aop_time = service_current_time();
		configuration_store->write_param(ParamID::ARGOS_AOP_DATE, new_aop_time);
		configuration_store->save_params();
		m_orbit_params_map.clear();
		m_constellation_status_map.clear();
		service_cancel();
		service_complete();
	}
}
