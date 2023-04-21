#pragma once

#include "gps.hpp"
#include "service.hpp"

extern ConfigurationStore *configuration_store;

class GNSSDetectorService : public Service, public GPSEventListener {
public:
	GNSSDetectorService(GPSDevice& device) : Service(ServiceIdentifier::UW_SENSOR, "GNSSDetector"),
		m_device(device)
	{
		m_device.subscribe(*this);
	}

private:
	GPSDevice& m_device;
	int m_current_state;
	unsigned int m_period_underwater_ms;
	unsigned int m_period_surface_ms;
	unsigned int m_min_num_dry_samples;
	unsigned int m_num_dry_samples;

	void notify_update(bool state) {
		if (state != m_current_state) {
			DEBUG_INFO("GNSSDetectorService: notify_update: new_state=%u", state);
			m_current_state = state;
			ServiceEventData event = state;
			service_complete(&event);
		} else {
			service_complete(nullptr);
		}
	}

    void react(const GPSEventError&) {
		if (is_initiated()) {
			DEBUG_TRACE("GNSSDetectorService: react: GPSEventError");
			notify_update(m_current_state);
			poweroff();
		}
    }

    void react(const GPSEventMaxSatSamples&) {
		if (is_initiated()) {
			DEBUG_TRACE("GNSSDetectorService: react: GPSEventMaxSatSamples");
			notify_update(true); // Assume wet if we reached maximum samples
			poweroff();
		}
    }

    void react(const GPSEventSatReport& e) {
		if (is_initiated()) {
			DEBUG_TRACE("GNSSDetectorService: react: GPSEventSatReport: nSv=%u bestQual=%u dry=%u/%u",
					e.numSvs,
					e.bestSignalQuality,
					m_num_dry_samples,
					m_min_num_dry_samples);
			// Use quality threshold of 3 as a dry sample
			if (e.bestSignalQuality >= 3) {
				m_num_dry_samples++;
				if (m_num_dry_samples >= m_min_num_dry_samples) {
					DEBUG_TRACE("GNSSDetectorService: react: GPSEventSatReport: dry threshold met");
					notify_update(false);
					poweroff();
				}
			}
		}
    }

	void service_init() {
		m_current_state = -1;
		m_period_underwater_ms = 1000 * service_read_param<unsigned int>(ParamID::SAMPLING_UNDER_FREQ);
		m_period_surface_ms = 1000 * service_read_param<unsigned int>(ParamID::SAMPLING_SURF_FREQ);
		m_min_num_dry_samples = service_read_param<unsigned int>(ParamID::UW_MIN_DRY_SAMPLES);
	}

	void poweroff(void) {
		m_device.power_off();
	}

	void service_term() {}

	bool service_cancel() {
		if (is_initiated()) {
			DEBUG_TRACE("GNSSDetectorService: service_cancel");
			poweroff();
			return true;
		}
		return false;
	}

	unsigned int service_next_schedule_in_ms() {
		return m_current_state == -1 ? 0 : (m_current_state ? m_period_underwater_ms : m_period_surface_ms);
	}

	unsigned int service_next_timeout() {
		return 0;
	}

	void service_initiate() {
		// Do not initiate if GNSS is already active
		DEBUG_TRACE("GNSSDetectorService: service_initiate: requesting GNSS power on");
		GNSSConfig gnss_config;
		configuration_store->get_gnss_configuration(gnss_config);
		GPSNavSettings nav_settings = {
			gnss_config.fix_mode,
			gnss_config.dyn_model,
			gnss_config.assistnow_enable,
			gnss_config.assistnow_offline_enable,
			gnss_config.hdop_filter_enable,
			gnss_config.hdop_filter_threshold,
			gnss_config.hacc_filter_enable,
			gnss_config.hacc_filter_threshold,
		};
		nav_settings.num_consecutive_fixes = gnss_config.min_num_fixes;
		nav_settings.sat_tracking = true;
		nav_settings.max_sat_samples = service_read_param<unsigned int>(ParamID::UW_MAX_SAMPLES);
		m_num_dry_samples = 0;
		m_device.power_on(nav_settings);
	}

    bool service_is_usable_underwater() { return true; }
	bool service_is_enabled() override {
		bool enabled = service_read_param<bool>(ParamID::UNDERWATER_EN);
		BaseUnderwaterDetectSource src = service_read_param<BaseUnderwaterDetectSource>(ParamID::UNDERWATER_DETECT_SOURCE);
		return enabled && (src == BaseUnderwaterDetectSource::GNSS);
	}
};
