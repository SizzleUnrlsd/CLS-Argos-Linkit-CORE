#pragma once

#include <functional>
#include "scheduler.hpp"
#include "service.hpp"


class UWDetector : public Service {
private:
	bool m_is_first_time;
	bool m_current_state;
	unsigned int m_period_underwater_ms;
	unsigned int m_period_surface_ms;
	unsigned int m_sample_iteration;
	unsigned int m_sched_units;
	unsigned int m_iterations;
	unsigned int m_period_ms;
	double m_activation_threshold;

public:
	UWDetector(unsigned int sched_units=1, unsigned int iterations=1, unsigned int period_ms=0) :
		Service(ServiceIdentifier::UW_SENSOR, "UWDetector")	{
		m_is_first_time = true;
		m_current_state = false;
		m_sched_units = sched_units;
		m_iterations = iterations;
		m_period_ms = period_ms;
	}
	virtual ~UWDetector() {}

private:
	virtual bool detector_state() { return false; };
	virtual bool service_is_enabled() = 0;
	void service_init() override;
	void service_term() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced() override;
	bool service_is_usable_underwater() override;
};
