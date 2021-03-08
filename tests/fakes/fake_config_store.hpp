#ifndef __FAKE_CONFIG_HPP_
#define __FAKE_CONFIG_HPP_

#include "config_store.hpp"

class FakeConfigurationStore : public ConfigurationStore {
protected:
	void serialize_config() {}
	void serialize_zone() {}
	void update_battery_level() {}

public:
	BasePassPredict m_pass_predict;
	void init() { m_params = default_params; }
	bool is_valid() { return true; }
	bool is_zone_valid() { return true; }
	void factory_reset() {}
	BasePassPredict& read_pass_predict() { return m_pass_predict; }
	void write_pass_predict(BasePassPredict& value) { m_pass_predict = value; }
	bool is_battery_level_low() { return false; }
};

#endif // __FAKE_CONFIG_HPP_
