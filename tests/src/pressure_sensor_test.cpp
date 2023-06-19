#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <iostream>

#include "pressure_sensor_service.hpp"
#include "mock_sensor.hpp"
#include "fake_config_store.hpp"
#include "fake_logger.hpp"
#include "fake_rtc.hpp"
#include "fake_timer.hpp"
#include "scheduler.hpp"


extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;


TEST_GROUP(PressureSensor)
{
	FakeConfigurationStore *fake_config_store;
	FakeTimer *fake_timer;
	FakeLog *fake_logger;
	Logger *logger;
	FakeRTC *fake_rtc;

	void setup() {
		fake_timer = new FakeTimer;
		system_timer = fake_timer;
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		fake_logger = new FakeLog;
		logger = fake_logger;
		fake_logger->create();
		system_scheduler = new Scheduler(system_timer);
		configuration_store->init();
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_config_store;
		delete fake_logger;
		delete fake_rtc;
	}
};


TEST(PressureSensor, SensorDisabled)
{
	MockSensor drv;
	PressureSensorService s(drv, logger);
	unsigned int num_callbacks = 0;

	system_timer->start();

	unsigned int period = 10;
	bool sensor_en = false;

	configuration_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE, sensor_en);
	configuration_store->write_param(ParamID::PRESSURE_SENSOR_PERIODIC, period);

	s.start([&num_callbacks](ServiceEvent &event) {
		if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			num_callbacks++;
		}
	});

	// Sampling should happen every 10
	for (unsigned int i = 0; i < 5; i++) {
		fake_timer->increment_counter(period*1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(0, num_callbacks);
	CHECK_EQUAL(0, logger->num_entries());

	s.stop();
}

TEST(PressureSensor, SchedulingPeriodic)
{
	MockSensor drv;
	PressureSensorService s(drv, logger);
	unsigned int num_callbacks = 0;

	system_timer->start();

	unsigned int period = 10;
	bool sensor_en = true;

	configuration_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE, sensor_en);
	configuration_store->write_param(ParamID::PRESSURE_SENSOR_PERIODIC, period);

	s.start([&num_callbacks](ServiceEvent &event) {
		if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			num_callbacks++;
		}
	});

	// Sampling should happen every 10
	for (unsigned int i = 0; i < 5; i++) {
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 0).andReturnValue((double)i);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 1).andReturnValue((double)i+1);
		fake_timer->increment_counter(period*1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(5, num_callbacks);
	CHECK_EQUAL(5, logger->num_entries());

	// Validate log entries
	for (unsigned int i = 0; i < 5; i++) {
		PressureLogEntry e;
		logger->read(&e, i);
		CHECK_EQUAL((double)i, e.pressure);
		CHECK_EQUAL((double)i+1, e.temperature);
	}

	s.stop();
}


TEST(PressureSensor, SchedulingNoPeriodic)
{
	MockSensor drv;
	PressureSensorService s(drv, logger);
	unsigned int num_callbacks = 0;

	system_timer->start();

	unsigned int period = 0;
	bool sensor_en = true;

	configuration_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE, sensor_en);
	configuration_store->write_param(ParamID::PRESSURE_SENSOR_PERIODIC, period);

	s.start([&num_callbacks](ServiceEvent &event) {
		if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			num_callbacks++;
		}
	});

	// Sampling should happen every 10
	for (unsigned int i = 0; i < 5; i++) {
		fake_timer->increment_counter(period*1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(0, num_callbacks);
	CHECK_EQUAL(0, logger->num_entries());

	s.stop();
}

TEST(PressureSensor, SchedulingPeriodicWithUWThresholdLoggingMode)
{
	MockSensor drv;
	PressureSensorService s(drv, logger);
	unsigned int num_callbacks = 0;

	system_timer->start();

	unsigned int period = 10;
	bool sensor_en = true;
	BasePressureSensorLoggingMode mode = BasePressureSensorLoggingMode::UW_THRESHOLD;

	configuration_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE, sensor_en);
	configuration_store->write_param(ParamID::PRESSURE_SENSOR_PERIODIC, period);
	configuration_store->write_param(ParamID::PRESSURE_SENSOR_LOGGING_MODE, mode);

	s.start([&num_callbacks](ServiceEvent &event) {
		if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			num_callbacks++;
		}
	});

	// Sampling should happen every 10 seconds (no trigger as below UW_THRESHOLD)
	for (unsigned int i = 0; i < 5; i++) {
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 0).andReturnValue((double)1.0);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 1).andReturnValue((double)24.0);
		fake_timer->increment_counter(period*1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(0, num_callbacks);
	CHECK_EQUAL(0, logger->num_entries());

	// Trigger UW state (should only log once)
	for (unsigned int i = 0; i < 5; i++) {
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 0).andReturnValue((double)1.1);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 1).andReturnValue((double)24.0);
		fake_timer->increment_counter(period*1000);
		system_scheduler->run();
	}

	CHECK_EQUAL(1, num_callbacks);
	CHECK_EQUAL(1, logger->num_entries());

	// Trigger surfaced state (should only log once)
	for (unsigned int i = 0; i < 5; i++) {
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 0).andReturnValue((double)1.0);
		mock().expectOneCall("read").onObject(&drv).withUnsignedIntParameter("port", 1).andReturnValue((double)24.0);
		fake_timer->increment_counter(period*1000);
		system_scheduler->run();
	}

	// Expect only two log entries
	CHECK_EQUAL(2, num_callbacks);
	CHECK_EQUAL(2, logger->num_entries());

	// Validate log entries
	PressureLogEntry e;

	// First log entry should be submerged (1.1 bar)
	logger->read(&e, 0);
	CHECK_EQUAL((double)1.1, e.pressure);
	CHECK_EQUAL((double)24.0, e.temperature);

	// Second log entry should be surfaced (1.0 bar)
	logger->read(&e, 1);
	CHECK_EQUAL((double)1.0, e.pressure);
	CHECK_EQUAL((double)24.0, e.temperature);

	s.stop();
}
