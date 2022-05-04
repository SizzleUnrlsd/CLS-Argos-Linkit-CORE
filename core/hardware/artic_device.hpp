#pragma once

#include <list>
#include <string>
#include <cstdint>
#include "debug.hpp"

using ArticPacket = std::string;

struct ArticEventPowerOn {};
struct ArticEventPowerOff {};
struct ArticEventTxStarted {};
struct ArticEventTxComplete {};
struct ArticEventRxStarted {};
struct ArticEventRxPacket {
	ArticPacket packet;
	unsigned int size_bits;
};
struct ArticEventDeviceIdle {};
struct ArticEventDeviceReady {};
struct ArticEventDeviceError {};

class ArticEventListener {
public:
	virtual void react(ArticEventPowerOn const& ) {}
	virtual void react(ArticEventPowerOff const& ) {}
	virtual void react(ArticEventTxStarted const& ) {}
	virtual void react(ArticEventTxComplete const& ) {}
	virtual void react(ArticEventRxStarted const& ) {}
	virtual void react(ArticEventRxPacket const& ) {}
	virtual void react(ArticEventDeviceIdle const& ) {}
	virtual void react(ArticEventDeviceReady const& ) {}
	virtual void react(ArticEventDeviceError const& ) {}
};

enum ArticMode {
	A2,
	A3,
	A4
};

class ArticDevice {
private:
	std::list<ArticEventListener*> m_listeners;

public:
	virtual ~ArticDevice() {}
	void subscribe(ArticEventListener& m) {
		m_listeners.push_back(&m);
	}
	void unsubscribe(ArticEventListener& m) {
		m_listeners.remove(&m);
	}
	template<typename E> void notify(E const& e) {
		for (auto m : m_listeners) {
			m->react(e);
		}
	}
	virtual void send(const ArticMode mode, const ArticPacket& packet, const unsigned int size_bits) = 0;
	virtual void send_ack(const ArticMode mode, const unsigned int a_dcs, const unsigned int dl_msg_id, const unsigned int exec_report) = 0;
	virtual void start_receive(const ArticMode mode) = 0;
	virtual void stop_receive() = 0;
	virtual void set_frequency(const double freq) = 0;
	virtual void set_tcxo_warmup_time(const unsigned int time) = 0;
	virtual unsigned int get_cumulative_receive_time() = 0;
};
