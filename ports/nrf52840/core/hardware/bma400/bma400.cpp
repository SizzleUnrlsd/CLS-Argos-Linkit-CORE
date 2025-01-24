#include <map>

#include "bma400.h"
#include "bma400.hpp"
#include "bma400_defs.h"

#include "debug.hpp"
#include "nrf_delay.h"
#include "nrf_i2c.hpp"
#include "bsp.hpp"
// #include "nrfx_twim.h" //I think this is unused
#include "pmu.hpp"
#include "error.hpp"
#include "nrf_irq.hpp"

static const char *getPowerModeName(uint8_t power_mode) {
    switch (power_mode) {
        case 0x02: return "BMA400_MODE_NORMAL";
        case 0x00: return "BMA400_MODE_SLEEP";
        case 0x01: return "BMA400_MODE_LOW_POWER";
        default:   return "UNKNOWN_MODE";
    }
}

/* -- BMA 400 LL MANAGER -- */

class BMA400LLManager {
private:
	static inline uint8_t m_uniq_id = 0;  // default = 0
	static inline std::map<uint8_t, BMA400LL&> m_map;

public:
	static uint8_t register_device(BMA400LL& device);
	static void unregister_device(uint8_t unique_id);
	static BMA400LL& lookup_device(uint8_t unique_id);
};

BMA400LL& BMA400LLManager::lookup_device(uint8_t unique_id)
{
	return m_map.at(unique_id);
}

uint8_t BMA400LLManager::register_device(BMA400LL& device)
{
    DEBUG_TRACE("BMA400LLManager::register_device -> m_unique_id=%d, &device=%p",m_uniq_id, &device);
    m_map.insert({m_uniq_id, device});

	return m_uniq_id++;
}

void BMA400LLManager::unregister_device(uint8_t unique_id)
{
	m_map.erase(unique_id);
}


void BMA400LL::enable_wakeup_normal_mode(std::function<void()> func)
{
    DEBUG_INFO("BMA400LL::%s", __FUNCTION__);
    int8_t rslt;
    uint8_t power_mode = 0;

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    DEBUG_INFO("BMA400LL::enable_wakeup_normal_mode::POWER MODE == <%s>", getPowerModeName((int)(power_mode)));

	// bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_power_mode), rslt);
    DEBUG_INFO("BMA400LL:%s::POWER_MODE == <%s>", __FUNCTION__, getPowerModeName((int)(power_mode)));
    


    // .gen_int_thres = anymotion_thr,

    m_bma400_sensor_conf[1].type = BMA400_GEN1_INT;
    m_bma400_sensor_conf[1].param.gen_int = {
        .gen_int_thres = 12,  // Adjust sensitivity
        .gen_int_dur = (uint8_t)(m_wakeup_duration - 1),    // Minimum duration for interrupt
        .axes_sel = BMA400_AXIS_XYZ_EN,
        .data_src = BMA400_DATA_SRC_ACC_FILT2,
        .criterion_sel = BMA400_ACTIVITY_INT,
        .evaluate_axes = BMA400_ANY_AXES_INT,
        .ref_update = BMA400_UPDATE_EVERY_TIME,
        .hysteresis = BMA400_HYST_96_MG,
        .int_thres_ref_x = 0, // not used
        .int_thres_ref_y = 0, // not used
        .int_thres_ref_z = 32, // not used
        .int_chan = BMA400_INT_CHANNEL_1
    };

    rslt = bma400_set_sensor_conf(m_bma400_sensor_conf, 2, &m_bma400_dev);
    bma400_check_rslt("bma400_set_sensor_conf: GEN1 interrupt", rslt);

    m_bma400_int_en.type = BMA400_GEN1_INT_EN;
    m_bma400_int_en.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&m_bma400_int_en, 1, &m_bma400_dev);
    bma400_check_rslt("bma400_enable_interrupt: GEN1", rslt);

    m_irq.enable([this, func]() {
        if (!m_irq_pending) {
            m_irq_pending = true;
            func();
        }
    });

    // bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
    // bma400_set_power_mode(BMA400_MODE_LOW_POWER, &m_bma400_dev);
    // rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    // bma400_check_rslt("BMA400LL::bma400_get_power_mode()", rslt);
    // rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    // DEBUG_INFO("BMA400LL::enable_wakeup_normal_mode::POWER MODE == <%s>", getPowerModeName((int)(power_mode)));
}

/* Generic Interrupt feature */
void BMA400LL::enable_wakeup_auto_mode(std::function<void()> func)
{
	int8_t rslt = 0;
	uint8_t power_mode = 0;
	/* Variable to store interrupt status */
	uint16_t int_status __attribute__((unused));
	/* Sensor configuration structure */
	// struct bma400_setting accel_settin[2];
	/* Interrupt configuration structure */
	// struct interrupt_enable int_en[2];

    struct bma400_sensor_conf accel_settin[2];
    struct bma400_int_enable int_en[2];;

	/* Select the GEN1 and GEN2 interrupts for configuration */
	accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].type = BMA400_GEN1_INT;
	accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].type = BMA400_GEN2_INT;

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_power_mode), rslt);
    DEBUG_INFO("BMA400LL:%s::POWER_MODE == <%s>", __FUNCTION__, getPowerModeName((int)(power_mode)));
    

	/* Get the configurations set in the sensor */
	rslt = bma400_get_sensor_conf(&accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)], 2, &m_bma400_dev);

	/* Modify the required parameters from the "gen_int" structure present 
	 * inside the "bma400_setting" structure to configure the selected 
	 * GEN1/GEN2 interrupts */
	
	if (rslt == BMA400_OK) {
		/* Set the GEN 1 interrupt for activity detection */
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_chan          =   BMA400_INT_CHANNEL_2;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.axes_sel         =   BMA400_AXIS_XYZ_EN;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.criterion_sel    =   BMA400_INACTIVITY_INT;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.evaluate_axes    =   BMA400_ALL_AXES_INT;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.ref_update       =   BMA400_UPDATE_EVERY_TIME;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.data_src=BMA400_DATA_SRC_ACC_FILT2;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.gen_int_thres    =   0x05;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.gen_int_dur      =   100;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.hysteresis       =   BMA400_HYST_0_MG;
		/* accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_thres_ref_x  =   0; */
		/* accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_thres_ref_y  =   0; */
		/* accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_thres_ref_z  =   512; */ /* (0, 0, 1g) for gen1 reference, can be ignored here. */

		/* Set the GEN 2 interrupt for in-activity detection */
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.int_chan          =   BMA400_INT_CHANNEL_2;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.axes_sel         =   BMA400_AXIS_XYZ_EN;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.criterion_sel    =   BMA400_INACTIVITY_INT;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.evaluate_axes    =   BMA400_ANY_AXES_INT;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.ref_update       =   BMA400_UPDATE_ONE_TIME;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.data_src=BMA400_DATA_SRC_ACC_FILT1;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.gen_int_thres    =   0x10;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.gen_int_dur      =   0x01;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.hysteresis       =   BMA400_HYST_0_MG;
		
		/* Set the configurations in the sensor */
		rslt = bma400_set_sensor_conf(&accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)], 2, &m_bma400_dev);

		if (rslt == BMA400_OK) {
		
			/* Enable the Generic interrupts in the sensor */
			int_en[static_cast<int>(BMA400MODE::LOW_POWER)].type = BMA400_GEN1_INT_EN;
			int_en[static_cast<int>(BMA400MODE::LOW_POWER)].conf = BMA400_ENABLE;
			
			int_en[static_cast<int>(BMA400MODE::MODE_NORMAL)].type = BMA400_GEN2_INT_EN;
			int_en[static_cast<int>(BMA400MODE::MODE_NORMAL)].conf = BMA400_ENABLE;   /* int this case, gen2 is disabled */

			rslt = bma400_enable_interrupt(&int_en[static_cast<int>(BMA400MODE::LOW_POWER)], 2, &m_bma400_dev);

        }
    }
    m_irq.enable([this, func]() {
        if (!m_irq_pending) {
            m_irq_pending = true;
            func();
        }
    });
}
