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


/* -- BMA 400 LL -- */

BMA400LL::BMA400LL(unsigned int bus, unsigned char addr, int wakeup_pin)
    : m_bus(bus), m_addr(addr), m_irq(NrfIRQ(wakeup_pin)),
		m_unique_id(BMA400LLManager::register_device(*this)), m_accel_sleep_mode(BMA400_MODE_SLEEP),
		m_irq_pending(false)
{
	try {
		init();
	} catch(...) {
		BMA400LLManager::unregister_device(m_unique_id);
		throw;
	}
}

BMA400LL::~BMA400LL() {
	BMA400LLManager::unregister_device(m_unique_id);
}


// RANGE 4G, ODR 100, OSR 1 -> but not apply
void BMA400LL::init()
{
    int8_t rslt;

	DEBUG_TRACE("BMA400LL::init_rev()");

    m_bma400_dev.intf           = BMA400_I2C_INTF;
    m_bma400_dev.intf_ptr       = &m_unique_id;
    m_bma400_dev.read           = (bma400_read_fptr_t)i2c_read;
    m_bma400_dev.write          = (bma400_write_fptr_t)i2c_write;
    m_bma400_dev.delay_us       = (bma400_delay_us_fptr_t)delay_us;
    m_bma400_dev.read_write_len = BMA400_READ_WRITE_LENGTH;
    // m_bma400_dev.chip_id       read from bma in bma400_init()
    // m_bma400_dev.dummy_byte    only used for SPI
    // m_bma400_dev.resolution  = 12; // not used anywhere

    rslt = bma400_init(&m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_init),rslt);

    rslt = bma400_soft_reset(&m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_soft_reset), rslt);

    /* Low-power mode is set by default */
    setup_lp_conf();

    /* note: Sleep mode: Registers readable and writable, no sensortime */
    /* sleep mode ? 
    setup_sleep_mode(); */
}

int8_t BMA400LL::i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    DEBUG_TRACE("BMA400LL::i2c_write -> intf_ptr=%p, *(uint8_t *)intf_ptr=%x", intf_ptr, *(uint8_t *)intf_ptr);

    BMA400LL& device = BMA400LLManager::lookup_device(*(uint8_t *)intf_ptr);

    if (!length)
        return BMA400_OK;
    
    if (length > BMA400_MAX_LEN + sizeof(reg_addr))
        // return BMA400_E_OUT_OF_RANGE; No longer available in the latest version of the BOSCH driver
        return BMA400_E_INVALID_CONFIG;
    
    uint8_t buffer[BMA400_MAX_LEN];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, length);

    NrfI2C::write(device.m_bus, device.m_addr, (uint8_t*)buffer, length + sizeof(reg_addr), false);

    DEBUG_TRACE("BMA400LL::i2c_write %x : %x",reg_addr, reg_data);

    return BMA400_OK;
}

int8_t BMA400LL::i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    DEBUG_TRACE("BMA400LL::i2c_read -> intf_ptr=%p, *(uint8_t *)intf_ptr=%x", intf_ptr, *(uint8_t *)intf_ptr);

    BMA400LL& device = BMA400LLManager::lookup_device(*(uint8_t *)intf_ptr);

    DEBUG_TRACE("BMA400LL::i2c_read -> device.m_addr=%p, &reg_addr=%p", device.m_addr, &reg_addr);

    NrfI2C::write(device.m_bus, device.m_addr, &reg_addr, sizeof(reg_addr), true);
    DEBUG_TRACE("BMA400LL::i2c_read -> (uint8_t*)reg_data=%p, reg_data=%p", (uint8_t*)reg_data, reg_data);

	NrfI2C::read(device.m_bus, device.m_addr, (uint8_t*)reg_data, length);
    DEBUG_TRACE("BMA400LL::i2c_read %x : %x",reg_addr, reg_data);

    return BMA400_OK;
}

void BMA400LL::delay_us(uint32_t period, void *intf_ptr)
{
    PMU::delay_us(period);
}

double BMA400LL::convert_g_force(unsigned int g_scale, int16_t axis_value)
{
    double g_force = (double)g_scale * axis_value / 32768;
    DEBUG_INFO("   ---   GFORCE <%f> GFORCE   ---   ", g_force);
    return g_force;
	// return (double)g_scale * axis_value / 32768;
}

void BMA400LL::bma400_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMA400_OK:
            /* Do nothing */
            DEBUG_TRACE("BMA400 [%s]",api_name);
            break;
        case BMA400_E_NULL_PTR:
            DEBUG_ERROR("BMA400 Error [%d] : Null pointer\r\n", rslt);
            throw ErrorCode::I2C_COMMS_ERROR;
            break;
        case BMA400_E_COM_FAIL:
            DEBUG_ERROR("BMA400 Error [%d] : Communication failure\r\n", rslt);
            throw ErrorCode::I2C_COMMS_ERROR;
            break;
        case BMA400_E_INVALID_CONFIG:
            DEBUG_ERROR("BMA400 Error [%d] : Invalid configuration\r\n", rslt);
            throw ErrorCode::I2C_COMMS_ERROR;
            break;
        case BMA400_E_DEV_NOT_FOUND:
            DEBUG_ERROR("BMA400 Error [%d] : Device not found\r\n", rslt);
            throw ErrorCode::I2C_COMMS_ERROR;
            break;
        default:
            DEBUG_ERROR("BMA400 Error [%d] : Unknown error code\r\n", rslt);
            throw ErrorCode::I2C_COMMS_ERROR;
            break;
    }
}

//todo: this is where I should read 128 samples at 20Hz so -> PMU:delay_ms() -> ok but test it
void BMA400LL::read_xyz(double& x, double& y, double& z)
{
    uint8_t power_mode = 0;
    int8_t rslt = 0;
    // Turn accelerometer on so AXL is updated
    DEBUG_INFO("entering in BMA400LL::read_xyz");
    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    DEBUG_INFO("BMA400LL::read_xyz::POWER MODE == <%s>", getPowerModeName((int)(power_mode)));

    // Wait 50ms for reading (4 averaged samples, @ 100 Hz)
    PMU::delay_ms(50);

    // Read and convert accelerometer values
    union __attribute__((packed)) {
        uint8_t buffer[6];
        struct {
        	int16_t x;
        	int16_t y;
        	int16_t z;
        };
    } data;
    rslt = bma400_get_regs(BMA400_REG_ACCEL_DATA, data.buffer, sizeof(data.buffer), &m_bma400_dev);
    bma400_check_rslt("BMA400LL::read_xyz() bma400_get_regs", rslt);

    DEBUG_INFO("   ---   X <%d> X   ---   ", data.x);
    DEBUG_INFO("   ---   Y <%d> Y   ---   ", data.y);
    DEBUG_INFO("   ---   Z <%d> Z   ---   ", data.z);

    // DEBUG_WARN("GFORCE<%s>", BMA_ACC::toString(BMA_ACC::AccelerometerRange::RANGE_4G));

    // Convert to double precision G-force result on each axis
    x = convert_g_force(m_g_force, data.x);
    y = convert_g_force(m_g_force, data.y);
    z = convert_g_force(m_g_force, data.z);

    DEBUG_TRACE("BMA400LL::read_xyz: xyz=%f,%f,%f", x, y, z);
}

//Untodo: this is where I should read 128 samples at 20Hz so -> PMU:delay_ms()
void BMA400LL::read_xyz_128_at_20hz(double& x, double& y, double& z)
{
    int8_t rslt = 0;

    // Buffer pour accumuler les données
    double accumulated_x = 0, accumulated_y = 0, accumulated_z = 0;

    // Lire 128 échantillons à une fréquence de 20 Hz
    for (int i = 0; i < 128; ++i) {
        // Wait for the next sample at 20 Hz (50 ms interval)
        PMU::delay_ms(50);

        // Lire et convertir les valeurs de l'accéléromètre
        union __attribute__((packed)) {
            uint8_t buffer[6];
            struct {
                int16_t x;
                int16_t y;
                int16_t z;
            };
        } data;
        // [.... ....] [.... ....] [.... ....]
        rslt = bma400_get_regs(BMA400_REG_ACCEL_DATA, data.buffer, sizeof(data.buffer), &m_bma400_dev);
        bma400_check_rslt("BMA400LL::read_xyz() bma400_get_regs", rslt);

        // Convertir en G-force et accumuler
        accumulated_x += convert_g_force(m_g_force, data.x);
        accumulated_y += convert_g_force(m_g_force, data.y);
        accumulated_z += convert_g_force(m_g_force, data.z);
    }

    // Calculer la moyenne des 128 échantillons
    x = accumulated_x / 128;
    y = accumulated_y / 128;
    z = accumulated_z / 128;

    DEBUG_TRACE("BMA400LL::read_xyz: avg_xyz=%f,%f,%f", x, y, z);
}

double BMA400LL::read_temperature()
{
    int8_t rslt = 0;
    uint8_t power_mode = 0;

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    DEBUG_INFO("BMA400LL::read_temperature::POWER MODE == <%s>", getPowerModeName((int)(power_mode)));

    // Wait 10ms +/-12% for temperature reading
    PMU::delay_us(11200);

    // Read temperature
    int16_t temperature_data;
    bma400_get_temperature_data(&temperature_data, &m_bma400_dev);
    bma400_check_rslt("BMA400LL::read_temperature() bma400_get_temperature_data", rslt);

    // temperature_data = 195 ---> 19,5 degrees Celsius.

    DEBUG_TRACE("BMA400LL::read_temperature: temperature_data=%ld", temperature_data);
    return temperature_data;
}

void BMA400LL::set_wakeup_threshold(double thresh)
{
	m_wakeup_threshold = thresh;
}

void BMA400LL::set_wakeup_duration(double duration)
{
	m_wakeup_duration = duration;
}

void BMA400LL::set_wakeup_gforce(unsigned int g_force)
{
	m_g_force = g_force;
}

void BMA400LL::set_power_mode(unsigned int power_mode)
{
    m_power_mode = power_mode;
}

bool BMA400LL::check_and_clear_wakeup()
{
	InterruptLock lock;
	bool value = m_irq_pending;
	m_irq_pending = false;
	return value;
}

void BMA400LL::disable_wakeup()
{
    int8_t rslt = 0;

	// Disable IRQ
	m_irq.disable();
	rslt = bma400_set_power_mode(BMA400_MODE_SLEEP, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_set_power_mode), rslt);

	// Disable Generic interrupt 1
	m_bma400_int_en.type = BMA400_GEN1_INT_EN;
	m_bma400_int_en.conf = BMA400_DISABLE;
	rslt = bma400_enable_interrupt(&m_bma400_int_en, 1, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_enable_interrupt), rslt);
    DEBUG_INFO("%s", GET_API_NAME(disable_wakeup));
	m_irq.disable();
}

/* Sleep mode */
void BMA400LL::setup_sleep_mode(void)
{
    int8_t rslt = 0;
    uint8_t power_mode = 0;

    rslt = bma400_set_power_mode(BMA400_MODE_SLEEP, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_set_power_mode), rslt);

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_power_mode), rslt);
    DEBUG_TRACE("%s::POWER MODE == <%s>", __FUNCTION__, getPowerModeName(static_cast<int>(power_mode)));
}

/* Low-power mode */

void BMA400LL::setup_lp_conf(void)
{
    int8_t rslt = 0;
    uint8_t power_mode = 0;

    rslt = bma400_set_power_mode(BMA400_MODE_LOW_POWER, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_set_power_mode), rslt);

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_power_mode), rslt);
    DEBUG_TRACE("%s::POWER MODE == <%s>", __FUNCTION__, getPowerModeName(static_cast<int>(power_mode)));

    // m_bma400_sensor_conf[static_cast<int>(BMA400MODE::LOW_POWER)].type                 = BMA400_ACCEL;
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::LOW_POWER)].type                 = BMA400_ACCEL;
    // acc_filt1 has data rate between 12.5Hz and 800Hz
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::LOW_POWER)].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;
    // if the bma400 is in low-power mode, data conversion is fixed at 25Hz.
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::LOW_POWER)].param.accel.odr      = BMA400_ODR_25HZ;
    // Must be generic TODO : get the value on the config file (pylinkit)
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::LOW_POWER)].param.accel.range    = BMA400_RANGE_2G;
    // Current consumption between ranges 800 nA and 1200 nA
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::LOW_POWER)].param.accel.osr_lp   = BMA400_ACCEL_OSR_SETTING_0;

    // accel.fil1_bw = 0.48 * ODR or 0.24 * ODR
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::LOW_POWER)].param.accel.filt1_bw = static_cast<uint8_t>(0.24 * 12.5);
    DEBUG_TRACE("m_bma400_sensor_conf.param.accel.filt1_bw=%d", static_cast<uint8_t>(0.24 * 12.5));

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(m_bma400_sensor_conf, 1, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_set_sensor_conf), rslt);

    return;
}

void BMA400LL::enable_wakeup_lp_mode(std::function<void()> func)
{
    int8_t rslt = 0;
    uint8_t power_mode = 0;

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_power_mode), rslt);
    DEBUG_INFO("BMA400LL:%s::POWER_MODE == <%s>", __FUNCTION__, getPowerModeName((int)(power_mode)));

    struct bma400_device_conf dev_setting[2];

    /* Selecting auto wakeup on wakeup interrupt event */
    dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].type   = BMA400_AUTOWAKEUP_INT;

    /* Get the previously set settings */
	rslt = bma400_get_device_conf(dev_setting, 1, &m_bma400_dev);
    if (rslt == BMA400_OK) {
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.wakeup_axes_en        = BMA400_AXIS_XYZ_EN;
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.wakeup_ref_update     = BMA400_UPDATE_EVERY_TIME;
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.sample_count          = BMA400_SAMPLE_COUNT_4;
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_threshold    = 3;
		/* dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_ref_x		= 0 */
		/* dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_ref_y		= 0 */
		/* dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_ref_z		= 32 (0, 0, 1g) */
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_chan              = BMA400_MAP_BOTH_INT_PINS;
    
		rslt = bma400_set_device_conf(dev_setting, 1, &m_bma400_dev);
        if (rslt == BMA400_OK) {
			/* Enable the Generic interrupts in the sensor */
			m_bma400_int_en.type 	= BMA400_GEN1_INT_EN;
			m_bma400_int_en.conf 	= BMA400_ENABLE;
			rslt = bma400_enable_interrupt(&m_bma400_int_en, 1, &m_bma400_dev);
        }
    }

    m_irq.enable([this, func]() {
        if (!m_irq_pending) {
            m_irq_pending = true;
            func();
        }
    });
}

/* Normal mode */

void BMA400LL::setup_normal_conf(void)
{
    int8_t rslt = 0;
    uint8_t power_mode = 0;


    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_set_power_mode), rslt);

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_power_mode), rslt);
    DEBUG_TRACE("BMA400LL::setup_normal_conf::POWER MODE == <%s>", getPowerModeName((int)(power_mode)));

    /* Get the accelerometer configurations which are set in the sensor */
    rslt = bma400_get_sensor_conf(m_bma400_sensor_conf, 1, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_sensor_conf), rslt);

    /* Modify the desired configurations as per macros - available in bma400_defs.h file */
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::MODE_NORMAL)].type                 = BMA400_ACCEL;
    /* m_bma400_sensor_conf[1].param.accel.odr      = BMA400_ODR_100HZ; */
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.accel.odr      = BMA400_ODR_400HZ;
    /* Must be generic TODO : get the value on the config file (pylinkit) */
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.accel.range    = BMA400_RANGE_4G;
    // m_bma400_sensor_conf[1].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;
    /* acc_filt1 has data rate between 12.5Hz and 800Hz */
    m_bma400_sensor_conf[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    // m_bma400_sensor_conf[1].param.accel.osr

    // m_bma400_sensor_conf[1].param.accel.osr_lp

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(m_bma400_sensor_conf, 2, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_set_sensor_conf), rslt);

    // // note: Sleep mode: Registers readable and writable, no sensortime
    // rslt = bma400_set_power_mode(BMA400_MODE_SLEEP, &m_bma400_dev); 
    // bma400_check_rslt("bma400_set_power_mode: sleep", rslt);

    // m_bma400_device_conf.type                    = BMA400_INT_PIN_CONF;
    // m_bma400_device_conf.type                    = BMA400_AUTOWAKEUP_INT;

    // m_bma400_device_conf.param.int_conf.int_chan = BMA400_INT_CHANNEL_1;
    // m_bma400_device_conf.param.int_conf.pin_conf = BMA400_INT_PUSH_PULL_ACTIVE_0;
    // rslt = bma400_set_device_conf(&m_bma400_device_conf, BMA400_INT_PIN_CONF, &m_bma400_dev);
    // bma400_check_rslt("bma400_set_device_conf", rslt);

    return;
}

void BMA400LL::enable_wakeup_normal_mode(std::function<void()> func)
{
    DEBUG_INFO("BMA400LL::%s", __FUNCTION__);
    int8_t rslt;
    uint8_t power_mode = 0;

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
}

/* Auto low-power mode */

void BMA400LL::setup_autowakeup_autolowpower_conf(void)
{
    int8_t rslt = 0;
    uint8_t power_mode = 0;

    struct bma400_device_conf dev_setting[2];
    struct bma400_int_enable int_en;

    rslt = bma400_set_power_mode(BMA400_MODE_LOW_POWER, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_set_power_mode), rslt);

    rslt = bma400_get_power_mode(&power_mode, &m_bma400_dev);
    bma400_check_rslt(GET_API_NAME(bma400_get_power_mode), rslt);
    DEBUG_TRACE("%s::POWER MODE == <%s>", __FUNCTION__, getPowerModeName(static_cast<int>(power_mode)));

    // set_auto_low_power ???


    /* Selecting auto wakeup on wakeup interrupt event */
    dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].type   = BMA400_AUTOWAKEUP_INT;

	/* Selecting auto low power mode*/
	dev_setting[static_cast<int>(BMA400MODE::MODE_NORMAL)].type = BMA400_AUTO_LOW_POWER;

    /* Get the previously set settings */
	rslt = bma400_get_device_conf(dev_setting, 2, &m_bma400_dev);
	if (rslt == BMA400_OK) {
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.wakeup_axes_en 		= BMA400_AXIS_XYZ_EN;
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.wakeup_ref_update 		= BMA400_UPDATE_EVERY_TIME;
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.sample_count 		= BMA400_SAMPLE_COUNT_4;
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_threshold 		= 3;
		/* dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_ref_x		= 0 */
		/* dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_ref_y		= 0 */
		/* dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_wkup_ref_z		= 32 (0, 0, 1g) */
		dev_setting[static_cast<int>(BMA400MODE::LOW_POWER)].param.wakeup.int_chan			= BMA400_INT_CHANNEL_1;
		
		/* Enable auto low power on Gen1 trigger  */
		dev_setting[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.auto_lp.auto_low_power_trigger 	= BMA400_AUTO_LP_GEN1_TRIGGER;

		/* Set the configurations in sensor */
		rslt = bma400_set_device_conf(dev_setting, 2, &m_bma400_dev);

		if (rslt == BMA400_OK) {
		
			/* Enable the Generic interrupts in the sensor */
			int_en.type 	= BMA400_AUTO_WAKEUP_EN;
			int_en.conf 	= BMA400_ENABLE;
			
			rslt = bma400_enable_interrupt(&int_en, 1, &m_bma400_dev);

		/* The sensor toggles between Low-power mode and Normal mode if tilt the device(sensor) 
		 * or place it back to flat 
		 * this can be verfied by reading the power mode and printing it continuously as follows
		 *	while (1) {
		 *		rslt = bma400_get_power_mode(&power_mode, dev);
		 *		printf("\n POWER MODE : %d",power_mode);
		 *	}
		 * The power mode toggling can be seen from the printed console output
		 */
	    }
    }
    return;
}

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
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_chan        =   BMA400_INT_CHANNEL_2;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.axes_sel        =   BMA400_AXIS_XYZ_EN;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.criterion_sel   =   BMA400_INACTIVITY_INT;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.evaluate_axes   =   BMA400_ALL_AXES_INT;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.ref_update      =   BMA400_UPDATE_EVERY_TIME;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.data_src        =   BMA400_DATA_SRC_ACC_FILT2;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.gen_int_thres   =   0x05;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.gen_int_dur     =   100;
		accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.hysteresis      =   BMA400_HYST_0_MG;
		/* accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_thres_ref_x  =   0; */
		/* accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_thres_ref_y  =   0; */
		/* accel_settin[static_cast<int>(BMA400MODE::LOW_POWER)].param.gen_int.int_thres_ref_z  =   512; */ /* (0, 0, 1g) for gen1 reference, can be ignored here. */

		/* Set the GEN 2 interrupt for in-activity detection */
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.int_chan          =   BMA400_INT_CHANNEL_2;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.axes_sel          =   BMA400_AXIS_XYZ_EN;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.criterion_sel     =   BMA400_INACTIVITY_INT;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.evaluate_axes     =   BMA400_ANY_AXES_INT;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.ref_update        =   BMA400_UPDATE_ONE_TIME;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.data_src          =   BMA400_DATA_SRC_ACC_FILT1;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.gen_int_thres     =   0x10;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.gen_int_dur       =   0x01;
		accel_settin[static_cast<int>(BMA400MODE::MODE_NORMAL)].param.gen_int.hysteresis        =   BMA400_HYST_0_MG;
		
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

/* Generic wakeup */

void BMA400LL::enable_wakeup(std::function<void()> func)
{
    uint8_t power_mode = m_power_mode;

    DEBUG_INFO("%s::power_mode=%d", GET_API_NAME(BMA400LL::enable_wakeup_generic), power_mode);

    switch (power_mode)
    {
    case 0:
        DEBUG_INFO("        |       -   MODE LWP   -         |");
        setup_lp_conf();
	    enable_wakeup_lp_mode(func);
        break;
    case 1:
        DEBUG_INFO("        |       -   MODE NORMAl   -         |");
        setup_normal_conf();
	    enable_wakeup_normal_mode(func);
        break;
    /* auto low-power and normal mode */
    case 2:
        DEBUG_INFO("        |       -   AUTO-MODE   -         |");
        setup_autowakeup_autolowpower_conf();
        enable_wakeup_auto_mode(func);
        break;
    default:
        break;
    }
}

/* -- BMA400 -- */

BMA400::BMA400() : Sensor("AXL"), m_bma400(BMA400LL(BMA400_DEVICE, BMA400_ADDRESS, BMA400_WAKEUP_PIN)), m_last_x(0), m_last_y(0), m_last_z(0) {
}

#define THRESHOLD 0
#define DURATION 1

void BMA400::calibration_write(const double value, const unsigned int offset)
{
    switch (offset)
    {
    case THRESHOLD:
	    DEBUG_TRACE("BMA400::calibration_write: THRESHOLD_value=%f offset=%u", value, offset);
        break;
    case DURATION:
	    DEBUG_TRACE("BMA400::calibration_write: DURATION_value=%f offset=%u", value, offset);
        break;
    default:
        break;
    }
	if (0 == offset) {
		m_bma400.set_wakeup_threshold(value);
	} else if (1 == offset) {
		m_bma400.set_wakeup_duration(value);
	} else if (2 == offset) {
        m_bma400.set_wakeup_gforce(value);
	} else if (3 == offset) {
        m_bma400.set_power_mode(value);
    }
}

double BMA400::read(unsigned int offset)
{
	switch(offset) {
	case 0:
		return m_bma400.read_temperature();
		// return 0.0;
		break;
	case 1: // x
		m_bma400.read_xyz(m_last_x, m_last_y, m_last_z);
		return m_last_x;
		break;
	case 2: // y
		return m_last_y;
		break;
	case 3: // z
		return m_last_z;
		break;
	case 4: // IRQ pending
		return (double)m_bma400.check_and_clear_wakeup();
		// return (double)0.0;
		break;
	default:
		return 0;
	}
}

void BMA400::install_event_handler(unsigned int, std::function<void()> handler)
{
    m_bma400.enable_wakeup(handler);
};

void BMA400::remove_event_handler(unsigned int)
{
	m_bma400.disable_wakeup();
};
