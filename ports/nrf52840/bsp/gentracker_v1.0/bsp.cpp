#include "bsp.hpp"

namespace BSP
{
	///////////////////////////////// GPIO definitions ////////////////////////////////
	const GPIO_InitTypeDefAndInst_t GPIO_Inits[GPIO_TOTAL_NUMBER] =
	{
		// pin number, direction, input, pull, drive sense
		/* GPIO_POWER_CONTROL   */ {NRF_GPIO_PIN_MAP(0,  4), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_DEBUG           */ {NRF_GPIO_PIN_MAP(0, 11), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SWS             */ {NRF_GPIO_PIN_MAP(0,  2), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIO_PIN_NOPULL, false, true, false}},
		/* GPIO_SWS_EN          */ {NRF_GPIO_PIN_MAP(0, 12), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SLOW_SWS_RX     */ {NRF_GPIO_PIN_MAP(0,  9), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SLOW_SWS_SEND   */ {NRF_GPIO_PIN_MAP(0, 10), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_WCHG_INTB       */ {NRF_GPIO_PIN_MAP(0, 17), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_AG_PWR          */ {NRF_GPIO_PIN_MAP(0, 25), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_G8_33           */ {NRF_GPIO_PIN_MAP(0, 28), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_G16_33          */ {NRF_GPIO_PIN_MAP(0, 29), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SAT_RESET       */ {NRF_GPIO_PIN_MAP(0, 31), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_SAT_EN          */ {NRF_GPIO_PIN_MAP(1, 15), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_EXT1_GPIO1      */ {NRF_GPIO_PIN_MAP(1, 14), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_EXT1_GPIO2      */ {NRF_GPIO_PIN_MAP(1, 13), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_EXT1_GPIO3      */ {NRF_GPIO_PIN_MAP(1,  1), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_GPS_EXT_INT     */ {NRF_GPIO_PIN_MAP(1, 11), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_GREEN       */ {NRF_GPIO_PIN_MAP(1, 10), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_RED         */ {NRF_GPIO_PIN_MAP(1,  7), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_LED_BLUE        */ {NRF_GPIO_PIN_MAP(1,  4), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_GPS_PWR_EN      */ {NRF_GPIO_PIN_MAP(1,  6), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_INT1_SAT        */ {NRF_GPIO_PIN_MAP(1,  5), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_INT2_SAT        */ {NRF_GPIO_PIN_MAP(0, 30), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_REED_SW         */ {NRF_GPIO_PIN_MAP(1,  3), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIO_PIN_NOPULL, false, true, false}},
		/* GPIO_INT1_AG         */ {NRF_GPIO_PIN_MAP(1,  2), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_INT2_AG         */ {NRF_GPIO_PIN_MAP(0, 13), NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_FLASH_IO2       */ {NRF_GPIO_PIN_MAP(0, 22), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
		/* GPIO_FLASH_IO3       */ {NRF_GPIO_PIN_MAP(1,  0), NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE, {}},
	};

/////////////////////////////////// UART definitions ////////////////////////////////

    // Supported baudrates:
    // NRF_UARTE_BAUDRATE_1200
    // NRF_UARTE_BAUDRATE_2400
    // NRF_UARTE_BAUDRATE_4800
    // NRF_UARTE_BAUDRATE_9600
    // NRF_UARTE_BAUDRATE_14400
    // NRF_UARTE_BAUDRATE_19200
    // NRF_UARTE_BAUDRATE_28800
    // NRF_UARTE_BAUDRATE_31250
    // NRF_UARTE_BAUDRATE_38400
    // NRF_UARTE_BAUDRATE_56000
    // NRF_UARTE_BAUDRATE_57600
    // NRF_UARTE_BAUDRATE_76800
    // NRF_UARTE_BAUDRATE_115200
    // NRF_UARTE_BAUDRATE_230400
    // NRF_UARTE_BAUDRATE_250000
    // NRF_UARTE_BAUDRATE_460800
    // NRF_UARTE_BAUDRATE_921600
    // NRF_UARTE_BAUDRATE_1000000

    const UART_InitTypeDefAndInst UART_Inits[UART_TOTAL_NUMBER] =
    {
    #if NRFX_UARTE0_ENABLED
        {
            .uarte = NRFX_UARTE_INSTANCE(0),
            {
                .pseltxd = NRF_GPIO_PIN_MAP(1, 9),
                .pselrxd = NRF_GPIO_PIN_MAP(1, 8),
                .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
                .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
                .p_context = NULL, // Context passed to interrupt handler
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_9600, // See table above
                .interrupt_priority = INTERRUPT_PRIORITY_UART_0,
            }
        },
    #endif
    #if NRFX_UARTE1_ENABLED
        {
            .uarte = NRFX_UARTE_INSTANCE(1),
            {
                .pseltxd = NRF_GPIO_PIN_MAP(0, 11),
                .pselrxd = NRF_GPIO_PIN_MAP(0, 14),
                .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
                .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
                .p_context = NULL, // Context passed to interrupt handler
                .hwfc = NRF_UARTE_HWFC_DISABLED,
                .parity = NRF_UARTE_PARITY_EXCLUDED,
                .baudrate = NRF_UARTE_BAUDRATE_460800, // See table above
                .interrupt_priority = INTERRUPT_PRIORITY_UART_1,
            }
        }
    #endif
    };

    /////////////////////////////////// QSPI definitions ////////////////////////////////

    // Supported frequencies:
    // NRF_QSPI_FREQ_32MDIV1    32MHz/1
    // NRF_QSPI_FREQ_32MDIV2    32MHz/2
    // NRF_QSPI_FREQ_32MDIV3    32MHz/3
    // NRF_QSPI_FREQ_32MDIV4    32MHz/4
    // NRF_QSPI_FREQ_32MDIV5    32MHz/5
    // NRF_QSPI_FREQ_32MDIV6    32MHz/6
    // NRF_QSPI_FREQ_32MDIV7    32MHz/7
    // NRF_QSPI_FREQ_32MDIV8    32MHz/8
    // NRF_QSPI_FREQ_32MDIV9    32MHz/9
    // NRF_QSPI_FREQ_32MDIV10   32MHz/10
    // NRF_QSPI_FREQ_32MDIV11   32MHz/11
    // NRF_QSPI_FREQ_32MDIV12   32MHz/12
    // NRF_QSPI_FREQ_32MDIV13   32MHz/13
    // NRF_QSPI_FREQ_32MDIV14   32MHz/14
    // NRF_QSPI_FREQ_32MDIV15   32MHz/15
    // NRF_QSPI_FREQ_32MDIV16   32MHz/16

    const QSPI_InitTypeDefAndInst QSPI_Inits[QSPI_TOTAL_NUMBER] =
    {
    #ifdef NRFX_QSPI_ENABLED
        {
            {
                .xip_offset = 0, // Address offset in the external memory for Execute in Place operation
                {
                    .sck_pin = NRF_GPIO_PIN_MAP(0, 19),
                    .csn_pin = NRF_GPIO_PIN_MAP(0, 24),
                    .io0_pin = NRF_GPIO_PIN_MAP(0, 21),
                    .io1_pin = NRF_GPIO_PIN_MAP(0, 23),
                    .io2_pin = NRF_GPIO_PIN_MAP(0, 22),
                    .io3_pin = NRF_GPIO_PIN_MAP(1,  0),
                },
                {
                    .readoc = NRF_QSPI_READOC_READ4IO, // Number of data lines and opcode used for reading
                    .writeoc = NRF_QSPI_WRITEOC_PP4O,  // Number of data lines and opcode used for writing
                    .addrmode = NRF_QSPI_ADDRMODE_24BIT,
                    .dpmconfig = false, // Deep power-down mode enable
                },
                {
                    .sck_delay = 0, // SCK delay in units of 62.5 ns  <0-255>
                    .dpmen = false, // Deep power-down mode enable
                    .spi_mode = NRF_QSPI_MODE_0,
                    .sck_freq = NRF_QSPI_FREQ_32MDIV1, // See table above
                },
                .irq_priority = INTERRUPT_PRIORITY_QSPI_0
            }
        }
    #endif
    };

    ////////////////////////////////// RTC definitions /////////////////////////////////
    const RTC_InitTypeDefAndInst_t RTC_Inits[RTC_TOTAL_NUMBER] =
    {
    #if NRFX_CHECK(NRFX_RTC0_ENABLED)
        {
            .rtc = NRFX_RTC_INSTANCE(0),
            .irq_priority = INTERRUPT_PRIORITY_RTC_0
        },
    #endif
    #if NRFX_CHECK(NRFX_RTC1_ENABLED)
        {
            .rtc = NRFX_RTC_INSTANCE(1),
            .irq_priority = INTERRUPT_PRIORITY_RTC_1
        },
    #endif
    #if NRFX_CHECK(NRFX_RTC2_ENABLED)
        {
            .rtc = NRFX_RTC_INSTANCE(2),
            .irq_priority = INTERRUPT_PRIORITY_RTC_2
        }
    #endif
    };

    ////////////////////////////////// SPI definitions /////////////////////////////////
    const SPI_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER] =
    {
    #if NRFX_SPIM0_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(0),
            {
                .sck_pin  = NRFX_SPIM_PIN_NOT_USED,
                .mosi_pin = NRFX_SPIM_PIN_NOT_USED,
                .miso_pin = NRFX_SPIM_PIN_NOT_USED,
                .ss_pin   = NRFX_SPIM_PIN_NOT_USED,
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_0,
                .orc = 0xFF, // Over-run character
                .frequency = NRF_SPIM_FREQ_4M,
                .mode = NRF_SPIM_MODE_0,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        },
    #endif
    #if NRFX_SPIM1_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(1),
            {
                .sck_pin  = NRFX_SPIM_PIN_NOT_USED,
                .mosi_pin = NRFX_SPIM_PIN_NOT_USED,
                .miso_pin = NRFX_SPIM_PIN_NOT_USED,
                .ss_pin   = NRFX_SPIM_PIN_NOT_USED,
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_1,
                .orc = 0xFF, // Over-run character
                .frequency = NRF_SPIM_FREQ_4M,
                .mode = NRF_SPIM_MODE_0,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        },
    #endif
    #if NRFX_SPIM2_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(2),
            {
                .sck_pin  = NRF_GPIO_PIN_MAP(0, 8),
                .mosi_pin = NRF_GPIO_PIN_MAP(0, 6),
                .miso_pin = NRF_GPIO_PIN_MAP(0, 7),
                .ss_pin   = NRF_GPIO_PIN_MAP(0, 5),
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_2,
                .orc = 0xFF, // Over-run character
                .frequency = NRF_SPIM_FREQ_8M,
                .mode = NRF_SPIM_MODE_1,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        },
    #endif
    #if NRFX_SPIM3_ENABLED
        {
            .spim = NRFX_SPIM_INSTANCE(3),
            {
                .sck_pin  = NRF_GPIO_PIN_MAP(0, 19),
                .mosi_pin = NRF_GPIO_PIN_MAP(0, 21),
                .miso_pin = NRF_GPIO_PIN_MAP(0, 23),
                .ss_pin   = NRF_GPIO_PIN_MAP(0, 24),
                .ss_active_high = false,
                .irq_priority = INTERRUPT_PRIORITY_SPI_3,
                .orc = 0xFF, // Over-run character
                .frequency = NRF_SPIM_FREQ_32M,
                .mode = NRF_SPIM_MODE_0,
                .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
            }
        }
    #endif
    };

    ////////////////////////////////// ADC definitions /////////////////////////////////
    const ADC_InitTypeDefAndInst_t ADC_Inits =
    {
        {
            NRF_SAADC_RESOLUTION_14BIT,
            NRF_SAADC_OVERSAMPLE_DISABLED,
            INTERRUPT_PRIORITY_ADC,
            false
        },
        {
            {
                // ADC_CHANNEL_0
                NRF_SAADC_RESISTOR_DISABLED,
                NRF_SAADC_RESISTOR_DISABLED,
                NRF_SAADC_GAIN1_6,
                NRF_SAADC_REFERENCE_INTERNAL,
                NRF_SAADC_ACQTIME_40US,
                NRF_SAADC_MODE_SINGLE_ENDED,
                NRF_SAADC_BURST_DISABLED,
                NRF_SAADC_INPUT_AIN1,
                NRF_SAADC_INPUT_DISABLED
            }
        }
    };
}
