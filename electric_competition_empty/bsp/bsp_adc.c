#include "bsp/bsp_adc.h"

#include "ti_msp_dl_config.h"

#define BSP_GRAY_ADC_INST           (ADC0)
#define BSP_GRAY_ADC_MEM_IDX        (DL_ADC12_MEM_IDX_0)
#define BSP_GRAY_ADC_INPUT_CHAN     (DL_ADC12_INPUT_CHAN_2)
#define BSP_GRAY_ADC_TIMEOUT_LOOPS  (20000U)

static bool g_gray_adc_initialized = false;

static const DL_ADC12_ClockConfig g_gray_adc_clock_cfg = {
    .clockSel = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};

void bsp_adc_gray_init(void)
{
    if (g_gray_adc_initialized) {
        return;
    }

    DL_GPIO_initPeripheralAnalogFunction(GPIO_LINE_LINE6_IOMUX);

    DL_ADC12_reset(BSP_GRAY_ADC_INST);
    DL_ADC12_enablePower(BSP_GRAY_ADC_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_ADC12_setClockConfig(BSP_GRAY_ADC_INST, (DL_ADC12_ClockConfig *) &g_gray_adc_clock_cfg);
    DL_ADC12_configConversionMem(BSP_GRAY_ADC_INST,
        BSP_GRAY_ADC_MEM_IDX,
        BSP_GRAY_ADC_INPUT_CHAN,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(BSP_GRAY_ADC_INST, DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(BSP_GRAY_ADC_INST, 64U);
    DL_ADC12_clearInterruptStatus(BSP_GRAY_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(BSP_GRAY_ADC_INST);

    g_gray_adc_initialized = true;
}

uint16_t bsp_adc_gray_read_raw(void)
{
    uint32_t timeout = BSP_GRAY_ADC_TIMEOUT_LOOPS;

    if (!g_gray_adc_initialized) {
        bsp_adc_gray_init();
    }

    DL_ADC12_clearInterruptStatus(BSP_GRAY_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(BSP_GRAY_ADC_INST);

    while ((DL_ADC12_getRawInterruptStatus(BSP_GRAY_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
        (timeout > 0U)) {
        timeout--;
    }

    return DL_ADC12_getMemResult(BSP_GRAY_ADC_INST, BSP_GRAY_ADC_MEM_IDX);
}
