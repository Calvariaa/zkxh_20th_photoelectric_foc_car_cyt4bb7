#include "zf_common_debug.h"
#include "zf_device_config.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"
#include "mt6701ct.h"
#include "encoder.h"

//
#include "scb/cy_scb_spi.h"
#include "sysclk/cy_sysclk.h"
#include "gpio/cy_gpio.h"

static const cy_stc_scb_spi_config_t MT6701CT_SCB_cfg =
    {
        .spiMode = CY_SCB_SPI_MASTER,       /*** Specifies the mode of operation    ***/
        .subMode = CY_SCB_SPI_MOTOROLA,     /*** Specifies the sub mode of SPI operation    ***/
        .sclkMode = CY_SCB_SPI_CPHA1_CPOL0, /*** Clock is active low, data is changed on first edge ***/
        .oversample = 4,                    /*** SPI_CLOCK divided by SCB_SPI_OVERSAMPLING should be baudrate  ***/
        .rxDataWidth = 16ul,                /*** The width of RX data (valid range 4-16). It must be the same as \ref txDataWidth except in National sub-mode. ***/
        .txDataWidth = 16ul,                /*** The width of TX data (valid range 4-16). It must be the same as \ref rxDataWidth except in National sub-mode. ***/
        .enableMsbFirst = true,             /*** Enables the hardware to shift out the data element MSB first, otherwise, LSB first ***/
        .enableFreeRunSclk = false,         /*** Enables the master to generate a continuous SCLK regardless of whether there is data to send  ***/
        .enableInputFilter = false,         /*** Enables a digital 3-tap median filter to be applied to the input of the RX FIFO to filter glitches on the line. ***/
        .enableMisoLateSample = true,       /*** Enables the master to sample MISO line one half clock later to allow better timings. ***/
        .enableTransferSeperation = true,   /*** Enables the master to transmit each data element separated by a de-assertion of the slave select line (only applicable for the master mode) ***/
        .ssPolarity0 = false,               /*** SS0: active low ***/
        .ssPolarity1 = false,               /*** SS1: active low ***/
        .ssPolarity2 = false,               /*** SS2: active low ***/
        .ssPolarity3 = false,               /*** SS3: active low ***/
        .enableWakeFromSleep = false,       /*** When set, the slave will wake the device when the slave select line becomes active. Note that not all SCBs support this mode. Consult the device datasheet to determine which SCBs support wake from deep sleep. ***/
        .rxFifoTriggerLevel = 0ul,          /*** Interrupt occurs, when there are more entries of 2 in the RX FIFO */
        .rxFifoIntEnableMask = 0ul,         /*** Bits set in this mask will allow events to cause an interrupt  */
        .txFifoTriggerLevel = 0ul,          /*** When there are fewer entries in the TX FIFO, then at this level the TX trigger output goes high. This output can be connected to a DMA channel through a trigger mux. Also, it controls the \ref CY_SCB_SPI_TX_TRIGGER interrupt source. */
        .txFifoIntEnableMask = 0ul,         /*** Bits set in this mask allow events to cause an interrupt  */
        .masterSlaveIntEnableMask = 0ul,    /*** Bits set in this mask allow events to cause an interrupt  */
        .enableSpiDoneInterrupt = false,
        .enableSpiBusErrorInterrupt = false,
};

void mt6701ct_spi_init(const cy_stc_scb_spi_config_t *__SCB_SPI_cfg, volatile stc_SCB_t *__SPI_TYPE, uint32 baud, en_clk_dst_t __SPI_CLOCK,
                      uint8 __CLK_GPIO, en_hsiom_sel_t __CLK_HSIOM,
                      uint8 __MOSI_GPIO, en_hsiom_sel_t __MOSI_HSIOM,
                      uint8 __MISO_GPIO, en_hsiom_sel_t __MISO_HSIOM)
{
    uint64_t targetFreq = 8 * baud;
    uint64_t sourceFreq_fp5 = ((uint64_t)SPI_FREQ << 5ull);
    uint32_t divSetting_fp5 = (uint32_t)(sourceFreq_fp5 / targetFreq);

    Cy_SysClk_PeriphAssignDivider((en_clk_dst_t)__SPI_CLOCK, CY_SYSCLK_DIV_24_5_BIT, 1ul);
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup((en_clk_dst_t)__SPI_CLOCK), CY_SYSCLK_DIV_24_5_BIT, 1ul, ((divSetting_fp5 & 0x1FFFFFE0ul) >> 5ul), (divSetting_fp5 & 0x0000001Ful));
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup((en_clk_dst_t)__SPI_CLOCK), CY_SYSCLK_DIV_24_5_BIT, 1ul);

    cy_stc_gpio_pin_config_t spi_pin_cfg = {0};

    spi_pin_cfg.driveMode = SCB_MOSI_DRIVE_MODE;
    spi_pin_cfg.hsiom = __MOSI_HSIOM;
    Cy_GPIO_Pin_Init(get_port(__MOSI_GPIO), (__MOSI_GPIO % 8), &spi_pin_cfg);

    spi_pin_cfg.driveMode = SCB_CLK_DRIVE_MODE;
    spi_pin_cfg.hsiom = __CLK_HSIOM;
    Cy_GPIO_Pin_Init(get_port(__CLK_GPIO), (__CLK_GPIO % 8), &spi_pin_cfg);

    spi_pin_cfg.driveMode = SCB_MISO_DRIVE_MODE;
    spi_pin_cfg.hsiom = __MISO_HSIOM;
    Cy_GPIO_Pin_Init(get_port(__MISO_GPIO), (__MISO_GPIO % 8), &spi_pin_cfg);

    Cy_SCB_SPI_Init(__SPI_TYPE, __SCB_SPI_cfg, NULL);
    Cy_SCB_SPI_SetActiveSlaveSelect(__SPI_TYPE, 0ul);
    Cy_SCB_SPI_Enable(__SPI_TYPE);
}

uint16_t mt6701ct_spi_read_16bit_angle(volatile stc_SCB_t *__SPI_TYPE, uint8 __CS_GPIO)
{
    uint16_t read_data = 0;

    // 切换单次通信长度为16位

    __SPI_TYPE->unCTRL.u32Register &= 0xffff3fff;
    __SPI_TYPE->unCTRL.u32Register |= 0x00004000;
    __SPI_TYPE->unTX_CTRL.u32Register &= 0xffffffe0;
    __SPI_TYPE->unTX_CTRL.u32Register |= 0x0000000F;
    __SPI_TYPE->unRX_CTRL.u32Register &= 0xffffffe0;
    __SPI_TYPE->unRX_CTRL.u32Register |= 0x0000000F;
    // gpio_low(__CS_GPIO);
    // Cy_SCB_WriteTxFifo(__SPI_TYPE, 0xffff); // 发送寄存器地址
    // // 等待发送到数据
    // while (Cy_SCB_IsTxComplete(__SPI_TYPE) == 0)
    //     ;
    // // 等待接收到数据
    // while (Cy_SCB_SPI_GetNumInRxFifo(__SPI_TYPE) == 0)
    //     ;
    // gpio_high(__CS_GPIO);

    // Cy_SCB_SPI_ClearRxFifo(__SPI_TYPE); // 清除接收缓冲区

    gpio_low(__CS_GPIO);
    Cy_SCB_WriteTxFifo(__SPI_TYPE, 0); // 发送寄存器地址

    // 等待发送到数据
    while (Cy_SCB_IsTxComplete(__SPI_TYPE) == 0)
        ;
    // 等待接收到数据
    while (Cy_SCB_SPI_GetNumInRxFifo(__SPI_TYPE) == 0)
        ;
    read_data = (uint16)(__SPI_TYPE->unRX_FIFO_RD.u32Register);
    gpio_high(__CS_GPIO);
    // Cy_SCB_SPI_ClearRxFifo(__SPI_TYPE); // 清除接收缓冲区

    return read_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MT6701CT 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     mt6701ct_read_register(MT6701CT_CHIP_ID);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint16_t mt6701ct_read_register_left()
{
    return mt6701ct_spi_read_16bit_angle(L_SPI_TYPE, L_CS_GPIO) >> 2;
}
static uint16_t mt6701ct_read_register_right()
{
    return mt6701ct_spi_read_16bit_angle(R_SPI_TYPE, R_CS_GPIO) >> 2;
}
void mt6701ct_init()
{
    mt6701ct_spi_init(&MT6701CT_SCB_cfg, L_SPI_TYPE, ENC_SPI_SPEED, L_SPI_CLOCK,
                     L_CLK_GPIO, L_CLK_HSIOM,
                     L_MOSI_GPIO, L_MOSI_HSIOM,
                     L_MISO_GPIO, L_MISO_HSIOM);
    gpio_init(L_CS_GPIO, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 配置CS端口
    
    mt6701ct_spi_init(&MT6701CT_SCB_cfg, R_SPI_TYPE, ENC_SPI_SPEED, R_SPI_CLOCK,
                     R_CLK_GPIO, R_CLK_HSIOM,
                     R_MOSI_GPIO, R_MOSI_HSIOM,
                     R_MISO_GPIO, R_MISO_HSIOM);
    gpio_init(R_CS_GPIO, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 配置CS端口
}

uint16_t mt6701ct_get_magnet_val_left()
{
    return mt6701ct_read_register_left();
}


uint16_t mt6701ct_get_magnet_val_right()
{
    return mt6701ct_read_register_right();
}
