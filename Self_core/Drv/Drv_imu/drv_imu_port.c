#include "drv_imu.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include <string.h>
enum { IDLE, ACC, GYRO_PENDING, GYRO, ABORTING };
struct { volatile uint32_t state, busy_count, error_count; uint32_t start_tick;
    drv_imu_dma_sample_t sample; } drv_imu_port_debug;
/* Both TX and RX DMA storage lives in ordinary SRAM, with static lifetime. */
/* Burst from CHIP_ID through the last data register, keeping two DMA phases.
 * ACC has command + dummy + registers 0x00..0x17; GYRO has no dummy byte.
 * The IDs distinguish a responding device from a successful DMA of 0x00/0xFF. */
static uint8_t s_acc_tx[26]={0x80}, s_gyro_tx[9]={0x80};
static uint8_t s_acc_rx[26],s_gyro_rx[9];
static drv_imu_sample_callback_t s_deliver;
static drv_imu_delay_t s_delay;
static uint8_t s_init_io_failed;
static void wake_task(SPI_HandleTypeDef *spi_handle) { (void)spi_handle; if(s_deliver)s_deliver(NULL); }
void drv_imu_port_bind(drv_imu_sample_callback_t callback,drv_imu_delay_t delay) { s_deliver=callback; s_delay=delay; }
static void acc_cs(void *ctx,uint8_t state) { (void)ctx; bsp_gpio_write_pin(GPIOA,GPIO_PIN_4,state); }
static void gyro_cs(void *ctx,uint8_t state) { (void)ctx; bsp_gpio_write_pin(GPIOB,GPIO_PIN_0,state); }
void drv_imu_port_delay_ms(uint32_t ms) { if(s_delay)s_delay(ms); else bsp_tim_delay_ms(ms); }
static void spi(void *ctx,const uint8_t *tx,uint8_t *rx,uint16_t len) {
    (void)ctx;
    if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)tx,rx,len,10)!=HAL_OK) s_init_io_failed=1;
}
static uint8_t ready(void) {
    return hspi1.State==HAL_SPI_STATE_READY && hspi1.hdmarx->State==HAL_DMA_STATE_READY &&
        hspi1.hdmatx->State==HAL_DMA_STATE_READY && !(hspi1.hdmarx->Instance->CR&DMA_SxCR_EN) &&
        !(hspi1.hdmatx->Instance->CR&DMA_SxCR_EN);
}
static void finished_error(SPI_HandleTypeDef *spi_handle) {
    (void)spi_handle; acc_cs(NULL,1); gyro_cs(NULL,1);
    drv_imu_port_debug.error_count++; drv_imu_port_debug.state=IDLE;
    if(s_deliver)s_deliver(NULL);
}
static void abort_dma(void) {
    drv_imu_port_debug.state=ABORTING;
    if(bsp_spi_abort_dma(&hspi1)!=HAL_OK && ready()) finished_error(&hspi1);
}
static uint8_t start_gyro(void) {
    drv_imu_port_debug.state=GYRO; gyro_cs(NULL,0);
    if(bsp_spi_transceive_dma(&hspi1,s_gyro_tx,s_gyro_rx,sizeof(s_gyro_tx))!=HAL_OK) { abort_dma(); return 0; }
    return 1;
}
static void completed(SPI_HandleTypeDef *spi_handle) {
    (void)spi_handle;
    if(drv_imu_port_debug.state==ACC) {
        acc_cs(NULL,1);
        memcpy(drv_imu_port_debug.sample.acc,s_acc_rx,2);
        memcpy(drv_imu_port_debug.sample.acc+2,s_acc_rx+20,6);
        drv_imu_port_debug.sample.acc_chip_id=s_acc_rx[2];
        drv_imu_port_debug.state=GYRO_PENDING;
        if(ready()) (void)start_gyro();
        else if(s_deliver)s_deliver(NULL);
    } else if(drv_imu_port_debug.state==GYRO) {
        gyro_cs(NULL,1);
        drv_imu_port_debug.sample.gyro[0]=s_gyro_rx[0];
        memcpy(drv_imu_port_debug.sample.gyro+1,s_gyro_rx+3,6);
        drv_imu_port_debug.sample.gyro_chip_id=s_gyro_rx[1];
        if(s_deliver)s_deliver(&drv_imu_port_debug.sample);
        drv_imu_port_debug.state=IDLE;
    }
}
uint8_t drv_imu_port_init(drv_imu_t *imu) {
    memset(&drv_imu_port_debug,0,sizeof(drv_imu_port_debug)); s_init_io_failed=0;
    drv_imu_bus_t bus={.spi_xfer=spi,.acc_cs=acc_cs,.gyro_cs=gyro_cs,.delay_ms=drv_imu_port_delay_ms};
    drv_imu_init(imu,&bus); drv_imu_start(imu);
    drv_imu_calibrate_gyro(imu,3000); drv_imu_read_acc_raw(imu); drv_imu_data_convert(imu);
    drv_imu_initial_alignment(imu); drv_imu_quat_to_euler(imu);
    bsp_spi_reg_dma_callbacks(&hspi1,completed,finished_error);
    bsp_spi_reg_dma_wake_callback(wake_task);
    return !s_init_io_failed;
}
uint8_t drv_imu_port_sample_trigger(uint32_t tick,uint32_t sequence,uint32_t cycles) {
    if(drv_imu_port_debug.state!=IDLE || !ready()) { drv_imu_port_debug.busy_count++; return 0; }
    drv_imu_port_debug.sample.tick=tick; drv_imu_port_debug.sample.sequence=sequence; drv_imu_port_debug.sample.cycles=cycles;
    drv_imu_port_debug.start_tick=bsp_tim_get_tick_ms(); drv_imu_port_debug.state=ACC; acc_cs(NULL,0);
    if(bsp_spi_transceive_dma(&hspi1,s_acc_tx,s_acc_rx,sizeof(s_acc_tx))!=HAL_OK) { abort_dma(); return 0; }
    return 1;
}
void drv_imu_port_poll(void) {
    uint32_t key=__get_PRIMASK(); __disable_irq();
    bsp_spi_poll_dma(&hspi1);
    if(drv_imu_port_debug.state==GYRO_PENDING && ready()) (void)start_gyro();
    else if(drv_imu_port_debug.state!=IDLE && drv_imu_port_debug.state!=ABORTING &&
        (uint32_t)(bsp_tim_get_tick_ms()-drv_imu_port_debug.start_tick)>2) abort_dma();
    else if(drv_imu_port_debug.state==ABORTING && ready()) finished_error(&hspi1);
    __set_PRIMASK(key);
}
uint32_t drv_imu_port_get_busy_count(void) { return drv_imu_port_debug.busy_count; }
uint32_t drv_imu_port_get_error_count(void) { return drv_imu_port_debug.error_count; }
void drv_imu_port_heater_start(void) { bsp_tim_pwm_start(&htim10,TIM_CHANNEL_1); }
void drv_imu_port_heater_set(uint16_t value) { bsp_tim_pwm_set_compare(&htim10,TIM_CHANNEL_1,value); }
