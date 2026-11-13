/** @file stm32f4xx_hal.h
 *  @brief 主机回归测试使用的 HAL 替身；不进入固件编译。
 */
#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; uint32_t idle; } UART_HandleTypeDef;
typedef struct { int instance; uint32_t ErrorCode; } CAN_HandleTypeDef;
typedef struct { void *Instance; } I2C_HandleTypeDef;
#define I2C3 ((void *)3)
typedef struct { int instance; } GPIO_TypeDef;
typedef struct {
    uint32_t FilterActivation, FilterBank, FilterMode, FilterScale;
    uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct {
    uint32_t StdId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId, DLC; } CAN_RxHeaderTypeDef;
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_FILTER_ENABLE 1U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_16BIT 0U
#define CAN_RX_FIFO0 0U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_TX_MAILBOX_EMPTY 2U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
typedef struct { int instance; } SPI_HandleTypeDef;
typedef struct { int instance; } TIM_HandleTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_BUSY 2
#define HAL_TIMEOUT 3
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_6 (1U << 6)
extern GPIO_TypeDef test_gpio_g;
#define GPIOG (&test_gpio_g)
#define I2C_MEMADD_SIZE_8BIT 1U
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_EXTI_Callback(uint16_t pin);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *port, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *port, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *, uint16_t);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *);
#define RESET 0
#define UART_FLAG_IDLE 1
#define UART_IT_IDLE 1
#define __HAL_UART_ENABLE_IT(port, flag) ((void)(port))
#define __HAL_UART_GET_FLAG(port, flag) ((port)->idle)
#define __HAL_UART_CLEAR_IDLEFLAG(port) ((port)->idle = 0)
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
#ifdef TEST_IRQ_STATE
extern uint32_t test_primask;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1; }
static inline void __set_PRIMASK(uint32_t state) { test_primask = state; }
#else
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t state) { (void)state; }
#endif
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *port);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *port, uint8_t *data, uint16_t len);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *can, uint32_t mailbox);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t notifications);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, const CAN_TxHeaderTypeDef *header,
                                     const uint8_t *data, uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data);
#endif
