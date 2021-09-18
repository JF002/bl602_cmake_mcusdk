#include "SpiMaster.h"

#include <algorithm>
extern "C" {

#include <bl602_glb.h>
#include <bl602_gpio.h>
#include <bl602_spi.h>
  //#include <bl_irq.h>
}
using namespace Pinetime::Drivers;

SpiMaster::SpiMaster(const SpiMaster::SpiModule spi, const SpiMaster::Parameters& params) : spi {spi}, params {params} {
  mutex = xSemaphoreCreateBinary();
}

bool SpiMaster::Init() {
#if 1
  if(mutex == nullptr){
    mutex = xSemaphoreCreateBinary();
  }
  uint8_t gpio_pins[] = { // TODO: Pin definitions
          0,11,17
  };
  GLB_GPIO_Func_Init(GPIO_FUN_SPI, (GLB_GPIO_Type*)gpio_pins, 3);
  GLB_Swap_SPI_0_MOSI_With_MISO(ENABLE);
  GLB_Set_SPI_0_ACT_MOD_Sel(GLB_SPI_PAD_ACT_AS_MASTER);

  SPI_IntMask((SPI_ID_Type)0, SPI_INT_ALL,MASK);
  //bl_irq_enable(SPI_IRQn);
  //bl_irq_register_with_ctx(SPI_IRQn, (void*)spiInterrupt, this);

  SPI_Disable((SPI_ID_Type)0, SPI_WORK_MODE_MASTER);
  SPI_Disable((SPI_ID_Type)0, SPI_WORK_MODE_SLAVE);


  /* clock */
  /*1  --->  40 Mhz
   *2  --->  20 Mhz
   *5  --->  8  Mhz
   *6  --->  6.66 Mhz
   *10 --->  4 Mhz
   * */
  uint8_t clk_div = 10;
  GLB_Set_SPI_CLK(ENABLE,0);
  SPI_ClockCfg_Type clockcfg;
  clockcfg.startLen = clk_div;
  clockcfg.stopLen = clk_div;
  clockcfg.dataPhase0Len = clk_div;
  clockcfg.dataPhase1Len = clk_div;
  clockcfg.intervalLen = clk_div;
  SPI_ClockConfig(SPI_ID_0, &clockcfg);

  SPI_CFG_Type config;
  config.deglitchEnable = DISABLE;
  config.continuousEnable = DISABLE;
  config.byteSequence = SPI_BYTE_INVERSE_BYTE0_FIRST;
  config.bitSequence = SPI_BIT_INVERSE_MSB_FIRST;
  config.clkPhaseInv = SPI_CLK_PHASE_INVERSE_0;
  config.clkPolarity = SPI_CLK_POLARITY_LOW;
  config.frameSize = SPI_FRAME_SIZE_8;

  auto ret = SPI_Init(SPI_ID_0, &config);
  if(ret == 0)
    MSG("SPI Init OK!\r\n");



  ret = SPI_Enable(SPI_ID_0, SPI_WORK_MODE_MASTER);
  if(ret == 0)
    MSG("SPI Enable OK!\r\n");

  xSemaphoreGive(mutex);
#endif
  return true;
}

void SpiMaster::OnEndEvent() {
  if (currentBufferAddr == 0) {
    return;
  }

  auto s = currentBufferSize;
  if (s > 0) {
    auto currentSize = std::min((size_t) 255, s);
    PrepareTx(currentBufferAddr, currentSize);
    currentBufferAddr += currentSize;
    currentBufferSize -= currentSize;

    //spiBaseAddress->TASKS_START = 1;
  } else {
    if (taskToNotify != nullptr) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(taskToNotify, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    //nrf_gpio_pin_set(this->pinCsn);
    currentBufferAddr = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(mutex, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void SpiMaster::OnStartedEvent() {
}

void SpiMaster::PrepareTx(const volatile uint32_t bufferAddress, const volatile size_t size) {

}

void SpiMaster::PrepareRx(const volatile uint32_t cmdAddress,
                          const volatile size_t cmdSize,
                          const volatile uint32_t bufferAddress,
                          const volatile size_t size) {

}

bool SpiMaster::Write(uint8_t pinCsn, const uint8_t* data, size_t size) {
#if 1
  taskToNotify = xTaskGetCurrentTaskHandle();
  SPI_Send_8bits(SPI_ID_0, const_cast<uint8_t*>(data), size, SPI_TIMEOUT_DISABLE);
  xTaskNotifyGive(taskToNotify);
#endif
  return true;

}

bool SpiMaster::Read(uint8_t pinCsn, uint8_t* cmd, size_t cmdSize, uint8_t* data, size_t dataSize) {
#if 1
  taskToNotify = xTaskGetCurrentTaskHandle();

  SPI_Send_8bits(SPI_ID_0, const_cast<uint8_t*>(cmd), cmdSize, SPI_TIMEOUT_DISABLE);

  SPI_Recv_8bits(SPI_ID_0, const_cast<uint8_t*>(data), dataSize, SPI_TIMEOUT_DISABLE);
  xTaskNotifyGive(taskToNotify);

#endif
  return true;

}

bool SpiMaster::ReadWrite(uint8_t *in, const uint8_t *out, size_t size) {
  taskToNotify = xTaskGetCurrentTaskHandle();

  SPI_SendRecv_8bits(SPI_ID_0, const_cast<uint8_t*>(out), in, size, SPI_TIMEOUT_DISABLE);
  xTaskNotifyGive(taskToNotify);

  return true;
}

void SpiMaster::Sleep() {

}

void SpiMaster::Wakeup() {

}

bool SpiMaster::WriteCmdAndBuffer(uint8_t pinCsn, const uint8_t* cmd, size_t cmdSize, const uint8_t* data, size_t dataSize) {
#if 1
  taskToNotify = xTaskGetCurrentTaskHandle();

  SPI_Send_8bits(SPI_ID_0, const_cast<uint8_t*>(cmd), cmdSize, SPI_TIMEOUT_DISABLE);

  SPI_Send_8bits(SPI_ID_0, const_cast<uint8_t*>(data), dataSize, SPI_TIMEOUT_DISABLE);
  xTaskNotifyGive(taskToNotify);

#endif
  return true;

}

