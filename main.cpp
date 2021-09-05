#include "drivers/Spi.h"
#include "drivers/SpiMaster.h"
#include "sdk/bl_mcu_sdk/drivers/bl602_driver/std_drv/inc/bl602_adc.h"
extern "C" {
#include <FreeRTOS.h>
#include <bl602_glb.h>
#include <bl602_gpio.h>
#include <hal_gpio.h>
}
#include <task.h>
#include "sx126x/SX126x.hpp"


static uint8_t freertos_heap[4096*2];
static HeapRegion_t xHeapRegions[] = {
        { (uint8_t *)freertos_heap, 0 },
        { NULL, 0 }, /* Terminates the array. */
        { NULL, 0 }  /* Terminates the array. */
};
static StackType_t helloWorldStack[10000];
static StaticTask_t helloWorldHandle;

extern "C" {
void vAssertCalled(void) {
  MSG("vAssertCalled\r\n");

  while (1)
    ;
}

void vApplicationTickHook(void) {
  //MSG("vApplicationTickHook\r\n");
}

void vApplicationStackOverflowHook(void) {
  MSG("vApplicationStackOverflowHook\r\n");

  while (1)
    ;
}

void vApplicationMallocFailedHook(void) {
  MSG("vApplicationMallocFailedHook\r\n");

  while (1)
    ;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
  /* If the buffers to be provided to the Idle task are declared inside this
  function then they must be declared static - otherwise they will be allocated on
  the stack and so not exists after this function exits. */
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

  /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
  state will be stored. */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

  /* Pass out the array that will be used as the Idle task's stack. */
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;

  /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
  Note that, as the array is necessarily of type StackType_t,
  configMINIMAL_STACK_SIZE is specified in words, not bytes. */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
  /* If the buffers to be provided to the Timer task are declared inside this
  function then they must be declared static - otherwise they will be allocated on
  the stack and so not exists after this function exits. */
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

  /* Pass out a pointer to the StaticTask_t structure in which the Timer
  task's state will be stored. */
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

  /* Pass out the array that will be used as the Timer task's stack. */
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;

  /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
  Note that, as the array is necessarily of type StackType_t,
  configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
}
/*
class JFSx1262 : public SX126x {
  public:
  JFSx1262(Pinetime::Drivers::Spi& spi) : spi{spi} {}

  uint8_t HalGpioRead(GpioPinFunction_t func) override {
    //std::cout << "GPIO READ " << func;
    if(func != GpioPinFunction_t::GPIO_PIN_BUSY) {
      return 0;
    }
    return gpio_read(10);
  }
  void HalGpioWrite(GpioPinFunction_t func, uint8_t value) override {
    //std::cout << "GPIO WRITE << " << func << " -- " << std::to_string(value) << std::endl;
    if(func != GpioPinFunction_t::GPIO_PIN_RESET) {
      return;
    }
    gpio_write(18, value);
  }

  void HalSpiTransfer(uint8_t *buffer_in, const uint8_t *buffer_out,
                      uint16_t size) override {
    spi.ReadWrite(buffer_in, buffer_out, size);
  }

  private:
  Pinetime::Drivers::Spi& spi;
};
*/
uint8_t data[100] = {0x55, 0xBB,0x55, 0xAA,0x55, 0xAA,0x55, 0xAA};



static void HelloWorldTask(void *pvParameters) {
  while(true) {
    for(int i = 0; i< 100; i++) {
      data[i] = i;
    }

    // Init GPIO
    GLB_GPIO_Type pins[2];
    pins[0] = static_cast<GLB_GPIO_Type>(18);
    pins[1] = static_cast<GLB_GPIO_Type>(10);

    GLB_GPIO_Func_Init(
            GPIO_FUN_SWGPIO,  //  Configure the pins as GPIO
            pins,             //  Pins to be configured
            sizeof(pins) / sizeof(pins[0])  //  4 pins
            );

    gpio_set_mode(10, GPIO_INPUT_MODE);
    gpio_set_mode(18, GPIO_OUTPUT_PP_MODE);

    //SPI
    Pinetime::Drivers::SpiMaster spi{
      Pinetime::Drivers::SpiMaster::SpiModule::SPI0,
      {Pinetime::Drivers::SpiMaster::BitOrder::Msb_Lsb,
       Pinetime::Drivers::SpiMaster::Modes::Mode3,
       Pinetime::Drivers::SpiMaster::Frequencies::Freq8Mhz, 0, 0, 0}};

    MSG("SPI\r\n");
    Pinetime::Drivers::Spi loraSpi{spi, 0};
    spi.Init();
    loraSpi.Init();

    MSG("Radio\r\n");
    SX126x radio(loraSpi);
    radio.Init();
    MSG("Radio Reset\r\n");
    radio.Reset();
    MSG("Radio GetStatus\r\n");
    auto status = radio.GetStatus();
    printf("Status : %d\r\n", status);

    MSG("Radio SetDeviceType\r\n");
    radio.SetDeviceType(SX126x::DeviceType_t::SX1262);
    auto error = radio.GetDeviceErrors();
    printf("Error : %d\r\n", error);

    radio.SetStandby(SX126x::STDBY_RC);
    //radio.SetDio3AsTcxoCtrl(SX126x::RadioTcxoCtrlVoltage_t::TCXO_CTRL_3_3V, 5 << 6);

    SX126x::CalibrationParams_t calibrationParams;
    calibrationParams.Fields.ADCBulkNEnable = 1;
    calibrationParams.Fields.ADCBulkPEnable = 1;
    calibrationParams.Fields.ImgEnable = 1;
    calibrationParams.Fields.ADCPulseEnable = 1;
    calibrationParams.Fields.PLLEnable = 1;
    calibrationParams.Fields.RC13MEnable  = 1;
    calibrationParams.Fields.RC64KEnable = 1;
    radio.Calibrate(calibrationParams);

    //radio.SetDio2AsRfSwitchCtrl(true);


    radio.SetStandby(SX126x::STDBY_XOSC);
    radio.SetRegulatorMode(SX126x::RadioRegulatorMode_t::USE_DCDC);
    radio.SetBufferBaseAddresses(0,0);
    radio.SetTxParams(22, SX126x::RADIO_RAMP_3400_US);
    radio.SetRfFrequency(868000000);

    radio.SetStopRxTimerOnPreambleDetect(false);
    radio.SetLoRaSymbNumTimeout(0);
    radio.SetPacketType(SX126x::RadioPacketTypes_t::PACKET_TYPE_LORA);
    radio.SetDioIrqParams(0xffff, 0x0001, 0, 0);


    MSG("Radio SetModulationParams\r\n");

    SX126x::ModulationParams_t modulationParams;
    modulationParams.PacketType = SX126x::RadioPacketTypes_t::PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = SX126x::RadioLoRaSpreadingFactors_t::LORA_SF12;
    modulationParams.Params.LoRa.CodingRate =  SX126x::RadioLoRaCodingRates_t::LORA_CR_4_5;
    modulationParams.Params.LoRa.Bandwidth = SX126x::RadioLoRaBandwidths_t::LORA_BW_500;
    modulationParams.Params.LoRa.LowDatarateOptimize = false;
    radio.SetModulationParams(modulationParams);

    SX126x::PacketParams_t packetParams;
    packetParams.PacketType = SX126x::RadioPacketTypes_t::PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = 100;
    packetParams.Params.LoRa.HeaderType = SX126x::RadioLoRaPacketLengthsMode_t::LORA_PACKET_VARIABLE_LENGTH;
    packetParams.Params.LoRa.PayloadLength = 100;
    packetParams.Params.LoRa.CrcMode = SX126x::RadioLoRaCrcModes_t::LORA_CRC_OFF;
    packetParams.Params.LoRa.InvertIQ = SX126x::RadioLoRaIQModes_t::LORA_IQ_NORMAL;
    radio.SetPacketParams(packetParams);

    auto mode = radio.GetOperatingMode();
    printf("Mode : %d\r\n", mode);
    //radio.SetRx(0xffff);
    mode = radio.GetOperatingMode();
    printf("Hello world! %d\r\n", mode);


    radio.SetPacketParams(packetParams);
    MSG("Radio WriteBuffer\r\n");

    radio.WriteBuffer(0, data, 100);




    uint8_t readData[8];
    //radio.ReadBuffer(0, readData, 8);
    //for(int i = 0; i < 8; i++) {
    //  printf("0x%x ", readData[i]);
    //}
    //printf("\n");


    int c = 0;

    radio.SetTx(0xffffffff);
    vTaskDelay(10000);


    error = radio.GetDeviceErrors();
    printf("Error : %d\r\n", error);

    while(1) {


      radio.ProcessIrqs();
      auto mode = radio.GetOperatingMode();
      printf("Hello world! %d\r\n", mode);
      vTaskDelay(1000);


    }


  }
}

int main(void) {
  bflb_platform_init(0);

  xHeapRegions[0].xSizeInBytes = 4096*2;
  vPortDefineHeapRegions(xHeapRegions);

  xTaskCreateStatic(HelloWorldTask, (char *)"HelloWorld", sizeof(helloWorldStack) / 4, NULL, 16, helloWorldStack, &helloWorldHandle);

  vTaskStartScheduler();

  while (1) {
    bflb_platform_delay_ms(100);
  }
}
