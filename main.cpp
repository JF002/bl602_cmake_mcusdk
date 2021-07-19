#include "sdk/bl_mcu_sdk/drivers/bl602_driver/std_drv/inc/bl602_adc.h"
#include <FreeRTOS.h>
#include <task.h>

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

static void HelloWorldTask(void *pvParameters) {
  while(true) {
    printf("Hello world!\r\n");
    vTaskDelay(1000);
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
