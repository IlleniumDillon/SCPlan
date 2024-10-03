#include "apps.h"

// Mutexes  
osMutexId_t mtx_batVoltageHandle;
const osMutexAttr_t mtx_batVoltage_attributes = {
  .name = "mtx_bat"
};

osMutexId_t mtx_imuRotationsHandle;
const osMutexAttr_t mtx_imuRotations_attributes = {
  .name = "mtx_imu"
};

osMutexId_t mtx_wheelSpeedsHandle;
const osMutexAttr_t mtx_wheelSpeeds_attributes = {
  .name = "mtx_wheel"
};

osMutexId_t mtx_armAnglesHandle;
const osMutexAttr_t mtx_armAngles_attributes = {
  .name = "mtx_arm"
};

// Semaphores
//osSemaphoreId_t sema_powerGoodHandle;
//const osSemaphoreAttr_t sema_powerGood_attributes = {
//  .name = "sema_power"
//};
// Timers

// Queues
osMessageQueueId_t queue_comInHandle;
const osMessageQueueAttr_t queue_comIn_attributes = {
  .name = "que_com"
};
osMessageQueueId_t queue_armTargetHandle;
const osMessageQueueAttr_t queue_armTarget_attributes = {
  .name = "que_arm"
};
// Threads
osThreadId_t thread_batMonitorHandle;
const osThreadAttr_t thread_batMonitor_attributes = {
  .name = "thd_bat",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
osThreadId_t thread_communicateHandle;
const osThreadAttr_t thread_communicate_attributes = {
  .name = "thd_com",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
osThreadId_t thread_ctrlHandle;
const osThreadAttr_t thread_ctrl_attributes = {
  .name = "thd_ctrl",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 1024 * 4
};
osThreadId_t thread_imuFXHandle;
const osThreadAttr_t thread_imuFX_attributes = {
  .name = "thd_imu",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 2048 * 4
};
osThreadId_t thread_uiHandle;
const osThreadAttr_t thread_ui_attributes = {
  .name = "thd_ui",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1024 * 4
};
osThreadId_t thread_armHandle;
const osThreadAttr_t thread_atm_attributes = {
  .name = "thd_atm",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
// Events


void board_init(void)
{
	ssd1306_advance_init(SSD1306_INTERFACE_SPI, SSD1306_ADDR_SA0_0);
	ssd1306_advance_display_on();
	ssd1306_advance_clear();

	HAL_ADC_Start(&hadc1);

	em_init(&leftWheel);
	em_init(&rightWheel);

	pwmservo_init(&tripod_handle);

	busservo_bus_init(&arm_handle);

	emag_disable();

	buzzer_init();

	ssd1306_advance_string(0, 0, "init: IMU      ", 16, 1, SSD1306_FONT_12);

	icm20948_driver_init();
	// icm20948_run_selftest();

	ssd1306_advance_string(0, 0, "init: IMU  [ok]", 16, 1, SSD1306_FONT_12);

	// buzzer_play(tone_init_done);
}

void apps_init(void)
{
    // init mutexes
    mtx_batVoltageHandle = osMutexNew(&mtx_batVoltage_attributes);
    mtx_imuRotationsHandle = osMutexNew(&mtx_imuRotations_attributes);
    mtx_wheelSpeedsHandle = osMutexNew(&mtx_wheelSpeeds_attributes);
    mtx_armAnglesHandle = osMutexNew(&mtx_armAngles_attributes);
    // init semaphores
    // sema_powerGoodHandle = osSemaphoreNew(1, 0, &sema_powerGood_attributes);
    // init timers
    // init queues
    /// TODO: define data type for queue_comInHandle
    queue_comInHandle = osMessageQueueNew(20, sizeof(DataIn), &queue_comIn_attributes);
    queue_armTargetHandle = osMessageQueueNew(20, sizeof(ArmTarget), &queue_armTarget_attributes);
}

void apps_start(void)
{
    // start threads
    thread_batMonitorHandle = osThreadNew(app_batMonitor, NULL, &thread_batMonitor_attributes);
    thread_communicateHandle = osThreadNew(app_communicate, NULL, &thread_communicate_attributes);
    thread_ctrlHandle = osThreadNew(app_ctrl, NULL, &thread_ctrl_attributes);
    thread_imuFXHandle = osThreadNew(app_imuFX, NULL, &thread_imuFX_attributes);
    thread_uiHandle = osThreadNew(app_ui, NULL, &thread_ui_attributes);
    thread_armHandle = osThreadNew(app_arm, NULL, &thread_atm_attributes);
}
