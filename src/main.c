/********************************************************************************
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 ********************************************************************************/
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#if defined(CONFIG_NRF_MODEM_LIB)
	#include <modem/lte_lc.h>
	#include <modem/nrf_modem_lib.h>
	#include <modem/modem_info.h>
	#include <nrf_modem.h>
#endif
#include <net/aws_iot.h>
#include <zephyr/sys/reboot.h>
#include <date_time.h>
#include <zephyr/dfu/mcuboot.h>
#include <cJSON.h>
#include <cJSON_os.h>
#include <zephyr/logging/log.h>
//
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <nrfx_timer.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nrfx_systick.h>

/********************************************************************************
 *
 ********************************************************************************/
#define TRIG 					21 // Output pin
#define ECHO 					18 // Input pin

#define RUBBISH_BIN_LID_CLOSED		0
#define RUBBISH_BIN_LID_OPENED		1


#define RUBBISH_BIN_EMPTY			0
#define RUBBISH_BIN_FULL			1
#define RUBBISH_BIN_LID_OPEN_SHORTER	0
#define RUBBISH_BIN_LID_OPEN_LONGER	1	
#define COLUMN_1					0
#define COLUMN_2					1

#define LOW 						0
#define HIGH 						1
#define OFF 						0
#define ON 							1
#define BUZZER_ON					0
#define BUZZER_OFF					1

// Output Pin Signals
#define BUZZER 						17	    /* sig pin of the buzzer */
#define LED_ONE 					2
#define LED_TWO			 			3
#define LED_THREE					4
#define LED_FOUR					5
#define BUTTON_ONE					6
#define BUTTON_TWO					7
#define SWITCH_ONE					6
#define SWITCH_TWO					7

// Input Pin Signals
#define RUBBISH_BIN_LEVEL_SENSOR	18 		/*  */
#define RUBBISH_BIN_LID_SWITCH 		19 		/*  */

#define MAX_OUTPUTS 				5
#define	OUTPUT_FLAGS				5
#define MAX_INPUTS 					6
#define INPUT_FLAGS 				6

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 				1000
#define SLEEP_TIME_S				1000
#define SLEEP_TIME_HALF_S			500
#define SLEEP_TIME_QUOTA_S			250

/* Option 1: by node label */
#define DEVICE_GPIO0 DT_NODELABEL(gpio0)

/********************************************************************************
 *
 ********************************************************************************/
bool rubbish_bin_lid_open_timer_state = 0;
static uint32_t rubbish_bin_open_counter = 0;
static uint8_t rubbish_bin_level_status = RUBBISH_BIN_EMPTY;
static uint8_t rubbish_bin_lid_status = RUBBISH_BIN_LID_CLOSED;
static uint32_t output_gpio[MAX_OUTPUTS][OUTPUT_FLAGS] = 
											{{LED_ONE, GPIO_OUTPUT_INACTIVE}, 
											{LED_TWO, GPIO_OUTPUT_INACTIVE}, 
											{LED_THREE, GPIO_OUTPUT_INACTIVE}, 
											{LED_FOUR, GPIO_OUTPUT_INACTIVE}, 
											{BUZZER, GPIO_OUTPUT_ACTIVE}};

static uint32_t input_gpio[MAX_INPUTS][INPUT_FLAGS] = 
											{{BUTTON_ONE, GPIO_PULL_UP}, 
											{BUTTON_TWO, GPIO_PULL_UP}, 
											{SWITCH_ONE, GPIO_PULL_UP}, 
											{SWITCH_TWO, GPIO_PULL_UP}, 
										  	{RUBBISH_BIN_LEVEL_SENSOR, GPIO_PULL_DOWN}, 
											{RUBBISH_BIN_LID_SWITCH, GPIO_PULL_DOWN}};
// const struct device *gpio_dev;
const struct device *gpio_dev = DEVICE_DT_GET(DEVICE_GPIO0);
struct k_timer buzzer_timer;
struct k_timer rubbish_timer;
struct k_timer rubbish_bin_lid_open_timer;
static struct gpio_callback rubbish_bin_lid_cb_data;
static struct gpio_callback rubbish_bin_level_cb_data;

/********************************************************************************
 * 
 ********************************************************************************/
LOG_MODULE_REGISTER(aws_iot_sample, CONFIG_AWS_IOT_SAMPLE_LOG_LEVEL);

BUILD_ASSERT(!IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT),
	     "This sample does not support LTE auto-init and connect");

#define APP_TOPICS_COUNT CONFIG_AWS_IOT_APP_SUBSCRIPTION_LIST_COUNT

static struct k_work_delayable shadow_update_work;
static struct k_work_delayable connect_work;
static struct k_work shadow_update_version_work;

static bool cloud_connected;

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(date_time_obtained, 0, 1);

#if defined(CONFIG_NRF_MODEM_LIB)
NRF_MODEM_LIB_ON_INIT(aws_iot_init_hook, on_modem_lib_init, NULL);

/* Initialized to value different than success (0) */
static int modem_lib_init_result = -1;

/** @brief Symbol specifying timer instance to be used. */
#define TIMER_INST_IDX 		0

/** @brief Symbol specifying time in milliseconds to wait for handler execution. */
#define TIME_TO_WAIT_MS 	500

// counter
static volatile uint32_t tCount = 0;

// count to us (micro seconds) conversion factor
// set in start_timer()
static volatile float countToUs = 1;

static float dist;

uint8_t prescaler = 0;
uint16_t comp1 = 500;

/********************************************************************************
 *
 ********************************************************************************/
void configuer_all_inputs(void);
void configuer_all_outputs(void);
bool getDistance(float* dist);

/********************************************************************************
 *
 ********************************************************************************/
static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	if(event_type == NRF_TIMER_EVENT_COMPARE0)
    {
        char * p_msg = p_context;
				// clear compare register event
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		//LOG_INF("Timer count: >%d<", tCount);
		// increment count
		tCount++;
        //LOG_INF("Timer finished. Context passed to the handler: >%s<", p_msg);
    }

	/*if(NRF_TIMER0->EVENTS_COMPARE[0] && NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk) {

		// clear compare register event
		NRF_TIMER0->EVENTS_COMPARE[0] = 0;
		LOG_INF("Timer count: >%d<", tCount);
		// increment count
		tCount++;
	}*/
}

/********************************************************************************
 * Stop Timer1
 ********************************************************************************/
void stop_timer(void)
{
	NRF_TIMER1->TASKS_STOP = 1;
}

/********************************************************************************
 * Set up and start Timer1
 ********************************************************************************/
void start_timer(void)
{
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->PRESCALER = prescaler;
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
	// set compare
	NRF_TIMER0->CC[0] = comp1;
	printf("timer tick = %f us\n", countToUs);
	// enable compare 1
	NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
	// use the shorts register to clear compare 1
	NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);
	// enable IRQ
	//NVIC_EnableIRQ(TIMER1_IRQn);
	// start timer
	NRF_TIMER0->TASKS_START = 1;
}

/********************************************************************************
 * @brief Function for application main entry.
 *
 * @return Nothing.
 ********************************************************************************/
void app_timer_init(void)
{
	uint32_t desired_ticks;
    nrfx_err_t status;
    (void)status;

    nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG;
    config.mode = NRF_TIMER_MODE_TIMER;
	config.bit_width = NRF_TIMER_BIT_WIDTH_16;
	config.frequency = NRF_TIMER_FREQ_16MHz;
	config.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
    config.p_context = "Timer Zero";

    status = nrfx_timer_init(&timer_inst, &config, timer_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

	#if defined(__ZEPHYR__)
    	#define TIMER_INST         NRFX_CONCAT_2(NRF_TIMER, TIMER_INST_IDX)
    	#define TIMER_INST_HANDLER NRFX_CONCAT_3(nrfx_timer_, TIMER_INST_IDX, _irq_handler)
    	IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(TIMER_INST), IRQ_PRIO_LOWEST, TIMER_INST_HANDLER, 0);
	#endif

    nrfx_timer_clear(&timer_inst);
    nrfx_timer_extended_compare(&timer_inst, 
								 NRF_TIMER_CC_CHANNEL0, 
								 TIME_TO_WAIT_MS,
                                 TIMER_SHORTS_COMPARE0_CLEAR_Msk, 
								 true);

    nrfx_timer_enable(&timer_inst);
    LOG_INF("Timer status: %s", nrfx_timer_is_enabled(&timer_inst) ? "enabled" : "disabled");
}

/********************************************************************************
 * @} 
 ********************************************************************************/
bool getDistance(float* dist)
{
	// set conversion factor
	countToUs = 0.0625*comp1*(1 << prescaler);
	gpio_pin_set(gpio_dev, TRIG, true);
	nrfx_systick_delay_us(20);
	gpio_pin_set(gpio_dev, TRIG, false);
	nrfx_systick_delay_us(12);
	gpio_pin_set(gpio_dev, TRIG, true);
	nrfx_systick_delay_us(20);
   
	while(!gpio_pin_get(gpio_dev, ECHO));
	// reset counter
	tCount = 0;
	// wait till Echo pin goes low
	while(gpio_pin_get(gpio_dev, ECHO));
	tCount = 10;
	float duration = countToUs*tCount;
	float distance = duration*0.017;

	if(distance < 400.0) {
		// save
		*dist = distance;
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * Define the callback function
 ********************************************************************************/
void rubbish_bin_lid_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	rubbish_bin_lid_status = RUBBISH_BIN_LID_OPENED;
	if(rubbish_bin_lid_open_timer_state == 0) {
		rubbish_bin_lid_open_timer_state = 1;
		LOG_INF("Rubbish Bin Lid Open Timer Counter started!");
		k_timer_start(&rubbish_bin_lid_open_timer, K_SECONDS(5), K_NO_WAIT);
	}
	gpio_pin_set(gpio_dev, LED_ONE, true);
	gpio_pin_set(gpio_dev, BUZZER, BUZZER_ON);
	/* start periodic timer that expires once every second */
	k_timer_start(&buzzer_timer, K_SECONDS(1), K_NO_WAIT);		
}

/********************************************************************************
 *
 ********************************************************************************/
void rubbish_bin_level_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if(rubbish_bin_lid_status == RUBBISH_BIN_LID_CLOSED) {
		rubbish_bin_level_status = RUBBISH_BIN_FULL;
		LOG_INF("Rubbish Bin is Full!");
	} else {
		rubbish_bin_level_status = RUBBISH_BIN_EMPTY;
		LOG_INF("Rubbish Bin is Empty!");
	}
	gpio_pin_set(gpio_dev, LED_TWO, true);
	/* start periodic timer that expires once every second */
	k_timer_start(&rubbish_timer, K_SECONDS(5), K_NO_WAIT);
}

/********************************************************************************
 * Define a variable of type static struct gpio_callback
 ********************************************************************************/
void rubbish_bin_lid_expiry_function(struct k_timer *timer_id)
{
	//k_timer_stop(&rubbish_bin_lid_open_timer);
	gpio_pin_set(gpio_dev, LED_ONE, false);
	gpio_pin_set(gpio_dev, BUZZER, BUZZER_OFF);
	rubbish_bin_lid_status = RUBBISH_BIN_LID_CLOSED;
}

/*
 *
 */
void rubbish_bin_lid_open_timer_expiry_function(struct k_timer *timer_id)
{
	rubbish_bin_open_counter += 5;
	if(rubbish_bin_lid_status == RUBBISH_BIN_LID_CLOSED) {
		rubbish_bin_lid_open_timer_state = 0;
		rubbish_bin_open_counter = 0;
		//k_timer_stop(&rubbish_bin_lid_open_timer);
		LOG_INF("The Rubbish Bin Lid is now closed!");
		// get HC-SR04 distance
		LOG_INF("Get Distance...\n");
		if(getDistance(&dist)) {
			// enable to print to serial port
			//printf("dist = %f cm\n", dist);
			LOG_INF("dist = %d cm\n", (uint32_t)dist);
		}
		// delay
		//k_msleep(250);
	} else {
		LOG_INF("Rubbish Bin Lid Open Counter: %d seconds", rubbish_bin_open_counter);
		k_timer_start(&rubbish_bin_lid_open_timer, K_SECONDS(5), K_NO_WAIT);
	}
}

/********************************************************************************
 * Define a variable of type static struct gpio_callback
 ********************************************************************************/
void rubbish_bin_level_expiry_function(struct k_timer *timer_id)
{
	gpio_pin_set(gpio_dev, LED_TWO, false);
}

/********************************************************************************
 *
 ********************************************************************************/
void configuer_all_outputs(void)
{
	int err;
	for (uint32_t i = 0; i < MAX_OUTPUTS; i++)
	{
		if (!device_is_ready(gpio_dev))
		{
			return;
		}

		err = gpio_pin_configure(gpio_dev, output_gpio[i][COLUMN_1], output_gpio[i][COLUMN_2]);
		if (err < 0)
		{
			return;
		}
	}
}

/********************************************************************************
 *
 ********************************************************************************/
void configuer_all_inputs(void)
{
	int err;
	for (uint32_t i = 0; i < MAX_INPUTS; i++)
	{
		if (!device_is_ready(gpio_dev))
		{
			return;
		}

		err = gpio_pin_configure(gpio_dev, input_gpio[i][COLUMN_1], GPIO_INPUT | input_gpio[i][COLUMN_2]);
		if (err < 0)
		{
			return;
		}
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
static void on_modem_lib_init(int ret, void *ctx)
{
	modem_lib_init_result = ret;
}
#endif

/********************************************************************************
 * 
 ********************************************************************************/
static int json_add_obj(cJSON *parent, const char *str, cJSON *item)
{
	cJSON_AddItemToObject(parent, str, item);

	return 0;		
}

/********************************************************************************
 * 
 ********************************************************************************/
static int json_add_str(cJSON *parent, const char *str, const char *item)
{
	cJSON *json_str;

	json_str = cJSON_CreateString(item);
	if (json_str == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_str);
}

/********************************************************************************
 * 
 ********************************************************************************/
static int json_add_number(cJSON *parent, const char *str, double item)
{
	cJSON *json_num;

	json_num = cJSON_CreateNumber(item);
	if (json_num == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_num);
}

/********************************************************************************
 * 
 ********************************************************************************/
static int shadow_update(bool version_number_include)
{
	int err;
	char *message;
	int64_t message_ts = 0;
	int16_t bat_voltage = 0;

	gpio_pin_set(gpio_dev, LED_THREE, true);
	//
	err = date_time_now(&message_ts);
	if (err) {
		LOG_ERR("date_time_now, error: %d", err);
		return err;
	}

#if defined(CONFIG_NRF_MODEM_LIB)
	/* Request battery voltage data from the modem. */
	err = modem_info_short_get(MODEM_INFO_BATTERY, &bat_voltage);
	if (err != sizeof(bat_voltage)) {
		LOG_ERR("modem_info_short_get, error: %d", err);
		return err;
	}
#endif

	cJSON *root_obj = cJSON_CreateObject();
			cJSON *state_obj = cJSON_CreateObject();
	cJSON *reported_obj = cJSON_CreateObject();

	if (root_obj == NULL || state_obj == NULL || reported_obj == NULL) {
		cJSON_Delete(root_obj);
		cJSON_Delete(state_obj);
		cJSON_Delete(reported_obj);
		err = -ENOMEM;
		return err;
	}

	if (version_number_include) {
		err = json_add_str(reported_obj, "app_version", CONFIG_APP_VERSION);
	} else {
		err = 0;
	}

	err += json_add_number(reported_obj, "batv", bat_voltage);
	err += json_add_number(reported_obj, "ts", message_ts);
	err += json_add_obj(state_obj, "reported", reported_obj);
	err += json_add_obj(root_obj, "state", state_obj);

	if (err) {
		LOG_ERR("json_add, error: %d", err);
		goto cleanup;
	}

	message = cJSON_Print(root_obj);
	if (message == NULL) {
		LOG_ERR("cJSON_Print, error: returned NULL");
		err = -ENOMEM;
		goto cleanup;
	}

	struct aws_iot_data tx_data = {
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
		.ptr = message,
		.len = strlen(message)
	};

	LOG_INF("Publishing: %s to AWS IoT broker", message);

	err = aws_iot_send(&tx_data);
	if (err) {
		LOG_ERR("aws_iot_send, error: %d", err);
	}
	//
	k_msleep(SLEEP_TIME_MS);	
	gpio_pin_set(gpio_dev, LED_THREE, false);

	cJSON_FreeString(message);

cleanup:

	cJSON_Delete(root_obj);

	return err;
}

/********************************************************************************
 * 
 ********************************************************************************/
static void connect_work_fn(struct k_work *work)
{
	int err;

	if (cloud_connected) {
		return;
	}

	err = aws_iot_connect(NULL);
	if (err) {
		LOG_ERR("aws_iot_connect, error: %d", err);
	}

	LOG_INF("Next connection retry in %d seconds", CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_work_schedule(&connect_work, K_SECONDS(CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

/********************************************************************************
 * 
 ********************************************************************************/
static void shadow_update_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected) {
		return;
	}

	err = shadow_update(false);
	if (err) {
		LOG_ERR("shadow_update, error: %d", err);
	}

	LOG_INF("Next data publication in %d seconds", CONFIG_PUBLICATION_INTERVAL_SECONDS);

	k_work_schedule(&shadow_update_work, K_SECONDS(CONFIG_PUBLICATION_INTERVAL_SECONDS));
}

/********************************************************************************
 * 
 ********************************************************************************/
static void shadow_update_version_work_fn(struct k_work *work)
{
	int err;

	err = shadow_update(true);
	if (err) {
		LOG_ERR("shadow_update, error: %d", err);
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
static void print_received_data(const char *buf, const char *topic,
				size_t topic_len)
{
	char *str = NULL;
	cJSON *root_obj = NULL;

	root_obj = cJSON_Parse(buf);
	if (root_obj == NULL) {
		LOG_ERR("cJSON Parse failure");
		return;
	}

	str = cJSON_Print(root_obj);
	if (str == NULL) {
		LOG_ERR("Failed to print JSON object");
		goto clean_exit;
	}

	LOG_INF("Data received from AWS IoT console: Topic: %.*s Message: %s",
		topic_len, topic, str);

	cJSON_FreeString(str);

clean_exit:
	cJSON_Delete(root_obj);
}

/********************************************************************************
 * 
 ********************************************************************************/
void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
	switch (evt->type) {
	case AWS_IOT_EVT_CONNECTING:
		LOG_INF("AWS_IOT_EVT_CONNECTING");
		break;
	case AWS_IOT_EVT_CONNECTED:
		LOG_INF("AWS_IOT_EVT_CONNECTED");

		cloud_connected = true;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&connect_work);

		if (evt->data.persistent_session) {
			LOG_INF("Persistent session enabled");
		}

#if defined(CONFIG_NRF_MODEM_LIB)
		/** Successfully connected to AWS IoT broker, mark image as
		 *  working to avoid reverting to the former image upon reboot.
		 */
		boot_write_img_confirmed();
#endif

		/** Send version number to AWS IoT broker to verify that the
		 *  FOTA update worked.
		 */
		k_work_submit(&shadow_update_version_work);

		/** Start sequential shadow data updates.
		 */
		k_work_schedule(&shadow_update_work,
				K_SECONDS(CONFIG_PUBLICATION_INTERVAL_SECONDS));

#if defined(CONFIG_NRF_MODEM_LIB)
		int err = lte_lc_psm_req(true);
		if (err) {
			LOG_ERR("Requesting PSM failed, error: %d", err);
		}
#endif
		break;
	case AWS_IOT_EVT_READY:
		LOG_INF("AWS_IOT_EVT_READY");
		break;
	case AWS_IOT_EVT_DISCONNECTED:
		LOG_INF("AWS_IOT_EVT_DISCONNECTED");
		cloud_connected = false;
		/* This may fail if the work item is already being processed,
		 * but in such case, the next time the work handler is executed,
		 * it will exit after checking the above flag and the work will
		 * not be scheduled again.
		 */
		(void)k_work_cancel_delayable(&shadow_update_work);
		k_work_schedule(&connect_work, K_NO_WAIT);
		break;
	case AWS_IOT_EVT_DATA_RECEIVED:
		LOG_INF("AWS_IOT_EVT_DATA_RECEIVED");
		print_received_data(evt->data.msg.ptr, evt->data.msg.topic.str,
				    evt->data.msg.topic.len);
		break;
	case AWS_IOT_EVT_PUBACK:
		LOG_INF("AWS_IOT_EVT_PUBACK, message ID: %d", evt->data.message_id);
		break;
	case AWS_IOT_EVT_FOTA_START:
		LOG_INF("AWS_IOT_EVT_FOTA_START");
		break;
	case AWS_IOT_EVT_FOTA_ERASE_PENDING:
		LOG_INF("AWS_IOT_EVT_FOTA_ERASE_PENDING");
		LOG_INF("Disconnect LTE link or reboot");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_offline();
		if (err) {
			LOG_ERR("Error disconnecting from LTE");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_ERASE_DONE:
		LOG_INF("AWS_FOTA_EVT_ERASE_DONE");
		LOG_INF("Reconnecting the LTE link");
#if defined(CONFIG_NRF_MODEM_LIB)
		err = lte_lc_connect();
		if (err) {
			LOG_ERR("Error connecting to LTE");
		}
#endif
		break;
	case AWS_IOT_EVT_FOTA_DONE:
		LOG_INF("AWS_IOT_EVT_FOTA_DONE");
		LOG_INF("FOTA done, rebooting device");
		aws_iot_disconnect();
		sys_reboot(0);
		break;
	case AWS_IOT_EVT_FOTA_DL_PROGRESS:
		LOG_INF("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)", evt->data.fota_progress);
	case AWS_IOT_EVT_ERROR:
		LOG_INF("AWS_IOT_EVT_ERROR, %d", evt->data.err);
		break;
	case AWS_IOT_EVT_FOTA_ERROR:
		LOG_INF("AWS_IOT_EVT_FOTA_ERROR");
		break;
	default:
		LOG_WRN("Unknown AWS IoT event type: %d", evt->type);
		break;
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
static void work_init(void)
{
	k_work_init_delayable(&shadow_update_work, shadow_update_work_fn);
	k_work_init_delayable(&connect_work, connect_work_fn);
	k_work_init(&shadow_update_version_work, shadow_update_version_work_fn);
}

/********************************************************************************
 * 
 ********************************************************************************/
#if defined(CONFIG_NRF_MODEM_LIB)
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		LOG_INF("Network registration status: %s",
			evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
			"Connected - home network" : "Connected - roaming");

		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_INF("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			LOG_INF("%s", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d",
			evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
static void modem_configure(void)
{
	int err;

	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already configured and LTE connected. */
	} else {
		err = lte_lc_init_and_connect_async(lte_handler);
		if (err) {
			LOG_ERR("Modem could not be configured, error: %d", err);
			return;
		}
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
static void nrf_modem_lib_dfu_handler(void)
{
	int err;

	err = modem_lib_init_result;

	switch (err) {
	case NRF_MODEM_DFU_RESULT_OK:
		LOG_INF("Modem update suceeded, reboot");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_UUID_ERROR:
	case NRF_MODEM_DFU_RESULT_AUTH_ERROR:
		LOG_INF("Modem update failed, error: %d", err);
		LOG_INF("Modem will use old firmware");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_HARDWARE_ERROR:
	case NRF_MODEM_DFU_RESULT_INTERNAL_ERROR:
		LOG_INF("Modem update malfunction, error: %d, reboot", err);
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case NRF_MODEM_DFU_RESULT_VOLTAGE_LOW:
		LOG_INF("Modem update cancelled due to low power, error: %d", err);
		LOG_INF("Please reboot once you have sufficient power for the DFU");
		break;
	default:
		break;
	}
}
#endif

/********************************************************************************
 * 
 ********************************************************************************/
static int app_topics_subscribe(void)
{
	int err;
	static char custom_topic[75] = "my-custom-topic/example";
	static char custom_topic_2[75] = "my-custom-topic/example_2";

	const struct aws_iot_topic_data topics_list[APP_TOPICS_COUNT] = {
		[0].str = custom_topic,
		[0].len = strlen(custom_topic),
		[1].str = custom_topic_2,
		[1].len = strlen(custom_topic_2)
	};

	err = aws_iot_subscription_topics_add(topics_list, ARRAY_SIZE(topics_list));
	if (err) {
		LOG_ERR("aws_iot_subscription_topics_add, error: %d", err);
	}

	return err;
}

/********************************************************************************
 * 
 ********************************************************************************/
static void date_time_event_handler(const struct date_time_evt *evt)
{
	switch (evt->type) {
	case DATE_TIME_OBTAINED_MODEM:
		/* Fall through */
	case DATE_TIME_OBTAINED_NTP:
		/* Fall through */
	case DATE_TIME_OBTAINED_EXT:
		LOG_INF("Date time obtained");
		k_sem_give(&date_time_obtained);

		/* De-register handler. At this point the sample will have
		 * date time to depend on indefinitely until a reboot occurs.
		 */
		date_time_register_handler(NULL);
		break;
	case DATE_TIME_NOT_OBTAINED:
		LOG_INF("DATE_TIME_NOT_OBTAINED");
		break;
	default:
		LOG_ERR("Unknown event: %d", evt->type);
		break;
	}
}

/********************************************************************************
 * 
 ********************************************************************************/
void main(void)
{
	int err;
	uint64_t i, j;

	nrfx_systick_init();
	LOG_INF("A Smart Rubbish Bin Collection IoT application started, version: %s", CONFIG_APP_VERSION);

	configuer_all_outputs();
	configuer_all_inputs();
	k_msleep(SLEEP_TIME_MS * 5);

	// set up HC-SR04 pins
	gpio_pin_configure(gpio_dev, TRIG, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio_dev, ECHO, GPIO_INPUT | GPIO_PULL_DOWN); 
	//
	for(i = 0; i < 3; i++)
	{
		gpio_pin_set(gpio_dev, LED_ONE, true);
		gpio_pin_set(gpio_dev, LED_TWO, true);
		gpio_pin_set(gpio_dev, LED_THREE, true);
		gpio_pin_set(gpio_dev, LED_FOUR, true);
		gpio_pin_set(gpio_dev, BUZZER, BUZZER_ON);
		k_msleep(SLEEP_TIME_MS);	
		gpio_pin_set(gpio_dev, LED_ONE, false);
		gpio_pin_set(gpio_dev, LED_TWO, false);
		gpio_pin_set(gpio_dev, LED_THREE, false);
		gpio_pin_set(gpio_dev, LED_FOUR, false);
		gpio_pin_set(gpio_dev, BUZZER, BUZZER_OFF);
		k_msleep(SLEEP_TIME_MS);
	}
	//
	 
	for(i = 0; i < 5; i++)
	{
		gpio_pin_set(gpio_dev, BUZZER, BUZZER_ON);
		gpio_pin_set(gpio_dev, TRIG, true);
		for(j = 0; j < 1000; j++)
		{nrfx_systick_delay_us(1000);}	
		gpio_pin_set(gpio_dev, BUZZER, BUZZER_OFF);
		gpio_pin_set(gpio_dev, TRIG, false);
		for(j = 0; j < 1000; j++)
		{nrfx_systick_delay_us(1000);}
	}
	
	// set up timers
	//app_timer_init();
	//
	//start_timer();
	// prints to serial port
	LOG_INF("starting...\n");

	// main loop:
	/*while(1) {
		// get HC-SR04 distance
		float dist;
		LOG_INF("Get Distance...\n");
		if(getDistance(&dist)) {
			// enable to print to serial port
			printf("dist = %f cm\n", dist);
		}
		// delay
		k_msleep(250);
	}*/

	//
	gpio_pin_set(gpio_dev, LED_THREE, true);

	/* Configure the interrupt on the reed switch's pin */
	err = gpio_pin_interrupt_configure(gpio_dev, RUBBISH_BIN_LID_SWITCH, GPIO_INT_LEVEL_INACTIVE);
	if (err < 0)
	{
		return;
	}

	/* Configure the interrupt on the distance sensor's pin */
	/*err = gpio_pin_interrupt_configure(gpio_dev, RUBBISH_BIN_LEVEL_SENSOR, GPIO_INT_LEVEL_INACTIVE);
	if (err < 0)
	{
		return;
	}*/

	/* Initialize the static struct gpio_callback variable */
	gpio_init_callback(&rubbish_bin_lid_cb_data, rubbish_bin_lid_interrupt_handler, BIT(RUBBISH_BIN_LID_SWITCH));
	/* Add the callback function by calling gpio_add_callback() */
	gpio_add_callback(gpio_dev, &rubbish_bin_lid_cb_data);

	/* Initialize the static struct gpio_callback variable */
	//gpio_init_callback(&rubbish_bin_level_cb_data, rubbish_bin_level_interrupt_handler, BIT(RUBBISH_BIN_LEVEL_SENSOR));
	/* Add the callback function by calling gpio_add_callback() */
	//gpio_add_callback(gpio_dev, &rubbish_bin_level_cb_data);

	//
	k_timer_init(&buzzer_timer, rubbish_bin_lid_expiry_function, NULL);
	//k_timer_init(&rubbish_timer, rubbish_bin_level_expiry_function, NULL);
	k_timer_init(&rubbish_bin_lid_open_timer, rubbish_bin_lid_open_timer_expiry_function, NULL);
	
	cJSON_Init();

#if defined(CONFIG_NRF_MODEM_LIB)
	nrf_modem_lib_dfu_handler();
#endif

	err = aws_iot_init(NULL, aws_iot_event_handler);
	if (err) {
		LOG_ERR("AWS IoT library could not be initialized, error: %d", err);
	}
	else
	{
		LOG_ERR("AWS IoT library initialized!");
	}

	/** Subscribe to customizable non-shadow specific topics
	 *  to AWS IoT backend.
	 */
	err = app_topics_subscribe();
	if (err) {
		LOG_ERR("Adding application specific topics failed, error: %d", err);
	}
	else
	{
		LOG_ERR("Application specific topics Added!");
	}

	work_init();
#if defined(CONFIG_NRF_MODEM_LIB)
	modem_configure();

	err = modem_info_init();
	if (err) {
		LOG_ERR("Failed initializing modem info module, error: %d", err);
	}
	else
	{
		LOG_ERR("Modem info module initialized!");
	}

	k_sem_take(&lte_connected, K_FOREVER);
#endif

	/** Trigger a date time update. The date_time API is used to timestamp data that is sent
	 *  to AWS IoT.
	 */
	date_time_update_async(date_time_event_handler);

	/* Postpone connecting to AWS IoT until date time has been obtained. */
	k_sem_take(&date_time_obtained, K_FOREVER);
	k_work_schedule(&connect_work, K_NO_WAIT);
}

	 