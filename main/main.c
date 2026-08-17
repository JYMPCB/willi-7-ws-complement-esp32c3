#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "c3_i2c_actuator";

// Pines solicitados por integracion
// RGB: G=5, R=6, B=7
// BEEP: 8
// I2C esclavo: SDA=4, SCL=3
static const gpio_num_t PIN_G = GPIO_NUM_5;
static const gpio_num_t PIN_R = GPIO_NUM_6;
static const gpio_num_t PIN_B = GPIO_NUM_7;
static const gpio_num_t PIN_BEEP = GPIO_NUM_8;

static const i2c_port_t I2C_PORT = I2C_NUM_0;
static const gpio_num_t I2C_SDA = GPIO_NUM_4;
static const gpio_num_t I2C_SCL = GPIO_NUM_3;
static const uint8_t I2C_ADDR = 0x2A;

static const uint8_t CMD_RGB = 0xA1;
static const uint8_t CMD_BEEP = 0xB1;
static const uint8_t CMD_STOP = 0xB2;

static const ledc_mode_t BEEP_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t BEEP_TIMER = LEDC_TIMER_0;
static const ledc_channel_t BEEP_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_bit_t BEEP_DUTY_RES = LEDC_TIMER_8_BIT;

// En driver i2c legado (IDF 5.5.x), en modo slave al menos uno de los buffers
// debe ser >100 bytes. Solo recibimos comandos, por eso RX grande y TX=0.
static const int I2C_RX_BUF_LEN = 256;
static const int I2C_TX_BUF_LEN = 0;

static TickType_t s_beep_deadline = 0;
static bool s_beep_active = false;
static bool s_warned_g_conflict = false;

static inline uint16_t u16_from_le(uint8_t lsb, uint8_t msb)
{
	return (uint16_t)lsb | ((uint16_t)msb << 8);
}

static bool pin_conflicts_i2c(gpio_num_t pin)
{
	return (pin == I2C_SDA) || (pin == I2C_SCL);
}

static void set_led_active_low(gpio_num_t pin, bool on)
{
	if (pin_conflicts_i2c(pin)) {
		if (!s_warned_g_conflict) {
			ESP_LOGW(TAG, "GPIO %d conflict with I2C; skipping control on that LED", (int)pin);
			s_warned_g_conflict = true;
		}
		return;
	}
	gpio_set_level(pin, on ? 0 : 1);
}

static void handle_rgb(uint8_t r_on, uint8_t g_on, uint8_t b_on)
{
	// Activo LOW para mantener compatibilidad con el ejemplo Arduino.
	set_led_active_low(PIN_R, r_on != 0);
	set_led_active_low(PIN_G, g_on != 0);
	set_led_active_low(PIN_B, b_on != 0);
}

static esp_err_t stop_beep(void)
{
	esp_err_t err = ledc_set_duty(BEEP_SPEED_MODE, BEEP_CHANNEL, 0);
	if (err != ESP_OK) {
		return err;
	}
	err = ledc_update_duty(BEEP_SPEED_MODE, BEEP_CHANNEL);
	if (err == ESP_OK) {
		s_beep_active = false;
		s_beep_deadline = 0;
	}
	return err;
}

static esp_err_t start_beep(uint16_t freq_hz, uint16_t dur_ms, uint8_t duty)
{
	esp_err_t err = ledc_set_freq(BEEP_SPEED_MODE, BEEP_TIMER, freq_hz);
	if (err != ESP_OK) {
		return err;
	}

	err = ledc_set_duty(BEEP_SPEED_MODE, BEEP_CHANNEL, duty);
	if (err != ESP_OK) {
		return err;
	}

	err = ledc_update_duty(BEEP_SPEED_MODE, BEEP_CHANNEL);
	if (err != ESP_OK) {
		return err;
	}

	if (dur_ms == 0) {
		return stop_beep();
	}

	s_beep_active = true;
	s_beep_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(dur_ms);
	return ESP_OK;
}

static void process_i2c_command(const uint8_t *buf, int len)
{
	if (len <= 0) {
		return;
	}

	uint8_t cmd = buf[0];

	if (cmd == CMD_RGB) {
		if (len < 4) {
			ESP_LOGW(TAG, "CMD_RGB invalido, len=%d", len);
			return;
		}
		handle_rgb(buf[1], buf[2], buf[3]);
		return;
	}

	if (cmd == CMD_BEEP) {
		if (len < 6) {
			ESP_LOGW(TAG, "CMD_BEEP invalido, len=%d", len);
			return;
		}
		uint16_t freq_hz = u16_from_le(buf[1], buf[2]);
		uint16_t dur_ms = u16_from_le(buf[3], buf[4]);
		uint8_t duty = buf[5];
		esp_err_t err = start_beep(freq_hz, dur_ms, duty);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "No se pudo iniciar beep: %s", esp_err_to_name(err));
		}
		return;
	}

	if (cmd == CMD_STOP) {
		esp_err_t err = stop_beep();
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "No se pudo detener beep: %s", esp_err_to_name(err));
		}
		return;
	}

	ESP_LOGW(TAG, "CMD desconocido: 0x%02X len=%d", cmd, len);
}

static void i2c_rx_task(void *arg)
{
	uint8_t rx_buf[I2C_RX_BUF_LEN];

	while (true) {
		int read_len = i2c_slave_read_buffer(I2C_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));
		if (read_len > 0) {
			process_i2c_command(rx_buf, read_len);
		}

		if (s_beep_active && (xTaskGetTickCount() >= s_beep_deadline)) {
			esp_err_t err = stop_beep();
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "Error al cortar beep por timeout: %s", esp_err_to_name(err));
			}
		}
	}
}

static esp_err_t init_rgb_gpio(void)
{
	gpio_config_t io_conf = {
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	uint64_t mask = 0;
	if (!pin_conflicts_i2c(PIN_R)) {
		mask |= (1ULL << PIN_R);
	}
	if (!pin_conflicts_i2c(PIN_G)) {
		mask |= (1ULL << PIN_G);
	}
	if (!pin_conflicts_i2c(PIN_B)) {
		mask |= (1ULL << PIN_B);
	}

	io_conf.pin_bit_mask = mask;
	if (mask != 0) {
		ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio_config RGB fallo");
	}

	// Estado inicial OFF (activo LOW => nivel HIGH)
	if (!pin_conflicts_i2c(PIN_R)) {
		gpio_set_level(PIN_R, 1);
	}
	if (!pin_conflicts_i2c(PIN_G)) {
		gpio_set_level(PIN_G, 1);
	}
	if (!pin_conflicts_i2c(PIN_B)) {
		gpio_set_level(PIN_B, 1);
	}

	return ESP_OK;
}

static esp_err_t init_beep_ledc(void)
{
	ledc_timer_config_t timer_conf = {
		.speed_mode = BEEP_SPEED_MODE,
		.timer_num = BEEP_TIMER,
		.duty_resolution = BEEP_DUTY_RES,
		.freq_hz = 2000,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_conf), TAG, "ledc_timer_config fallo");

	ledc_channel_config_t ch_conf = {
		.gpio_num = PIN_BEEP,
		.speed_mode = BEEP_SPEED_MODE,
		.channel = BEEP_CHANNEL,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = BEEP_TIMER,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_conf), TAG, "ledc_channel_config fallo");
	return ESP_OK;
}

static esp_err_t init_i2c_slave(void)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_SLAVE,
		.sda_io_num = I2C_SDA,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_SCL,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.slave = {
			.slave_addr = I2C_ADDR,
			.addr_10bit_en = 0,
		},
		.clk_flags = 0,
	};

	ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &conf), TAG, "i2c_param_config fallo");
	ESP_RETURN_ON_ERROR(
		i2c_driver_install(I2C_PORT, I2C_MODE_SLAVE, I2C_RX_BUF_LEN, I2C_TX_BUF_LEN, 0),
		TAG,
		"i2c_driver_install fallo"
	);
	return ESP_OK;
}

void app_main(void)
{
	ESP_LOGI(TAG, "Init C3 I2C actuator @0x%02X", I2C_ADDR);

	ESP_ERROR_CHECK(init_rgb_gpio());
	ESP_ERROR_CHECK(init_beep_ledc());
	ESP_ERROR_CHECK(init_i2c_slave());

	xTaskCreate(i2c_rx_task, "i2c_rx_task", 4096, NULL, 10, NULL);

	if (PIN_G == I2C_SCL || PIN_G == I2C_SDA) {
		ESP_LOGW(
			TAG,
			"PIN_G=%d entra en conflicto con bus I2C (SDA=%d,SCL=%d). Cambia PIN_G o pines I2C para controlar verde.",
			(int)PIN_G,
			(int)I2C_SDA,
			(int)I2C_SCL
		);
	}
}
