/**
 * @file keypad_gtx312l.c
 * @brief Low-level GTX312L driver. See keypad_gtx312l.h for the evidence notes.
 */
#include "keypad_gtx312l.h"

#include <string.h>
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "gtx312l";

static i2c_master_dev_handle_t s_dev;

#define GTX312L_TIMEOUT_MS  100

esp_err_t gtx312l_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz)
{
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, TAG, "bus is NULL");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = GTX312L_I2C_ADDR,
        .scl_speed_hz    = scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
}

esp_err_t gtx312l_read_reg(uint8_t reg, uint8_t *val)
{
    ESP_RETURN_ON_FALSE(s_dev != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(val != NULL, ESP_ERR_INVALID_ARG, TAG, "val is NULL");

    /* Wire shape (sniffer): S B2 W <reg> | Sr B3 R <val> NAK P */
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, GTX312L_TIMEOUT_MS);
}

esp_err_t gtx312l_write_reg(uint8_t reg, uint8_t val)
{
    ESP_RETURN_ON_FALSE(s_dev != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    uint8_t frame[2] = { reg, val };
    return i2c_master_transmit(s_dev, frame, sizeof(frame), GTX312L_TIMEOUT_MS);
}

/** @brief Consecutive-registers write (one frame: reg, d0..dn) as the OEM does. */
static esp_err_t gtx312l_write_burst(uint8_t reg, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(s_dev != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(len > 0 && len <= 16, ESP_ERR_INVALID_ARG, TAG, "bad len");

    uint8_t frame[17];
    frame[0] = reg;
    memcpy(&frame[1], data, len);
    return i2c_master_transmit(s_dev, frame, len + 1, GTX312L_TIMEOUT_MS);
}

esp_err_t gtx312l_read_touch_out(uint16_t *mask12)
{
    ESP_RETURN_ON_FALSE(mask12 != NULL, ESP_ERR_INVALID_ARG, TAG, "mask12 is NULL");

    /* The OEM polls 02h..05h in one frame; we only need 02h/03h, but reading
     * the same 4 bytes keeps the wire shape identical to the OEM poll. */
    uint8_t reg = GTX312L_REG_TOUCH_OUT1;
    uint8_t buf[4];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_dev, &reg, 1, buf, sizeof(buf),
                                                    GTX312L_TIMEOUT_MS),
                        TAG, "touch read failed");
    *mask12 = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x0F) << 8);
    return ESP_OK;
}

esp_err_t gtx312l_self_test(void)
{
    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(gtx312l_read_reg(GTX312L_REG_CHIP_ID, &chip_id),
                        TAG, "CHIP_ID read failed - check wiring/pull-ups/clock");
    ESP_RETURN_ON_FALSE(chip_id == GTX312L_CHIP_ID_CTRL_OPEN, ESP_ERR_NOT_FOUND, TAG,
                        "unexpected CHIP_ID 0x%02X (expected 0x%02X, CTRL=OPEN)",
                        chip_id, GTX312L_CHIP_ID_CTRL_OPEN);

    uint8_t sens1 = 0;
    ESP_RETURN_ON_ERROR(gtx312l_read_reg(GTX312L_REG_SENSITIVITY1, &sens1),
                        TAG, "SENSITIVITY1 read failed");
    if (sens1 != 0x0F) {
        /* Not fatal: the chip keeps its registers across MCU resets. The full
         * config written afterwards brings it to a known state anyway. */
        ESP_LOGW(TAG, "SENSITIVITY1=0x%02X (reset value is 0x0F): chip was already "
                      "configured since POR, continuing", sens1);
    }

    ESP_LOGI(TAG, "self-test OK (CHIP_ID=0x%02X, SENSITIVITY1=0x%02X)", chip_id, sens1);
    return ESP_OK;
}

esp_err_t gtx312l_apply_preset(gtx312l_preset_t preset, const gtx312l_tune_t *tune)
{
    ESP_RETURN_ON_FALSE(tune != NULL, ESP_ERR_INVALID_ARG, TAG, "tune is NULL");

    if (preset == GTX312L_PRESET_ACTIVE) {
        /* OEM wake delta: one 12-byte frame 20h..2Bh, then 14h/15h fast scan. */
        uint8_t sens[12];
        memset(sens, tune->sens_active, sizeof(sens));
        ESP_RETURN_ON_ERROR(gtx312l_write_burst(GTX312L_REG_SENSITIVITY1, sens, sizeof(sens)),
                            TAG, "active sens write failed");
        uint8_t fast[2] = { 0x00, 0x01 };           /* SEN_IDLE_TIME=0, SUFFIX=1 (~11 ms) */
        ESP_RETURN_ON_ERROR(gtx312l_write_burst(GTX312L_REG_SEN_IDLE_TIME, fast, sizeof(fast)),
                            TAG, "active timing write failed");
        return ESP_OK;
    }

    /* IDLE: full going-to-sleep burst, same order as the OEM (10 frames). */
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_SOFT_RST, 0x00), TAG, "soft_rst 1");
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_SOFT_RST, 0x00), TAG, "soft_rst 2");
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_INT_MULTI, 0x01), TAG, "int pulse+multi");
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_SEN_IDLE_TIME, 0x00), TAG, "idle_time 0");
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_SLIDE_MODE, 0x01), TAG, "slide on");
    /* Undocumented parity blobs (datasheet map ends at 2Dh). */
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_UNK_66, 0x02), TAG, "unk 66h");
    const uint8_t unk40[8] = { 0x02, 0x01, 0x01, 0x00, 0x02, 0x03, 0x00, 0x02 };
    ESP_RETURN_ON_ERROR(gtx312l_write_burst(GTX312L_REG_UNK_40, unk40, sizeof(unk40)),
                        TAG, "unk 40h-47h");
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_CAL_TIME, 0x05), TAG, "cal 500ms");

    uint8_t sens[12];
    memset(sens, tune->sens_idle, sizeof(sens));
    ESP_RETURN_ON_ERROR(gtx312l_write_burst(GTX312L_REG_SENSITIVITY1, sens, sizeof(sens)),
                        TAG, "idle sens write failed");
    uint8_t slow[2] = { tune->idle_time, tune->idle_suffix };   /* OEM: 03/03 = 330 ms */
    ESP_RETURN_ON_ERROR(gtx312l_write_burst(GTX312L_REG_SEN_IDLE_TIME, slow, sizeof(slow)),
                        TAG, "idle timing write failed");
    return ESP_OK;
}

esp_err_t gtx312l_boot_sequence(const gtx312l_tune_t *tune)
{
    ESP_RETURN_ON_FALSE(tune != NULL, ESP_ERR_INVALID_ARG, TAG, "tune is NULL");

    /* Undocumented magic seen once at power-on, right after the self-test. */
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_SOFT_RST, 0x5B), TAG, "magic 5B");
    ESP_RETURN_ON_ERROR(gtx312l_write_reg(GTX312L_REG_SOFT_RST, 0x00), TAG, "magic 00");

    ESP_RETURN_ON_ERROR(gtx312l_apply_preset(GTX312L_PRESET_ACTIVE, tune), TAG, "boot active");
    ESP_RETURN_ON_ERROR(gtx312l_apply_preset(GTX312L_PRESET_IDLE, tune), TAG, "boot idle");
    return ESP_OK;
}
