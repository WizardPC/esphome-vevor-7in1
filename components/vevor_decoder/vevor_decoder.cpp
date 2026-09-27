#include "vevor_decoder.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome {
namespace vevor_decoder {

static const char *const TAG = "vevor_decoder";

static const int FRAME_BYTES = 21;
static const int FRAME_BITS = FRAME_BYTES * 8;  // 168

static const uint8_t SYNC_WORD[16] = {
    1, 1, 0, 0, 1, 0, 1, 0,  // 0xCA
    0, 1, 0, 1, 0, 1, 0, 0,  // 0x54
};

static const int MIN_RAW_TIMINGS = 40;
static const int MIN_DECODED_BITS = 16 + FRAME_BITS;
static const int MAX_RUN_LENGTH = 64;

static const uint8_t RAIN_RESET_CONFIRMATIONS = 3;
static const int32_t RAIN_MISSED_CARRY_TICKS = 256;
static const float LUX_PER_UV_STEP = 20000.0f;

static const int BASELINE = 257;
static const float TEMP_OFFSET = 500.0f;
static const float TEMP_SCALE = 0.1f;
static const float WIND_SCALE = 8.333f;
static const float GUST_SCALE = 1.25f;
static const float RAIN_SCALE = 0.233f;

void VevorDecoder::setup() {
  this->receiver_->register_dumper(this);
  ESP_LOGD(TAG, "Registered with remote_receiver");
}

void VevorDecoder::dump_config() {
  ESP_LOGCONFIG(TAG, "Vevor 7-in-1 Weather Station Decoder (v2):");
  if (this->sensor_id_ == SENSOR_ID_ANY) {
    ESP_LOGCONFIG(TAG, "  Station ID filter: any");
  } else {
    ESP_LOGCONFIG(TAG, "  Station ID filter: 0x%04X", (unsigned) this->sensor_id_);
  }
  ESP_LOGCONFIG(TAG, "  Bit period: %u us", (unsigned) this->bit_period_);
  ESP_LOGCONFIG(TAG, "  Rain hold on decrease: %s", YESNO(this->rain_hold_));
  ESP_LOGCONFIG(TAG, "  Illuminance plausibility filter: %s", YESNO(this->illuminance_filter_));
  LOG_SENSOR("  ", "Temperature", this->temperature_);
  LOG_SENSOR("  ", "Humidity", this->humidity_);
  LOG_SENSOR("  ", "Wind speed", this->wind_speed_);
  LOG_SENSOR("  ", "Wind gust", this->wind_gust_);
  LOG_SENSOR("  ", "Wind direction", this->wind_dir_);
  LOG_SENSOR("  ", "Rain", this->rain_);
  LOG_SENSOR("  ", "UV index", this->uv_);
  LOG_SENSOR("  ", "Illuminance", this->light_);
  LOG_BINARY_SENSOR("  ", "Battery low", this->battery_);
}

int VevorDecoder::timings_to_bits_(const std::vector<int32_t> &raw, int skew_us) {
  const int period = (int) this->bit_period_;
  const int half_period = period / 2;
  int bit_count = 0;

  for (int32_t val : raw) {
    const uint8_t bit_val = val > 0 ? 1 : 0;
    // Compense l'asymétrie FSK éventuelle du démodulateur CC1101 :
    // - Si skew_us > 0 : réduit la durée des '1' (val > 0) et augmente la durée absolue des '0' (val < 0)
    // - Si skew_us < 0 : inversement
    int duration = (val > 0) ? (val - skew_us) : (-val + skew_us);
    if (duration < 1)
      duration = 1;

    int num = (duration + half_period) / period;
    if (num < 1)
      num = 1;
    if (num > MAX_RUN_LENGTH)
      continue;

    if (bit_count + num > MAX_BITS)
      return MAX_BITS;
    for (int j = 0; j < num; j++)
      this->bits_[bit_count++] = bit_val;
  }
  return bit_count;
}

bool VevorDecoder::extract_frame_(int bit_offset, uint8_t inv, uint8_t *out) {
  for (int k = 0; k < FRAME_BYTES; k++) {
    uint8_t byte = 0;
    for (int m = 0; m < 8; m++)
      byte = (byte << 1) | (this->bits_[bit_offset + k * 8 + m] ^ inv);
    out[k] = byte;
  }

  if (out[0] != 0xAA)
    return false;

  uint16_t checksum = 0;
  for (int k = 0; k < FRAME_BYTES - 2; k++)
    checksum += out[k];
  return (checksum & 0xFF) == out[FRAME_BYTES - 2];
}

bool VevorDecoder::dump(remote_base::RemoteReceiveData src) {
  const auto &raw = src.get_raw_data();
  const int raw_size = (int) raw.size();

  if (raw_size < MIN_RAW_TIMINGS)
    return false;

  // Balayage multi-passes des biais d'asymétrie FSK courants (en microsecondes) :
  // Permet de décoder les modules CC1101 présentant un décalage de fréquence
  // qui élargit les bits '1' et raccourcit les bits '0' (ou inversement).
  static const int SKEW_CANDIDATES[] = {0, 7, -7, 14, -14, 21};

  for (int skew : SKEW_CANDIDATES) {
    const int bit_count = this->timings_to_bits_(raw, skew);
    if (bit_count < MIN_DECODED_BITS)
      continue;

    const int limit = bit_count - 16 - FRAME_BITS;
    for (int i = 0; i <= limit; i++) {
      for (uint8_t inv = 0; inv < 2; inv++) {
        bool match = true;
        for (int j = 0; j < 16; j++) {
          if (this->bits_[i + j] != (SYNC_WORD[j] ^ inv)) {
            match = false;
            break;
          }
        }
        if (!match)
          continue;

        uint8_t frame[FRAME_BYTES];
        if (!this->extract_frame_(i + 16, inv, frame))
          continue;

        const uint16_t sensor_id = (frame[2] << 8) | frame[3];
        if (this->sensor_id_ != SENSOR_ID_ANY && sensor_id != (uint16_t) this->sensor_id_) {
          ESP_LOGV(TAG, "Ignoring frame from station 0x%04X (filtered)", sensor_id);
          return false;
        }

        this->publish_frame_(frame);
        return true;
      }
    }
  }
  return false;
}

void VevorDecoder::publish_frame_(const uint8_t *b) {
  const uint16_t sensor_id = (b[2] << 8) | b[3];
  const bool battery_low = (b[4] & 0x80) != 0;

  const float temperature = (((b[5] << 8) | b[6]) - TEMP_OFFSET) * TEMP_SCALE;
  const float humidity = (float) b[7];

  // Garde-fou : rejette les trames corrompues qui passeraient le checksum 8-bit par hasard (1/256)
  if (temperature < -50.0f || temperature > 70.0f || humidity < 1.0f || humidity > 100.0f) {
    ESP_LOGW(TAG, "[%04X] Implausible T=%.1f°C or H=%.0f%%, dropping corrupted frame",
             sensor_id, temperature, humidity);
    return;
  }

  if ((int32_t) sensor_id != this->last_sensor_id_) {
    if (this->last_sensor_id_ >= 0) {
      ESP_LOGI(TAG, "Station id changed %04X -> %04X, restarting rain tracking",
               (unsigned) this->last_sensor_id_, sensor_id);
    }
    this->last_sensor_id_ = sensor_id;
    this->last_rain_ticks_ = -1;
    this->pending_rain_reset_count_ = 0;
  }

  float wind_speed = (((b[8] << 8) | b[9]) - BASELINE) / WIND_SCALE;
  if (wind_speed < 0.0f)
    wind_speed = 0.0f;
  const float wind_gust = b[10] / GUST_SCALE;

  int wind_direction = (((b[11] & 0x0F) << 8) | b[12]) - BASELINE;
  if (wind_direction < 0)
    wind_direction += 360;

  const int32_t rain_ticks = ((b[13] << 8) | b[14]) - BASELINE;
  const int32_t rain_floor = (this->rain_hold_ && this->last_rain_ticks_ > 0) ? this->last_rain_ticks_ : 0;
  float rain_mm = NAN;
  if (rain_ticks >= rain_floor) {
    rain_mm = rain_ticks * RAIN_SCALE;
    this->last_rain_ticks_ = rain_ticks;
    this->pending_rain_reset_count_ = 0;
  } else if (rain_ticks == 0) {
    this->pending_rain_reset_count_++;
    if (this->pending_rain_reset_count_ >= RAIN_RESET_CONFIRMATIONS) {
      ESP_LOGI(TAG, "[%04X] Rain counter reset confirmed over %u frames", sensor_id,
               (unsigned) this->pending_rain_reset_count_);
      rain_mm = 0.0f;
      this->last_rain_ticks_ = 0;
      this->pending_rain_reset_count_ = 0;
    } else {
      rain_mm = this->last_rain_ticks_ * RAIN_SCALE;
      ESP_LOGW(TAG, "[%04X] Rain counter reported zero (%u/%u), holding %.2f mm", sensor_id,
               (unsigned) this->pending_rain_reset_count_, (unsigned) RAIN_RESET_CONFIRMATIONS, rain_mm);
    }
  } else {
    const int32_t lost = this->last_rain_ticks_ - rain_ticks;
    const char *cause = (lost > 0 && lost % RAIN_MISSED_CARRY_TICKS == 0) ? " (missed carry in the station)" : "";
    this->pending_rain_reset_count_ = 0;
    if (this->last_rain_ticks_ >= 0) {
      rain_mm = this->last_rain_ticks_ * RAIN_SCALE;
      ESP_LOGW(TAG, "[%04X] Impossible rain count %d ticks%s, holding %.2f mm", sensor_id, (int) rain_ticks, cause,
               rain_mm);
    } else {
      ESP_LOGW(TAG, "[%04X] Impossible rain count %d ticks%s, dropping the reading", sensor_id, (int) rain_ticks,
               cause);
    }
  }

  int uv_index = (b[15] & 0x1F) - 1;
  if (uv_index < 0)
    uv_index = 0;

  const bool lux_x10 = (b[16] & 0x80) != 0;
  float illuminance = (float) ((((b[16] & 0x7F) << 8) | b[17]) - BASELINE) * (lux_x10 ? 10.0f : 1.0f);
  if (illuminance < 0.0f)
    illuminance = 0.0f;

  bool illuminance_valid = true;
  if (this->illuminance_filter_) {
    if (uv_index > 0 && illuminance == 0.0f)
      illuminance_valid = false;
    if (illuminance > (uv_index + 1) * LUX_PER_UV_STEP)
      illuminance_valid = false;
  }

  ESP_LOGD(TAG, "[%04X] T=%.1f°C H=%.0f%% wind=%.1f (gust %.1f) km/h dir=%d° rain=%.1fmm UV=%d lux=%.0f%s%s",
           sensor_id, temperature, humidity, wind_speed, wind_gust, wind_direction, rain_mm, uv_index, illuminance,
           illuminance_valid ? "" : " [lux filtered]", battery_low ? " [battery low]" : "");

  if (this->temperature_ != nullptr)
    this->temperature_->publish_state(temperature);
  if (this->humidity_ != nullptr)
    this->humidity_->publish_state(humidity);
  if (this->wind_speed_ != nullptr)
    this->wind_speed_->publish_state(wind_speed);
  if (this->wind_gust_ != nullptr)
    this->wind_gust_->publish_state(wind_gust);
  if (this->wind_dir_ != nullptr && wind_direction <= 360)
    this->wind_dir_->publish_state((float) wind_direction);
  if (this->rain_ != nullptr && !std::isnan(rain_mm))
    this->rain_->publish_state(rain_mm);
  if (this->uv_ != nullptr)
    this->uv_->publish_state((float) uv_index);
  if (this->light_ != nullptr && illuminance_valid)
    this->light_->publish_state(illuminance);
  if (this->battery_ != nullptr)
    this->battery_->publish_state(battery_low);
}

}  // namespace vevor_decoder
}  // namespace esphome
