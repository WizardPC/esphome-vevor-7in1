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
static const int32_t RAIN_MAX_STEP_TICKS = 3;  // Au-delà de +3 basculements en 20s, exige confirmation
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
  ESP_LOGCONFIG(TAG, "Vevor 7-in-1 Weather Station Decoder (v4 Anti-Collision):");
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

// Vérifie la cohérence physique de la trame AVANT de l'accepter comme valide
// (élimine les collisions 1/256 du checksum 8-bit lors d'un décalage d'un bit).
bool VevorDecoder::is_frame_plausible_(const uint8_t *b) {
  const float temperature = (((b[5] << 8) | b[6]) - TEMP_OFFSET) * TEMP_SCALE;
  const float humidity = (float) b[7];

  float wind_speed = (((b[8] << 8) | b[9]) - BASELINE) / WIND_SCALE;
  if (wind_speed < 0.0f)
    wind_speed = 0.0f;
  const float wind_gust = b[10] / GUST_SCALE;

  int wind_direction = (((b[11] & 0x0F) << 8) | b[12]) - BASELINE;
  if (wind_direction < 0)
    wind_direction += 360;

  int uv_index = (b[15] & 0x1F) - 1;
  if (uv_index < 0)
    uv_index = 0;

  // 1. Limites absolues des capteurs
  if (temperature < -40.0f || temperature > 60.0f || humidity < 5.0f || humidity > 100.0f ||
      wind_speed > 160.0f || wind_gust > 200.0f || wind_direction > 360 || uv_index > 15) {
    return false;
  }

  // 2. Détection du bit-shift sur le vent (ex: 0x0101 -> 0x0181 donnant wind=15.4 km/h avec gust=0.0 km/h)
  if ((wind_gust == 0.0f && wind_speed > 4.0f) || (wind_speed > wind_gust + 8.0f)) {
    ESP_LOGW(TAG, "Rejet bit-shift vent : wind=%.1f km/h incoherent avec gust=%.1f km/h", wind_speed, wind_gust);
    return false;
  }

  // 3. Continuité thermique et hygrométrique (empêche un saut > 4°C ou > 12% d'humidité en 20s)
  if (!std::isnan(this->last_temp_) && std::fabs(temperature - this->last_temp_) > 4.0f) {
    ESP_LOGW(TAG, "Rejet saut T° suspect : %.1f°C vs %.1f°C", temperature, this->last_temp_);
    return false;
  }
  if (!std::isnan(this->last_hum_) && std::fabs(humidity - this->last_hum_) > 12.0f) {
    ESP_LOGW(TAG, "Rejet saut Humidite suspect : %.0f%% vs %.0f%%", humidity, this->last_hum_);
    return false;
  }

  return true;
}

bool VevorDecoder::try_decode_raw_(const std::vector<int32_t> &raw, int max_skews) {
  // Paliers resserrés : évite que des paliers extrêmes (25, 28) ne fabriquent de fausses trames
  static const int SKEW_CANDIDATES[] = {0, 6, -6, 12, -12, 18};
  const int count = (max_skews < 6) ? max_skews : 6;

  for (int s = 0; s < count; s++) {
    const int bit_count = this->timings_to_bits_(raw, SKEW_CANDIDATES[s]);
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
          return false;
        }

        if (!this->is_frame_plausible_(frame)) {
          continue;
        }

        this->publish_frame_(frame);
        return true;
      }
    }
  }
  return false;
}

bool VevorDecoder::dump(remote_base::RemoteReceiveData src) {
  const auto &raw = src.get_raw_data();
  const int raw_size = (int) raw.size();

  if (raw_size < MIN_RAW_TIMINGS)
    return false;

  // 1. Décodage direct d'une trame entière (6 paliers de skew autorisés)
  if (this->try_decode_raw_(raw, 6)) {
    this->prev_fragment_.clear();
    return true;
  }

  // 2. Recollage strict des trames coupées par wind_gust = 0 km/h :
  // Uniquement -180 us (2 bits) et -90 us (1 bit), et uniquement sur les 3 premiers skews (0, +6, -6)
  // pour réduire par 10 le risque de collision de checksum 8-bit !
  if (raw_size >= 65 && raw_size <= 120) {
    if (!this->prev_fragment_.empty()) {
      static const int32_t EXTRA_ZERO_US[] = {-180, -90};
      for (int32_t extra : EXTRA_ZERO_US) {
        std::vector<int32_t> stitched = this->prev_fragment_;
        if (stitched.back() < 0) {
          stitched.back() += extra;
        } else {
          stitched.push_back(extra);
        }
        stitched.insert(stitched.end(), raw.begin(), raw.end());

        if (this->try_decode_raw_(stitched, 3)) {
          ESP_LOGI(TAG, "Trame coupee reconstruite (%d + %d impulsions, extra=%d us)",
                   (int) this->prev_fragment_.size(), raw_size, (int) extra);
          this->prev_fragment_.clear();
          return true;
        }
      }
    }
    this->prev_fragment_ = raw;
  } else {
    this->prev_fragment_.clear();
  }

  return false;
}

void VevorDecoder::publish_frame_(const uint8_t *b) {
  const uint16_t sensor_id = (b[2] << 8) | b[3];
  const bool raw_battery_low = (b[4] & 0x80) != 0;

  const float temperature = (((b[5] << 8) | b[6]) - TEMP_OFFSET) * TEMP_SCALE;
  const float humidity = (float) b[7];

  this->last_temp_ = temperature;
  this->last_hum_ = humidity;

  // Anti-rebond sur battery_low : exige 2 trames consécutives pour changer d'état
  if (raw_battery_low != this->last_battery_low_) {
    this->battery_low_confirm_++;
    if (this->battery_low_confirm_ >= 2) {
      this->last_battery_low_ = raw_battery_low;
      this->battery_low_confirm_ = 0;
    }
  } else {
    this->battery_low_confirm_ = 0;
  }
  const bool battery_low = this->last_battery_low_;

  float wind_speed = (((b[8] << 8) | b[9]) - BASELINE) / WIND_SCALE;
  if (wind_speed < 0.0f)
    wind_speed = 0.0f;
  const float wind_gust = b[10] / GUST_SCALE;

  int wind_direction = (((b[11] & 0x0F) << 8) | b[12]) - BASELINE;
  if (wind_direction < 0)
    wind_direction += 360;

  if ((int32_t) sensor_id != this->last_sensor_id_) {
    if (this->last_sensor_id_ >= 0) {
      ESP_LOGI(TAG, "Station id changed %04X -> %04X, restarting rain tracking",
               (unsigned) this->last_sensor_id_, sensor_id);
    }
    this->last_sensor_id_ = sensor_id;
    this->last_rain_ticks_ = -1;
    this->pending_rain_jump_ticks_ = -1;
    this->pending_rain_reset_count_ = 0;
  }

  const int32_t rain_ticks = ((b[13] << 8) | b[14]) - BASELINE;
  const int32_t rain_floor = (this->rain_hold_ && this->last_rain_ticks_ > 0) ? this->last_rain_ticks_ : 0;
  float rain_mm = NAN;

  if (rain_ticks >= rain_floor) {
    // Protection anti-empoisonnement du compteur de pluie :
    // Si la pluie bondit de plus de 3 ticks (> 0.7 mm en 20s) d'un seul coup,
    // on attend qu'une 2e trame confirme cette nouvelle valeur avant de verrouiller last_rain_ticks_ !
    if (this->last_rain_ticks_ >= 0 && (rain_ticks - this->last_rain_ticks_) > RAIN_MAX_STEP_TICKS) {
      if (std::abs(rain_ticks - this->pending_rain_jump_ticks_) <= 2) {
        ESP_LOGI(TAG, "[%04X] Saut de pluie confirme (%d -> %d ticks)",
                 sensor_id, (int) this->last_rain_ticks_, (int) rain_ticks);
        this->last_rain_ticks_ = rain_ticks;
        this->pending_rain_jump_ticks_ = -1;
        rain_mm = rain_ticks * RAIN_SCALE;
      } else {
        this->pending_rain_jump_ticks_ = rain_ticks;
        rain_mm = this->last_rain_ticks_ * RAIN_SCALE;
        ESP_LOGW(TAG, "[%04X] Saut de pluie suspect (%d -> %d ticks) mis en attente de confirmation, maintien de %.1f mm",
                 sensor_id, (int) this->last_rain_ticks_, (int) rain_ticks, rain_mm);
      }
    } else {
      this->last_rain_ticks_ = rain_ticks;
      this->pending_rain_jump_ticks_ = -1;
      rain_mm = rain_ticks * RAIN_SCALE;
    }
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
