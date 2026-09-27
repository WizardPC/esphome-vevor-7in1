#pragma once
// Vevor 7-in-1 Weather Station decoder for ESPHome.

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/remote_receiver/remote_receiver.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace vevor_decoder {

// Augmenté de 512 à 2048 bits pour que les salves précédées de bruit radio
// (~500 impulsions) ne soient jamais tronquées avant la fin de la trame.
static const int MAX_BITS = 2048;

static const int32_t SENSOR_ID_ANY = -1;

class VevorDecoder : public Component, public remote_base::RemoteReceiverDumperBase {
 public:
  void set_receiver(remote_receiver::RemoteReceiverComponent *receiver) { this->receiver_ = receiver; }

  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { this->humidity_ = s; }
  void set_wind_speed_sensor(sensor::Sensor *s) { this->wind_speed_ = s; }
  void set_wind_gust_sensor(sensor::Sensor *s) { this->wind_gust_ = s; }
  void set_wind_direction_sensor(sensor::Sensor *s) { this->wind_dir_ = s; }
  void set_rain_sensor(sensor::Sensor *s) { this->rain_ = s; }
  void set_uv_index_sensor(sensor::Sensor *s) { this->uv_ = s; }
  void set_illuminance_sensor(sensor::Sensor *s) { this->light_ = s; }
  void set_battery_low_binary_sensor(binary_sensor::BinarySensor *s) { this->battery_ = s; }

  void set_sensor_id(int32_t sensor_id) { this->sensor_id_ = sensor_id; }
  void set_bit_period(uint32_t bit_period_us) { this->bit_period_ = bit_period_us; }
  void set_rain_hold(bool rain_hold) { this->rain_hold_ = rain_hold; }
  void set_illuminance_filter(bool enabled) { this->illuminance_filter_ = enabled; }

  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;
  bool dump(remote_base::RemoteReceiveData src) override;

 protected:
  // Convertit les durées brutes en bits NRZ en compensant un éventuel biais
  // d'asymétrie FSK (skew_us) entre les impulsions positives et négatives.
  int timings_to_bits_(const std::vector<int32_t> &raw, int skew_us = 0);
  bool extract_frame_(int bit_offset, uint8_t inv, uint8_t *out);
  void publish_frame_(const uint8_t *b);

  remote_receiver::RemoteReceiverComponent *receiver_{nullptr};

  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *humidity_{nullptr};
  sensor::Sensor *wind_speed_{nullptr};
  sensor::Sensor *wind_gust_{nullptr};
  sensor::Sensor *wind_dir_{nullptr};
  sensor::Sensor *rain_{nullptr};
  sensor::Sensor *uv_{nullptr};
  sensor::Sensor *light_{nullptr};
  binary_sensor::BinarySensor *battery_{nullptr};

  int32_t sensor_id_{SENSOR_ID_ANY};
  uint32_t bit_period_{90};
  bool rain_hold_{true};
  bool illuminance_filter_{true};

  uint8_t bits_[MAX_BITS];

  int32_t last_sensor_id_{-1};
  int32_t last_rain_ticks_{-1};
  uint8_t pending_rain_reset_count_{0};
};

}  // namespace vevor_decoder
}  // namespace esphome
