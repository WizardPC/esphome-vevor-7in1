#pragma once
// Vevor 7-in-1 Weather Station decoder for ESPHome.
//
// Hooks into remote_receiver as a dumper and decodes the FSK-demodulated
// output of a CC1101 (or similar) 868/915 MHz receiver. The decode path does
// no heap allocation.

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/remote_receiver/remote_receiver.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace vevor_decoder {

// Frames are 21 bytes (168 bits) after the sync word. Allow room for the
// preamble plus a couple of repeats so the sync search has somewhere to look.
static const int MAX_BITS = 1024; //512

// Sentinel for "accept any station id".
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

  // Only publish frames from this station id. SENSOR_ID_ANY accepts all.
  void set_sensor_id(int32_t sensor_id) { this->sensor_id_ = sensor_id; }
  // NRZ bit period of the on-air signal, in microseconds.
  void set_bit_period(uint32_t bit_period_us) { this->bit_period_ = bit_period_us; }
  // Hold the previous rain total when a frame reports a lower one.
  void set_rain_hold(bool rain_hold) { this->rain_hold_ = rain_hold; }
  // Drop illuminance readings that contradict the UV index in the same frame.
  void set_illuminance_filter(bool enabled) { this->illuminance_filter_ = enabled; }

  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;
  bool dump(remote_base::RemoteReceiveData src) override;

 protected:
  // Turns the raw mark/space timings into NRZ bits in bits_.
  // Returns the number of bits produced.
  int timings_to_bits_(const std::vector<int32_t> &raw);
  // Reads 21 bytes starting at bit offset, applying inversion, and verifies
  // the header and checksum. Returns true if the frame is sound.
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

  uint8_t bits_[MAX_BITS];  // fixed array, no heap allocation

  // Rain is a monotonic tick counter: it climbs, or it restarts at exactly zero
  // after a battery pull. Any other decrease is corruption - typically the
  // station sampling its own 16-bit counter mid-carry, which reports 256 ticks
  // too few with a perfectly valid checksum - and is never accepted, however
  // often it repeats. A drop to zero is the one ambiguous case, so it is held
  // back until the station has repeated it, which corruption will not do.
  // Counts are kept as raw ticks so the comparisons are exact integer ones.
  // -1 = nothing seen yet.
  //
  // A station that has been power-cycled usually announces itself with a new
  // id, which resolves the ambiguity immediately: the rain counter behind a
  // new id has nothing to do with the one we were tracking. That is only a
  // shortcut, not a guarantee, so the repeat check above still backs it up.
  int32_t last_sensor_id_{-1};
  int32_t last_rain_ticks_{-1};
  uint8_t pending_rain_reset_count_{0};
};

}  // namespace vevor_decoder
}  // namespace esphome
