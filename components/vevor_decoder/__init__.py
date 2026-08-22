"""Vevor 7-in-1 Weather Station decoder for ESPHome."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, remote_base, remote_receiver, sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_ILLUMINANCE,
    DEVICE_CLASS_PRECIPITATION,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WIND_SPEED,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_DEGREES,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_LUX,
    UNIT_MILLIMETER,
    UNIT_PERCENT,
)

CODEOWNERS = ["@parrel"]
DEPENDENCIES = ["remote_receiver"]
AUTO_LOAD = ["sensor", "binary_sensor"]
MULTI_CONF = True

vevor_ns = cg.esphome_ns.namespace("vevor_decoder")
VevorDecoder = vevor_ns.class_(
    "VevorDecoder",
    cg.Component,
    remote_base.RemoteReceiverDumperBase,
)

CONF_RECEIVER_ID = "receiver_id"
CONF_SENSOR_ID = "sensor_id"
CONF_BIT_PERIOD = "bit_period"
CONF_RAIN_HOLD = "rain_hold"
CONF_ILLUMINANCE_FILTER = "illuminance_filter"

CONF_TEMPERATURE = "temperature"
CONF_HUMIDITY = "humidity"
CONF_WIND_SPEED = "wind_speed"
CONF_WIND_GUST = "wind_gust"
CONF_WIND_DIRECTION = "wind_direction"
CONF_RAIN = "rain"
CONF_UV_INDEX = "uv_index"
CONF_ILLUMINANCE = "illuminance"
CONF_BATTERY_LOW = "battery_low"

# Accepts 0x1A2B as well as a plain integer; the station id is 16 bits.
SENSOR_ID_SCHEMA = cv.All(cv.hex_int, cv.Range(min=0, max=0xFFFF))

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(VevorDecoder),
        cv.GenerateID(CONF_RECEIVER_ID): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),
        cv.Optional(CONF_SENSOR_ID): SENSOR_ID_SCHEMA,
        cv.Optional(CONF_BIT_PERIOD, default="90us"): cv.All(
            cv.positive_time_period_microseconds,
            cv.Range(min=cv.TimePeriod(microseconds=10)),
        ),
        cv.Optional(CONF_RAIN_HOLD, default=True): cv.boolean,
        cv.Optional(CONF_ILLUMINANCE_FILTER, default=True): cv.boolean,
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_WIND_SPEED,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_GUST): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_WIND_SPEED,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WIND_DIRECTION): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=0,
            icon="mdi:compass-outline",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_RAIN): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_PRECIPITATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_UV_INDEX): sensor.sensor_schema(
            accuracy_decimals=0,
            icon="mdi:sun-wireless",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_LUX,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ILLUMINANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_LOW): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

SENSORS = {
    CONF_TEMPERATURE: "set_temperature_sensor",
    CONF_HUMIDITY: "set_humidity_sensor",
    CONF_WIND_SPEED: "set_wind_speed_sensor",
    CONF_WIND_GUST: "set_wind_gust_sensor",
    CONF_WIND_DIRECTION: "set_wind_direction_sensor",
    CONF_RAIN: "set_rain_sensor",
    CONF_UV_INDEX: "set_uv_index_sensor",
    CONF_ILLUMINANCE: "set_illuminance_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    receiver = await cg.get_variable(config[CONF_RECEIVER_ID])
    cg.add(var.set_receiver(receiver))

    # -1 is the "accept any station" sentinel on the C++ side.
    cg.add(var.set_sensor_id(config.get(CONF_SENSOR_ID, -1)))
    cg.add(var.set_bit_period(config[CONF_BIT_PERIOD]))
    cg.add(var.set_rain_hold(config[CONF_RAIN_HOLD]))
    cg.add(var.set_illuminance_filter(config[CONF_ILLUMINANCE_FILTER]))

    for key, setter in SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))

    if CONF_BATTERY_LOW in config:
        bin_sens = await binary_sensor.new_binary_sensor(config[CONF_BATTERY_LOW])
        cg.add(var.set_battery_low_binary_sensor(bin_sens))
