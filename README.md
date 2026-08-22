# ESPHome Vevor 7-in-1 Weather Station

An ESPHome external component that decodes the 868/915 MHz FSK telegrams sent by
Vevor 7-in-1 weather stations (also sold under other brands using the same
protocol) and publishes them as native ESPHome sensors.

It requires `remote_receiver`.

> **Before you buy anything:** you need a CC1101 module for the **868 MHz**
> band *with an 868 MHz antenna* (915 MHz for US stations). The 433 MHz modules
> and antennas that dominate search results will not work — see
> [Hardware](#hardware).

## What you get

| Option | Type | Unit |
| --- | --- | --- |
| `temperature` | sensor | °C |
| `humidity` | sensor | % |
| `wind_speed` | sensor | km/h |
| `wind_gust` | sensor | km/h |
| `wind_direction` | sensor | ° |
| `rain` | sensor | mm (total increasing, wraps at 15 209.8 mm) |
| `uv_index` | sensor | index |
| `illuminance` | sensor | lx |
| `battery_low` | binary sensor | — |

All of them are optional; leave out the ones you don't want.

## Hardware

You need a receiver that hands ESPHome a **demodulated** FSK bitstream on a
GPIO — a bare 433 MHz OOK module will not work, and neither will an SX1276 in
packet mode. A CC1101 is the usual choice, driven by ESPHome's built-in
`cc1101` component:

- CC1101 module **for the 868 MHz band** (EU) — or 915 MHz if your station is a
  US model. See the warning below: the band matters, and so does the antenna.
- SPI wired to the ESP32, plus the module's data output (GDO0 or GDO2) to the
  GPIO named in `remote_receiver`

The `cc1101` component puts the radio into asynchronous 2-FSK mode, after which
it streams demodulated bits at the data pin and this component takes over.

## Configuration

```yaml
external_components:
  - source: github://parrel/esphome-vevor-7in1@main
    components: [vevor_decoder]

spi:
  clk_pin: GPIO18
  mosi_pin: GPIO22
  miso_pin: GPIO19

cc1101:
  cs_pin: GPIO15
  # 868.35 MHz is the EU band; use 915 MHz for a US station.
  frequency: 868.35MHz
  modulation_type: 2-FSK
  symbol_rate: 11111
  fsk_deviation: 70kHz
  filter_bandwidth: 100kHz

remote_receiver:
  id: rf_receiver
  # The CC1101's data output pin.
  pin: GPIO12
  filter: 65us
  idle: 2000us

vevor_decoder:
  receiver_id: rf_receiver
  # Set this once you have seen your station's id in the logs, so a
  # neighbour's station cannot feed data into your sensors.
  #sensor_id: 0x1A2B

  temperature:
    name: "Outside Temperature"
  humidity:
    name: "Outside Humidity"
  wind_speed:
    name: "Wind Speed"
  wind_gust:
    name: "Wind Gust"
  wind_direction:
    name: "Wind Direction"
  rain:
    name: "Rain Total"
  uv_index:
    name: "UV Index"
  illuminance:
    name: "Illuminance"
  battery_low:
    name: "Weather Station Battery Low"
```

Every sensor is optional; leave out the ones you do not want.

### Options

- **`receiver_id`** (*optional*, ID): the `remote_receiver` to attach to.
  Defaults to the only one in your configuration.
- **`sensor_id`** (*optional*, 0–0xFFFF): only publish frames from this station
  id. **Set this.** Without it, a neighbour's station on the same protocol will
  silently overwrite your readings. Leave it out for the first run, read the id
  out of the logs, then add it.
- **`bit_period`** (*optional*, time, default `90us`): the on-air NRZ bit
  period. Only worth touching if your receiver's timing is skewed and frames
  never validate.
- **`rain_hold`** (*optional*, boolean, default `true`): rain is a counter that
  can only climb, so a frame reporting a *lower* total is either corruption or a
  genuine counter reset. If the station id changed, it is a reset and the new
  total is taken immediately. Otherwise the previous total is held until the
  station repeats the lower one three times, which corruption will not do but a
  real reset will. Set to `false` to publish every total as received.
- **`illuminance_filter`** (*optional*, boolean, default `true`): drop
  illuminance readings that contradict the UV index in the same frame — zero lux
  with non-zero UV, or lux far above what the reported UV index allows. This is
  a heuristic; turn it off if you see legitimate readings being suppressed.
- Any of the sensor blocks above, each taking the standard ESPHome
  sensor/binary sensor options, including `filters:`.

## Multiple stations

One receiver can serve any number of stations. Declare a `vevor_decoder` block
per station, each pinned to its own `sensor_id`:

```yaml
vevor_decoder:
  - sensor_id: 0x1A2B
    temperature:
      name: "Garden Temperature"
    rain:
      name: "Garden Rain"

  - sensor_id: 0x7C41
    temperature:
      name: "Allotment Temperature"
    rain:
      name: "Allotment Rain"
```

Every block sees every frame and keeps the ones matching its own `sensor_id`,
so the stations stay fully independent — including their rain totals, which are
tracked per block.

`sensor_id` is what makes this work, and it is not optional here. Without it
each block accepts every frame, and all your stations end up writing to all your
sensors.

Two things worth knowing. Each block decodes each frame independently, so the
cost grows with the number of stations — irrelevant for a handful, worth
remembering if you have many. And since stations generally pick a new id at
power-up, replacing batteries in one of them means finding its new id and
editing the matching block.

## Finding your station id

Leave `sensor_id` out, set the logger to `DEBUG`, and watch for lines like:

```
[D][vevor_decoder]: [1A2B] T=18.4°C H=72% wind=3.2 (gust 5.6) km/h dir=214° rain=12.6mm UV=2 lux=18400
```

`1A2B` is the id to put in your config. Stations of this kind generally pick a
new id each time they power up, so expect to update this after a battery
change — check the logs again rather than assuming it survived.

## License

MIT — see [LICENSE](LICENSE).

---

Protocol reference: [rtl_433](https://github.com/merbanan/rtl_433),
`src/devices/vevor_7in1.c`.
