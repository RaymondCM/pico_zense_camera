# This message is for storing data from the odroid-weather-board2 https://wiki.odroid.com/accessory/sensor/weather-board
# uv_index is measured in the standard UV Index format
# visible_lux and ir_lux are in is the SI derived unit of illuminance and luminous emittance (flux per unit area)
# temperature is provided in celsius
# humidity is in percent of how much of the moisture capacity of the air is used
# pressure is provided in hectopascal (hPa, 1 hPa = 100 Pa)
# altitude is provided in meters
Header header
float32 uv_index
int32 visible_lux
int32 ir_lux
float32 temperature
float32 humidity
float32 pressure
float32 altitude