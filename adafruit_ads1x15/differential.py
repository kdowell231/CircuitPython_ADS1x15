from adafruit_ads1x15.adafruit_ads1x15 import *

class ADS1015(ADS1x15):
    def __init__():
        pass

class ADS1115(ADS1x15):
    """ADS1115 16-bit analog to digital converter instance."""

    def __init__(self, *args, **kwargs):
        super(ADS1115, self).__init__(*args, **kwargs)
        self.bits = 16

    def _data_rate_default(self):
        # Default from datasheet page 16, config register DR bit default.
        return 128

    def _data_rate_config(self, data_rate):
        if data_rate not in ADS1115_CONFIG_DR:
            raise ValueError('Data rate must be one of: 8, 16, 32, 64, 128, 250, 475, 860')
        return ADS1115_CONFIG_DR[data_rate]

    def _conversion_value(self, low, high):
        # Convert to 16-bit signed value.
        value = ((high & 0xFF) << 8) | (low & 0xFF)
        # Check for sign bit and turn into a negative value if set.
        if value & 0x8000 != 0:
            value -= 1 << 16
        return value

    def _read_channel(self, channel):
        return self.read_adc_difference(channel)

    def _read_channel_volts(self, channel):
        return self.read_volts_difference(channel)
