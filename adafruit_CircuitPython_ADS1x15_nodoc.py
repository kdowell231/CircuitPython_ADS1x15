import time
from adafruit_bus_device.i2c_device import I2CDevice

ADS1x15_DEFAULT_ADDRESS        = 0x48
ADS1x15_POINTER_CONVERSION     = 0x00
ADS1x15_POINTER_CONFIG         = 0x01
ADS1x15_POINTER_LOW_THRESHOLD  = 0x02
ADS1x15_POINTER_HIGH_THRESHOLD = 0x03
ADS1x15_CONFIG_OS_SINGLE       = 0x8000
ADS1x15_CONFIG_MUX_OFFSET      = 12
ADS1x15_CONFIG_GAIN = {
    2/3: 0x0000,
    1:   0x0200,
    2:   0x0400,
    4:   0x0600,
    8:   0x0800,
    16:  0x0A00
}
ADS1x15_PGA_RANGE = {
    2/3: 6.144,
    1:   4.096,
    2:   2.048,
    4:   1.024,
    8:   0.512,
    16:  0.256
}
ADS1x15_CONFIG_MODE_CONTINUOUS  = 0x0000
ADS1x15_CONFIG_MODE_SINGLE      = 0x0100
ADS1015_CONFIG_DR = {
    128:   0x0000,
    250:   0x0020,
    490:   0x0040,
    920:   0x0060,
    1600:  0x0080,
    2400:  0x00A0,
    3300:  0x00C0
}
ADS1115_CONFIG_DR = {
    8:    0x0000,
    16:   0x0020,
    32:   0x0040,
    64:   0x0060,
    128:  0x0080,
    250:  0x00A0,
    475:  0x00C0,
    860:  0x00E0
}
ADS1x15_CONFIG_COMP_WINDOW      = 0x0010
ADS1x15_CONFIG_COMP_ACTIVE_HIGH = 0x0008
ADS1x15_CONFIG_COMP_LATCHING    = 0x0004
ADS1x15_CONFIG_COMP_QUE = {
    1: 0x0000,
    2: 0x0001,
    4: 0x0002
}
ADS1x15_CONFIG_COMP_QUE_DISABLE = 0x0003


class ADS1x15(object):
    def __init__(self, address=ADS1x15_DEFAULT_ADDRESS, i2c=None, **kwargs):
        self.buf = bytearray(3)
        if i2c is None:
            import board
            import busio
            i2c = busio.I2C(board.SCL, board.SDA)
        self.i2c_device = I2CDevice(i2c, address)

        self.bits = None

    def _data_rate_default(self):
        raise NotImplementedError('Subclasses must implement _data_rate_default!')

    def _data_rate_config(self, data_rate):
        raise NotImplementedError('Subclass must implement _data_rate_config function!')

    def _conversion_value(self, low, high):
        raise NotImplementedError('Subclass must implement _conversion_value function!')

    def _read(self, mux, gain, data_rate, mode):
        config = ADS1x15_CONFIG_OS_SINGLE
        config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET
        if gain not in ADS1x15_CONFIG_GAIN:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        config |= ADS1x15_CONFIG_GAIN[gain]
        config |= mode
        if data_rate is None:
            data_rate = self._data_rate_default()
        config |= self._data_rate_config(data_rate)
        config |= ADS1x15_CONFIG_COMP_QUE_DISABLE  # Disble comparator mode.
        self.buf[0] = ADS1x15_POINTER_CONFIG
        self.buf[1] = (config >> 8) & 0xFF
        self.buf[2] = config & 0xFF
        with self.i2c_device as i2c:
            i2c.write(self.buf)
            time.sleep(1.0/data_rate+0.0001)
            self.buf[0] = ADS1x15_POINTER_CONVERSION
            i2c.write(self.buf, end=1, stop=False)
            i2c.read_into(self.buf, start=1)
        return self._conversion_value(self.buf[2], self.buf[1])

    def _read_comparator(self, mux, gain, data_rate, mode, high_threshold,
                         low_threshold, active_low, traditional, latching,
                         num_readings):
        assert num_readings == 1 or num_readings == 2 or num_readings == 4, 'Num readings must be 1, 2, or 4!'
        self._device.writeList(ADS1x15_POINTER_HIGH_THRESHOLD, [(high_threshold >> 8) & 0xFF, high_threshold & 0xFF])
        self._device.writeList(ADS1x15_POINTER_LOW_THRESHOLD, [(low_threshold >> 8) & 0xFF, low_threshold & 0xFF])
        config = ADS1x15_CONFIG_OS_SINGLE
        config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET
        if gain not in ADS1x15_CONFIG_GAIN:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        config |= ADS1x15_CONFIG_GAIN[gain]
        config |= mode
        if data_rate is None:
            data_rate = self._data_rate_default()
        config |= self._data_rate_config(data_rate)
        if not traditional:
            config |= ADS1x15_CONFIG_COMP_WINDOW
        if not active_low:
            config |= ADS1x15_CONFIG_COMP_ACTIVE_HIGH
        if latching:
            config |= ADS1x15_CONFIG_COMP_LATCHING
        config |= ADS1x15_CONFIG_COMP_QUE[num_readings]

        self.buf[0] = ADS1x15_POINTER_CONFIG
        self.buf[1] = (config >> 8) & 0xFF
        self.buf[2] = config & 0xFF
        with self.i2c_device as i2c:
            i2c.write(self.buf)
            time.sleep(1.0/data_rate+0.0001)
            self.buf[0] = ADS1x15_POINTER_CONVERSION
            i2c.write(self.buf, end=1, stop=False)
            i2c.read_into(self.buf, start=1)
        return self._conversion_value(self.buf[2], self.buf[1])

    def read_adc(self, channel, gain=1, data_rate=None):
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        return self._read(channel + 0x04, gain, data_rate, ADS1x15_CONFIG_MODE_SINGLE)

    def read_volts(self, channel, gain=1, data_rate=None):
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        raw = self.read_adc(channel, gain, data_rate)
        volts = raw * (ADS1x15_PGA_RANGE[gain] / (2**(self.bits-1) - 1))
        return volts

    def read_adc_difference(self, differential, gain=1, data_rate=None):
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        return self._read(differential, gain, data_rate, ADS1x15_CONFIG_MODE_SINGLE)

    def read_adc_difference_volts(self, differential, gain=1, data_rate=None):
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        raw = self.read_adc_difference(differential, gain, data_rate)
        volts = raw * (ADS1x15_PGA_RANGE[gain] / (2**(self.bits) - 1))
        return volts

    def start_adc(self, channel, gain=1, data_rate=None):
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        return self._read(channel + 0x04, gain, data_rate, ADS1x15_CONFIG_MODE_CONTINUOUS)

    def start_adc_difference(self, differential, gain=1, data_rate=None):
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        return self._read(differential, gain, data_rate, ADS1x15_CONFIG_MODE_CONTINUOUS)

    def start_adc_comparator(self, channel, high_threshold, low_threshold,
                             gain=1, data_rate=None, active_low=True,
                             traditional=True, latching=False, num_readings=1):
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        return self._read_comparator(channel + 0x04, gain, data_rate,
                                     ADS1x15_CONFIG_MODE_CONTINUOUS,
                                     high_threshold, low_threshold, active_low,
                                     traditional, latching, num_readings)

    def start_adc_difference_comparator(self, differential, high_threshold, low_threshold,
                                        gain=1, data_rate=None, active_low=True,
                                        traditional=True, latching=False, num_readings=1):
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        return self._read_comparator(differential, gain, data_rate,
                                     ADS1x15_CONFIG_MODE_CONTINUOUS,
                                     high_threshold, low_threshold, active_low,
                                     traditional, latching, num_readings)

    def stop_adc(self):
        self.buf[0] = ADS1x15_POINTER_CONFIG
        self.buf[1] = 0x85
        self.buf[2] = 0x83
        with self._device as i2c:
            i2c.write(self.buf)

    def get_last_result(self):
        self.buf[0] = ADS1x15_POINTER_CONVERSION
        i2c.write(self.buf, end=1, stop=False)
        i2c.read_into(self.buf, start=1)
        return self._conversion_value(buf[2], buf[1])

# class ADS1115(ADS1x15):
#     def __init__(self, *args, **kwargs):
#         super(ADS1115, self).__init__(*args, **kwargs)
#         self.bits = 16
#
#     def _data_rate_default(self):
#         return 128
#
#     def _data_rate_config(self, data_rate):
#         if data_rate not in ADS1115_CONFIG_DR:
#             raise ValueError('Data rate must be one of: 8, 16, 32, 64, 128, 250, 475, 860')
#         return ADS1115_CONFIG_DR[data_rate]
#
#     def _conversion_value(self, low, high):
#         value = ((high & 0xFF) << 8) | (low & 0xFF)
#         if value & 0x8000 != 0:
#             value -= 1 << 16
#         return value

class ADS1015(ADS1x15):
    def __init__(self, *args, **kwargs):
        super(ADS1015, self).__init__(*args, **kwargs)
        self.bits = 12

    def _data_rate_default(self):
        return 1600

    def _data_rate_config(self, data_rate):
        if data_rate not in ADS1015_CONFIG_DR:
            raise ValueError('Data rate must be one of: 128, 250, 490, 920, 1600, 2400, 3300')
        return ADS1015_CONFIG_DR[data_rate]

    def _conversion_value(self, low, high):
        value = ((high & 0xFF) << 4) | ((low & 0xFF) >> 4)
        if value & 0x800 != 0:
            value -= 1 << 12
        return value
