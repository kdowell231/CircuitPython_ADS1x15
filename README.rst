
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-CircuitPython_ADS1x15/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/CircuitPython_ADS1x15/en/latest/
    :alt: Documentation Status

.. image :: https://badges.gitter.im/adafruit/circuitpython.svg
    :target: https://gitter.im/adafruit/circuitpython?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge
    :alt: Gitter

Support for the ADS1x15 series of analog-to-digital converters. Available in 12-bit (ADS1015)
and 16-bit (ADS1115) versions.

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============

.. code-block:: python

  # import the module
  import adafruit_CircuitPython_ADS1x15
  # create the instance
  adc = adafruit_CircuitPython_ADS1x15.ADS1015()
  # raw ADC value
  adc.read_adc(0)
  # or reading in volts
  adc.read_volts(0)

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_CircuitPython_ADS1x15/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

API Reference
=============

.. toctree::
   :maxdepth: 2

   api
