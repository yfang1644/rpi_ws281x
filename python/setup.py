# Python wrapper for the rpi_ws281x library.
# Author: Tony DiCola (tony@tonydicola.com)
try:
    from setuptools import setup, Extension
except:
    from distutils.core import setup, Extension

setup(name              = 'rpi_ws281x',
      version           = '1.0.0',
      author            = 'Jeremy Garff',
      author_email      = 'jer@jers.net',
      description       = 'Userspace Raspberry Pi PWM library for WS281X LEDs.',
      license           = 'MIT',
      url               = 'https://github.com/jgarff/rpi_ws281x/',
      py_modules        = ['neopixel'],
      ext_modules       = [Extension('_rpi_ws281x', 
                           sources=['rpi_ws281x.i'],
                           library_dirs=['../.'],
                           libraries=['ws2811'])])
