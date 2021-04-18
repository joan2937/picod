#!/usr/bin/env python

from setuptools import setup

with open('README.md') as f:
    long_description = f.read()

setup(name='picod',
      version='0.0.3.0',
      zip_safe=False,
      author='joan',
      author_email='joan@abyz.me.uk',
      maintainer='joan',
      maintainer_email='joan@abyz.me.uk',
      url='http://abyz.me.uk/picod/py_picod.html',
      download_url='http://abyz.me.uk/picod/picod.zip',
      description='Raspberry Pi pico module',
      long_description=long_description,
      long_description_content_type="text/markdown",
      license='unlicense.org',
      py_modules=['picod'],
      keywords=['gpio', 'i2c', 'serial', 'spi', 'pwm', 'servo'],
      classifiers=[
         "Programming Language :: Python :: 2",
         "Programming Language :: Python :: 3",
      ]
     )

