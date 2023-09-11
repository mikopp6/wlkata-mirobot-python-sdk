#!/usr/bin/env python3

import setuptools
from setuptools import find_packages

setuptools.setup(name='wlkata-mirobot-python-sdk',
                 version='0.1.15',
                 description="WKlata Mirobot Python SDK",
                 author='Miko Palojärvi',
                 author_email='miko.p777@gmail.com',
                 long_description=open("README.rst", "r", encoding="utf-8").read(),
                 long_description_content_type = 'text/markdown',
                 url="https://github.com/mikopp6/wlkata-mirobot-python-sdk",
                 packages=find_packages(exclude=["script", "example", "doc"]),
                 classifiers="""
                 Development Status :: 4 - Beta
                 Programming Language :: Python :: 3 :: Only
                 Programming Language :: Python :: 3.6
                 Programming Language :: Python :: 3.7
                 Programming Language :: Python :: 3.8
                 Programming Language :: Python :: 3.9
                 License :: OSI Approved :: MIT License
                 Operating System :: OS Independent
                 Operating System :: Microsoft :: Windows
                 Operating System :: POSIX
                 Operating System :: Unix
                 Operating System :: MacOS
                 Topic :: Scientific/Engineering
                 Topic :: Education
                 Topic :: Documentation
                 Topic :: Home Automation
                 Topic :: Scientific/Engineering :: Artificial Intelligence
                 Topic :: Scientific/Engineering :: Electronic Design Automation (EDA)
                 Topic :: Scientific/Engineering :: Image Recognition
                 Topic :: Software Development :: Embedded Systems
                 Topic :: Software Development :: Version Control :: Git
                 Topic :: Terminals :: Serial
                 Intended Audience :: Education
                 Intended Audience :: Science/Research
                 Intended Audience :: Manufacturing
                 Intended Audience :: Developers
                 """.splitlines(),
                 python_requires='>=3.6',
                 install_requires=[
                     'pyserial', 
                 ],
                 package_data={
                    'wlkata_mirobot': ['resources/*'], 
                 }
)
