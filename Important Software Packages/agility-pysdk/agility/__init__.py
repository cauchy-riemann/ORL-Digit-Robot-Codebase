"""The Agility Robotics Python SDK is a collection of classes and functions
that are meant to simplify the process of interacting with the JSON API
provided by Agility Robotics robots.

Requirements:
   - Python 3.6+
   - The websockets library
   - Code written in an asynchronous style using asyncio

The SDK is distributed as a prebuilt "wheel" package. It can be installed using
pip.

"""

from .exceptions import *
from .jsonapi import *
from .monitors import *
from .simulator import *
