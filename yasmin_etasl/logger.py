# logger.py
#
# A bit of glue code to print to the console if ROS2 is not available 
# and use ROS2 otherwise.
#
# Copyright (C) Erwin Aertbeliën, Santiago Iregui, 2024
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

import sys
import rclpy
from rclpy.node import Node

from colorama import Fore, Back, Style

class LogPrinter:
    def debug(self,*args, **kwargs):
        print(Style.DIM    +Fore.WHITE+"DEBUG : " + str(*args) +Style.RESET_ALL)              
    def info(self,*args):
        print(Style.NORMAL +Fore.WHITE+"INFO : " + str(*args) +Style.RESET_ALL)              
    def warn(self,*args):
        print(Style.NORMAL +Fore.YELLOW+"WARN : " + str(*args) +Style.RESET_ALL)              
    def error(self,*args):
        print(Style.NORMAL +Fore.RED+"ERROR : " + str(*args) +Style.RESET_ALL)              
        pass
    def fatal(self,*args):
        print(Style.NORMAL+Fore.RED+"FATAL : " + str(*args) +Style.RESET_ALL)              
        pass

default_logger=LogPrinter()

def get_logger():
    """
    Use this to get a logger
    """
    global default_logger
    return default_logger

def set_logger(logger):
    """
    Use this to set the logger, either LogPrinter() or ROS2node.get_logger()
    """
    global default_logger
    default_logger=logger

