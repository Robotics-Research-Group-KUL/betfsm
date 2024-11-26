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


class DummyLogPrinter:
    """
    Can be used with set_looger for a do-nothing logger.
    """
    def debug(self,*args):
        pass
    def info(self,*args):
        pass
    def warn(self,*args):
        pass
    def error(self,*args):        
        pass
    def fatal(self,*args):
        pass

class LogPrinter:
    """
    Can be used with set_logger to install a logger that prints to console.
    Is the default if not overridden by a call to set_logger()
    """
    def debug(self,*args):
        print(Style.DIM    +Fore.WHITE+"DEBUG : " + str(*args) +Style.RESET_ALL)              

    def info(self,*args):
        print(Style.NORMAL +Fore.WHITE+"INFO : " + str(*args) +Style.RESET_ALL)              

    def warn(self,*args):
        print(Style.NORMAL +Fore.YELLOW+"WARN : " + str(*args) +Style.RESET_ALL)              

    def error(self,*args):
        print(Style.NORMAL +Fore.RED+"ERROR : " + str(*args) +Style.RESET_ALL)              

    def fatal(self,*args):
        print(Style.NORMAL+Fore.RED+"FATAL : " + str(*args) +Style.RESET_ALL)              


default_loggers={"default":LogPrinter(),"unknown":DummyLogPrinter()}

def get_logger(category:str="default"):
    """
    Use this to get a logger

    Parameters:
        category: 
            arbitrary name. You can associate a specific LogPrinter to a name,
            if not specified, "default" is used.

    Note:
      Known categories used in betfsm_etasl:

        - unknown (used when the category is not known or specified)
        - default
        - state (entering/exiting states, with outcome)            
    """
    global default_loggers
    if category in default_loggers:
        return default_loggers[category]
    else:
        return default_loggers["unknown"]

def set_logger(category:str,logger):
    """
    Use this to set the logger, either LogPrinter() or ROS2node.get_logger()

    Parameters:
        category: 
            arbitrary name. You can associate a specific LogPrinter to a name,
            You probably want to specify at least the "default" logger
            (if not LogPrinter() is used, i.e. to Console)
        logger:
            Logger to associate with this category.

    
    Note:
      Known categories used in betfsm_etasl:

        - unknown (used when the category is not known or specified)
        - default
        - state (entering/exiting states, with outcome)   
      
        `default_loggers={"default":LogPrinter(),"unknown":DummyLogPrinter()}`
        
    Note:
      Loggers:

        - `ROS2 logger`, i.e. node.get_logger()
        - `Logprinter()`, i.e. prints to console
        - `DummyLogPrinter`(), i.e. does nothing

    """
    global default_loggers
    default_loggers[category]=logger

