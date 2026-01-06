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

from colorama import Fore, Back, Style


class DummyLogPrinter:
    """
    Can be used with set_logger for a do-nothing logger.
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

class ConfigurableLogPrinter:

    DEBUG=0
    INFO=1
    WARN=2
    ERROR=3
    FATAL=4

    def __init__(self, level=DEBUG, name=""):
        self.level = level
        self.name = f"{name}: " if name else ""

    """
    Can be used with set_logger to install a logger that prints to console.
    Is the default if not overridden by a call to set_logger()
    """
    def debug(self,*args):
        if self.level <= ConfigurableLogPrinter.DEBUG:
            print(Style.DIM    +Fore.WHITE+self.name+"DEBUG : " + str(*args) +Style.RESET_ALL)              

    def info(self,*args):
        if self.level <= ConfigurableLogPrinter.INFO:
            print(Style.NORMAL +Fore.WHITE+self.name+"INFO : " + str(*args) +Style.RESET_ALL)              

    def warn(self,*args):
        if self.level <= ConfigurableLogPrinter.WARN:
            print(Style.NORMAL +Fore.YELLOW+self.name+"WARN : " + str(*args) +Style.RESET_ALL)              

    def error(self,*args):
        if self.level <= ConfigurableLogPrinter.ERROR:
            print(Style.NORMAL +Fore.RED+self.name+"ERROR : " + str(*args) +Style.RESET_ALL)              

    def fatal(self,*args):
        if self.level <= ConfigurableLogPrinter.FATAL:
            print(Style.NORMAL+Fore.RED+self.name+"FATAL : " + str(*args) +Style.RESET_ALL)              


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


def set_loggers_from_specification_string(s: str="", add_name: bool=""):
    """
    Sets the loggers using a specification string s.
    e.g.: "default:DEBUG,state:INFO"

    The add_name argument specifies whether to add the name of the category
    to the log line.

    If s is empty does nothing.
    """
    level_map = {
        "DEBUG": ConfigurableLogPrinter.DEBUG,
        "INFO":  ConfigurableLogPrinter.INFO,
        "WARN":  ConfigurableLogPrinter.WARN,
        "ERROR": ConfigurableLogPrinter.ERROR,
        "FATAL": ConfigurableLogPrinter.FATAL,
    }

    if not s or not s.strip():
        return

    pairs = s.split(",")
    for pair in pairs:
        if ":" not in pair:
            continue
            
        category, level_str = pair.split(":", 1)
        category = category.strip()
        level_str = level_str.strip().upper()

        # 3. Determine the integer level
        level_int = level_map.get(level_str, ConfigurableLogPrinter.INFO)

        # 4. Determine if we should pass the category name to the printer
        logger_name = category if add_name else ""

        # 5. Create the printer and register it
        new_logger = ConfigurableLogPrinter(level=level_int, name=logger_name)
        set_logger(category, new_logger)


