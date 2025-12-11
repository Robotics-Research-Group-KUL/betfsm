#!/usr/bin/env python3

#
# Example of the generic logging mechanism
# Shows how you can connect BeTFSM's generic log functions
# to your own logging framework, such as printing to console
# or e.g. the facilities of ROS2
#

import time

from betfsm.betfsm import (
    BeTFSMRunner, Sequence,  
    Message, SUCCEED, TICKING, CANCEL, Generator, Blackboard
)

from betfsm.logger import get_logger, set_logger,LogPrinter, DummyLogPrinter


class MyLogPrinter:
    """
    Can be used with set_logger for a do-nothing logger.
    """
    def debug(self,*args):
        print(str(*args)) 
    def info(self,*args):
        print(str(*args)) 
        pass
    def warn(self,*args):
        print(str(*args)) 
        pass
    def error(self,*args):        
        print(str(*args)) 
        pass
    def fatal(self,*args):
        print(str(*args)) 
        pass



def main():

    # configure some categories:
    set_logger("default", LogPrinter())
    set_logger("my", MyLogPrinter())

    get_logger().info("--- logging to the default logger (LogPrinter) ---")
    get_logger().debug("Debugging message")
    get_logger().info("Info message")
    get_logger().warn(f"Warning message")
    get_logger().error("Error message")
    get_logger().fatal("Fatal message")

    get_logger().info("--- logging to the 'my' logger (MyLogPrinter) ---")
    get_logger("my").debug("Debugging message")
    get_logger("my").info("Info message")
    get_logger("my").warn(f"Warning message")
    get_logger("my").error("Error message")
    get_logger("my").fatal("Fatal message")


if __name__ == "__main__":
    main()


