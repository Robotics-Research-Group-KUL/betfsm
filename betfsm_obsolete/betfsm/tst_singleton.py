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
    global default_logger
    return default_logger

def set_logger(logger):
    global default_logger
    default_logger=logger

# main
def main(args=None):
    print(args)
    rclpy.init(args=args)

    my_node = Node("my_node")

    #set_logger(my_node.get_logger())

    get_logger().debug("Hello world")
    get_logger().info("Hello world")
    get_logger().warn("Hello world")
    get_logger().error("Hello world")
    get_logger().fatal("Hello world")

    rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(main(sys.argv))
