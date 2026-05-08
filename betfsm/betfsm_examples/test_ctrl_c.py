#!/usr/bin/python3
#
# Demonstrates how to install a Ctrl-C handler that adapts a location in the
# blackboard to True if Ctrl-C is pressed
#

import betfsm
import sys

def main(args):
    bb={}
    handler = betfsm.Ctrl_C_Handler(bb,"/cancelation/ctrl_c",repeated=3)


    import time
    print("Handler is replaced, try pressing ctrl-c")
    time.sleep(5)
    print(f"ctrl-c default handler, in blackbloard: {betfsm.get_path_value(bb,'/cancelation/ctrl_c')}")
    print("restoring default ctrl-c, try pressing ctrl-c")
    handler = betfsm.Ctrl_C_Handler(bb)
    time.sleep(10)

if __name__ == "__main__":
    sys.exit(main(sys.argv))
