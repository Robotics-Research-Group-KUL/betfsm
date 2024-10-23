# Copyright (C) 2023  Miguel Ángel González Santamarta
# Adapted by Erwin Aertbelien,2024
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import uuid
from threading import Thread, RLock

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


class YasminTickingNode(Node):

    _instance: "YasminTickingNode" = None
    _lock: RLock = RLock()
    _executor: MultiThreadedExecutor = None
    _spin_thread: Thread = None

    @staticmethod
    def get_instance(*args,**kwargs) -> "YasminTickingNode":
        #print("YasmiTickingNode.get_instance called with:",*args,**kwargs)
        with YasminTickingNode._lock:
            if YasminTickingNode._instance == None:
                YasminTickingNode._instance = YasminTickingNode(*args,**kwargs)
            #print(YasminTickingNode._instance)
            return YasminTickingNode._instance

    def __init__(self,*args,**kwargs) -> None:

        if not YasminTickingNode._instance is None:
            raise Exception("This class is a Singleton")

        # automatically generated name, if no name of the node is provided
        if len(args)<1:
            args.append( f"yasmin_{str(uuid.uuid4()).replace('-', '_')}_node" ) 
        super().__init__(*args,**kwargs)
            
        # executor
        # self._executor = MultiThreadedExecutor()
        # self._executor.add_node(self)
        # self._spin_thread = Thread(target=self._executor.spin)
        # self._spin_thread.start()
