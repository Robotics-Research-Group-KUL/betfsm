# Erwin Aertbelien,2024
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


class BeTFSMNode(Node):

    _instance: "BeTFSMNode" = None
    _lock: RLock = RLock()
    _executor: MultiThreadedExecutor = None
    _spin_thread: Thread = None

    @staticmethod
    def get_instance(*args,**kwargs) -> "BeTFSMNode":
        with BeTFSMNode._lock:
            if BeTFSMNode._instance == None:
                BeTFSMNode._instance = BeTFSMNode(*args,**kwargs)
            return BeTFSMNode._instance

    def __init__(self,*args,**kwargs) -> None:

        if not BeTFSMNode._instance is None:
            raise Exception("This class is a Singleton")

        # automatically generated name, if no name of the node is provided
        if len(args)<1:
            args.append( f"BeTFSM_{str(uuid.uuid4()).replace('-', '_')}_node" ) 
        super().__init__(*args,**kwargs)
            
