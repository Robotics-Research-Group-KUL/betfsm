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
from typing import Any
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster,TransformListener, Buffer


class BeTFSMNode(Node):
    _instance: "BeTFSMNode" = None
    _lock: RLock = RLock()
    _executor: MultiThreadedExecutor = None
    _spin_thread: Thread = None

    @staticmethod
    def get_instance(*args,**kwargs) -> "BeTFSMNode":
        """get the single instance of this node

        Returns
        -------
        BeTFSMNode
            A ROS2 node, with some additional functionalities to maintain instances of listeners and action clients,
            and transform broadcasters and listeners
        """
        with BeTFSMNode._lock:
            if BeTFSMNode._instance == None:
                BeTFSMNode._instance = BeTFSMNode(*args,**kwargs)
            return BeTFSMNode._instance

    def __init__(self,*args,**kwargs) -> None:
        if not BeTFSMNode._instance is None:
            raise Exception("This class is a Singleton")

        # automatically generated name, if no name of the node is provided
        if not args:
            node_name = f"BeTFSM_{str(uuid.uuid4()).replace('-', '_')}_node"
            super().__init__(node_name, **kwargs)
        else:
            super().__init__(*args, **kwargs)

        self.clients_created = {}
        self.actions_created = {}
        self.transformBroadcaster = None
        # start when application is starting to avoid missing transformations (what could happen when it would
        # be created on-the-fly )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self,spin_thread=True)
 
    def get_client(self, srv_type:Any, srv_name:str):
        """get a (singleton) client for the given service

        Parameters
        ----------
        srv_type : type
            type of the service
        srv_name : str 
             name of the service_

        Returns
        -------
        _type_
            _description_
        """
        if srv_name not in self.clients_created:
            self.get_logger().info(f"Creating new client for {srv_name}")
            self.clients_created[srv_name] = self.create_client(srv_type, srv_name)
        return self.clients_created[srv_name]
    
    def get_action(self, action_type, action_name):
        """get a (singleton) action client for the given action_name

        Parameters
        ----------
        action_type : type
            type of the action
        action_name : str 
            name of the action

        Returns
        -------
        ActionClient 
            action client to call action_type/action_name action. 
        """
        if action_name not in self.actions_created:
            self.get_logger().info(f"Creating new action for {action_name}")
            self.actions_created[action_name] = ActionClient(self, action_type, action_name)
        return self.actions_created[action_name]

    def get_transformBroadcaster(self):
        """get a (singleton) transform broadcaster, the first time you need it
        it is created.

        Returns
        -------
        TransformBroadcaster 
            transformbroadcaster 
        """
        if self.transformBroadcaster is None:
            self.transformBroadcaster = TransformBroadcaster(self)
        return self.transformBroadcaster

    def get_transformListenerBuffer(self):
        """get a (singleton) tf Buffer that you can query for transformations.  The first
        time it is called, a TransformListener will be created.

        Returns
        -------
        Buffer 
            TF2 Buffer
        """
        return self.tf_buffer
    
    def destroy_node(self):
        if hasattr(self, 'tf_listener') and self.tf_listener:
            try:
                # This explicitly stops the background thread executor
                self.tf_listener.executor.shutdown()
            except Exception:
                pass
        super().destroy_node()