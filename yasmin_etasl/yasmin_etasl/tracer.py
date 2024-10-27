# tracer.py
#
# Copyright (C) Erwin Aertbeliën,  2024
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



from .yasmin_ticking import *
from .yasmin_ticking_ros import *
from .yasmin_ticking_etasl import *

from yasmin_etasl_interfaces.msg import Trace
import rclpy.time
import rclpy

class Trace_Visitor(Visitor):
    """
    """
    def __init__(self, do_not_expand_types:List[type]=[], do_not_expand_instances:List[str]=[]):
        """
        Visitor to list all active states

        Parameters:
            do_not_expand_types:
                list of types that you don't want to expand (go in detail).
                The names correspond to `type(class).__name__` (i.e. without module)
            do_not_expand_instances
                list of instance names that you don't want to expand (go in detail)
        """        
        self.stack = [""]
        self.children = [0]
        self.do_not_expand_types = do_not_expand_types
        self.do_not_expand_instances = do_not_expand_instances        
        self.active=[]
        return 
 
    def check_expand(self,shortname,statetype):
        return shortname not in self.do_not_expand_instances and statetype not in self.do_not_expand_types

    def pre(self, state) -> bool:        
        self.stack.append(  self.stack[-1]+"."+state.name)
        self.children.append(0)
        self.children[-2] = self.children[-2] + 1
        return self.check_expand(state.name,type(state))
        
    def post(self, state):
        is_active = (state.status == TickingState_Status.DOO)            
        if  is_active and self.children[-1]==0:
            self.active.append(self.stack[-1])        
        self.children.pop()
        self.stack.pop()
        
    def get_active(self):

        return [ name[1:] for name in self.active ]


from std_msgs.msg import String

class TracePublisher(Generator):
    "Simple state to print the graphviz representation of a statemachine to a file"
    def __init__(self, name:str,topic:str, sm:TickingState, node=None,skip=1, do_not_expand_types:List[type]=[], do_not_expand_instances:List[str]=[]):
        """
        Publishes a graphviz representation on a topic.  This node runs forever.
        ( probably you want to run it in parallel with some statemachine using
         a ConcurrentFallback)

        Parameters:
            name:
                name of the node
            topic:
                topic to publish on
            sm:
                statemachine whose representation you want to publish
            node:
                node, YasminTickingNode.get_instance() if None
            skip:
                skip this amount of cycles before sending out a topic
            do_not_expand_types:
                list of typenames that you don't want to expand (go in detail).
                The names correspond to `type(class).__name__` (i.e. without module)
            do_not_expand_instances
                list of instance names that you don't want to expand (go in detail)
        """
        if node is None:
            node = YasminTickingNode.get_instance()
        self.node = node
        super().__init__("trace_publisher",[SUCCEED])
        self.sm = sm
        self.topic = topic
        self.publisher = node.create_publisher(Trace,topic,10)        
        self.skip = skip
        self.do_not_expand_types = do_not_expand_types
        self.do_not_expand_instances = do_not_expand_instances
        self.previous = []

        
    def co_execute(self,bb):
        count = 0
        while True:
            count = count +1
            if count < self.skip:                
                yield TICKING
                continue
            count = 0
            vis = Trace_Visitor(do_not_expand_instances=self.do_not_expand_instances, do_not_expand_types=self.do_not_expand_types)
            self.sm.accept(vis)    
            activestates = vis.get_active()
            if activestates != self.previous:
                msg = Trace()
                msg.stamp = self.node.get_clock().now().to_msg()
                msg.active_states = activestates
                self.publisher.publish(msg)                
                self.previous = activestates            
            yield TICKING


def run_while_tracing( sm):
    """
    small state machine that executes `sm` while publishing graphviz to a topic /gz
    """

    #do_not_expand_types doesn't work for now...
    return Concurrent("concurrent",[
            sm,
        TracePublisher("tracer","/smtrace",sm,None,skip=10,do_not_expand_types=[])
        #GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[])
])

