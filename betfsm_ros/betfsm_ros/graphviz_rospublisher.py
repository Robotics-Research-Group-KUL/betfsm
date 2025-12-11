# graphviz_rospublisher.py
# part of BeTFSM
#
# Copyright (C) Erwin Aertbeliën,  2025
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



from .betfsm import *
from .betfsm_ros import *


from std_msgs.msg import String

class GraphvizPublisher(Generator):
    "Simple state to print the graphviz representation of a statemachine to a file"
    def __init__(self, name:str,topic:str, sm:TickingState, node=None,skip=10, do_not_expand_types:List[type]=[], do_not_expand_instances:List[str]=[]):
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
                node, BeTFSMNode.get_instance() if None
            skip:
                skip this amount of cycles before sending out a topic
            do_not_expand_types:
                list of typenames that you don't want to expand (go in detail).
                The names correspond to `type(class).__name__` (i.e. without module)
            do_not_expand_instances
                list of instance names that you don't want to expand (go in detail)
        """
        if node is None:
            node = BeTFSMNode.get_instance()
        self.node = node
        super().__init__("print_graphviz",[SUCCEED])
        self.sm = sm
        self.topic = topic
        self.publisher = node.create_publisher(String,topic,10)        
        self.skip = skip
        self.do_not_expand_types = do_not_expand_types
        self.do_not_expand_instances = do_not_expand_instances

        
    def co_execute(self,bb):
        count = 0
        while True:
            count = count +1
            if count < self.skip:                
                yield TICKING
                continue
            count = 0
            vis = GraphViz_Visitor(do_not_expand_instances=self.do_not_expand_instances, do_not_expand_types=self.do_not_expand_types)
            self.sm.accept(vis)    
            msg = String()
            msg.data = vis.graphviz()
            self.publisher.publish(msg)
            yield TICKING



