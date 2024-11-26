# graphviz_visitor.py
# part of BeTFSM
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



from .betfsm import *
from .betfsm_ros import *
#from .betfsm_etasl import *



#   sm = ConcurrentSequence("parallel", children=[
#             ("task1", Sequence("my_sequence", children=[
#                         ("movinghome",eTaSL_StateMachine("MovingHome") ),
#                         ("movingup",eTaSL_StateMachine("MovingUp") ),
#                         ("movingdown",eTaSL_StateMachine("MovingDown") ),            
#                         ("movingup",eTaSL_StateMachine("MovingUp")),
#                         ("my_message",MyMessage("Hello world"))
#                       ]) 
#             ),
#             ("task2",Sequence("timer", children=[
#                         ("timer",TimedWait(Duration(seconds=3.0) ) ),
#                         ("hello",MyMessage("Timer went off!"))
#                     ])
#             )
#     ])






class GraphViz_Visitor(Visitor):
    """
    Todo:
        - All names should be identifiers! is not checked!
    """
    def __init__(self, do_not_expand_types:List[type]=[], do_not_expand_instances:List[str]=[]):
        """
        Visitor to generate a graphviz representation.

        Parameters:
            do_not_expand_types:
                list of types that you don't want to expand (go in detail).
                The names correspond to `type(class).__name__` (i.e. without module)
            do_not_expand_instances
                list of instance names that you don't want to expand (go in detail)
        """        
        self.stack = ["base"]
        self.childnr = [0]
        self.preamble = """

digraph G {
    rankdir=LR
    base[label="", shape=point];
    node [shape=rectangle style="filled,rounded" fillcolor=lightblue ];
"""
        self.postamble = """
}

"""
        self.doc = ""
        self.clustercount=0
        self.indent = 2
        self.tab = 2
        self.activecolor = "#eb5f5f" 
        self.inactivecolor = "#9395c2" 
        self.do_not_expand_types = do_not_expand_types
        self.do_not_expand_instances = do_not_expand_instances        
        return 

    def check_expand(self,shortname,statetype):
        return shortname not in self.do_not_expand_instances and statetype not in self.do_not_expand_types

    def add_to_stack(self, state):
        # self.stack is never empty:
        self.childnr.append(0)
        self.childnr[-2] += 1
        self.stack.append(f"{self.stack[-1]}_{state.name}_{self.childnr[-2]}")
        return self.stack[-1], self.stack[-2]
        

    def pre(self, state) -> bool:
        fullname, previousname = self.add_to_stack(state)        
        self.indent += self.tab
        shortname = state.name
        typename = type(state).__name__
        if state.status == TickingState_Status.DOO:
            color = self.activecolor
        else:
            color = self.inactivecolor
        indentation = f'{" ":{self.indent}}'
    
        self.doc = self.doc + indentation + f'{fullname} [shape=rectangle style="filled,rounded" fillcolor="{color}" label="{shortname}\\n<{typename}>" ];\n'
        self.doc = self.doc + indentation + f"{previousname} -> {fullname};\n"
            
        return self.check_expand(shortname,type(state))
        
    
    
    def post(self, state):
        self.stack.pop()
        self.childnr.pop()
        self.indent -= self.tab
    
    def print(self):
        dotstring = self.preamble + self.doc + self.postamble
        print(dotstring)                

    def graphviz(self):
        """
        returns a string corresponding to the Graphviz representation of the state machine that was visited
        ( using statemachine.accept(visitor) ).
        """
        return self.preamble + self.doc + self.postamble


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





class GraphViz_Visitor2(Visitor):
    """
    Todo:
        - All names should be identifiers!
    """
    def __init__(self):
        self.stack = ["base"]
        self.childnr = [0]
        self.active = [0]
        self.preamble = """

digraph G {
    rankdir=LR
    base[label="", shape=point];
    node [shape=rectangle style="filled,rounded" fillcolor=lightblue ];
"""
        self.postamble = """
}

"""
        self.doc = ""
        self.clustercount=0
        self.indent = 2
        self.tab = 2
        self.activecolor = "#eb5f5f" 
        self.inactivecolor = "#9395c2" 
        return 

    def add_to_stack(self, state):
        # self.stack is never empty:
        self.childnr.append(0)
        self.childnr[-2] += 1
        self.stack.append(f"{self.stack[-1]}_{state.name}_{self.childnr[-2]}")
        if state.status==TickingState_Status.DOO:
            self.active[-1] = self.active[-1] + 1
        self.active.append(0)            
        return self.stack[-1], self.stack[-2]
        


    def generate_graph_state(self,fullname, previousname,state):
        shortname = state.name
        typename = type(state).__name__
        if state.status == TickingState_Status.DOO:
            color = self.activecolor
        else:
            color = self.inactivecolor
        indentation = f'{" ":{self.indent}}'
        
        self.doc = self.doc + indentation + f'{fullname} [shape=rectangle style="filled,rounded" fillcolor="{color}" label="{shortname}\\n<{typename}>" ];\n'
        self.doc = self.doc + indentation + f"{previousname} -> {fullname};\n"


    def pre(self, state) -> bool:
        fullname, previousname = self.add_to_stack(state)        
        self.indent += self.tab
        return True
        
    
    
    def post(self, state):
        fullname = self.stack[-1]
        previousname = self.stack[-2]
        if (self.active[-2]>0) or (state.status == TickingState_Status.DOO):
            self.generate_graph_state(fullname,previousname,state)
        self.active.pop()
        self.stack.pop()
        self.childnr.pop()
        self.indent -= self.tab
    
    def print(self):
        dotstring = self.preamble + self.doc + self.postamble
        print(dotstring)                

    def graphviz(self):
        return self.preamble + self.doc + self.postamble
