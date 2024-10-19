# graphviz_visitor.py
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
        - All names should be identifiers!
    """
    def __init__(self):
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
        self.indent = 4
        self.tab = 4
        # {' ':{self.indent}}
    def add_to_stack(self, state):
        # self.stack is never empty:
        self.childnr.append(0)
        self.childnr[-2] += 1
        self.stack.append(f"{self.stack[-1]}_{state.name}_{self.childnr[-2]}")
        return self.stack[-1], self.stack[-2]
        

    def pre(self, state) -> bool:
        if isinstance(state,StateMachineElement):
            fullname, previousname = self.add_to_stack(state.state)
            shortname = state.name            
            self.doc = self.doc+f'{" ":{self.indent}}{fullname} [label="{shortname}"]\n'
            return False
        if isinstance(state,TickingStateMachine):            
            fullname, previousname = self.add_to_stack(state)
            shortname = state.name 
            self.doc = self.doc+f'{" ":{self.indent}}{previousname} -> {fullname} [lhead="cluster_{self.clustercount}"];\n'            
            self.doc = self.doc+f"{' ':{self.indent}}subgraph cluster_{self.clustercount}" +"{\n"      
            self.indent += self.tab
            self.doc = self.doc+f'{" ":{self.indent}}{fullname}[label="{shortname}",style="invis"]\n'
            self.doc = self.doc+f'{" ":{self.indent}}label="{shortname}"\n'
            self.clustercount = self.clustercount + 1            
            return False
        else:
            fullname, previousname = self.add_to_stack(state)
            shortname = state.name
            self.doc = self.doc+f'{" ":{self.indent}}{previousname} -> {fullname};\n{" ":{self.indent}}{fullname}[label="{shortname}"]\n'            
            return True
        
    
    
    def post(self, state):
        if isinstance(state,StateMachineElement):
            self.stack.pop()
            self.childnr.pop()            
            return 
        if isinstance(state,TickingStateMachine):
            self.doc = self.doc+"    }\n"
            self.stack.pop()
            self.childnr.pop()
            self.indent -= self.tab
            pass
        else:        
            self.stack.pop()
            self.childnr.pop()

    
    def print(self):
        dotstring = self.preamble + self.doc + self.postamble
        print(dotstring)