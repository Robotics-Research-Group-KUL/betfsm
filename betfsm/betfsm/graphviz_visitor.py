# graphviz_visitor.py
# part of BeTFSM
#
#region Copyright (C) Erwin Aertbeliën,  2024
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
#endregion


from .betfsm import TickingState, Visitor
from typing import List
from pathlib import Path

graphviz_prefix = """
digraph StateMachine {
    rankdir=LR;
    splines=true;
    nodesep=0.7;
    ranksep=0.4;

    edge [
        fontname="Helvetica",
        fontsize=9,
        color="#333333",
        arrowsize=0.8
    ];

    
    node [
        shape=rectangle,
        style="rounded,filled",
        fillcolor="#f7f7ff",
        color="#4a4a8a",
        fontname="Helvetica",
        fontsize=10,
        penwidth=1.2
    ];


"""
graphviz_postfix = """
}
"""


def graphviz_node(state:TickingState):
    return f"""
    "{state.uid}" [
        label=<
            <table border="0" cellborder="0" cellspacing="0" align="left">
                <tr><td><b>{state.name}</b></td></tr>
                <tr><td align="left">Type: {state.__class__.__name__}</td></tr>
            </table>
        >
    ];
    """

def graphviz_arrow_between_nodes(from_uid:str, to_uid:str):
    return f"""
    "{from_uid}" -> "{to_uid}" [];
    """

def parse_filter(filter_str: str) -> List[str]:
    """
    Parse a colon-separated string of types
    """
    if not filter_str:
        return []

    parts = [p.strip() for p in filter_str.split(":") if p.strip()]
    return parts

def id(state:TickingState):
    return state.uid.replace('-','_')

def typeid(state):
    return state.__class__.__name__ 



class GraphViz_Visitor(Visitor):
    """
    Todo:
        - All names should be identifiers! is not checked!
    """

    
    def __init__(self,  type_filter:str="",start: str = None):
        """
        Visitor to generate a graphviz representation.

        Parameters:
            type_filter
                a colon-separated list of types you do not want to go in detail (with module specification, just the type),   
            start:
                where to start with the visitor. If None, start from the base

        """        
        self.doc = ""
        self.type_filter = parse_filter(type_filter)
        if start =='':
            start = None
        self.start = start
        self.from_level=1E9
        self.depth = 0
        #print("==================================")
        #print(f"{self.start=}\t\t{self.type_filter=}")
        #print("==================================")

        return 

    def should_descend(self, state:TickingState):
        """Return True if we should NOT  descend into this node."""
        # Check type filter
        for ftype in self.type_filter:
            if typeid(state) == ftype:
                return False         
        return True

    def pre(self,state:TickingState) -> bool:
        self.depth = self.depth + 1
        #print(f"{self.depth=} '{state.name=}' '{self.start=}' {self.from_level=}")
        if state.name == self.start:
            self.from_level = self.depth
            self.doc = self.doc + graphviz_node(state)    
        if (self.depth > self.from_level) or self.start is None:
            self.doc = self.doc + graphviz_node(state)
            if state.parent is not None:
                self.doc = self.doc + graphviz_arrow_between_nodes(state.parent.uid,state.uid)                
        return self.should_descend(state)

    def post(self,state:TickingState) -> bool:
        if self.depth<= self.from_level:        
            self.from_level = 1E9
        self.depth = self.depth - 1

    
    def print(self):
        dotstring = graphviz_prefix + self.doc + graphviz_postfix
        print(dotstring)                

    def graphviz(self):
        """
        returns a string corresponding to the Graphviz representation of the state machine that was visited
        ( using statemachine.accept(visitor) ).
        """
        return graphviz_prefix + self.doc + graphviz_postfix


def to_graphviz_dotfile(filename:Path, sm:TickingState, type_filter:str="", start=None):
    """
    Prints a graphviz dot representation of a statemachine-tree to a file with
    the given filename.  Use the xdot tool to visualize or the dot tool from 
    graphviz to convert to other image formats.

    Parameters:
        filename:
            name of the file to write to
        sm:
            TickingState that is the root of a tree of statemachines/behavior-tree nodes.
    """
    viz = GraphViz_Visitor(type_filter, start)
    sm.accept(viz)
    with open(filename,"w") as f:
        print(viz.graphviz(),file=f)

