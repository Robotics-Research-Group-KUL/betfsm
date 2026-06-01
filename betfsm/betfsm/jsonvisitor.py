# jsonvisitor.py 
#
#region  Copyright (C) Erwin Aertbeliën, 2025
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



from betfsm.betfsm import TickingState, Select_Node,get_logger
from typing import List


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



class JsonVisitor:
    def __init__(self, type_filter=None):
        """
        start: string with a name of a node to start with 
        type_filter: string with :-separated list of types
        """
        self.nodes = {}
        self.root_id = None
        self.type_filter =  parse_filter(type_filter)

    def should_descend(self, state:TickingState):
        """Return True if we should NOT descend into this node."""
        # Check type filter
        return typeid(state) not in self.type_filter

    def pre(self, state:TickingState) -> bool:
        if self.root_id is None:
            self.root_id = state.uid
            parentId = None
        else:
            parentId = state.parent.uid # state.parent always exists, since state isn't global root

        node = {
            "id": state.uid,
            "name": state.name,
            "type": state.typename,
            "fqn" : state.fqn,
            "status": state.status.name,
            "available_outcomes": list(state.get_outcomes()),
            "children": [],      # filled in later
            "parentId": parentId,
            "collapsible": len(state.children())>0
        }
        self.nodes[state.uid] = node
        if self.should_descend(state):
            node["children"] = [s.uid for s in state.children()]
            return True
        else:
            return False
    
    def post(self, state:TickingState):
        pass

    def result(self):
        return {"rootId": self.root_id, "nodes": list(self.nodes.values())}



def to_json(sm:TickingState, type_filter:str="", select_name:str=None):
    """
    Generates a json representation of a statemachine-tree. Its error
    messages are given as a node.

    Parameters:
        sm:
            TickingState that is the root of a tree of statemachines/behavior-tree nodes.
        type_filter : str, optional
            a ":"-separated list of types where the json generator should NOT descend into, by default ""
        select_name : str, optional
            start from the node with select_name (if multiple, the one closest to the root) 
    """
    if sm is None:
            node = {"id":"error_2", "name": "Tree is not set (None)", "type":"ErrorMessage", "available_outcomes":[], "children":[], "parentId":None, "collapsible": False}
            get_logger().warning("Tree is not set") 
            return {"rootId":"error_2", "nodes": [node]}
    if select_name is None:
        select_name=""
    select_name = select_name.strip()
    if (select_name !=""): 
        selector = Select_Node(select_name) 
        sm.accept(selector)
        node =  selector.get_node()
        if node is None:
            node = {"id":"error_1", "name": "select-name: name was not found", "type":"ErrorMessage", "available_outcomes":[], "children":[], "parentId":None, "collapsible": False}
            get_logger().warning("select-name:  name was not found") 
            return {"rootId":"error_1", "nodes": [node]}
    else:
        node = sm
    visitor = JsonVisitor(type_filter)
    node.accept(visitor)
    return visitor.result()
