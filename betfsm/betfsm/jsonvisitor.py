# jsonvisitor.py 
#
# Copyright (C) Erwin Aertbeliën, 2025
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



from betfsm.betfsm import TickingState, GeneratorWithList,GeneratorWithState
import re
from typing import List

import re
from betfsm.betfsm import TickingState, GeneratorWithList, GeneratorWithState,TickingStateMachine

def parse_filter(filter_str: str) -> List[re.Pattern]:
    """
    Parse a colon-separated string of regex patterns.
    Example: "Debug.*,InternalNode,Temp.*"
    """
    if not filter_str:
        return []

    parts = [p.strip() for p in filter_str.split(":") if p.strip()]
    return parts



# return [re.compile(p) for p in parts]
# def _compile_name_filters(self, filters):
#     if not filters:
#         return []
#     compiled = []
#     for f in filters:
#         if isinstance(f, str):
#             compiled.append(re.compile(f))
#         else:
#             compiled.append(f)  # assume already a compiled regex
#     return compiled
# Check name regex filters
# for pattern in self.name_filter:
#     if pattern.search(state.name):
#         return True


class JsonVisitor:
    def __init__(self, name_filter=None, type_filter=None):
        """
        name_filter: string with :-separated list of names
        type_filter: string with :-separated list of types
        """
        self.nodes = {}
        self.root_id = None

        # Normalize filters
        self.name_filter = parse_filter(name_filter)
        self.type_filter =  parse_filter(type_filter)



    def should_descend(self, state:TickingState):
        """Return True if we should NOT descend into this node."""
        # Check type filter
        for ftype in self.type_filter:
            if state.__class__.__name__ == ftype:
                return False
        for fname in self.name_filter:
            if state.name == fname:
                return False            
        return True

    def pre(self, state:TickingState) -> bool:
        # Build node info
        node = {
            "id": state.uid,
            "name": state.name,
            "type": state.typename,
            "fqn" : state.fqn,
            "status": state.status.name,
            "available_outcomes": list(state.get_outcomes()),
            "children": [],
            "parentId": state.parent.uid if state.parent is not None else None,
            "collapsible": isinstance(state, GeneratorWithList)
        }



        self.nodes[state.uid] = node

        if state.parent is None:
            self.root_id = state.uid

        # Return False to stop descending
        if self.should_descend(state):
            node["children"] = [s.uid for s in state.children()]
            return True
        else:
            return False
        
    def post(self, state:TickingState):
        pass

    def result(self):
        return {"rootId": self.root_id, "nodes": list(self.nodes.values())}


