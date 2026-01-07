from betfsm.betfsm import TickingState, GeneratorWithList,GeneratorWithState
import re
from typing import List

#
#class JsonVisitor:
#    def __init__(self):
#        self.nodes = {}
#        self.root_id = None
#
#    def pre(self, state) -> bool:
#        node = {
#            "id": state.uid,
#            "name": state.name,
#            "type": type(state).__name__,
#            "status": state.status.name,
#            "available_outcomes": list(state.get_outcomes()),
#            "children": [],
#            "parentId": state.parent.uid if state.parent is not None else None,
#            "collapsible": isinstance(state, GeneratorWithList)
#        }
#        if isinstance(state, GeneratorWithList):
#            node["children"] = [s["state"].uid for s in state.states]
#
#        if isinstance(state, GeneratorWithState):
#            node["children"] = [state.state.uid]
#        self.nodes[state.uid] = node
#        if state.parent is None:
#            self.root_id = state.uid
#        return True
#
#    def post(self, state): pass
#
#    def result(self):
#        return {"rootId": self.root_id, "nodes": list(self.nodes.values())}
#

import re
from betfsm.betfsm import TickingState, GeneratorWithList, GeneratorWithState

def parse_name_filter(filter_str: str) -> List[re.Pattern]:
    """
    Parse a comma-separated string of regex patterns.
    Example: "Debug.*,InternalNode,Temp.*"
    """
    if not filter_str:
        return []

    parts = [p.strip() for p in filter_str.split(",") if p.strip()]
    return [re.compile(p) for p in parts]



class JsonVisitor:
    def __init__(self, name_filter=None, type_filter=None):
        """
        name_filter: list of strings or regex patterns (compiled or raw)
        type_filter: list of classes to stop descending into
        """
        self.nodes = {}
        self.root_id = None

        # Normalize filters
        self.name_filter = self._compile_name_filters(name_filter)
        self.type_filter = type_filter if type_filter else [] 

    def _compile_name_filters(self, filters):
        if not filters:
            return []
        compiled = []
        for f in filters:
            if isinstance(f, str):
                compiled.append(re.compile(f))
            else:
                compiled.append(f)  # assume already a compiled regex
        return compiled

    def _should_skip_descend(self, state):
        """Return True if we should NOT descend into this node."""
        # Check type filter
        for ftype in self.type_filter:
            if isinstance(state, ftype):
                return True

        # Check name regex filters
        for pattern in self.name_filter:
            if pattern.search(state.name):
                return True

        return False

    def pre(self, state) -> bool:
        # Build node info
        node = {
            "id": state.uid,
            "name": state.name,
            "type": type(state).__name__,
            "status": state.status.name,
            "available_outcomes": list(state.get_outcomes()),
            "children": [],
            "parentId": state.parent.uid if state.parent is not None else None,
            "collapsible": isinstance(state, GeneratorWithList)
        }

        # Children extraction
        if isinstance(state, GeneratorWithList):
            node["children"] = [s["state"].uid for s in state.states]

        if isinstance(state, GeneratorWithState):
            node["children"] = [state.state.uid]

        self.nodes[state.uid] = node

        if state.parent is None:
            self.root_id = state.uid

        # Return False to stop descending
        return not self._should_skip_descend(state)

    def post(self, state):
        pass

    def result(self):
        return {"rootId": self.root_id, "nodes": list(self.nodes.values())}


