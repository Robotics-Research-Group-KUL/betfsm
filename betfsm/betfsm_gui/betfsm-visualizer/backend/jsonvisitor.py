
from betfsm.betfsm import TickingState, GeneratorWithList

class JsonVisitor:
    def __init__(self):
        self.nodes = {}
        self.root_id = None

    def pre(self, state) -> bool:
        node = {
            "id": state.uid,
            "name": state.name,
            "type": type(state).__name__,
            "status": state.status.name,
            "outcomes": list(state.get_outcomes()),
            "children": [],
            "collapsible": isinstance(state, GeneratorWithList)
        }
        if isinstance(state, GeneratorWithList):
            node["children"] = [s["state"].uid for s in state.states]
        self.nodes[state.uid] = node
        if state.parent is None:
            self.root_id = state.uid
        return True

    def post(self, state): pass

    def result(self):
        return {"rootId": self.root_id, "nodes": list(self.nodes.values())}
