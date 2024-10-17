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
    base[label="", shape=point];
    node [shape=rectangle style="filled,rounded" fillcolor=lightblue ];
"""
        self.postamble = """
}

"""
        self.doc = ""

    def add_to_stack(self, state):
        # self.stack is never empty:
        self.childnr.append(0)
        self.childnr[-2] = self.childnr[-2] + 1
        self.stack.append(f"{self.stack[-1]}_{state.name}_{self.childnr[-2]}")
        return self.stack[-1], self.stack[-2]
        

    def pre(self, state) -> bool:
        if isinstance(state,StateMachineElement):
            return True
       
        else:
            fullname, previousname = self.add_to_stack(state)
            shortname = state.name
            #if isinstance(state,GeneratorWithState):
            #    return True
            #elif isinstance(state,GeneratorWithList):            
            #    return True
            #if isinstance(state,TickingState):
            self.doc = self.doc+f'    {previousname} -> {fullname};\n    {fullname} [label="{shortname}"]'
            if isinstance(state,TickingStateMachine):
                # don't look inside a TickingStatemachine (for now...)
                return False 
            else:
                return True

        
    
    
    def post(self, state):
        if isinstance(state,StateMachineElement):
            return 
        else:        
            self.stack.pop()
            self.childnr.pop()

    
    def print(self):
        dotstring = self.preamble + self.doc + self.postamble
        print(dotstring)