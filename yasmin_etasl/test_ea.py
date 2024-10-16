#!/usr/bin/env python3


import rclpy
import threading
import sys
#import rclpy.clock
from rclpy.duration import Duration
import rclpy.time
from yasmin_ros.yasmin_node import YasminNode
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub
from yasmin_action_server.yasmin_action_server import YasminActionServer,get_logger,set_logger, action_state_cb, CancelState,EmptyStateMachine
from yasmin_viewer import YasminViewerPub
from .yasmin_ticking import *
from .yasmin_ticking_ros import *
from .yasmin_ticking_etasl import *



class Counter(TickingState):
    """
    Counter(name,maxcount) constructs a TickingState that implements a counter
    returns maxcount times TICKING
    and then SUCCEED
    """
    def __init__(self,name:str,maxcount:int):
        super().__init__([SUCCEED])
        self.maxcount = maxcount
        self.name = name
        self.count = 0
        self.log = YasminNode.get_instance().get_logger()

    def entry(self, blackboard:Blackboard) -> str:        
        self.log.info(f"{self.name} : starting to count")
        self.count = 0
        if self.count < self.maxcount:
            return CONTINUE 
        else:
            return SUCCEED
    
    def doo(self, blackboard:Blackboard) -> str:
        self.log.info(f"{self.name} :  count (count:{self.count}/{self.maxcount})")
        self.count = self.count + 1
        if self.count <= self.maxcount:
            return TICKING
        else:
            return SUCCEED
            
    def exit (self) -> str:        
        self.log.info(f"{self.name} : stop  counting")
        return SUCCEED





class MyMessage(State):
    """
    Message(msg) returns a State that displays a message
    """
    def __init__(self,msg) -> None:
        super().__init__([SUCCEED,])
        self.msg = msg
    def execute(self,blackboard: Blackboard)-> str:
        my_node = YasminNode.get_instance()
        log = my_node.get_logger()
        log.info(f'Entering MyMessage : {self.msg}')
        return SUCCEED

class Waiting(TickingState):
    """
    Waiting(msg,maxcount) implements a TickingState that simulates a process that takes
    some time.
    """
    def __init__(self,msg, maxcount):
        super().__init__([SUCCEED])
        self.msg = msg
        self.maxcount = maxcount
        self.count = 0


    def entry(self, blackboard:Blackboard) -> str:
        log = YasminNode.get_instance().get_logger()
        log.info(f"starting to wait {self.msg} : (count:{self.count}/{self.maxcount})")
        self.count = 0
        if self.count < self.maxcount:
            return CONTINUE  # could also be TICKING
        else:
            return SUCCEED
    
    def doo(self, blackboard:Blackboard) -> str:
        log = YasminNode.get_instance().get_logger()
        log.info(f"wait {self.msg} : (count:{self.count}/{self.maxcount})")
        self.count = self.count + 1
        if self.count <= self.maxcount:
            return TICKING
        else:
            return SUCCEED
            
    def exit (self) -> str:
        log = YasminNode.get_instance().get_logger()
        log.info(f"stop  waiting {self.msg} ")
        return SUCCEED

    
class MyStateMachine(cbStateMachine):
    """
    A small state machine to demonstrated the above implemented nodes
    """
    def __init__(self,name="",maxcount=5,maxwait=10):
        super().__init__(outcomes=[SUCCEED,CANCEL],statecb=default_statecb)
        if name!="":
            name = name +"."
        # add states

        self.add_state(
            name+"CTR",
            Counter(name+"CTR",maxcount=maxcount),
            transitions={
                TICKING: name+"MovingDown",  # This is another way to use a ticking state by catching the TICKING 
                                             # for a state transition instead of outcome of the state machine
                SUCCEED: SUCCEED  #you can connect a transition to another transition!
            }
        )        
        self.add_state(name+"MovingDown", MyMessage(name+"moving down"),transitions={SUCCEED: name+"Waiting"})
        self.add_state(name+"Waiting",  Waiting(name+"Waiting",maxwait), transitions={SUCCEED:name+"MovingUp"})
        self.add_state(name+"MovingUp", MyMessage(name+"moving up"),transitions={SUCCEED: name+"CTR"}) 

class MySequence(Sequence):
    def __init__(self):
        name = "mysequence"
        super().__init__(name,[SUCCEED,ABORT])
        name = "name" + "."
        self.add_state(name+"msg1", MyMessage(name+"msg1"))
        self.add_state(name+"ctr1", Counter(name+"ctr1",3))
        self.add_state(name+"msg2", MyMessage(name+"msg2"))
        self.add_state(name+"ctr2", Counter(name+"ctr2",3))
        self.add_state(name+"msg3", MyMessage(name+"msg3"))




class MyStateMachine2(cbStateMachine):
    def __init__(self,name,maxcount=5):
        super().__init__(outcomes=[SUCCEED])
        def cb(self,blackboard):
            for i in range(maxcount):
                yield TICKING
            yield SUCCEED
        name = name + "."
        self.add_state(name+"ctr", 
                       Generator(name+"ctr",outcomes=[SUCCEED],execute_cb=cb),
                       transitions={SUCCEED:name+"msg"}
                       )
        self.add_state(name+"msg", MyMessage("my_message"))





class ParallelStates(ConcurrentSequence):
    def __init__(self):
        super().__init__("ParallelStates",outcomes=[SUCCEED,CANCEL])
        #self.add_state("SM1", MyStateMachine("SM1",1,15))
        #self.add_state("SM2", MyStateMachine("SM2",2,5))
        self.add_state("Timer", TimedWait(Duration(seconds=2.5)))
        #self.add_state("MySequence", MySequence())
        #self.add_state("SM3", MyStateMachine2("SM3",5))
        #self.add_state("Repeat", Repeat(10,MyMessage("repeated item")))
        self.add_state("my_sequence",Sequence("my_sequence").add_state(
                "movinghome",eTaSL_StateMachine("MovingHome")
            ).add_state(
                "movingup",eTaSL_StateMachine("MovingUp")
            )
            .add_state(
                "movingdown",eTaSL_StateMachine("MovingDown")
            ).add_state(
                "movingup",eTaSL_StateMachine("MovingUp")
            ).add_state(
                "my_message",MyMessage("Hello world")
            )
        )
            
        #self.add_state("TimedRepeat", 
        #               TimedRepeat(
        #                   maxcount=5,
        #                   timeout=Duration(seconds=2.0), 
        #                   state=MyMessage("hello") 
        #                )
        #)



class YasminRunner:
    def __init__(self,node:Node, statemachine:TickingState, blackboard: Blackboard, sampletime):
        self.node  = node
        self.sm    = statemachine
        self.bm    = blackboard
        self.timer = self.node.create_timer(sampletime, self.timer_cb)
        self.outcome = "TICKING"
        self.outcome_lock = Lock()

    def timer_cb(self):
        outcome = self.sm(self.bm)
        #resetprint("---"),
        if outcome!=TICKING:
            self.timer.cancel()
            self.set_outcome(outcome)

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome

# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    my_node = YasminNode.get_instance()
    set_logger(my_node)
    blackboard = Blackboard()

    load_task_list("$[yasmin_etasl]/tasks/my_tasks.json",blackboard)
    
    sm = ParallelStates()


    sm = Sequence("my_sequence").add_state(
                "movinghome",eTaSL_StateMachine("MovingHome")
            ).add_state(
                "movingup",eTaSL_StateMachine("MovingUp")
            ).add_state(
                "movingdown",eTaSL_StateMachine("MovingDown")
            ).add_state(
                "movingup",eTaSL_StateMachine("MovingUp")
            ).add_state(
                "my_message",MyMessage("Hello world")
            )











    YasminViewerPub("Complete FSM", sm)
    runner = YasminRunner(my_node,sm,blackboard,0.5)
    
    try:
        while (runner.get_outcome()=="TICKING"):
            rclpy.spin_once(my_node)
        #outcome = TICKING
        # while rclpy.ok() and outcome==TICKING:
        #     outcome = sm(blackboard) #This function will block until the output of sm_out is achieved
        #     print("---")
        #     rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    print("final outcome : ",runner.get_outcome())

if __name__ == "__main__":
    sys.exit(main(sys.argv))
