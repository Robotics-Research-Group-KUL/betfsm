from rclpy.executors import MultiThreadedExecutor
from yasmin_etasl.yasmin_ticking_etasl import *
from yasmin_etasl.logger import get_logger,set_logger
from yasmin_etasl.graphviz_visitor import *
from yasmin_etasl.yasmin_action_server import YasminActionServer,CheckForCanceledAction,WhileNotCanceled
import rclpy 



# import some example state machines:
from . import sm_up_and_down as examples


# define your shutdown procefdure:
class CheckingCancelAndShutdown(TickingStateMachine):
    def __init__(self,name:str,state:TickingState,srv_name:str="/etasl_node",timeout:Duration = Duration(seconds=1.0), node : Node = None):
        # execute in sequence but don't care about ABORT, only way to fail is TIMEOUT
        super().__init__(name,[CANCEL,SUCCEED])

        self.add_state(state=WhileNotCanceled("while_not_canceled",state), transitions={CANCEL:"DEACTIVATE_ETASL",SUCCEED:SUCCEED, TIMEOUT:"DEACTIVATE_ETASL"})

        self.add_state(state=LifeCycle("DEACTIVATE_ETASL",srv_name,Transition.DEACTIVATE,timeout,node),
                       transitions={SUCCEED: "CLEANUP_ETASL",  ABORT: "CLEANUP_ETASL", TIMEOUT:"CLEANUP_ETASL"} )
        self.add_state( state=LifeCycle("CLEANUP_ETASL",srv_name,Transition.CLEANUP,timeout,node),
                       transitions={SUCCEED: CANCEL, ABORT: CANCEL,TIMEOUT: CANCEL} )

                  


def run_while_publishing( sm):
    """
    small state machine that executes `sm` while publishing graphviz to a topic /gz
    """

    #do_not_expand_types doesn't work for now...
    return Concurrent("concurrent",[
            sm,
            GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[eTaSL_StateMachine])
    ])







def main(args=None):

    rclpy.init(args=args)
    
    node = YasminTickingNode.get_instance("yasmin_action_server")


    set_logger("default",node.get_logger())
    #set_logger("service",node.get_logger())
    set_logger("state",node.get_logger())


    blackboard = {} 
    # load your tasks
    load_task_list("$[yasmin_etasl_demos]/tasks/my_tasks.json",blackboard)


    # which state-machine will be exeuted for which task name:
    statemachines={}
    statemachines["up_and_down_as_function"] =  run_while_publishing( CheckingCancelAndShutdown("check", examples.up_and_down_as_a_function(node), node=node  ) )
       
    statemachines["up_and_down"] =  run_while_publishing( CheckingCancelAndShutdown("check", examples.Up_and_down_with_parameters_lambda(node), node=node  ) )
    # if you add additional member `input_parameters_schema` the action server will use this to validate the input:
    statemachines["up_and_down"].input_parameters_schema=examples.my_schema
    
    print(statemachines)

    action_server = YasminActionServer(blackboard,statemachines,100,node)

    # single or multi threaded executor does not matter here, only default callback group is used (which is mutually exclusive)
    executor = MultiThreadedExecutor()  
    executor.add_node(action_server.node)    
    executor.spin()
    
    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
