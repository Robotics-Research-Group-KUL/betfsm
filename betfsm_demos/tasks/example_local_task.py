#!/usr/bin/env python3
#
# Example with output of crospi tasks.
#region Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Erwin Aertbelien
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#endregion

import sys
import rclpy

from betfsm import (
    Sequence,  Repeat, CANCEL, NO_EVENT,TICKING,SUCCEED,set_logger,get_logger,
    get_path, EventSequential, Ctrl_C_Condition, Generator, Concurrent,
    TickingState, Blackboard, GeneratorWithList, EventSequential,
    Timeout_Condition, AlwaysOutcome, CC_array, CC_bb_array, CC_bb_table,
    wait_for, wait_for_and_check, UnexpectedOutcome
)


from betfsm_crospi import (
    load_task_list, CrospiTask, CrospiDeactivate, CrospiOutput
)

from betfsm_ros import (
    MarkerPublisher,Marker,ColorRGBA,TF2Broadcaster,TF2Listener,TFSpec
)
from betfsm_ros import BeTFSMNode,ROSRunner
from typing import Callable,List,Dict
from typing import Any, Dict
import numpy as np


#region Now obsolete functions:
# def from_bb_table(path:str,lbls:List[str],nrows:int) -> Callable[[TickingState,dict], NDArray[Any]]:
#     """
#     Factory function that creates a callback that select from a table structure with heading, path points to a dict with:
#         "heading" key : [ "name1","name2",...]
#         "data" key :    CircularNumpyBuffer or NDArray of numpy
#     Look at the newest values and do NOT consume. (default: True).
#     Parameters
#     ----------
#     path : str
#         path (can be relative w.r.t. the calling BeTFSM node)
#     lbls : List[str]
#         labels  you want to return
#     nrows : int
#         maximum number of rows to return (can be less if not available)
#     Returns
#     -------
#     Callable[[TickingState,dict], NDArray[Any]]
#         callback that will be used when executing. Returns a COPIED array 
#     """
#     first_time = True
#     data = None
#     idx  = []
#     def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
#         nonlocal data, path, lbls, idx, first_time,nrows
#         if first_time:
#             base                          = get_path(state, blackboard, path)
#             if base is None:
#                 return None
#             data                          = base.get("data",None)
#             if data is None:
#                 return None
#             header                        = base.get("header", None)
#             if header is None:
#                 return None
#             idx = [ header.index(lbl) for lbl in lbls ]
#             first_time = False
#         if isinstance(data,CircularNumpyBuffer):
#             return data.peek_newest_n(nrows)[:,idx]
#         else:
#             return data[-min(nrows,data.shape[0]):,idx].copy()
#     return getter
# def consume_from_bb_table(path:str,lbls:List[str],nrows:int) -> Callable[[TickingState,dict], NDArray[Any]]:
#     """
#     Factory function that creates a callback that select from a table structure with heading, path points to a dict with:
#         "heading" key : [ "name1","name2",...]
#         "data" key :    CircularNumpyBuffer
#     Looks at OLDEST values and CONSUMES.  
#     Only works with an underlying CicularNumpyBuffer (no direct NDArray allowed for the data key, in contrast to from_bb_table)
#     Parameters
#     ----------
#     path : str
#         path (can be relative w.r.t. the calling BeTFSM node)
#     lbls : List[str]
#         labels  you want to return
#     nrows : int
#         maximum number of rows to return (can be less if not available)
#     peek_newest: bool
#         look at the newest values and do NOT consume. (default: True).
#         Only one of peek_newest and consume_oldest can be True.
#     consume_oldest: bool
#         look at the oldest values and consume. (default: False).
#         Only allowed for CircularNumpyBuffer

#     Returns
#     -------
#     Callable[[TickingState,dict], NDArray[Any]]
#         callback that will be used when executing. Returns a COPIED array 
#     """
#     first_time = True
#     data:CircularNumpyBuffer = []
#     idx  = []
#     def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
#         nonlocal path, lbls, data, idx, first_time,nrows
#         if first_time:
#             base                          = get_path(state, blackboard, path)
#             data                          = base.get("data",None)
#             header                        = base.get("header", None)
#             idx = [ header.index(lbl) for lbl in lbls ]
#             first_time = False
#         return data.consume_oldest_n(nrows)[:,idx]
#     return getter



# def from_bb_array(path:str,idx:List[int]=None) -> Callable[[TickingState,dict], NDArray[Any]]:
#     """
#     Factory function that creates a callback that select from a numpy array:
#     The path refers to the array (for table with heading, see select_from_bb_table)    

#     Parameters
#     ----------
#     path : str
#         path (can be relative w.r.t. the calling BeTFSM node)
#     idx : List[int]
#         column indices you want to return, can by None for all columns.(default=None)

#     Returns
#     -------
#     Callable[[TickingState,dict], NDArray[Any]]
#         callback that will be used when executing. Returns a COPIED array 
#     """
#     first_time = True
#     data = []
#     def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
#         nonlocal path, data, idx, first_time
#         if first_time:
#             data                         = get_path(state, blackboard, path)
#             if idx is None:
#                 idx = np.arange(0,data.shape[1]);
#             first_time = False
#         return data[:,idx].copy()
#     return getter


# def from_array(arr:NDArray[Any]) -> Callable[[TickingState,dict], NDArray[Any]]:
#     """
#     Factory function that creates a callback that select from the given numpy array reference.

#     Parameters
#     ----------
#     arr: NDArray 
#         maximum number of rows to return (can be less if not available). 

#     Returns
#     -------
#     Callable[[TickingState,dict], NDArray[Any]]
#         callback that will be used when executing. Returns a COPIED array 
#     """
#     def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
#         return arr 
#     return getter
#endregion

def param_from_bb(path: str):
    def getter(state:TickingState, blackboard:dict)->Dict[str,Any]:
        nonlocal path
        retval= get_path(state,blackboard,path)
        return retval
    return getter




#################################################################
#   CrospiOutput
#################################################################


class Compute_v2(Generator):
    def __init__(self, 
                 name:str, 
                 cb:Callable[ [TickingState,Blackboard], str ],
                 outcomes:List[str]
                ):
        """
        State that performs some computations using a callback function and places the
        results in a predetermined location in the blackboard.

        you can specify outcome to return.  You can return TICKING to keep ticking and
        re-evaluating the callback at each tick.  Of course, this only makes sense if
        you run other TickingStates concurrently.

        Parameters:
            name:
                name of this node
            cb: 
                callback function with signature `def cb(state,blackboard)->str` that can
                adapt the blackboard. It can return an outcome,
                If outcome!=TICKING, called once, otherwise called
                repeatedly.
            outcomes:
                allowable outcomes
        """
        assert( isinstance(outcomes, list))
        super().__init__(name,outcomes)
        self.cb = cb
    def co_execute(self,blackboard):
        print("Compute_v2 started")
        while True:
            print("evaluating callback")
            outcome=self.cb(self,blackboard)
            print(f"callback returns {outcome}")
            if outcome!=TICKING:
                break
            yield outcome
        print("Compute_v2 finished")
        yield outcome

#region
# class ResetLifeCycleState(Generator):
#     def __init__(self,  name:str, srv_node:str, timeout:Duration=Duration(seconds=1.0), node:BeTFSMNode=None):
#         """_summary_

#         Parameters
#         ----------
#         name : str
#             _description_
#         srv_node : str
#             node that offers the service, service will be e.g. "{srv_node}/get_state"
#         timeout : Duration, optional
#             _description_, by default Duration(seconds=1.0)
#         node : BeTFSMNode, optional
#             BeTFSMNode, if not specified the singleton instance, by default None
#         """
#         if node is None:
#             self.node = BeTFSMNode.get_instance()
#         else:
#             self.node = node
#         super().__init(name,[SUCCEED,TIMEOUT,CANCEL])
#         self.srv_node = srv_node
#         self.get_state_srv_name = f"{srv_node}/get_state"
#         self.change_state_srv_name = f"{srv_node}/change_state"
#         self.get_state_client = self.node.get_client(GetState,self.get_state_srv_name)
#         self.change_state_client = self.node.get_client(ChangeState,self.change_state_srv_name)
#         self.timeout=timeout



#     def set_start_time(self):
#         self.starttime = self.node.get_clock()


#     def wait_with_timeout(self, description:str, cb):
#         while not cb():
#             if self.node.get_clock().now() - self.starttime > self.timeout:
#                 get_logger().warning(f"ToLifeCycleState {self.name}: timout occurred  in {description}")
#                 yield TIMEOUT
#             yield TICKING
#         return SUCCEED # ensures that yield from ends

#     def co_execute(self, blackboard):
#         self.set_start_time()
#         try:
#             while True:
#                 # waiting for service is ready:
#                 yield from self.wait_with_timeout(
#                     f"{self.get_state_srv_name}.service_is_ready",
#                     self.get_state_client.service_is_ready)
                
#                 # wating for get_state()
#                 current_future = self.get_state_client.call_async( GetState.Request())
#                 yield from self.wait_with_timeout( f"GetState request to {self.srv_node}", current_future.done)
#                 response = current_future.result() 
#                 current_label = response.current_state.label 
#                 get_logger("ros").info(f"ResetLifeCycleState: target {self.srv_node} is currently in state '{current_label}")

#                 # deciding next transition:
#                 req = ChangeState.Request()
#                 lbl = ""
#                 if current_label=="unconfigured":
#                     yield SUCCEED
#                     break
#                 elif current_label=="active":
#                     lbl="TRANSITION_DEACTIVATE"
#                     req.transition.id = Transition.TRANSITION_DEACTIVATE
#                     current_future = self.change_state_client.call_async( req)
#                 elif current_label=="inactive":
#                     lbl="TRANSITION_CLEANUP"
#                     req.transition.id = Transition.TRANSITION_CLEANUP
#                     current_future = self.change_state_client.call_async( req)
#                 else:
#                     get_logger().error("Cannot reset from state: {current_label}")
#                     yield CANCEL
#                     break 
#                 # processing respone of transition request:
#                 yield from self.wait_with_timeout( f"ChangeState request of '{self.srv_node}' with transition {lbl} ", current_future.done)
#                 response = current_future.result()
#                 if response.success:
#                     continue 
#                 else:
#                     get_logger().error(f"Transition request ({lbl}) rejected by {self.srv_node}")
#                     yield CANCEL
#                     break
#         except Exception as e:
#             get_logger().error( f"ResetLifeCycleState: transition servie {self.srv_node} failed: {str(e)}")
#             yield CANCEL
#         return
#endregion

#################################################################
#   B-Spline related utilities
#################################################################


def linspace_knots(nrofcp:int, degree:int):
    """Aux. function to place knots evenly spread with the correct multiplicity
       at beginning and end.

    Parameters
    ----------
    nrofcp : int
        number of control points
    degree : int 
        highest order power in the Bspline (i.e. cubic leads to degree=3) 
    """
    L = nrofcp+degree-1  # total length knot vector
    Li = L - 2*degree
    return [0.0]*degree + [ (i+1)/(Li+1) for i in range(Li)] + [1.0]*degree


#################################################################
#   Application
#################################################################


class MyApplication(Repeat):
    def __init__(self):
        cp = np.array([
            [ 0., 0.05, 0.1, 0.15, 0.2, 0.25, 0.3],
            [ 0., 0.0, 0.2, 0.2, 0.2, 0.0, 0.0],
            [ 0., 0.0, 0.0, 0.0, 0.15, 0.15, 0.15]
        ])
        degree = 3
        knots = linspace_knots(7,3)

        def compute_initial_frame(self,bb):
            loc=get_path(self,bb,"../initial")
            loc["posquat"] = [ 0.6,0.6,0.6, 1.0, 0, 0, 0]
            return SUCCEED

        def set_spline_param(self,bb):
            #    len(knots) = len(cp) + degree - 1
            #    multiplicity of degree at beginning and end
            #    only the parameters that you want to override
            p={}
            p["degree"] = degree
            p["knots"]  = knots 
            p["cp_x"] = cp[0] 
            p["cp_y"] = cp[1] 
            p["cp_z"] = cp[2] 
            return p 

        super().__init__("repeat",-1,Sequence("my_sequence",[
            CrospiTask("MoveHome","MoveHome"),
            Compute_v2("compute",compute_initial_frame,[SUCCEED]),
            TF2Listener("listen",[ TFSpec("base_link","tool0","../initial_tool0",is_matrix=False) ],SUCCEED),
            Concurrent("concurrent",[
                CrospiTask("MoveBSpline","MoveBSpline",cb=set_spline_param),
                CrospiOutput("output","/my_topic", queue_size=300, path='../../output'),
                MarkerPublisher("marker_traj",CC_bb_table("../../output",["x_tf","y_tf","z_tf"],3000), marker=Marker.LINE_STRIP, frequency=10, marker_id=1,lifetime_s=10),
                TF2Broadcaster("tfbroadcaster",[TFSpec("base_link","initial_tool0","../../initial_tool0",is_matrix=False)],frequency=10),
                MarkerPublisher("marker_cp1",CC_array(cp.T),marker=Marker.SPHERE_LIST,marker_id=2,frequency=10,frame_id="initial_tool0",lifetime_s=3),
                MarkerPublisher("marker",CC_array(cp.T),marker=Marker.LINE_STRIP,marker_id=3,frequency=10,frame_id="initial_tool0",lifetime_s=3,color=ColorRGBA(r=1.,g=1.,b=0.,a=1.)),
            ])
        ]))


class MyApplication_v2(GeneratorWithList):
    """ 
    Small application BeTFSM node to demonstrate an imperative way of defining. 

    Uses "yield from" together with "wait_for" function to wait until
    a subtree has finished. Achieving something similar to "await ..."

    subtrees need to be registered as children in order to be (automatically) cleaned up
    and reset to initial state when this application node ends.

    The advantage is that you have direct control over what is happening at run-time, and it
    becomes easier to do some additional computations without having to introduce a "Compute" node.
    """
    def __init__(self):
        super().__init__("My_application_v2",[SUCCEED, CANCEL])
       
        self.move_home =  CrospiTask("MoveHome","MoveHome")
        self.movingUp   =  CrospiTask("MovingUp","MovingUp")
        self.movingDown =  CrospiTask("MovingDown","MovingDown")
        self.add_state(self.move_home)
        self.add_state(self.movingUp)
        self.add_state(self.movingDown)

    def co_execute(self, blackboard):
        get_logger().info("MyApplication_v2 started")
        for i in range(2):
            result = yield from wait_for(self.move_home,blackboard)
            if result!=SUCCEED:
                break 
            result = yield from wait_for(self.movingUp,blackboard)
            if result!=SUCCEED:
                break
            result = yield from wait_for(self.movingDown,blackboard)
            if result!=SUCCEED:
                break
        get_logger().info(f"MyApplication_v2 ended with {result}")
        yield result


class MyApplication_v3(GeneratorWithList):
    """  
    Similar to MyApplication_v2, but now we are using exceptions to 
    report unexpected outcomes
    """
    def __init__(self):
        super().__init__("My_application_v2",[SUCCEED, CANCEL])
       
        self.move_home =  CrospiTask("MoveHome","MoveHome")
        self.movingUp   =  CrospiTask("MovingUp","MovingUp")
        self.movingDown =  CrospiTask("MovingDown","MovingDown")
        self.add_state(self.move_home)
        self.add_state(self.movingUp)
        self.add_state(self.movingDown)

    def co_execute(self, blackboard):
        try:
            
            for i in range(2):
                yield from wait_for_and_check(self.move_home,blackboard)
                yield from wait_for_and_check(self.movingUp,blackboard)
                result = yield from wait_for_and_check(self.movingDown,blackboard)
                if result==SUCCEED:
                    get_logger().info("I can still do something with an expected outcome if needed")
            
            yield SUCCEED   #don't forget the report the outcome!

        except UnexpectedOutcome as e:
            get_logger().warn(f"MyApplication_v2 failed: {e}")
            yield CANCEL


#region Idea: introducing a scene
#
#  A style where we start with a node that describes the scene, and as only chil;d
#  it takes the application.
#
#  An idea: put all the information of the scene and let it
# be a base node of the application.
#  It could e.g. do visualization or assembly information comming from vision.
#  Some sensor are local and belong near or in the skill, some sensors are global and
#  belong in the scene.
#
# class MyScene(GeneratorWithList):
#     def __init__(self,my_app:TickingState):
#         super().__init__("MyScene", [SUCCEED, CANCEL])

#         self.scene = Concurrent("concurrent",[
#             my_app,
#             MarkerPublisher("marker_tgt",CC_bb_array('../../tgt'),marker=Marker.SPHERE_LIST,marker_id=100,
#                             frequency=2,frame_id="world",lifetime_s=1,color=ColorRGBA(r=1.0,g=0.5,b=0.0))
#         ])

#         # IMPORTANT: do not forget to add all the executed nodes!
#         # they need to be part of the tree, for cleanup/reset, and for get_path references
#         self.add_state(self.scene)

#     def co_execute(self, blackboard):
#         try:
#             local = get_path(self,blackboard,'.')
#             N = 3
#             x = np.linspace(-0.2,0.2,N)
#             y = np.linspace(-0.2,0.2,N)
#             X,Y = np.meshgrid(x,y)
#             Z   = np.ones((N*N,))
#             tgt = np.stack([X.ravel(), Y.ravel(),Z*0.3], axis=1) 
#             print(tgt)
#             local['tgt'] = tgt

#             yield from wait_for_and_check(self.scene,blackboard)

#             yield SUCCEED   #don't forget the report the outcome !
#         except UnexpectedOutcome as e:
#             get_logger().warn(f"MyScene failed: {e}")
#             yield CANCEL
#endregion


class MyApplication_v4(GeneratorWithList):
    """ 
    """
    def __init__(self):
        super().__init__("My_application_v4",[SUCCEED, CANCEL])
        # note that if we make references to a local blackboard, we make sure to go up
        # to the MyApplication_v4 local blackboard

        self.move_home =  CrospiTask("MoveHome","MoveHome")
        # call MovingDown task and set its parameters from the local blackboard:
        self.moving_down =  CrospiTask("MovingDown","MovingDown", cb=param_from_bb("../moving_down"))
        self.listen_tf = TF2Listener("listen",[ TFSpec("base_link","tool0","../initial_tool0",is_matrix=False) ],SUCCEED)
        self.spline_motion =    Concurrent("concurrent",[
                CrospiTask("MoveBSpline","MoveBSpline",cb=param_from_bb('../../spline')),
                CrospiOutput("output","/my_topic", queue_size=3000, path='../../output'),
                MarkerPublisher("marker_traj",CC_bb_table("../../output",["x_tf","y_tf","z_tf"],3000), marker=Marker.LINE_STRIP, frequency=10, marker_id=1,lifetime_s=6),
                TF2Broadcaster("tfbroadcaster",[TFSpec("base_link","initial_tool0","../../initial_tool0",is_matrix=False)],frequency=10),
                MarkerPublisher("marker_cp1",CC_bb_array('../../cp'),marker=Marker.SPHERE_LIST,marker_id=2,frequency=10,frame_id="initial_tool0",lifetime_s=3,scale=0.3),
                MarkerPublisher("marker_cp2",CC_bb_array('../../cp'),marker=Marker.LINE_STRIP,marker_id=3,frequency=10,frame_id="initial_tool0",lifetime_s=3,color=ColorRGBA(r=1.,g=1.,b=0.,a=1.)),
                MarkerPublisher("marker_cp1",CC_bb_array('../../tgt'),marker=Marker.SPHERE_LIST,marker_id=4,frequency=10,frame_id="initial_tool0",lifetime_s=3,color=ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.)),
            ])
        # really, really important not to forget this:  
        self.add_state(self.move_home)
        self.add_state(self.moving_down)
        self.add_state(self.listen_tf)
        self.add_state(self.spline_motion)

    def co_execute(self, blackboard):
        try:
            # this spline starts and ends straight and has no vel/acc in x and y
            # the last three control points will have added the current tgt[i]
            cp = np.array([
                [ 0., 0.0,  0.0, 0.0,  0.0, 0.0,    0  ],
                [ 0., 0.0,  0.0, 0.0,  0.0, 0.0,    0  ],
                [ 0., 0.05, 0.1, 0.15, 0.2, 0.25,   0.3]
            ])

            degree = 5
            knots = linspace_knots(7,degree)
            local = get_path(self,blackboard,'.')

            # make a mesh
            N = 5
            x = np.linspace(-0.2,0.2,N)
            y = np.linspace(-0.2,0.2,N)
            X,Y = np.meshgrid(x,y)
            Z   = np.ones((N*N,))
            tgt = np.stack([X.ravel(), Y.ravel(),Z*0.3], axis=1) 
            local["tgt"]= tgt
            print(tgt)
            get_logger().info("====================== MOVING HOME ===========================")
            yield from wait_for_and_check(self.move_home,blackboard)

            get_logger().info("====================== MOVING DOWN (randomly) ===========================")
            p = get_path(self,blackboard,"./moving_down")
            r = np.random.randn()*0.1
            p["delta_pos"] = [0, 0, r]
            get_logger().info(f"moving down with {r} [m] in the z-direction")
            yield from wait_for_and_check(self.moving_down,blackboard)

            get_logger().info("====================== MOVING UP (randomly) ===========================")
            p["delta_pos"] = [0, 0, -r]
            get_logger().info(f"moving down with {r} [m] in the z-direction")
            yield from wait_for_and_check(self.moving_down,blackboard)

            get_logger().info("====================== listen for tf (once) ===========================")
            yield from wait_for_and_check(self.listen_tf,blackboard)
            with np.printoptions(precision=5,suppress=True,edgeitems=10):
                print("initial_tool0 ", local["initial_tool0"])

            for i in range(tgt.shape[0]):
                get_logger().info("====================== calling Bspline task ===========================")
                cp_copy = cp.copy()
                cp_copy[0,-3:]  += tgt[i,0] 
                cp_copy[1,-3:]  += tgt[i,1]
                cp_copy[2,-3:]  += 0  # tgt[i,2]  
                print(cp_copy)
                local["spline"]={
                    "cp_x":cp_copy[0,:], "cp_y":cp_copy[1,:] , "cp_z":cp_copy[2,:], 
                    "degree":degree, "knots":knots }

                local["cp"] = cp_copy.T   # unfortunately the format for markers (full matrix) is different than the one for the CrospiTask (vector-by-vector)
                #print(dumps_blackboard(local))
                yield from wait_for_and_check(self.spline_motion,blackboard)

                self.move_home.reset()
                self.moving_down.reset()
                self.listen_tf.reset()
                self.spline_motion.reset()
            yield SUCCEED   #don't forget the report the outcome !
        except UnexpectedOutcome as e:
            get_logger().warn(f"MyApplication_v4 failed: {e}")
            yield CANCEL


def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("example_local_task")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    
    blackboard = {}

    blackboard["numpy"]     = np.array([1,2,3.0]).tolist()
    blackboard["numpy_mat"] = np.array( [[1,2,3.0],[2.0,1,2]] ).tolist()

    load_task_list("betfsm_demos_tasks.json",blackboard)

    # running MySequence() and cleaning up when CTRL_C is pressed        
    nominal_sm = MyApplication_v4()
    #nominal_sm = MyApplication()
    cleanup_sm = CrospiDeactivate(force_outcome=CANCEL)
    sm = EventSequential("check_cancel", Ctrl_C_Condition("CTRL_C",repeated=3),{NO_EVENT:nominal_sm, "CTRL_C":cleanup_sm})

    # ROSRunner accepts command-line parameters (see --help)
    # has many more optional arguments than used below, see API documentation
    runner = ROSRunner(my_node,sm,blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)

    try:
        runner.run()
    except KeyboardInterrupt:
        my_node.destroy_node()
        return   
    my_node.destroy_node()
    rclpy.shutdown()
    print("shutdown")


if __name__ == "__main__":
    sys.exit(main(sys.argv))
