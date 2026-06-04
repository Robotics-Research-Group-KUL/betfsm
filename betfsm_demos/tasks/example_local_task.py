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
    Sequence,  Repeat,
    CANCEL, NO_EVENT,TICKING,SUCCEED,
    set_logger,get_logger,
    get_path_location,get_path_value,
    EventSequential, Ctrl_C_Condition,Generator,Concurrent,TickingState,Blackboard,GeneratorWithState,
    GeneratorWithList,EventSequential,EventConcurrent,EventOutcome,
    Timeout_Condition,AlwaysOutcome,CircularNumpyBuffer,
    to_graphviz_dotfile,
    dumps_blackboard
)
from betfsm.circularbuffer import CircularNumpyBuffer,numpy_json_serializer


from betfsm_crospi import load_task_list, CrospiTask, CrospiDeactivate,CrospiOutput,deque,Output,QoSDurabilityPolicy,QoSLivelinessPolicy,QoSHistoryPolicy,QoSReliabilityPolicy,QoSProfile
from betfsm_ros import BeTFSMNode,ROSRunner,Node
from rclpy.time import Duration,Time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from typing import Callable,List,Dict
from itertools import islice
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer, TransformListener, TransformException
from typing import Any, Dict
from dataclasses import dataclass,Field
from scipy.spatial.transform import Rotation as spR 
import numpy as np

from numpy.typing import NDArray



# def points_from_table(state:TickingState, path:str,lbl_x:str,lbl_y:str,lbl_z:str,length:int) -> Callable[[dict], tuple[float,float,float]]:
#     first_time = True
#     loc = []
#     ix = 0
#     iy = 1
#     iz = 2
#     error_encountered = False
#     def getter(blackboard:dict)->tuple[float,float,float]:
#         nonlocal loc,ix,iy,iz,first_time, path,lbl_x,lbl_y,lbl_z,length, state, error_encountered
#         if first_time:
#             base = get_path(state,blackboard,path)
#             loc      = base.get("data",None)
#             header   = base.get("header", None)
#             if loc is None:
#                 if not error_encountered:
#                     get_logger().error(f"points_from_table: cannot find location {path+'/data'} in blackboard")
#                 error_encountered = True 
#                 return None
#             if len(loc)==0:
#                 return None
#             if header is None:
#                 if not error_encountered:
#                     get_logger().error(f"points_from_table: cannot find location {path+'/header'} in blackboard")
#                 error_encountered = True 
#                 return None
#             if lbl_x not in header:
#                 if not error_encountered:
#                     get_logger().error(f"points_from_table: cannot find {lbl_x} in {header}")
#                 error_encountered = True 
#                 return None
#             ix = header.index(lbl_x)
#             if lbl_y not in header:
#                 if not error_encountered:
#                     get_logger().error(f"points_from_table: cannot find {lbl_y} in {header}")
#                 error_encountered = True 
#                 return None
#             iy = header.index(lbl_y)
#             if lbl_z not in header:
#                 if not error_encountered:
#                     get_logger().error(f"points_from_table: cannot find {lbl_z} in {header}")
#                 error_encountered = True 
#                 return None
#             iz = header.index(lbl_z)
#             if error_encountered:
#                 error_encoutered = False
#                 get_logger().error(f"points_from_table: error restored")
#             first_time=False
#         L = len(loc) 
#         start = max(L - length,0)
#         return [ Point(x=loc[i][ix], y=loc[i][iy], z= loc[i][iz]) for i in range(start,L)  ]
#     return getter

# def points_from_list(lst) -> Callable[[dict], tuple[float,float,float]]:
#     def getter(blackboard:dict)->tuple[float,float,float]:
#         return [ Point(x=lst[0][i], y=lst[1][i], z=lst[2][i]) for i in range(len(lst[0]))]
#     return getter



def from_bb_table(path:str,lbls:List[str],nrows:int) -> Callable[[TickingState,dict], NDArray[Any]]:
    """
    Factory function that creates a callback that select from a table structure with heading, path points to a dict with:
    
        "heading" key : [ "name1","name2",...]
        "data" key :    CircularNumpyBuffer or NDArray of numpy

        
    Look at the newest values and do NOT consume. (default: True).

    Parameters
    ----------
    path : str
        path (can be relative w.r.t. the calling BeTFSM node)
    lbls : List[str]
        labels  you want to return
    nrows : int
        maximum number of rows to return (can be less if not available)

    Returns
    -------
    Callable[[TickingState,dict], NDArray[Any]]
        callback that will be used when executing. Returns a COPIED array 
    """
    first_time = True
    data = None
    idx  = []
    def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
        nonlocal data, path, lbls, idx, first_time,nrows
        if first_time:
            base                          = get_path(state, blackboard, path)
            if base is None:
                return None
            data                          = base.get("data",None)
            if data is None:
                return None
            header                        = base.get("header", None)
            if header is None:
                return None
            idx = [ header.index(lbl) for lbl in lbls ]
            first_time = False
        if isinstance(data,CircularNumpyBuffer):
            return data.peek_newest_n(nrows)[:,idx]
        else:
            return data[-min(nrows,data.shape[0]):,idx].copy()
    return getter

def consume_from_bb_table(path:str,lbls:List[str],nrows:int) -> Callable[[TickingState,dict], NDArray[Any]]:
    """
    Factory function that creates a callback that select from a table structure with heading, path points to a dict with:
    
        "heading" key : [ "name1","name2",...]
        "data" key :    CircularNumpyBuffer

    Looks at OLDEST values and CONSUMES.  
    Only works with an underlying CicularNumpyBuffer (no direct NDArray allowed for the data key, in contrast to from_bb_table)

    Parameters
    ----------
    path : str
        path (can be relative w.r.t. the calling BeTFSM node)
    lbls : List[str]
        labels  you want to return
    nrows : int
        maximum number of rows to return (can be less if not available)
    peek_newest: bool
        look at the newest values and do NOT consume. (default: True).
        Only one of peek_newest and consume_oldest can be True.
    consume_oldest: bool
        look at the oldest values and consume. (default: False).
        Only allowed for CircularNumpyBuffer

    Returns
    -------
    Callable[[TickingState,dict], NDArray[Any]]
        callback that will be used when executing. Returns a COPIED array 
    """
    first_time = True
    data:CircularNumpyBuffer = []
    idx  = []
    def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
        nonlocal path, lbls, data, idx, first_time,nrows
        if first_time:
            base                          = get_path(state, blackboard, path)
            data                          = base.get("data",None)
            header                        = base.get("header", None)
            idx = [ header.index(lbl) for lbl in lbls ]
            first_time = False
        return data.consume_oldest_n(nrows)[:,idx]
    return getter

def from_bb_array(path:str,idx=None) -> Callable[[TickingState,dict], NDArray[Any]]:
    """
    Factory function that creates a callback that select from a numpy array:
    The path refers to the array (for table with heading, see select_from_bb_table)    

    Parameters
    ----------
    path : str
        path (can be relative w.r.t. the calling BeTFSM node)
    lbls : List[str]
        labels  you want to return
    nrows : int
        maximum number of rows to return (can be less if not available)
    Returns
    -------
    Callable[[TickingState,dict], NDArray[Any]]
        callback that will be used when executing. Returns a COPIED array 
    """
    first_time = True
    data = []
    def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
        nonlocal path, data, idx, first_time
        if first_time:
            data                         = get_path(state, blackboard, path)
            if idx is None:
                idx = np.arange(0,data.shape[1]);
            first_time = False
        return data[:,idx].copy()
    return getter

def from_array(arr:NDArray[Any]) -> Callable[[TickingState,dict], NDArray[Any]]:
    """
    Factory function that creates a callback that select from the given numpy array reference.

    Parameters
    ----------
    arr: NDArray 
        maximum number of rows to return (can be less if not available). 

    Returns
    -------
    Callable[[TickingState,dict], NDArray[Any]]
        callback that will be used when executing. Returns a COPIED array 
    """
    def getter(state:TickingState,blackboard:dict)->NDArray[Any]:
        return arr 
    return getter






class MarkerPublisher(Generator):
    def __init__(self, name:str, 
                 getter:Callable[[TickingState,dict],NDArray[Any]],
                 marker:Marker|int|None=None,
                 frequency:float=10,
                 array_type: bool = True,   #an array type of marker such as LINE_STRIP or POINTS
                 marker_id:int=0,
                 frame_id:str="world",
                 node:Node=None
                 ):
        """_summary_

        Accepts getters from_bb_table, consume_form_bb_table, from_bb_array and from_array callback factories

        Parameters
        ----------
        name : str
            _description_
        getter : Callable[[TickingState,dict],NDArray[Any]]
            _description_
        marker : Marker | int | None, optional
            _description_, by default None
        frequency : float, optional
            _description_, by default 10
        array_type : bool, optional
            _description_, by default True
        frame_id : str, optional
            _description_, by default "world"
        node : Node, optional
            _description_, by default None
        """
        if node is None:
            node = BeTFSMNode.get_instance()
        super().__init__(name,[])
        self.node         = node
        self.getter       = getter 
        if marker is None or marker == Marker.LINE_STRIP:
            marker = Marker()
            marker.type   = Marker.LINE_STRIP # Type of marker (SPHERE, CUBE, ARROW, CYLINDER, etc.)
            marker.action = Marker.ADD    # ADD or DELETE marker
            marker.scale.x = 0.003  # Diameter along X [m]
            marker.scale.y = 0.003  # Diameter along X [m]
            marker.scale.z = 0.003  # Diameter along X [m]
            marker.color.r = 1.0  # from 0..1
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Opacity (1.0 = completely solid)
            marker.id      = 0
            marker.ns     = "betfsm"
            marker.lifetime = Duration(seconds=0).to_msg()
            marker.header.frame_id = "world"
        if marker is None or marker == Marker.SPHERE_LIST:
            marker = Marker()
            marker.type   = Marker.SPHERE_LIST # Type of marker (SPHERE, CUBE, ARROW, CYLINDER, etc.)
            marker.action = Marker.ADD    # ADD or DELETE marker
            marker.scale.x = 0.02  # Diameter along X [m]
            marker.scale.y = 0.02  # Diameter along X [m]
            marker.scale.z = 0.02  # Diameter along X [m]
            marker.color.r = 1.0  # from 0..1
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Opacity (1.0 = completely solid)
            marker.id      = 0
            marker.ns     = "betfsm"
            marker.lifetime = Duration(seconds=0).to_msg()
            marker.header.frame_id = "world"
 
        self.marker       = marker
        if marker_id is not None:
            self.marker.id = marker_id
        if frame_id is not None:
            self.marker.header.frame_id = frame_id
        self.frequency    = frequency
    #.doc("type of the marker: \n ARROW=0,CUBE=1,SPHERE=2,CYLINDER=3,LINE_STRIP=4,LINE_LIST=5,CUBE_LIST=6,SPHERE_LIST=7,\n POINTS=8,TEXT=9,MESH_RESOURCE=10 (see ROS doc)")
    
    def co_execute(self, blackboard):
        self.publisher = self.node.create_publisher(Marker, '/visualization_marker', 10)
        sample_time = 0
        now         = self.node.get_clock().now()
        previous    = now
        count=0
        warnings=[]
        while True:
            now = self.node.get_clock().now()
            if (now-previous).nanoseconds >= sample_time:
                sample_time = 1E9 / self.frequency
                lst = self.getter(self,blackboard)
                if lst is not None:
                    self.marker.header.stamp = now.to_msg()
                    self.marker.points = [ Point( x=r[0], y=r[1], z=r[2] ) for r in lst ] 
                    self.publisher.publish(self.marker)
                    previous = now
                else:
                    if not 1 in warnings:
                        get_logger("crospi").error(f"MarkerPublisher: could not get marker points from getter callback")
                        warnings.append(1)
            yield TICKING

    def exit(self):
        # this is also called when its externally reset or naturally finishes by itself
        self.node.destroy_subscription(self.publisher)
        return super().exit()



def get_path(state: TickingState, blackboard: Blackboard, path: str,default: Any = {},force=False) -> Dict[str, Any]:
    """ Gets a reference to a dict inside the blackboard, following strict path rules.

    Path rules:

    - Must start with ".", "..", or "/".
    - "." refers to this state's local context (created lazily).
    - "./x/y" refers to nested dicts inside this state's local context.
    - "/some_name" refers to a global dict in the blackboard root.
    - ".." moves to parent state and its local context (created lazily).
      Can be repeated: "../../meas" is valid.
    - Any path starting with a bare name, e.g. "output/...", "foo/bar" is illegal.
    - Missing dicts along the path are created on the fly.
    - If a path component exists but is not a dict, it is overwritten with a dict.
    - the last path component will get the value default if it does not exist
    - if force==True, the last value will be forced to default.

    The options default and force make it possible to use get_path to be used also for
    changing the blackboard.  For scalar types such as int,number and str , you
    can pass the value in default.  For other types dict, list, NDArray, you have
    a reference to the location in the dashboard, and with careful assignment you
    can make sure that the blackboard is adapted (use "[:]" to change the content instead
    of the reference )

    local context to a state are stored in blackboard["local"][state.uid]

    Parameters
    ----------
    state : TickingState
       if it is a local path, this is the state to take as reference. 
    blackboard : Blackboard
        global blackboard 
    path : str
        path inside the local blackboard can be absolute for global variables and relative for local variables. 
    default : Any, optional
        default value that is returned (and put in the blackboard), if there is no value present.
    force : bool, optional
        force default to be used, even if there is a value.

    Returns
    -------
    Dict[str, Any]
        _description_
    """

    def local_namespace(state,blackboard):
        if "local" not in blackboard:
            locals = {}
            blackboard["local"] = locals
        else:
            locals = blackboard["local"]
        if state.uid not in locals:
            local = {"name":state.name}  # for now, for debugging
            locals[state.uid] = local
        else:
            local = locals[state.uid]
        return local


    path = path.strip()
    ns    = blackboard
    phase = 1     

    parts = path.split("/")
    for i,part in enumerate(parts):
        if not part:
            continue
        if (phase==1):
            # phase 1: is looking for "." local namespace 
            if part==".":
                phase = 3
                ns    = local_namespace(state,blackboard)
                continue
            phase = 2
        if (phase==2):
            # phase 2: navigating to parents of state to find local namespace 
            if part=="..":
                state = state.parent
                ns    = local_namespace(state,blackboard)
                continue
            if part==".":
                raise ValueError("illegal path, '.' in the middle of path not allowed")
            phase = 3
        if (phase==3):
            # phase 3: navigating inside namespace
            if i < len(parts)-1:
                if part not in ns or not isinstance(ns[part], dict):
                    ns[part] = {}
            else: # last part is not necessarily dict.
                if part not in ns:
                    ns[part] = default 
            ns = ns[part]
    return ns 



class CrospiOutput_v2(Generator):
    """
    Record output of Crospi in a topic while executing subtree
    """
    def __init__(
            self,
            name:str,
            topic: str,
            queue_size: int = 1_000_000,
            path: str = "../output",            
            node: Node = None,
        ) -> None:
        """Record output of Crospi for a topic in a queue while executing subtree
           This queue is put in a specified location in the blackboard under two children "header" and "data".  The header is
           taken from the labels of the first message that arrives.

        Parameters:
            name : str
                name of the node
            topic : str
                ROS2 topic to subscribe to, should be of message type Output
            queue_size : int, optional
                maximum length of the buffer to record the data, by default 1_000_000
            path : str, optional
                path inside the blackboard, by default "/output"
            node : Node, optional
                ROS2 node, if None, BeTFSMNode.get_instance() will be used. by default None
        """
        super().__init__("eTaSLOutput",[SUCCEED, CANCEL])
        if node==None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        self.topic = topic
        self.qos = QoSProfile(
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.VOLATILE
        )
        self.path       = path
        self.queue_size = queue_size

    def cb_msg(self,msg) -> None:
        if len(msg.is_declared)>0 and all(msg.is_declared):
            if self.first_time:
                #self.header.append([n for n in msg.names ])
                self.header[:]   = [n for n in msg.names ]
                self.queue               = CircularNumpyBuffer(self.queue_size,len(self.header))
                self.path_ns['data']     = self.queue   # path_ns['data'] and queue point to the same data
                self.first_time          = False
            self.queue.add( [d for d in msg.data ])

            

    def co_execute(self,blackboard):
        self.path_ns               = get_path(self,blackboard,self.path)
        self.header           = []        
        self.queue            = None         # don't know the number of columns yet
        self.path_ns["data"]  = self.queue
        self.path_ns["header"]= self.header
        self.first_time       = True
        self.subscription     = self.node.create_subscription(Output,self.topic,self.cb_msg,self.qos)
        while True:
            yield TICKING        

    def exit(self):
        # this is also called when its externally reset or naturally finishes by itself
        self.node.destroy_subscription(self.subscription)
        if self.queue is not None and len(self.queue)>0:
            loc = {}
            self.path_ns["last"] = loc
            self.path_ns["is-circularbuffer"] = True
            self.path_ns["capacity"]          = self.queue.get_capacity() 
            last = self.queue.peek_newest()
            for lbl,val in zip(self.header, last):
                loc[lbl] = val
        return super().exit()



# def compute_example(state:TickingState, bb:Blackboard):
#     target = get_path(state.parent,bb,"./target")
#     meas   = get_path(state.parent,bb,"./meas")
#     local  = get_path(state,".")

#     local["sum"] = local.get("sum",0.0) + meas["force"]
#     local['N']   = local.get("N",0.0)   + 1

#     target["force"] = meas["force"] + 5
#     if abs( meas["force"]) > 10:
#         return CANCEL
#     return SUCCEED



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


def pose_from_crospiOutput(path:str,lbls:List[str]) -> Callable[[dict], tuple[float,float,float]]:
    first_time = True
    loc = []
    ndx = []
    def getter(blackboard:dict)->tuple[float,float,float]:
        nonlocal loc,ndx,first_time, path,lbls
        if first_time:
            if len(lbls)!=7:
                #get_logger().error(f"pose_from_crospiOutput: there should be 7 labels corresponding to x,y,z, qw,qx,qy,qz")
                return None
            loc = get_path_value(blackboard,path+"/data",None)
            if loc is None:
                #get_logger().error(f"pose_from_crospiOutput: cannot find location {path+'/data'} in blackboard")
                return None
            if len(loc)==0:
                #get_logger().error("pose_from_crospiOutput: output empty")
                return None
            header = get_path_value(blackboard,path+"/header",None)
            if header is None:
                #get_logger().error(f"pose_from_crospiOutput: cannot find location {path+'/header'} in blackboard")
                return None
            try:
                ndx = [ header.index[lbl] for lbl in lbls] 
            except ValueError:
                #get_logger().error(f"pose_from_crospiOutput: on of the labels {lbls}  not found in header {header}")
                return None 
            first_time=False
        L = len(loc) 
        i = -1
        return [loc[-1][i] for i in ndx]
    return getter


@dataclass
class TFSpec:
    """ 
    semantics : v_{wrt_to} = T_{wrt_to}_{of} *  v_{of} 
    """
    wrt_to: str 
    of: str
    path  : str
    is_matrix: bool = False


class TF2Listener(Generator):
    def __init__(self, name:str, tf_list:List[TFSpec], desired_outcome=str,node:BeTFSMNode=None):
        # tf_list: (tgt,src,path)
        if not node:
            node = BeTFSMNode.get_instance()
        self.node = node
        super().__init__(name,outcomes=[desired_outcome])
        self.tf_buffer = node.get_transformListenerBuffer()
        self.tf_list   = tf_list
        self.desired_outcome = desired_outcome

    def co_execute(self, blackboard):
        default_quat = np.array([0,0,0,1,0,0,0])
        default_mat  = np.eye(4)
        tf_bb = []
        for spec in self.tf_list:
            # if does not exist we choose default==[], such that we can assign a list to it without
            # needing access to the parent dict, using [:]
            if spec.is_matrix:
                tf_bb.append( get_path(self,blackboard,spec.path,default=default_mat.copy(),force=True))
            else:
                tf_bb.append( get_path(self,blackboard,spec.path,default=default_quat.copy(),force=True))
        while True:
            for i,spec in enumerate(self.tf_list):
                T=self.tf_buffer.lookup_transform(spec.wrt_to,spec.of,Time())
                loc =  tf_bb[i]
                if spec.is_matrix:
                    loc[:3,:3] = spR.from_quat( [T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z], scalar_first=True).as_matrix()
                    loc[:3,3]  = [  T.transform.translation.x, T.transform.translation.y, T.transform.translation.z]
                else:
                    # this still accesses the blackboard due to [:]
                    print(loc)
                    loc[:] = [  T.transform.translation.x, T.transform.translation.y, T.transform.translation.z,
                                        T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z]
            if self.desired_outcome!=TICKING:
                break 
            yield self.desired_outcome
        yield self.desired_outcome


# class TF2Publisher(Generator):
#     def __init__(self, name:str, 
#                  frame_id:str,
#                  child_frame_id:str,
#                  get_tf: Callable[[dict],List[float]],  #x,y,z, qw,qx,qy,qz
#                  eval_once: bool = False,
#                  frequency:float=10,
#                  node:BeTFSMNode=None
#                  ):
#         if node is None:
#             self.node = BeTFSMNode.get_instance()
#         else:
#             self.node = node
#         super().__init__(name,[])
#         self.get_tf       = get_tf
#         self.frame_id     = frame_id
#         self.child_frame_id = child_frame_id
#         self.frequency    = frequency

#     def co_execute(self, blackboard):
#         get_logger("crospi").info(f"TF2Publisher publishes {self.child_frame_id} with respect to {self.frame_id}")
#         self.tf_broadcaster     = self.node.get_transformBroadcaster() 
#         self.tf = TransformStamped()
#         self.tf.header.frame_id = self.frame_id
#         self.tf.child_frame_id  = self.child_frame_id

#         ns_interval = int(1E9 / self.frequency)
#         previous    = self.node.get_clock().now() - Duration(nanoseconds=ns_interval)
#         while True:
#             now = self.node.get_clock().now()
#             if (now-previous).nanoseconds >= ns_interval:
#                 previous = now
#                 # do something
#                 [x,y,z,qw,qx,qy,qz] = self.get_tf(blackboard)
#                 self.tf.header.stamp = now.to_msg()    
#                 self.tf.transform.translation.x = float(x)
#                 self.tf.transform.translation.y = float(y)
#                 self.tf.transform.translation.z = float(z)
#                 self.tf.transform.rotation.x = float(qx)
#                 self.tf.transform.rotation.y = float(qy)
#                 self.tf.transform.rotation.z = float(qz)
#                 self.tf.transform.rotation.w = float(qw)
#                 self.tf_broadcaster.sendTransform(self.tf)
#             yield TICKING

#     def exit(self):
#         # this is also called when its externally reset or naturally finishes by itself
#         return super().exit()


class TF2Broadcaster_v2(Generator):
    def __init__(self, name:str, 
                 tf_list:List[TFSpec],
                 frequency=10,
                 node:BeTFSMNode=None
                 ):
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        super().__init__(name,[])
        self.tf_list       = tf_list 
        self.frequency     = frequency

    def co_execute(self, blackboard):
        self.tf_broadcaster     = self.node.get_transformBroadcaster() 
        self.tf = TransformStamped()
        tf_bb = []
        for spec in self.tf_list:
            tf_bb.append( get_path(self,blackboard,spec.path))
        ns_interval = int(1E9 / self.frequency)
        previous    = self.node.get_clock().now() - Duration(nanoseconds=ns_interval)
        warnings      = []
        while True:
            now = self.node.get_clock().now()
            if (now-previous).nanoseconds >= ns_interval:
                previous = now
                # do something
                for i,spec in enumerate(self.tf_list):
                    #[x,y,z,qw,qx,qy,qz] = self.get_tf(blackboard)
                    loc=tf_bb[i] 
                    if not isinstance(loc,np.ndarray):
                        if not 1 in warnings:
                            get_logger().warning(f"TF2Broadcaster: location in blackboard is not a numpy array: {loc}")
                            warnings.append(1)
                        continue
                    if spec.is_matrix:
                        if loc.shape != (4,4):
                            if not 2 in warnings:
                                get_logger().warning(f"TF2Broadcaster: location in blackboard has wrong shape {loc.shape}\n{loc}")
                                warnings.append(2)
                            continue
                        q=spR.from_matrix(loc[:3,:3]).as_quat(scalar_first=True)
                        p=loc[:3,3]
                        x,y,z,qw,qx,qy,qz = p[0],p[1],p[2], q[0], q[1],q[2],q[3]
                    else:
                        if loc.shape != (7,):
                            if not 3 in warnings:
                                get_logger().warning(f"TF2Broadcaster: location in blackboard has wrong shape {loc.shape}\n{loc}")
                                warnings.append(3)
                            continue
                        [x,y,z,qw,qx,qy,qz] = loc
                    self.tf.header.frame_id = spec.wrt_to
                    self.tf.child_frame_id  = spec.of
                    self.tf.header.stamp = now.to_msg()    
                    self.tf.transform.translation.x = float(x)
                    self.tf.transform.translation.y = float(y)
                    self.tf.transform.translation.z = float(z)
                    self.tf.transform.rotation.x = float(qx)
                    self.tf.transform.rotation.y = float(qy)
                    self.tf.transform.rotation.z = float(qz)
                    self.tf.transform.rotation.w = float(qw)
                    self.tf_broadcaster.sendTransform(self.tf)
            yield TICKING

    def exit(self):
        # this is also called when its externally reset or naturally finishes by itself
        return super().exit()


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

        def set_spline_param(bb):
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
                CrospiOutput_v2("output","/my_topic", queue_size=6000, path='../output'),
                TF2Broadcaster_v2("tfbroadcaster",[TFSpec("initial","base_link","../../initial_tool0",is_matrix=False)],frequency=10),
                MarkerPublisher("marker",from_bb_table("../../output",["x_tf","y_tf","z_tf"],600), marker=Marker.LINE_STRIP, frequency=10, marker_id=1),
                MarkerPublisher("marker",from_array(cp.T),marker=Marker.SPHERE_LIST,marker_id=2,frequency=10,frame_id="initial_tool0"),
            ])
        ]))


def wait_for(child:TickingState, blackboard:Blackboard):
    while True:
        outcome = child(blackboard)
        if outcome!=TICKING:
            return outcome
        yield TICKING

# input and output
#
#    from marker publishers   : time series of values (3-elements per row), input
#    from CrospiTask          : time series of values (n-elements per row,  with header row and data part)
#    TF2Listener              :  1 TFSpec or list of individual elements   wrt_to, of, path or callback or value, or array of callbacks
#    TF2Broadcaster           :  1 TFSpec

#   callback, path : for input parameters to node; getter+setter members
#   callback, path : for output parameters to node; value is the default value;   getter+setter members
#   The callback and the paths basically have the same output.
#   function to select columns when there is a header 
   

class MyApplication_v2(GeneratorWithList):

    def __init__(self):
        super().__init__("My_application_v2",[SUCCEED, CANCEL])

        self.move_home = CrospiTask("MoveHome","MoveHome");
        self.listen_tf = TF2Listener("listen",[ TFSpec("base_link","tool0","../initial_tool0",is_matrix=False) ],SUCCEED) # once
        self.spline_motion = Concurrent("concurrent",[
                CrospiTask("MoveBSpline","MoveBSpline",cb= lambda bb: get_path(self,bb,"./spline") ),
                CrospiOutput_v2("output","/my_topic", queue_size=6000, path='../../output'),
                MarkerPublisher("marker", from_bb_table("../../output",[],200),  marker=Marker.LINE_STRIP,frequency=10,marker_id=0),
                TF2Broadcaster_v2("tfbroadcaster",[TFSpec("base_link","initial_tool0","../../initial_tool0",is_matrix=False)],frequency=10)
#                MarkerPublisher("marker",points_from_list(cp),
#                                marker=Marker.SPHERE_LIST,marker_id=1,frequency=10,frame_id="initial_tool0"),
            ])
        self.add_state(self.move_home)
        self.add_state(self.listen_tf)
        self.add_state(self.spline_motion)

    def co_execute(self, blackboard):
        loc=get_path(self,blackboard,"./spline")
        loc["cp_x"] = [ 0., 0.05, 0.1, 0.15, 0.2,  0.25, 0.3]
        loc["cp_y"] = [ 0., 0.0, 0.2,  0.2,  0.2,  0.0,  0.0]
        loc["cp_z"] = [ 0., 0.0, 0.0,  0.0,  0.15, 0.15, 0.15]
        loc["degree"] = 3
        loc["knots"] = linspace_knots(7,3)
        from pprint import pprint

        yield from wait_for(self.move_home,blackboard)
        yield from wait_for(self.listen_tf,blackboard)
        pprint( get_path(self,blackboard,"."))
        pprint(blackboard["local"])

        while True:
            yield from wait_for(self.spline_motion,blackboard)

    
class MyApplication2_old(GeneratorWithList):
    def __init__(self):
        super().__init__("My_application_v2",[SUCCEED, CANCEL])

        self.movehome   = Concurrent("MoveHome", [
            CrospiTask("MoveHome","MoveHome"),
            CrospiOutput_v2("output", "/my_topic", queue_size=1, path="../../output")   # in the context of MyApplication2
        ])

        self.listen_tf = TF2Listener("listen",[
                TFSpec("tool0","base_link","../tool0",is_matrix=True)
            ],SUCCEED) # once

        spline=CrospiTask("MoveBSpline","MoveBSpline")

        self.movespline = EventSequential("wait",
                Timeout_Condition("finish",10.0),
                event_map={"finish":AlwaysOutcome(CANCEL),NO_EVENT:spline}
        )

        # important to add to the list, e.g. for cleanup 
        self.add_state(self.movehome)
        self.add_state(self.listen_tf)
        self.add_state(self.movespline)

    def co_execute(self, blackboard):
        while True:
            print("MOVE_HOME")
            retval=yield from wait_for(self.movehome,blackboard)
            if retval!=SUCCEED:
                break
            loc = get_path(self,blackboard,"./output/last")
            print(loc)
            
            retval=yield from wait_for(self.listen_tf, blackboard)
            if retval!=SUCCEED:
                break
            loc = get_path(self,blackboard,"./tool0")
            print(loc)
            print("MOVESPLINE")
            retval=yield from wait_for(self.movespline,blackboard)
            if retval!=SUCCEED:
                break
            print("FINISHED")
        print("there was an error")



import numpy as np
# main
def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("example_local_task")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    
    blackboard = {}

    blackboard["numpy"]     = np.array([1,2,3.0]).tolist()
    blackboard["numpy_mat"] = np.array( [[1,2,3.0],[2.0,1,2]] ).tolist()

    load_task_list("betfsm_demos_tasks.json",blackboard)

    # running MySequence() and cleaning up when CTRL_C is pressed        
    nominal_sm = MyApplication()
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
