# statemachine_visitor.py 
#
# Copyright (C) Erwin Aertbeliën, 2026
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


from betfsm import (
    TICKING, ABORT,
    TickingState, GeneratorWithList, GeneratorWithState,TickingStateMachine,Visitor
)

import re
from typing import List
import uuid
import re


def statemachine_graphviz(sm:TickingStateMachine):
    graphviz_prefix = """
    digraph StateMachine {
        rankdir=TB;
        splines=true;
        nodesep=0.7;
        ranksep=0.4;

        {
            rank=sink;
            annotation [shape=none, fontname="Helvetica",fontsize=12,label="ABORT and TICKING outcomes exit the state machine unless specified explicitly"];
        }

        // --- Default node style ---

        edge [
            fontname="Helvetica",
            fontsize=9,
            color="#333333",
            arrowsize=0.8
        ];

        // --- Pseudo-state style (black filled circle) ---
        node [shape=circle, width=0.2, height=0.2, label="", style=filled, fillcolor=black] entry;
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


    def graphviz_node(state):
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
    def graphviz_unknown_node(uid,name):        
        return f"""
        "{uid}" [
            fillcolor="#ff6b6b",
            color="#b33939",
            label=<
                <table border="0" cellborder="0" cellspacing="0" align="left">
                    <tr><td><b>{name}</b></td></tr>
                    <tr><td align="left">UKNOWN TARGET</td></tr>
                </table>
            >
        ];
        """    
    def graphviz_unknown_outcome_node(uid,name):        
        return f"""
        "{uid}" [
            fillcolor="#ff6b6b",
            color="#b33939",
            label="unknown outcome"
        ];
        """   
    
    # error &rarr;  success
    def graphviz_arrow_between_nodes(from_uid, to_uid, label):
        return f"""
        "{from_uid}" -> "{to_uid}" [
            label=<   {label}   >
        ];
        """
    
    graphviz_exit = """
        { node [shape=circle, width=0.2, height=0.2, label="", style=filled, fillcolor=black] exit;}
    """
    result = graphviz_prefix
    for state in sm.states_ordered:
        result = result + graphviz_node(state)
    exit_encountered=False

    for state in sm.states_ordered:
        transitions = sm.states[state.name]["transitions"]
        outcome_set = {s for s in state.get_outcomes()}
        for outcome in outcome_set:
            unknown_outcome = ""
            if outcome in transitions:
                target = transitions[outcome]
            else:
                unknown_outcome = outcome
                target = outcome # try to further handle it
                # unhandled outcome               
            if target in transitions:
                # transition redirect outcome to target
                target = transitions[target]
                result = result + graphviz_arrow_between_nodes(state.uid,state.uid,f"{outcome} &rarr;  {target}")
            elif target in sm.get_outcomes():
                if not target==TICKING and not target==ABORT:  
                    # transition from state to exit
                    if not exit_encountered:
                        result = result + graphviz_exit;
                        exit_encountered=True
                    result = result + graphviz_arrow_between_nodes(state.uid,"exit",f"{outcome}")                                    
            elif target in sm.states: 
                # transition from state to target state
                result = result + graphviz_arrow_between_nodes(state.uid,sm.states[target]["state"].uid,f"{outcome}")
            else:
                # transition from state to unknown target state with name target
                if unknown_outcome == "":
                    uid = str(uuid.uuid4())
                    result =  result + graphviz_unknown_node(uid,target)
                    result = result  + graphviz_arrow_between_nodes(state.uid,uid,f"{outcome} &rarr;  {target}")
                else:
                    uid = str(uuid.uuid4())
                    result =  result + graphviz_unknown_outcome_node(uid,target)
                    result = result  + graphviz_arrow_between_nodes(state.uid,uid,f"{unknown_outcome}")
    start = sm.get_start_state()
    result = result  + graphviz_arrow_between_nodes("entry",sm.states[start]["state"].uid,f"")    
    result = result + graphviz_postfix
    
    return result

class StateMachineVisitor(Visitor):

    def __init__(self, name_filter=None):
        """
        name_filter: string a name of a single state machine

        """
        self.nodes = {}
        self.root_id = None

        # Normalize filters
        self.name_filter = name_filter
        self.resultstr = None
    def pre(self, state) -> bool:
        if state.name == self.name_filter:
            self.resultstr = statemachine_graphviz(state)
        return self.resultstr is None       
    def post(self, state):
        pass
    def result(self):
        return self.resultstr


