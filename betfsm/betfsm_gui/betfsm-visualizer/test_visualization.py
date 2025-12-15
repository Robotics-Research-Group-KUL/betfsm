#!/usr/bin/env python3

import json
import sys
import os

# Add the current directory to Python path so we can import the visualizer module
import asyncio
import websockets
import threading
import time
from backend.jsonvisitor import JsonVisitor
from backend.app import BeTFSMRunnerGUI
from betfsm.betfsm import (
    Sequence, ConcurrentSequence, TimedWait, TimedRepeat, 
    Message, SUCCEED, Generator, Blackboard, TickingState
)
from betfsm.logger import get_logger

def build_test_tree():
    """Build a simple test tree"""
    sm = Sequence("test_sequence", [
        Message(msg="Test message 1"),
        TimedWait("wait_1", 1.0),
        Message(msg="Test message 2"),
        ConcurrentSequence("concurrent_test", [
            Sequence("seq1", [Message(msg="Sub message 1"), TimedWait("wait_2", 0.5)]),
            Sequence("seq2", [Message(msg="Sub message 2"), TimedWait("wait_3", 0.5)])
        ])
    ])
    return sm

def test_json_visitor():
    """Test the JSON visitor to see what data format it produces"""
    sm = build_test_tree()
    visitor = JsonVisitor()
    sm.accept(visitor)
    result = visitor.result()
    
    print("JSON Visitor Output:")
    print(json.dumps(result, indent=2))
    
    # Check if we can find parent relationships
    nodes = result.get('nodes', [])
    root_id = result.get('rootId')
    
    print(f"\nRoot ID: {root_id}")
    print(f"Number of nodes: {len(nodes)}")
    
    # Test parent finding logic
    for node in nodes:
        print(f"Node {node['id']} ({node['name']}) - Children: {node.get('children', [])}")
        
        # Find parent
        parent_id = None
        for other_node in nodes:
            if other_node.get('children') and node['id'] in other_node['children']:
                parent_id = other_node['id']
                break
        print(f"  -> Parent: {parent_id}")

def run_test_machine():
    """Run the state machine in a background thread"""
    sm = build_test_tree()
    bb = {}
    runner = BeTFSMRunnerGUI(sm, bb, frequency=1.0, debug=True, display_active=True)
    runner.run()

if __name__ == "__main__":
    print("Testing JSON visitor data format...")
    test_json_visitor()
    
    print("\n" + "="*50)
    print("Starting state machine (will run for a few seconds)...")
    
    # Run the machine in a thread for a short time
    machine_thread = threading.Thread(target=run_test_machine, daemon=True)
    machine_thread.start()
    
    time.sleep(3)  # Let it run for 3 seconds
    
    print("\nTest completed!")
