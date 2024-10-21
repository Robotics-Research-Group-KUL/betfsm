def dumps_blackboard(blackboard,indent=0):
    """
    returns a string-dump of a (piece of the ) blackboard
    """
    s = ""
    indent += 4
    space = f'{" ":{indent}}'
    if isinstance(blackboard,bool):
        s = f'{blackboard}\n'
    elif isinstance(blackboard,int):
        s = f'{blackboard}\n'
    elif isinstance(blackboard,float):
        s = f'{blackboard}\n'    
    elif isinstance(blackboard,dict):
        s = s + "\n"
        for k,v in blackboard.items():
            s+= space + k + " : " + dumps_blackboard(v,indent)
    elif isinstance(blackboard,str):
        s = f'"{blackboard}"\n'       
    elif isinstance(blackboard,list):
        s = f'{blackboard}\n'             
    else:
        s = f'{type(blackboard)}'
    return s



bb = { "output" : 4, "task": {"a":4,"b":3}}
bb["task"]["c"] = "asdf"

loc = ["output","moving_home"]


print(dumps_blackboard(bb))
