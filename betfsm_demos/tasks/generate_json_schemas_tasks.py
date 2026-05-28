#!/bin/env python3
from pathlib import Path
import subprocess
import ament_index_python as aip
from dataclasses import dataclass,field
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass


def find_ros2_package_root(start_file_path: str | Path) -> Path | None:
    """
    Crawls up the directory tree from a given file path to find the 
    root directory of the ROS 2 package.
    
    Returns the Path to the root directory, or None if package.xml isn't found.
    """
    current_path = Path(start_file_path).resolve()
    if current_path.is_file():
        current_path = current_path.parent
    while current_path != current_path.parent:
        # Check if this directory contains the ROS 2 package manifest
        if (current_path / "package.xml").exists():
            return current_path
        current_path = current_path.parent
    return None

def get_ros2_package_name(package_root: str | Path) -> str | None:
    """Reads the package.xml at the given root directory and extracts
    the official ROS 2 package name.
    """
    manifest_path = Path(package_root).resolve() / "package.xml"
    if not manifest_path.exists():
        return None
    try:
        tree = ET.parse(manifest_path)
        root = tree.getroot()
        name_element = root.find("name")
        if name_element is not None and name_element.text:
            return name_element.text.strip()
    except (ET.ParseError, PermissionError) as e:
        print(f"Error parsing package.xml at {manifest_path}: {e}")
        return None
    return None


@dataclass
class Library:
    task_library:   Path                # (resolved) location of task_library.json file describing name and version of library
    tasks_lua:      list[Path]          # (resolved) locaton of *.etasl.lua specifications.
    tasks_lua_ref:  list[str]           # references to task_lua (using $[..])    
    tasks_schema:   list[Path] = None   # (resolved) location of *.etasl.json schema definition describing parameters of specification.

@dataclass    
class Libraries:
    task_libraries:  list[Library] = None  # list of Library
    task_schema_loc: Path          = None  # location of task-schema.json **file**

def generate_json_schema_task(filepath_task_lua, filepath_task_schema_json, uri_task_lua, filepath_task_library_json):
    """Generate a schema for the specified etasl task specification

    Parameters
    ----------
    filepath_task_lua : string
        path to the task description (*.etasl.lua file)
    filepath_task_schema_json : string
        path to write the the json schema to that describes the task's parameters
    uri_task_lua : string
        the URI to refer to the original task description file (can contain $[...] references)
    filepath_task_library_json : string
        the task library json file where the name and version number of the library can be found

    Returns
    -------
    number
        returns error code, 0 means everything ok.
    """
    arg=f"require('task_requirements2');\
        _GENERATE=true; \
        _FILEPATH_TASK_SCHEMA_JSON='{filepath_task_schema_json}';\
        _URI_TASK_LUA='{uri_task_lua}';\
        _FILEPATH_TASK_LIBRARY_JSON='{filepath_task_library_json}'; \
        dofile('{filepath_task_lua}');\
        print('finished generating {filepath_task_schema_json}') "
    result = subprocess.run(["lua","-e", arg],check=False, capture_output=True,text=True)
    print("return code : ",result.returncode)
    print(result.stdout)
    print(result.stderr)
    return result.returncode

def get_libraries_and_tasks(location:Path|str):
    """get all task libraries and tasks in a directory subtree

    Parameters
    ----------
    location : Path|str
        location of the subtree

    Returns
    -------
    Libraries
        a datastructure of type Libraries with all libraries and tasks
    """
    location = Path(location).resolve()
    package_root = find_ros2_package_root(location)
    package_name = get_ros2_package_name(package_root)
    task_libraries=location.glob("**/task_library.json")
    libs=[]
    for L in task_libraries:
        tasks = L.parent.rglob("*.etasl.lua")
        tasks_lua     = []
        tasks_lua_ref = []
        for task in tasks:
            tasks_lua.append(task)
            tasks_lua_ref.append( "$["+package_name+"]/"+task.relative_to(package_root).as_posix() )
        libs.append(
            Library(
                task_library=L,
                tasks_lua     = tasks_lua,
                tasks_lua_ref = tasks_lua_ref 
            )
        )    
    return Libraries(task_libraries=libs)



def use_flat_layout(libs:Libraries, task_schema_loc: Path|str):    
    """determines location using the flat layout strategy

    Parameters
    ----------
    libs : Libraries
        data structure obtained with `get_libraries_and_tasks(...)`
    task_schema_loc : Path | str
        location where you want the `tasks_schema.json` file to be

    Returns
    -------
    Libraries
        Updated libs data-structure
    """
    for lib in libs.task_libraries:
        lib.tasks_schema = []
        for p in lib.tasks_lua:
            lib.tasks_schema.append(p.with_suffix('.json'))
    libs.task_schema_loc = Path(task_schema_loc).resolve()
    return libs


def use_hierarchal_layout( libs:Libraries, task_schema_loc:Path):
    """determines location using the hierarchical layout strategy

    Parameters
    ----------
    libs : Libraries
        data structure obtained with `get_libraries_and_tasks(...)`
    task_schema_loc : Path | str
        location where you want the `tasks_schema.json` file to be

    Returns
    -------
    Libraries
        Updated libs data-structure
    """    
    for lib in libs.task_libraries:
        lib.tasks_schema = []
        for p in lib.tasks_lua:
            json_file  = p.parent.parent / "task_json_schemas" / p.with_suffix('.json').name
            lib.tasks_schema.append(json_file)
    libs.task_schema_loc = Path(task_schema_loc).resolve()
    return libs



def generate_task_schemas(libs:Libraries) -> list[str]:
    """call lua to generate the individual json schema's to verify the arguments to the
       eTaSL tasks

    Parameters
    ----------
    libs : Libraries
         data structure obtained with `get_libraries_and_tasks(...)`

    Returns
    -------
    list[str]
        list of strings indicating `$ref` references to the individual json schema's
    """
    #task_specs=tasks_schema["properties"]["tasks"]["items"]["properties"]["task_specification"]["oneOf"]
    task_specs=[]
    #base = libs.task_schema_loc.parent 
    for lib in libs.task_libraries:
        filepath_task_library_json = lib.task_library.as_posix()
        for i in range(len(lib.tasks_lua)):                        
            filepath_task_schema_json  = lib.tasks_schema[i]
            filepath_task_schema_json.parent.mkdir(parents=True,exist_ok=True) # ensure dir exists
            generate_json_schema_task(
                filepath_task_lua          = lib.tasks_lua[i].as_posix(), 
                filepath_task_schema_json  = filepath_task_schema_json.as_posix(),
                uri_task_lua               = lib.tasks_lua_ref[i],
                filepath_task_library_json = filepath_task_library_json)            
            # use relative path w.r.t. the tasks-schema.json file.
            #task_specs.append(  filepath_task_schema_json.relative_to(base).as_posix() )
    return task_specs             


def get_task_schemas(libs:Libraries) -> list[str]:
    """get a list of task schema's, relative to the location of tasks-schema.json

    Parameters
    ----------
    libs : Libraries
         data structure obtained with `get_libraries_and_tasks(...)`

    Returns
    -------
    list[str]
        list of `$ref` schema references to the task schemas.
    """    
    task_specs=[]
    base = libs.task_schema_loc.parent 
    for lib in libs.task_libraries:
        for i in range(len(lib.tasks_lua)):
            task_specs.append( {"$ref":lib.tasks_schema[i].relative_to(base).as_posix()} )
    return task_specs

def get_robot_specifications_from_source(robot_spec_dir:Path|str) -> list[str]:
    """find a list of robot specifications

    Parameters
    ----------
    robot_spec_dir : Path | str
        directory location where to look for the robot specifications

    Returns
    -------
    list[str]
        list of strings indicating references to robot specifications that
        can contain package references using "$[..]"
    """
    robot_specs=[]
    robot_spec_dir = Path(robot_spec_dir).resolve()
    root           = find_ros2_package_root(robot_spec_dir).resolve()
    package_name   = get_ros2_package_name(root)
    for p in robot_spec_dir.glob("*.etasl.lua"):
        robot_specs.append(f"$[{package_name}]/{p.relative_to(root).as_posix()}")
    return robot_specs
    

def get_robot_specifications_from_package(package_name:str, relative_path:Path|str)->list[str]:
    """find a list of robot specifications (from package + relative path)

    Parameters
    ----------
    package_name : str
        package name
    relative_path : Path | str
        relative path

    Returns
    -------
    list[str]
        list of strings indicating references to robot specifications that
        can contain package references using "$[..]"
    """
    robot_specs=[]
    root           = aip.get_package_share_path(package_name).resolve()
    robot_spec_dir = root / Path(relative_path)
    for p in robot_spec_dir.glob("*.etasl.lua"):
        robot_specs.append(f"$[{package_name}]/{p.relative_to(root).as_posix()}")
    return robot_specs


def generate_tasks_schema(tasks_schema_loc:Path|str,list_of_tasks:list[str], list_of_robot_refs:list[str]):
    """generates the `tasks-schema.json` file at specified location.  This file is used to define
       the actual instantiations and parameters in a json file of your choice.

    Parameters
    ----------
    tasks_schema_loc : Path | str
        location of the tasks_schema file
    list_of_tasks : list[str]
        lists of tasks
    list_of_robot_refs : list[str]
        list of robot references (get_robot_specifications_from_.... )
    """    
    jsonmodel = json.loads("""
        {
            "$schema":"http://json-schema.org/draft-06/schema",
            "$id":"task-schema.json",
            "title":"Schema for configuration of tasks",
            "type":"object",
            "description":"Schema that enables all etasl task specifications in all the libraries installed within this application package for the creation of tasks",
            "properties": {
                "tasks": {
                    "title":"Tasks",
                    "description":"Tasks (i.e. instances of task specifications) with specific parameters based on the application at hand.",
                    "type" : "array",
                    "items" : {
                        "type": "object",
                        "properties": {
                            "name":{
                                "description":"Name of the task (unique to the task instance)",
                                "type":"string"
                            },
                            "robot_specification_file":{
                                "description":"(optional) If overriding the default_robot_specification, provide the name of the etasl lua file containing the robot specification.",
                                "type": "string",
                                "pattern": ".*\\\.lua$",
                                "examples":  ["$[crospi_application_template]/robot_models/robot_specifications/kuka_iiwa.etasl.lua", "$[crospi_application_template]/robot_models/robot_specifications/ur10.etasl.lua", "$[crospi_application_template]/robot_models/robot_specifications/ur10e.etasl.lua" ] 
                            },
                            "task_specification":{
                                "oneOf": []
                            }
                        },
                        "required": ["name","task_specification"]
                    }
                }
            },
            "required": ["tasks"]
        }
    """)
    item_properties = jsonmodel["properties"]["tasks"]["items"]["properties"]
    item_properties["task_specification"]["oneOf"] = list_of_tasks
    item_properties["robot_specification_file"]["examples"] = list_of_robot_refs
    with open(tasks_schema_loc,"w") as fp:
        json.dump(jsonmodel,fp,indent=4)



from pprint import pprint

tasks_schema_loc = Path("./tasks-schema.json")

libs = get_libraries_and_tasks('betfsm_demos_lib')
libs = use_flat_layout(libs,"./tasks_schema.json")
generate_task_schemas(libs)
list_of_tasks = get_task_schemas(libs)
# list_of_robot_refs = get_robot_specifications_from_source("../../../crospi_application_template/robot_models/robot_specifications")
list_of_robot_refs = get_robot_specifications_from_package("crospi_application_template", 'robot_models/robot_specifications')
generate_tasks_schema(tasks_schema_loc,list_of_tasks, list_of_robot_refs)




