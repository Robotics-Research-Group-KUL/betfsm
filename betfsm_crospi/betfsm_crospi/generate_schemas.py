#!/bin/env python3
#
# program to generate schema's for .etasl.lua constraint-based task
# specifications and to refer to external libraries.
# Also generates the tasks-schema.json
#
# region Copyright (C) Erwin Aertbeliën, 2024-2026
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
# endregion

from pathlib import Path
import subprocess
import ament_index_python as aip
from dataclasses import dataclass,field
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
import argparse


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


def create_Libraries(location:Path|str):
    """create the Libraries data structure from the directory tree at given location.

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
    """determines location of task schemas using the flat layout strategy

    Parameters
    ----------
    libs : Libraries
        data structure obtained with `get_libraries_and_tasks(...)`
    task_schema_loc : Path | str
        location where you want the `tasks_schema.json` file to be

    Returns
    -------
    Libraries
        Updated Libraries data-structure
    """
    for lib in libs.task_libraries:
        lib.tasks_schema = []
        for p in lib.tasks_lua:
            lib.tasks_schema.append(p.with_suffix('.json'))
    libs.task_schema_loc = Path(task_schema_loc).resolve()
    return libs


def use_hierarchical_layout( libs:Libraries, task_schema_loc:Path):
    """determines location of task schemas using the hierarchical layout strategy
      
    
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


def generate_json_schema_for_task(filepath_task_lua, filepath_task_schema_json, uri_task_lua, filepath_task_library_json,verbose=False):
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
    if verbose:        
        print("\t\t",result.stdout.replace("\n","\n\t\t"))
        print("\t\treturn code : ",result.returncode)
    if result.returncode!=0:
        print(result.stderr)
        print("\t\treturn code : ",result.returncode)
    return result.returncode


def generate_json_schema_for_tasks(libs:Libraries,verbose=False) -> list[str]:
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
        if verbose:
            print("\tFound library at: ",filepath_task_library_json)
        for i in range(len(lib.tasks_lua)):                        
            filepath_task_schema_json  = lib.tasks_schema[i]
            filepath_task_schema_json.parent.mkdir(parents=True,exist_ok=True) # ensure dir exists
            generate_json_schema_for_task(
                filepath_task_lua          = lib.tasks_lua[i].as_posix(), 
                filepath_task_schema_json  = filepath_task_schema_json.as_posix(),
                uri_task_lua               = lib.tasks_lua_ref[i],
                filepath_task_library_json = filepath_task_library_json,
                verbose=verbose)            
            # use relative path w.r.t. the tasks-schema.json file.
            #task_specs.append(  filepath_task_schema_json.relative_to(base).as_posix() )
    return task_specs             


def deduce_task_schemas(package:str, relative_path:Path|str, task_schema_loc: Path|str, verbose=False) -> list[str]:
    """get a list of task schema's, relative to the location of tasks-schema.json for a task library at package + relative path

    Parameters
    ----------
    package : str
        package name
    relative_path : Path | str
        relative path of the subtree in the package
    task_schema_loc : Path | str
        location where you want the `tasks_schema.json` file to be.

    Returns
    -------
    list[str]
        list of `$ref` schema references to the task schemas.
    """
    package_loc = aip.get_package_share_path(package) 
    location = package_loc / Path(relative_path)
    if verbose:
        lst = [ json.relative_to(package_loc).as_posix()     for json in location.rglob("*.etasl.json")     ]
        if len(lst)>0:
            print(f"Found tasks in ROS2 package {package}:")
            for tsk in lst:
                print(f"\t{tsk}")
        else:
            print("No tasks found")
        return [ {"$ref": (location / json).as_posix()}     for json in location.rglob("*.etasl.json")     ]
    else:
        return [ {"$ref": (location / json).as_posix()}     for json in location.rglob("*.etasl.json")     ]


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
        for i in range(len(lib.tasks_schema)):
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
    jsonmodel = json.loads(r"""
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
                           "description" : {
                                "description":"Description of the instantiated skill (optional)",
                                "type": "string"
                           },
                            "robot_specification_file":{
                                "description":"(optional) If overriding the default_robot_specification, provide the name of the etasl lua file containing the robot specification.",
                                "type": "string",
                                "pattern": ".*\\.lua$",
                                "examples":  [] 
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



def generate_task_library_schema(pth:Path):
    task_library_schema = r""" 
{
    "$schema":"http://json-schema.org/draft-06/schema",
    "$id":"task_library.json",
    "title":"eTaSL task specifications libraries",
    "type":"object",
    "description":"Libraries containing etasl task specifications",
    "required": ["name", "version","description","authors","maintainers"],
    "properties": {
        "$schema" : {
            "type":"string"
        },
        "name":{
            "description": "Name of the library",
            "type":"string",
            "pattern": "^[a-zA-Z0-9_]+$"
        },
        "version":{
            "description": "Version of library",
            "type":"string",
            "pattern": "^(\\d+\\.){2}\\d+(-[a-zA-Z]+)?$",
            "default": "0.0.0",
            "examples": [
                "1.0.0-beta"
            ]
        },
        "description":{
            "description": "Text describing the purpose of the library",
            "type":"string"
        },
        "license":{
            "description": "",
            "type":"string",
            "examples":[
                "GNU GPLv3",
                "Apache License 2.0"
            ]
        },
        "authors": {
            "title":"List of authors",
            "description":"List of authors that participated on the development of the library",
            "type" : "array",
            "items" : {
                "$ref" : "#/$defs/author" 
            }
        },
        "maintainers": {
            "title":"List of maintainers",
            "description":"List of maintainers that currently maintain the library",
            "type" : "array",
            "items" : {
                "$ref" : "#/$defs/maintainer" 
            }
        },
        "acknowledgements":{
            "description": "acknowledgements to people who made contributions or to project funding",
            "type":"string"
        }
    },
    "$defs": {
        "author": { 
            "title":"Author",
            "type":"object",
            "description":"Data of an author",
            "required": ["name", "affiliation","email"],
            "properties": {
                "name":{
                    "description": "Name of the author",
                    "type":"string"
                },
                "affiliation":{
                    "description": "Affiliation of the author",
                    "type":"string",
                    "default": "KU Leuven"
                },
                "email":{
                    "description": "Email of the author",
                    "type":"string",
                    "pattern": "^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}$",
                    "examples": [
                        "author_mail@gmail.com"
                    ]
                }
            }
        },
        "maintainer": { 
            "title":"Maintainer",
            "type":"object",
            "description":"Data of an maintainer",
            "required": ["name", "affiliation","email"],
            "properties": {
                "name":{
                    "description": "Name of the maintainer",
                    "type":"string"
                },
                "affiliation":{
                    "description": "Affiliation of the maintainer",
                    "type":"string",
                    "default": "KU Leuven"
                },
                "email":{
                    "description": "Email of the maintainer",
                    "type":"string",
                    "pattern": "^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}$",
                    "examples": [
                        "maintainer_mail@gmail.com"
                    ]
                }
            }
        }
    },
    "additionalProperties":false
}

"""
    with open(pth,"w") as f:
        f.write(task_library_schema)
    return


def old_main():

    from pprint import pprint

    tasks_schema_loc = Path("./tasks-schema.json")


    print("TODO: delete existing .etasl.json files before generating in the library")

    print("----------------------------------------------------------")
    print("until integrated with crospi: ")
    print("adapt LUA_PATH such that task_requirements2.lua can be found")
    print('e.g. export LUA_PATH="$LUA_PATH;/home/eaertbel/Temp/crospi-erwin/crospi_ws/src/betfsm/betfsm_demos/tasks/?.lua')
    print("----------------------------------------------------------")

    libs = create_Libraries('betfsm_demos_lib')
    libs = use_flat_layout(libs,"./tasks_schema.json")
    #pprint(libs)
    generate_json_schema_for_tasks(libs)
    list_of_tasks = get_task_schemas(libs)
    list_of_robot_refs = get_robot_specifications_from_package("crospi_application_template", 'robot_models/robot_specifications')


    # add libraries from crospi_application_template
    list_of_tasks += deduce_task_schemas("crospi_application_template","task_specifications/libraries","./tasks_schema.json")  



    generate_tasks_schema(tasks_schema_loc,list_of_tasks, list_of_robot_refs)


def main(args=None):
    parser = argparse.ArgumentParser(description= "Tool to create and manipulate tasks_schema.json files and *.etasl.lua files that describe task parameters.")   
    subparsers = parser.add_subparsers(dest="command", required=True, help="Available commands")

    # Create the parser for the "create" command
    parser_create = subparsers.add_parser("create", 
        help="Create a new stasks-schema.json file from a directory with task libraries and *.etasl.lua task specifications",
        description="Create a new tasks-schema.json file from a directory with task libraries and *.etasl.lua task specifications"
    )
    layout_group = parser_create.add_mutually_exclusive_group(required=True)
    layout_group.add_argument("--flat", action="store_true", help="Generate *.etasl.json files for a flat directory structure")
    layout_group.add_argument("--hierarchical", action="store_true", help="Generate *.etasl.json files for an hierarchical directory structure")

    parser_create.add_argument("tasks_schema", type=str, help="File path for the tasks-schema.json file")
    parser_create.add_argument("search_path", type=str, help="Directory to (recursively) search for task libraries and task specifications")
    parser_create.add_argument("package", type=str, help="ROS2 package name for the package that contains robot descriptions")
    parser_create.add_argument("localpath", type=str, help="local path inside the ROS2 package that contains the robot descriptions")
    parser_create.add_argument("--verbose","-v", action="store_true",help="Increas verbosity")
    # Create the parser for the "add" command
    parser_add = subparsers.add_parser("add", 
        help="Adds external references to task libraries and *.etal.json parameter schemas ",
        description="Adds external references to task libraries and *.etal.json parameter schemas "
    )
    parser_add.add_argument("tasks_schema", type=str, help="File path to an existing tasks-schema.json file")
    parser_add.add_argument("package", type=str, help="ROS2 package name for the additional task libraries and task specifications")
    parser_add.add_argument("localpath", type=str, help="local path inside the ROS2 package")
    parser_add.add_argument("--verbose","-v", action="store_true",help="Increas verbosity")
 
    parser_writelib = subparsers.add_parser("writelib", 
        help="Write the standard task_library.schema.json to the given location",
        description="Write the standard task_library.schema.json to the given location"
    )
    parser_writelib.add_argument("task_library_schema", type=str, nargs="?",help="File path for the task_library.schema file",default="task_library.schema.json")

    parser_explain = subparsers.add_parser("explain", help="Explains the file-structore for cROSpi task specifications") 
    args = parser.parse_args()
    
    if args.command == "create":
        libs = create_Libraries(args.search_path)
        if args.flat:
            libs = use_flat_layout(libs,args.tasks_schema)
        if args.hierarchical:
            libs = use_hierarchical_layout(libs,args.tasks_schema)        
        generate_json_schema_for_tasks(libs,verbose=args.verbose)
        list_of_tasks      = get_task_schemas(libs)
        list_of_robot_refs = get_robot_specifications_from_package("crospi_application_template", 'robot_models/robot_specifications')
        generate_tasks_schema(args.tasks_schema,list_of_tasks, list_of_robot_refs)
        print(f"{args.tasks_schema} created")
    elif args.command == "add":
        with open(args.tasks_schema,"r") as fp:
            data = json.load(fp)
        # add libraries from crospi_application_template
        list_of_tasks = deduce_task_schemas(args.package,args.localpath,args.tasks_schema,args.verbose)  
        lst = data["properties"]["tasks"]["items"]["properties"]["task_specification"]["oneOf"]
        lst += list_of_tasks
        with open(args.tasks_schema,"w") as fp:
            json.dump(data,fp,indent=4)
    elif args.command == "writelib":
        print("writing the schema for a task library to ", args.task_library_schema)
        generate_task_library_schema(args.task_library_schema)
    elif args.command == "explain":
        txt="""
Crospi/etasl constraint-based task specifications are specified in libraries.
Each library is defined by a subdirectory with a task_library.json file that
specifies name, version and authors of the library.

All *.etasl.lua files in this subdirectory and below belong to the library. For
each *.etasl.lua file a *.etasl.json file is generated that describe the
parameters of this task specifications.

All these tasks with their argument specifications are bundled in a
tasks-schema.json file that defines all available tasks together
with robot definitions.

You can then instantiate tasks in your own .json file: add a first line that
refers to the tasks-schema.json file, i.e.  "$schema":"tasks-schema.json" and
use auto-complete to fill in the task instantiations.  You can then use
load_task_lists from betfsm_crospi to load these definitions in your BeTFSM
tree.   If needed, you can further override the parameters with dynamically
determined parameters in BeTFSM.

Using the "create" command, you can generate your *.etasl.lua files
hierarchically, i.e. you put them in a task_specifications subdirectory of the
library and use the "--hierarchical" option to generate *.etasl.json files in
the task_json_schemas subdirectory.

You can also use the "--flat" option to generate *.etasl.json in the same
directory as the *.etasl.lua files.

Using the "add" command, you can add externally defined task libraries to your
collection of task definitions in tasks-schema.json

The "writelib" command writes a task_library.schema.json for the task_library.json 
that describes your library or libraries.
        """   
        print(txt)
 
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

