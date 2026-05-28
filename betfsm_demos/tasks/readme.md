## Adaptations for parameters:

See [task_requirements2.lua](task_requirements2.lua)


## translation of original parameters towards 


|            new var                  |       relation with old def |  path or URI | type | in task requirements | in python function |
| -- | -- | -- | -- | -- | -- |
| filepath_task_lua |  **from lua debug** |    path | lua |  | YES |
| filepath_task_schema_json            | to be written to this file | path | schema | YES
| uri_task_lua      |   "$["..  application_name .. "]/task_specifications/libraries/" .. lib_directory_name .. "/task_specifications/" .. filename_lua **                **lua debug + __APPLICATION_NAME** | URI  | lua  | YES
| filepath_task_library_json    | deduced from _LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA  | path | json | YES |


Make sure to install your directories to share, see setup.py

