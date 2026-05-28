--
-- Follow sine, E. Aertbelien (2026)
-- 
require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements2")

print("start")
-- ========================================= PARAMETERS ===================================
task_description = "This task specification to generate a sine wave  w.r.t. current position"

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="amplitude", description="Amplitude in [m] of the sine", default = 0.1, required=false}),
    reqs.params.scalar({name="frequency", description="Frequency of the sine", default = 0.25, required=false}),
    reqs.params.array({name="direction", description="Direction for the sine wave (will be normalized)", default = {0,0,1}, minItems=3, maxItems=3, type="number",required=false}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),    
    reqs.params.scalar({name="execution_time", description="Duration of the motion, if negative, forever", default=-1, required=false})
})

ampl           = param.get("amplitude")
freq           = constant(param.get("frequency"))
dir_array      = param.get("direction")
execution_time = param.get("execution_time")
dir            = constant( ampl*normalized(Vector(dir_array[1],dir_array[2],dir_array[3])) )
taskframe      = param.get("task_frame")

print("after getparam")

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))
-- =============================================================================================================


T_base_start    = initial_value(time, task_frame)
T_start_current = inv(T_base_start)*task_frame
p_start_current = origin(T_start_current)

Constraint{
    context  = ctx,
    name     = "sine direction constraint",
    expr     = p_start_current - dir*sin(2*pi*freq*time),
    K        = 4,
    weight   = 1,
    priority = 2
}

Constraint{
    context  = ctx,
    name     = "sine orientation constraint",
    expr     = rotation(T_start_current),
    K        = 4,
    weight   = 1,
    priority = 2
}



quat_tf = toQuat(rotation(task_frame))

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
ctx:setOutputExpression("qx_tf",coord_x(vec(quat_tf)))
ctx:setOutputExpression("qy_tf",coord_y(vec(quat_tf)))
ctx:setOutputExpression("qz_tf",coord_z(vec(quat_tf)))
ctx:setOutputExpression("qw_tf",w(quat_tf))





