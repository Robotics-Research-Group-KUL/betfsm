
--
-- BSpline 
-- 

--- #knots = #cp + degree -1
--- multiplicity start and end == degree
--- different from traditional B-splines, they have $knots = #cp + degree + 1 

require("context")
require("geometric")
bs = require("bspline")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")

-- ========================================= PARAMETERS ===================================
task_description = [[
A first eB-spline executed w.r.t. the starting frame, only considers translations.
Allows for arbitrary number of knots
Note : in eTaSL: #knots = #cp + degree -1  (i.e. minus 1 <-> traditional definition)
]]

param = reqs.parameters(task_description,{
    reqs.params.array({name="cp_x", type=reqs.array_types.number, default={0,0.1}, description="Array with control points for x-coordinate", required=true, maximum=10, minItems = 2}),
    reqs.params.array({name="cp_y", type=reqs.array_types.number, default={0,0.1}, description="Array with control points for y-coordinate", required=true, maximum=10, minItems = 2}),
    reqs.params.array({name="cp_z", type=reqs.array_types.number, default={0,0.1}, description="Array with control points for z-coordinate", required=true, maximum=10, minItems = 2}),
    reqs.params.array({name="knots", type=reqs.array_types.number, default={0,1}, description="Array with knots, #knots = #cp + degree", required=true, maximum=10, minItems = 2}),
    reqs.params.scalar({name="degree", description="Degree of the B-splines", default = 1, required=false}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false})
})

knots = param.get("knots")
cp_x  = param.get("cp_x")
cp_y  = param.get("cp_y")
cp_z  = param.get("cp_z")
degree = param.get("degree")


-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
T_world_taskframe = robot.getFrame(param.get("task_frame"))
-- =============================================================================================================


print("======== 1 ========")

-- s = Variable{context=ctx, name="s",vartype='feature'}
s = (1-cos(time))/2


x = bs.generate_spline(s, knots, cp_x, degree)
y = bs.generate_spline(s, knots, cp_y, degree)
z = bs.generate_spline(s, knots, cp_z, degree)



T_world_base     = initial_value(time, T_world_taskframe)

T_base_taskframe = inv(T_world_base)*T_world_taskframe
p_base_taskframe = origin(T_base_taskframe)

print("======== 2 ========")
Constraint{
    context  = ctx,
    name     = "sine direction constraint",
    expr     = p_base_taskframe - vector(x,y,z),
    K        = 4,
    weight   = 1,
    priority = 2
}

Constraint{
    context  = ctx,
    name     = "sine orientation constraint",
    expr     = rotation(T_base_taskframe),
    K        = 4,
    weight   = 1,
    priority = 2
}


print("======== 3 ========")
Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=2*pi,
        actionname='exit',
        expr=time
}



task_frame = T_world_taskframe

quat_tf = toQuat(rotation(task_frame))
ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
ctx:setOutputExpression("qx_tf",coord_x(vec(quat_tf)))
ctx:setOutputExpression("qy_tf",coord_y(vec(quat_tf)))
ctx:setOutputExpression("qz_tf",coord_z(vec(quat_tf)))
ctx:setOutputExpression("qw_tf",w(quat_tf))
ctx:setOutputExpression("tst",constant(1.0))
ctx:setOutputExpression("base",T_base_taskframe)




