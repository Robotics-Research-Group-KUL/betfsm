--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
-- 
-- Code made based on Erwin Aertbeliën's code. 
--
--  GNU Lesser General Public License Usage
--  Alternatively, this file may be used under the terms of the GNU Lesser
--  General Public License version 3 as published by the Free Software
--  Foundation and appearing in the file LICENSE.LGPLv3 included in the
--  packaging of this file. Please review the following information to
--  ensure the GNU Lesser General Public License version 3 requirements
--  will be met: https://www.gnu.org/licenses/lgpl.html.
-- 
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Lesser General Public License for more details.

require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")

task_description = "Moves in joint space to a target pose specified using joint angles."

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity rad/s", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration rad/s^2", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.array({name="target_joints", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, description="Array with target angles. Its values correspond to the defined robot.robot_joints in the same order", required=true, minimum = -360, maximum=360, minItems = 1}),
    reqs.params.enum({name="units", type=reqs.enum_types.string, default="radians", description="Units to be used for specifying the joints", required=false, accepted_vals = {"degrees","radians"}}),
    reqs.params.scalar{name="dwell_time", description="time to remain still after motion is finished", default = 0.1, required=false, minimum=0},
})

robot = reqs.robot_model({
    -- "tcp_frame",
    -- "forearm",
    --Add all frames that are required by the task specification
})


-- -- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
dwell_time = param.get("dwell_time")

target_joints = param.get("target_joints")


if #robot.robot_joints ~= #target_joints then
    error("The number of robot.robot_joints (" .. tostring(#robot.robot_joints) .. ") and the specified number of target_joints (" .. tostring(#target_joints) ..  ") must coincide")
end


units = param.get("units")
-- print("units: ",units)
target_joints = reqs.adapt_to_units(target_joints,units)

-- ========================================= VELOCITY PROFILE ===================================

mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value


for i=1,#robot.robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot.robot_joints[i])
    mp:addOutput( initial_value(time, current_jnt[i]), constant(target_joints[i]), maxvel, maxacc)
end




duration = get_duration(mp)
-- print(duration:value())

-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
tracking_error = {}
for i=1,#robot.robot_joints do
    tgt[i]        = get_output_profile(mp,i-1)
    Constraint{
        context=ctx,
        name="joint_trajectory"..i,
        expr= current_jnt[i] - tgt[i] ,
        priority = 2,
        K=3
    };
    tracking_error[i] = current_jnt[i] - tgt[i]
end


Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-duration -constant(dwell_time)
}

-- Monitor {
--     context = ctx,
--     name    = "time_elapsed",
--     expr    = time,
--     upper   = 1.0,
--     actionname = "print",
--     argument = "addtional argument"
-- }




ctx:setOutputExpression("time",time)

for i=1,#robot.robot_joints do
    ctx:setOutputExpression("jpos"..i,current_jnt[i])
    ctx:setOutputExpression("tracking_error_j"..i,tracking_error[i])
end
