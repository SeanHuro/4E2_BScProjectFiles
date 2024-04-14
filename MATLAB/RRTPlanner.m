%Imports URDF file into MATLAB as rigidBodyTree
robot = importrobot('irb120.urdf');
robot.DataFormat = 'row';
home = homeConfiguration(robot);
    
%RRT Path Planner
rrt = manipulatorRRT(robot,{});
rrt.SkippedSelfCollisions = "parent";

startConfig = home;
goalConfig = ikConfig(1,:);

rng(0);
tic
path = plan(rrt, startConfig, goalConfig);
toc
intPath = interpolate(rrt,path);
clf

rate = rateControl(size(intPath,1));

for i = 1:1:size(intPath,1)
    show(robot,intPath(i,:),"PreservePlot",false, "FastUpdate",true);
    waitfor(rate);
end