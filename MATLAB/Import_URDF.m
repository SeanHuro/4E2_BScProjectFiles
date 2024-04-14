%Imports URDF file into MATLAB as rigidBodyTree
robot = importrobot('irb120.urdf');
robot.DataFormat = 'row';
home = homeConfiguration(robot);

%Analytical Solver
    %Selects kinematic group for the IK solver
    aik = analyticalInverseKinematics(robot);
    
    %Generates the analytical IK function
    generateIKFunction(aik, 'robotIK');
    
    %Specifies desired end effector pose
    eePosition = [0.2 0.4 0.4];
    eePos = trvec2tform(eePosition);
    eeRotation = [1 0 0 0];
    eeRot = quat2tform(eeRotation);
    eePose = eePos * eeRot;
    
    %Solving all possible joint configurations for desired pose
    tic
    ikConfig = robotIK(eePose); 
    toc
    
    %Displays robot in 1st configuration
    figure(1);
    subplot(1,2,1)
    show(robot,ikConfig(1,:));
    hold on
    %Plots the end effectors coordinate frame
    plotTransforms(eePosition,tform2quat(eePose))
    hold off
    title('(a)', 'Position',[0.45 0.3 -0.32])

%Numerical Solver
    %Initialises IK solver using Levenberg-Marquardt Method
    ik = inverseKinematics("RigidBodyTree",robot, "SolverAlgorithm",'LevenbergMarquardt');
    %Define weightings for IK solver. This will tell the solver which
    %parameters of the pose to consider when solving
    weights = [1 1 1 1 1 1];
    %Define initial guess as robot's home configuration. (All joints = 0°)
    initialguess = home;
    
    %Solves IK for desired pose
    tic
    [configSoln,solnInfo] = ik("link_6",eePose,weights,initialguess);
    toc
    
    figure(1);
    subplot(1,2,2)
    %Plots Robot pose
    show(robot,configSoln,PreservePlot=false);
    hold on
    %Plots the end effectors coordinate frame
    plotTransforms(eePosition,tform2quat(eePose))
    hold off 
    xlim([-0.3 0.5])
    ylim([-0.3 0.3])
    zlim([-0.1 0.8])
    title('(b)','Position',[0.45 0.3 -0.32])
