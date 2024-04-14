% Define Waypoints as Homogeneous Transforms
    Home = trvec2tform([0.36435 0 0.594])*quat2tform([0.9659 0 0.2588 0]);
    Home2 = trvec2tform([0 -0.36536 0.53941])*quat2tform([0.6672 0.2342 0.2342 -0.6672]);
    Pos1 = trvec2tform([0 -0.47865 0.420])*quat2tform([0.7071 -2.1789e-33 0 -0.7071]);
    Pos2 = trvec2tform([0.150 -0.47865 0.470])*quat2tform([0.7071 -2.1789e-33 0 -0.7071]);
    Pos3 = trvec2tform([-0.150 -0.47865 0.470])*quat2tform([0.7071 -2.1789e-33 0 -0.7071]);
    Pos4 = trvec2tform([-0.150 -0.47865 0.370])*quat2tform([0.7071 -2.1789e-33 0 -0.7071]);
    Pos5 = trvec2tform([0.150 -0.47865 0.370])*quat2tform([0.7071 -2.1789e-33 0 -0.7071]);

    tformWaypoints(:,:,1) = Home;
    tformWaypoints(:,:,2) = Home2;
    tformWaypoints(:,:,3) = Pos1;
    tformWaypoints(:,:,4) = Pos2;
    tformWaypoints(:,:,5) = Pos3;
    tformWaypoints(:,:,6) = Pos4;
    tformWaypoints(:,:,7) = Pos5;

%Imports URDF file into MATLAB as rigidBodyTree
    robot = importrobot('irb120.urdf');
    robot.DataFormat = 'row';
    home = homeConfiguration(robot);

% Find Joint Configurations for all Waypoints
    % Create IK Solvers
    aik = analyticalInverseKinematics(robot);
    generateIKFunction(aik, 'robotIK');

    ik = inverseKinematics("RigidBodyTree",robot, "SolverAlgorithm",'LevenbergMarquardt');
    weights = [1 1 1 1 1 1];
    initialguess = home;

    % Solve IK
    ikConfig = zeros(7,6);
    configSoln = zeros(7,6);
    for i = 1:1:size(tformWaypoints,3)
        x = robotIK(tformWaypoints(:,:,i));
        ikConfig(i,:) = x(1,:);
        [configSoln(i,:),solnInfo] = ik("link_6",tformWaypoints(:,:,i),weights,initialguess);
    end

    configSolnDeg = rad2deg(configSoln);
    ikConfigDeg = rad2deg(ikConfig);

% Create Trajectories for Robot to follow
    % Joint Space Trajectory ( Home to Pos 1)
    tpts = 0:2;
    sampleRate = 20;
    tvec = tpts(1):1/sampleRate:tpts(end);
    numSamples = length(tvec);
    
    waypoints1 = [configSoln(1,:)' configSoln(2,:)' configSoln(3,:)'];  
    [q1,qd] = trapveltraj(waypoints1,numSamples);

    % Task Space Trajectory
    tpts = 0:5;
    sampleRate = 20;
    tvec = tpts(1):1/sampleRate:tpts(end);
    numSamples = length(tvec);

    poswaypoints = [tform2trvec(Pos1)' tform2trvec(Pos2)' ... 
        tform2trvec(Pos3)' tform2trvec(Pos4)' tform2trvec(Pos5)' ...
        tform2trvec(Pos2)'];

    [pos,vel] = trapveltraj(poswaypoints,numSamples);
    release(ik);
    for i = 1:size(pos,2)
        targetpose = trvec2tform(pos(:,i)')*quat2tform([0.7071 -2.1789e-33 0 -0.7071]);
        confSoln = robotIK(targetpose);
        %[confSoln ,solnInfo2] = ik('link_6',targetpose,weights,initialguess);
        q2(:,i) = confSoln(1,:)';
        initialguess = q2(:,i)';
    end

    % Joint Space Trajectory (Pos2 to Home)
    tpts = 0:1;
    sampleRate = 20;
    tvec = tpts(1):1/sampleRate:tpts(end);
    numSamples = length(tvec);

    waypoints1 = [configSoln(4,:)' configSoln(1,:)'];  
    [q3,qd] = trapveltraj(waypoints1,numSamples);

    q = [q1' ;q2'; q3'];

    figure(1);
    set(gcf,"Visible","on");
    grid off
    xlim([-0.3 0.6]);
    ylim([-0.8 0.3]);
    zlim([0 0.8]);
    rc = rateControl(sampleRate);
    for i = 1:1:size(q,1)
        show(robot,q(i,:),FastUpdate=true,PreservePlot=false);
        eePosit(i,:) = tform2trvec(getTransform(robot,q(i,:),"link_6"));
        waitfor(rc);
    end
    hold on
    plot3(eePosit(:,1),eePosit(:,2),eePosit(:,3),'b-','LineWidth',3);

    %%
    figure(2)
    set(gcf,"Visible","on", 'position',[0,0,1000,500]);
    plot(1:1:length(q),q,'LineWidth',2);
    xlabel('Position along Trajectory','FontSize',16);
    ylabel('Joint Angle (rad)','FontSize',16);
    lgd = legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6', ...
        'Location','northwest');
    ylim([-3 4]);
    fontsize(lgd,20,'points');
    
%%
% Create Path
rrt = manipulatorRRT(robot,{});
rrt.SkippedSelfCollisions = "parent";

rng(0);
path1 = plan(rrt, configSoln(1,:), configSoln(2,:));
path2 = plan(rrt, configSoln(2,:), configSoln(3,:));
path3 = plan(rrt, configSoln(3,:), configSoln(4,:));
path4 = plan(rrt, configSoln(4,:), configSoln(5,:));
path5 = plan(rrt, configSoln(5,:), configSoln(6,:));
path6 = plan(rrt, configSoln(6,:), configSoln(7,:));
path7 = plan(rrt, configSoln(7,:), configSoln(4,:));
path8 = plan(rrt, configSoln(4,:), configSoln(1,:));

path1 = interpolate(rrt,path1);
path2 = interpolate(rrt,path2);
path3 = interpolate(rrt,path3);
path4 = interpolate(rrt,path4);
path5 = interpolate(rrt,path5);
path6 = interpolate(rrt,path6);
path7 = interpolate(rrt,path7);
path8 = interpolate(rrt,path8);

mainPath = [path1; path2; path3; path4; path5; path6; path7; path8];

rate = rateControl(size(mainPath,1));

figure(2)
for i = 1:1:size(mainPath,1)
    show(robot,mainPath(i,:),"PreservePlot",false, "FastUpdate",true);
    eePosit(i,:) = tform2trvec(getTransform(robot,mainPath(i,:),"link_6"));
    waitfor(rate);
end
hold on
plot3(eePosit(:,1),eePosit(:,2),eePosit(:,3),'b-','LineWidth',3);

    