%Imports URDF file into MATLAB as rigidBodyTree
robot = importrobot('irb120.urdf');
robot.DataFormat = 'column';
home = homeConfiguration(robot);

% Joint Space
    tpts = 0:4;
    sampleRate = 20;
    tvec = tpts(1):1/sampleRate:tpts(end);
    numSamples = length(tvec);
    
    waypoints = [home ikConfig(1,:)'];
    
    rng default
    [q,qd] = trapveltraj(waypoints,numSamples);
    
    figure(1);
    set(gcf,"Visible","on");
    rc = rateControl(sampleRate);
    for i = 1:numSamples
        show(robot,q(:,i),FastUpdate=true,PreservePlot=false);
        eePosit(i,:) = tform2trvec(getTransform(robot,q(:,i),"link_6"));
        waitfor(rc);
    end
    hold on
    plot3(eePosit(:,1),eePosit(:,2),eePosit(:,3),'b-','LineWidth',3);
    xlim([-0.3 0.6]);
    ylim([-0.3 0.6]);
    zlim([0 0.8]);

% Task Space
    Twaypoints = [0.374 0 0.63; 0.2 0.4 0.4]';
    Timepoints = linspace(tvec(1),tvec(end),3);
    [pos,vel] = trapveltraj(Twaypoints,numSamples);

    rng(0);
    Nik = inverseKinematics('RigidBodyTree',robot, "SolverAlgorithm",'LevenbergMarquardt');
    Nik.SolverParameters.AllowRandomRestart = false;
    weight = [0.2 0.2 0.2 1 1 1];
    q = zeros(6,numSamples);
    initialguess = home;

    tic
    for i = 1:size(pos,2)
        targetpose = trvec2tform(pos(:,i)')*rotm2tform([1 0 0; 0 1 0; 0 0 1]);
        configSoln = robotIK(targetpose);
        %[configSoln ,solnInfo] = Nik('link_6', targetpose,weight,initialguess);
        q(:,i) = configSoln(1,:)';
        initialguess = q(:,i);
    end
    toc

    eePosit = zeros(numSamples,3);
    figure(2)
    set(gcf,"Visible","on")
    for i = 1:numSamples
        show(robot, q(:,i),FastUpdate=true,PreservePlot=false);
        eePosit(i,:) = tform2trvec(getTransform(robot,q(:,i),"link_6"));
        waitfor(rc);
    end
    hold on
    plot3(eePosit(:,1),eePosit(:,2),eePosit(:,3),'b-','LineWidth',3);
    xlim([-0.3 0.6]);
    ylim([-0.3 0.6]);
    zlim([0 0.8]);

    %%
    figure(3);
    subplot(2,1,1)
    plot(tvec, q);
    subplot(2,1,2)
    plot(tvec, vel);
    