r = Quadrotor();

if(~exist('trajectoryLibrary.mat'))
    %generate trajectory library
    disp('building trajectory library...')
    tgen = WaypointTrajectoryLibraryGenerator(r);
    tgen = tgen.setCyclicCoordinateIndexes([1,2,3,6]);

    x0 = Point(r.getStateFrame);
    x0.base_z = .5;
    u0 = double(nominalThrust(r));

    tgen = tgen.setInitialState(double(x0));
    tgen = tgen.setInitialInput(u0);

    relativePositions = [cos(pi/6*(-2:2));sin(pi/6*(-2:2))];
    start = [1;0];

    tgen = tgen.addWaypoint([start;0.5;0]);
    
    for i = 1:size(relativePositions,2)
        start = start + relativePositions(:, i);
        tgen = tgen.addWaypoint([start;0.5;0]);
    end

    xf = x0;
    xf.base_x = start(1) + 1;

    tgen = tgen.setFinalState(double(xf));
    tgen = tgen.setFinalInput(u0);

    tgen = tgen.setTrajectoryLength(11);
    trajLib = tgen.generateTrajectories();
    

    save 'trajectoryLibrary.mat' trajLib
else
    disp('loading trajectoryLibrary.mat...');
    load('trajectoryLibrary.mat');
    trajLib = trajLib.setupFrames(r);
end

disp('stabilizing trajectories...');
Q = diag([10*ones(6,1);ones(6,1)]);
R = 0.1*eye(4);
Qf = diag([100*ones(6,1);ones(6,1)]);    
trajLib = trajLib.stabilizeTrajectories(r, Q, R, Qf);

funLib = FunnelLibrary(trajLib);
funLib.computeFunnels(r, 95, 12);

for i = 1:numel(xtrajs)

Vxframe = V_funnel{i}.inFrame(r.getStateFrame());
options.plotdims = [1 2];
options.x0 = xtraj;
options.ts = ts;
options.inclusion = 'projection';
plotFunnel(Vxframe,options);
hold on;
fnplt(xtrajs{i},[1 2]); 

end


disp('done');