
%Set Robot Params
robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('simpleMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

%PRM Path planning Algorithm
mapInflated = copy(robot.Map);
inflate(mapInflated, robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;

%Compute path from given start and end
startLocation = [2.0 1.0];
endLocation = [12.0 10.0];
path = findpath(prm, startLocation, endLocation)

show(prm, 'Map', 'off', 'Roadmap', 'off');

%Define path following controller
controller = robotics.PurePursuit

%Give path to controller
controller.Waypoints = path;

%Set controller parameters
controller.DesiredLinearVelocity = 0.5; %0.3
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.6; %0.5 for 0.3 linear velocity

%set robotlocation and orientation
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

initialOrientation = 0;

robotCurrentPose = [robotCurrentLocation initialOrientation];

robot.setRobotPose(robotCurrentPose);
distanceToGoal = norm(robotCurrentLocation - robotGoal);

goalRadius = 0.1;

controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = step(controller,robot.getRobotPose);

    % Simulate the robot using the controller outputs
    drive(robot, v, omega);

    % Extract current location information from the current pose
    robotCurrentPose = robot.getRobotPose;

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);
end

drive(robot, 0, 0);

%delete(robot);
