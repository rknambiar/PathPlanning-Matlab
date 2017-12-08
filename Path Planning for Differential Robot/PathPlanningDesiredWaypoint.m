
% Path Planning for a differential Robot

path = [2.00    1.00;
    1.25    1.75;
    5.25    8.25;
    7.25    8.75;
    11.75   10.75;
    12.00   10.00];

%Set current Location, initial orientation
%and goal location
robotCurrentLocation  = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;

%Set Current Pose
robotCurrentPose = [robotCurrentLocation  initialOrientation];

% Initialize the Robot Simulator
robotRadius = 0.4;
robot  = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);


%Visualize path
plot(path(:,1), path(:,2), 'k--d')
xlim([0 13])
ylim([0 13])

%Define path following controller
controller = robotics.PurePursuit

%Give path to controller
controller.Waypoints = path;

%Set controller parameters
controller.DesiredLinearVelocity = 0.5; %0.3
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.6; %0.5 for 0.3 linear velocity

goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation  - robotGoal);

controlRate = robotics.Rate(10);

while(distanceToGoal > goalRadius)
   
    %Compute the controller outputs
    [v, omega]  = step(controller,robot.getRobotPose);
    
    %Simulate the robot
    drive(robot, v, omega);
    
    %Extract current location from current pose
    robotCurrentPose = robot.getRobotPose;
    
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    
end

delete(robot)






























