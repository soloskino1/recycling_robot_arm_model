%My recycling robot code. to move items from a conveyor belt to different
%bins depending on each item picked
% SOLOMON AYANSOLA M00881460 Middlesex University


% Defining the robot's DH parameters
L1 = Link('d', 0.1, 'a', 0, 'alpha', pi/2, 'offset', 0);
L2 = Link('d', 0, 'a', 0.3, 'alpha', 0, 'offset', 0);
L3 = Link('d', 0, 'a', 0.2, 'alpha', 0, 'offset', 0);
L4 = Link('d', 0, 'a', 0.35, 'alpha', 0, 'offset', 0);
L5 = Link('d', 0, 'a', 0.6, 'alpha', -pi/2, 'offset', 0);
L6 = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', 0);
%L6 = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', 0);

% Defining the robot using seriallink
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'My recycling robot');

% the robot's home configuration
q0 = [0, 0, 0, 0, 0, 0];%, 0];


% Define the joint limits
robot.qlim = [-pi/2, pi/2; 0, 0.7*pi; -pi/2, 0; -pi, 0; -pi, 0; -pi/2, pi/2];

% Defining the end-effector
tool = transl(0, 0, 0.2);

% Defining the five points given ( the two ends of the conveyor and the top
% of the bins) - converted to meters
points = [0, 1.5, 0;
          0, 0.5, 0;
          0.6, 0, 0.5;
          0.8, 0, 0.6;
          1, 0, 0.7];

% Generate a trajectory for moving the robot from point 1 to point 3
% Using ikcon fuction to calculate the inverse kinematics
q1 = robot.ikcon(tool * transl(points(1, :)), q0);
q3 = robot.ikcon(tool * transl(points(3, :)), q1);
traj1 = jtraj(q1, q3, 10);

% Generate a trajectory for moving the robot from point 2 to point 4
q2 = robot.ikcon(tool * transl(points(2, :)), q3);
q4 = robot.ikcon(tool * transl(points(4, :)), q2);
traj2 = jtraj(q2, q4, 10);

% Generate a trajectory for moving the robot from point 1 to point 5
q5 = robot.ikcon(tool * transl(points(5, :)), q4);
traj3 = jtraj(q4, q1, 10);
traj4 = jtraj(q1, q5, 10);

% Concatenate the trajectories
traj = [traj1; traj2; traj3; traj4];

% Labelling the points
labels = {'1', '2', '3', '4', '5'};
descriptions = {'conveyor side1', 'conveyor side2', 'plastic bin', 'metal bin', 'glass bin'};
for i = 1:size(points, 1)
    text(points(i,1), points(i,2), points(i,3), [labels{i} ' (' descriptions{i} ')'], 'FontSize', 5, 'FontWeight', 'bold');
end

robot.plot([0 0 0 0 0 0]);

% Animating the robot to move through the five points
robot.plot(traj);



traj = [points(1,:); points(3,:); points(2,:); points(4,:); points(1,:); points(5,:)];

%Move from point 1-5 in increasing order
for i = 1:size(traj, 1)
    q = robot.ikcon(transl(traj(i,:))*tool, q0);
    robot.plot(q);
%    text(traj(i,1), traj(i,2), traj(i,3), num2str(i), 'FontSize', 10, 'Color', 'red');
    pause(1);
end



% Ask the user to select two points to move the robot
fprintf('Select two points to move the robot from point 1 to point 5.\n');
fprintf('Enter the numbers of the two points, separated by a space (e.g. 2 4): ');
selected_points = input('', 's');
selected_points = str2num(selected_points);

% Generate a trajectory for moving the robot from the selected points
q1 = robot.ikcon(tool * transl(points(selected_points(1), :)), q0);
q2 = robot.ikcon(tool * transl(points(selected_points(2), :)), q1);
traj = jtraj(q1, q2, 10);

% Animate the robot moving through the selected points
robot.plot(traj);


% To move from one predefined point to the other
% Keep asking for user inputs until the user enters 0 to cancel
while true
    from_point = input('Enter the starting point (1-5) or 0 to cancel: ');
    if from_point == 0
        break
    end
    to_point = input('Enter the ending point (1-5) or 0 to cancel: ');
    if to_point == 0
        break
    end

    % Generate a trajectory for moving the robot from the starting point to the ending point
    q1 = robot.ikcon(tool * transl(points(from_point, :)), q0);
    q2 = robot.ikcon(tool * transl(points(to_point, :)), q1);
    traj = jtraj(q1, q2, 10);

    % Animate the robot moving through the trajectory
    robot.plot(traj);
end

% To pick an item on the conveyor to any of the bins
while true
    % from_point = input('Enter the starting point (1-5) or 0 to cancel: ');
    % Requesting for input from the user to specify where the rubbish is
    x = input('Enter the x-coordinate of the starting point: ');
    y = input('Enter the y-coordinate of the starting point: ');
    z = input('Enter the z-coordinate of the starting point: ');
    start_point = [x, y, z];
    
    % to check if any of the coordinate is empty or not a number
    if isempty(x) || isnan(x) || isempty(y) || isnan(y) || isempty(z) || isnan(z)
        break
    end

    % if from_point == 0
    %     break
    % end
    to_point = getRandomPoint();
%    fprintf('Moving from point %d to point %d.\n', start_point, to_point);
    fprintf('Moving from point (%d, %d, %d) to point %d.\n', start_point(1), start_point(2), start_point(3), to_point);
    % q1 = robot.ikcon(tool * transl(points(from_point, :)), q0);
    q1 = robot.ikcon(tool * transl(start_point), q0);
    q2 = robot.ikcon(tool * transl(points(to_point, :)), q1);
    traj = jtraj(q1, q2, 10);
    robot.plot(traj);
end


% To choose between any of the 3 bins (points 3-5)
function pointz = getRandomPoint()
    pointz = randi([3,5]);
end
