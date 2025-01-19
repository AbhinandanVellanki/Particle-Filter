clear;

rng(0); % initialize random number generator

b1 = [5,5]; % position of beacon 1
b2 = [15,5]; % position of beacon 2

% load pfData.mat
load('pfData.mat')

numSteps = length(t);

% initialize movie array
M(numSteps) = struct('cdata',[],'colormap',[]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         put particle filter initialization code here                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set particle filter parameters
numParticles = 2000;
processNoise = diag([0.7, 0.7, 0.7]); % [x, y, theta] process noise
measurementNoise = diag([0.8, 0.8]); % Range measurement noise

% Initialize particles
particles = zeros(3, numParticles);
particles(1,:) = unifrnd(0, 20, 1, numParticles); % x
particles(2,:) = unifrnd(0, 10, 1, numParticles); % y
particles(3,:) = unifrnd(-pi, pi, 1, numParticles); % theta

weights = ones(1, numParticles) / numParticles;

% here is some code to plot the initial scene
figure(1)
plotParticles(particles); % particle cloud plotting helper function
hold on
plot([b1(1),b2(1)],[b1(2),b2(2)],'s',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
drawRobot(q_groundTruth(:,1), 'cyan'); % robot drawing helper function
axis equal
axis([0 20 0 10])
M(1) = getframe; % capture current view as movie frame
pause
disp('hit return to continue')

for k = 2:numSteps

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %              put particle filter prediction step here               %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Apply motion model to particles
    dt = t(k) - t(k-1);
    for i = 1:numParticles
        v = u(1,k-1) + sqrt(processNoise(1,1)) * randn;
        w = u(2,k-1) + sqrt(processNoise(3,3)) * randn;
        
        particles(1,i) = particles(1,i) + dt * v * cos(particles(3,i));
        particles(2,i) = particles(2,i) + dt * v * sin(particles(3,i));
        particles(3,i) = particles(3,i) + dt * w;
    end

    % Normalize angles to -pi, pi
    particles(3,:) = mod(particles(3,:) + pi, 2*pi) - pi;
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                put particle filter update step here                 %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate expected measurements for each particle
    expected_measurements = zeros(2, numParticles);
    for i = 1:numParticles
        expected_measurements(1,i) = norm(particles(1:2,i) - b1');
        expected_measurements(2,i) = norm(particles(1:2,i) - b2');
    end

    % Calculate particle weights
    for i = 1:numParticles
        diff = y(:,k) - expected_measurements(:,i);
        weights(i) = mvnpdf(diff, [0;0], measurementNoise);
    end

    % Normalize weights
    weights = weights / sum(weights);

    % Resample particles (now done at every step)
    indices = randsample(numParticles, numParticles, true, weights);
    particles = particles(:, indices);
    weights = ones(1, numParticles) / numParticles;
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot particle cloud, robot, robot estimate, and robot trajectory here %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    clf;
    hold on;
    
    % Plot particle cloud
    plotParticles(particles);
    
    % Plot beacons
    plot([b1(1),b2(1)],[b1(2),b2(2)],'s',...
        'LineWidth',2,...
        'MarkerSize',10,...
        'MarkerEdgeColor','r',...
        'MarkerFaceColor',[0.5,0.5,0.5]);
    
    % Plot ground truth robot and trajectory
    drawRobot(q_groundTruth(:,k), 'yellow');
    plot(q_groundTruth(1,1:k), q_groundTruth(2,1:k), 'b-');
    
    % Plot estimated robot pose
    estimated_pose = sum(particles .* weights, 2);
    drawRobot(estimated_pose, 'blue');
    
    axis equal
    axis([0 20 0 10])
    title(['Step ', num2str(k), ' of ', num2str(numSteps)]);

    % capture current figure and pause
    M(k) = getframe; % capture current view as movie frame
    pause(0.01)
        
end

% when you're ready, the following block of code will export the created 
% movie to an mp4 file
videoOut = VideoWriter('result.mp4','MPEG-4');
videoOut.FrameRate=5;
open(videoOut);
for k=1:numSteps
  writeVideo(videoOut,M(k));
end
close(videoOut);



% helper function to plot a particle cloud
function plotParticles(particles)
    plot(particles(1, :), particles(2, :), 'go')
    line_length = 0.1;
    quiver(particles(1, :), particles(2, :), line_length * cos(particles(3, :)), line_length * sin(particles(3, :)))
end


% helper function to plot a differential drive robot
function drawRobot(pose, color)
    
% draws a SE2 robot at pose
x = pose(1);
y = pose(2);
th = pose(3);

% define robot shape
robot = [-1 .5 1 .5 -1 -1;
          1  1 0 -1  -1 1 ];
tmp = size(robot);
numPts = tmp(2);
% scale robot if desired
scale = 0.5;
robot = robot*scale;

% convert pose into SE2 matrix
H = [ cos(th)   -sin(th)  x;
      sin(th)    cos(th)  y;
      0          0        1];

% create robot in position
robotPose = H*[robot; ones(1,numPts)];

% plot robot
plot(robotPose(1,:),robotPose(2,:),'k','LineWidth',2);
rFill = fill(robotPose(1,:),robotPose(2,:), color);
alpha(rFill,.2); % make fill semi transparent
end

