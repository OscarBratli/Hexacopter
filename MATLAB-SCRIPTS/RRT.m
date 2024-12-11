clear all; close all; clc;

% Basic parameters
mapSize = 100;
startPos = [0,0,0];
goalPos = [80,80,50];

% Some random spherical obstacles
obstacles = [
    30,40,10,5;
    45,70,20,5;
    60,60,40,5
];

% Set up figure
figure; hold on; grid on;
axis([0 mapSize 0 mapSize 0 mapSize]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);

% Plot start and goal
plot3(startPos(1), startPos(2), startPos(3), 'bo','MarkerFaceColor','b','MarkerSize',8);
plot3(goalPos(1), goalPos(2), goalPos(3), 'ro','MarkerFaceColor','r','MarkerSize',8);

% Plot obstacles as black spheres
for i = 1:size(obstacles,1)
    [Xs,Ys,Zs] = sphere(20);
    Xs = Xs*obstacles(i,4)+obstacles(i,1);
    Ys = Ys*obstacles(i,4)+obstacles(i,2);
    Zs = Zs*obstacles(i,4)+obstacles(i,3);
    surf(Xs,Ys,Zs,'FaceColor','k','EdgeColor','none');
end
drawnow;

% Run the RRT function
path = rrt(startPos,goalPos,obstacles,mapSize);

% If path found, plot it
if isempty(path)
    disp('No path found.');
else
    plot3(path(:,1), path(:,2), path(:,3),'g-','LineWidth',2);
    disp('Path found and displayed.');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RRT function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function path = rrt(startPos, goalPos, obstacles, mapSize)
    maxIter = 5000;
    stepSize = 5;
    goalThresh = 5;
    tree = startPos;
    parent = 0;

    for i=1:maxIter
        % Sometimes pick goal, otherwise pick random
        if rand<0.3
            randPt = goalPos;
        else
            randPt = rand(1,3)*mapSize;
        end
        
        % Find closest node in tree
        [~,idx] = min(sum((tree - randPt).^2,2));
        nearestNode = tree(idx,:);
        
        % Move towards randPt by stepSize
        dir = randPt - nearestNode;
        dist = norm(dir);
        if dist > stepSize
            dir = dir/dist;
            newNode = nearestNode + dir*stepSize;
        else
            newNode = randPt;
        end
        
        % Check boundaries
        if any(newNode<0) || any(newNode>mapSize)
            continue;
        end
        
        % Check collision
        if ~checkCollision(nearestNode,newNode,obstacles)
            tree = [tree;newNode];
            parent = [parent;idx];
            
            % Check if near goal
            if norm(newNode - goalPos)<goalThresh
                path = newNode;
                curr = size(tree,1);
                while curr~=1
                    curr = parent(curr);
                    path = [tree(curr,:); path];
                end
                return;
            end
        end
    end
    path = [];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Collision check
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function col = checkCollision(node1,node2,obs)
    col = false;
    steps = ceil(norm(node2-node1));
    for t=linspace(0,1,steps)
        pt = node1 + t*(node2-node1);
        for i=1:size(obs,1)
            c = obs(i,1:3);
            r = obs(i,4);
            if norm(pt-c)<=r
                col = true;
                return;
            end
        end
    end
end
