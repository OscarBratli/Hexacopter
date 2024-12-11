function void=m_localization_landmark()
% Simulates navigation using landmarks

%% Initializing
randinit();                             % Initializes the radom number generator

%% Navigation with a map
map = LandmarkMap(20);                   % Create a landmark object
map.plot();                             % Plot the map with landmarks
pause;

%% Vehicle model
V = diag([0.02, 0.5*pi/180].^2);        % Specify process covariance (distance/heading)
veh = Bicycle('covar',V);               % Creates a vehicle from the bicycle model with covariance V
veh.add_driver( RandomPath(map.dim) );  % Adds driver that steers robot to random WPs

%% Sensor measurments
W = diag([0.1, 1*pi/180].^2);           % Specify the sensor covariance matrix
sensor = RangeBearingSensor(veh, map,'covar',W, 'angle', [-pi/2 pi/2],'range',4,'animate');    
                                        % Create sensor object with defined range and covariance matrices

%% Kalman filter
P0 = diag( [0.005, 0.005, 0.001].^2 );  % Inital covarance matrix
ekf = EKF(veh,V,P0, sensor,W,map);      % Create Kalman filter object
ekf.run(1000);                          % Runs the EKF for 1000 time steps

%% Plotting the results
veh.plot_xy();                          % Plotting vehicle trajectory
ekf.plot_xy();                          % Plotting estimated trajectory
ekf.plot_ellipse();                     % Plotting error ellipses

