% Number of captures to take
numCaptures = 1;  % Set the number of capture pairs you want

% Initialize Camera
cam = webcam;  % Initialize USB camera

% Initialize LiDAR (replace IP address with your actual LiDAR IP address)
lidar = velodynelidar('VLP16');  % Set IP address of your LiDAR
start(lidar);  % Start streaming point cloud data

% Loop through each capture
for i = 1:numCaptures
    % Capture image from camera
    %% 
    img = snapshot(cam);
    imwrite(img, sprintf('camera_image_%d.jpg', i));  % Save image with unique name
    
    % Capture point cloud from LiDAR
    ptCloud = read(lidar);  % Read a single point cloud frame
    pcwrite(ptCloud, sprintf('lidar_pointcloud_%d.pcd', i));  % Save point cloud with unique name
    
    % Optional: Display captured image and point cloud to confirm
    figure;
    subplot(1, 2, 1); imshow(img); title(sprintf('Camera Image %d', i));
    subplot(1, 2, 2); pcshow(ptCloud.Location); title(sprintf('LiDAR Point Cloud %d', i));
    
    pause(1);  % Add a short pause if needed (adjust based on your system's speed)
end

% Stop the LiDAR and release the camera
stop(lidar);  % Stop streaming point cloud data
clear cam;    % Release the camera
