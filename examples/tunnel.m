% create matlab node to communicate with other nodes
matlab_node = ros2node("/matlab_node");

% create publisher to send position and velocity commands to the tunnel
tunnel_publisher = ros2publisher(matlab_node, ...
    "/tunnel_joint_target", "sensor_msgs/JointState");

% create message to publish
tunnel_message = ros2message("sensor_msgs/JointState");

% specify motor axis names
tunnel_message.name = {'x', 'y', 'z'};

% first tunnel position
tunnel_message.position = [1000, 1000, 1000];
send(tunnel_publisher, tunnel_message);

%second tunnel position
tunnel_message.position = [100, 200, 300];
send(tunnel_publisher, tunnel_message);

% cleanup and close nodes when finished
clear tunnel_publisher
clear tunnel_message
clear matlab_node
