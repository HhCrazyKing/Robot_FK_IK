%% 启动机器人工具箱
% startup_rvc 

%% 使用机器人工具箱建立机器人模型
% UR5机械臂参数
alpha = [pi/2  , 0       , 0       , pi/2  , -pi/2  , 0];
a =     [0     , -0.425, -0.392, 0     , 0      , 0];
d =     [0.162, 0       , 0       , 0.13, 0.1, 0.1];

% 建立UR5机械臂模型
L1 = Link('d', d(1),  'a', a(1), 'alpha', alpha(1),  'standard');
L2 = Link('d', d(2),  'a', a(2), 'alpha', alpha(2),  'standard');
L3 = Link('d', d(3),  'a', a(3), 'alpha', alpha(3),  'standard');
L4 = Link('d', d(4),  'a', a(4), 'alpha', alpha(4),  'standard');
L5 = Link('d', d(5),  'a', a(5), 'alpha', alpha(5),  'standard');
L6 = Link('d', d(6),  'a', a(6), 'alpha', alpha(6),  'standard');
tool_robot = SerialLink([L1,L2,L3,L4,L5,L6], 'name', 'UR5');
% tool_robot.display();
% view(3); % 必须添加，否则报错，错误是由版本引起
% tool_robot.teach();

%% 验证机器人正运动学模型
theta = [pi/2, 0, pi/2, 0, pi/2, 0];

T = myfkine(theta, d, a, alpha);

% T1 = tool_robot.fkine(theta);
T1 = [1, 0, 0, -0.817; 0, 0, -1, -0.833; 0, 1, 0, 0.062; 0, 0, 0, 1];
q = tool_robot.ikine(T1);