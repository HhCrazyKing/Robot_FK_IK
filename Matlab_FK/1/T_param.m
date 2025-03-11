function T = T_param(theta, d, a, alpha)
% 输入：theta，d，a和alpha为变量，两个相邻连杆坐标系之间DH参数
% 输出：机械臂两个相邻连杆坐标系的正运动学变换矩阵

    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha) , a*cos(theta);
         sin(theta), cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta);
         0         , sin(alpha)            , cos(alpha)            , d           ;
         0         , 0                     , 0                     , 1            ];
     
end
