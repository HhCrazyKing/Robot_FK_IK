function T = myfkine(theta, d, a, alpha)
% 输入：theta，d，a和alpha为数组，存储机械臂DH参数
% 输出：机械臂正运动学变换矩阵

    T01 = T_param(theta(1), d(1), a(1), alpha(1));
    T12 = T_param(theta(2), d(2), a(2), alpha(2));
    T23 = T_param(theta(3), d(3), a(3), alpha(3));
    T34 = T_param(theta(4), d(4), a(4), alpha(4));
    T45 = T_param(theta(5), d(5), a(5), alpha(5));
    T56 = T_param(theta(6), d(6), a(6), alpha(6));
    
    T = T01*T12*T23*T34*T45*T56;
    
end
