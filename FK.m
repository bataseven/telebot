L1 = 46.4055; % mm
L2 = 181.1; % mm
L3 = 194.2521; % mm

L1 = 46.4; % mm
L2 = 181.1; % mm
L3 = 194.3; % mm

theta1 = 0; % deg
theta2 = 0; % deg
theta3 = 0; % deg

syms alpha_i_minus_one ...
    a_i_minus_one ...
    d_i...
    theta_i...
    alpha1...
    alpha2...
    alpha3...
    alpha1_dot...
    alpha2_dot...
    alpha3_dot...
    
T_i_relative_to_i_minus_one = ... % Description of frame i relative to i-1
    [cosd(theta_i)                       -sind(theta_i)                          0                        a_i_minus_one ;...
    sind(theta_i)*cosd(alpha_i_minus_one) cosd(theta_i)*cosd(alpha_i_minus_one) -sind(alpha_i_minus_one) -sind(alpha_i_minus_one)*d_i ;...
    sind(theta_i)*sind(alpha_i_minus_one) cosd(theta_i)*sind(alpha_i_minus_one)  cosd(alpha_i_minus_one)  cosd(alpha_i_minus_one)*d_i ;...
    0                                     0                                      0                        1 ...
    ];

T_1_0 = T_i_relative_to_i_minus_one; % T_1_0 is the transformation from frame 1 to frame 0
T_1_0=subs(T_1_0, alpha_i_minus_one, 0);
T_1_0=subs(T_1_0, a_i_minus_one,0);
T_1_0=subs(T_1_0,theta_i, theta1);
T_1_0=subs(T_1_0, d_i,0);

T_2_1 = T_i_relative_to_i_minus_one;
T_2_1=subs(T_2_1, alpha_i_minus_one,-90);
T_2_1=subs(T_2_1, a_i_minus_one, L1);
T_2_1=subs(T_2_1,theta_i, -theta2);
T_2_1=subs(T_2_1, d_i,0);

T_3_2 = T_i_relative_to_i_minus_one;
T_3_2=subs(T_3_2, alpha_i_minus_one, 0);
T_3_2=subs(T_3_2, a_i_minus_one, L2);
T_3_2=subs(T_3_2,theta_i, -theta3);
T_3_2=subs(T_3_2, d_i, 0);

T_4_3 = T_i_relative_to_i_minus_one;
T_4_3=subs(T_4_3, alpha_i_minus_one,0);
T_4_3=subs(T_4_3, a_i_minus_one,L3);
T_4_3=subs(T_4_3,theta_i,0);
T_4_3=subs(T_4_3, d_i,0);

T_4_0 = eval(T_1_0 * T_2_1 * T_3_2 * T_4_3);

% Extract x,y,z coordinates from T_4_0
x_pos = T_4_0(1,4);
y_pos = T_4_0(2,4);
z_pos = T_4_0(3,4);

fprintf('x = %.1f y = %.1f z = %.1f\n', x_pos, y_pos, z_pos);


T_1_0 = T_i_relative_to_i_minus_one; % T_1_0 is the transformation from frame 1 to frame 0
T_1_0=subs(T_1_0, alpha_i_minus_one, 0);
T_1_0=subs(T_1_0, a_i_minus_one,0);
T_1_0=subs(T_1_0,theta_i, alpha1);
T_1_0=subs(T_1_0, d_i,0);

T_2_1 = T_i_relative_to_i_minus_one;
T_2_1=subs(T_2_1, alpha_i_minus_one,-90);
T_2_1=subs(T_2_1, a_i_minus_one, L1);
T_2_1=subs(T_2_1,theta_i, -alpha2);
T_2_1=subs(T_2_1, d_i,0);

T_3_2 = T_i_relative_to_i_minus_one;
T_3_2=subs(T_3_2, alpha_i_minus_one, 0);
T_3_2=subs(T_3_2, a_i_minus_one, L2);
T_3_2=subs(T_3_2,theta_i, -alpha3);
T_3_2=subs(T_3_2, d_i, 0);

T_4_3 = T_i_relative_to_i_minus_one;
T_4_3=subs(T_4_3, alpha_i_minus_one,0);
T_4_3=subs(T_4_3, a_i_minus_one,L3);
T_4_3=subs(T_4_3,theta_i,0);
T_4_3=subs(T_4_3, d_i,0);

% Simplified symbolic forward kinematics
T_4_0 = eval(T_1_0 * T_2_1 * T_3_2 * T_4_3);

% Simplify
T_4_0 = simplify(T_4_0)

x = T_4_0(1,4);
y = T_4_0(2,4);
z = T_4_0(3,4);

x_dot = diff(x,alpha1)*alpha1_dot + diff(x,alpha2)*alpha2_dot + diff(x,alpha3)*alpha3_dot
y_dot = diff(y,alpha1)*alpha1_dot + diff(y,alpha2)*alpha2_dot + diff(y,alpha3)*alpha3_dot
z_dot = diff(z,alpha1)*alpha1_dot + diff(z,alpha2)*alpha2_dot + diff(z,alpha3)*alpha3_dot

% Jacobian
J = [diff(x,alpha1) diff(x,alpha2) diff(x,alpha3);...
    diff(y,alpha1) diff(y,alpha2) diff(y,alpha3);...
    diff(z,alpha1) diff(z,alpha2) diff(z,alpha3)];

% Simplify
J = simplify(J)

% Inverse Jacobian
J_inv = inv(J)


desired_x_dot = 1;
desired_y_dot = 2;
desired_z_dot = 3;



% Solve for alpha1_dot, alpha2_dot, alpha3_dot
alpha1_dot = J_inv(1,1)*desired_x_dot + J_inv(1,2)*desired_y_dot + J_inv(1,3)*desired_z_dot;
alpha2_dot = J_inv(2,1)*desired_x_dot + J_inv(2,2)*desired_y_dot + J_inv(2,3)*desired_z_dot;
alpha3_dot = J_inv(3,1)*desired_x_dot + J_inv(3,2)*desired_y_dot + J_inv(3,3)*desired_z_dot;

% substitute alpha1, alpha2, alpha3 with theta1, theta2, theta3. And calculate
a1 = eval(subs(alpha1_dot, [alpha1 alpha2 alpha3], [theta1 theta2 theta3]))
a2 = eval(subs(alpha2_dot, [alpha1 alpha2 alpha3], [theta1 theta2 theta3]))
a3 = eval(subs(alpha3_dot, [alpha1 alpha2 alpha3], [theta1 theta2 theta3]))