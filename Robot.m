% Angles of the robot links
global alpha
global beta
global theta
global fixed_angle

alpha = 0;
beta = 0;
theta = 0;
fixed_angle = 42.1505;
% fixed_angle = ;

% Lengths of the robot links
global a1
global a2
global a3

a1 = 60.32;
a2 = 181.1;
a3 = 194.2521;

global xlimit
global ylimit

xlimit = 350;
ylimit = 350;

figure
update_plot()

% Create sliders side by side
% Make a slider to control the angle of the first link
slider1 = uicontrol('Style', 'slider', ...
    'SliderStep', [0.01 0.1], ... % Make the slider more precise
    'Min', -180, 'Max', 180, 'Value', alpha, ...
    'Position', [50 20 120 20], ...
    'Callback', @slider1_callback);
% Add listener to the slider
addlistener(slider1, 'ContinuousValueChange', @slider1_callback);
% Label the slider on top
uicontrol('Style', 'text', ...
    'Position', [50 45 120 20], ...
    'String', 'Alpha');

% Make a slider to control the angle of the second link
slider2 = uicontrol('Style', 'slider', ...
    'SliderStep', [0.01 0.1], ... % Make the slider more precise
    'Min', -180, 'Max', 180, 'Value', beta, ...
    'Position', [200 20 120 20], ...
    'Callback', @slider2_callback);
% Add listener to the slider
addlistener(slider2, 'ContinuousValueChange', @slider2_callback);
% Label the slider on top
uicontrol('Style', 'text', ...
    'Position', [200 45 120 20], ...
    'String', 'Beta');

% Make a slider to control the angle of the third link
slider3 = uicontrol('Style', 'slider', ...
    'SliderStep', [0.01 0.1], ... % Make the slider more precise
    'Min', -180, 'Max', 180, 'Value', theta, ...
    'Position', [350 20 120 20], ...
    'Callback', @slider3_callback);
% Add listener to the slider
addlistener(slider3, 'ContinuousValueChange', @slider3_callback);
% Label the slider on top
uicontrol('Style', 'text', ...
    'Position', [350 45 120 20], ...
    'String', 'Gamma');

syms P1 P2 P3 L1 L2 L3 theta1 theta2 theta3 const
eq = [P1; P2; P3] == [L1 * cosd(const) * cosd(theta1) + L2 * cosd(const + theta2) * cosd(theta1) + L3 * cosd(const + theta2 + theta3) * cosd(theta1); ...
    L1 * cosd(const) * sind(theta1) + L2 * cosd(const + theta2) * sind(theta1) + L3 * cosd(const + theta2 + theta3) * sind(theta1); ...
    L1 * sind(const) + L2 * sind(const + theta2) + L3 * sind(const + theta2 + theta3)];

eq5 = subs(eq, [L1 L2 L3 const], [a1 a2 a3 fixed_angle]);

% Solve for theta1
% eq1 = subs(eq, [theta2  theta3], [beta theta]);

% % Solve for theta2
% eq2 = subs(eq, [theta1  theta3], [alpha theta]);

% % Solve for theta3
% eq3 = subs(eq, [theta1  theta2], [alpha beta]);

% % Solve the equations
% sol1 = solve(eq1, theta1)
% sol2 = solve(eq2, theta2)
% sol3 = solve(eq3, theta3)

function slider1_callback(source, eventdata)
global alpha
alpha = get(source, 'Value');
update_plot()
end

function slider2_callback(source, eventdata)
global beta;
beta = get(source, 'Value');
update_plot()
end

function slider3_callback(source, eventdata)
global theta;
theta = get(source, 'Value');
update_plot()
end

function update_plot()
global alpha
global beta
global theta
global fixed_angle

global a1
global a2
global a3

global xlimit
global ylimit

figure(gcf)
sgtitle({'Robot Arm Angles', ['Alpha: ' num2str(alpha, '%3.1f') ' Beta: ' num2str(beta, '%3.1f') ' Gamma: ' num2str(theta, '%3.1f')]})

% Top View (XY plane)
subplot(1, 2, 2)
title('Top view')
ylabel('Y (mm)')
xlabel('X (mm)')
grid on
axis equal
cla

% Draw a filled circle with the radius of the first link
circle(0, 0, a1 * cosd(fixed_angle), 'y');

line([a1 * cosd(fixed_angle) * cosd(alpha) a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha)], ...
    [a1 * cosd(fixed_angle) * sind(alpha) a1 * cosd(fixed_angle) * sind(alpha) + a2 * cosd(fixed_angle + beta) * sind(alpha)], ...
    'LineWidth', 5, 'Color', 'b');
% Draw a circle at the end of the first link
circle(a1 * cosd(fixed_angle) * cosd(alpha), a1 * cosd(fixed_angle) * sind(alpha), 12, 'c');

line([a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) + a3 * cosd(fixed_angle + beta + theta) * cosd(alpha)], ...
    [a1 * cosd(fixed_angle) * sind(alpha) + a2 * cosd(fixed_angle + beta) * sind(alpha) a1 * cosd(fixed_angle) * sind(alpha) + a2 * cosd(fixed_angle + beta) * sind(alpha) + a3 * cosd(fixed_angle + beta + theta) * sind(alpha)], ...
    'LineWidth', 3, 'Color', 'r');
% Draw a circle at the end of the second link
circle(a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha), a1 * cosd(fixed_angle) * sind(alpha) + a2 * cosd(fixed_angle + beta) * sind(alpha), 8, 'm');

position_text = sprintf('X: %.1f mm, Y: %.1f mm, Z: %.1f mm', ...
    a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) + a3 * cosd(fixed_angle + beta + theta) * cosd(alpha), ...
    a1 * cosd(fixed_angle) * sind(alpha) + a2 * cosd(fixed_angle + beta) * sind(alpha) + a3 * cosd(fixed_angle + beta + theta) * sind(alpha), ...
    a1 * sind(fixed_angle) + a2 * sind(fixed_angle + beta) + a3 * sind(fixed_angle + beta + theta));

xlim([-xlimit xlimit])
ylim([-ylimit ylimit])

% Side View (ZX plane)
subplot(1, 2, 1)
title('Side view')
ylabel('Z (mm)')
xlabel('X (mm)')
grid on
axis equal
cla

% Draw a rectangle with the length of the first link
rectangle('Position', [-a1 * cosd(fixed_angle) 0 a1 * cosd(fixed_angle) * 2 a1 * sind(fixed_angle)], 'FaceColor', 'y');

% Start drawing from the second link (the first link is fixed)
line([a1 * cosd(fixed_angle) * cosd(alpha) a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha)], ...
    [a1 * sind(fixed_angle) a2 * sind(fixed_angle + beta)], ...
    'LineWidth', 5, 'Color', 'b');
% Draw a circle at the end of the first link
circle(a1 * cosd(fixed_angle) * cosd(alpha), a1 * sind(fixed_angle), 12, 'c');

line([a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) + a3 * cosd(fixed_angle + beta + theta) * cosd(alpha)], ...
    [a2 * sind(fixed_angle + beta) a2 * sind(fixed_angle + beta) + a3 * sind(fixed_angle + beta + theta)], ...
    'LineWidth', 3, 'Color', 'r');
% Draw a circle at the end of the second link
circle(a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha), a2 * sind(fixed_angle + beta), 8, 'm');

xlim([-xlimit xlimit])
ylim([-ylimit ylimit])

% End effector position
x = a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) + a3 * cosd(fixed_angle + beta + theta) * cosd(alpha);
y = a1 * cosd(fixed_angle) * sind(alpha) + a2 * cosd(fixed_angle + beta) * sind(alpha) + a3 * cosd(fixed_angle + beta + theta) * sind(alpha);
z = a2 * sind(fixed_angle + beta) + a3 * sind(fixed_angle + beta + theta);

% Display the end effector position
disp(['x = ' num2str(x) ', y = ' num2str(y) ', z = ' num2str(z)]) % Tip of the third link

subplot(1, 2, 2)
hold on
str = sprintf('%.1f, %.1f', x, y);
text(a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) + a3 * cosd(fixed_angle + beta + theta) * cosd(alpha), ...
    a1 * cosd(fixed_angle) * sind(alpha) + a2 * cosd(fixed_angle + beta) * sind(alpha) + a3 * cosd(fixed_angle + beta + theta) * sind(alpha), ...
    str, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12, 'Color', 'k');

subplot(1, 2, 1)
str = sprintf('%.1f, %.1f', x, z);
text(a1 * cosd(fixed_angle) * cosd(alpha) + a2 * cosd(fixed_angle + beta) * cosd(alpha) + a3 * cosd(fixed_angle + beta + theta) * cosd(alpha), ...
    a1 * sind(fixed_angle) + a2 * sind(fixed_angle + beta) + a3 * sind(fixed_angle + beta + theta), ...
    str, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 12, 'Color', 'k');

[t1,t2,t3] = inverseKinematics(x, y, z);
fprintf('t1 = %f, t2 = %f, t3 = %f\n', t1*180/pi, t2*180/pi, t3*180/pi)

end

function h = circle(center_x, center_y, r, color)
hold on
angle = linspace(0, 2 * pi, 1000);
xunit = r * cos(angle) + center_x;
yunit = r * sin(angle) + center_y;
h = plot(xunit, yunit, 'k');
fill(xunit, yunit, color);
end

function [t1, t2, t3] = inverseKinematics(x, y, z)
global a1
global a2
global a3

x_base = 0;
y_base = 0;
z_base = 0;

t1 = atan2(y - y_base, x - x_base);

L1 = a2;
L2 = a3;
D = (-L1 * L1 - L2 * L2 + x * x + z * z) / (2 * L1 * L2);
% t2 = atan2(-sqrt(1 - D * D), D);
t2= 1;
a = L2 * sin(t2);
b = L1 + L2 * cos(t2);
c = z;
t3 = c / sqrt(a * a + b * b - c * c) - atan2(a, b);
end