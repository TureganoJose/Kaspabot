
% Define joint variables
q1 = sym('q1','real');
q2 = sym('q2','real');
q3 = sym('q3','real');
q4 = sym('q4','real');
q5 = sym('q5','real');
q6 = sym('q6','real');
q= [q1 q2 q3 q4 q5 q6];

% Define dimensions
a2 = 130;
d4 = 160;
d6 = 130;

% End effector position, p6 relative to 0
p6_0 = [a2*cos(q1)*cos(q2)+d4*cos(q1)*sin(q2+q3)+d6*(cos(q1)*(cos(q2+q3)*cos(q4)*sin(q5)+sin(q2+q3)*cos(q5))+sin(q1)*sin(q4)*sin(q5));
        a2*sin(q1)*cos(q2)+d4*sin(q1)*sin(q2+q3)+d6*(sin(q1)*(cos(q2+q3)*cos(q4)*sin(q5)+sin(q2+q3)*cos(q5))-cos(q1)*sin(q4)*sin(q5));
        a2*sin(q2)-d4*cos(q2+q3)+d6*(sin(q2+q3)*cos(q4)*sin(q5)-cos(q2+q3)*cos(q5))];
matlabFunction(p6_0,'file','func_p6_0.m','vars',[q1 q2 q3 q4 q5])

% End effector jacobian = dp/dq
J6_0 = jacobian(p6_0,q)
matlabFunction(J6_0,'file','func_J6_0.m','vars',[q1 q2 q3 q4 q5])
% pseudo_inv_J_6_0 = pinv(J_6_0)
% matlabFunction(pseudo_inv_J_6_0,'file','func_pseudo_inv_J_6_0.m','vars',[q1 q2 q3 q4 q5])


% Define trajectory
q0 = [0 0 pi/2 0 -pi/2 0];
dq0 = zeros(1,6);
pCenter = func_p6_0(q0(1),q0(2),q0(3),q0(4),q0(5));
radius = 0.05;
f = 0.25;
pGoal = @(t) pCenter + radius*[sin(2*pi*f*t),0,cos(2*pi*f*t)]';
dpGoal = @(t) 2*pi*f*radius*[cos(2*pi*f*t),0,-sin(2*pi*f*t)]';


% define here the time resolution
deltaT = 0.01;
timeArr = 0:deltaT:1/f;

% q, p, and pGoal logging
qArr = zeros(6,length(timeArr));
pArr = zeros(3,length(timeArr));
pGoalArr = zeros(3,length(timeArr));

q = q0;
dq = dq0;
for i=1:length(timeArr)
    t = timeArr(i);
    % data logging, don't change this!
    q = q+deltaT*dq;
    qArr(:,i) = q;
    pArr(:,i) = func_p6_0(q(1),q(2),q(3),q(4),q(5));
    pGoalArr(:,i) = pGoal(t);
    
    % controller: 
    % step 1: create a simple p controller to determine the desired foot
    % point velocity
    %v = ...;
    % step 2: perform inverse differential kinematics to calculate the
    % generalized velocities
    dq = pinv(func_J6_0(q(1),q(2),q(3),q(4),q(5)))*(pGoal(t)-pArr(:,i));
    
end