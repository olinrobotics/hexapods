% Forward kinematics calculator
l1 = 10; % length of first arm
l2 = 7; % length of second arm

theta1 = 0:0.1:pi/2; % all possible theta1 values
theta2 = 0:0.1:pi; % all possible theta2 values

[THETA1,THETA2] = meshgrid(theta1,theta2); % generate a grid of theta1 and theta2 values

X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2); % compute x coordinates
Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2); % compute y coordinates

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-theta2 dataset

plot(X(:),Y(:),'r.')
axis equal;
xlabel('X','fontsize',10)
ylabel('Y','fontsize',10)
title('X-Y coordinates generated for all theta1 and theta2 combinations using forward kinematics formula','fontsize',10)

% Calculate the X, Y positions given angles of the arm in degrees.
alpha1 = 0;
alpha2 = 0;
alphaRot = 30; % The angle that the leg is rotated about the Z axis
% Convert to radians
alpha1 = alpha1*(pi/180);
alpha2 = alpha2*(pi/180);
alphaRot = alphaRot*(pi/180);
% Origin of the arm
Xorigin = 0;
Yorigin = 0;
Zorigin = 0;
% Position of the first joint of the arm
Xjoint = l1*cos(alpha1);
Yjoint = l1*sin(alpha1);
% Position of the end effector
Xend = l1*cos(alpha1) + l2*cos(alpha1 + alpha2);
Yend = l1*sin(alpha1) + l2*sin(alpha1 + alpha2);
Xpos = [Xorigin, Xjoint, Xend];
Ypos = [Yorigin, Yjoint, Yend];

% Calculate the joint point in 3D using a rotation matrix
R = eul2rotm([alphaRot 0 0]); % generates a rotation matrix about the rotated axis.
                              % Parameters are ZYX in radians
newJoint = R*[Xjoint; Yjoint; 0];

% Calculate the end point in 3D using a rotation matrix
R = eul2rotm([alphaRot 0 0]); % generates a rotation matrix about the rotated axis.
                              % Parameters are ZYX in radians
newEnd = R*[Xend; Yend; 0];

newXpos = [Xorigin, newJoint(1), newEnd(1)];
newYpos = [Yorigin, newJoint(2), newEnd(2)];
newZpos = [Zorigin, newJoint(3), newEnd(3)];

figure
plot3(newXpos, newYpos, newZpos)
xlabel("X")
ylabel("Y")
zlabel("Z")
grid on;
xticks(-30:30);
yticks(-30:30);
zticks(-30:30);
daspect([1 1 1])

