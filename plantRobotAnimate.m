clc; clear all; close all;

prompt = {'Plant Pot Height in m (range from 0.1 to 0.7 m): ','Robot Center distance from Pot (range from 0.17 to 0.7 m): '};
dlgtitle = 'Input';
answer = inputdlg(prompt,dlgtitle);

%% Setting Final Values of Joints using Inverse Kinematics and Plotting Trajectories
% Inverse Kinematic Equations Derived:
% Given: location and orientation of end effector (nozzle) i.e. xE, zE, phi
% (angle between Base frame Z axis and nozzle)
% 1. q1 = xE/tan(phi) + zE
% 2. q2 = 2*pi - phi
% 3. q3 = xE/sin(phi)

potH = str2double(answer{1});   % given pot height
xE = str2double(answer{2});     % given distance to pot from center of robot
zE = potH+0.01-0.35;     % offset for safe distance of nozzle

q1f = 2.05 * zE;      % general safe distance for joint 1 established
phi = atan(xE / (q1f-zE));
q2f = 2*pi - phi;
q3f = xE / (sin(phi));

% Home configuration joint positions
q10 = 0.1;
q20 = 3*pi/2;
q30 = 0.05;

% defining time 
t = transpose(linspace(0,2,50));

% Joint trajectories made LSPB, cubic polynomial for all three joints

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Trajectory q1:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms q1a2 q1a3
eqn11 = 0.1 + 4*q1a2 + 8*q1a3 == q1f;
eqn12 = 4*q1a2 + 12*q1a3 == 0;
sol1 = solve([eqn11 eqn12],[q1a2 q1a3]);
q1a2 = sol1.q1a2;
q1a3 = sol1.q1a3;

q1 = 0.1 + q1a2.*t.^2 + q1a3.*t.^3;
q1 = double(q1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Trajectory q2:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms q2a2 q2a3
eqn21 = (3*pi/2) + 4*q2a2 + 8*q2a3 == q2f;
eqn22 = 4*q2a2 + 12*q2a3 == 0;
sol2 = solve([eqn21 eqn22],[q2a2 q2a3]);
q2a2 = sol2.q2a2;
q2a3 = sol2.q2a3;

q2 = (3*pi/2) + q2a2.*t.^2 + q2a3.*t.^3;
q2 = double(q2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Trajectory q3:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms q3a2 q3a3
eqn31 = 0.05 + 4*q3a2 + 8*q3a3 == q3f;
eqn32 = 4*q3a2 + 12*q3a3 == 0;
sol3 = solve([eqn31 eqn32],[q3a2 q3a3]);
q3a2 = sol3.q3a2;
q3a3 = sol3.q3a3;

q3 = 0.05 + q3a2.*t.^2 + q3a3.*t.^3;
q3 = double(q3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Joint Velocities        %
% Joint 1
q1v = diff(q1)./diff(t);
q1v=[0;q1v];
q1v(50,1) = 0;

% Joint 2
q2v = diff(q2)./diff(t);
q2v=[0;q2v];
q2v(50,1) = 0;

% Joint 3
q3v = diff(q3)./diff(t);
q3v=[0;q3v];
q3v(50,1) = 0;

%% Defining and Plotting Robot

% Links with DH Table ([theta d a alpha p=1 or r=0])

Lx(1) = Link([pi 0 0 pi/2 1]);
Lx(2) = Link([0 0 0 pi/2 0]);
Lx(3) = Link([0 0 0 0 1]);

Robo = SerialLink(Lx);          % Using Dr. Peter Corke's Robotics Toolbox for MATLAB
Robo.name = 'Houseplant Watering Robot';

% Representation of base of robot with reservoir
r=0.1;
h=-0.3;
[X,Y,Z]=cylinder(r);
Z=Z*h;
surf(X,Y,Z)

hold on
Robo.plot([(q1),(q2),(q3)],'workspace',[-0.5 1.0 -0.5 0.5 -0.35 0.1],'floorlevel',-0.35,'name','joints','jointcolor',[0.9412 0.4824 0.1804],'trail',{'r'},'lightpos',[5 -20 10],'movie','file.mp4')

% End Effector Path
figure
subplot(1,2,1)
plot(Robo.trail(:,1),Robo.trail(:,3))
title('End Effector Path')
xlabel('X_{0}-Axis Direction (m)')
ylabel('Z_{0}-Axis Direction (m)')
grid minor
dist = sqrt((Robo.trail(:,1)).^2 + (Robo.trail(:,3)).^2);
subplot(1,2,2)
plot(t,dist,'r-')
title('End Effector Trajectory')
xlabel('Time (s)')
ylabel('Distance Travelled (m)')
grid minor
hold on

% End Effector Trajectory Equation
trE = polyfit(t,dist,3);
txt = ['End Effector Trajectory Equation: EE = ' num2str(trE(1)) 't^{3} + ' num2str(trE(2)) 't^{2} + ' num2str(trE(3)) 't + ' num2str(trE(4))];
%text(0.05,0.3,txt)
a = annotation('textbox',[.58 .8 .02 .05],'String',txt,'EdgeColor','none');
a.FontSize = 8;

% End Effector Velocity
eeV = diff(dist)./diff(t);
eeV = [eeV;0];
eeV(1) = 0;

% Plotting End Effector Velocity
figure
plot(t,eeV,'r-')
title('End Effector Velocity')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid minor

% Plotting Joint Trajectories
figure
subplot(1,2,1)
plot(t,q1,'b-',t,q3,'m-')
legend('Joint 1','Joint 3','Location','northwest')
title('Joints 1 and 3 Trajectories')
xlabel('Time (s)')
ylabel('Joint Positions (m)')
grid minor
subplot(1,2,2)
plot(t,q2,'color',[0.5176,0.2196,0.9608])
title('Joint 2 Trajectory')
xlabel('Time (s)')
ylabel('Joint Angle (rad)')
grid minor

% Plotting Joint Velocities
figure
subplot(1,2,1)
plot(t,q1v,'b-',t,q3v,'m-')
legend('Joint 1','Joint 3','Location','northwest')
title('Joints 1 and 3 Velocities')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid minor
subplot(1,2,2)
plot(t,q2v,'color',[0.5176,0.2196,0.9608])
title('Joint 2 Velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
grid minor



