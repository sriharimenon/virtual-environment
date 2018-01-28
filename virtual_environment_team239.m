%% virtual_environment_team239
%
% This script is the starter code for Part 2 of Project 2, where you
% build an interactive virtual environment that can be touched using
% the Phantom.
%
% Written by Katherine J. Kuchenbecker for MEAM 520 at the University
% of Pennsylvania.


%% Clean up

clear
close all
clc


%% Set hardware mode, duration, and warnings

% Set whether to use the actual PHANToM hardware.  If this variable is
% set to false, the software will simulate the presence of a user by
% reading a pre-recorded trajectory.  You should use this mode to
% debug your code before running anything on the PHANToM computer.
% Once you are on the PHANToM/PUMA computer in Towne B2, you may set
% this variable to true.  Be sure to run your code with the emergency
% stop down to prevent the application of forces to make sure
% everything works correctly on the real robot.  Make sure to hold
% onto the PHANToM tightly and keep a hand on the emergency stop.
hardware = false;

% Set how many times we want our servo loop to run.  Each cycle takes
% about two milliseconds on the computer in Towne B2.
nCycles = 5000;

% If not using the hardware, turn off warnings triggered when you
% command joint torques above the limits.
if (~hardware)
    warning('off','PHANToM:JointTorque')
else
    warning('on','PHANToM:JointTorque')
end


%% Define global variables for keyboard control

global keyacctime amag adur

% A matrix that holds the list of currently active pulses applied by
% the keyboard.  Each time the user presses a key, an additional pulse
% is added to the right side of this 5 x n matrix.  The first three
% rows are acceleration in x, y, and z directions.  The fourth row is
% the time when the pulse starts, and the last row is the time when
% the pulse ends.  The minimum number of columns is one, with a zero
% acceleration pulse that is always active.
keyacctime = [0 0 0 0 inf]'; % mm/s^2, mm/s^2, mm/s^2, s, s

% Set the magnitude of the acceleration pulse caused by a key press.
amag = 10; % mm/s^2

% Set the duration of the acceleration pulse caused by a key press.
adur = 0.25; % s


%% Define the virtual environment


% Set the stiffness of the virtual walls in newtons per millimeter.
k = 0.5; % Keep this value positive and smaller than 0.5
% Surface friction
c = 0.02;
% Surface roughness wave number (texture)
kk=1;
% Siltering weight parameter
w=0.15;
% Viscous damping for the finger tip motion
mu=0.005;
% Viscous damping for the ball 
alpha=0.99;

% Initialization
hx=50;
hy=0;
hz=0;
Vxball=0;
Vyball=10;
Vzball=0;
% toc=cputime;

% The vertical position of the floor in millimeters.
floorPositionZ = 50;

% Set the color of the floor.
floorColor = [0 .7 0]; % green

% Set the alpha (transparency) of the floor.
floorAlpha = 0.5;

% The horizontal position of the wall in millimeters.
wallPositionX = 50;
wallPositionY = 50;

% Set the color of the wall.
wallColor = [0 .7 0]; % green

% Set the alpha (transparency) of the wall.
wallAlpha = 0.5;

% The location of the center of the ball, relative to the origin of
% the Phantom's coordinate frame.  X is positive toward the user, Y is
% positive to the right, and Z is positive up. All dimensions are in
% millimeters.
ballCenterX = 200;
ballCenterY = 0;
ballCenterZ = 175;

% Set the color of the ball.
ballColor = [.5 0 .5];

% Set the radius of the ball in millimeters.
ballRadius = 20;

% Set the alpha (transparency) of the ball.
ballAlpha = 0.5;

% PUT MORE VARIABLES HERE

% Set the scale of the force vector in millimeters per newton.
fScale = 40;


%% Set up the figure for graphing

% Open figure 1 and clear it, keeping the handle.
f = figure(1);
clf

%Plot a black dot that always stays in the code
hBoundaryDot = plot3(0, 0, 0,'k.','markersize',15);
hold on

% Plot a small red dot to represent the position of the PHANToM.  For
% now, we just put it at the origin.  We keep the handle to this plot
% (hPhantomDot) so that we can move the dot later.
hPhantomDot = plot3(0, 0, 0,'r.','markersize',15);

% Call tic to latch the time.
tic

% If we are not using hardware, set the key press function to enable
% keyboard control of the virtual Phantom.
if (~hardware)
    set(f,'keypressfcn',@setAccTime)
end

% Turn hold on to let us put more things on the graph.
hold on

% Plot a thick black line to represent the force vector.  For now, we
% just have it go from the origin to [10 10 10]; later we will set it
% to be a scaled version of the commanded force vector.  We keep the
% handle to this plot (hForceLine) so we can move the line later.
hForceLine = plot3([0 10], [0 10], [0 10],'k-','linewidth',2);

% Set the axis limits, the viewing direction, and other properties of
% the view.
axis equal vis3d
axlimit = 250;
axstep = 50;
axis(300 * [0 1 -.5 .5 0 1])
set(gca,'Xtick',0:axstep:axlimit,'ytick',-(.5*axlimit):axstep:(.5*axlimit), ...
    'ztick',0:axstep:axlimit)
view(100,16)
box on
grid on

% Label the axes.
xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')


%% Draw the graphics

% Plot the floor.
hfloor = fill3(axlimit*[0 0 1 1], 300*[-.5 .5 .5 -.5], ...
    floorPositionZ*[1 1 1 1], floorColor);
set(hfloor,'facealpha', floorAlpha)

% Plot the walls.
hwall1 = fill3([250,250,250,250],[-150,-150,150,150],[50,300,300,50], [0.7 0 0]);
set(hwall1,'facealpha', floorAlpha)

hwall2 = fill3([250,250,0,0],[150,150,150,150],[50,300,300,50], floorColor);
set(hwall2,'facealpha', floorAlpha)

hwall3 = fill3([250,250,0,0],[-150,-150,-150,-150],[50,300,300,50], floorColor);
set(hwall3,'facealpha', floorAlpha)

hwall4 = fill3([0,0,0,0],[-150,-150,150,150],[50,300,300,50], floorColor);
set(hwall4,'facealpha', floorAlpha)

hceiling = fill3(axlimit*[0 0 1 1], 300*[-.5 .5 .5 -.5], ...
    floorPositionZ*[1 1 1 1]+250*[1,1,1,1], floorColor);
set(hceiling,'facealpha', floorAlpha)

hplayerlimit = fill3([150,150,150,150],[-150,-150,150,150],[50,300,300,50], [0.847 0 0.8392]);
set(hplayerlimit,'facealpha', floorAlpha)

% Get the coordinates of the points on a unit sphere with resolution
% 20.
[xsphere, ysphere, zsphere] = sphere(20);

% Plot the ball as a surface at its center location, with its radius.
hBall = surf(ballRadius*xsphere+ballCenterX, ballRadius*ysphere+ballCenterY, ballRadius*zsphere+ballCenterZ);

% Set the transparency of the ball's faces and edges.
set(hBall,'facealpha',ballAlpha,'edgeAlpha',ballAlpha,'facecolor',ballColor)

%Button

[ybutton, zbutton, xbutton]=cylinder(10);

hButton = surf(-5*xbutton+250, ybutton+0, zbutton+175);

set(hButton,'facealpha',ballAlpha,'edgeAlpha',ballAlpha,'facecolor',[1 0 0])


% Turn hold off to stop allowing more graphing.
hold off


%% Do final preparations

% Start the Phantom, passing in hardware (true or false).
phantomStart(hardware)

% Initialize a vector to hold the time stamp for each cycle.
t = zeros(nCycles,1);

% Initialize vectors to store tip position and force output for each
% cycle.
hx_history = zeros(nCycles,1);
hy_history = zeros(nCycles,1);
hz_history = zeros(nCycles,1);
Fx_history = zeros(nCycles,1);
Fy_history = zeros(nCycles,1);
Fz_history = zeros(nCycles,1);

% Initialize a variable to hold the last time the graphics update was
% run.
lastGraphicsTime = 0;

%% Run the servo loopteam239/virtual_environment/phantomJointTorques.p>phantomJointTorques at 49
for i = 1:nCycles
    % Measure the time and store it in our vector of time stamps.  
    % The units are seconds.
%     toc=cputime;
    t(i) = toc;
    if i~=1
        deltat=t(i)-t(i-1);
    else
        deltat=0;
    end
    % Get the Phantom's joint angles in radians.
    theta123 = phantomJointAngles;

    % Use these joint angles to calculate the Phantom's tip position
    % in millimeters.
    pos = phantomTipPosition(theta123);
    deltax=pos(1)-hx;
    deltay=pos(1)-hy;
    deltaz=pos(1)-hz;
    
    % Split out the x, y, and z components of the tip position.
    hx = pos(1);
    hy = pos(2);
    hz = pos(3);
    
    %initally set them equal
    hx_bound = hx;
    hy_bound = hy;
    hz_bound = hz;
    
    %%% PHANTOM VELOCITY %%%%
    if i==1
        vx=0;
        vy=0;
        vz=0;
    else if i==2
        vx=deltax/deltat;
        vy=deltax/deltat;
        vz=deltax/deltat;
        end
    end
    vx=w*deltax/deltat+(1-w)*vx;
    vy=w*deltax/deltat+(1-w)*vy;
    vz=w*deltax/deltat+(1-w)*vz;
    
    %%%PHANTOM FORCES%%%%
    
    % Re-set force components to zero
    Fx = 0;
    Fy = 0;
    Fz = 0;
     
    % Wall, floor and ceiling repulsion
    if hz<50
        Fz=k*(50-hz);
        Fx=-c*abs(Fz)*vx*(1+0.01*sin(kk*hx)*sin(kk*hy));
        Fy=-c*abs(Fz)*vy*(1+0.01*sin(kk*hx)*sin(kk*hy));
        hz_bound = 50;
    end
    if hz>300
        Fz=k*(300-hz);
        Fx=-c*abs(Fz)*vx*(1+0.01*sin(kk*hx)*sin(kk*hy));
        Fy=-c*abs(Fz)*vy*(1+0.01*sin(kk*hx)*sin(kk*hy));
        hz_bound = 300;
    end
    if hx>250
        Fx=k*(300-hx);
        Fy=-c*abs(Fx)*vy*(1+0.01*sin(kk*hz)*sin(kk*hy));
        Fz=-c*abs(Fx)*vz*(1+0.01*sin(kk*hz)*sin(kk*hy));
        hx_bound = 250;
    end
    if hx<0
        Fx=k*(0-hx);
        Fy=-c*abs(Fx)*vy*(1+0.01*sin(kk*hz)*sin(kk*hy));
        Fz=-c*abs(Fx)*vz*(1+0.01*sin(kk*hz)*sin(kk*hy));
        hx_bound = 0;
    end
    if hy>150
        Fy=k*(150-hy);
        Fx=-c*abs(Fy)*vx*(1+0.01*sin(kk*hx)*sin(kk*hz));
        Fz=-c*abs(Fy)*vz*(1+0.01*sin(kk*hx)*sin(kk*hz));
        hy_bound = 150;
    end
    if hy<-150
        Fy=k*(-150-hy);
        Fx=-c*abs(Fy)*vx*(1+0.01*sin(kk*hx)*sin(kk*hz));
        Fz=-c*abs(Fy)*vz*(1+0.01*sin(kk*hx)*sin(kk*hz));
        hy_bound = -150;
    end
    
    % Ball repulsion
    if norm([hx-ballCenterX;hy-ballCenterY;hz-ballCenterZ])<=ballRadius
        Fx=Fx+k*(hx-ballCenterX);
        Fy=Fy+k*(hy-ballCenterY);
        Fz=Fz+k*(hz-ballCenterZ);
    end
    
    % Viscous force
    if (hx>150 && hx<250)
        Fx=Fx-mu*vx;
        Fy=Fy-mu*vy;
        Fz=Fz-mu*vz;
    end
    
    %%button physics  
     if (hx>245 && norm([hy-0;hz-175])<5)
        if hx<=250 
            Fx=-0.2*(hx-245);
        else if hx<253
           
            Fx=0;
            else
                Fx=-5;
            end
        end
     end
    
     %%point attractive force
     
     %point located on button at (250,0,175)
     
     ball_dist = norm([(ballCenterX-250);(ballCenterY-0);(ballCenterZ-175)]);
     
     tip_dist = norm([(hx-250);(hy-0);(hz-175)]);
     
     %checking if attractive force is on and if tip is in range of force
     if (button==1 && tip_dist<100)
            Fattract_x = g*(hx-250)/tip_dist;
            Fattract_y = g*(hy-0)/tip_dist;
            Fattract_z = g*(hz-175)/tip_dist;
         else
            Fattract_x=0;
            Fattract_y=0;
            Fattract_z=0;
     end
     
     %checking if attractive force is on and if ball is in range of force
     if (button==1 && ball_dist<100)
         Battract_x = g*(ballCenterX-250)/ball_dist;
         Battract_y = g*(ballCenterY-0)/ball_dist;
         Battract_z = g*(ballCenterZ-175)/ball_dist;
     else
         Battract_x=0;
         Battract_y=0;
         Battract_z=0;
     end
     
     %attraction on end-effector
     Fx = Fx+Fattract_x;
     Fy = Fy+Fattract_y;
     Fz = Fz+Fattract_z;
     
     %attraction on ball
     Vxball = vxball+Battract_x*deltat;
     Vyball = Vyball+Battract_y*deltat;
     Vzball = Vzball+Battract_z*deltat;
     
     
    %%% Ball MOVEMENT %%
    % Collision with the ball
    if ballCenterZ+ballRadius>300
        Vzball=-abs(Vzball);
    end
    if ballCenterZ-ballRadius<50
        Vzball=abs(Vzball);
    end
    if ballCenterX+ballRadius>250
        Vxball=-abs(Vxball);
    end
    if ballCenterX-ballRadius<0
        Vxball=abs(Vxball);
    end
    if ballCenterY+ballRadius>150
        Vyball=-abs(Vyball);
    end
    if ballCenterY-ballRadius<-150
        Vyball=abs(Vyball);
    end
    % Collision with the finger tip
    if norm([hx-ballCenterX;hy-ballCenterY;hz-ballCenterZ])<ballRadius
        Vxball=vx+Vxball-2*Vxball*(hx-ballCenterX)/norm([hx-ballCenterX;hy-ballCenterY;hz-ballCenterZ]);
        Vyball=vy+Vyball-2*Vyball*(hy-ballCenterY)/norm([hx-ballCenterX;hy-ballCenterY;hz-ballCenterZ]);
        Vzball=vz+Vzball-2*Vzball*(hz-ballCenterZ)/norm([hx-ballCenterX;hy-ballCenterY;hz-ballCenterZ]);
    end
    Vxball=min(max(Vxball,-1000),1000);
    Vyball=min(max(Vyball,-1000),1000);
    Vzball=min(max(Vzball,-1000),1000);
    
    % Viscous damping for the ball
    if hx <150
        Vxball=alpha*Vxball;
        Vyball=alpha*Vyball;
        Vzball=alpha*Vzball;
    end

    % Position update
    ballCenterX=ballCenterX+Vxball*deltat;
    ballCenterY=ballCenterY+Vyball*deltat;
    ballCenterZ=ballCenterZ+Vzball*deltat;

    % Command the calculated tip force, also providing the joint
    % angles.
    phantomTipForce(Fx, Fy, Fz, theta123);
        
    % Store this cycle's values in the history vectors.
    hx_history(i) = hx;
    hy_history(i) = hy;
    hz_history(i) = hz;
    Fx_history(i) = Fx;
    Fy_history(i) = Fy;
    Fz_history(i) = Fz;
   
    % Check how much time has elapsed since we last updated the
    % graphics.
    if (t(i) - lastGraphicsTime > 0.03)
        % Enough time has passed.
        
        % Update the graph by setting the data for the PHANToM's dot
        % to the position of the haptic device.
        set(hPhantomDot, 'xdata', hx, 'ydata', hy, 'zdata', hz)
        
        %Update boundry dot
        set(hBoundaryDot, 'xdata', hx_bound, 'ydata', hy_bound, 'zdata', hz_bound)
    
        % Update the graph by setting the data for the force line to
        % show a scaled version of the commanded force.
        set(hForceLine,'xdata',[hx hx+Fx*fScale],'ydata',[hy hy+Fy*fScale], 'zdata',[hz hz+Fz*fScale]);

        % Store this time for future comparisons.
        lastGraphicsTime = t(i);
        
        % Update the circle
        set(hBall, 'xdata', ballRadius*xsphere+ballCenterX , 'ydata', ballRadius*ysphere+ballCenterY, 'zdata', ballRadius*zsphere+ballCenterZ )
    end
    
    % Pause for one millisecond to keep things moving at a reasonable
    % pace.
    pause(.001)
end

% Stop the Phantom.
phantomStop


%% Plot the results

% Open figure 2 and clear it.
figure(2)
clf

% Plot positions over time in the top subplot.
subplot(2,1,1)
h = plot(t,[hx_history hy_history hz_history]);
set(h(1),'color',[0 .7 .7])
set(h(2),'color',[.7 .7 0])
set(h(3),'color',[.7 0 .7])
xlabel('Time (s)')
ylabel('Tip Position (mm)')
legend('h_x', 'h_y', 'h_z')

% Plot forces over time in the bottom subplot.
subplot(2,1,2)
h = plot(t,[Fx_history Fy_history Fz_history]);
set(h(1),'color',[0 .7 .7])
set(h(2),'color',[.7 .7 0])
set(h(3),'color',[.7 0 .7])
xlabel('Time (s)')
ylabel('Force (N)')
legend('F_x', 'F_y', 'F_z')