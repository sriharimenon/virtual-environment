%% virtual_environment_team239
%
% Build an interactive virtual environment that can be touched using
% the Phantom.
%
% Starter Code written by Katherine J. Kuchenbecker for MEAM 520 at the University
% of Pennsylvania.
%
% Final Part 2 of Project 2 code written by JoAna Smith, Srihari Menon, and
% Hungtang KO

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
nCycles = 100000;

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

%%%%%%%%%%%%%%%%%%%%%%%%%% GIVEN VALUES %%%%%%%%%%%%%%%%%%%%%%%
% The vertical position of the floor in millimeters.
floorPositionZ = 50;

% Set the color of the floor.
floorColor = [0 0 1]; % blue

% Set the alpha (transparency) of the floor.
floorAlpha = 0.2;

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
ballAlpha = 0.9;

% Set the scale of the force vector in millimeters per newton.
fScale = 40;

%%%%%%%%%%%%%%%%%%%%%% OUR ADDITIONAL CONSTANTS %%%%%%%%%%%%%%%%%%%%%%
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
% Virtual mass
m = .01;
% button constants
g = -5; %negative implies attractive force

%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx=50; %initalize the position of the Phantom end-effector
hy=0;
hz=0;
Vxball=0; %initalize the velocity of the ball
Vyball=0;
Vzball=0;
button = 0; %initialize button to be "off"
count = 0; % used to prevent excessive button on/off toggling

%% Set up the figure for graphing

% Open figure 1 and clear it, keeping the handle.
f = figure(1);
clf

%Plot a black dot that always stays on the surface and in the boxed region
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

%%%%%%%%%%%%%%%%%%%%%%%%% WALLS %%%%%%%%%%%%%%%%%%%%%%%%
%The walls define the perimeter of our region. They are all haptically
% rendered as one-sided linear springs as well as have viscous
% friction that you can feel. To make it easy to identify, we have made all
% these walls Green
hwall1 = fill3([250,250,250,250],[-150,-150,150,150],[50,300,300,50], wallColor); %Green
set(hwall1,'facealpha', floorAlpha)

hwall2 = fill3([250,250,0,0],[150,150,150,150],[50,300,300,50], wallColor);%Green
set(hwall2,'facealpha', floorAlpha)

hwall3 = fill3([250,250,0,0],[-150,-150,-150,-150],[50,300,300,50], wallColor); %Green
set(hwall3,'facealpha', floorAlpha)

hwall4 = fill3([0,0,0,0],[-150,-150,150,150],[50,300,300,50], wallColor); %Green
set(hwall4,'facealpha', floorAlpha)

hceiling = fill3(axlimit*[0 0 1 1], 300*[-.5 .5 .5 -.5], ...
    floorPositionZ*[1 1 1 1]+250*[1,1,1,1], wallColor); %Green
set(hceiling,'facealpha', floorAlpha)

%%%%%%%%%%%%%%%%%%%%%%%% FLOOR %%%%%%%%%%%%%%%%%%%%%%%%
%In addition to spring forces and viscous friction, the floor will have
% haptic texture. The method of implementing texture will be explained in 
% another section. To make if easy to identify, we have made the color of
% the floor Blue
hfloor = fill3(axlimit*[0 0 1 1], 300*[-.5 .5 .5 -.5], ...
    floorPositionZ*[1 1 1 1], floorColor); %Blue
set(hfloor,'facealpha', 0.4)

%%%%%%%%%%%%%%%%%%%%%%%%%%%PLAYER WALL%%%%%%%%%%%%%%%%%%%%%%
% This is a 'virtual' wall that splits our region into 2 zones: the non-user
% zone (right side of the wall) and the user-zone (left side of the wall).
% The user can move about on the user-zone, but cannot be in the non-user 
% zone, while the ball can freely enter and exit both zones. Both the user and
% the ball feel viscous forces in the user-zone, but the ball does not
% feel these forces in the non-user zone. Because this is a 'virtual' wall
% whose aim is only to divide the region, there are no spring or viscous 
% forces exerted on the user or ball when coming in contact with it. 
% To make it easy to identify, we have made the color of this wall Blue.
hplayerlimit = fill3([150,150,150,150],[-150,-150,150,150],[50,300,300,50], floorColor); %Blue
set(hplayerlimit,'facealpha', 0.4)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SPHERE %%%%%%%%%%%%%%%%%%%%%%%%
% We have created a sphere that can move around its environment through 
% either an inital push by the user or turning 'On' the button attractive
% forces. The viscous forces present in the user environment result in the
% ball eventually slowing down to a stop. This makes it easier for the user
% to find the ball and push on it again. The lack of viscous forces in the
% non-user environment means that the ball will never stop in this region
% (as we are assuming all colisions are elastic, no loss in energy)

% Get the coordinates of the points on a unit sphere with resolution
% 20.
[xsphere, ysphere, zsphere] = sphere(20);

% Plot the ball as a surface at its center location, with its radius.
hBall = surf(ballRadius*xsphere+ballCenterX, ballRadius*ysphere+ballCenterY, ballRadius*zsphere+ballCenterZ);

% Set the transparency of the ball's faces and edges.
set(hBall,'facealpha',ballAlpha,'edgeAlpha',ballAlpha,'facecolor',ballColor) %black/purpleish

%%%%%%%%%%%%%%%%%%%%%%%%%%% CYLINDRICAL BUTTON %%%%%%%%%%%%%%%%%%%%%%
% We have created a virtual button that when pressed, initates an attractive
% forces about a point on the button (250,0,175). These attractive forces
% are felt on both the ball and the user. The method of initating this
% attractive force is explained in more detail in a later section. To make
% it easy to identify, the button is RED when 'OFF' and BLUE when 'ON'
[ybutton, zbutton, xbutton]=cylinder(10);
hButton = surf(-5*xbutton+250, ybutton+0, zbutton+175);
set(hButton,'facealpha',ballAlpha,'edgeAlpha',ballAlpha,'facecolor',[1 0 0]) %Initiate to Red

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
    
    %%%%%%%%%%%%%%%%%%%%%Intialize%%%%%%%%%%%%%%%%%%%% 
    
    % Measure the time and store it in our vector of time stamps.  
    % The units are seconds.
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
    
    %Calculate the change in position from the previous time
    deltax=pos(1)-hx;
    deltay=pos(1)-hy;
    deltaz=pos(1)-hz;
    
    % Split out the x, y, and z components of the tip position.
    hx = pos(1);
    hy = pos(2);
    hz = pos(3);
    
    %The h_bound variables represent the position of haptic device dot
    % that always stays within the boxed region. We initally set these
    % position values to be equal to the position of the Phantom's end
    % effector
    hx_bound = hx;
    hy_bound = hy;
    hz_bound = hz;    
    
    % Re-set force components to zero each time
    %Phantom end-effector Forces
    Fx = 0;
    Fy = 0;
    Fz = 0; 
    %Sphere forces
    Fballx = 0;
    Fbally = 0;
    Fballz = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% BUTTON %%%%%%%%%%%%%%%%%%%%%%%%
    %Click Button
    if (hx>=245 && norm([hy-0;hz-175])<5) %Phantom is close to the button
        if (hx>=245) && (hx<247) % button HAS NOT been fully pressed 
            Fx=-0.2*(hx-247); %gradual increase in force as button is being pressed
        elseif (hx >= 247 && hx < 250) % button HAS been pressed
            Fx = 0; %no longer feels a force back
            if (count == 1 ) %prevent it from toggling too much
            button = ~button; %toggle button value
            count = 0;
            end
        else %hit the wall
            Fx = -5; %feel strong repulsion force back
            if (count == 1 ) %prevent it from toggling too much
            button = ~button; %toggle button value
            count = 0;
            end
        end
    else
        count =1;
    end
    
    %Button 'ON' - Begin Attractive Force
     %When the button is'ON' an attractive force will be placed
     % on both the user (Phantom end effector) and the sphere. The location
     % of this attractive force will be at button (250,0,175) and the
     % radius of the attractive force is 100.
     
     %Calculate the distance of the end-effector (tip) and ball form the point of
     %attraction
     ball_dist = norm([(ballCenterX-250+ballRadius);(ballCenterY-0);(ballCenterZ-175)]);
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
         Battract_x = g*(ballCenterX-250+ballRadius)/ball_dist;
         Battract_y = g*(ballCenterY-0)/ball_dist;
         Battract_z = g*(ballCenterZ-175)/ball_dist;
     else
         Battract_x=0;
         Battract_y=0;
         Battract_z=0;
     end
     
     %Attraction on end-effector
     %Always add new forces to the previous forces calculated (if no
     %previous forces were calculated then F = 0 + Fattract because we
     %initialize all the forces to be 0 each time)
     Fx = Fx+Fattract_x;
     Fy = Fy+Fattract_y;
     Fz = Fz+Fattract_z;
     
     %Attraction on ball
     %Always add new forces to the previous forces calculated 
     Fballx = Fballx +Battract_x;
     Fbally = Fbally +Battract_y;
     Fballz = Fballz +Battract_z;
    
    %%%%%%%%%%%%%%%%%%%%%%% PHANTOM VELOCITY %%%%%%%%%%%%%%%%%%%%%%%%
    %Calclate velocity
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
    
    %Filter velocity 
    vx=w*deltax/deltat+(1-w)*vx;
    vy=w*deltax/deltat+(1-w)*vy;
    vz=w*deltax/deltat+(1-w)*vz;
    
    %%%%%%%%%%%%%%%%%%%%%%%PHANTOM FORCES%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Wall, floor and ceiling repulsion
    if hz<50 %Floor
        Fz= Fz+ k*(50-hz); %Spring force
        Fx= Fx-c*abs(Fz)*vx*(1+0.1*sin(kk*hx)*sin(kk*hy)); %viscous friction + sinusoidal texture
        Fy= Fy-c*abs(Fz)*vy*(1+0.1*sin(kk*hx)*sin(kk*hy));%viscous friction + sinusoidal texture
        hz_bound = 50; %for plotting black dot that stays on the surface
    end
    if hz>300 %Wall
        Fz= Fz + k*(300-hz);%Spring force
        Fx= Fx -c*abs(Fz)*vx;%viscous friction only
        Fy= Fy -c*abs(Fz)*vy;%viscous friction only
        hz_bound = 300;  %for plotting black dot that stays on the surface
    end
    if hx>250 %Wall
        Fx=Fx + k*(250-hx);
        Fy= Fy -c*abs(Fx)*vy;
        Fz= Fz -c*abs(Fx)*vz;
        hx_bound = 250;
    end
    if hx<150 %Wall
        Fx= Fx+k*(150-hx);
        Fy= Fy+-c*abs(Fx)*vy;
        Fz= Fz+-c*abs(Fx)*vz;
        hx_bound = 150;
    end
    if hy>150 %Wall
        Fy= Fy+k*(150-hy);
        Fx= Fx-c*abs(Fy)*vx;
        Fz= Fz-c*abs(Fy)*vz;
        hy_bound = 150;
    end
    if hy<-150 %Wall
        Fy= Fy+k*(-150-hy);
        Fx= Fx-c*abs(Fy)*vx;
        Fz= Fz-c*abs(Fy)*vz;
        hy_bound = -150;
    end
    
    % Hits Ball
    if norm([hx-ballCenterX;hy-ballCenterY;hz-ballCenterZ])<=ballRadius %has hit the ball
        Fballtipx=k*(hx-ballCenterX); %spring theory
        Fballtipy=k*(hy-ballCenterY);
        Fballtipz=k*(hz-ballCenterZ);
    else %did not hit the ball
        Fballtipx=0;
        Fballtipy=0;
        Fballtipz=0;
    end
    
    %Force felt on the Phantom when hit the ball
    Fx=Fx+Fballtipx;
    Fy=Fy+Fballtipy;
    Fz=Fz+Fballtipz;
    
    % Viscous damping
    if (hx>150 && hx<250) %when in the User region
        Fx=Fx-mu*vx;
        Fy=Fy-mu*vy;
        Fz=Fz-mu*vz;
    end
    
    % Command the calculated tip force, also providing the joint
    % angles.
    phantomTipForce(Fx, Fy, Fz, theta123);
     
    %%%%%%%%%%%%%%%%%%%%%%% BALL FORCES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Wall collision
    if ballCenterZ+ballRadius>300
        Vzball=-abs(Vzball); %elastic collision, only change velocity direction
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
    
    % Phantom Collision
    % feels the same force felt by the phantom but in the opposite
    % direction
    Fballx= Fballx -Fballtipx;
    Fbally= Fbally -Fballtipy;
    Fballz= Fballz -Fballtipz;
    
    % Viscous damping for the ball
    if ballCenterX >150 %when in the user region
        Fballx=Fballx-mu*Vxball;
        Fbally=Fbally-mu*Vyball;
        Fballz=Fballz-mu*Vzball;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%% BALL POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Use Forces to calculate the ball Velocity
    %if no forces are exerted on the ball, the velcoity will remain the
    %same as the previous time
    Vxball=Vxball+Fballx/m*deltat;
    Vyball=Vyball+Fbally/m*deltat;
    Vzball=Vzball+Fballz/m*deltat;
   
    % Velocity limit - don't want extremely high ball velocities
    Vxball=min(max(Vxball,-1000),1000);
    Vyball=min(max(Vyball,-1000),1000);
    Vzball=min(max(Vzball,-1000),1000);

    % Use Velocity to calculate the ball Position
    ballCenterX=ballCenterX+Vxball*deltat;
    ballCenterY=ballCenterY+Vyball*deltat;
    ballCenterZ=ballCenterZ+Vzball*deltat;   

    %%%%%%%%%%%%%%%%%%%%%%% STORE VALUES %%%%%%%%%%%%%%%%%%%%%%%%%
    % Store this cycle's values in the history vectors.
    hx_history(i) = hx;
    hy_history(i) = hy;
    hz_history(i) = hz;
    Fx_history(i) = Fx;
    Fy_history(i) = Fy;
    Fz_history(i) = Fz;
     
    %%%%%%%%%%%%%%%%%%%%%%% PLOT CHANGES %%%%%%%%%%%%%%%%%%%%%%%%
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
        
        % Update the sphere
        set(hBall, 'xdata', ballRadius*xsphere+ballCenterX , 'ydata', ballRadius*ysphere+ballCenterY, 'zdata', ballRadius*zsphere+ballCenterZ )
        
        %Change button color depending if it's 'ON' or 'OFF'
        if button==1
        	set(hButton,'facealpha',ballAlpha,'edgeAlpha',ballAlpha,'facecolor',[0 0 1]) %Blue - "On"
        else
            set(hButton,'facealpha',ballAlpha,'edgeAlpha',ballAlpha,'facecolor',[1 0 0])% Red - "Off"
        end 

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