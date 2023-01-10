clc;
clear all;
%close all;


%======Test Conditions List======
testConditions = [-10 -5 0 0 0 0;
    -10 -5 0 0 0 pi/2;
    -10 -5 0 0 0 pi;
    -10 -5 0 0 0 3/2*pi;
    -10 -5 0 0 0 -pi/4;
    -10 -5 0 -10 0 0;
    0 0 0 -10 -5 0;
    0 0 0 -10 -5 pi]; %x_i,y_i,theta_i,x_g,y_g,theta_g

%======Choose which test to do=====
test = 1

%for test=1:1:8
%======Code starts here========
%input initial coordinate & orientation here
x(1) = testConditions(test,1);
y(1) = testConditions(test,2);
theta(1) = testConditions(test,3);

t = 0;
dt = 0.05; 

xd = testConditions(test,4);
yd = testConditions(test,5);
thetad = testConditions(test,6);

kv = 0.5;
kw = 10;
kphi = 5;
loop = 40;

% initialize variables
dx = x(1)-xd;
dy = y(1)-yd;
dtheta = theta(1)-thetad; 
i = 1;
finish = 0;

while (abs(dx) > 0.01 || abs(dy) > 0.01 || abs(dtheta) > 0.01  || i <= 500) && i <= 1000
    % deviation from goal
    dx = x(i) - xd;
    dy = y(i) - yd;
    dtheta = mod(thetad - theta(i),  2*pi);
    
    %feedback: x, y, theta
    rho(i) = sqrt(dy^2 + dx^2);
    alpha(i) = mod(atan2(-dy, -dx), 2*pi) - theta(i);
    alpha(i) = mod(alpha(i),2*pi);
    
%     if alpha(i) > 2*pi
%         alpha(i) = alpha(i) - 2*pi;
%     elseif alpha(i) < - 2*pi
%         alpha(i) = alpha(i) + 2*pi;
%     end

    phi(i) = thetad - (theta(i) + alpha(i));
    phi(i) = mod(phi(i), 2*pi);

%     if phi(i) > 2*pi
%        phi(i) = phi(i) - 2*pi;
%     elseif alpha(i) < - 2*pi
%        phi(i) = phi(i) + 2*pi;
%     end
    
    v(i) = kv*rho(i)*cos(alpha(i));
    if abs(v(i)) > 5
        v(i) = sign(v(i))*5;
    end
    
    if alpha(i)==0
        omega(i) = kw*alpha(i) + (alpha(i) - kphi*phi(i))*kv;    
    else
        omega(i) = kw*alpha(i) + (alpha(i) - kphi*phi(i))*kv*cos(alpha(i))*sin(alpha(i))/alpha(i);
    end
    
    %simulation
    x(i+1) = x(i) + v(i)*cos(theta(i))*dt;
    y(i+1) = y(i) + v(i)*sin(theta(i))*dt;
    theta(i+1) = theta(i) + omega(i)*dt;
    
    t(i+1) = t(i) + dt;
    i = i + 1;
    
    if abs(dx) < 0.01 && abs(dy) < 0.01 && abs(dtheta) < 0.01
        if finish == 0
        finish = t(i)
        end
    end

end
%end

x = x'; y = y'; theta = theta'; t = t';

%plot(t, x, 'k-', t, y, 'b-.', t, theta, 'r-.');

pathLength = length(x);
Xc = x(1);
Yc = y(1);
fig1 = figure("Name","Simulation"); 
hold on;

plot (Xc,Yc,'r-','LineWidth',1);
% if (y(1)<=8) && (y(1)>=2) && (x(1)<42) && (x(1)>=1)
% plot(yd1,yd2,'y.','LineWidth',1);
% hold on;
% elseif (y(1)<=8) && (y(1)>=2) && (x(1)<=90) && (x(1)>48)
% plot(yd1,yd2,'y.','LineWidth',1);
% hold on;
% elseif (y(1)<16) && (y(1)>2) && (x(1)>=42) && (x(1)<=48)
% plot(xd,yd,'y.','LineWidth',1);
hold on;
%end
    
    
    
    
    
%f=[50:90];
%g=[0:40];
%h=[10:20];
%plot (g,10,'go',40,h,'go',50,h,'go',f,10,'go');
%hold on

axis equal
axis([-15 15 -15 15])
grid on
Xr_c  = 5;
Yr_c  = 0;
Xr_c1 = 5/1.4142;
Yr_c1 = 0;
gama  = pi/4;

path = size(t);
Na   = path(1,1);

for j = 1:1:length(t)  % change to M for sinewave, N for circle
R1   = 5/1.414;
R2   = 5/1.414;
Xc   = x(j);
Yc   = y(j);
fai  = theta(j); 
robot_width = 1.5;
Xr   = Xc+Xr_c*cos(fai)-Yr_c*sin(fai);
Yr   = Yc+Xr_c*sin(fai)+Yr_c*cos(fai);
Xr1  = Xc+Xr_c1*cos(fai+gama)-Yr_c1*sin(fai+gama);
Yr1  = Yc+Xr_c1*sin(fai+gama)+Yr_c1*cos(fai+gama);
%theta_r_dot = q(j,6);
%theta_l_dot = q(j,7);
sinfai(j) = sin(fai);
cosfai(j) = cos(fai);
%linearvelocity(j) = (theta_l_dot+theta_r_dot)*0.75/2;
%theta_r_dotre(j)  = theta_r_dot;
%theta_l_dotre(j)  = theta_l_dot;
%theta_r(j) = q(j,4);
%theta_l(j) = q(j,5);
%Xcre(j)  = q(j,1);
%Ycre(j)  = q(j,2);
faire(j) = fai;

[Robot] = Robotplot(Xc, Yc, fai, robot_width);

%********************* Create the 4 lines that draw the box of the mobile robot *************************************

Robot1x = [Robot(1,1) Robot(1,3)];
Robot1y = [Robot(1,2) Robot(1,4)];

Robot2x = [Robot(1,3) Robot(1,5)];
Robot2y = [Robot(1,4) Robot(1,6)];

Robot3x = [Robot(1,5) Robot(1,7)];
Robot3y = [Robot(1,6) Robot(1,8)];

Robot4x = [Robot(1,7) Robot(1,1)];
Robot4y = [Robot(1,8) Robot(1,2)];

Robotlinkx  = [Xr Xc];
Robotlinky  = [Yr Yc];

Robotlinkx1 = [Xr1 Xc];
Robotlinky1 = [Yr1 Yc];

Robotlinkx2 = [Xr1 Xr];
Robotlinky2 = [Yr1 Yr];


RobotBox(1,:) = [Robot1x Robot2x Robot3x Robot4x];
RobotBox(2,:) = [Robot1y Robot2y Robot3y Robot4y];

plot (Xc,Yc); 

XrYr         = [Xr Yr];
XcYc         = [Xc Yc];
Xr1Yr1       = [Xr1 Yr1];
XrYrtoXcYc   = pdist([XrYr;XcYc]);
Xr1Yr1toXcYc = pdist([Xr1Yr1;XcYc]);
XrYrtoXr1Yr1 = pdist([XrYr;Xr1Yr1]);

MMcos = (-(XrYrtoXcYc*XrYrtoXcYc-Xr1Yr1toXcYc*Xr1Yr1toXcYc-XrYrtoXr1Yr1*XrYrtoXr1Yr1)/(2*XrYrtoXr1Yr1*Xr1Yr1toXcYc));
MM(j) = sqrt(1 - MMcos^2);

if j==1
    
    h1 =plot(Robot1x, Robot1y,'b-','LineWidth',4);
    h2 =plot(Robot2x, Robot2y,'r-','LineWidth',1);
    h3 =plot(Robot3x, Robot3y,'b-','LineWidth',4);
    h4 =plot(Robot4x, Robot4y,'or-','LineWidth',1);
   % h5 =plot(Robotlinkx, Robotlinky,'ro','LineWidth',1);
    %h6 =plot(Robotlinkx1, Robotlinky1,'r-','LineWidth',1);
    %h7 =plot(Robotlinkx2, Robotlinky2,'r-','LineWidth',1);

%    set(h1,'EraseMode','xor');
%    set(h2,'EraseMode','xor');
%    set(h3,'EraseMode','xor');
%    set(h4,'EraseMode','xor');
    %set(h5,'EraseMode','xor');

elseif j==2
    h1 =plot(Robot1x, Robot1y,'b-','LineWidth',4);
    h2 =plot(Robot2x, Robot2y,'r-','LineWidth',1);
    h3 =plot(Robot3x, Robot3y,'b-','LineWidth',4);
    h4 =plot(Robot4x, Robot4y,'or-','LineWidth',1);
   %
else
    
    set(h1,'XData', Robot1x,'YData',Robot1y)
    set(h2,'XData', Robot2x,'YData',Robot2y)
    set(h3,'XData', Robot3x,'YData',Robot3y)
    set(h4,'XData', Robot4x,'YData',Robot4y)
    %set(h5,'XData', Robotlinkx,'YData',Robotlinky)
    %set(h6,'XData', Robotlinkx1,'YData',Robotlinky1)
    %set(h7,'XData', Robotlinkx2,'YData',Robotlinky2)

    end
    pause(0.001)
    
end
plot(x,y);legend(sprintf('Finish Time: %.2f',finish));
fig2 = figure("Name","Coordinate"); plot(t(1:500), x(1:500), 'k-', t(1:500), y(1:500), 'b-', t(1:500), theta(1:500), 'r-.'); legend('x','y','theta');
fig3 = figure("Name", "Velocity"); plot(t(2:500),omega(2:500),t(2:500),v(2:500)); legend('omega','v');
%saveas(fig1,'Result/Orig_'+ string(test)+'_Simulation.png');
%saveas(fig2,'Result/Orig_'+ string(test)+'_Coordinate.png');
%saveas(fig3,'Result/Orig_'+ string(test)+'_Velocity.png');
hold on;
