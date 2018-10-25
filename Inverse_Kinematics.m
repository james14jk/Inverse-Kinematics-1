clc; clear;
%DESIRED END EFFECTOR VALUES
%End Effector Position in Meters
x=0.4933;
y=0.0317;
z=0.0518;
%James testing values
% x=0.4062;
% y=-0.1082;
% z=-0.1768;
%James testing values
%Link Lengths in Meters
L1=.1;
L2=.25;
L3=.25;

%TOP VIEW
%Angle of Rotation in xy-plane (YAW)
t1=atan(y/x);
%Joint Position in xy-plane (4 joints: a,b,c,d)
xa=0;
ya=0;
xb=L1*cos(t1);
yb=L1*sin(t1);
xc=(L1+L2)*cos(t1);
yc=(L1+L2)*sin(t1);
xd=x;
%xd=(L1+L2+L3)*cos(t1);
yd=y;
%yd=(L1+L2+L3)*sin(t1);

%SIDE VIEW
%Joint Position in xz-plane (4 joints: a,b,c,d)
za=0;
zb=0;
%zc=L2*cos(t2) - don't know yet
zd=z;            %don't know yet?

%MAJOR CALCULATIONS
%Reference Lengths
r1=sqrt((x-xb)^2+(y-yb)^2);
r2=zd;
r3=sqrt(r1^2+r2^2);
%r3= sqrt(r1^2+r2^2); original
r4=sqrt(x^2+y^2);

%Reference Angles
phi1=acos((L2^2+r3^2-L3^2)/(2*L2*r3));
%phi1=acos((-L2^2+L1^2+r3^2)/(2*L1*r3)); original
phi2=atan(r2/r1);
phi3=acos((L2^2+L3^2-r3^2)/(2*L2*L3));
%phi3=acos((-r3^2+L1^2+L2^2)/(2*L1*L2)); original 

%Joint Angles in Radians (2^2=4 possibilities for t2: 1,2,3,4)
t1=atan(y/x);
t3=pi-phi3;
%normal walking (below body horizontal)
%assuming zd < 0    
t2=phi1-phi2 %possible joint 2 angle; dz=neg, cz=pos
if    t2 < 0    %if guess of cz is incorrect
    t2=phi2-phi1 %possible joint 2 angle; dz=neg, cz=neg
    A=[t2,t2f]
    t2=min(A) %choose least theta 2 (leg always arch upward)
    t2=t2*(-1)
    %t2=find(A==min(A))
end   
    t2f=phi1+phi2 %bad configuration for these scenarios
    A=[t2,t2f]
    t2=min(A) %choose least theta 2 (leg always arch upward)
    %t2=find(A==min(A)) 
if  zd > 0    %stepping up  (above body horizontal)
    t23=phi1+phi2 %possible joint 2 angle; dz=pos, cz=pos
    %t2d=...       %dz=pos, cz=neg; leg would alwyas be U-shapped
    t2f=phi1-phi2 %bad configuration for this scenario
    A=[t2,t2f]
    t2=max(A) %choose greatest theta 2 (leg always arch upward)
    %t2=find(A==min(A))
end
%original
% t3=pi-phi3;
% if t3 < 0  %theta 3 always positive 
%       fprintf('theta 3 changed to positive' )
%       t3=(pi-phi3)*(-1);
% end
% ta=t2a*(180/pi)
% tb=t2b*(180/pi)
%original

%Joint Position in xz-plane (last joint: c)
zc=L2*cos(t2)

%FINAL VALUES
%Joint Angles in Radians
t1=t1*(180/pi)
t2=t2*(180/pi)
t3=t3*(180/pi)

%GRAPHING THE LEG
x=[xa xb xc xd]
y=[ya yb yc yd]
z=[za zb zc zd]
%x=[0 L3*cos(t1) L3*cos(t1)+L1*cos(t2) L3*cos(t1)+L1*cos(t2)+L2*cos(t2+t3)]  %original
%y=[0 L3*sin(t1) L3*sin(t1)+L1*cos(t2)*sin(t1) L3*sin(t1)+L1*cos(t2)*sin(t1)+L2*cos(t2+t3)*sin(t1)]  %original
%z=[0 0          L1*sin(t2) L1*sin(t2)+L2*sin(t2+t3)]  %original
%James checking values
% x = [0 1 3 4]
% y = [0 0 .3 -1]
% z = [0 0 .6 -1.7]
%James checking values
hold on
plot3(0,0,0,'o-')   %colors: https://www.mathworks.com/help/matlab/ref/colorspec.html
plot3(x,y,z,'o-')   %colors: https://www.mathworks.com/help/matlab/ref/colorspec.html
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
