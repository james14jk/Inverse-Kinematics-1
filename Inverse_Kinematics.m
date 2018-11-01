clc; clear;
%DESIRED END EFFECTOR VALUES
    %NEXT THING IS TO DRAW IT ALL OUT AT ONCE AND CHECK ANGLES (MAY BE
    %WRONG) - ALSO CHECK OUT IF DAVID'S INVERSE KIN
%End Effector Position in Meters
x=0.1384;
y=0.1384;
z=-0.4810;
% x=0.1384;
% y=0.1384;
% z=-0.4810;
%Link Lengths in Meters
L1=.1;
L2=.25;
L3=.25;

%TOP VIEW
%Angle of Rotation in xy-plane (YAW)
t1=atan(y/x)
%Joint Position in xy-plane (4 joints: a,b,c,d)
xa=0
ya=0
xb=L1*cos(t1)
yb=L1*sin(t1)

%SIDE VIEW
%Joint Position in xz-plane (4 joints: a,b,c,d)
za=0
zb=0
%zc=L2*cos(t2) - don't know yet
zd=z

%MAJOR CALCULATIONS
%Reference Lengths
r1=sqrt((x-xb)^2+(0+0)^2) %between joint b & d (along x)
%r1=sqrt((x-xb)^2+(y-yb)^2) - PROBS WRONG
r2=abs(z)                 %between joint d & L1 horizontal (along x)
r3=sqrt((x-xb)^2+(z-zb)^2)         %between joint b & d
%r3=sqrt(r1^2+r2^2) - PROBS WRONG

%Reference Angles
phi1=acos((L2^2+r3^2-L3^2)/(2*L2*r3))
phi2=acos(r1/r3)
%phi2=atan(r2/r1) - MAYBE WRONG
%phi3=acos((L2^2+L3^2-r3^2)/(2*L2*L3)) %not needed

%Joint Angles in Radians (2^2=4 possibilities for t2: 1,2,3,4)
t1=pi/4           %setting to actual value to test last part***********
%t1=atan(y/x)
t3=-pi/8           %setting to actual value to test last part***********
%t3=-(pi-phi3)     %IS THIS SUPPOSED TO BE POSITIVE?

%normal walking (below body horizontal)
%assuming zd < 0    
t2=phi1-phi2     %possible1 joint 2 angle; dz=neg, cz=pos
t2f=phi1+phi2    %bad configuration for these scenarios
A=[t2,t2f]
t2=min(A)        %choose least theta 2 (leg always arch upward)
%t2=find(A==min(A)); 
if  t2 < 0       %if guess of cz is incorrect
    t2=phi2-phi1 %possible2 joint 2 angle; dz=neg, cz=neg
    fprintf('possibility2 dz=neg, cz=neg THIS HASNT BEEN FULLY TESTED needs negative?')
    A=[t2,t2f]
    t2=min(A)    %choose least theta 2 (leg always arch upward)
    t2=t2*(-1)   %THIS HASN'T BEEN FULLY TESTED needs negative?
end
if  zd > 0       %stepping up (above body horizontal)
    t2=phi1+phi2 %possible3 joint 2 angle; dz=pos, cz=pos
    fprintf('possibility3 dz=pos, cz=pos')
    t2f=phi1-phi2%bad configuration for this scenario
    A=[t2,t2f]
    t2=max(A)    %choose greatest theta 2 (leg always arch upward)
end
    fprintf('possibility1 dz=neg, cz=pos')
t2=-3*pi/8           %setting to actual value to test last part***********
%Reference Angles and Lengths continued (joint: c)
tb=pi-abs(t2)                          %angle next to joint b
r6=sqrt(L2^2+L1^2-(2*L2*L1*cos(tb)))   %between joint a & c
%ta=acos((L1^2+r6^2-L2^2)/(2*L1*r6))    %angle next to joint b %- NOT NEEDED
%tc=pi-(pi/2)-ta %- NOT NEEDED

%Joint Position in xz-plane (joints: c,d)
xc=L1+L2*cos(t2) %- v3
yc=xc*tan(t1) %- v3
xd=x
yd=y
zc=L2*sin(t2)
%zc=-L2*cos(tc) - original

%FINAL VALUES
%Joint Angles in Radians
t1=t1*(180/pi)
t2=t2*(180/pi)
t3=t3*(180/pi)

%GRAPHING THE LEG
x=[xa xb xc xd]
y=[ya yb yc yd]
z=[za zb zc zd]
hold on
plot3(0,0,0,'o-')   %colors: https://www.mathworks.com/help/matlab/ref/colorspec.html
plot3(x,y,z,'o-')   %colors: https://www.mathworks.com/help/matlab/ref/colorspec.html
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
