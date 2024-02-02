rosshutdown
ipaddress = 'http://10.10.234.54:11311';
rosinit(ipaddress)
[r,c]=size(path);
P=path/200-startLocation/200;
Yrr=P(:,1);
Xrr=P(:,2);
XR=[];
YR=[];
XR=Xrr(1,1):0.05:Xrr(end,1);
YR=spline(Xrr,Yrr,XR);
XR=XR';
YR=YR';
rostopic list
odom = rossubscriber('/odom');
robot = rospublisher('/cmd_vel') ;
velmsg = rosmessage(robot);
res = rospublisher('/reset'); 
resmsg = rosmessage(res);
send(res, resmsg);

XA=[];
YA=[];
i=0;
%Path_1
kp_d=2;
kd_d=0.3;
ki_d=2;
kp_p=2;
kd_p=0.1;
ki_p=0.6;
% kp_d=2;
% kd_d=0.3;
% ki_d=5;
% kp_p=2;
% kd_p=0.1;
% ki_p=0.6;
PDE=0;PPE=0;
IIP=0;IID=0;
for TT=1:length(XR)
    i=i+1;
    dt=1;
    %Gettin the position
    odomdata = receive(odom, 3); % timout is 3s
    pose = odomdata.Pose.Pose;
    x1 = pose.Position.X
    y1 = pose.Position.Y
   
    odomdata = receive(odom, 3); % timout is 3s
    pose = odomdata.Pose.Pose;
    x2 = pose.Position.X
    y2 = pose.Position.Y
    XA=[XA,x2];
    YA=[YA,y2];
    theta=atan2(y2-y1,x2-x1);
    %z = pose.Position.Z
    %X(i,1:3)=[x,y,z];
    %Desired position
    %Line trajec: Y=X
    xr=XR(TT,1);
    yr=YR(TT,1);
    %error
    xe=xr-x2;
    ye=yr-y2;
    
    D=sqrt(xe^2+ye^2)
    cp=cross([cos(theta),sin(theta),0],[xe,ye,0]);
    sgn=sign(cp(1,3));
    phe=sgn*acos((xe*cos(theta)+ye*sin(theta))/D);
    %Controller
    DDD=(D-PDE)/dt;
    IID=IID+D*dt;
    dp=kp_d*D;
    dd=kd_d*DDD;
    di=ki_d*IID;
    velc=dp+dd+di;
    DDP=(phe-PPE)/dt;
    IIP=IIP+phe*dt;
    pp=kp_p*phe;
    pd=kd_p*phe;
    pi=ki_p*phe;
    angu=pp+pd+pi;
    %Setting velocities and angular velocities
    velocity = velc;     
    w = angu;
    velmsg.Linear.X= velocity;
    velmsg.Angular.Z = w;
    send(robot,velmsg);
    PDE=D;PPE=phe;
    pause(dt/5);
    plot(XA(:,2:end),YA(:,2:end));
    hold on
    plot(XR(2:end,:),YR(2:end,:));
    hold on
    drawnow

end

% to stop the robot
velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(robot, velmsg);
% plot(XA(:,2:end),YA(:,2:end));
% hold on
% plot(XR(2:end,:),YR(2:end,:));
    
