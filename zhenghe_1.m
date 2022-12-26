clear all;

%% 静态场景
axis equal;
axis([0 90 0 20]) 
point.start=[3.5 14];
point.end=[79 2.5];
obstacle.obs1.y=20.-[6 3 3 6];
obstacle.obs1.x=[20 20 27 27];
obstacle.obs2.y=20.-[12 9 9 12];
obstacle.obs2.x=[41 41 48 48];
obstacle.obs3.y=20.-[17 14 14 17];
obstacle.obs3.x=[41 41 48 48];
obstacle.obs4.y=20.-[17 14 14 17];
obstacle.obs4.x=[60 60 67 67];
patch(obstacle.obs1.x,obstacle.obs1.y,'b')
patch(obstacle.obs2.x,obstacle.obs2.y,'b')
patch(obstacle.obs3.x,obstacle.obs3.y,'b')
patch(obstacle.obs4.x,obstacle.obs4.y,'b')
rectangle('Position',[point.start 7 3],'EdgeColor','g')
rectangle('Position',[point.end 7 3],'EdgeColor','y')

%% 数据定义
%PID参数
Kp = 30;
Ti =inf;
Td = 0;

verxs = [0];%误差序列 0便于微分/积分操作
verys = [0];
l = 5;%轴距
l_2=l/2;%半轴距
s=2;
aerfa=pi/6;
T = 0.01;%积分间隔
%x轴逆时针方向旋转规定为正
vx = 0;%x方向速度
vy = 0;%y方向速度
vxs=[vx];
vys=[vy];
v = sqrt(vx^2+vy^2);%当前速度
r = 1+2*v*T;%视界 随v改变而变化
theta = atan2(vy,vx);%航向角（速度方向与地面坐标系夹角）
phit = 0;%参考点姿态
xt = 0;%参考点位置
yt = 0;%参考点位置
vxt = 0;%参考点x方向速度大小
vyt = 0;%参考点y方向速度大小
%% 小车尺寸
Pos=[4,15.5,0];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
A.R_w = 3/2;   %robot width/2
A.R_l = 7/2;   % robot length/2
A.a1 = [-A.R_l -A.R_w]';
A.b1 = [A.R_l -A.R_w]';
A.b2 = [A.R_l A.R_w]';
A.c = [-A.R_l A.R_w]';
A.P = [A.a1 A.b1 A.b2 A.c]; %四个角点的位置
A.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*A.P; 
%rotated car：旋转矩阵*四个角点的位置
A.Prot_trasl = A.Rot + [ ones(1,4)*Pos(1); ones(1,4)*Pos(2)];
%汽车位置的变化（结果仍是四个角点的位置）
A.P_robot=patch(A.P(1,:),A.P(2,:),'r'); 
%Patch：绘制一个填充多边形区域
A.P_robot.XData=A.Prot_trasl(1,:)';
A.P_robot.YData=A.Prot_trasl(2,:)';

%% 车轮
% 后轮
B.wheel1.Pos=[Pos(1)-s*cos(aerfa+Pos(3)),Pos(2)-s*sin(aerfa+Pos(3))];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
B.wheel1.R_w = 1/4;   %robot width/2
B.wheel1.R_l = 7/12;   % robot length/2
B.wheel1.a1 = [-B.wheel1.R_l -B.wheel1.R_w]';
B.wheel1.b1 = [B.wheel1.R_l -B.wheel1.R_w]';
B.wheel1.b2 = [B.wheel1.R_l B.wheel1.R_w]';
B.wheel1.c = [-B.wheel1.R_l B.wheel1.R_w]';
B.wheel1.P = [B.wheel1.a1 B.wheel1.b1 B.wheel1.b2 B.wheel1.c]; %四个角点的位置
B.wheel1.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel1.P; 
%rotated car：旋转矩阵*四个角点的位置
B.wheel1.Prot_trasl = B.wheel1.Rot + [ ones(1,4)*B.wheel1.Pos(1); ones(1,4)*B.wheel1.Pos(2)];
%汽车位置的变化（结果仍是四个角点的位置）
B.wheel1.P_robot=patch(B.wheel1.P(1,:),B.wheel1.P(2,:),'k'); 
%Patch：绘制一个填充多边形区域
B.wheel1.P_robot.XData=B.wheel1.Prot_trasl(1,:)';
B.wheel1.P_robot.YData=B.wheel1.Prot_trasl(2,:)';

B.wheel2.Pos=[Pos(1)-s*cos(aerfa-Pos(3)),Pos(2)+s*sin(aerfa-Pos(3))];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
B.wheel2.R_w = 1/4;   %robot width/2
B.wheel2.R_l = 7/12;   % robot length/2
B.wheel2.a1 = [-B.wheel2.R_l -B.wheel2.R_w]';
B.wheel2.b1 = [B.wheel2.R_l -B.wheel2.R_w]';
B.wheel2.b2 = [B.wheel2.R_l B.wheel2.R_w]';
B.wheel2.c = [-B.wheel2.R_l B.wheel2.R_w]';
B.wheel2.P = [B.wheel2.a1 B.wheel2.b1 B.wheel2.b2 B.wheel2.c]; %四个角点的位置
B.wheel2.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel2.P; 
%rotated car：旋转矩阵*四个角点的位置
B.wheel2.Prot_trasl = B.wheel2.Rot + [ ones(1,4)*B.wheel2.Pos(1); ones(1,4)*B.wheel2.Pos(2)];
%汽车位置的变化（结果仍是四个角点的位置）
B.wheel2.P_robot=patch(B.wheel2.P(1,:),B.wheel2.P(2,:),'k'); 
%Patch：绘制一个填充多边形区域
B.wheel2.P_robot.XData=B.wheel2.Prot_trasl(1,:)';
B.wheel2.P_robot.YData=B.wheel2.Prot_trasl(2,:)';

% 前轮
B.wheel3.Pos=[Pos(1)+s*cos(aerfa+Pos(3)),Pos(2)+s*sin(aerfa+Pos(3))];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
B.wheel3.R_w = 1/4;   %robot width/2
B.wheel3.R_l = 7/12;   % robot length/2
B.wheel3.a1 = [-B.wheel3.R_l -B.wheel3.R_w]';
B.wheel3.b1 = [B.wheel3.R_l -B.wheel3.R_w]';
B.wheel3.b2 = [B.wheel3.R_l B.wheel3.R_w]';
B.wheel3.c = [-B.wheel3.R_l B.wheel3.R_w]';
B.wheel3.P = [B.wheel3.a1 B.wheel3.b1 B.wheel3.b2 B.wheel3.c]; %四个角点的位置
B.wheel3.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel3.P; 
%rotated car：旋转矩阵*四个角点的位置
B.wheel3.Prot_trasl = B.wheel3.Rot + [ ones(1,4)*B.wheel3.Pos(1); ones(1,4)*B.wheel3.Pos(2)];
%汽车位置的变化（结果仍是四个角点的位置）
B.wheel3.P_robot=patch(B.wheel3.P(1,:),B.wheel3.P(2,:),'k'); 
%Patch：绘制一个填充多边形区域
B.wheel3.P_robot.XData=B.wheel3.Prot_trasl(1,:)';
B.wheel3.P_robot.YData=B.wheel3.Prot_trasl(2,:)';

B.wheel4.Pos=[Pos(1)+s*cos(aerfa-Pos(3)),Pos(2)-s*sin(aerfa-Pos(3))];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
B.wheel4.R_w = 1/4;   %robot width/2
B.wheel4.R_l = 7/12;   % robot length/2
B.wheel4.a1 = [-B.wheel4.R_l -B.wheel4.R_w]';
B.wheel4.b1 = [B.wheel4.R_l -B.wheel4.R_w]';
B.wheel4.b2 = [B.wheel4.R_l B.wheel4.R_w]';
B.wheel4.c = [-B.wheel4.R_l B.wheel4.R_w]';
B.wheel4.P = [B.wheel4.a1 B.wheel4.b1 B.wheel4.b2 B.wheel4.c]; %四个角点的位置
B.wheel4.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel4.P; 
%rotated car：旋转矩阵*四个角点的位置
B.wheel4.Prot_trasl = B.wheel4.Rot + [ ones(1,4)*B.wheel4.Pos(1); ones(1,4)*B.wheel4.Pos(2)];
%汽车位置的变化（结果仍是四个角点的位置）
B.wheel4.P_robot=patch(B.wheel4.P(1,:),B.wheel4.P(2,:),'k'); 
%Patch：绘制一个填充多边形区域
B.wheel4.P_robot.XData=B.wheel4.Prot_trasl(1,:)';
B.wheel4.P_robot.YData=B.wheel4.Prot_trasl(2,:)';
%% 载入路径
load('path_node');
vset=15;
paths=getpath(path_node,ones(length(path_node)-1)*vset);
%hold on;
%plot(paths(1,:),paths(2,:));

%% 寻路
flag = 1;
%初始化车辆参数
CTE_list = [0];
vx = 0;
vy = 0;
xs = [Pos(1)];
ys = [Pos(2)];

% scatter(Pos(1),Pos(2))
% hold on
%车辆速度
i = 1;
refvx = [0];
refvy = [0];
refphi = [0];
phis = [0];
time = [0];
while flag
r = 1+2*v*T;%视界 随v改变而变化
pt = find((paths(1,:)-Pos(1)).^2+(paths(2,:)-Pos(2)).^2-r^2<=0,1,'last');
xt = paths(1,pt);
yt = paths(2,pt);
erx = xt - Pos(1);
ery = yt - Pos(2);
d = sqrt(erx^2+ery^2);
vxt = paths(3,pt);
vyt = paths(4,pt);
vt = sqrt(vxt^2+vyt^2);%目标速度大小
refvx(end+1) = vxt;
refvy(end+1) = vyt;
refphi(end+1) = atan2(vyt,vxt);
time(end+1) = i*T;
i = i + 1;
vvxt = erx/d*vt;%车辆需要追踪的x方向速度
vvyt = ery/d*vt;%车辆需要追踪的y方向速度
verx = vvxt-vx;%x方向速度误差
very = vvyt-vy;%y方向速度误差
dverx = verx - verxs(end);%微分操作
dvery = very - verys(end);%微分操作
verxs(end+1) = verx;
verys(end+1) = very;
sverx = sum(verxs);%积分操作
svery = sum(verys);%积分操作
dvx = Kp*(verx + Td*dverx + sverx/Ti);
dvy = Kp*(very + Td*dvery + svery/Ti);
vx = vx + T*dvx;
vy = vy + T*dvy;
vxs(end+1) = vx;
vys(end+1) = vy;
v = vx^2+vy^2;
Pos(1) = Pos(1)+vx*T;
Pos(2) = Pos(2)+vy*T;
[CTE,~] = min((paths(1,:)-Pos(1)).^2+(paths(2,:)-Pos(2)).^2);
CTE_list(end+1) = CTE;
xs(end+1) = Pos(1);
ys(end+1) = Pos(2);
theta = atan2(vy,vx);
b = theta - Pos(3);%质心侧偏角（车辆速度与车头指向夹角）(车头指向逆时针方向为正)
w = v*sin(b)/l;%车辆角速度
Pos(3) = Pos(3)+w*T;
phis(end+1) = Pos(3);
% scatter(Pos(1),Pos(2))
% hold on
% pause(0.001)
if sqrt((Pos(1)-paths(1,end))^2+(Pos(2)-paths(2,end))^2)<1
    flag = 0;
end
end

%% 画图
figure()
subplot(3,1,1);
plot(time,vxs,time,refvx)
legend('实际速度','参考速度')
title('x方向速度')

subplot(3,1,2);
plot(time,vys,time,refvy)
legend('实际速度','参考速度')
title('y方向速度')
subplot(3,1,3);
plot(time,phis,time,refphi)
legend('实际横摆角','参考横摆角')
title('横摆角')
figure()
plot(time,sqrt(vys.*vys+vxs.*vxs),time,sqrt(refvy.^2+refvx.^2))
legend('速度大小','参考速度大小')
title('合速度')
axis([-0.5,max(time),0,vset+1.5])
figure()
plot(time,CTE_list);
title('横向跟踪误差')
% figure()
% plot(paths(1,:),paths(2,:));
% hold on
% plot(xs,ys)
% legend('参考路径','实际路径')
% title('跟踪结果')



