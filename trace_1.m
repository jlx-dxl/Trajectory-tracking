%% 数据定义
%PID参数
Kp = 10;
Ki = 0.1;
Kd = 0.5;

verxs = [0];%误差序列 0便于微分/积分操作
verys = [0];
l = 1;%半车长
T = 0.01;%积分间隔
%x轴逆时针方向旋转规定为正
x = 0;%车辆当前位置
y = 0;%车辆当前位置
vx = 0;%x方向速度
vy = 0;%y方向速度
v = sqrt(vx^2+vy^2);%当前速度
r = 1+2*v*T;%视界 随v改变而变化


phi = 0;%车辆横摆角（车头指向与地面坐标系夹角）
theta = atan2(vy,vx);%航向角（速度方向与地面坐标系夹角）
b = theta-phi;%质心侧偏角（车辆速度与车头指向夹角）(车头指向逆时针方向为正)

w = v*sin(b)/l;%车辆角速度

phit = 0;%参考点姿态
xt = 0;%参考点位置
yt = 0;%参考点位置
vxt = 0;%参考点x方向速度大小
vyt = 0;%参考点y方向速度大小

%% 生成参考路径
%x方向单位长度代表1m
nodes = [0,50;30,50;30,20;50,20];
vp = [10,15,10];
%{
path = [];
subpath = ones(1,300)*50;%首段路径长度及位置
pathx = 0.1:0.1:30;%首段路径点间隔
subvx = ones(1,300)*10;%首段路径参考速度
subvy = zeros(1,300);
path = [pathx;subpath;subvx;subvy;];
paths = [paths,path];

path = [];
subpath = 50:-0.1:20;%首段路径长度及位置
pathx = ones(1,301)*30;%首段路径点间隔
subvx = zeros(1,301);%首段路径参考速度
subvy = ones(1,301)*5;
path = [pathx;subpath;subvx;subvy;];
%}
paths = getpath(nodes,vp);
plot(paths(1,:),paths(2,:));
hold on
%% 寻路
flag = 1;
%初始化车辆参数
x = 0.1;
y = 50;
phi = 0;
vx = 0;
vy = 0;
vxs = [vx];
vys = [vy];
vrefx = [0];
vrefy = [0];
time = [0];
i = 1;
phis = [phi];
phiref = [0];
%scatter(x,y)
%hold on
%车辆速度
while flag
r = 1+2*v*T;%视界 随v改变而变化
pt = find((paths(1,:)-x).^2+(paths(2,:)-y).^2-r^2<=0,1,'last');
xt = paths(1,pt);
yt = paths(2,pt);
erx = xt - x;
ery = yt - y;
d = sqrt(erx^2+ery^2);
vxt = paths(3,pt);
vyt = paths(4,pt);
vt = sqrt(vxt^2+vyt^2);%目标速度大小
vvxt = erx/d*vt;%车辆需要追踪的x方向速度
vvyt = ery/d*vt;%车辆需要追踪的y方向速度
vrefx(end+1) = vxt;
vrefy(end+1) = vyt;
phiref(end+1) = atan2(vyt,vxt);
verx = vvxt-vx;%x方向速度误差
very = vvyt-vy;%y方向速度误差
dverx = verx - verxs(end);%微分操作
dvery = very - verys(end);%微分操作
verxs(end+1) = verx;
verys(end+1) = very;
sverx = sum(verxs);%积分操作
svery = sum(verys);%积分操作
dvx = Kp*verx + Kd*dverx + Ki*sverx;
dvy = Kp*very + Kd*dvery + Ki*svery;
vx = vx + T*dvx;
vy = vy + T*dvy;
vxs(end+1) = vx;
vys(end+1) = vy;
time(end+1) = i*T;
i = i+1;
v = vx^2+vy^2;
x = x+vx*T;
y = y+vy*T;
theta = atan2(vy,vx);
b = theta - phi;
w = v*sin(b)/l;
phi = phi+w*T;
phis(end+1) = phi;
scatter(x,y)
hold on
if sqrt((x-paths(1,end))^2+(y-paths(2,end))^2)<1
    flag = 0;
end
end
figure()
subplot(1,3,1);
plot(time,vxs,time,vrefx)
legend('实际速度','参考速度')
subplot(1,3,2);
plot(time,vys,time,vrefy)
legend('实际速度','参考速度')
subplot(1,3,3);
plot(time,phis,time,phiref)
legend('实际横摆角','参考横摆角')











