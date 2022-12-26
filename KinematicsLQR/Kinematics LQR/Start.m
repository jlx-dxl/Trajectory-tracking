clear;
clc;
load('autocross_2020.mat');
xr = refPoses(:,1)';      %全局x坐标
yr = refPoses(:,2)';      %全局y坐标
thetar =  refPoses(:,3)'; %以x轴为基准，逆时针旋转的航向角
kappar = refPoses(:,5)';  %道路曲率
index_num = length(xr);
Pre_distance = 1;%预瞄距离m
d_kr = sqrt((xr(2)-xr(1))^2+(yr(2)-yr(1))^2);%求两个点之间的距离；
Pre_kr_num =floor(Pre_distance/d_kr);%求出2m之内有多少个点
plot(refPoses(:,1),refPoses(:,2))
%% 车辆参数设置
%车身参数
cf=-54000;  %%前轮侧偏刚度之和
cr=-54000;  %%后轮侧偏刚度等于前轮侧偏刚度
m=260;      %%整车空载质量
Iz=300;     %%z轴转动惯量
a=1.0;      %%前轴距
b=0.6;      %%后轴距
Veh_Width = 1.49; % 车宽
Cg_Height = 0.35; %质心高





