clear;
clc;
load('autocross_2020.mat');
xr = refPoses(:,1)';      %ȫ��x����
yr = refPoses(:,2)';      %ȫ��y����
thetar =  refPoses(:,3)'; %��x��Ϊ��׼����ʱ����ת�ĺ����
kappar = refPoses(:,5)';  %��·����
index_num = length(xr);
Pre_distance = 1;%Ԥ�����m
d_kr = sqrt((xr(2)-xr(1))^2+(yr(2)-yr(1))^2);%��������֮��ľ��룻
Pre_kr_num =floor(Pre_distance/d_kr);%���2m֮���ж��ٸ���
plot(refPoses(:,1),refPoses(:,2))
%% ������������
%�������
cf=-54000;  %%ǰ�ֲ�ƫ�ն�֮��
cr=-54000;  %%���ֲ�ƫ�նȵ���ǰ�ֲ�ƫ�ն�
m=260;      %%������������
Iz=300;     %%z��ת������
a=1.0;      %%ǰ���
b=0.6;      %%�����
Veh_Width = 1.49; % ����
Cg_Height = 0.35; %���ĸ�





