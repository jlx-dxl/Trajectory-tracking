%%
clear all;
clc;

%%
path=load('path_node.mat');
% t=[1:size(path,1)]';
% path_t=[t path];
v=animatedline;
path_1=animatedline('LineStyle','-','Color','r');
h_car= animatedline('color','m');
h_car_center = animatedline('Marker','o','Color','g','MaximumNumPoints',1); 
K=0.5;
s=2;
aerfa=pi/6;
% Ki=0.1;
% Kd=0.01;
l=0.5;
delta_T=0.5;

%% 静态场景
axis equal;
axis([0 90 0 20]) 
point.start=[4 14];
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

%% 小车尺寸
Pos=[7,15.5,0];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
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
B.wheel1.Pos=[Pos(1)-s*cos(aerfa),Pos(2)-s*sin(aerfa)];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
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

B.wheel2.Pos=[Pos(1)-s*cos(aerfa),Pos(2)+s*sin(aerfa)];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
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
%% 动态显示
for k=1:size(path,1)
    addpoints(h_car_center,Pos(1),Pos(2));
    addpoints(h_car,Pos(1),Pos(2));
    
    e_x=Pos(1)-path(k,1);   %x坐标期望与实际的差
    e_y=Pos(2)-path(k,2);   %y坐标期望与实际的差
    u_x=-K*e_x;
    u_y=-K*e_y;
%     u_x=-K*e_x(k)+Ki*sume_x(k)+Kd(e(k)-e(k-1));
%     u_y=-K*e_y(k)+Ki*sume_x(k)+Kd(e(k)-e(k-1));
    A_mat=[cos(Pos(3)) -l*sin(Pos(3));sin(Pos(3)) l*cos(Pos(3))];  %对向前l偏移点的旋转矩阵
    P=[cos(Pos(3)) 0;sin(Pos(3)) 0;0 1];
    U_mat=[u_x,u_y]';
    v_w_mat=A_mat\U_mat; %A\B表示A的逆乘以B
%     addpoints(v,k,v_w_mat(1));

%     A_I=kron(A_t,eye(Nu));
%     Ut=kron(ones(Nc,1),U);
%     umin=[-0.2;-0.54;];
%     umax=[0.2;0.332];
%     delta_umin=[0.05;-0.0082;];
%     delta_umax=[0.05;0.0082];
%     Umin=kron(ones(Nc,1),umin);
%     Umax=kron(ones(Nc,1),umax);
%     A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
%     b_cons_cell={Umax-Ut;-Umin+Ut};
%     A_cons=cell2mat(A_cons_cell);
%     b_cons=cell2mat(b_cons_cell);
%    % 状态量约束
%     M=10; 
%     delta_Umin=kron(ones(Nc,1),delta_umin);
%     delta_Umax=kron(ones(Nc,1),delta_umax);
%     lb=[delta_Umin;0];
%     ub=[delta_Umax;M];

    Pos(1)=Pos(1)+v_w_mat(1)*cos(Pos(3))-v_w_mat(2)*l*sin(Pos(3)); %下一时刻的x坐标
    Pos(2)=Pos(2)+v_w_mat(1)*sin(Pos(3))+v_w_mat(2)*l*cos(Pos(3)); %下一时刻的y坐标
    Pos(3)=Pos(3)+v_w_mat(2)*delta_T; %下一时刻的θ坐标
    addpoints(path_1,path(k,1),path(k,2));
    
    A.Rot = [cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*A.P; %车辆旋转
    A.Prot_trasl = A.Rot + [ones(1,4)*Pos(1); ones(1,4)*Pos(2)]; %车辆移动
    A.P_robot.XData=A.Prot_trasl(1,:)';
    A.P_robot.YData=A.Prot_trasl(2,:)'; %更新车辆位姿

    B.wheel1.Pos=[Pos(1)-s*cos(aerfa+Pos(3)),Pos(2)-s*sin(aerfa+Pos(3))];    
    B.wheel1.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel1.P; 
    %rotated car：旋转矩阵*四个角点的位置
    B.wheel1.Prot_trasl = B.wheel1.Rot + [ ones(1,4)*B.wheel1.Pos(1); ones(1,4)*B.wheel1.Pos(2)];
    %汽车位置的变化（结果仍是四个角点的位置）
    B.wheel1.P_robot.XData=B.wheel1.Prot_trasl(1,:)';
    B.wheel1.P_robot.YData=B.wheel1.Prot_trasl(2,:)';

    B.wheel2.Pos=[Pos(1)-s*cos(aerfa-Pos(3)),Pos(2)+s*sin(aerfa-Pos(3))];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
    B.wheel2.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel2.P; 
    %rotated car：旋转矩阵*四个角点的位置
    B.wheel2.Prot_trasl = B.wheel2.Rot + [ ones(1,4)*B.wheel2.Pos(1); ones(1,4)*B.wheel2.Pos(2)];
    %汽车位置的变化（结果仍是四个角点的位置）
    B.wheel2.P_robot.XData=B.wheel2.Prot_trasl(1,:)';
    B.wheel2.P_robot.YData=B.wheel2.Prot_trasl(2,:)';

    B.wheel3.Pos=[Pos(1)+s*cos(aerfa+Pos(3)),Pos(2)+s*sin(aerfa+Pos(3))];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
    B.wheel3.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel3.P; 
    %rotated car：旋转矩阵*四个角点的位置
    B.wheel3.Prot_trasl = B.wheel3.Rot + [ ones(1,4)*B.wheel3.Pos(1); ones(1,4)*B.wheel3.Pos(2)];
    %汽车位置的变化（结果仍是四个角点的位置）
    B.wheel3.P_robot.XData=B.wheel3.Prot_trasl(1,:)';
    B.wheel3.P_robot.YData=B.wheel3.Prot_trasl(2,:)';

    B.wheel4.Pos=[Pos(1)+s*cos(aerfa-Pos(3)),Pos(2)-s*sin(aerfa-Pos(3))];    %pos为车位姿信息（X,Y,θ），初始值为（0,12,1.5pi）
    B.wheel4.Rot = [ cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]*B.wheel4.P; 
    %rotated car：旋转矩阵*四个角点的位置
    B.wheel4.Prot_trasl = B.wheel4.Rot + [ ones(1,4)*B.wheel4.Pos(1); ones(1,4)*B.wheel4.Pos(2)];
    %汽车位置的变化（结果仍是四个角点的位置）
    B.wheel4.P_robot.XData=B.wheel4.Prot_trasl(1,:)';
    B.wheel4.P_robot.YData=B.wheel4.Prot_trasl(2,:)';

    pause(0.1)
    frame=getframe(gcf);
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256);
    if k==1
         imwrite(imind,cm,'experiment2.gif','gif', 'LoopCount',inf,'DelayTime',0.000001);
    end
    if rem(k,2)==0
         imwrite(imind,cm,'experiment2.gif','gif','WriteMode','append','DelayTime',0.000001);
    end
    drawnow
    drawnow
end
