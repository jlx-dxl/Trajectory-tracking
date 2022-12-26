clc
clear

%% 1.建立原始栅格地图
%% 构建颜色MAP图
% identifier setting
Obstacle = 2;
Origin = 3;
Destination = 4;
Finished = 5;
Unfinished = 6;
Path = 7;

% color setting
white = [1,1,1];
black = [0,0,0];
green = [0,1,0];
yellow = [1,1,0];
red = [1,0,0];
blue = [0,0,1];
cyan = [0,1,1];
color_list = [white; black; green; yellow; red; blue; cyan];
colormap(color_list);

%% 构建栅格地图场景
% 栅格界面大小:行数和列数
rows = 20;  
cols = 90; 

% 定义栅格地图全域，并初始化空白区域
field = ones(rows, cols);

% 障碍物区域
% obstacle1(4,24)
for i=1:3
    for j=1:7
        field(3+i,20+j)=2;
    end
end
% obstacle2(10,44)
for i=1:3
    for j=1:7
        field(9+i,40+j)=2;
    end
end
% obstacle3(15,44)
for i=1:3
    for j=1:7
        field(14+i,40+j)=2;
    end
end
% obstacle4(15,64)
for i=1:3
    for j=1:7
        field(14+i,60+j)=2;
    end
end

% 起始点和目标点
% start(4,7)
for i=1:3
    for j=1:7
        field(3+i,3+j)=3;
    end
end
% goal(15,84)
for i=1:3
    for j=1:7
        field(14+i,80+j)=4;
    end
end

%% 画栅格图
figure(1);
image(0.5,0.5,field);
grid on;
axis equal;
axis([0,cols,0,rows])
set(gca,'gridline','-','gridcolor','k','linewidth',0.1,'GridAlpha',1);  
%设置栅格线条的样式（颜色、透明度等）
set(gca,'xtick',0:1:cols,'ytick',0:1:rows)

save('field.mat',"field")

%% 2.建立抽象栅格地图
%% 对障碍物进行膨胀处理
dilateR=4;
field1 = ones(rows, cols);
% 障碍物区域膨胀
% obstacle1
for i=-dilateR:dilateR
    for j=-2*dilateR:2*dilateR
        field1(5+i,24+j)=2;
    end
end
% obstacle2
for i=-dilateR:dilateR
    for j=-2*dilateR:2*dilateR
        field1(11+i,44+j)=2;
    end
end
% obstacle3
for i=-dilateR:dilateR
    for j=-2*dilateR:2*dilateR
        field1(16+i,44+j)=2;
    end
end
% obstacle3
for i=-dilateR:dilateR
    for j=-2*dilateR:2*dilateR
        field1(16+i,64+j)=2;
    end
end
% start
field1(5,7)=3;
% goal
field1(16,84)=4;

%% 画栅格图
figure(2);
colormap(color_list);
image(0.5,0.5,field1);
grid on;
axis equal;
axis([0,cols,0,rows])
set(gca,'gridline','-','gridcolor','k','linewidth',0.1,'GridAlpha',1); 
set(gca,'xtick',0:1:cols,'ytick',0:1:rows)

save('field1.mat',"field1")



