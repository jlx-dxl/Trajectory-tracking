plot(path_node(:,2),path_node(:,1));
hold on;
x=path_node(:,2)';
y=path_node(:,1)';
values = spcrv([x;y],16);
% path=values';
% save('path.mat',"path");
plot(values(1,:),values(2,:))



