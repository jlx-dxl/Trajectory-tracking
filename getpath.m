function paths = getpath(nodes,v)
    paths = [];
    for i = 1:1:length(nodes)-1
        x = linspace(nodes(i,1),nodes(i+1,1),500);
        y = linspace(nodes(i,2),nodes(i+1,2),500);
        theta = atan2(nodes(i+1,2)-nodes(i,2),nodes(i+1,1)-nodes(i,1));
        subvx = v(i)*cos(theta)*ones(1,length(x));
        subvy = v(i)*sin(theta)*ones(1,length(y));
        path = [x;y;subvx;subvy];
        paths = [paths,path];
    end
end