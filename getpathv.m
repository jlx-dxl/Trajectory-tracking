function path = getpathv(nodes,v)
    paths = [];
    l = length(nodes);
    for i = 1:1:ceil(l/2)
        x = linspace(nodes(i,1),nodes(i+1,1),50);
        y = linspace(nodes(i,2),nodes(i+1,2),50);
        theta = atan2(nodes(i+1,2)-nodes(i,2),nodes(i+1,1)-nodes(i,1));
        subvx = v(1)*cos(theta)*ones(1,length(x));
        subvy = v(1)*sin(theta)*ones(1,length(y));
        path = [x;y;subvx;subvy];
        paths = [paths,path];
    end
    for i = ceil(l/2):1:l-1
        x = linspace(nodes(i,1),nodes(i+1,1),50);
        y = linspace(nodes(i,2),nodes(i+1,2),50);
        theta = atan2(nodes(i+1,2)-nodes(i,2),nodes(i+1,1)-nodes(i,1));
        subvx = v(2)*cos(theta)*ones(1,length(x));
        subvy = v(2)*sin(theta)*ones(1,length(y));
        path = [x;y;subvx;subvy];
        paths = [paths,path];
    end
end