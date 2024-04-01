function pnts = get_points(point)
%     point = [1,2,3;
%         4,5,6;
%         7,8,9;
%         10,11,12;
%         13,14,15];
    % Breaking the input points to 2 lists
    l1 = point(1:end-1,:);
    l2 = point(2:end,:);
    val = size(l1);

    % Number of points needed between two points
    n = 20;
    cnt = 1;
    for i = 1:val(1)
        point1 = l1(i,:);
        point2 = l2(i,:);
        
        % Calculate the unit vector
        direction(i,:) = point2 - point1;
        distance(i) = norm(direction(i,:));
        spacing(i) = distance(i)/(n+1);
        unit_vector(i,:) = (direction(i,:)/distance(i))*spacing(i);

        % Calculate points between two points
        pnts(cnt,:) = point(i,:);
        cnt = cnt+1;
        for j = 1:n
            pnts(cnt,:) = point1 +j*unit_vector(i,:);
            cnt = cnt+1;
        end
    end
    pnts(end+1,:) = point(end,:);
end
