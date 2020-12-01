function [ ]=pathplan_outside(current_position,destination)
    obstacle_point_x=[2000 1700 3800 6000 8200 10300 10000];%从左到右排序列出
    obstacle_point_y=[6000 4200 2000 2000 2000 3800 6000];%
    obstacle_point=[obstacle_point_x;obstacle_point_y];
    size_obstacle_point=size(obstacle_point_x);        %障碍物个数
    num_obstacle_point=size_obstacle_point(1,2);
    num_obstacle_point=int8(num_obstacle_point);
    vertices=[current_position]';
    plot(destination(1),destination(2),'v',current_position(1),current_position(2),'ms');
    line([destination(1) current_position(1)],[destination(2) current_position(2)]);
    for i=1:num_obstacle_point-1
        flag=line_intersect(current_position,destination,obstacle_point(:,i)',obstacle_point(:,i+1)');
        if flag==1
            current_position=obstacle_point(:,i+1)'-[0,500];
            vertices=[vertices,current_position'];
        end
    end
    vertices=[vertices,destination'];
    plot(vertices(1,:),vertices(2,:),'r')
    pause(0.1);
    hold off;
end