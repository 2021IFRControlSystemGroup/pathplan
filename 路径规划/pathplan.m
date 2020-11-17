for robot_y=1000:50:10000
    destination=[11000 5000]; 
%     h=300;
%     obstacle_point_x=[2000 1700 3800 6000 8200 10300 10000];%从左到右排序列出
%     obstacle_point_y=[6000 4200 2000 2000 2000 3800 6000];%
%     obstacle_point=[obstacle_point_x;obstacle_point_y];
%     size_obstacle_point=size(obstacle_point_x);        %障碍物个数
%     num_obstacle_point=size_obstacle_point(1,2);
%     num_obstacle_point=int8(num_obstacle_point);
    current_position_x=5700;
    current_position_y=robot_y;
    current_position=[current_position_x current_position_y];
%     vertices=[current_position]';
    k=(current_position(2)-destination(2))/(current_position(1)-destination(1));
    b=k*(-current_position(1))+current_position(2);
%     plot(destination(1),destination(2),'v',current_position(1),current_position(2),'ms');
%     line([destination(1) current_position(1)],[destination(2) current_position(2)]);
    map_cspace();
    hold on;
    if((current_position_y>-current_position_x+6000) && (current_position_y<current_position_x+6000) && ((2000<current_position_y) && (current_position_y<9950)) && ((2000<current_position_x) && (current_position_x<5975)))
        pathplan_inside(current_position);
    else
        pathplan_outside(current_position,destination);
%         for i=1:num_obstacle_point-1
%             flag=line_intersect(current_position,destination,obstacle_point(:,i)',obstacle_point(:,i+1)');
%             if flag==1
%                 current_position=obstacle_point(:,i+1)'-[0,500];
%                 vertices=[vertices,current_position'];
%             end
%         end
%         vertices=[vertices,destination'];
%         plot(vertices(1,:),vertices(2,:),'r')
%         pause(0.1);
    end
%     vertices=[vertices,destination'];
%     plot(vertices(1,:),vertices(2,:),'r')
%     pause(0.1);
     hold off;
end
