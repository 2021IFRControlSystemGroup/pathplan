tic;
position_start=[12000 1800];
destination=[11500 5800]; 
%position_now=position_start;
position_now=[10000 3500];
%destination=[11000 5000]; 
obstacle_outside_x=[2000 2000 4000 6000 8000 10000 10000];%从左到右排序列出
obstacle_outside_y=[6000 4000 2000 2000 2000 4000 6000];%
obstacle_point=[obstacle_outside_x;obstacle_outside_y];
size_obstacle_point=size(obstacle_outside_x);        %障碍物个数
num_obstacle_point=size_obstacle_point(1,2);
num_obstacle_point=int8(num_obstacle_point);
% current_position=[11900 5000];
vertices=[position_start]';
k=(position_start(2)-destination(2))/(position_start(1)-destination(1));
b=k*(-position_start(1))+position_start(2);
if((position_start(2)>=-position_start(1)+6000) && (position_start(2)<=position_start(1)+6000) && ((2000<=position_start(2)) && (position_start(2)<=9950)) && ((2000<=position_start(1)) && (position_start(1)<=5975)))
    destination=[5500 1750];      
    obstacle_inside_x=[4000 6000 6000 6000];%
    obstacle_inside_y=[6000 3500 6000 8500];%
    size_obstacle=size(obstacle_inside_x);        %障碍物个数
    num_obstacle=size_obstacle(1,2);
    count=0;
    x=[];y=[];
    robot_width=420;        %车宽的一半
    radius=150+20;
    beta(1)=atan((position_start(2)-destination(2))/(position_start(1)-destination(1)));
    robot_dis_obs=abs((robot_width+radius)/cos(beta(1)));
    k=(position_start(2)-destination(2))/(position_start(1)-destination(1));
    b=k*(-position_start(1))+position_start(2);
    flag=0;
    %判断直行路径范围内是否有障碍物，如果有记录障碍物点做标
    if num_obstacle==0
        x=[position_start(1),destination(1)];
        y=[position_start(2),destination(2)];
    else
    for i=1:num_obstacle
        line_k=k;line_b=b;array_o=[obstacle_inside_x(i),obstacle_inside_y(i)];
        if line_k==inf||line_k==-inf
            dis_dot_line=abs(array_o(2)-line_b);
        else
            molecule=(line_k*array_o(1)-array_o(2)+line_b);
            dis_dot_line=abs(molecule/sqrt(power(line_k,2)+1));
        end
%         dis_line=dot_line(k,b,[obstacle_inside_x(i),obstacle_inside_y(i)]);
        dis_line=dis_dot_line;
        if (dis_line<robot_width+radius || dis_line==robot_width+radius)&&(obstacle_inside_x(i)<max(destination(1),position_start(1))+robot_width+radius&&obstacle_inside_x(i)>min(destination(1),position_start(1))-robot_width+radius&&obstacle_inside_y(i)<max(destination(2),position_start(2))+robot_width+radius&&obstacle_inside_y(i)>min(destination(2),position_start(2))-robot_width+radius)
            in_obstacle(count+1)=obstacle_inside_x(i);
            in_obstacle(count+2)=obstacle_inside_y(i);
            count=count+2;
        end
    end
    cosalph=(position_start(1)-destination(1))/norm(destination-position_start);
    sinalph=(position_start(2)-destination(2))/norm(destination-position_start);
    if position_start(1)<destination(1)
        cosalph=-cosalph;
        sinalph=-sinalph;
    end
        if (count==0)       %(k==inf||k==-inf)||
            x=[position_start(1),destination(1)];
            y=[position_start(2),destination(2)];
        else
            %关于障碍物点x、y轴的偏移量
            x_offset=(robot_width+radius);
            y_offset=(robot_width+radius);
            %旋转坐标系下的偏移量
            acos=x_offset*cosalph;asin=x_offset*sinalph;
            bcos=y_offset*cosalph;bsin=y_offset*sinalph;

            if count==2 %一个障碍物
                %当障碍物在两点连线上方时
                if (in_obstacle(2)>(k*in_obstacle(1)+b))||(k==inf&&destination(1)>obstacle_inside_x(i))
                    %先进行选取四点坐标
                    temp_position1=[in_obstacle(1)+acos+bsin,in_obstacle(2)+asin-bcos];
                    temp_position2=[in_obstacle(1)-acos+bsin,in_obstacle(2)-asin-bcos];
                    if position_start(1)>destination(1)
                        if temp_position1(1)>temp_position2(1)
                            vertices=[position_start;temp_position1;temp_position2;destination]';
                        else
                            vertices=[position_start;temp_position2;temp_position1;destination]';
                        end
                    else
                        if temp_position1(1)>temp_position2(1)
                            vertices=[position_start;temp_position2;temp_position1;destination]';
                        else
                            vertices=[position_start;temp_position1;temp_position2;destination]';
                        end
                    end
                else
                    temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                    temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                    if position_start(1)>destination(1)
                        if temp_position1(1)>temp_position2(1)
                            vertices=[position_start;temp_position1;temp_position2;destination]';
                        else
                            vertices=[position_start;temp_position2;temp_position1;destination]';
                        end
                    else
                        if temp_position1(1)>temp_position2(1)
                            vertices=[position_start;temp_position2;temp_position1;destination]';
                        else
                            vertices=[position_start;temp_position1;temp_position2;destination]';
                        end
                    end
                    for i=1:num_obstacle
                         if (obstacle_inside_x(i)~=in_obstacle(1))&&(obstacle_inside_x(i)<max(vertices(1,1),vertices(1,2))&&obstacle_inside_x(i)>min(vertices(1,1),vertices(1,2)))&&(obstacle_inside_y(i)<((vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*obstacle_inside_x(i)+vertices(2,2)-(vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*vertices(1,2)+robot_dis_obs))&&(obstacle_inside_y(i)>((vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*obstacle_inside_x(i)+vertices(2,2)-(vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*vertices(1,2)-robot_dis_obs))||(obstacle_inside_x(i)~=in_obstacle(1))&&(obstacle_inside_x(i)<max(vertices(1,3),vertices(1,4))&&obstacle_inside_x(i)>min(vertices(1,3),vertices(1,4)))&&(obstacle_inside_y(i)<((vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*obstacle_inside_x(i)+vertices(2,4)-(vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*vertices(1,4)+robot_dis_obs))&&(obstacle_inside_y(i)>((vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*obstacle_inside_x(i)+vertices(2,4)-(vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*vertices(1,4)-robot_dis_obs))
                            in_obstacle(count+1)=obstacle_inside_x(i);
                            in_obstacle(count+2)=obstacle_inside_y(i);
                            count=count+2;
                            break;
                        end
                    end
                end
            else
                if k*max(obstacle_inside_x)+b<min(obstacle_inside_y)
                    temp_position1=[max(obstacle_inside_x)+x_offset,min(obstacle_inside_y)+y_offset];
                    temp_position2=[max(obstacle_inside_x)-x_offset,min(obstacle_inside_y)+y_offset];
                    if norm(position_start-temp_position1)<norm(position_start-temp_position2)
                        temp1=temp_position1;
                        temp2=temp_position2;
                    else
                        temp1=temp_position2;
                        temp2=temp_position1;
                    end
                    vertices=[position_start;temp1;temp2;destination]';
                else
                    temp_position1=[max(obstacle_inside_x)+x_offset,max(obstacle_inside_y)-y_offset];
                    temp_position2=[max(obstacle_inside_x)-x_offset,max(obstacle_inside_y)-y_offset];
                    if norm(position_start-temp_position1)<norm(position_start-temp_position2)
                        temp1=temp_position1;
                        temp2=temp_position2;
                    else
                        temp1=temp_position2;
                        temp2=temp_position1;
                    end
                    vertices=[position_start;temp1;temp2;destination]';
                end
            end
            if count==4%两个障碍物
                %当两个都在直线上方时
                if (in_obstacle(2)>(k*in_obstacle(1)+b)||in_obstacle(2)==(k*in_obstacle(1)+b))&&(in_obstacle(4)>(k*in_obstacle(3)+b)||in_obstacle(4)==(k*in_obstacle(3)+b))
                    temp_position1=[in_obstacle(1)+acos+bsin,in_obstacle(2)+asin-bcos];
                    temp_position2=[in_obstacle(1)-acos+bsin,in_obstacle(2)-asin-bcos];
                    temp_position3=[in_obstacle(3)+acos+bsin,in_obstacle(4)+asin-bcos];
                    temp_position4=[in_obstacle(3)-acos+bsin,in_obstacle(4)-asin-bcos];
                    flag=1;
                    if norm(position_start-[in_obstacle(1),in_obstacle(2)])<norm(position_start-[in_obstacle(3),in_obstacle(4)])
                        if norm(position_start-temp_position1)<norm(position_start-temp_position2)
                            temp1=temp_position1;
                            temp2=temp_position2;
                        else
                            temp1=temp_position2;
                            temp2=temp_position1;
                        end
                        if norm(destination-temp_position3)<norm(destination-temp_position4)
                            temp3=temp_position4;
                            temp4=temp_position3;
                        else
                            temp3=temp_position3;
                            temp4=temp_position4;
                        end
                    else
                        if norm(position_start-temp_position3)<norm(position_start-temp_position4)
                            temp1=temp_position3;
                            temp2=temp_position4;
                        else
                            temp1=temp_position4;
                            temp2=temp_position3;
                        end
                        if norm(destination-temp_position1)<norm(destination-temp_position2)
                            temp3=temp_position2;
                            temp4=temp_position1;
                        else
                            temp3=temp_position1;
                            temp4=temp_position2;
                        end
                    end
                    vertices=[position_start;temp1;temp4;destination ]';
                    %当右侧的在直线下方，左侧的在直线上方
                elseif in_obstacle(2)<(k*in_obstacle(1)+b)&&in_obstacle(4)>(k*in_obstacle(3)+b)
                    temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                    temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                    temp_position3=[in_obstacle(3)+acos+bsin,in_obstacle(4)+asin-bcos];
                    temp_position4=[in_obstacle(3)-acos+bsin,in_obstacle(4)-asin-bcos];
                    if norm(position_start-[in_obstacle(1),in_obstacle(2)])<norm(position_start-[in_obstacle(3),in_obstacle(4)])
                        if norm(position_start-temp_position1)<norm(position_start-temp_position2)
                            temp1=temp_position1;
                            temp2=temp_position2;
                        else
                            temp1=temp_position2;
                            temp2=temp_position1;
                        end
                        if norm(destination-temp_position3)<norm(destination-temp_position4)
                            temp3=temp_position4;
                            temp4=temp_position3;
                        else
                            temp3=temp_position3;
                            temp4=temp_position4;
                        end
                    else
                        if norm(position_start-temp_position3)<norm(position_start-temp_position4)
                            temp1=temp_position3;
                            temp2=temp_position4;
                        else
                            temp1=temp_position4;
                            temp2=temp_position3;
                        end
                        if norm(destination-temp_position1)<norm(destination-temp_position2)
                            temp3=temp_position2;
                            temp4=temp_position1;
                        else
                            temp3=temp_position1;
                            temp4=temp_position2;
                        end
                    end
                    vertices=[position_start;temp1;temp2;temp3;temp4;destination ]';
                else%当两点都在直线下方时
                    temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                    temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                    temp_position3=[in_obstacle(3)+acos-bsin,in_obstacle(4)+asin+bcos];
                    temp_position4=[in_obstacle(3)-acos-bsin,in_obstacle(4)-asin+bcos];
                    if norm(position_start-[in_obstacle(1),in_obstacle(2)])<norm(position_start-[in_obstacle(3),in_obstacle(4)])
                        if norm(position_start-temp_position1)<norm(position_start-temp_position2)
                            temp1=temp_position1;
                            temp2=temp_position2;
                        else
                            temp1=temp_position2;
                            temp2=temp_position1;
                        end
                        if norm(destination-temp_position3)<norm(destination-temp_position4)
                            temp3=temp_position4;
                            temp4=temp_position3;
                        else
                            temp3=temp_position3;
                            temp4=temp_position4;
                        end
                    else
                        if norm(position_start-temp_position3)<norm(position_start-temp_position4)
                            temp1=temp_position3;
                            temp2=temp_position4;
                        else
                            temp1=temp_position4;
                            temp2=temp_position3;
                        end
                        if norm(destination-temp_position1)<norm(destination-temp_position2)
                            temp3=temp_position2;
                            temp4=temp_position1;
                        else
                            temp3=temp_position1;
                            temp4=temp_position2;
                        end
                    end
                    vertices=[position_start;temp1;temp4;destination ]';
                end
            end
            if count==6
            %当三个都在直线下方时
                temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                temp_position3=[in_obstacle(3)+acos-bsin,in_obstacle(4)+asin+bcos];
                temp_position4=[in_obstacle(3)-acos-bsin,in_obstacle(4)-asin+bcos];
                temp_position5=[in_obstacle(5)+acos-bsin,in_obstacle(6)+asin+bcos];
                temp_position6=[in_obstacle(5)-acos-bsin,in_obstacle(6)-asin+bcos];
                vertices  = [position_start;temp_position5;temp_position6;temp_position3;temp_position4;temp_position1;temp_position2;destination]';
                if(position_start(2)<in_obstacle(end))
                    vertices(:,2)=[];
                end
            end
            if(position_start(2)<in_obstacle(end))                   
                vertices(:,(end-2))=[];
            end
            vertices(2,2)=vertices(2,2)-1.5*radius;
        end
    end
else %在外部运行的情况
    flag_change1=0;
    flag_change2=0;
    current_position_add=position_start;
    destination_add=destination;
    for i=1:num_obstacle_point-1%判断是否相交        
        A=current_position_add;B=destination_add;C=obstacle_point(:,i)';D=obstacle_point(:,i+1)';
        flag_intersect=0;
        AB=B-A;CD=D-C;
        k1=AB(2)/AB(1);
        k2=CD(2)/CD(1);
        if(k1==inf || k1==-inf)
            x_intersect=B(1);
            b2=C(2)-k2*C(1);
            y_intersect=k2*x_intersect+b2;
        elseif(k2==inf || k2==-inf)
            x_intersect=D(1);
            b1=A(2)-k1*A(1);
            y_intersect=k1*x_intersect+b1;
        else
            b1=A(2)-k1*A(1);
            b2=C(2)-k2*C(1);
            x_intersect=(b2-b1)/(k1-k2);
            y_intersect=k1*x_intersect+b1;
        end
        
        if((min(A(1),B(1))<=x_intersect) && (x_intersect<=max(A(1),B(1))) && (min(C(1),D(1))<=x_intersect) && (x_intersect<=max(C(1),D(1))) && (min(A(2),B(2))<=y_intersect) && (y_intersect<=max(A(2),B(2))) && (min(C(2),D(2))<=y_intersect) && (y_intersect<=max(C(2),D(2))))
            flag_intersect=1;
        end 
        
        if flag_intersect==1
            if(current_position_add(1)<destination_add(1))
                current_position_add=obstacle_point(:,i+1)'-[0,500];
                vertices=[vertices,current_position_add'];
                flag_change1=1;
            else
                destination_add=obstacle_point(:,i+1)'-[0,500];
                vertices=[vertices,destination_add'];
                flag_change2=1;
                if(i==num_obstacle_point-2)
                    vertices(:,1)=[];
                    vertices=[vertices,current_position_add'];         
                end
            end
        else
            %vertices=[vertices,destination'];
        end
    end
    if(flag_change1==1)
        vertices=[vertices,destination'];
    elseif (flag_change2==1)
        vertices(:,1)=[];
        vertices=[vertices,position_start'];
    else 
        vertices=[vertices,destination'];
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%发送角度
%规定逆时针为正,顺时针为负,角度为负右转,角度为正左转
%初始角度
angle_all=[];
line_target=destination-position_start;
line_x=[line_target(1) 0];
b=vertices(:,1)';
c=vertices(:,2)';
ab=line_x;
bc=c-b;
if(ab(1)*bc(2)-ab(2)*bc(1)>=0)%bc在ab的逆时针方向
    angle_flag=1;
else
    angle_flag=-1;
end
angle_flag_test=angle_flag*(acos(dot(ab,bc)/(norm(ab)*norm(bc))))*180/pi;
angle_all=[angle_all,angle_flag_test];
%障碍点之间的角度
for i=1:length(vertices)-2
    a=vertices(:,i)';
    b=vertices(:,i+1)';
    c=vertices(:,i+2)';
    ab=b-a;
    bc=c-b;
    if(ab(1)*bc(2)-ab(2)*bc(1)>=0)%bc在ab的逆时针方向
        angle_flag=1;
    else
        angle_flag=-1;
    end
    angle_flag_test=angle_flag*(acos(dot(ab,bc)/(norm(ab)*norm(bc))))*180/pi;
    angle_all=[angle_all,angle_flag_test];
end
%角度发送
angle_send_flag=1;
if(position_start==vertices(:,angle_send_flag)')
    angle_send=angle_all(angle_send_flag);
    angle_send_flag=angle_send_flag+1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%发送速度
start_position=position_start;
speed_max=15000;
distance_all=0;distance_now=0;
for i=1:length(vertices)-1
    distance_all=distance_all+norm(vertices(:,i+1)-vertices(:,i));
end

vertices_now=[];
for i=1:length(vertices)-1
    l1=vertices(:,i)'-position_now;
    l2=vertices(:,i+1)'-position_now;
    angle_in=(acos(dot(l1,l2)/(norm(l1)*norm(l2))));%判断点在哪两个点之间,即判断当前位置与周围两个点的夹角
    vertices_now=[vertices_now,vertices(:,i)];
    if(angle_in==0 || isnan(angle_in))
        vertices_now=[vertices_now,position_now'];
        break;
    end
%    vertices_now=[vertices_now,vertices(:,i)];
end

for i=1:size(vertices_now,2)-1
    distance_now=distance_now+norm(vertices_now(:,i+1)-vertices_now(:,i));
end

k_speed=0.2;
if(distance_now<=k_speed*distance_all)%加速时的速度变化
    speed_send=distance_now*speed_max/(k_speed*distance_all);
elseif(distance_now>=(1-k_speed)*distance_all)%减速时的速度变化
    speed_send=speed_max-distance_now*speed_max/distance_all;
else%其余情况
    speed_send=speed_max;
end


t=toc;


map_cspace();
hold on;
%pause(0.05);
plot(vertices(1,:),vertices(2,:),'r');
hold off;
% (k2+1).*(k2<0)+(-k2+1).*(k2>=0)
