t=0:0.1:1;
x=[];y=[];          %拟合后曲线x、y坐标


destination=[5700 2300]; 
%destination=[12000 5000]; 
obstacle_outside_x=[2000 1700 3800 6000 8200 10300 10000];%从左到右排序列出
obstacle_outside_y=[6000 4200 2000 2000 2000 3800 6000];%
obstacle_point=[obstacle_outside_x;obstacle_outside_y];
size_obstacle_point=size(obstacle_outside_x);        %障碍物个数
num_obstacle_point=size_obstacle_point(1,2);
num_obstacle_point=int8(num_obstacle_point);
current_position=[5550 5000];
%current_position=[5000 1000];
vertices=[current_position]';
k=(current_position(2)-destination(2))/(current_position(1)-destination(1));
b=k*(-current_position(1))+current_position(2);
if((current_position(2)>=-current_position(1)+6000) && (current_position(2)<=current_position(1)+6000) && ((2000<=current_position(2)) && (current_position(2)<=9950)) && ((2000<=current_position(1)) && (current_position(1)<=5975)))
    destination=[5500 1750];      %目标点坐标3090,4420,5750,7080,8410
    obstacle_inside_x=[4000 6000 6000 6000];%
    obstacle_inside_y=[6000 3500 6000 8500];%
    size_obstacle=size(obstacle_inside_x);        %障碍物个数
    num_obstacle=size_obstacle(1,2);
    count=0;
    x=[];y=[];
    robot_width=420;        %车宽的一半
    radius=150+20;
    beta(1)=atan((current_position(2)-destination(2))/(current_position(1)-destination(1)));
    robot_dis_obs=abs((robot_width+radius)/cos(beta(1)));
    k=(current_position(2)-destination(2))/(current_position(1)-destination(1));
    b=k*(-current_position(1))+current_position(2);
    flag=0;
    %判断直行路径范围内是否有障碍物，如果有记录障碍物点做标
    if num_obstacle==0
        x=[current_position(1),destination(1)];
        y=[current_position(2),destination(2)];
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
        if (dis_line<robot_width+radius || dis_line==robot_width+radius)&&(obstacle_inside_x(i)<max(destination(1),current_position(1))+robot_width+radius&&obstacle_inside_x(i)>min(destination(1),current_position(1))-robot_width+radius&&obstacle_inside_y(i)<max(destination(2),current_position(2))+robot_width+radius&&obstacle_inside_y(i)>min(destination(2),current_position(2))-robot_width+radius)
            in_obstacle(count+1)=obstacle_inside_x(i);
            in_obstacle(count+2)=obstacle_inside_y(i);
            count=count+2;
        end
    end
    cosalph=(current_position(1)-destination(1))/norm(destination-current_position);
    sinalph=(current_position(2)-destination(2))/norm(destination-current_position);
    if current_position(1)<destination(1)
        cosalph=-cosalph;
        sinalph=-sinalph;
    end
        if (count==0)       %(k==inf||k==-inf)||
            x=[current_position(1),destination(1)];
            y=[current_position(2),destination(2)];
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
                    if current_position(1)>destination(1)
                        if temp_position1(1)>temp_position2(1)
                            vertices=[current_position;temp_position1;temp_position2;destination]';
                        else
                            vertices=[current_position;temp_position2;temp_position1;destination]';
                        end
                    else
                        if temp_position1(1)>temp_position2(1)
                            vertices=[current_position;temp_position2;temp_position1;destination]';
                        else
                            vertices=[current_position;temp_position1;temp_position2;destination]';
                        end
                    end
                else
                    temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                    temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                    if current_position(1)>destination(1)
                        if temp_position1(1)>temp_position2(1)
                            vertices=[current_position;temp_position1;temp_position2;destination]';
                        else
                            vertices=[current_position;temp_position2;temp_position1;destination]';
                        end
                    else
                        if temp_position1(1)>temp_position2(1)
                            vertices=[current_position;temp_position2;temp_position1;destination]';
                        else
                            vertices=[current_position;temp_position1;temp_position2;destination]';
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
                    if norm(current_position-temp_position1)<norm(current_position-temp_position2)
                        temp1=temp_position1;
                        temp2=temp_position2;
                    else
                        temp1=temp_position2;
                        temp2=temp_position1;
                    end
                    vertices=[current_position;temp1;temp2;destination]';
                else
                    temp_position1=[max(obstacle_inside_x)+x_offset,max(obstacle_inside_y)-y_offset];
                    temp_position2=[max(obstacle_inside_x)-x_offset,max(obstacle_inside_y)-y_offset];
                    if norm(current_position-temp_position1)<norm(current_position-temp_position2)
                        temp1=temp_position1;
                        temp2=temp_position2;
                    else
                        temp1=temp_position2;
                        temp2=temp_position1;
                    end
                    vertices=[current_position;temp1;temp2;destination]';
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
                    if norm(current_position-[in_obstacle(1),in_obstacle(2)])<norm(current_position-[in_obstacle(3),in_obstacle(4)])
                        if norm(current_position-temp_position1)<norm(current_position-temp_position2)
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
                        if norm(current_position-temp_position3)<norm(current_position-temp_position4)
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
                    vertices=[current_position;temp1;temp4;destination ]';
                    %当右侧的在直线下方，左侧的在直线上方
                elseif in_obstacle(2)<(k*in_obstacle(1)+b)&&in_obstacle(4)>(k*in_obstacle(3)+b)
                    temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                    temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                    temp_position3=[in_obstacle(3)+acos+bsin,in_obstacle(4)+asin-bcos];
                    temp_position4=[in_obstacle(3)-acos+bsin,in_obstacle(4)-asin-bcos];
                    if norm(current_position-[in_obstacle(1),in_obstacle(2)])<norm(current_position-[in_obstacle(3),in_obstacle(4)])
                        if norm(current_position-temp_position1)<norm(current_position-temp_position2)
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
                        if norm(current_position-temp_position3)<norm(current_position-temp_position4)
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
                    vertices=[current_position;temp1;temp2;temp3;temp4;destination ]';
                else%当两点都在直线下方时
                    temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                    temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                    temp_position3=[in_obstacle(3)+acos-bsin,in_obstacle(4)+asin+bcos];
                    temp_position4=[in_obstacle(3)-acos-bsin,in_obstacle(4)-asin+bcos];
                    if norm(current_position-[in_obstacle(1),in_obstacle(2)])<norm(current_position-[in_obstacle(3),in_obstacle(4)])
                        if norm(current_position-temp_position1)<norm(current_position-temp_position2)
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
                        if norm(current_position-temp_position3)<norm(current_position-temp_position4)
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
                    vertices=[current_position;temp1;temp4;destination ]';
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
                vertices  = [current_position;temp_position5;temp_position6;temp_position3;temp_position4;temp_position1;temp_position2;destination]';
                if(current_position(2)<in_obstacle(end))
                    vertices(:,2)=[];
                end
            end
            if(current_position(2)<in_obstacle(end))                   
                vertices(:,(end-2))=[];
            end
            vertices(2,2)=vertices(2,2)-1.5*radius;
        end
    end
else %在外部运行的情况
    flag_change1=0;
    flag_change2=0;
    for i=1:num_obstacle_point-1
        A=current_position;B=destination;C=obstacle_point(:,i)';D=obstacle_point(:,i+1)';
        flag_intersect=0;
        AB=B-A;
        AC=C-A;
        AD=D-A;
        AB_cross_AC=AB(1)*AC(2)-AC(1)*AB(2);
        AB_cross_AC=AB_cross_AC/abs(AB_cross_AC);
        AB_cross_AD=AB(1)*AD(2)-AD(1)*AB(2);
        AB_cross_AD=AB_cross_AD/abs(AB_cross_AD);
        if (AB_cross_AC*AB_cross_AD<=0)
            flag_intersect=1;
        end         
        if flag_intersect==1
            if(current_position(1)<destination(1))
                current_position=obstacle_point(:,i+1)'-[0,500];
                vertices=[vertices,current_position'];
                flag_change1=1;
            else
                destination=obstacle_point(:,i+1)'-[0,500];
                vertices=[vertices,destination'];
                flag_change2=1;
                if(i==num_obstacle_point-2)
                    vertices(:,1)=[];
                    vertices=[vertices,current_position'];         
                end
            end
        end
    end
    if(flag_change1==1)
        vertices=[vertices,destination'];
    end
    if(flag_change2==1)
        vertices(:,1)=[];
        vertices=[vertices,current_position'];
    end
end

% x=vertices(1,:);
% y=vertices(2,:);

%贝塞尔曲线拟合
NumPoint=size(vertices,2)-1;
x=(1-t).^(NumPoint)*vertices(1,1);
y=(1-t).^(NumPoint)*vertices(2,1);
for j=1:NumPoint
   w=factorial(NumPoint)/(factorial(j)*factorial(NumPoint-j))*(1-t).^(NumPoint-j).*t.^(j);
   x=x+w*vertices(1,j+1);
   y=y+w*vertices(2,j+1);
end

map_cspace();
hold on;
%pause(0.05);
plot(x,y,'r');
hold off;
