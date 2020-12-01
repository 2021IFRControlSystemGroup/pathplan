function [ ]=pathplan_inside(current_position)
destination=[5500 1750];      %Ŀ�������3090,4420,5750,7080,8410
obstacle_inside_x=[4000 6000 6000 6000];%
obstacle_inside_y=[6000 3500 6000 8500];%
%current_position=[5600 ji];
size_obstacle=size(obstacle_inside_x);        %�ϰ������
num_obstacle=size_obstacle(1,2);
count=0;
x=[];y=[];
robot_width=420;        %�����һ��
radius=150+20;

plot(destination(1),destination(2),'v',current_position(1),current_position(2),'ms');
line([destination(1) current_position(1)],[destination(2) current_position(2)]);

beta(1)=atan((current_position(2)-destination(2))/(current_position(1)-destination(1)));
robot_dis_obs=abs((robot_width+radius)/cos(beta(1)));
k=(current_position(2)-destination(2))/(current_position(1)-destination(1));
b=k*(-current_position(1))+current_position(2);
flag=0;

map_cspace();
hold on;

%�ж�ֱ��·����Χ���Ƿ����ϰ������м�¼�ϰ��������
if num_obstacle==0
    x=[current_position(1),destination(1)];
    y=[current_position(2),destination(2)];
    
    plot(x,y,'r')
    
else
for i=1:num_obstacle
    dis_line=dot_line(k,b,[obstacle_inside_x(i),obstacle_inside_y(i)]);
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
        %�����ϰ����x��y���ƫ����
        x_offset=(robot_width+radius);
        y_offset=(robot_width+radius);
        %��ת����ϵ�µ�ƫ����
        acos=x_offset*cosalph;asin=x_offset*sinalph;
        bcos=y_offset*cosalph;bsin=y_offset*sinalph;

        if count==2 %һ���ϰ���
            %���ϰ��������������Ϸ�ʱ
            if (in_obstacle(2)>(k*in_obstacle(1)+b))||(k==inf&&destination(1)>obstacle_inside_x(i))
                %�Ƚ���ѡȡ�ĵ�����
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
                %������������������Ƿ�������ϰ���
    %             for i=1:num_obstacle
    %                  if (obstacle_inside_x(i)~=in_obstacle(1))&&(obstacle_inside_x(i)<max(vertices(1,1),vertices(1,2))&&obstacle_inside_x(i)>min(vertices(1,1),vertices(1,2)))&&(obstacle_inside_y(i)<((vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*obstacle_inside_x(i)+vertices(2,2)-(vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*vertices(1,2)+robot_dis_obs))&&(obstacle_inside_y(i)>((vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*obstacle_inside_x(i)+vertices(2,2)-(vertices(2,2)-vertices(2,1))/(vertices(1,2)-vertices(1,1))*vertices(1,2)-robot_dis_obs))||(obstacle_inside_x(i)~=in_obstacle(1))&&(obstacle_inside_x(i)<max(vertices(1,3),vertices(1,4))&&obstacle_inside_x(i)>min(vertices(1,3),vertices(1,4)))&&(obstacle_inside_y(i)<((vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*obstacle_inside_x(i)+vertices(2,4)-(vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*vertices(1,4)+robot_dis_obs))&&(obstacle_inside_y(i)>((vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*obstacle_inside_x(i)+vertices(2,4)-(vertices(2,4)-vertices(2,3))/(vertices(1,4)-vertices(1,3))*vertices(1,4)-robot_dis_obs))       
    %                     in_obstacle(count+1)=obstacle_inside_x(i);
    %                     in_obstacle(count+2)=obstacle_inside_y(i);
    %                     count=count+2;
    %                     break;
    %                 end
    %             end
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

        if count==4%�����ϰ���
            %����������ֱ���Ϸ�ʱ
            if (in_obstacle(2)>(k*in_obstacle(1)+b)||in_obstacle(2)==(k*in_obstacle(1)+b))&&(in_obstacle(4)>(k*in_obstacle(3)+b)||in_obstacle(4)==(k*in_obstacle(3)+b))
                temp_position1=[in_obstacle(1)+acos+bsin,in_obstacle(2)+asin-bcos];
                temp_position2=[in_obstacle(1)-acos+bsin,in_obstacle(2)-asin-bcos];
                temp_position3=[in_obstacle(3)+acos+bsin,in_obstacle(4)+asin-bcos];
                temp_position4=[in_obstacle(3)-acos+bsin,in_obstacle(4)-asin-bcos];
                flag=1;
                vertices  = B_sort( current_position,temp_position1,temp_position2,temp_position3,temp_position4,destination,in_obstacle,flag);

                %���Ҳ����ֱ���·���������ֱ���Ϸ�
            elseif in_obstacle(2)<(k*in_obstacle(1)+b)&&in_obstacle(4)>(k*in_obstacle(3)+b)
                temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                temp_position3=[in_obstacle(3)+acos+bsin,in_obstacle(4)+asin-bcos];
                temp_position4=[in_obstacle(3)-acos+bsin,in_obstacle(4)-asin-bcos]; 
                vertices  = B_sort( current_position,temp_position1,temp_position2,temp_position3,temp_position4,destination,in_obstacle,flag);

    %             %���Ҳ����ֱ���Ϸ���������ֱ���·�
    %         elseif in_obstacle(2)>(k*in_obstacle(1)+b)&&in_obstacle(4)<(k*in_obstacle(3)+b)
    %             temp_position1=[in_obstacle(1)+acos+bsin,in_obstacle(2)+asin-bcos];
    %             temp_position2=[in_obstacle(1)-acos+bsin,in_obstacle(2)-asin-bcos];
    %             temp_position3=[in_obstacle(3)+acos-bsin,in_obstacle(4)+asin+bcos];
    %             temp_position4=[in_obstacle(3)-acos-bsin,in_obstacle(4)-asin+bcos];
    %             vertices  = B_sort( current_position,temp_position1,temp_position2,temp_position3,temp_position4,destination,in_obstacle,flag);

             else%�����㶼��ֱ���·�ʱ
                temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
                temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
                temp_position3=[in_obstacle(3)+acos-bsin,in_obstacle(4)+asin+bcos];
                temp_position4=[in_obstacle(3)-acos-bsin,in_obstacle(4)-asin+bcos];
                flag=1;
                vertices  = B_sort( current_position,temp_position1,temp_position2,temp_position3,temp_position4,destination,in_obstacle,flag);
            end
       else

        end
       if count==6
       %����������ֱ���·�ʱ
       % if (in_obstacle(2)>(k*in_obstacle(1)+b)||in_obstacle(2)==(k*in_obstacle(1)+b))&&(in_obstacle(4)>(k*in_obstacle(3)+b)||in_obstacle(4)==(k*in_obstacle(3)+b))&&(in_obstacle(6)>(k*in_obstacle(5)+b)||in_obstacle(6)==(k*in_obstacle(5)+b))
            temp_position1=[in_obstacle(1)+acos-bsin,in_obstacle(2)+asin+bcos];
            temp_position2=[in_obstacle(1)-acos-bsin,in_obstacle(2)-asin+bcos];
            temp_position3=[in_obstacle(3)+acos-bsin,in_obstacle(4)+asin+bcos];
            temp_position4=[in_obstacle(3)-acos-bsin,in_obstacle(4)-asin+bcos];
            temp_position5=[in_obstacle(5)+acos-bsin,in_obstacle(6)+asin+bcos];
            temp_position6=[in_obstacle(5)-acos-bsin,in_obstacle(6)-asin+bcos];
%            flag=1;
            vertices  = [current_position;temp_position5;temp_position6;temp_position3;temp_position4;temp_position1;temp_position2;destination]';
       % end
            if(current_position(2)<in_obstacle(end))
                vertices(:,2)=[];
            end
       end
       if(current_position(2)<in_obstacle(end))
           vertices(:,(end-2))=[];
       end
       vertices(2,2)=vertices(2,2)-1.5*radius;
       plot(vertices(1,:),vertices(2,:),'r')
    end
end
pause(0.1);
hold off;
end