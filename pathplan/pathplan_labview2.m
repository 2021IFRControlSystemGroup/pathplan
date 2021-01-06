tic;
position_now=[10000 2000]; 

% destination=[5700 5000];
% position_start=[2701 7500];

position_start=[2800 7200];
destination=[8000 900];

if (destination(1)<position_start(1))%���յ�������ұߣ����������λ��
    position_temp=position_start;
    position_start=destination;
    destination=position_temp;
end

obstacle_outside_x=[2000 2000 4000 8000 10000 10000];%�����������г�
obstacle_outside_y=[6000 4000 2000 2000 4000 6000];%
obstacle_point=[obstacle_outside_x;obstacle_outside_y];
size_obstacle_point=size(obstacle_outside_x);        %�ϰ������
num_obstacle_point=size_obstacle_point(1,2);
% num_obstacle_point=int8(num_obstacle_point);
vertices=[position_start]';
vertices_add=[];
k=(position_start(2)-destination(2))/(position_start(1)-destination(1));
b=k*(-position_start(1))+position_start(2);

flag_inside=0;flag_outside=0;flag_start_inside_dif=0;flag_start_outside_dif=0;
if (((position_start(2)>=-position_start(1)+6000) && (position_start(2)<=position_start(1)+6000) && ((2000<=position_start(2)) && (position_start(2)<=9950)) && ((2000<=position_start(1)) && (position_start(1)<=5975))) || ((destination(2)>=-destination(1)+6000) && (destination(2)<=destination(1)+6000) && ((2000<=destination(2)) && (destination(2)<=9950)) && ((2000<=destination(1)) && (destination(1)<=5975))))%��һ�����ڲ������
    flag_inside=1;
else
    flag_outside=1;
end

if ((((position_start(2)>=-position_start(1)+6000) && (position_start(2)<=position_start(1)+6000) && ((2000<=position_start(2)) && (position_start(2)<=9950)) && ((2000<=position_start(1)) && (position_start(1)<=5975))) && ~((destination(2)>=-destination(1)+6000) && (destination(2)<=destination(1)+6000) && ((2000<=destination(2)) && (destination(2)<=9950)) && ((2000<=destination(1)) && (destination(1)<=5975)))) || (~((position_start(2)>=-position_start(1)+6000) && (position_start(2)<=position_start(1)+6000) && ((2000<=position_start(2)) && (position_start(2)<=9950)) && ((2000<=position_start(1)) && (position_start(1)<=5975))) && ((destination(2)>=-destination(1)+6000) && (destination(2)<=destination(1)+6000) && ((2000<=destination(2)) && (destination(2)<=9950)) && ((2000<=destination(1)) && (destination(1)<=5975)))))%�ж�������յ�λ���Ƿ����죬����������յ㲻���ڣ���㲻�����յ�����
    if ((position_start(2)>=-position_start(1)+6000) && (position_start(2)<=position_start(1)+6000) && ((2000<=position_start(2)) && (position_start(2)<=9950)) && ((2000<=position_start(1)) && (position_start(1)<=5975)))%������ڣ��յ�����
        flag_start_inside_dif=1;%�������
    else
        flag_start_outside_dif=1;%�������
        position_temp=position_start;
        position_start=destination;
        destination=position_temp;
    end  
    destination1=[5500 1850];
end

if (flag_inside || flag_start_inside_dif || flag_start_outside_dif)
    if (flag_start_inside_dif || flag_start_outside_dif) %�����������
        destination_temp=destination;
        destination=destination1;
    end      
    obstacle_inside_x=[4000 6000 6000 6000];%
    obstacle_inside_y=[6000 3500 6000 8500];%
    size_obstacle=size(obstacle_inside_x);        %�ϰ������
    num_obstacle=size_obstacle(1,2);
    count=0;
    x=[];y=[];
    robot_width=420;        %�����һ��
    radius=150+100;
    beta(1)=atan((position_start(2)-destination(2))/(position_start(1)-destination(1)));
    robot_dis_obs=abs((robot_width+radius)/cos(beta(1)));
    k=(position_start(2)-destination(2))/(position_start(1)-destination(1));
    b=k*(-position_start(1))+position_start(2);
    flag=0;
    %�ж�ֱ��·����Χ���Ƿ����ϰ������м�¼�ϰ��������
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
            vertices=[vertices,destination'];
        else
            %�����ϰ����x��y���ƫ����
            x_offset=(robot_width+radius);
            y_offset=(robot_width+radius);
            %��ת����ϵ�µ�ƫ����
            a_cos=x_offset*cosalph;a_sin=x_offset*sinalph;
            b_cos=y_offset*cosalph;b_sin=y_offset*sinalph;

            if count==2 %һ���ϰ���
                %���ϰ��������������Ϸ�ʱ
                if (in_obstacle(2)>(k*in_obstacle(1)+b))||(k==inf&&destination(1)>obstacle_inside_x(i))
                    %�Ƚ���ѡȡ�ĵ�����
                    temp_position1=[in_obstacle(1)+a_cos+b_sin,in_obstacle(2)+a_sin-b_cos];
                    temp_position2=[in_obstacle(1)-a_cos+b_sin,in_obstacle(2)-a_sin-b_cos];
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
                    temp_position1=[in_obstacle(1)+a_cos-b_sin,in_obstacle(2)+a_sin+b_cos];
                    temp_position2=[in_obstacle(1)-a_cos-b_sin,in_obstacle(2)-a_sin+b_cos];
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
            if count==4%�����ϰ���
                %����������ֱ���Ϸ�ʱ
                if (in_obstacle(2)>(k*in_obstacle(1)+b)||in_obstacle(2)==(k*in_obstacle(1)+b))&&(in_obstacle(4)>(k*in_obstacle(3)+b)||in_obstacle(4)==(k*in_obstacle(3)+b))
                    temp_position1=[in_obstacle(1)+a_cos+b_sin,in_obstacle(2)+a_sin-b_cos];
                    temp_position2=[in_obstacle(1)-a_cos+b_sin,in_obstacle(2)-a_sin-b_cos];
                    temp_position3=[in_obstacle(3)+a_cos+b_sin,in_obstacle(4)+a_sin-b_cos];
                    temp_position4=[in_obstacle(3)-a_cos+b_sin,in_obstacle(4)-a_sin-b_cos];
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
                    %���Ҳ����ֱ���·���������ֱ���Ϸ�
                elseif in_obstacle(2)<(k*in_obstacle(1)+b)&&in_obstacle(4)>(k*in_obstacle(3)+b)
                    temp_position1=[in_obstacle(1)+a_cos-b_sin,in_obstacle(2)+a_sin+b_cos];
                    temp_position2=[in_obstacle(1)-a_cos-b_sin,in_obstacle(2)-a_sin+b_cos];
                    temp_position3=[in_obstacle(3)+a_cos+b_sin,in_obstacle(4)+a_sin-b_cos];
                    temp_position4=[in_obstacle(3)-a_cos+b_sin,in_obstacle(4)-a_sin-b_cos];
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
                else%�����㶼��ֱ���·�ʱ
                    temp_position1=[in_obstacle(1)+a_cos-b_sin,in_obstacle(2)+a_sin+b_cos];
                    temp_position2=[in_obstacle(1)-a_cos-b_sin,in_obstacle(2)-a_sin+b_cos];
                    temp_position3=[in_obstacle(3)+a_cos-b_sin,in_obstacle(4)+a_sin+b_cos];
                    temp_position4=[in_obstacle(3)-a_cos-b_sin,in_obstacle(4)-a_sin+b_cos];
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
            %����������ֱ���·�ʱ
                temp_position1=[in_obstacle(1)+a_cos-b_sin,in_obstacle(2)+a_sin+b_cos];
                temp_position2=[in_obstacle(1)-a_cos-b_sin,in_obstacle(2)-a_sin+b_cos];
                temp_position3=[in_obstacle(3)+a_cos-b_sin,in_obstacle(4)+a_sin+b_cos];
                temp_position4=[in_obstacle(3)-a_cos-b_sin,in_obstacle(4)-a_sin+b_cos];
                temp_position5=[in_obstacle(5)+a_cos-b_sin,in_obstacle(6)+a_sin+b_cos];
                temp_position6=[in_obstacle(5)-a_cos-b_sin,in_obstacle(6)-a_sin+b_cos];
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
end

if (flag_outside || flag_start_inside_dif || flag_start_outside_dif)%���ⲿ���е����
    
%     flag_change1=0;
%     flag_change2=0;
    
    
    
    if (flag_start_inside_dif || flag_start_outside_dif)
        position_start_add=destination;
        destination=destination_temp;
        position_destination_add=destination_temp;
    else
        position_start_add=position_start;
        position_destination_add=destination;
    end
    
%     if (position_start_add(1)<position_destination_add(1))%������յ��ұ�
%         
%     else%������յ����
%         position_temp_add=position_start_add;
%         position_start_add=position_destination_add;   
%         position_destination_add=position_temp_add;
%     end
    
        if (position_destination_add(1)<position_start_add(1))%���յ��������ߣ����������λ��
            position_temp=position_start_add;
            position_start_add=position_destination_add;
            position_destination_add=position_temp;
        end
        
    for i=1:num_obstacle_point-1%�ж��Ƿ��ཻ  
        flag_intersect=0;     
        A=position_start_add;B=position_destination_add;C=obstacle_point(:,i)';D=obstacle_point(:,i+1)';
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
        
%         if((min(A(1),B(1))<=x_intersect) && (x_intersect<=max(A(1),B(1))) && (min(C(1),D(1))<=x_intersect) && (x_intersect<=max(C(1),D(1))) && (min(A(2),B(2))<=y_intersect) && (y_intersect<=max(A(2),B(2))) && (min(C(2),D(2))<=y_intersect) && (y_intersect<=max(C(2),D(2))))
        zero_set=0.01;
        if((min(A(1),B(1))-x_intersect<=zero_set) && (x_intersect-max(A(1),B(1))<=zero_set) && (min(C(1),D(1))-x_intersect<=zero_set) && (x_intersect-max(C(1),D(1))<=zero_set) && (min(A(2),B(2))-y_intersect<=zero_set) && (y_intersect-max(A(2),B(2))<=zero_set) && (min(C(2),D(2))-y_intersect<=zero_set) && (y_intersect-max(C(2),D(2))<=zero_set))   
            flag_intersect=1;
        end 
        
        if flag_intersect==1
            if (position_destination_add(1)<position_start_add(1))%���յ��������ߣ����������λ��
                position_temp=position_start_add;
                position_start_add=position_destination_add;
                position_destination_add=position_temp;
            end
           % if(position_start(1)<destination(1))
%                 vertices=[vertices,position_start_add'];
                position_start_add=obstacle_point(:,i+1)'-[0,400];%��������յ���
                vertices_add=[vertices_add,position_start_add'];              
%                 vertices=[vertices,position_start_add'];
%                 flag_change1=1; %������յ���ߵı��
%             else
%                 position_destination_add=obstacle_point(:,i+1)'-[300,400];%���յ��������,��ȡ�м��������ټ�����
%                 vertices=[vertices,position_destination_add'];
% %                 flag_change2=1; %������յ��ұߵı��
% %                 if(i==num_obstacle_point-2)
% %                     vertices(:,1)=[];
% %                     vertices=[vertices,current_position_add'];         
% %                 end
           % end
        %else
            %vertices=[vertices,destination'];
       end
    end
%     if(flag_change1==1)
%         vertices=[vertices,destination'];
%     elseif (flag_change2==1)
%         vertices(:,1)=[];
%         vertices=[destination',vertices,position_start'];
%         vertices=fliplr(vertices);
%     else %��ֱ�ߵ����
%         vertices=[vertices,destination'];
%     end
    if (destination(1)<destination1(1))
        vertices_add=fliplr(vertices_add);
    end
    vertices=[vertices,vertices_add];
    vertices=[vertices,destination'];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%���ͽǶ�
%�涨��ʱ��Ϊ��,˳ʱ��Ϊ��,�Ƕ�Ϊ����ת,�Ƕ�Ϊ����ת
%��ʼ�Ƕ�

% if(position_start(1)<destination(1))
%     flag_pos_de=1;
% else
%     flag_de_pos=1;
%    % vertices=fliplr(vertices);
% end



angle_all=[];
line_target=destination-position_start;
line_x=[line_target(1) 0];
b=vertices(:,1)';
c=vertices(:,2)';
ab=line_x;
bc=c-b;
if(ab(1)*bc(2)-ab(2)*bc(1)>=0)%bc��ab����ʱ�뷽��
    angle_flag=1;
else
    angle_flag=-1;
end
angle_flag_test=angle_flag*(acos(dot(ab,bc)/(norm(ab)*norm(bc))))*180/pi;
angle_all=[angle_all,angle_flag_test];
%�ϰ���֮��ĽǶ�
for i=1:length(vertices)-2
    a=vertices(:,i)';
    b=vertices(:,i+1)';
    c=vertices(:,i+2)';
    ab=b-a;
    bc=c-b;
    if(ab(1)*bc(2)-ab(2)*bc(1)>=0)%bc��ab����ʱ�뷽��
        angle_flag=1;
    else
        angle_flag=-1;
    end
    angle_flag_test=angle_flag*(acos(dot(ab,bc)/(norm(ab)*norm(bc))))*180/pi;
    angle_all=[angle_all,angle_flag_test];
end
%�Ƕȷ���
angle_send_flag=1;
if(position_start==vertices(:,angle_send_flag)')
    angle_send=angle_all(angle_send_flag);
    angle_send_flag=angle_send_flag+1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�����ٶ�
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
    angle_in=(acos(dot(l1,l2)/(norm(l1)*norm(l2))));%�жϵ�����������֮��,���жϵ�ǰλ������Χ������ļн�
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
if(distance_now<=k_speed*distance_all)%����ʱ���ٶȱ仯
    speed_send=distance_now*speed_max/(k_speed*distance_all);
elseif(distance_now>=(1-k_speed)*distance_all)%����ʱ���ٶȱ仯
    speed_send=speed_max-distance_now*speed_max/distance_all;
else%�������
    speed_send=speed_max;
end


t=toc;


map_cspace();
hold on;
%pause(0.05);
plot(vertices(1,:),vertices(2,:),'r');
hold off;
% (k2+1).*(k2<0)+(-k2+1).*(k2>=0)
