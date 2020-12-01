function [ vertices ] = B_sort( current_position,temp_position1,temp_position2,temp_position3,temp_position4,destination,in_obstacle,flag)
    %UNTITLED 此处显示有关此函数的摘要
    %   此处显示详细说明
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
    if flag==1
        vertices=[current_position;temp1;temp4;destination ]';
    else
        vertices=[current_position;temp1;temp2;temp3;temp4;destination ]';
    end
end

