function [flag]=line_intersect(A,B,C,D)
%     A=[x1,y1];
%     B=[x2,y2];
%     C=[x3,y3];
%     D=[x4,y4];
    flag=0;
    AB=B-A;
    AC=C-A;
    AD=D-A;
    AB_cross_AC=AB(1)*AC(2)-AC(1)*AB(2);
    AB_cross_AD=AB(1)*AD(2)-AD(1)*AB(2);
    if (AB_cross_AC*AB_cross_AD<=0)
        flag=1;
    end
end
