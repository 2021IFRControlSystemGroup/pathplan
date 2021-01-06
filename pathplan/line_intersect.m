function [flag]=line_intersect(A,B,C,D)
    flag=0;
    AB=B-A;CD=D-C;
    k1=AB(2)/AB(1);
    k2=CD(2)/CD(1);
    b1=A(2)-k1*A(1);
    b2=C(2)-k2*C(1);
    x_intersect=(b2-b1)/(k1-k2);
    y_intersect=k1*x_intersect+b1;
    if((min(A(1),B(1))<=x_intersect) && (x_intersect<=max(A(1),B(1))) && (min(C(1),D(1))<=x_intersect) && (x_intersect<=max(C(1),D(1))) && (min(A(2),B(2))<=y_intersect) && (y_intersect<=max(A(2),B(2))) && (min(C(2),D(2))<=y_intersect) && (y_intersect<=max(C(2),D(2))))
        flag=1;
    end




%     A=[x1,y1];
%     B=[x2,y2];
%     C=[x3,y3];
%     D=[x4,y4];
%     flag=0;
%     AB=B-A;
%     AC=C-A;
%     AD=D-A;
%     BC=C-B;
%     BD=D-B;
%     AB_cross_AC=AB(1)*AC(2)-AC(1)*AB(2);
%     AB_cross_AD=AB(1)*AD(2)-AD(1)*AB(2);
%     AB_cross_BC=AB(1)*BC(2)-BC(1)*AB(2);
%     AB_cross_BD=AB(1)*BD(2)-BD(1)*AB(2);
%     if (AB_cross_AC*AB_cross_AD<=0 && AB_cross_BC*AB_cross_BD<=0)
%         flag=1;
%     end
end
%flag_test=line_intersect([2000 1200],[1000 1000],[8200 2000],[10300 3800]);
