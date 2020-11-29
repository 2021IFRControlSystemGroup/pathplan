function [ ]=map_cspace()
rectangle('Position',[0,0,12000,12000],'LineWidth',1,'LineStyle','-');
hold on;
rectangle('Position',[0,6000,2000,6000],'LineWidth',1,'LineStyle','-','FaceColor','black');
hold on;
rectangle('Position',[10000,6000,2000,6000],'LineWidth',1,'LineStyle','-','FaceColor','black');
hold on;
rectangle('Position',[2000,9950,8000,2000],'LineWidth',1,'LineStyle','-','FaceColor','black');
hold on;

triangle_x=[2000,2000,4000,2000];
triangle_y=[9950,8000,9950,9950];
fill(triangle_x,triangle_y,'black');
hold on;
triangle_x=[10000,10000,8000,10000];
triangle_y=[9950,8000,9950,9950];
fill(triangle_x,triangle_y,'black');
hold on;

x=[4975 4000 2000 2000 4000 5975 5975 8000 10000 10000 8000 7025];
y=[2000 2000 4000 8000 9950 9950 2000 2000 4000 8000 9950 9950];
plot(x,y,'LineWidth',2,'color','r');

rectangle('Position',[5000,1950,1000,1000],'LineStyle','-','FaceColor','yellow');
hold on;
rectangle('Position',[6025,9000,1000,1000],'LineStyle','-','FaceColor','yellow');
hold on;

rectangle('Position',[4000-150,5700-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[4000-150,6300-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[5650-150,3500-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[5650-150,6000-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[5650-150,8500-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[8000-150,5700-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[8000-150,6300-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[6350-150,3500-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[6350-150,6000-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');
rectangle('Position',[6350-150,8500-150,2*150,2*150],'Curvature',[1,1],'FaceColor','black');

rectangle('Position',[6000-575,8500-575,2*575,2*575],'Curvature',[1,1]);
rectangle('Position',[6000-575,6000-575,2*575,2*575],'Curvature',[1,1]);
rectangle('Position',[6000-575,3500-575,2*575,2*575],'Curvature',[1,1]);

axis equal;
axis([0 12000 0 12000]);

end


