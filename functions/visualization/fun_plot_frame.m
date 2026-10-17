function fun_plot_frame(T,scale,linewidth)

p = T(1:3,4);
R = T(1:3,1:3);

x_axis = [1,0,0]' * scale;
y_axis = [0,1,0]' * scale;
z_axis = [0,0,1]' * scale;

x_axis = R * x_axis + p;
y_axis = R * y_axis + p;
z_axis = R * z_axis + p;

plot3([p(1),x_axis(1)],[p(2),x_axis(2)],[p(3),x_axis(3)],'r','LineWidth',linewidth);
hold on
plot3([p(1),y_axis(1)],[p(2),y_axis(2)],[p(3),y_axis(3)],'g','LineWidth',linewidth);
hold on
plot3([p(1),z_axis(1)],[p(2),z_axis(2)],[p(3),z_axis(3)],'b','LineWidth',linewidth);
hold on

axis equal