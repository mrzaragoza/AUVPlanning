% function [ yn_1 ] = runggekutta( t, h, yn )
clc; 
clear; 
close all;

fileID = fopen('pruebaControles.txt','r');
controles = textscan(fileID,'%s %s %s %s');
num_controles = length(controles{1});
t_tot = 0;
t_array = [0];
for t_i = 1:num_controles
    t_array = [t_array t_array(t_i)+str2double(controles{4}{t_i})];
    t_tot = t_tot + str2double(controles{4}{t_i});
end
num_controles = num_controles;
t_array = round(t_array(1:end));
t_tot = round(t_tot);
% return
h=0.1;
t = zeros(1,t_tot/h);
t_index = 1;
yn = zeros(t_tot/h,8);
% yn = zeros(100,4);
yn(1,:) = [0 0 0 0 0 0 0 0];
t_actual = 0;
for j = 1:num_controles

    inicio = t_array(j);
    fin    = t_array(j+1);
    t_saltos = (fin - inicio) / h;
    if inicio == 0
       t_saltos = t_saltos - 1; 
    end
    for i = 1:t_saltos
    % for i= 1:length(t)
        t_actual = t_actual+1;
        control = [str2double(controles{1}{j}) str2double(controles{2}{j}) str2double(controles{3}{j})];
        k1 = ode(yn(t_actual,:), control(:)); 
        k2 = ode(yn(t_actual,:) + h/2*k1,control(:));
        k3 = ode(yn(t_actual,:) + h/2*k2,control(:));
        k4 = ode(yn(t_actual,:) + h*k3,control(:));

        yn(t_actual+1,:) = yn(t_actual,:) + (h/6) .* (k1 + 2*k2 + 2*k3 + k4);

        yn(t_actual+1,4) = rem(yn(t_actual+1,4),2*pi);
        if yn(t_actual+1,4)>pi && yn(t_actual+1,4)<2*pi
            yn(t_actual+1,4) = yn(t_actual+1,4)-2*pi;
        elseif yn(t_actual+1,4)<-pi && yn(t_actual+1,4)>-2*pi
            yn(t_actual+1,4) = yn(t_actual+1,4)+2*pi;
        end
        
        if yn(t_actual+1,5)>3
            yn(t_actual+1,5) = 3;
        elseif yn(t_actual+1,5)<-3
            yn(t_actual+1,5) = -3;
        end
        
        if yn(t_actual+1,6)>3
            yn(t_actual+1,6) = 3;
        elseif yn(t_actual+1,6)<-3
            yn(t_actual+1,6) = -3;
        end
        
        if yn(t_actual+1,7)>3
            yn(t_actual+1,7) = 3;
        elseif yn(t_actual+1,7)<-3
            yn(t_actual+1,7) = -3;
        end
        
        t(t_index+1) = t(t_index) + h;
        t_index = t_index+1;
    end

end

figure('name','x_y');
 
plot3(yn(:,2),yn(:,1),yn(:,3));
hold on;
for i_plot = 2:length(t_array)
plot3(yn(t_array(i_plot)/h,2),yn(t_array(i_plot)/h,1),yn(t_array(i_plot)/h,3),'*r')
end
% plot(yn(:,1),yn(:,2));
% set(spxy,'ydir','reverse');
legend('XYZ')

figure('name','x_y_z');

spxy=subplot(2,3,1); 
plot(yn(:,2),yn(:,1));
hold on;
for i_plot = 2:length(t_array)
plot(yn(t_array(i_plot)/h,2),yn(t_array(i_plot)/h,1),'*r')
end
% plot(yn(:,1),yn(:,2));
% set(spxy,'ydir','reverse');
legend('XY')

subplot(2,3,2)
plot(t(),yn(:,5));
hold on;
for i_plot = 1:length(t_array)
plot([t_array(i_plot) t_array(i_plot)],[-2 2],'r')
end
legend('vel_x')

subplot(2,3,3)
plot(t(),yn(:,6));
hold on;
for i_plot = 1:length(t_array)
plot([t_array(i_plot) t_array(i_plot)],[-3 3],'r')
end
legend('vel_y')

spz=subplot(2,3,4);
plot(t(),yn(:,3));
hold on;
for i_plot = 2:length(t_array)
plot(t(t_array(i_plot)/h),yn(t_array(i_plot)/h,3),'*r')
end
set(spz,'ydir','reverse');
legend('Z')

subplot(2,3,5)
plot(t(:),yn(:,7));
hold on;
for i_plot = 1:length(t_array)
plot([t_array(i_plot) t_array(i_plot)],[-2 2],'r')
end
legend('vel_z')

subplot(2,3,6)
plot(yn(:,6),yn(:,5));
legend('vel_x vel_y')

figure('name','yaw');
subplot(1,2,1)
plot(t(:),yn(:,4));
hold on;
for i_plot = 2:length(t_array)
plot(t(t_array(i_plot)/h),yn(t_array(i_plot)/h,4),'*r')
end
legend('yaw')

subplot(1,2,2)
plot(t(:),yn(:,8));
hold on;
for i_plot = 1:length(t_array)
plot([t_array(i_plot) t_array(i_plot)],[-2 2],'r')
end
legend('vel_y_a_w')



% end

