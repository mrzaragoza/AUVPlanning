function [ control, tiempo ] = controlSampler( st_actual, st_final, st_inicial )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

control = zeros(1,3);
max_diff_thrusters = 0.5;
margen_cercania = 0.2;
tiempo_base = 60; %segundos
control_z_estable = 0.07;

%Distancias con estado inicial
diff_x_inicial = st_final(1)-st_inicial(1);
diff_y_inicial = st_final(2)-st_inicial(2);
diff_z_inicial = st_final(3)-st_inicial(3);
dist_total = sqrt(diff_x_inicial^ 2 + diff_y_inicial^2 );
dist_total_3D = sqrt(diff_x_inicial^ 2 + diff_y_inicial^2 + diff_z_inicial^2 );

%Distancias con estado actual
diff_x = st_final(1)-st_actual(1);
diff_y = st_final(2)-st_actual(2);
diff_z = st_final(3)-st_actual(3)
dist_actual = sqrt(diff_x^ 2 + diff_y^2 );
dist_actual_3D = sqrt(diff_x^ 2 + diff_y^2 + diff_z^2);

%Normalización entre 0 y 1 de la distancia actual respecto a la inicial
if(dist_total ~= 0)
    norm_distancia = dist_actual / dist_total
else
    norm_distancia = 0
end;

if(dist_total_3D ~= 0)
    norm_distancia_3D = dist_actual_3D / dist_total_3D;
else
    norm_distancia_3D = 0;
end;

if(diff_z_inicial ~= 0)
    norm_distancia_z = diff_z / diff_z_inicial;
else
    norm_distancia_z = 0;
end;

%Calculo del ángulo que hay entre el estado actual y el final y la
%diferencia entre éste y el yaw del vehículo
heading = atan2(diff_y,diff_x) %radianes
diff_heading = heading - st_actual(4)

num_aleatorio = rand(1);
if(diff_heading > margen_cercania)
    control(2) = num_aleatorio + rand(1)*max_diff_thrusters/2;
    control(3) = num_aleatorio - rand(1)*max_diff_thrusters/2;
elseif(diff_heading < -margen_cercania)
    control(2) = num_aleatorio - rand(1)*max_diff_thrusters/2;
    control(3) = num_aleatorio + rand(1)*max_diff_thrusters/2;
else %diff_heading ~ 0
    control(2) = num_aleatorio + (rand(1)*2 -1)*max_diff_thrusters/8;
    control(3) = num_aleatorio + (rand(1)*2 -1)*max_diff_thrusters/8;
end

if(diff_z > margen_cercania)
    control(1) = rand(1);
elseif(diff_z < -margen_cercania)
    control(1) = rand(1)*(-1);
else %diff_z ~ 0
    control(1) = (rand(1)*2 - 1)
    control(1) = control(1) * 0.1 * control_z_estable
end

%Se multiplica por la normalización de la distancia, haciendo que cuanto
%más cerca se esté del objetivo, los controles sean más pequeños.

if(norm_distancia_z ~= 0 )
    control(1) = control(1) * norm_distancia_z;
end;
    
control(2) = control(2) * norm_distancia;
control(3) = control(3) * norm_distancia;

if(control(1)>1)
    control(1) = 1;
elseif(control(1)<-1)
    control(1) = -1;
end

if(control(2)>1)
    control(2) = 1;
elseif(control(2)<-1)
    control(2) = -1;
end

if(control(3)>1)
    control(3) = 1;
elseif(control(3)<-1)
    control(3) = -1;
end

%Tiempo a ejecutar
tiempo = rand(1) * tiempo_base * norm_distancia_3D;

end

