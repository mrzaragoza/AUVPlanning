function [ v_estado_dot ] = ode4D( v_estado, v_control )

l1 = -0.3; l2 = 0.3;

v_control = v_control .* 5;
v_estado_dot = zeros(1,4);
vx = (v_control(2)+v_control(3)) * cos(v_estado(4));
vy = (v_control(2)+v_control(3)) * sin(v_estado(4));
vz = v_control(1);
vyaw = (-l1 * v_control(2) - l2 * v_control(3));
v_estado_dot(1:4) = [vx, vy, vz, vyaw];
end

