function [ v_estado_dot ] = ode( v_estado, v_control )

mass = 63.2;
long = 1.615; rad = 0.11;
l1 = 0.3; l2 = 0.3;
vol_fluid_displaced = pi * rad^2 * long;
c_rbm = [mass, mass, mass, 8 ];
c_am = 0.1.*c_rbm;
c_ld = [20, 60, 60, 8];
c_qd = [30, 60, 60, 10];

W =  mass * 9.81;
B = 1030 * 9.81 * vol_fluid_displaced;

v_control = v_control .* 5;

vel = v_estado(5:8);

v_estado_dot = zeros(1,8);
v_estado_dot(1:4) = vel;

%%%%%%%%%%%%%%%%%%%% Pasamos la velocidad del sist. NE al sistema XY
vel_N = vel(1);
vel_E = vel(2);
vel(1) =  cos(v_estado(4)) * vel_N + sin(v_estado(4)) * vel_E;
vel(2) = -sin(v_estado(4)) * vel_N + cos(v_estado(4)) * vel_E;
%%%%%%%%%%%%%%%%%%%%

ac_surge = (  ( v_control(2) + v_control(3) ) ...
    - ( c_ld(1) + c_qd(1) * abs(vel(1)) )*vel(1) ) ...
    / ( c_rbm(1) + c_am(1) );

ac_sway = (  0 - ( c_ld(2) + c_qd(2) * abs(vel(2)) )*vel(2) ) ...
    / ( c_rbm(2) + c_am(2) );

ac_heave = ( ( v_control(1) ) ...
    - ( c_ld(3) + c_qd(3) * abs(vel(3)) )*vel(3) + (W - B) )  ...
    / ( c_rbm(3) + c_am(3) );

ac_yaw = ( ( l1*v_control(2) - l2*v_control(3) ) ...
    - ( c_ld(4) + c_qd(4) * abs(vel(4)) )*vel(4) ) ...
    / ( c_rbm(4) + c_am(4) );

%%%%%%%%%%%%%%%%%%%% Pasamos la acceleración del sist. XY al sistema NE
acc_N = ac_surge * cos(v_estado(4)) - ac_sway * sin(v_estado(4));
acc_E = ac_surge * sin(v_estado(4)) + ac_sway * cos(v_estado(4));
%%%%%%%%%%%%%%%%%%%%

v_estado_dot(5:8) = [acc_N, acc_E, ac_heave, ac_yaw];
end

