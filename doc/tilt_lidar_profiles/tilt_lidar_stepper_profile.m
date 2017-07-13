clear all
close all

%%Stepper specific variables
stepper_gear_ratio_num = 74.0;
stepper_gear_ratio_den = 16.0;
stepper_gear_ratio = stepper_gear_ratio_num / stepper_gear_ratio_den;
steps_per_rev = 200;
micro_steps_per_step = 128;
micro_steps_per_rev = (steps_per_rev * micro_steps_per_step);
rad_per_micro_step = 2.0*pi/(micro_steps_per_rev * stepper_gear_ratio);

%%Hardware timer variables
timebase = 84000000;
prescaler = 1;
tics_per_sec = timebase ./ prescaler;
timer_bits = 32;
max_tics = (2^timer_bits) - 1;
max_time = max_tics ./ tics_per_sec

%%Tilt Angles (rad)
tilt_half_angle = 2.0*pi/4.0;
min_tilt_rad = -tilt_half_angle;
max_tilt_rad = tilt_half_angle;

max_rad_per_sec = 10.0;

%Discrete Time Information (Motor Control State Machine Period)
dt = 0.001;
%Time for Profile (sec)
total_time = 1.0;

avg_vel = (max_tilt_rad - min_tilt_rad) ./ total_time;

t = 0:dt:total_time;

%%Minimum Jerk Equation
% s(t) = xi + (xf - xi)(10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5)
s_min_jerk = min_tilt_rad + (max_tilt_rad - min_tilt_rad).*(10.*((t./total_time).^3) 
  - 15.*((t./total_time).^4) + 6.*((t./total_time).^5));
  
s_stepper = min_tilt_rad:rad_per_micro_step:max_tilt_rad;
t_stepper = interp1 (s_min_jerk, t, s_stepper, "spline");

t_stepper_tics(1) = 0;
for ii=2:size(t_stepper,2)
  t_stepper_tics(ii) = round((t_stepper(ii) - t_stepper(ii-1)) .* tics_per_sec);
endfor

t_stepper_discrete = t_stepper_tics ./ tics_per_sec;
for ii=2:size(t_stepper_discrete,2)
  t_stepper_discrete(ii) += t_stepper_discrete(ii-1);
endfor

graphics_toolkit("gnuplot");
figure(1)
plot(t,s_min_jerk,"+;Minimum Jerk Profile;", t_stepper_discrete,s_stepper,"*;Stepper Minimum Jerk;")
title("Rotating LIDAR Position Command")
xlabel("Time (sec)")
ylabel("Position (rad)")

fid = fopen("tilt_stepper_motor_profile.h", "w");
fprintf(fid, "#ifndef TILT_STEPPER_MOTOR_PROFILE_H\n");
fprintf(fid, "#define TILT_STEPPER_MOTOR_PROFILE_H\n");
fprintf(fid, "\n");
fprintf(fid, "/* Parameters used to calculate this table. */\n");
fprintf(fid, "/* steps_per_rev = %u */\n", steps_per_rev);
fprintf(fid, "/* micro_steps_per_step = %u */\n", micro_steps_per_step);
fprintf(fid, "uint32_t micro_steps_per_rev = %u;\n", micro_steps_per_rev);
fprintf(fid, "/* tics_per_sec = %u */\n", tics_per_sec);
fprintf(fid, "/* Table is in units of tics per micro step. */\n");
fprintf(fid, "float stepper_gear_ratio_num = %.12ff;\n", stepper_gear_ratio_num);
fprintf(fid, "float stepper_gear_ratio_den = %.12ff;\n", stepper_gear_ratio_den);
fprintf(fid, "float rad_per_micro_step = %.12ff;\n", rad_per_micro_step);
fprintf(fid, "static uint32_t tilt_elements = %u;\n", size(t_stepper_tics,2)-1);
fprintf(fid, "static uint32_t stepper_profile[] = { \n");
fprintf(fid, "%u, %u, %u, %u, %u, %u, %u, %u,\n", t_stepper_tics(2:size(t_stepper_tics,2)));
fprintf(fid, "0};\n");
fprintf(fid, "\n");
fprintf(fid, "#endif\n");

fclose(fid);

