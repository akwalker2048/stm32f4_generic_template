clear all
close all

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

graphics_toolkit("gnuplot");
figure(1)
plot(t,s_min_jerk,";Minimum Jerk Profile;")
title("Rotating LIDAR Position Command")
xlabel("Time (sec)")
ylabel("Position (rad)")

fid = fopen("profile.c", "w");
fprintf(fid, "static uint32_t tilt_elements = %u;\n", size(t,2));
fprintf(fid, "static float tilt_profile[] = { \n");
fprintf(fid, "%ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff,\n", s_min_jerk);
fprintf(fid, "};");
fclose(fid);

%%Try Martin's Tangent Profile
tilt_angle = linspace(min_tilt_rad, max_tilt_rad, length(t));
normalized_vel_ideal = abs(1.0./tan(tilt_angle));

practical_vel = normalized_vel_ideal.^1;
for ii = 1:length(practical_vel)
  if(practical_vel(ii) > max_rad_per_sec)
    practical_vel(ii) = max_rad_per_sec;
  endif
endfor
scale_factor = sum(practical_vel) / length(practical_vel);
practical_vel = practical_vel .* avg_vel ./ scale_factor;

practical_pos = cumtrapz(practical_vel).*dt + min_tilt_rad;


figure(2)
plot(tilt_angle, normalized_vel_ideal)

figure(3)
plot(t, practical_pos,";Tangent Curve;", t,s_min_jerk,";Min Jerk;")


fid = fopen("tangent.c", "w");
fprintf(fid, "static uint32_t tilt_elements = %u;\n", size(t,2));
fprintf(fid, "static float tilt_profile[] = { \n");
fprintf(fid, "%ff, %ff, %ff, %ff, %ff, %ff, %ff, %ff,\n", s_min_jerk);
fprintf(fid, "};");
fclose(fid);