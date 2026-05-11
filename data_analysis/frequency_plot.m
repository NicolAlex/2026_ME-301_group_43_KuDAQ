point_25_01 = 0.144;
point_25_02 = 0.133;
point_25_03 = 0.131;

point_10_01 = 0.127;
point_10_02 = 0.137;

point_50_01 = 0.055;
point_50_02 = 0.048;
point_50_03 = 0.039;

close all;

figure("Name", "Frequency v.s. nozzle depth");
plot([25 25 25], [point_25_01 point_25_02 point_25_03], 'ro', 'DisplayName', 'Nozzle Depth 25 mm', 'LineWidth', 2);
hold on;
plot([10 10], [point_10_01 point_10_02], 'go', 'DisplayName', 'Nozzle Depth 10 mm', 'LineWidth', 2);
plot([50 50 50], [point_50_01 point_50_02 point_50_03], 'bo', 'DisplayName', 'Nozzle Depth 50 mm', 'LineWidth', 2);
xlabel('Nozzle Depth (mm)');
ylabel('Frequency (Hz)');
title('Frequency vs. Nozzle Depth');
legend;
grid on;
