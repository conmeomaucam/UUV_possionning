function roll_boresight_plot(result)
%ROLL_BORESIGHT_PLOT Visualization only; excluded from code-generation core.

time = result.time;
calibration_end = result.calibration_end_s;
underwater = result.underwater_mask;
position_true = result.position_true_ned;
position_est = result.position_est_ned;
velocity_true_ned = result.velocity_true_ned;
velocity_est_ned = result.velocity_est_ned;
attitude_true = result.attitude_true_rpy;
attitude_est = result.attitude_est_rpy;
parameters = result.parameter_history;
parameter_sigma = result.parameter_sigma_history;
parameter_truth = result.parameter_truth;

figure('Name', 'Roll-boresight: trajectory', 'Color', 'w', 'Position', [40 60 1350 620]);
subplot(1, 2, 1);
plot3(position_true(:, 2), position_true(:, 1), -position_true(:, 3), ...
    'r-', 'LineWidth', 1.8); hold on; grid on;
plot3(position_est(:, 2), position_est(:, 1), -position_est(:, 3), ...
    'k--', 'LineWidth', 1.3);
plot3(0, 0, 0, 'go', 'MarkerFaceColor', 'g');
plot3(position_true(result.dive_index, 2), position_true(result.dive_index, 1), ...
    -position_true(result.dive_index, 3), 'm^', 'MarkerFaceColor', 'm');
plot3(result.destination_ned(2), result.destination_ned(1), ...
    -result.destination_ned(3), 'bp', 'MarkerFaceColor', 'b');
xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
title('Quỹ đạo 3D');
legend('UUV thật', 'ES-EKF', 'Xuất phát', 'Bắt đầu lặn', 'Đích', 'Location', 'best');
axis equal; view([-35 25]);

subplot(1, 2, 2);
plot(position_true(:, 2), position_true(:, 1), 'r-', 'LineWidth', 1.8); hold on; grid on;
plot(position_est(:, 2), position_est(:, 1), 'k--', 'LineWidth', 1.3);
xline(position_true(result.dive_index, 2), 'm:', 'Tắt GNSS');
xlabel('East (m)'); ylabel('North (m)'); title('Mặt bằng N-E');
legend('UUV thật', 'ES-EKF', 'Location', 'best'); axis equal;

figure('Name', 'Roll-boresight: UUV attitude and errors', ...
    'Color', 'w', 'Position', [80 40 1300 760]);
attitude_true_deg = [attitude_true(:, 1:2), unwrap(attitude_true(:, 3))] * 180/pi;
attitude_est_deg = attitude_est * 180/pi;
attitude_est_deg(:, 3) = unwrap(attitude_est(:, 3)) * 180/pi;
yaw_offset_deg = 360 * round((attitude_true_deg(1, 3) - attitude_est_deg(1, 3)) / 360);
attitude_est_deg(:, 3) = attitude_est_deg(:, 3) + yaw_offset_deg;
attitude_names = {'Roll UUV \phi (deg)', 'Pitch UUV \theta (deg)', 'Yaw UUV \psi (deg)'};

for axis_idx = 1:3
    subplot(3, 2, 2*axis_idx - 1);
    plot(time, attitude_true_deg(:, axis_idx), 'r-', 'LineWidth', 1.4); hold on; grid on;
    plot(time, attitude_est_deg(:, axis_idx), 'k--', 'LineWidth', 1.1);
    xline(calibration_end, 'm--', 'Tắt GNSS');
    ylabel(attitude_names{axis_idx});
    title([attitude_names{axis_idx}, ': tư thế UUV']);
    legend('UUV thật', 'ES-EKF ước lượng', 'Location', 'best');

    subplot(3, 2, 2*axis_idx);
    if axis_idx == 1
        plot(time, result.velocity_true_body(:, 2), 'b-', 'LineWidth', 1.2); hold on;
        plot(time, result.velocity_true_body(:, 3), 'Color', [0 0.6 0], 'LineWidth', 1.2);
        ylabel('Vận tốc thân (m/s)');
        legend('Sway v', 'Heave w', 'Location', 'best');
    elseif axis_idx == 2
        plot(time, position_true(:, 3), 'r-', 'LineWidth', 1.3); hold on;
        plot(time, position_est(:, 3), 'k--', 'LineWidth', 1.1);
        ylabel('Độ sâu Down (m)');
        legend('UUV thật', 'ES-EKF', 'Location', 'best');
    else
        underwater_time = time(underwater) - calibration_end;
        plot(underwater_time, result.position_error_horizontal, 'b-', 'LineWidth', 1.2); hold on;
        plot(underwater_time, result.position_error_norm, 'k--', 'LineWidth', 1.2);
        ylabel('Sai số vị trí (m)');
        legend('Ngang', '3D', 'Location', 'best');
    end
    grid on; xlabel('Thời gian (s)'); xline(calibration_end, 'm--', 'Tắt GNSS');
end

figure('Name', 'Roll-boresight: parameter convergence', ...
    'Color', 'w', 'Position', [120 60 1200 820]);
parameter_names = {'Scale factor DVL s (%)', ...
    'Boresight alpha: roll DVL-IMU (deg)', ...
    'Boresight beta: pitch DVL-IMU (deg)', ...
    'Boresight gamma: yaw DVL-IMU (deg)'};
for parameter_idx = 1:4
    subplot(4, 1, parameter_idx);
    h_estimate = plot(time, parameters(:, parameter_idx), 'b-', 'LineWidth', 1.4); hold on; grid on;
    h_sigma = plot(time, parameter_truth(parameter_idx) + 3*parameter_sigma(:, parameter_idx), 'k:');
    plot(time, parameter_truth(parameter_idx) - 3*parameter_sigma(:, parameter_idx), ...
        'k:', 'HandleVisibility', 'off');
    h_truth = yline(parameter_truth(parameter_idx), 'r--');
    xline(calibration_end, 'm--', 'Tắt GNSS');
    ylabel(parameter_names{parameter_idx});
    title(parameter_names{parameter_idx});
    if parameter_idx == 1
        legend([h_estimate, h_sigma, h_truth], ...
            {'ES-EKF ước lượng', 'Dải bất định ±3 sigma', 'Giá trị thật'}, ...
            'Location', 'best');
    end
end
xlabel('Thời gian (s)');

figure('Name', 'Roll-boresight: NED velocity', 'Color', 'w', ...
    'Position', [160 80 1100 720]);
ned_names = {'North v_N (m/s)', 'East v_E (m/s)', 'Down v_D (m/s)'};
for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    plot(time, velocity_true_ned(:, axis_idx), 'r-', 'LineWidth', 1.4); hold on; grid on;
    plot(time, velocity_est_ned(:, axis_idx), 'k--', 'LineWidth', 1.1);
    xline(calibration_end, 'm--', 'Tắt GNSS');
    ylabel(ned_names{axis_idx});
    legend('UUV thật', 'ES-EKF ước lượng', 'Location', 'best');
end
xlabel('Thời gian (s)');
end