% Copyright (c) 2026 Masazumi Imai
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

clc; clear; close all;

%% User settings

type = "to";  % kj/gj/lrst/ramp-pmd/ramp-fmd/to
% csv_file = "csv" + filesep + type + "_" + "rosbag2_2026_04_22-04_44_11" + ".csv";
csv_file = "csv" + filesep + "to_rosbag2_2026_04_24-17_08_17" + ".csv";

save_fig = true;  % true/false

plot_ft = true;
plot_ee_error = true;
plot_joint_torque = true;
plot_base_pose = true;

%% Parameters

limb_names = ["limb_1", "limb_2"];
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"];

%% Read Data

data = readtable(csv_file);

sim_start_time = data.x__time(1, 1);
% time_vec = data.x__time - sim_start_time;

%% Synchronize

trigger_raw = data.(matlab.lang.makeValidName("x_start_control_data"));
t_trigger_idx = find(trigger_raw == 1, 1);
t_trigger = data.x__time(t_trigger_idx);

cmd_stamp_raw = data.(matlab.lang.makeValidName("x_joint_cmds_header_stamp"));
t_motion_idx = find(~isnan(cmd_stamp_raw(:, 1)), 1);
t_motion = data.x__time(t_motion_idx);

delay = t_motion - t_trigger;
fprintf('--- \nComputation Delay (Dead Time): %.4f [s]\n', delay);

% time_vec = data.x__time - t_trigger;
time_vec = data.x__time - t_motion;

%% Plot Force/Torque
if (plot_ft)
  for limb_id = 1 : length(limb_names)
    limb = limb_names(limb_id);

    force_raw(:, 1) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_force_x"));
    force_raw(:, 2) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_force_y"));
    force_raw(:, 3) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_force_z"));
    torque_raw(:, 1) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_torque_x"));
    torque_raw(:, 2) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_torque_y"));
    torque_raw(:, 3) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_torque_z"));

    valid_idx = ~isnan(force_raw(:, 1)) & ~isnan(torque_raw(:, 1));
    time_valid = time_vec(valid_idx);
    force_valid = force_raw(valid_idx, :);
    torque_valid = torque_raw(valid_idx, :);

    % Resultant Force and Torque
    res_force = vecnorm(force_valid, 2, 2);
    res_torque = vecnorm(torque_valid, 2, 2);

    mask = (time_valid >= 0);
    time_valid = time_valid(mask);
    res_force = res_force(mask);
    res_torque = res_torque(mask);

    % Plot Force
    y_label = "Limb " + num2str(limb_id) + " Reaction Force [N]";
    legends = [""];
    plotGraph(time_valid, res_force, y_label, legends);
    if (save_fig)
      fig_file_name = limb + "_reaction_force";
      saveas(gcf, fig_file_name + ".fig", "fig");
    end
    % Plot Torque
    y_label = "Limb " + num2str(limb_id) + " Reaction Moment [Nm]";
    legends = [""];
    plotGraph(time_valid, res_torque, y_label, legends);
    if (save_fig)
      fig_file_name = limb + "_reaction_moment";
      saveas(gcf, fig_file_name + ".fig", "fig");
    end

  end
end

%% Plot Swing EE Error
if (plot_ee_error)

  target_prefix = "x_tf_world_target_limb_2_palm_link_translation_";
  planned_prefix = "x_tf_world_planned_limb_2_palm_link_translation_";
  actual_prefix = "x_tf_world_limb_2_gripper_site_translation_";

  target_raw(:, 1) = data.(matlab.lang.makeValidName(target_prefix + "x"));
  target_raw(:, 2) = data.(matlab.lang.makeValidName(target_prefix + "y"));
  target_raw(:, 3) = data.(matlab.lang.makeValidName(target_prefix + "z"));

  planned_raw(:, 1) = data.(matlab.lang.makeValidName(planned_prefix + "x"));
  planned_raw(:, 2) = data.(matlab.lang.makeValidName(planned_prefix + "y"));
  planned_raw(:, 3) = data.(matlab.lang.makeValidName(planned_prefix + "z"));

  actual_raw(:, 1) = data.(matlab.lang.makeValidName(actual_prefix + "x"));
  actual_raw(:, 2) = data.(matlab.lang.makeValidName(actual_prefix + "y"));
  actual_raw(:, 3) = data.(matlab.lang.makeValidName(actual_prefix + "z"));

  valid_idx_planned = ~isnan(planned_raw(:, 1));
  time_planned_rel = time_vec(valid_idx_planned);
  planned_pos = planned_raw(valid_idx_planned, :);

  valid_idx_actual = ~isnan(actual_raw(:, 1));
  time_actual_rel = time_vec(valid_idx_actual);
  actual_pos = actual_raw(valid_idx_actual, :);

  mask_actual = (time_actual_rel >= 0);
  time_plot = time_actual_rel(mask_actual);
  actual_pos_synced = actual_pos(mask_actual, :);

  % planned_pos_interp = interp1(time_planned_rel, planned_pos, time_plot, 'linear', 'extrap');
  time_plot_clamped = min(time_plot, max(time_planned_rel));
  planned_pos_interp = interp1(time_planned_rel, planned_pos, time_plot_clamped, 'linear', 'extrap');

  error_vec = actual_pos_synced - planned_pos_interp;
  error_norm = vecnorm(error_vec, 2, 2);

  % RMSE for whole trajectory
  rmse_val = sqrt(mean(error_norm.^2));
  fprintf('Trajectory RMSE: %.4f [m] (%.2f [mm])\n', rmse_val, rmse_val * 1000);

  % Max Error for whole trajectory
  max_error = max(error_norm);
  fprintf('Max Absolute Error: %.4f [m] (%.2f [mm])\n', max_error, max_error * 1000);

  y_label = "Limb 2 End-Effector Error [mm]";
  legends = [""];
  plotGraph(time_plot, error_norm * 1000, y_label, legends);

  if (save_fig)
    saveas(gcf, "limb_2_tracking_error.fig", "fig");
  end

  valid_idx_target = find(~isnan(target_raw(:, 1)), 1);
  target_pos = target_raw(valid_idx_target, :);

  error_norm = norm(target_pos - actual_pos_synced(end, :));
  rmse_val = sqrt(mean(error_norm.^2));
  fprintf('Target RMSE: %.4f [m] (%.2f [mm])\n', rmse_val, rmse_val * 1000);

end

%% Plot Joint Torque

if (plot_joint_torque)

  total_control_effort_all = 0;

  for limb_id = 1 : length(limb_names)
    limb_name = limb_names(limb_id);

    for joint_id = 1 : length(joint_names)
      joint_name = joint_names(joint_id);
      tau_raw(:, joint_id) = data.(matlab.lang.makeValidName("x_joint_states_" + limb_name + "_" + joint_name + "_effort"));
    end

    valid_idx = ~isnan(tau_raw(:, 1));
    time_valid = time_vec(valid_idx);
    tau_valid = tau_raw(valid_idx, :);

    mask = (time_valid >= 0);
    time_valid = time_valid(mask);
    tau_valid = tau_valid(mask, :);

    tau_squared_sum = sum(tau_valid.^2, 2);

    limb_control_effort = trapz(time_valid, tau_squared_sum);
    fprintf('%s Total Control Effort: %.4f [N^2 m^2 s]\n', limb_name, limb_control_effort);

    total_control_effort_all = total_control_effort_all + limb_control_effort;

    % Plot Force
    y_label = "Limb " + num2str(limb_id) + " Joint Torque [Nm]";
    legends = joint_names;
    plotGraph(time_valid, tau_valid, y_label, legends);
    if (save_fig)
      fig_file_name = limb_name + "_joint_torque";
      saveas(gcf, fig_file_name + ".fig", "fig");
    end
  end

  fprintf('Whole Body Total Control Effort: %.4f [N^2 m^2 s]\n', total_control_effort_all);

end

%% Plot Base Orientation
if (plot_base_pose)

  base_pose_raw(:, 1) = data.(matlab.lang.makeValidName("x_odom_pose_pose_position_x"));
  base_pose_raw(:, 2) = data.(matlab.lang.makeValidName("x_odom_pose_pose_position_y"));
  base_pose_raw(:, 3) = data.(matlab.lang.makeValidName("x_odom_pose_pose_position_z"));
  base_pose_raw(:, 4) = data.(matlab.lang.makeValidName("x_odom_pose_pose_orientation_roll"));
  base_pose_raw(:, 5) = data.(matlab.lang.makeValidName("x_odom_pose_pose_orientation_pitch"));
  base_pose_raw(:, 6) = data.(matlab.lang.makeValidName("x_odom_pose_pose_orientation_yaw"));

  valid_idx = ~isnan(base_pose_raw(:, 1));
  time_valid = time_vec(valid_idx);
  base_pose_valid = base_pose_raw(valid_idx, :);

  base_pos = base_pose_valid(:, 1:3);
  base_ori = base_pose_valid(:, 4:6);

  % Plot base orientation
  y_label = "Base Orientation [rad]";
  legends = ["Roll", "Pitch", "Yaw"];
  plotGraph(time_valid, base_ori, y_label, legends);
  if (save_fig)
    fig_file_name = "base_orientation";
    saveas(gcf, fig_file_name + ".fig", "fig");
  end

end

%% Plot Time History of Data
function plotGraph(time, data, y_label, legends)

% === Visualize settings ===
% Line
line_width = 3;
% Axis
axis_font_name = "Helvetica";
axis_font_size = 20;
% x, y label
label_font_name = "Arial";
label_font_size = 20;
% Legend
legend_font_name = "Arial";
legend_font_size = 18;
% Figure size
graph_width  = 680;
graph_height = 420;
% ==========================

figure; hold on; box on; grid on;
for i = 1 : size(data, 2)
  plot(time, data(:, i), "LineWidth", line_width);
end
set(gcf, Color = "w", Position = [100, 100, graph_width, graph_height]);
set(gca, FontName = axis_font_name, FontSize = axis_font_size, LineWidth = line_width / 2);
xlabel("Time [s]", FontName = label_font_name, FontSize = label_font_size);
ylabel(y_label, FontName = label_font_name, FontSize = label_font_size);
if (length(legends) > 1 && legends(1) ~= "")
  legend(legends, FontName = legend_font_name, FontSize = legend_font_size);
end

end
