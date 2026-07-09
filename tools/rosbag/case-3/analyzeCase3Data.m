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

case_id = 3;

% Select one method and one uncertainty level.
% Examples: "baseline", "gjm", "ramp-pmd", "ramp-fmd", "resto"
method_name = "gjm";

% Uncertainty level in percent. Examples: 5, 10, 20
uncertainty_percent = 20;

% Seeds to analyze.
% This script assumes seed IDs from seed_start to seed_end.
% Example: seed_start = 1; seed_end = 4; analyzes seed01, seed02, seed03, seed04.
seed_start = 0;
seed_end = 4;

% Case 3 uses the same swing motion as Case 1 by default.
swing_duration = 10.0;

% Directory settings
csv_dir = method_name + filesep + "csv";
output_dir = "analysis";

% File name format.
% Default expected format:
%   <method>_err<uncertainty_percent>_seed<seed_id>.csv
% Example:
%   baseline_err10_seed01.csv
file_pattern = "inertial_err_" + uncertainty_percent + "_seed%02d.csv";

% Robot settings
limb_names = ["limb_1", "limb_2"];
support_limb = "limb_1";
swing_limb = "limb_2";
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", ...
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"];

% Success threshold settings
% Set Inf if you do not want to use that condition.
terminal_error_threshold_m = Inf;  % e.g., 0.05 for 5 cm
max_base_angle_threshold_deg = Inf;
max_reaction_force_threshold_N = Inf;

% Save outputs
save_summary_csv = true;

%% Analyze selected seeds

if ~exist(output_dir, "dir")
    mkdir(output_dir);
end

seed_ids = seed_start:seed_end;
num_trials = numel(seed_ids);

results = table();

fprintf('=== Case 3 analysis ===\n');
fprintf('Method: %s\n', method_name);
fprintf('Uncertainty: %d %%\n', uncertainty_percent);
fprintf('Seeds: %d to %d\n', seed_start, seed_end);
fprintf('Swing duration: %.2f s\n', swing_duration);
fprintf('CSV directory: %s\n', csv_dir);
fprintf('-----------------------\n');

for trial_idx = 1:num_trials
    seed_id = seed_ids(trial_idx);
    csv_name = sprintf(file_pattern, seed_id);
    csv_file = csv_dir + filesep + string(csv_name);

    fprintf('\n[%d/%d] Reading: %s\n', trial_idx, num_trials, csv_file);

    if ~isfile(csv_file)
        warning('CSV file not found. This seed is skipped: %s', csv_file);

        result_row = makeEmptyResultRow(method_name, uncertainty_percent, seed_id, csv_file);
        results = [results; result_row]; %#ok<AGROW>
        continue;
    end

    try
        metrics = analyzeSingleCsv(csv_file, swing_duration, limb_names, support_limb, swing_limb, joint_names);

        success = true;
        if metrics.terminal_error_m > terminal_error_threshold_m
            success = false;
        end
        if metrics.base_max_angle_deg > max_base_angle_threshold_deg
            success = false;
        end
        if metrics.support_peak_force_N > max_reaction_force_threshold_N
            success = false;
        end

        result_row = table( ...
            string(method_name), uncertainty_percent, seed_id, string(csv_file), success, ...
            metrics.delay_s, ...
            metrics.support_peak_force_N, metrics.support_rms_force_N, metrics.support_impulse_Ns, ...
            metrics.support_peak_moment_Nm, metrics.support_rms_moment_Nm, ...
            metrics.terminal_error_m, metrics.terminal_error_m * 1000.0, ...
            metrics.trajectory_rmse_m, metrics.trajectory_rmse_m * 1000.0, ...
            metrics.trajectory_max_error_m, metrics.trajectory_max_error_m * 1000.0, ...
            metrics.total_effort_N2m2s, metrics.support_effort_N2m2s, metrics.swing_effort_N2m2s, ...
            metrics.base_final_angle_deg, metrics.base_max_angle_deg, ...
            'VariableNames', { ...
            'Method', 'UncertaintyPercent', 'Seed', 'CsvFile', 'Success', ...
            'Delay_s', ...
            'SupportPeakForce_N', 'SupportRmsForce_N', 'SupportImpulse_Ns', ...
            'SupportPeakMoment_Nm', 'SupportRmsMoment_Nm', ...
            'TerminalError_m', 'TerminalError_mm', ...
            'TrajectoryRMSE_m', 'TrajectoryRMSE_mm', ...
            'TrajectoryMaxError_m', 'TrajectoryMaxError_mm', ...
            'TotalEffort_N2m2s', 'SupportEffort_N2m2s', 'SwingEffort_N2m2s', ...
            'BaseFinalAngle_deg', 'BaseMaxAngle_deg' ...
            });

        results = [results; result_row]; %#ok<AGROW>

        fprintf('  Peak force      : %.4f N\n', metrics.support_peak_force_N);
        fprintf('  RMS force       : %.4f N\n', metrics.support_rms_force_N);
        fprintf('  Terminal error  : %.4f mm\n', metrics.terminal_error_m * 1000.0);
        fprintf('  Total effort    : %.4f N^2 m^2 s\n', metrics.total_effort_N2m2s);
        fprintf('  Max base angle  : %.4f deg\n', metrics.base_max_angle_deg);
        fprintf('  Success         : %d\n', success);

    catch ME
        warning('Failed to analyze %s: %s', csv_file, ME.message);

        result_row = makeEmptyResultRow(method_name, uncertainty_percent, seed_id, csv_file);
        results = [results; result_row]; %#ok<AGROW>
    end
end

%% Summarize statistics

valid_results = results(~isnan(results.SupportPeakForce_N), :);
success_rate = mean(valid_results.Success) * 100.0;

fprintf('\n=== Summary ===\n');
fprintf('Valid trials: %d / %d\n', height(valid_results), num_trials);
fprintf('Success rate: %.1f %%\n', success_rate);

summary = summarizeResults(valid_results, method_name, uncertainty_percent, success_rate);
disp(summary);

if save_summary_csv
    trial_output_file = output_dir + filesep + method_name + "_err" + uncertainty_percent + "_trial_metrics.csv";
    summary_output_file = output_dir + filesep + method_name + "_err" + uncertainty_percent + "_summary.csv";

    writetable(results, trial_output_file);
    writetable(summary, summary_output_file);

    fprintf('\nSaved trial metrics to: %s\n', trial_output_file);
    fprintf('Saved summary to      : %s\n', summary_output_file);
end

%% Local functions

function metrics = analyzeSingleCsv(csv_file, swing_duration, limb_names, support_limb, swing_limb, joint_names)

data = readtable(csv_file);

% Synchronize time with the first joint command, as in plotData.m.
trigger_raw = data.(matlab.lang.makeValidName("x_start_control_data"));
t_trigger_idx = find(trigger_raw == 1, 1);
if isempty(t_trigger_idx)
    error('No control trigger was found.');
end
if (any(strcmp(data.Properties.VariableNames, 'x__time')))
    x__time = data.x__time;
else
    x__time = data.time;
end
t_trigger = x__time(t_trigger_idx);

if (any(strcmp(data.Properties.VariableNames, 'x_joint_cmds_header_stamp')))
    cmd_stamp_raw = data.(matlab.lang.makeValidName("x_joint_cmds_header_stamp"));
else
    cmd_stamp_raw = data.(matlab.lang.makeValidName("x_joint_cmds_header_stamp_sec"));
end
t_motion_idx = find(~isnan(cmd_stamp_raw(:, 1)), 1);
if isempty(t_motion_idx)
    error('No joint command timestamp was found.');
end
t_motion = x__time(t_motion_idx);

time_vec = x__time - t_motion;
metrics.delay_s = t_motion - t_trigger;

% Supporting limb reaction force and moment.
[time_ft, res_force, res_moment] = getEndEffectorWrench(data, time_vec, support_limb);
mask_swing = (time_ft >= 0 & time_ft <= swing_duration);
time_ft_eval = time_ft(mask_swing);
force_eval = res_force(mask_swing);
moment_eval = res_moment(mask_swing);

if isempty(time_ft_eval)
    error('No valid force/torque data within the swing duration.');
end

metrics.support_peak_force_N = max(force_eval);
metrics.support_rms_force_N = sqrt(mean(force_eval.^2));
metrics.support_impulse_Ns = trapz(time_ft_eval, force_eval);
metrics.support_peak_moment_Nm = max(moment_eval);
metrics.support_rms_moment_Nm = sqrt(mean(moment_eval.^2));

% Swing end-effector tracking and terminal target error.
[terminal_error_m, trajectory_rmse_m, trajectory_max_error_m] = getSwingEndEffectorErrors(data, time_vec, swing_duration, swing_limb);
metrics.terminal_error_m = terminal_error_m;
metrics.trajectory_rmse_m = trajectory_rmse_m;
metrics.trajectory_max_error_m = trajectory_max_error_m;

% Joint torque effort.
[total_effort, support_effort, swing_effort] = getJointTorqueEffort(data, time_vec, swing_duration, limb_names, support_limb, swing_limb, joint_names);
metrics.total_effort_N2m2s = total_effort;
metrics.support_effort_N2m2s = support_effort;
metrics.swing_effort_N2m2s = swing_effort;

% Base angular distance.
[base_final_angle_deg, base_max_angle_deg] = getBaseAngularDistance(data, time_vec, swing_duration);
metrics.base_final_angle_deg = base_final_angle_deg;
metrics.base_max_angle_deg = base_max_angle_deg;

end

function [time_valid, res_force, res_moment] = getEndEffectorWrench(data, time_vec, limb)

force_raw = zeros(height(data), 3);
moment_raw = zeros(height(data), 3);

force_raw(:, 1) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_force_x"));
force_raw(:, 2) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_force_y"));
force_raw(:, 3) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_force_z"));
moment_raw(:, 1) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_torque_x"));
moment_raw(:, 2) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_torque_y"));
moment_raw(:, 3) = data.(matlab.lang.makeValidName("x_mlivr_sim_" + limb + "_ee_ft_sensor_wrench_torque_z"));

valid_idx = ~isnan(force_raw(:, 1)) & ~isnan(moment_raw(:, 1));
time_valid = time_vec(valid_idx);
force_valid = force_raw(valid_idx, :);
moment_valid = moment_raw(valid_idx, :);

res_force = vecnorm(force_valid, 2, 2);
res_moment = vecnorm(moment_valid, 2, 2);

end

function [terminal_error_m, trajectory_rmse_m, trajectory_max_error_m] = getSwingEndEffectorErrors(data, time_vec, swing_duration, swing_limb)

% This follows the naming convention used in plotData.m.
target_prefix = "x_tf_world_target_" + swing_limb + "_palm_link_translation_";
planned_prefix = "x_tf_world_planned_" + swing_limb + "_palm_link_translation_";
actual_prefix = "x_tf_world_" + swing_limb + "_gripper_site_translation_";

target_raw = getVector3(data, target_prefix);
planned_raw = getVector3(data, planned_prefix);
actual_raw = getVector3(data, actual_prefix);

valid_idx_planned = ~isnan(planned_raw(:, 1));
time_planned = time_vec(valid_idx_planned);
planned_pos = planned_raw(valid_idx_planned, :);

valid_idx_actual = ~isnan(actual_raw(:, 1));
time_actual = time_vec(valid_idx_actual);
actual_pos = actual_raw(valid_idx_actual, :);

mask_actual = (time_actual >= 0 & time_actual <= swing_duration);
time_eval = time_actual(mask_actual);
actual_eval = actual_pos(mask_actual, :);

if isempty(time_eval)
    error('No valid actual end-effector data within the swing duration.');
end

% Tracking error relative to the planned trajectory during the swing.
time_clamped = min(time_eval, max(time_planned));
planned_interp = interp1(time_planned, planned_pos, time_clamped, 'linear', 'extrap');
error_vec = actual_eval - planned_interp;
error_norm = vecnorm(error_vec, 2, 2);
trajectory_rmse_m = sqrt(mean(error_norm.^2));
trajectory_max_error_m = max(error_norm);

% Terminal error relative to the target pose at t = swing_duration.
valid_idx_target = find(~isnan(target_raw(:, 1)), 1);
if isempty(valid_idx_target)
    error('No target end-effector position was found.');
end
target_pos = target_raw(valid_idx_target, :);

actual_terminal = actual_eval(end, :);
terminal_error_m = norm(target_pos - actual_terminal);

end

function vec = getVector3(data, prefix)
vec = zeros(height(data), 3);
vec(:, 1) = data.(matlab.lang.makeValidName(prefix + "x"));
vec(:, 2) = data.(matlab.lang.makeValidName(prefix + "y"));
vec(:, 3) = data.(matlab.lang.makeValidName(prefix + "z"));
end

function [total_effort, support_effort, swing_effort] = getJointTorqueEffort(data, time_vec, swing_duration, limb_names, support_limb, swing_limb, joint_names)

total_effort = 0;
support_effort = NaN;
swing_effort = NaN;

for limb_id = 1:length(limb_names)
    limb_name = limb_names(limb_id);
    tau_raw = zeros(height(data), length(joint_names));

    for joint_id = 1:length(joint_names)
        joint_name = joint_names(joint_id);
        tau_raw(:, joint_id) = data.(matlab.lang.makeValidName("x_joint_states_" + limb_name + "_" + joint_name + "_effort"));
    end

    valid_idx = ~isnan(tau_raw(:, 1));
    time_valid = time_vec(valid_idx);
    tau_valid = tau_raw(valid_idx, :);

    mask = (time_valid >= 0 & time_valid <= swing_duration);
    time_eval = time_valid(mask);
    tau_eval = tau_valid(mask, :);

    if isempty(time_eval)
        limb_effort = NaN;
    else
        tau_squared_sum = sum(tau_eval.^2, 2);
        limb_effort = trapz(time_eval, tau_squared_sum);
    end

    total_effort = total_effort + limb_effort;

    if limb_name == support_limb
        support_effort = limb_effort;
    end
    if limb_name == swing_limb
        swing_effort = limb_effort;
    end
end

end

function [base_final_angle_deg, base_max_angle_deg] = getBaseAngularDistance(data, time_vec, swing_duration)

base_quat_raw = zeros(height(data), 4);
base_quat_raw(:, 1) = data.(matlab.lang.makeValidName("x_odom_pose_pose_orientation_x"));
base_quat_raw(:, 2) = data.(matlab.lang.makeValidName("x_odom_pose_pose_orientation_y"));
base_quat_raw(:, 3) = data.(matlab.lang.makeValidName("x_odom_pose_pose_orientation_z"));
base_quat_raw(:, 4) = data.(matlab.lang.makeValidName("x_odom_pose_pose_orientation_w"));

valid_idx = ~isnan(base_quat_raw(:, 1));
time_valid = time_vec(valid_idx);
base_quat_valid = base_quat_raw(valid_idx, :);

mask = (time_valid >= 0 & time_valid <= swing_duration);
base_quat_eval = base_quat_valid(mask, :);

if isempty(base_quat_eval)
    error('No valid base orientation data within the swing duration.');
end

q0 = base_quat_eval(1, :);
num_samples = size(base_quat_eval, 1);
angle_rad = zeros(num_samples, 1);

for i = 1:num_samples
    qt = base_quat_eval(i, :);
    dot_prod = abs(sum(q0 .* qt));
    dot_prod = min(1.0, max(-1.0, dot_prod));
    angle_rad(i) = 2.0 * acos(dot_prod);
end

angle_deg = rad2deg(angle_rad);
base_final_angle_deg = angle_deg(end);
base_max_angle_deg = max(angle_deg);

end

function result_row = makeEmptyResultRow(method_name, uncertainty_percent, seed_id, csv_file)

result_row = table( ...
    string(method_name), uncertainty_percent, seed_id, string(csv_file), false, ...
    NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, ...
    'VariableNames', { ...
    'Method', 'UncertaintyPercent', 'Seed', 'CsvFile', 'Success', ...
    'Delay_s', ...
    'SupportPeakForce_N', 'SupportRmsForce_N', 'SupportImpulse_Ns', ...
    'SupportPeakMoment_Nm', 'SupportRmsMoment_Nm', ...
    'TerminalError_m', 'TerminalError_mm', ...
    'TrajectoryRMSE_m', 'TrajectoryRMSE_mm', ...
    'TrajectoryMaxError_m', 'TrajectoryMaxError_mm', ...
    'TotalEffort_N2m2s', 'SupportEffort_N2m2s', 'SwingEffort_N2m2s', ...
    'BaseFinalAngle_deg', 'BaseMaxAngle_deg' ...
    });

end

function summary = summarizeResults(results, method_name, uncertainty_percent, success_rate)

if isempty(results)
    summary = table(string(method_name), uncertainty_percent, 0, success_rate, ...
        'VariableNames', {'Method', 'UncertaintyPercent', 'ValidTrials', 'SuccessRate_percent'});
    return;
end

summary = table( ...
    string(method_name), uncertainty_percent, height(results), success_rate, ...
    mean(results.SupportPeakForce_N, 'omitnan'), std(results.SupportPeakForce_N, 'omitnan'), ...
    mean(results.SupportRmsForce_N, 'omitnan'), std(results.SupportRmsForce_N, 'omitnan'), ...
    mean(results.SupportImpulse_Ns, 'omitnan'), std(results.SupportImpulse_Ns, 'omitnan'), ...
    mean(results.SupportPeakMoment_Nm, 'omitnan'), std(results.SupportPeakMoment_Nm, 'omitnan'), ...
    mean(results.SupportRmsMoment_Nm, 'omitnan'), std(results.SupportRmsMoment_Nm, 'omitnan'), ...
    mean(results.TerminalError_mm, 'omitnan'), std(results.TerminalError_mm, 'omitnan'), ...
    mean(results.TrajectoryRMSE_mm, 'omitnan'), std(results.TrajectoryRMSE_mm, 'omitnan'), ...
    mean(results.TotalEffort_N2m2s, 'omitnan'), std(results.TotalEffort_N2m2s, 'omitnan'), ...
    mean(results.SupportEffort_N2m2s, 'omitnan'), std(results.SupportEffort_N2m2s, 'omitnan'), ...
    mean(results.SwingEffort_N2m2s, 'omitnan'), std(results.SwingEffort_N2m2s, 'omitnan'), ...
    mean(results.BaseFinalAngle_deg, 'omitnan'), std(results.BaseFinalAngle_deg, 'omitnan'), ...
    mean(results.BaseMaxAngle_deg, 'omitnan'), std(results.BaseMaxAngle_deg, 'omitnan'), ...
    'VariableNames', { ...
    'Method', 'UncertaintyPercent', 'ValidTrials', 'SuccessRate_percent', ...
    'PeakForceMean_N', 'PeakForceStd_N', ...
    'RmsForceMean_N', 'RmsForceStd_N', ...
    'ImpulseMean_Ns', 'ImpulseStd_Ns', ...
    'PeakMomentMean_Nm', 'PeakMomentStd_Nm', ...
    'RmsMomentMean_Nm', 'RmsMomentStd_Nm', ...
    'TerminalErrorMean_mm', 'TerminalErrorStd_mm', ...
    'TrajectoryRMSEMean_mm', 'TrajectoryRMSEStd_mm', ...
    'TotalEffortMean_N2m2s', 'TotalEffortStd_N2m2s', ...
    'SupportEffortMean_N2m2s', 'SupportEffortStd_N2m2s', ...
    'SwingEffortMean_N2m2s', 'SwingEffortStd_N2m2s', ...
    'BaseFinalAngleMean_deg', 'BaseFinalAngleStd_deg', ...
    'BaseMaxAngleMean_deg', 'BaseMaxAngleStd_deg' ...
    });

end
