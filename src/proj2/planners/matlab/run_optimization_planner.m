function success = run_optimization_planner()

%% Package Path stuff
%% Put the absolute path to tbx manager here.
%% eg. tbxmanager_dir = '/home/cc/ee106b/sp20/staff/ee106b-taa/ros_workspaces/proj2/tbxmanager';

tbxmanager_dir = ;

disp('Adding tbxmanager')
addpath(tbxmanager_dir)

tbxmanager restorepath

%% Load mat file

disp('Loading input file')

try
    input = load('input.mat');
catch
    success = false;
    disp('Failed to load input file')
%     return
end

%% cast things

start = reshape(double(input.start), [4,1]);
goal = reshape(double(input.goal), [4,1]);
obstacles = double(input.obstacles);
upper_state_bounds = reshape(double(input.upper_state_bounds), [4,1]);
lower_state_bounds = reshape(double(input.lower_state_bounds), [4,1]);
upper_input_bounds = reshape(double(input.upper_input_bounds), [2,1]);
lower_input_bounds = reshape(double(input.lower_input_bounds), [2,1]);
N = double(input.N);
dt = double(input.dt);

%% Running optimization-based planning

disp('Initializing Optimization')

[q_opt, u_opt, exitval_opt] = bicycle_planning( ...
                                start, ...
                                goal, ...
                                obstacles, ...
                                upper_state_bounds, ...
                                lower_state_bounds, ...
                                upper_input_bounds, ...
                                lower_input_bounds, ...
                                N, ...
                                dt);

%% Set success bool

success = ~exitval_opt.problem;
    

%% Save plan to mat file

cur = fileparts(mfilename('fullpath'));
if isempty(cur)
    cur = pwd;
end

save(fullfile(cur, 'output.mat'), 'q_opt', 'u_opt', '-v7'); % Save using a slightly older format so scipy can read


end