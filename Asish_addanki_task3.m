num_switches = 0;  % Initialize num_switches to 0
dns = 0;  % Initialize dns to 0
% Define the mean and standard deviation values for LCW and HCW
% Define the mean and standard deviation values for LCW and HCW
mean_LCW_HR = 61;
std_dev_LCW_HR = 14;
mean_HCW_HR = 92;
std_dev_HCW_HR = 23;

mean_LCW_RR = 17;
std_dev_LCW_RR = 8;
mean_HCW_RR = 26;
std_dev_HCW_RR = 16;

% Number of samples
num_samples = 100; % You can adjust the number of samples as needed

% Initialize arrays to store sampled HR and RR values
sampled_HR_LCW = normrnd(mean_LCW_HR, std_dev_LCW_HR, num_samples, 1);
sampled_HR_HCW = normrnd(mean_HCW_HR, std_dev_HCW_HR, num_samples, 1);
sampled_RR_LCW = normrnd(mean_LCW_RR, std_dev_LCW_RR, num_samples, 1);
sampled_RR_HCW = normrnd(mean_HCW_RR, std_dev_HCW_RR, num_samples, 1);

% Calculate the average HR and RR values for LCW and HCW
avg_LCW_HR = mean(sampled_HR_LCW);
avg_HCW_HR = mean(sampled_HR_HCW);
avg_LCW_RR = mean(sampled_RR_LCW);
avg_HCW_RR = mean(sampled_RR_HCW);

% Calculate the human reaction time (tr)
tr_HCW = 0.01*(avg_HCW_HR/avg_HCW_RR)*5;
tr_LCW = 0.01*(avg_LCW_HR/avg_LCW_RR)*5;

decellim_low=-150;
decellim_hcw=-200;
for Init_Speed = 20:60
    % Generate a random value for 'Case' (1 or 2)
    Case = randi([1, 2]);
    
    if Case == 1
        decellimit = decellim_low;
        tr = tr_LCW;
        Gain = 790000;
    else
        decellimit = decellim_hcw;
        tr = tr_HCW;
        Gain = 800000;
    end
    
    % Design control with a random value
    [A, B, C, D, Kess, Kr, Ke, uD] = designControl(secureRand(), Gain);
    
    % Load the system model
    load_system('LaneMaintainSystem.slx');
    
    % Set parameters for the LaneMaintainSystem model
    set_param('LaneMaintainSystem/VehicleKinematics/Saturation', 'LowerLimit', num2str(decellimit));
    set_param('LaneMaintainSystem/VehicleKinematics/vx', 'InitialCondition', num2str(Init_Speed));
    
    % Simulate the LaneMaintainSystem model
    simModel = sim('LaneMaintainSystem.slx');
    
    if max(simModel.sx1.data) < 0
        disp("Do Not Switch: No Collision. Gain= " + Gain + " Initial Speed: " + Init_Speed);
        dns = dns + 1;
    else
	tc = max(double(simModel.sx1.Time));
	% Load the HumanActionModel model
			load_system('HumanActionModel.slx');
			% Set parameters for the HumanActionModel model
			set_param('HumanActionModel/VehicleKinematics/Saturation', 'LowerLimit', num2str(decellimit));
			set_param('HumanActionModel/VehicleKinematics/vx', 'InitialCondition', num2str(Init_Speed));
			set_param('HumanActionModel/Step', 'Time', num2str(tr));
			set_param('HumanActionModel/Step', 'After', num2str(1.1 * decellimit));
			set_param('HumanActionModel/VehicleKinematics/Saturation', 'LowerLimit', num2str(1.1 * decellimit));
			
			% Simulate the HumanActionModel model
			simModel2 = sim('HumanActionModel.slx');
			
			% Calculate hc (deceleration limit of the controller)
			hc = -decellimit;  % In this example, it's assumed to be the negative of the deceleration limit
			
			% Calculate ha (human action deceleration)
			ha = 1.1 * hc;
			
			% Calculate ta (action time) using a static deceleration
			simModel_action = sim('HumanActionModel.slx');  % Simulate without the controller
			ta = max(simModel_action.simout.time);  % Maximum time to stop with human action
			% Calculate hstop (total stopping time)
			hstop = tr + ta;
		if hstop < tc
            num_switches = num_switches + 1; % Increment the switch counter	
    		%switch=seitch+1
			% Display results
			disp("Switch to Human. Gain= " + Gain + " Initial speed= " + Init_Speed);
		else
            disp("Do Not switch: Collision. Gain= " + Gain + " Initial speed= " + Init_Speed);
        	%collison=collison+1 
		end
    end
end

% Display the number of switches required to avoid collision
fprintf('Number of Switches to Avoid Collision: %d\n', num_switches);
% if collsion>1; collsion status = yes. else no.
% disp(switch)  