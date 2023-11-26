num_switches = 0;  % Initialize num_switches to 0
dns = 0;  % Initialize dns to 0
% Define the means and standard deviations
LCW_HR_means = [80, 65, 61];
LCW_HR_std_devs = [14, 15, 14];

LCW_RR_means = [16, 13, 17];
LCW_RR_std_devs = [6, 4, 8];

HCW_HR_means = [95, 71, 92];
HCW_HR_std_devs = [26, 21, 23];

HCW_RR_means = [21, 14, 26];
HCW_RR_std_devs = [14, 5, 16];

% Define the number of samples
num_samples = 100;

% Calculate the average values for HR and RR based on the specified means and standard deviations
LCW_HR_avg = zeros(1, length(LCW_HR_means));
LCW_RR_avg = zeros(1, length(LCW_RR_means));
HCW_HR_avg = zeros(1, length(HCW_HR_means));
HCW_RR_avg = zeros(1, length(HCW_RR_means));

for i = 1:length(LCW_HR_means)
    sampled_HR_LCW = normrnd(LCW_HR_means(i), LCW_HR_std_devs(i), num_samples, 1);
    sampled_RR_LCW = normrnd(LCW_RR_means(i), LCW_RR_std_devs(i), num_samples, 1);
    sampled_HR_HCW = normrnd(HCW_HR_means(i), HCW_HR_std_devs(i), num_samples, 1);
    sampled_RR_HCW = normrnd(HCW_RR_means(i), HCW_RR_std_devs(i), num_samples, 1);

    LCW_HR_avg(i) = mean(sampled_HR_LCW);
    LCW_RR_avg(i) = mean(sampled_RR_LCW);
    HCW_HR_avg(i) = mean(sampled_HR_HCW);
    HCW_RR_avg(i) = mean(sampled_RR_HCW);
end

% Display the calculated average values
disp('LCW HR Averages:');
disp(LCW_HR_avg);
disp('LCW RR Averages:');
disp(LCW_RR_avg);  
disp('HCW HR Averages:');
disp(HCW_HR_avg);

disp('HCW RR Averages:');
disp(HCW_RR_avg);

    % Calculate the respiratory quotient (Rq) as HR divided by RR
    Rq_LCW(i) = LCW_HR_avg / LCW_RR_avg;
    Rq_HCW(i) = HCW_HR_avg / HCW_RR_avg;

% Calculate the human reaction time (tr)
tr_HCW = 0.01 * average_Rq_HCW;
tr_LCW = 0.01 * average_Rq_LCW;

% Display the average user profile
fprintf('Average Tr under HCW: %.2f\n', tr_HCW);
fprintf('Average Tr under LCW: %.2f\n', tr_LCW);

decellim_low=-150;
decellim_hcw=-200;
for Init_Speed = 20:40
    % Generate a random value for 'Case' (1 or 2)
    Case = randi([1, 2]);
    
    if Case == 1
        decellimit = decellim_low;
        tr = tr_LCW;
        Gain = 25000;
    else
        decellimit = decellim_hcw;
        tr = tr_HCW;
        Gain = 25000;
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
			disp("ta: " + ta + " s");
			disp("tr: " + tr + " s");
			disp("hstop: " + hstop + " s");
			disp("hc (Controller Deceleration Limit): " + hc);
			disp("ha (Human Action Deceleration): " + ha);
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