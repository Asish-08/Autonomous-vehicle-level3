% Parameters
initial_speed_min = 20;
initial_speed_max = 60;
noramlGain = 25000;
poorGain = 25000;
normalDecelLim = -200;
poorDecelLim = -150;
numSteps = 100;
P = [0.6 0.4; 0.85 0.15];
mc = dtmc(P);
roadConditions = simulate(mc, numSteps); % 1 is Normal, 2 is Poor
% Display the generated road condition sequence
%disp("Generated Road Condition Sequence:")
%disp(roadConditionSequence)

% Road Condition Constants
LCW_meanHRU3 = 61;
LCW_meanRRU3 = 17;
LCW_stdHRU3 = 14;
LCW_stdRRU3 = 8;

HCW_meanHRU3 = 92;
HCW_meanRRU3 = 26;
HCW_stdHRU3 = 23;
HCW_stdRRU3 = 16;

f = 18;

initial_speeds = zeros(1, numSteps);
reaction_times = zeros(1, numSteps);
collisions = zeros(1, numSteps);
switches = zeros(1, numSteps);

for i = 1:numSteps
    % Generate initial speeds and reaction times
    initial_speeds(i) = initial_speed_min + (initial_speed_max - initial_speed_min) * rand();
    if roadConditions(i) == 1
        HR = normrnd(LCW_meanHRU3, LCW_stdHRU3);
        RR = normrnd(LCW_meanRRU3, LCW_stdRRU3);
    else
        HR = normrnd(HCW_meanHRU3, HCW_stdHRU3);
        RR = normrnd(HCW_meanRRU3, HCW_stdRRU3);
    end
    reaction_times(i) = 0.01 * (HR / RR);

    % Control parameters based on road conditions
    if roadConditions(i) == 1
        Gain = noramlGain;
        decelLim = normalDecelLim;
        tr = HCW_reaction_time;
    else
        Gain = poorGain;
        decelLim = poorDecelLim;
        tr = LCW_reaction_time;
    end

    % Design control with a random value
    [A, B, C, D, Kess, Kr, Ke, uD] = designControl(secureRand(), Gain);

    % Simulate the LaneMaintainSystem model
    open_system('LaneMaintainSystem.slx')
    set_param('LaneMaintainSystem/VehicleKinematics/Saturation', 'LowerLimit', num2str(decelLim))
    set_param('LaneMaintainSystem/VehicleKinematics/vx', 'InitialCondition', num2str(initial_speeds(i)))
    simModel = sim('LaneMaintainSystem.slx');

    if max(simModel.sx1.Data) < 0
        collisions(i) = 0;
    else
        % Simulate the HumanActionModel model
        open_system('HumanActionModel.slx')
        set_param('HumanActionModel/Step', 'Time', num2str(tr * f))
        set_param('HumanActionModel/Step', 'After', num2str(1.1 * decelLim))
        set_param('HumanActionModel/VehicleKinematics/vx', 'InitialCondition', num2str(initial_speeds(i)))
        simModel1 = sim('HumanActionModel.slx');

        tc = simModel.sx1.Time(end);
        hstop = simModel1.sx1.Time(end);
        if hstop < tc
            switches(i) = 1;

            % Simulate the HumanActionModel model for reaction time
            open_system('HumanActionModel.slx')
            set_param('HumanActionModel/Step', 'Time', num2str(reaction_times(i)))
            set_param('HumanActionModel/Step', 'After', num2str(1.1 * decelLim))
            set_param('HumanActionModel/VehicleKinematics/vx', 'InitialCondition', num2str(initial_speeds(i)))
            simModel2 = sim('HumanActionModel.slx');

            if max(simModel2.sx1.Data) > 0
                collisions(i) = 1;
            end
        else
            collisions(i) = 0;
        end
    end
end

% Display results
disp("Switches: " + sum(switches(:) == 1));
disp("Collisions: " + sum(collisions(:) == 1));
disp("Sampled Initial Speeds (in kmph):")
disp(initial_speeds)

disp("Sampled HR for User 3:")
disp(HR)

disp("Sampled RR for User 3:")
disp(RR)

disp("Sampled Reaction Times (s):")
disp(reaction_times)

disp("Collision Status:")
disp(collisions)

