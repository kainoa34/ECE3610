%%%%%%%%%%%%%
% ECE 3610
% LAB 12 -- PID-Based Line Following
%%%%%%%%%%%%%
%%%%%%%%%%%%%
% In this lab, you will be working in teams (ideally your final project 
% teams, but this is not required for this lab) to create a PID feedback loop
% and use it to tune a robot to smoothly follow a black line on a white
% background. Line following will be a core part of your final project, so
% it's good to get some experience with it early!
%
% Deliverables:
%   - Demonstrate that your robot accurately and smoothly follows a 
%     provided line without losing tracking.
%%%%%%%%%%%%%%
%% WIRING
% This lab has lots of wiring! Be sure to check out the wiring diagram on
% Canvas to map all of the wires where they need to go. Double check this
% before connecting the Arduino to power.
%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!
clc
clear all
% for PC:
nb = nanobot('COM3', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');
%% 2. RECORD WHICH ROBOT YOU'RE USING 
% From now on (for the final project), your group will use the same robot 
% for consistency.  Record here which robot you are using for future 
% reference.
% ROBOT = 'R107'
%% 3.  TEST IF ROBOT GOES STRAIGHT (NO LINE FOLLOWING YET)
% First, make sure the battery pack is plugged in and is on.  If the
% battery has sufficient charge, the red lights under the sensing array
% should be on.
%
% There is an emergency shutoff section at the end of the code that you 
% can use if needed.
%
% Your motors may not turn at the same rate when fed the same duty
% cycle, meaning your robot will constantly drift to one side, which can
% make tuning difficult. To combat this, find a factor mOffScale that 
% roughly makes your robot go in a straight line. Apply this value later 
% to the control signal of the stronger/weaker motor to even out the speed 
% difference. (Make sure you use switch out the LiPo battery when it starts
% to get low, so that your values and tuning aren't impacted by a low
% battery.)
% At first, the best way to see if the wheels are moving at the same speed 
% might be to pick the robot up and watch the wheels.
mOffScale = 1; % Start with 1 so both motors have same duty cycle.
% The base motor duty cycle (speed) you wish to travel. 
% (recommended values are 9 or 10)
motorBaseSpeed = 10;
% Set the duty cycle of each motor
m1Duty = mOffScale * motorBaseSpeed;
m2Duty = motorBaseSpeed;
tic % start the time
% It can be helpful to initialize your motors to a higher duty cycle
% for a very brief moment (here 0.03 seconds), just to overcome the 
% gearbox force of static friction so that lower duty cycles don't stall 
% out at the start.
% (recommendation: ~10, with mOffScale if needed)
nb.setMotor(1, mOffScale * 10);
nb.setMotor(2, 10);
pause(0.03);
while (toc < 3) % adjust the time to test if robot goes in straight line
                % (shorter time saves battery; longer tests longer path)
    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);
end
% Turn off the motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);
%% 4.  CALIBRATE THE SENSOR VALUES 
% The values provided by the sensor array at the front of the robot will 
% vary depending on lighting conditions even though it uses IR and each
% sensor has its own source.  For example, if the projector is on in the 
% room, it could change the performance of your line sensing.  As a result, 
% its a good idea to use calibrated sensor values to account for the 
% current conditions.  In the next two sections, we will record the max 
% and min values.
%% MIN REFLECTANCE VALUE CALIBRATION (white background)
% Remember, if the red LEDs on the bottom of the array do not light up and 
% stay lit, you will not get reliable readings.  If the red lights do not 
% light up:
% -- Double check your wiring.
% -- The LiPo battery may be too low. 
% -- Double check that that two VUSB pads are soldered together on the 
%    underside of the Arduino.
% -- You can also see if connecting or disconnecting the +5V on the 
%    JP1-2 helps. 
% Put the sensor array over a white background and find the expected min
% reflectance array values.
% Initialize the reflectance array.
nb.initReflectance();
%Average a few values 
avgVals = zeros(10, 6);
for i = 1:10
    read = nb.reflectanceRead();
    avgVals(i, 1) = read.one;
    avgVals(i, 2) = read.two;
    avgVals(i, 3) = read.three;
    avgVals(i, 4) = read.four;
    avgVals(i, 5) = read.five;
    avgVals(i, 6) = read.six;
end
minVals = [mean(avgVals(:,1)), mean(avgVals(:,2)), mean(avgVals(:,3)), ...
    mean(avgVals(:,4)), mean(avgVals(:,5)), mean(avgVals(:,6))];
fprintf(['Min Reflectance - one: %.2f, two: %.2f, three: %.2f four:' ...
    '%.2f five: %.2f six: %.2f\n'], ...
    minVals(1), minVals(2), minVals(3), minVals(4), minVals(5), minVals(6));
minReflectance = [326.8,246.8,216.5,216.5,251.6,334.2]; % Set me to min reflectance 
                                             % values for each sensor for
                                             % future reference
%% MAX REFLECTANCE VALUE CALIBRATION (all sensors over black tape)
% Put the sensor array over a black background and find the expected max
% reflectance array values.
% Initialize the reflectance array.
nb.initReflectance();
%Average a few values
avgVals = zeros(10, 6);
for i = 1:10
    read = nb.reflectanceRead();
    avgVals(i, 1) = read.one;
    avgVals(i, 2) = read.two;
    avgVals(i, 3) = read.three;
    avgVals(i, 4) = read.four;
    avgVals(i, 5) = read.five;
    avgVals(i, 6) = read.six;
end
maxVals = [mean(avgVals(:,1)), mean(avgVals(:,2)), mean(avgVals(:,3)), ...
    mean(avgVals(:,4)), mean(avgVals(:,5)), mean(avgVals(:,6))];
fprintf(['Max Reflectance - one: %.2f, two: %.2f, three: %.2f '...
    'four: %.2f five: %.2f six: %.2f\n'], ...
    maxVals(1), maxVals(2), maxVals(3), maxVals(4), maxVals(5), maxVals(6));
maxReflectance = [1552,1734,1689,1712,1796,1445]; % Set me to max reflectance 
                                             % values for each sensor for
                                             % future reference
%% 5.  LINE FOLLOWING PID LOOP  (I-line follow -> 1st T turn 180 -> line reacquire -> 2nd T stop)
%% 5. LINE FOLLOWING PID LOOP
% Path behavior:
%   1) Follow a straight/curved black line using PID
%   2) When the robot reaches the FIRST T-intersection, rotate in place
%      until the center sensors see the line again, then continue forward
%   3) When the robot reaches the SECOND T-intersection, stop
%
% Extra robustness:
%   - If the robot briefly loses the line at a corner, do NOT stop instantly
%   - Only declare "lost line" after several consecutive bad readings
%   - When line is lost, search in the OPPOSITE direction of the last turn
%     because the robot likely overshot the corner

minVals = [326.8,246.8,216.5,216.5,251.6,334.2];
maxVals = [1552,1734,1689,1712,1796,1445];

nb.initReflectance();

% PID gains
kp = 1.5;
ki = 0.0;
kd = 0.2;

% PID memory
prevError = 0;
prevTime  = 0;
integral  = 0;

% Motor settings
mOffScale = 1;
motorBaseSpeed = 11;

% T-intersection detection settings
tCount = 0;                  % Number of T-intersections detected so far
turnDoneThisT = false;       % Prevent double-trigger while still on same T
ignoreTUntilClear = false;   % After a T event, wait until sensors leave T area

turnSpeed = 9.0;             % Speed used for in-place turning at the first T
settleTime = 0.10;           % Small forward settle after reacquiring the line

tDetectThresh = 0.55;        % Sensor value considered "black" for T detection
tSeenCount = 0;              % Consecutive frames that look like a T
tSeenNeeded = 2;             % Require multiple frames to reduce false triggers

minTurnTime = 0.45;          % Minimum turn time before allowing line reacquire
maxTurnTime = 3.0;           % Safety timeout for the first-T turn
lineSeenThresh = 0.45;       % Center sensor threshold for line reacquire

% Lost-line recovery settings
% Instead of stopping immediately when the line disappears, wait a few loops.
% If the line is still missing, perform a short recovery search.
linePresentThresh = 0.15;    % Very small total reflectance means "possibly lost"
lostLineCount = 0;           % Counts consecutive low-line readings
lostLineNeeded = 4;          % Need this many consecutive misses before recovery

searchMaxTime = 0.35;        % Max time for short recovery search
searchTurnSpeed = 8.0;       % Turn speed during recovery search
searchSeenThresh = 0.45;     % Threshold for center sensors to confirm line
searchSeenCount = 0;         % Consecutive confirmation frames during recovery
searchSeenNeeded = 2;        % Require multiple frames to confirm recovery

% Last turn direction memory
% +1 = robot was turning right most recently
% -1 = robot was turning left most recently
%  0 = unknown / not established yet
%
% NOTE:
% Depending on your motor wiring and inversion, you may need to swap the
% meaning of +1 and -1 if the recovery search turns the wrong way.
lastTurnDir = 0;

tic

% Small startup kick to overcome static friction
nb.setMotor(1, mOffScale * 10);
nb.setMotor(2, 10);
pause(0.03);

while (toc < 25)
    
    % Compute loop time step for PID
    dt = toc - prevTime;
    prevTime = toc;
    if dt <= 0
        dt = 0.001;
    end

   % Read reflectance sensors
    valsStruct = nb.reflectanceRead();
    vals = [valsStruct.one, valsStruct.two, valsStruct.three, ...
            valsStruct.four, valsStruct.five, valsStruct.six];

    % Normalize raw sensor values to the range [0, 1]
    calibratedVals = zeros(1,6);
    for i = 1:6
        denom = (maxVals(i) - minVals(i));

        if abs(denom) < 1e-6
            calibratedVals(i) = 0;
        else
            calibratedVals(i) = (vals(i) - minVals(i)) / denom;
        end

        calibratedVals(i) = max(0, min(1, calibratedVals(i)));
    end

    % ===================== T-INTERSECTION DETECTION =====================
    % A T is detected when center sensors and side sensors see black
    % at the same time, and enough total sensors are active.
    numBlack = sum(calibratedVals > tDetectThresh);

    centerSeen = (calibratedVals(3) > tDetectThresh) || ...
                 (calibratedVals(4) > tDetectThresh);

    sideSeen = (calibratedVals(1) > tDetectThresh) || ...
               (calibratedVals(2) > tDetectThresh) || ...
               (calibratedVals(5) > tDetectThresh) || ...
               (calibratedVals(6) > tDetectThresh);

    rawT = centerSeen && sideSeen && (numBlack >= 4);

    % After handling a T, do not allow another T until the robot leaves
    % the dark region enough to clearly separate intersections.
    if ignoreTUntilClear
        if numBlack <= 2
            ignoreTUntilClear = false;
        end
    end

    % Require T-like readings for multiple consecutive frames
    if rawT
        tSeenCount = tSeenCount + 1;
    else
        tSeenCount = 0;
    end

    isT = (tSeenCount >= tSeenNeeded);

    % ===================== HANDLE T-INTERSECTION =====================
    if isT && ~turnDoneThisT && ~ignoreTUntilClear
        tCount = tCount + 1;
        turnDoneThisT = true;
        tSeenCount = 0;

        % SECOND T -> stop permanently
        if tCount >= 2
            nb.setMotor(1, 0);
            nb.setMotor(2, 0);
            break;
        end

        % FIRST T -> rotate until the line is found again
        turnStart = tic;
        foundLine = false;

        % This command direction defines the turn direction used at the first T.
        % If the robot turns the wrong way here, swap the motor signs.
        lastTurnDir = 1;   % assumed "right turn" for recovery logic

        while toc(turnStart) < maxTurnTime

            % In-place rotation
            nb.setMotor(1,  mOffScale * turnSpeed);
            nb.setMotor(2, -turnSpeed);

            % Read sensors during turning
            v = nb.reflectanceRead();
            rawTurn = [v.one, v.two, v.three, v.four, v.five, v.six];

            calTurn = zeros(1,6);
            for k = 1:6
                denom = (maxVals(k) - minVals(k));

                if abs(denom) < 1e-6
                    calTurn(k) = 0;
                else
                    calTurn(k) = (rawTurn(k) - minVals(k)) / denom;
                end

                calTurn(k) = max(0, min(1, calTurn(k)));
            end

            % We say the line is reacquired when the center sees black,
            % but only after some minimum turning time so we do not stop
            % immediately while still sitting on the T region.
            centerLineSeen = (calTurn(3) > lineSeenThresh) || ...
                             (calTurn(4) > lineSeenThresh);

            % If too many sensors still see black, we are probably still
            % in the T area, not back on the single line yet.
            notBigT = sum(calTurn > tDetectThresh) <= 3;

            if toc(turnStart) > minTurnTime && centerLineSeen && notBigT
                foundLine = true;
                break;
            end
        end

        % Stop turning
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
        pause(0.03);

        % Continue forward whether line was found or timeout occurred.
        % If you want stricter behavior, you could stop when foundLine == false.
        nb.setMotor(1, mOffScale * motorBaseSpeed);
        nb.setMotor(2, motorBaseSpeed);
        pause(settleTime);

        % Reset PID memory after the turn
        prevError = 0;
        integral = 0;
        prevTime = toc;
        lostLineCount = 0;

        % Prevent immediate re-count of the same T
        ignoreTUntilClear = true;

        continue;
    end

    % Re-arm T detection after the robot clearly leaves the current T region
    if ~rawT
        turnDoneThisT = false;
    end

    % ===================== LOST-LINE CHECK =====================
    % If the line disappears for just one frame, do nothing yet.
    % This helps when the robot briefly loses the line on corners.
    if sum(calibratedVals) < linePresentThresh
        lostLineCount = lostLineCount + 1;
    else
        lostLineCount = 0;
    end

    % ===================== LOST-LINE RECOVERY =====================
    if lostLineCount >= lostLineNeeded

        % Search in the opposite direction of the most recent turn.
        % Idea: if the robot overshot a corner while turning one way,
        % a short turn in the opposite direction often reacquires the line.
        searchDir = -lastTurnDir;

        % If direction is still unknown, choose a default direction
        if searchDir == 0
            searchDir = 1;
        end

        searchStart = tic;
        recovered = false;
        searchSeenCount = 0;

        while toc(searchStart) < searchMaxTime

            if searchDir > 0
                % Turn one way
                nb.setMotor(1,  mOffScale * searchTurnSpeed);
                nb.setMotor(2, -searchTurnSpeed);
            else
                % Turn the opposite way
                nb.setMotor(1, -mOffScale * searchTurnSpeed);
                nb.setMotor(2,  searchTurnSpeed);
            end

            % Read sensors during recovery search
            v = nb.reflectanceRead();
            rawSearch = [v.one, v.two, v.three, v.four, v.five, v.six];

            calSearch = zeros(1,6);
            for k = 1:6
                denom = (maxVals(k) - minVals(k));

                if abs(denom) < 1e-6
                    calSearch(k) = 0;
                else
                    calSearch(k) = (rawSearch(k) - minVals(k)) / denom;
                end

                calSearch(k) = max(0, min(1, calSearch(k)));
            end

            % Use center sensors to confirm that the line is really found again
            centerRecovered = (calSearch(3) > searchSeenThresh) || ...
                              (calSearch(4) > searchSeenThresh);

            if centerRecovered
                searchSeenCount = searchSeenCount + 1;
            else
                searchSeenCount = 0;
            end

            if searchSeenCount >= searchSeenNeeded
                recovered = true;
                break;
            end
        end

        % Stop after recovery attempt
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
        pause(0.03);

        if recovered
            % Resume forward motion and reset PID memory
            nb.setMotor(1, mOffScale * motorBaseSpeed);
            nb.setMotor(2, motorBaseSpeed);
            pause(0.05);

            prevError = 0;
            integral = 0;
            prevTime = toc;
            lostLineCount = 0;

            continue;
        else
            % If recovery fails, stop the program safely
            break;
        end
    end

    % ===================== PID LINE FOLLOWING =====================
    % Weighted error:
    % Left sensors are negative, right sensors are positive.
    % Centered line -> error near 0.
    error = sum([-3 -2 -1 1 2 3] .* calibratedVals) / (sum(calibratedVals) + eps);

    integral = integral + error * dt;
    derivative = (error - prevError) / dt;

    control = (kp * error) + (ki * integral) + (kd * derivative);

    % Store most recent turning direction for future lost-line recovery.
    % IMPORTANT:
    % Depending on your robot wiring, you may need to swap these meanings.
    if control > 0
        lastTurnDir = 1;
    elseif control < 0
        lastTurnDir = -1;
    end

    m1Duty = mOffScale * (motorBaseSpeed + control);
    m2Duty =            (motorBaseSpeed - control);

    % Limit motor commands to safe values
    m1Duty = max(min(m1Duty, 15), -15);
    m2Duty = max(min(m2Duty, 15), -15);

    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);

    prevError = error;
end

% Make sure motors are off when loop ends
nb.setMotor(1, 0);
nb.setMotor(2, 0);
%% EMERGENCY MOTOR SHUT OFF
% If this section doesn't turn off the motors, turn off the power switch 
% on your motor carrier board.
% Clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);
%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.
clc
delete(nb);
clear('nb');
clear all