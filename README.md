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
%% 5. LINE FOLLOWING PID LOOP (simplified)

% ---------- Calibration ----------
minVals = [326.8,246.8,216.5,216.5,251.6,334.2];
maxVals = [1552,1734,1689,1712,1796,1445];

nb.initReflectance();

% ---------- PID ----------
kp = 1.5;
ki = 0.0;
kd = 0.2;

% ---------- Motion ----------
mOffScale = 1;
motorBaseSpeed = 11;
maxMotorDuty = 15;

% ---------- T detection ----------
tDetectThresh  = 0.55;
tSeenNeeded    = 2;
turnSpeed      = 9.0;
minTurnTime    = 0.45;
maxTurnTime    = 3.0;
lineSeenThresh = 0.45;
settleTime     = 0.10;

% ---------- Lost-line recovery ----------
linePresentThresh = 0.15;
lostLineNeeded    = 4;
searchTurnSpeed   = 8.0;
searchMaxTime     = 0.35;
searchSeenThresh  = 0.45;
searchSeenNeeded  = 2;

% ---------- State ----------
prevError = 0;
integral = 0;
prevTime = 0;

tCount = 0;
tSeenCount = 0;
turnDoneThisT = false;
ignoreTUntilClear = false;

lostLineCount = 0;
lastTurnDir = 0;

tic

% Startup kick
driveForward(nb, mOffScale * 10, 10, 0.03);

while toc < 25

    % Time step
    dt = toc - prevTime;
    prevTime = toc;
    if dt <= 0
        dt = 0.001;
    end

    % Read sensors
    cal = readLineSensors(nb, minVals, maxVals);

    % Check T-intersection
    [rawT, isT, tSeenCount, ignoreTUntilClear] = checkT( ...
        cal, tDetectThresh, tSeenNeeded, tSeenCount, ignoreTUntilClear);

    if isT && ~turnDoneThisT && ~ignoreTUntilClear
        tCount = tCount + 1;
        turnDoneThisT = true;
        tSeenCount = 0;

        % Stop at second T
        if tCount >= 2
            stopRobot(nb);
            break;
        end

        % First T: turn and reacquire line
        [~, lastTurnDir] = turnAtFirstT( ...
            nb, minVals, maxVals, mOffScale, turnSpeed, ...
            minTurnTime, maxTurnTime, lineSeenThresh, tDetectThresh);

        stopRobot(nb);
        driveForward(nb, mOffScale * motorBaseSpeed, motorBaseSpeed, settleTime);

        prevError = 0;
        integral = 0;
        prevTime = toc;
        lostLineCount = 0;
        ignoreTUntilClear = true;
        continue;
    end

    if ~rawT
        turnDoneThisT = false;
    end

    % Lost-line counter
    if sum(cal) < linePresentThresh
        lostLineCount = lostLineCount + 1;
    else
        lostLineCount = 0;
    end

    % Recovery if line was lost for several frames
    if lostLineCount >= lostLineNeeded
        [recovered, lastTurnDir] = recoverLine( ...
            nb, minVals, maxVals, mOffScale, lastTurnDir, ...
            searchTurnSpeed, searchMaxTime, searchSeenThresh, searchSeenNeeded);

        stopRobot(nb);

        if recovered
            driveForward(nb, mOffScale * motorBaseSpeed, motorBaseSpeed, 0.05);
            prevError = 0;
            integral = 0;
            prevTime = toc;
            lostLineCount = 0;
            continue;
        else
            break;
        end
    end

    % PID follow
    [control, error, integral, lastTurnDir] = pidLineFollow( ...
        cal, kp, ki, kd, dt, prevError, integral, lastTurnDir);

    [m1Duty, m2Duty] = motorCommand(motorBaseSpeed, control, mOffScale, maxMotorDuty);
    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);

    prevError = error;
end

stopRobot(nb);
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


%% Sensor Reading and Calibration
function cal = readLineSensors(nb, minVals, maxVals)
% Read six reflectance sensors and scale them to [0,1].

    s = nb.reflectanceRead();
    raw = [s.one, s.two, s.three, s.four, s.five, s.six];
    cal = zeros(1,6);

    for i = 1:6
        denom = maxVals(i) - minVals(i);
        if abs(denom) < 1e-6
            cal(i) = 0;
        else
            cal(i) = (raw(i) - minVals(i)) / denom;
        end
        cal(i) = max(0, min(1, cal(i)));
    end
end


%% T-Intersection Detection
function [rawT, isT, tSeenCount, ignoreTUntilClear] = checkT( ...
    cal, tDetectThresh, tSeenNeeded, tSeenCount, ignoreTUntilClear)
% Detect whether the robot is on a T-intersection.

    numBlack = sum(cal > tDetectThresh);
    centerSeen = (cal(3) > tDetectThresh) || (cal(4) > tDetectThresh);
    sideSeen = (cal(1) > tDetectThresh) || (cal(2) > tDetectThresh) || ...
               (cal(5) > tDetectThresh) || (cal(6) > tDetectThresh);

    rawT = centerSeen && sideSeen && (numBlack >= 4);

    if ignoreTUntilClear && numBlack <= 2
        ignoreTUntilClear = false;
    end

    if rawT
        tSeenCount = tSeenCount + 1;
    else
        tSeenCount = 0;
    end

    isT = (tSeenCount >= tSeenNeeded);
end


%% First T Turn and Line Reacquisition
function [foundLine, lastTurnDir] = turnAtFirstT( ...
    nb, minVals, maxVals, mOffScale, turnSpeed, ...
    minTurnTime, maxTurnTime, lineSeenThresh, tDetectThresh)
% First T behavior: rotate until the center sensors see the line again.

    startTime = tic;
    foundLine = false;
    lastTurnDir = 1;

    while toc(startTime) < maxTurnTime
        nb.setMotor(1,  mOffScale * turnSpeed);
        nb.setMotor(2, -turnSpeed);

        cal = readLineSensors(nb, minVals, maxVals);

        centerSeen = (cal(3) > lineSeenThresh) || (cal(4) > lineSeenThresh);
        notBigT = sum(cal > tDetectThresh) <= 3;

        if toc(startTime) > minTurnTime && centerSeen && notBigT
            foundLine = true;
            break;
        end
    end
end


%% Lost-Line Recovery
function [recovered, searchDir] = recoverLine( ...
    nb, minVals, maxVals, mOffScale, lastTurnDir, ...
    searchTurnSpeed, searchMaxTime, searchSeenThresh, searchSeenNeeded)
% Recover the line by turning opposite to the last turn direction.

    searchDir = -lastTurnDir;
    if searchDir == 0
        searchDir = 1;
    end

    startTime = tic;
    recovered = false;
    seenCount = 0;

    while toc(startTime) < searchMaxTime
        if searchDir > 0
            nb.setMotor(1,  mOffScale * searchTurnSpeed);
            nb.setMotor(2, -searchTurnSpeed);
        else
            nb.setMotor(1, -mOffScale * searchTurnSpeed);
            nb.setMotor(2,  searchTurnSpeed);
        end

        cal = readLineSensors(nb, minVals, maxVals);
        centerSeen = (cal(3) > searchSeenThresh) || (cal(4) > searchSeenThresh);

        if centerSeen
            seenCount = seenCount + 1;
        else
            seenCount = 0;
        end

        if seenCount >= searchSeenNeeded
            recovered = true;
            break;
        end
    end
end


%% PID Line Following
function [control, error, integral, lastTurnDir] = pidLineFollow( ...
    cal, kp, ki, kd, dt, prevError, integral, lastTurnDir)
% Compute PID control from the line sensor values.

    weights = [-3 -2 -1 1 2 3];
    error = sum(weights .* cal) / (sum(cal) + eps);

    integral = integral + error * dt;
    derivative = (error - prevError) / dt;

    control = kp * error + ki * integral + kd * derivative;

    if control > 0
        lastTurnDir = 1;
    elseif control < 0
        lastTurnDir = -1;
    end
end


%% Motor Command Calculation
function [m1Duty, m2Duty] = motorCommand(baseSpeed, control, mOffScale, maxMotorDuty)
% Convert control effort into left/right motor commands.

    m1Duty = mOffScale * (baseSpeed + control);
    m2Duty =             (baseSpeed - control);

    m1Duty = max(min(m1Duty, maxMotorDuty), -maxMotorDuty);
    m2Duty = max(min(m2Duty, maxMotorDuty), -maxMotorDuty);
end


%% Basic Motor Helpers
function driveForward(nb, m1Duty, m2Duty, pauseTime)
% Drive forward for a short time, then return to caller.

    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);
    pause(pauseTime);
end

function stopRobot(nb)
% Stop both motors.

    nb.setMotor(1, 0);
    nb.setMotor(2, 0);
