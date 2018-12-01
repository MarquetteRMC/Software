%% Initial Battery Sizing
% Dean Koumoutsos
% Last Updated: 10/22/2018
%
% In order to size the battery at this stage, where there is little
% detail in the mechanical design, the team observes teams who successfully
% scored points in the mining portion and will adjust the times as applicable
% to our designs. 

% To be competitive, the team should complete at least two trips from start
% area, to the dig area, and back. When the team has more data on our
% mechanical design including dig mechanism and hopper, this code will be
% updated.

clc, clear all, close all

% Declare times of each section as observed from other teams.
StartToDigT = [20 40]/3600; % hours
Dig1T = [120 240]/3600; % hours
ReturnT = [20 30]/3600; % hours
DumpT = [20 40]/3600; % hours
Dig2T = [45 180]/3600; % hours

% Sum of min and sum of max
TotalTime = (StartToDigT + Dig1T + ReturnT + DumpT + ReturnT + Dig2T + ReturnT + DumpT)*60 % minutes

% Motor Current Draw
driveA = 27; % Amps
digA = 25; % Amps guessing, this is going to require some collaboration
expelConveyerA = 15; % Amps
dumpConveyerA = 20; % Amps
linearActuatorA = 4; % Amps
computerA = 1; % Amps

% Amp-hours
AHStartToDig = ((2 * driveA)) *StartToDigT; %Ah
AHDig1 = (Dig1T * (expelConveyerA + digA + (2 * (0.8 * driveA)))); %Ah
AHReturn = ReturnT * (2 * driveA); %Ah
AHDump = (((2 * driveA) + dumpConveyerA) * DumpT); %Ah
AHDig2 = (Dig2T * (expelConveyerA + digA + (2 * driveA))); %Ah
AHCompute = TotalTime * computerA; %Ah

AHTotal = (AHStartToDig + AHDig1 + AHReturn + AHDump + AHReturn + AHDig2 + AHReturn + AHDump + AHCompute) % Ah
