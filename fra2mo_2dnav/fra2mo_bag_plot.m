
%% Script to accomplish Step 2c of HW04

clear all
close all
clcS

bag = rosbag('bag_2c.bag');

pose = select(bag,'Topic','/fra2mo/pose');
msgStructs = readMessages(pose,'DataFormat','struct');

xPoints = cellfun(@(m) double(m.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Position.Y),msgStructs);
plot(xPoints,yPoints)