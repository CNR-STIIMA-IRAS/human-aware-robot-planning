clc
clear all

bag=rosbag('prova20170321-1exp_2017-03-20-14-27-53.bag')
bagSelection = select(bag, ... 
                     'Time', [bag.StartTime, bag.StartTime + 0.15], ...
                     'Topic','/tf');
%bagSelection.EndTime =  bagSelection.EndTime - bagSelection.StartTime;
%bagSelection.StartTime = 0;
[ts,cols] = timeseries(bagSelection)
ts.plot
