clear all
clc
close all
filename = 'Data/Results/data.csv';

T = readtable(filename); %check T.Properties
VariableNames = T.Properties.VariableNames;
Arr = table2array(T);
[m,n] = size(Arr);

for i=2:n
    subplot(4,2,i-1)
    yy = i;
     yy = i;
    plot(Arr(:,1),Arr(:,yy),'r');
     ylabel(cell2mat(VariableNames(yy)))
     xlabel(cell2mat(VariableNames(1)))
end
