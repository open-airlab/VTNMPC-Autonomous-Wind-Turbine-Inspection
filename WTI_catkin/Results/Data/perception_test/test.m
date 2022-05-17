clc
clear all
close all


M1 = dlmread('new_test/weight_heading_high.txt');    %VTMPC data
M2 = dlmread('new_test/old_weight.txt');    %VTMPC data

%plot(M1(3948:50:end,19))
%color_point_traj = [0.4980 0.3250 0.85];
color_NMPC =[0 0.9 0.1];
color_NMPC =[0.9 0.1 0];

color_VTNMPC_nw = [0.9100 0.4100 0.1700];
color_VTNMPC_nw = [0 0 1];

color_PAMPC = [0.1 0.9 0.1];
%color_NMPC_nw = [0.2 0.7 0.2];
color_NMPC_nw= [0.1 0.1 0.1];

%plot(M2(2036:50:end,19))



%M1(3948:3948+497,19)=M1(3948:3948+497,19)-30;
%M2(2036:2036+497,19)=M2(2036:2036+497,19)-30;


%M1(3948+497:3948+1505,19)=M1(3948+497:3948+1505,19)-60;
%M2(2036+497:2036+1505,19)=M2(2036+497:2036+1505,19)-60;





%M1(3948+2004:3948+2496,19)=M1(3948+2004:3948+2496,19)-90;
%M2(2036+2004:2036+2496,19)=M2(2036+2004:2036+2496,19)-90;




%plot(M1(3948:3948+497,19))

hold on

%plot(M2(2036:2036+497,19))

hold on 

%plot([1:1:length(M1(3949:3949+497,19))]'/50,M1(3949:3949+497,19),'LineWidth', 3, 'Color',color_PAMPC)
plot(M1(:,19))
hold on

%plot([1:length(M2(1382:1382+497,19))]/50,M2(1382:1382+497,19),'LineWidth', 3, 'Color',color_VTNMPC_nw)

hold on
yline(30,'LineWidth', 3,'Color',[0 0 0])
leg_han = legend('PAMPC','VT-NMPC','Desired Angle');
hold on 
set(leg_han,'FontSize',10,'Location','northeast','Orientation','horizontal');
xlabel('Time [s]');
ylabel('Error in angle [Deg]');
ax_han = gca;
set(ax_han,'FontSize',10)
%xlim([0 10])
ylim([0 31])
grid minor




