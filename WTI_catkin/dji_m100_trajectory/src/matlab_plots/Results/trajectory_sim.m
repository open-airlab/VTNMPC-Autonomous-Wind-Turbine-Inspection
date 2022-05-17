close all;
clear;
clc;
addpath(genpath([pwd,'/../Exp_data/']));

save_dest = [pwd,'/../../../Figures/Trajectory_sim/'];

wind_num = 3;
file_name_post = ['_Wind',num2str(wind_num),'-XY-3N-10sec_Noise_Gauss-03_sim'];

file_name_NMPC = ['WS_NMPC',file_name_post];
file_name_NMHE_NMPC = ['WS_NMHE-NMPC',file_name_post];
file_name_GP_NMPC = ['WS_GP-NMPC',file_name_post];

color_NMPC = [0 0.4470 0.7410];
color_NMHE_NMPC = [0.5 0 0.5];
color_GP_NMPC = [0.85 0.3250 0.0980];

skip = 4;

fig_han = figure('name','Position 3D','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734]);
% Create axes
axes1 = axes('Parent',fig_han,...
    'Position',[0.120393013100438 0.111280831210491 0.823908296943232 0.878182479175905]);
hold(axes1,'on');
load(file_name_NMPC);
plot3(trajectory_des_m(1:skip:end,1), trajectory_des_m(1:skip:end,2), trajectory_des_m(1:skip:end,3),'k','LineWidth',3);
hold on
plot3(position_m(1:skip:end,1), position_m(1:skip:end,2), position_m(1:skip:end,3),'LineWidth',1.5,'color',color_NMPC);
load(file_name_NMHE_NMPC);
plot3(position_m(1:skip:end,1), position_m(1:skip:end,2), position_m(1:skip:end,3),'LineWidth',1.5,'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
plot3(position_m(1:skip:end,1), position_m(1:skip:end,2), position_m(1:skip:end,3),'LineWidth',1.5,'color',color_GP_NMPC);
xlabel('{\it x}-axis (m)');
ylabel(['{\it y}-axis';'         (m)']);
zlabel('{\it z}-axis (m)');
% xlim([-0.05 0.05]);
% ylim([-0.05 0.05]);
zlim([1.7 3.9]);
ax_han = gca;
set(ax_han,'FontSize',35)
leg_han = legend('Ref.','NMPC','NMHE-NMPC','GP-NMPC');
set(leg_han,'FontSize',32,'Location','northeast','Orientation','horizontal');
view(axes1,[-13.0836105158727 31.8655476563905]);
vec_pos = get(get(gca, 'XLabel'), 'Position');
set(get(gca, 'XLabel'), 'Position', vec_pos + [-0.4 0.15 0.1]);
vec_pos = get(get(gca, 'YLabel'), 'Position');
set(get(gca, 'YLabel'), 'Position', vec_pos + [1.7 0.7 0]);
vec_pos = get(get(gca, 'ZLabel'), 'Position');
set(get(gca, 'ZLabel'), 'Position', vec_pos + [0.5 0 0]);
grid on
saveas(gcf, [save_dest,'fig_pos_3D_sim_',num2str(wind_num)], 'epsc');

figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
subplot(3,1,1)
load(file_name_NMPC);
plot(time(1:skip:end),trajectory_des_delay_m(1:skip:end,1),'k');
hold on
plot(time(1:skip:end),position_m(1:skip:end,1),'color',color_NMPC);
load(file_name_NMHE_NMPC);
plot(time(1:skip:end),position_m(1:skip:end,1),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
plot(time(1:skip:end),position_m(1:skip:end,1),'color',color_GP_NMPC);
ylabel('{\it x}-axis (m)','FontSize',30);
xlim([0 max(time)]);
ylim([-1 30]);
ax_han = gca;
set(ax_han,'FontSize',30)
leg_han = legend('Ref.','NMPC','NMHE-NMPC','GP-NMPC');
set(leg_han,'FontSize',28,'Location','northeast','Orientation','horizontal');
grid on

subplot(3,1,2)
load(file_name_NMPC);
plot(time(1:skip:end),trajectory_des_delay_m(1:skip:end,2),'k');
hold on
plot(time(1:skip:end),position_m(1:skip:end,2),'color',color_NMPC);
load(file_name_NMHE_NMPC);
plot(time(1:skip:end),position_m(1:skip:end,2),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
plot(time(1:skip:end),position_m(1:skip:end,2),'color',color_GP_NMPC);
ylabel('{\it y}-axis (m)','FontSize',30);
xlim([0 max(time)]);
ylim([-2 1]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on

subplot(3,1,3)
load(file_name_NMPC);
plot(time(1:skip:end),trajectory_des_delay_m(1:skip:end,3),'k');
hold on
plot(time(1:skip:end),position_m(1:skip:end,3),'color',color_NMPC);
hold on
load(file_name_NMHE_NMPC);
plot(time(1:skip:end),position_m(1:skip:end,3),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
plot(time(1:skip:end),position_m(1:skip:end,3),'color',color_GP_NMPC);
xlabel('Time (s)','FontSize',30);
ylabel('{\it z}-axis (m)','FontSize',30);
xlim([0 max(time)]);
ylim([1.8 3.7]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on
saveas(gcf, [save_dest,'fig_pos_sim_',num2str(wind_num)], 'epsc');

figure('name','Error','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
subplot(3,1,1)
load(file_name_NMPC);
mean_abs_x_err_NMPC = mean(abs_x_err)
plot(time(1:skip:end),abs_x_err(1:skip:end,1),'color',color_NMPC);
hold on
load(file_name_NMHE_NMPC);
mean_abs_x_err_NMHE_NMPC = mean(abs_x_err)
plot(time(1:skip:end),abs_x_err(1:skip:end,1),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
mean_abs_x_err_GP_NMPC = mean(abs_x_err)
plot(time(1:skip:end),abs_x_err(1:skip:end,1),'color',color_GP_NMPC);
ylabel('|e_{\itx}| (m)','FontSize',30);
xlim([0 max(time)]);
ylim([0 2]);
ax_han = gca;
set(ax_han,'FontSize',30)
leg_han = legend('NMPC','NMHE-NMPC','GP-NMPC');
set(leg_han,'FontSize',28,'Location','northeast','Orientation','horizontal');
grid on

subplot(3,1,2)
load(file_name_NMPC);
mean_abs_y_err_NMPC = mean(abs_y_err)
plot(time(1:skip:end),abs_y_err(1:skip:end,1),'color',color_NMPC);
hold on
load(file_name_NMHE_NMPC);
mean_abs_y_err_NMHE_NMPC = mean(abs_y_err)
plot(time(1:skip:end),abs_y_err(1:skip:end,1),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
mean_abs_y_err_GP_NMPC = mean(abs_y_err)
plot(time(1:skip:end),abs_y_err(1:skip:end,1),'color',color_GP_NMPC);
ylabel('|e_{\ity}| (m)','FontSize',30);
xlim([0 max(time)]);
ylim([0 1]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on

subplot(3,1,3)
load(file_name_NMPC);
mean_abs_z_err_NMPC = mean(abs_z_err)
plot(time(1:skip:end),abs_z_err(1:skip:end,1),'color',color_NMPC);
hold on
load(file_name_NMHE_NMPC);
mean_abs_z_err_NMHE_NMPC = mean(abs_z_err)
plot(time(1:skip:end),abs_z_err(1:skip:end,1),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
mean_abs_z_err_GP_NMPC = mean(abs_z_err)
plot(time(1:skip:end),abs_z_err(1:skip:end,1),'color',color_GP_NMPC);
xlabel('Time (s)','FontSize',30);
ylabel('|e_{\itz}| (m)','FontSize',30);
xlim([0 max(time)]);
ylim([0 0.35]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on
saveas(gcf, [save_dest,'fig_abserr_sim_',num2str(wind_num)], 'epsc');

figure('name','Euclidean Error','units', 'normalized', 'outerposition', [0 0.266 0.559 0.708])
load(file_name_NMPC);
mean_euc_NMPC = mean(euc_err)
plot(time(1:skip:end),euc_err(1:skip:end),'color',color_NMPC);
hold on
load(file_name_NMHE_NMPC);
mean_euc_NMHE_NMPC = mean(euc_err)
plot(time(1:skip:end),euc_err(1:skip:end),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
mean_euc_GP_NMPC = mean(euc_err)
plot(time(1:skip:end),euc_err(1:skip:end),'color',color_GP_NMPC);
xlabel('Time (s)');
ylabel('Euclidean (m)');
xlim([0 max(time)]);
ylim([0 1.3]);
ax_han = gca;
set(ax_han,'FontSize',30)
leg_han = legend('NMPC','NMHE-NMPC','GP-NMPC');
set(leg_han,'FontSize',28,'Location','northeast','Orientation','horizontal');
grid on
saveas(gcf, [save_dest,'fig_eucerr_sim_',num2str(wind_num)], 'epsc');

figure('name','Estimated wind Force in body frame','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
subplot(3,1,1)
load(file_name_NMHE_NMPC);
% FX_body = FY_world
plot(time(1:skip:end),windCom_force_N(1:skip:end,2),'color',color_NMPC);
hold on
plot(time(1:skip:end),windEst_forceMean_0_N(1:skip:end,1),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
plot(time(1:skip:end),windEst_forceMean_0_N(1:skip:end,1),'color',color_GP_NMPC);
% set(gca,'XTickLabel',{' '})
% set(gca,'YTickLabel',{' '})
% set(gca,'xTick',(0:20:80));
% set(gca,'ytick',(-2:1:6));
ylabel('$F^{x_{dist}}$ (N)','Interpreter','latex');
xlim([0 max(time)]);
ylim([-2.5 7]);
ax_han = gca;
set(ax_han,'FontSize',30)
leg_han = legend('True','NMHE','GP');
set(leg_han,'FontSize',28,'Location','northeast','Orientation','horizontal');
grid on

subplot(3,1,2)
load(file_name_NMHE_NMPC);
% FY_body = -FX_world
plot(time(1:skip:end),-windCom_force_N(1:skip:end,1),'color',color_NMPC);
hold on
plot(time(1:skip:end),windEst_forceMean_0_N(1:skip:end,2),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
plot(time(1:skip:end),windEst_forceMean_0_N(1:skip:end,2),'color',color_GP_NMPC);
ylabel('$F^{y_{dist}}$ (N)','Interpreter','latex');
xlim([0 max(time)]);
ylim([-4.7 2.5]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on

subplot(3,1,3)
load(file_name_NMHE_NMPC);
% FZ_body = FZ_world
plot(time(1:skip:end),windCom_force_N(1:skip:end,3),'color',color_NMPC);
hold on
plot(time(1:skip:end),windEst_forceMean_0_N(1:skip:end,3),'color',color_NMHE_NMPC);
load(file_name_GP_NMPC);
plot(time(1:skip:end),windEst_forceMean_0_N(1:skip:end,3),'color',color_GP_NMPC);
xlabel('Time (s)','FontSize',30);
ylabel('$F^{z_{dist}}$ (N)','Interpreter','latex');
xlim([0 max(time)]);
ylim([-2 1.5]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on
saveas(gcf, [save_dest,'fig_wind_forcemean_sim_',num2str(wind_num)], 'epsc');