close all;
clear;
clc;

% addpath(genpath('data/'));
input_dim = 3;
time_idx = 1;

file_name_train = 'Mat_data_train';
data = csvread([file_name_train,'.txt']);

time_vec_train_colNum = time_idx;
time_vec_train = data(:,time_vec_train_colNum);
if input_dim > 1
    points_train_colNum = time_idx+1:time_idx+input_dim;
    targets_train_colNum = time_idx+input_dim+1;
    points_train = data(:,points_train_colNum);
else
    targets_train_colNum = time_idx+1;
end
targets_train = data(:,targets_train_colNum);

figure;
plot(time_vec_train,targets_train,'b.');

file_name_test = 'Mat_data_test';
data = csvread([file_name_test,'.txt']);

time_vec_test_colNum = time_idx;
if input_dim > 1
    points_test_colNum = time_idx+1:time_idx+input_dim;
    true_targets_test_colNum = time_idx+input_dim+1;
    mu_test_colNum = time_idx+input_dim+2;
    var_test_colNum = time_idx+input_dim+3;
    absError_test_colNum = time_idx+input_dim+4;
else
    true_targets_test_colNum = time_idx+1;
    mu_test_colNum = time_idx+2;
    var_test_colNum = time_idx+3;
    absError_test_colNum = time_idx+4;
end


time_vec_test = data(:,time_vec_test_colNum);
if input_dim > 1
    points_test = data(:,points_test_colNum);
end
true_targets_test = data(:,true_targets_test_colNum);
mu_test = data(:,mu_test_colNum);
% var_test = data(:,var_test_colNum);
var_test = 10*data(:,var_test_colNum);
absError_test = abs(data(:,absError_test_colNum));

figure;
f = [mu_test+2*sqrt(var_test); flipdim(mu_test-2*sqrt(var_test),1)];
fill([time_vec_test; flipdim(time_vec_test,1)], f, [7 7 7]/8)
hold on; plot(time_vec_test, mu_test,'r'); plot(time_vec_train, targets_train, 'b.')

