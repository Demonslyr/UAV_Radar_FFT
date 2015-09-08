% Using Matlab to check 1-D FFT results from Daniel's GPU processing
% Lei Shi
% 8/16/14

clear all; close all; clc;
load_GPU2 = csvread('mtx_out_2Dfft_28shifted.csv',0,0);

[M,N] = size(load_GPU2);

% 2-D FFT GPU Results Analysis ------------------------------------------
R_ind2 = [1:2:N-2];      % NOTE: N-1 because last column is all zeros error
I_ind2 = [2:2:N-2];      % NOTE: N-1 because last column is all zeros error

R_GPU2 = load_GPU2(:,R_ind2);
I_GPU2 = load_GPU2(:,I_ind2);

data_GPU2 = R_GPU2+i*I_GPU2;

data_GPU2_dB = 10*log10(abs(data_GPU2));

figure;
surf(data_GPU2_dB);
colorbar;
caxis([50,80]);
%view(0,0);
title('Microcontroller Calculated 2-D FFT result TRUNCATED');
xlabel('Fast time index');
ylabel('Slow time index');
zlabel('Log Magnitude');


