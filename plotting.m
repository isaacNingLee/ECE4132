clear all;close all;clc;

mat1 = load('first_order_no_td.mat').RMSE_arr;
mat2 = load('second_order_no_td.mat').RMSE_arr;
mat3 = load('first_order_td.mat').RMSE_arr;

hold on
plot(mean(mat1,2))
plot(mean(mat2,2))
plot(mean(mat3,2))
legend('1st order','2nd order','1st order td')