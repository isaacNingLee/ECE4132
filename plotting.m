clear all;close all;clc;

mat1 = load('first_order_no_td_upload.mat').RMSE_arr;
mat2 = load('second_order_no_td_upload.mat').RMSE_arr;
mat3 = load('first_order_clemen.mat').RMSE_arr;
mat4 = load('second_order_clemen.mat').RMSE_arr;
mat5 = load('kush_test_first_order_no_td2.mat').RMSE_arr;
mat6 = load('kush_test_second_order_no_td2.mat').RMSE_arr;

hold on
plot(mean(transpose(mat1)))
plot(mean(transpose(mat2)))
plot(mean(transpose(mat3)))
plot(mean(transpose(mat4)))
plot(mean(transpose(mat5)))
plot(mean(transpose(mat6)))
title("Mean of 4 model per run")
legend('1st order Isaac','2nd order Isaac','1st order Clemen','2nd order Clemen','1st order Kush','2nd order Kush')

mean(mat1, 'all')
mean(mat2, 'all')
mean(mat3, 'all')
mean(mat4, 'all')
mean(mat5, 'all')
mean(mat6, 'all')


figure()
hold on
plot(mean(transpose(mat1+mat3+mat5))/3)
plot(mean(transpose(mat2+mat4+mat6))/3)
legend('1st irder','2nd order')