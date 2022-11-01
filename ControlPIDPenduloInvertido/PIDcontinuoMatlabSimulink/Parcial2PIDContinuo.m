clear all;
close all;
clc;

M=0.510;%kg
m=0.03; %kg
L=0.25;%m
g=9.81;%m/s^2
ts=5;
%0.01<Ts0.15
Ts=0.015;

%%Valor pendulo continuo
kpp=-54.8683679286121;
kip=2.04775747078051;
kdp=0.0904146272305055;
Np=83.9576991418546;
%%Valor carro continuo
kpc=0.0857737044561583;
kic=0.0832220773397635;
kdc=1.96746042698113;
Nc=7.47498988089675;