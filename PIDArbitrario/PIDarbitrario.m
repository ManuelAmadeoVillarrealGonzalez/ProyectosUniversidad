clear all;
close all;
clc;

M=0.510;%kg
m=0.03; %kg
L=0.25;%m
g=9.81;%m/s^2
ts=5;
%0.01<Ts0.15
Ts=0.01;%Sirve Fraccional


%%Valor pendulo continuo
kpp=-556.27969825542;%Ganancia Proporcional 
kip=-3726.07419650531;%Ganancia Integral
kdp=-15.864761054699;%Ganancia Derivativa
Np=477.576634483601;%Filtro Pendulo
%%Valor carro continuo
kpc=0.0704426191079745;%Ganancia Proporcional
kic=0.00199754552839596;%Ganancia Integral
kdc=0.334436620011803;%Ganancia Derivativa
Nc=5.49731405490558;%Filtro Carrito

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Valores para PID arbitrario y fraccional
alphapendulo=.753;
alphacarrito=.82;
betapendulo=.9;
betacarrito=.5;
%Seleccionamos el valor de alpha para nuestro PID.
% Si es Mayor que 1 se utiliza el PID arbitrario si es menor se utiliza el PID fraccional
wl=0.01;
wh=100;
orden=6;%Seleccionamos el orden de PID
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Main Program
%llamamos a Derivative para encontrar el valor de Nd y Dd del pendulo y carrito
if (alphapendulo>=1)
    alphapendulo=alphapendulo-1;
    alphacarrito=alphacarrito-1;
    betapendulo=betapendulo-1;
    betacarrito=betacarrito-1;
    Ts=0.010;
end
[Ndp,Ddp,Hadp]=Derivative(betapendulo,wl,wh,orden,Np,Ts);
[Ndc,Ddc,Hadc]=Derivative(betacarrito,wl,wh,orden,Nc,Ts);
[Nip,Dip,Haip]=Integral(-alphapendulo,wl,wh,orden,Ts);
[Nic,Dic,Haic]=Integral(-alphacarrito,wl,wh,orden,Ts);
%Ndp,Ddp,Ndc,Ddc,Nip,Dip,Nic,Dic
Hadp,Hadc,Haip,Haic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Para la discretizacion de continuo a discreto utilizaremos el criterio de
%tustin (2/Ts)*((z-1)/(z+1)). Como matlab nos permite utilizar la funcion
%c2d con tustin como argumento se utilizara esta opcion para crear un
%codigo mas eficiente.

%Se crearan 2 funciones que llamaremos desde el main de matlab una sera
%para calcular la derivada de ambos PID y la otra para la integral de los
%PID

%Valor Derivativo
function [Nd,Dd,Hadd]=Derivative(beta,wl,wh,orden,filtro,Ts)
[Ha,numd,dend]=CFI(beta,wl,wh,orden);
[numd,dend] = tfdata(Ha);
%Utilizamos los filtros 
    if(filtro<200)
        dend{1}=numd{1}+dend{1}*filtro;
        numd{1}=numd{1}*filtro;
        Ha=tf(numd{1},dend{1});
    end
Hadd=c2d(Ha,Ts,'tustin'); %Hadd(Ha Discrete Derivative)
[numdmin,dendmin] = tfdata(minreal(Hadd));
Nd=numdmin{1};
Dd=dendmin{1};
end

function [Ni,Di,Hadi]=Integral(alpha,wl,wh,orden,Ts)
[Ha,numi,deni]=CFI(alpha,wl,wh,orden);
Hadi=c2d(Ha,Ts,'tustin'); %Hadd(Ha Discrete Integral)
[numimin,denimin] = tfdata(minreal(Hadi));
Ni=numimin{1};
Di=denimin{1};
end

function [Ha,num,den]= CFI(alpha,wl,wh,N)
w=logspace(log10(wl),log10(wh));
A=(j*w).^alpha;
Ha=fitfrd(frd(A,w),N);
[num,den]=ss2tf(Ha.A,Ha.B,Ha.C,Ha.D);
Ha=minreal(tf(num,den));
end 