%% ----------------- Projeto de Controlo Ótimo: Fase 2 --------------------
% Este script apresenta um menu que permite ao utilizador correr uma
% simulação dos modelos linear e não linear do drone
%
% Catarina Pires    90230
% Mª Sofia Canas    90323
% Mariana Semião    90328
% Ricardo Henriques 90349
% Última edição a 07 de Maio 2021
% ------------------------------------------------------------------------

%%
clc
clear
close all
%% Parâmetros

% Propriedades Físicas do Drone
m      =    0.063;                                %Kg
J_x    =    5.8286e-5;                            %Kg.m^2
J_y    =    7.1691e-5;                            %Kg.m^2
J_z    =    1.0e-4;                               %Kg.m^2
L      =    0.0624;                               %m

% Aceleração Gravítica
g      =    9.81;                                 %m/s^2
g_m    =   [0 ;0 ;g];


% Parâmetros dos Rotores
K_T     =    4.72e-8;                             %N.(rad/s)^-2
K_Q     =    1.1393e-10;                          %N.(rad/s)^-2
K_Omega =    3.8013e-4;                           %(rad/s)^-1

% Voltagem Máxima da Bateria
V_bat   =    3.7;                                 %V

%Ponto de Equilibrio
Omega_0 = sqrt((m*g/4)/K_T);                      %rad/s

%Constantes Movimento
b_z = 2*K_T*Omega_0;
b_p = (b_z*L*cos(pi/4))/J_x;
b_q = (b_z*L*sin(pi/4))/J_y;
b_r = (2*K_Q*Omega_0)/J_z;

% Transformação Para a Obtenção da Tensão Normalizada
act2motor = [1 1  1  1;
    -1 1 -1  1;
    1 1 -1 -1;
    1 -1  -1 1];
act2motor_I = K_Omega*inv(act2motor);

%% Perturbações
% Wind disturbance variance
Wind    = 1e-4*[1; 1; 1];

% Noise Variance
var_px     = 7.3221*10^(-4);
var_py     = 4.2829*10^(-4);
var_pz     = 1.8000*10^(-4);
var_roll   = 3.4817*10^(-4);
var_pitch  = 2.6060*10^(-4);
var_yaw    = 3.8313*10^(-5);
var_omegax = 4.9424*10^(-7);
var_omegay = 5.2748*10^(-7);
var_omegaz = 4.2642*10^(-7);

% System model variance
Noise_v     = [Wind(3)/(m); 1];
Noise_lr    = [10^-3; 0; Wind(2)/(m); 10^-1];
Noise_lp    = [10^-3; 0; Wind(1)/(m); 10^-1];
Noise_g     = [10^-6; 10^-1];

%% Subsistemas
% Eixo Vertical
A_v = [0 0;1 0];
B_v = [-b_z ;0];
C_v = [0 1];

% Eixo Guinada
A_g = [0 0;1 0];
B_g = [b_r;0];
C_g = [0  1];

% Eixo Longitudinal + Picada
A_lp = [0  0 0 0; 1 0 0 0; 0 -g 0 0; 0 0 1 0];
B_lp = [b_q;0;0;0];
C_lp = [0 1 0 0; 0 0 0 1];

% Eixo Lateral + Rolamento
A_lr = [0  0 0 0; 1 0 0 0; 0 g 0 0; 0 0 1 0];
B_lr = [b_p;0;0;0];
C_lr = [0 1 0 0; 0 0 0 1];

%% Augmented Systems
% Eixo Vertical
A_v_barra = [0 0 0; 1 0 0; 0 -1 0];
B_v_barra = [-b_z ;0; 0];

% Eixo Guinada
A_g_barra = [0 0 0; 1 0 0; 0 -1 0];
B_g_barra = [b_r ;0; 0];

% Eixo Longitudinal + Picada
A_lp_barra = [0  0  0  0  0;
    1  0  0  0  0;
    0 -g  0  0  0;
    0  0  1  0  0;
    0  0  0 -1  0];
B_lp_barra = [b_q;0;0;0;0];

% Eixo Lateral + Rolamento
A_lr_barra = [0  0  0  0  0;
    1  0  0  0  0;
    0  g  0  0  0;
    0  0  1  0  0;
    0  0  0 -1  0];
B_lr_barra = [b_p;0;0;0;0];

%% LQR Q e R
%Eixo Vertical
Q_v = diag([5000; 8000; 5000]);
R_v = 0.0001;

% Eixo Guinada
Q_g = diag([0.5; 0; 100]);
R_g = 1/5000;

% Eixo Longitudinal + Picada
Q_lp = diag([100; 0; 5000; 3000; 1500]);
R_lp =0.01;

% Eixo Lateral + Rolamento
Q_lr = diag([100; 0; 5000; 3000; 1500]);
R_lr = 0.01;

%% Kalman-Bucy Q0 e R0
%Eixo Vertical

Q_v0 = diag(Noise_v);
R_v0 = var_pz;

% Eixo Guinada
Q_g0 = diag(Noise_g);
R_g0 = var_yaw;

% Eixo Longitudinal + Picada
Q_lp0 = diag(Noise_lp);
R_lp0 = [var_pitch 0;0 var_px];

% Eixo Lateral + Rolamento
Q_lr0 = diag(Noise_lr);
R_lr0 = [var_roll 0;0 var_py];

%% Observadores Kalman-Bucy
G_2 = [1 0;0 1];
% Subsistema Eixo Guinada
sys_g = ss(A_g,[B_g G_2],C_g,[0 [0 0]]);
[Kestimator_g,Lk_g,P_g] = kalman(sys_g,Q_g0,R_g0);

% Subsistema Eixo Vertical
sys_v = ss(A_v,[B_v G_2],C_v,[0 [0 0]]);
[Kestimator_v,Lk_v,P_v] = kalman(sys_v,Q_v0,R_v0);

% Subsistema Eixo Lateral + Rolamento
G_4 = eye(4);
H_4 = zeros(2,4);
sys_lr = ss(A_lr,[B_lr G_4],C_lr,[[0;0] H_4]);
[Kestimator_lr,Lk_lr,P_lr] = kalman(sys_lr,Q_lr0,R_lr0);

% Subsistema Eixo Longitudinal + Picada
sys_lp = ss(A_lp,[B_lp G_4],C_lp,[[0;0] H_4]);
[Kestimator_lp,Lk_lp,P_lp] = kalman(sys_lp,Q_lp0,R_lp0);


%% LQR - Fase 2
[K_v] = lqr(A_v_barra,B_v_barra,Q_v,R_v);
[K_g] = lqr(A_g_barra,B_g_barra,Q_g,R_g);
[K_lp] = lqr(A_lp_barra,B_lp_barra,Q_lp,R_lp);
[K_lr] = lqr(A_lr_barra,B_lr_barra,Q_lr,R_lr);

%%
fprintf ('Projeto de Controlo Ótimo: Fase 2\n\n');
repetir = true;
while repetir
    switch input('Por favor escolha um modelo para analisar:\n\nA) Modelo linear\n\nB) Modelo não linear\n\nC) Sair\n\n','s')
        case {'A' 'a'}
            ref = input('\n\nDeseja testar a simulação com uma referência constante (1) ou com uma trajetória pré-definida (2)? [1/2]: ','s');
            
            disturbances_w = input('\n\nDeseja testar a simulação tendo em conta perturbações de processo? [s/n]: ','s');
            
            disturbances_v = input('\n\nDeseja testar a simulação tendo em conta perturbações dos sensores? [s/n]: ','s');
                        
            time = input('\n\nDefina o tempo de simulação [s]: ','s');
            time = str2num(time);
            
            open_system('Modelo_Linear_Fase_2_Grupo_10');
            % Referência
            if ref == '1'
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_Ref','sw','0')
                
            elseif ref == '2'
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_Ref','sw','1')
                
            end
            
            % Process Disturbances
            if disturbances_w == 's' || disturbances_w == 'S'
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_g','sw','0')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_lp','sw','0')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_lr','sw','0')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_v','sw','0')
                
            elseif disturbances_w == 'n' || disturbances_w == 'N'
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_g','sw','1')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_lp','sw','1')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_lr','sw','1')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_w_v','sw','1')
                
            end
            
            % Measurement Disturbances
            if disturbances_v == 's' || disturbances_v == 'S'
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_g','sw','0')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_lp','sw','0')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_lr','sw','0')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_v','sw','0')
                
            elseif disturbances_v == 'n' || disturbances_v == 'N'
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_g','sw','1')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_lp','sw','1')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_lr','sw','1')
                set_param('Modelo_Linear_Fase_2_Grupo_10/Switch_v_v','sw','1')
                
            end
            save_system('Modelo_Linear_Fase_2_Grupo_10');
            close_system('Modelo_Linear_Fase_2_Grupo_10');
            
            sim('Modelo_Linear_Fase_2_Grupo_10',time)
            
            Graficos_Fase2(Position, Rotation);
            
            repetir = true;
        case {'B' 'b'}
            ref = input('\n\nDeseja testar a simulação com uma referência constante (1) ou com uma trajetória pré-definida (2)? [1/2]: ','s');
            
            disturbances_vento = input('\n\nDeseja testar a simulação tendo em conta perturbações provenientes da resistência do ar? [s/n]: ','s');
            
            disturbances_forca = input('\n\nDeseja testar a simulação tendo em conta um força exterior a atuar sobre o drone? [s/n]: ','s');
            
            disturbances_v = input('\n\nDeseja testar a simulação tendo em conta perturbações dos sensores? [s/n]: ','s');
            
            time = input('\n\nDefina o tempo de simulação [s]: ','s');
            time = str2num(time);
             
            open_system('Modelo_Nao_Linear_Fase_2_Grupo_10');
            % Referencia
            if ref == '1'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Switch_Ref','sw','0')
                
            elseif ref == '2'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Switch_Ref','sw','1')
                
            end
            
            % Air Resistence Disturbances
            if disturbances_vento == 's' || disturbances_vento == 'S'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Sistema de Movimento/Switch_vento','sw','1')
                
            elseif disturbances_vento == 'n' || disturbances_vento == 'N'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Sistema de Movimento/Switch_vento','sw','0')
                
            end
            
            % External Forces
            if disturbances_forca == 's' || disturbances_forca == 'S'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Sistema de Movimento/Switch_forca','sw','1')
                
            elseif disturbances_forca == 'n' || disturbances_forca == 'N'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Sistema de Movimento/Switch_forca','sw','0')
                
            end
            
            % Measurement Disturbances
            if disturbances_v == 's' || disturbances_v == 'S'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_g','sw','0')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_lp','sw','0')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_lr','sw','0')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_v','sw','0')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_ang_vel','sw','0')
                
            elseif disturbances_v == 'n' || disturbances_v == 'N'
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_g','sw','1')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_lp','sw','1')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_lr','sw','1')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_v_v','sw','1')
                set_param('Modelo_Nao_Linear_Fase_2_Grupo_10/Observador Kalman-Bucy/Switch_ang_vel','sw','1')
            end
            save_system('Modelo_Nao_Linear_Fase_2_Grupo_10');
            close_system('Modelo_Nao_Linear_Fase_2_Grupo_10');
            
            sim('Modelo_Nao_Linear_Fase_2_Grupo_10',time)
            
            Graficos_Fase2(Position, Rotation);
            
            repetir = true;
            
        case {'C' 'c'}
            repetir = false;
            
        otherwise
            fprintf(2,'\nOpção Inválida \n\n');
            repetir = true;
    end
end






