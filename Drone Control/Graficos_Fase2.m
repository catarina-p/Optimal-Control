function Graficos_Fase2(Position, Rotation)
% Esta função faz o plot dos dados obtidos na simulação e cria uma animação
% da trajetória do drone
%
% Input: Position - variável tipo timeseries que contém os valores de
%                   posição do drone (x, y e z) obtidos durante a simulação
%                   em simulink
%
%       Rotation - variável tipo timeseries que contém os valores de
%                  rotação do drone (roll, pitch e yaw) obtidos durante a 
%                  simulação em simulink
%
% Catarina Pires    90230
% Mª Sofia Canas    90323
% Mariana Semião    90328
% Ricardo Henriques 90349
%
% Última edição a 07 de Maio 2021
% ------------------------------------------------------------------------

time = Position.Time;

p_D = -Position.Data(:,3); %z
p_E = Position.Data(:,2); %y
p_N = Position.Data(:,1); %x

psi = Rotation.Data(:,3);   %Guinada
phi = Rotation.Data(:,1);   %Rolamento
theta = Rotation.Data(:,2); %Picada

% Gráficos de Posição e Rotação

figure('Position', [300, 200, 1000, 500]) % Fase 2
subplot(1,2,1);
plot(time,p_N,time,p_E,time,p_D);
legend('p_N','p_E','p_D')
title('Posição')
xlabel('Tempo [s]')
ylabel('Distância [m]')
grid on

subplot(1,2,2);
plot(time,phi*180/pi,time,theta*180/pi,time,psi*180/pi);
legend('Roll','Pitch','Yaw')
title('Rotação')
xlabel('Tempo [s]')
ylabel('Ângulo [º]')
grid on

% Trajetoria
figure
curve = animatedline('LineWidth', 0.3);

if max(abs(p_N)) == 0 && max(abs(p_E)) ~= 0
    set(gca,'XLim',[-10 10],'YLim',[-max(abs(p_E)) max(abs(p_E))],'ZLim',...
        [-max(abs(p_D)) max(abs(p_D))]);

elseif max(abs(p_E)) == 0 && max(abs(p_N)) ~= 0
    set(gca,'XLim',[-max(abs(p_N)) max(abs(p_N))],'YLim',[-10 10],'ZLim',...
        [-max(abs(p_D)) max(abs(p_D))]);

elseif max(abs(p_E)) == 0 && max(abs(p_N)) == 0
set(gca,'XLim',[-10 10],'YLim',[-10 10],'ZLim',[0 max(p_D)]);

else
set(gca,'XLim',[-max(abs(p_N)) max(abs(p_N))],'YLim',...
    [-max(abs(p_E)) max(abs(p_E))],'ZLim',[-max(abs(p_D)) max(abs(p_D))]);

end

view(43,24);
title('Trajetória do Drone')
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
hold on
grid on
for i = 1:length(p_D)
    addpoints(curve,p_N(i),p_E(i),p_D(i));
    head = scatter3(p_N(i),p_E(i),p_D(i),'filled','MarkerFaceColor','b',...
        'MarkerEdgeColor','b');
    drawnow        
    delete(head);
end
head = scatter3(p_N(i),p_E(i),p_D(i),'filled','MarkerFaceColor','b',...
    'MarkerEdgeColor','b');
end
