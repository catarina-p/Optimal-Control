function Graficos_Fase2(Position, Rotation)
% Esta fun��o faz o plot dos dados obtidos na simula��o e cria uma anima��o
% da trajet�ria do drone
%
% Input: Position - vari�vel tipo timeseries que cont�m os valores de
%                   posi��o do drone (x, y e z) obtidos durante a simula��o
%                   em simulink
%
%       Rotation - vari�vel tipo timeseries que cont�m os valores de
%                  rota��o do drone (roll, pitch e yaw) obtidos durante a 
%                  simula��o em simulink
%
% Catarina Pires    90230
% M� Sofia Canas    90323
% Mariana Semi�o    90328
% Ricardo Henriques 90349
%
% �ltima edi��o a 07 de Maio 2021
% ------------------------------------------------------------------------

time = Position.Time;

p_D = -Position.Data(:,3); %z
p_E = Position.Data(:,2); %y
p_N = Position.Data(:,1); %x

psi = Rotation.Data(:,3);   %Guinada
phi = Rotation.Data(:,1);   %Rolamento
theta = Rotation.Data(:,2); %Picada

% Gr�ficos de Posi��o e Rota��o

figure('Position', [300, 200, 1000, 500]) % Fase 2
subplot(1,2,1);
plot(time,p_N,time,p_E,time,p_D);
legend('p_N','p_E','p_D')
title('Posi��o')
xlabel('Tempo [s]')
ylabel('Dist�ncia [m]')
grid on

subplot(1,2,2);
plot(time,phi*180/pi,time,theta*180/pi,time,psi*180/pi);
legend('Roll','Pitch','Yaw')
title('Rota��o')
xlabel('Tempo [s]')
ylabel('�ngulo [�]')
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
title('Trajet�ria do Drone')
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
