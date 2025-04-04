
%speed_to_erpm_gain  = 4277.5
%speed_to_duty (m/s) = 0.0602

Ts = 20;      % Tempo de amostragem (20 Hz)

L = 0.35; % Distância entre eixos
servo_gain=-0.840; % Ganho do servo (rad/unidade)


servo_max=0.4; % Ângulo máximo do servo
v_max = 1;        % Velocidade máxima (m/s)


delta_max = abs(servo_max/servo_gain); % Ângulo máximo de direção (radianos)

% Definir o horizonte de predição
N = 20; % Horizonte do MPC
T_total = 100;  % Tempo total de simulação (em steps)


n = (2*pi)/T_total;
n1 = 1:(T_total+N);
n1=n1*n;
x_ref = 2.5 * cos(n1);
y_ref = 1.75 * sin(n1);

% h_ref = plot(x_ref(1), y_ref(1), 'r.', 'MarkerSize', 10);
% xlim([-3, 3]);
% ylim([-3, 3]);
% for i = 1:length(x_ref)-N
%     set(h_ref, 'XData', x_ref(i:i+N), 'YData', y_ref(i:i+N));
%     drawnow;
%     pause(1/Ts); % Adjust the pause duration as needed
% end


% Estados iniciais
x0 = [-2.5; 0; 0; 0]; % [x, y, theta, v]
u0 = zeros(N, 2);  % Controles iniciais [delta, velocidade]


% Parâmetros do MPC
Q = diag([10, 10, 1, 1]);  % Ponderação dos estados
R = diag([0.1, 0.1]);       % Ponderação dos controles

% Restrições
%lb = [-delta_max * ones(N,1), zeros(N,1)];
lb = [-delta_max * ones(N,1), -v_max * ones(N,1)];
ub = [delta_max * ones(N,1), v_max * ones(N,1)];

% Estado inicial
x = [2.5; 0; 0; 0]; % [x, y, theta, v]
history = x';

% Gráfico Trajetoria
figure; hold on; grid on;
plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5);
traj = plot(x(1), x(2), 'bo-', 'LineWidth', 2);
traj_ref = plot(x_ref(1:N), y_ref(1:N), 'y--', 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]');
title('MPC rodando em loop');
legend('Trajetória de referência', 'Trajetória do veículo');

figure;
subplot(2,1,1);
p_angle=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Tempo [s]');
ylabel('Ângulo de direção [rad]');
title('Ângulo de direção do veículo');
subplot(2,1,2);
p_v=plot(0, 0, 'b-', 'LineWidth', 2);
xlabel('Tempo [s]');
ylabel('Velocidade [m/s]');
title('Velocidade do veículo');

% Loop principal do MPC
for t_idx = 1:T_total-1
    % Trajetória de referência para o horizonte atual
    x_ref_step = x_ref(t_idx:t_idx+N-1);
    y_ref_step = y_ref(t_idx:t_idx+N-1);

    % Chute inicial do controle
    u0 = zeros(N,2); % [delta, v]

    % Função de custo
    cost_fun = @(U) mpc_cost(U, x, x_ref_step, y_ref_step, Ts, L, N, Q, R);

    % Otimização
    options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');
    U_opt = fmincon(cost_fun, u0, [], [], [], [], lb, ub, [], options);

    % Aplica o primeiro controle
    u = U_opt(1,:)'; % [delta; v]

    % Atualiza estado com o modelo da bicicleta
    x = simulate_dynamics(x, u, Ts, L);

    history = [history; x'];

    % Atualiza gráfico
    set(traj, 'XData', history(:,1), 'YData', history(:,2));
    drawnow;
    set(traj_ref, 'XData', x_ref_step , 'YData', y_ref_step);
    drawnow;
    set(p_angle , 'XData', 0:t_idx, 'YData', history(:,3));
    set(p_v , 'XData', 0:t_idx, 'YData',history(:,4));
    drawnow;
end

%Plotar angulo de direção
% figure;
% subplot(2,1,1);
% plot(1:T_total, history(:,3), 'b-', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Ângulo de direção [rad]');
% title('Ângulo de direção do veículo');
% subplot(2,1,2);
% plot(1:T_total, history(:,4), 'b-', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Velocidade [m/s]');
% title('Velocidade do veículo');


% ---- Simulação da Dinâmica ----
function x_sim = simulate_dynamics(x, u, Ts, L)

delta = u(1);
v = u(2);
x(4)=v;

x_sim = x + Ts * [
    v * cos(x(3));        % dx/dt
    v * sin(x(3));        % dy/dt
    (v / L) * tan(delta); % dtheta/dt
    0                     % dv/dt = 0 (v é entrada direta)
    ];

end


% ---- Função de Custo ----
function J = mpc_cost(U, x0, x_ref, y_ref, Ts, L, N, Q, R)
x = x0;
J = 0;

for k = 1:N
    u = U(k, :)'; % Controle atual

    x = simulate_dynamics(x, u, Ts, L);
    

    % Erro em relação à referência
    e = [x(1)-x_ref(k); x(2)-y_ref(k); x(3); x(4)];
    J = J + e' * Q * e + u' * R * u;
end
end