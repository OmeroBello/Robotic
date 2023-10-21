%Punto 1
close all;
clear;
clc;

%Parametri

Puma_560 = carica_Puma_560();

orientamento_iniziale = [pi/2; pi/2; 0]; %secondo convenzione ZYZ
centro = [0; -0.3; 0.5];
rho = 0.3; %raggio
angolo_arco_circonferenza = pi;
arco_circonferenza = rho * angolo_arco_circonferenza;

%si definisca come evolve il parametro s
s_pos_start = 0;
s_pos_end = arco_circonferenza;

%si generi ora il profilo di s
[p, F] = polynomial_5(10, [s_pos_start, 0.0, 0.0, s_pos_end, 0.0, 0.0]);
s_pos = F(1, :); %evoluzione di s. Estraiamo la posizione dalla polinomiale
s_vel = F(2, :);
s_acc = F(3, :);

%l'evoluzione della posizione può essere calcolata con l'equazione parametrica della circonferenza, per ogni valore s che abbiamo generato
for i=1:length(s_pos)
    x_e(i, 1) = rho * cos(s_pos(i)/rho);
    x_e(i, 2) = rho * sin(s_pos(i)/rho);
    x_e(i, 3) = centro(3);
    x_e(i, 4:6) = orientamento_iniziale; %aggiungiamo anche l'orientamento che rimane fisso a quello iniziale
end

%si calcoli ora la velocità dell'EE, aggiungendo anche la velocità angolare dell'orientamento che tanto è nulla
for i=1:length(s_vel)
    dx_e(i, 1) = s_vel(i) * (-sin(s_pos(i)/rho));
    dx_e(i, 2) = s_vel(i) * cos(s_pos(i)/rho);
    dx_e(i, 3) = 0;
    dx_e(i, 4:6) = 0;
end

%si calcoli ora l'accelerazione dell'EE
for i=1:length(s_vel)
    ddx_e(i, 1) = s_acc(i) * (-sin(s_pos(i)/rho));
    ddx_e(i, 2) = s_acc(i) * cos(s_pos(i)/rho);
    ddx_e(i, 3) = 0;
    ddx_e(i, 4:6) = 0;
end

figure(1)
subplot(3,1,1) %ultimo 1 serve a specificare tipo di plot
plot(p, x_e(:, 1))
xlabel("Time [s]")
ylabel("Posizione [x]")

subplot(3,1,2) %ultimo 1 serve a specificare tipo di plot
plot(p, x_e(:, 2))
xlabel("Time [s]")
ylabel("Posizione [y]")

subplot(3, 1, 3) %ultimo 1 serve a specificare tipo di plot
plot(p, x_e(:, 3))
xlabel("Time [s]")
ylabel("Posizione [z]")

figure(2)
subplot(3,1,1) %ultimo 1 serve a specificare tipo di plot
plot(p, dx_e(:, 1))
xlabel("Time [s]")
ylabel("Velocità x [rad/s]")

subplot(3,1,2) %ultimo 1 serve a specificare tipo di plot
plot(p, dx_e(:, 2))
xlabel("Time [s]")
ylabel("Velocità y [rad/s]")

subplot(3,1,3) %ultimo 1 serve a specificare tipo di plot
plot(p, dx_e(:, 3))
xlabel("Time [s]")
ylabel("Velocità z [rad/s]")

figure(3)
subplot(3,1,1) %ultimo 1 serve a specificare tipo di plot
plot(p, ddx_e(:, 1))
xlabel("Time [s]")
ylabel("Accelerazione x [rad/s^2]")

subplot(3,1,2) %ultimo 1 serve a specificare tipo di plot
plot(p, ddx_e(:, 2))
xlabel("Time [s]")
ylabel("Accelerazione y [rad/s^2]")

subplot(3,1,3) %ultimo 1 serve a specificare tipo di plot
plot(p, ddx_e(:, 3))
xlabel("Time [s]")
ylabel("Accelerazione z [rad/s^2]")

%si individuino le varie matrici di trasformazione
for i = 1:length(x_e)
    trasformazioni(:, :, i) = transl(x_e(i, 1:3)) * eul2tr(x_e(i, 4:6));
end

%si individuino le configurazioni di giunto corrispondenti
q_target = ikunc(Puma_560, trasformazioni);

%eseguiamo la pianificazione della traiettoria anche nello spazio di giunto

%posizioni e velocità dei giunti
figure(4)
subplot(2, 1, 1); %2 righe, una colonna. In questo primo plot, inseriamo posizione di tutti i giunti
plot(p, q_target)
xlabel("Time [s]")
ylabel("Orientamento [rad]")

%calcoliamoci le velocità dei giunti attraverso la cinematica differenziale inversa, con l'ausilio del jacobiano geometrico
dq_target = [];
for i = 1:length(q_target)
    dq_target(:, i) = inv(Puma_560.jacob0(q_target(i, :))) * dx_e(i, :).';
end

subplot(2, 1, 2); %2 righe, una colonna. In questo secondo plot, inseriamo velocità di tutti i giunti
plot(p, dq_target.')
xlabel("Time [s]")
ylabel("Velocità [rad/s]")

l = legend({'Giunto 1', 'Giunto 2', 'Giunto 3', 'Giunto 4', 'Giunto 5', 'Giunto 6'}); %si crei una legenda! Per andare a distinguere tutti i giunti...
newPosition = [0.95, 0.4, 0.025, 0.2]; %per andare a posizionare la legenda
set(l, 'Position', newPosition);

%%

figure(5)
for i = 1:length(q_target) 
    Puma_560.plot(q_target(i, :)) %accediamo a tutti i giunti allo step i
end