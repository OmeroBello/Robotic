%Punto 3
close all;
clear;
clc;

p_iniziale_colonna = [-0.4; 0.4; 0.1; 0; 0; 0];
p_finale_colonna = [0.4; 0.4; 0.7; pi; -pi/2; 0]; %orientamento definito con la convenzione ZYZ

%si riscrivano le pose come vettori riga
p_iniziale = p_iniziale_colonna.';
p_finale = p_finale_colonna.';

%tempi [s]
t_finale = 3; 
t_accelerazione = 0.5;

step_time = 0.004; %step time con cui vogliamo calcolare la nostra simulazione. Sarebbe il tempo di ciclo di un controllore di un robot

%si effettui la cinematica inversa per ottenere le configurazioni di giunto iniziale e finale

T_iniziale = transl(p_iniziale(1:3)) * eul2tr(p_iniziale(4:6));
T_finale = transl(p_finale(1:3)) * eul2tr(p_finale(4:6));

Puma_560 = carica_Puma_560();

q_iniziale = ikunc(Puma_560, T_iniziale);
q_finale = ikunc(Puma_560, T_finale);

%si trovi subito l'accelerazione e decelerazione per i tratti di accelerazione/decelerazione
accelerazione = (q_finale - q_iniziale)/(t_accelerazione*(t_finale - t_accelerazione));

% ------------------------------ 1) FASE DI ACCELERAZIONE COSTANTE ------------------------------
NPoints = 125; %numero di punti per la discretizzazione del tempo
p_1 = 0:t_accelerazione/NPoints:t_accelerazione;

for i = 1:6
    for j = 1:length(p_1)
        ddq_target_1(i, j) = accelerazione(i);
        dq_target_1(i, j) = accelerazione(i) * p_1(j);
        q_target_1(i, j) = q_iniziale(i) + (1/2) * accelerazione(i) * p_1(j)^2;
    end
end

% ------------------------------ 2) FASE DI VELOCITÀ COSTANTE ------------------------------
NPoints = 500; %numero di punti per la discretizzazione del tempo
p_2 = t_accelerazione:(t_finale - (t_accelerazione * 2))/NPoints:(t_finale - t_accelerazione);

for i = 1:6
    for j = 1:length(p_2)
        ddq_target_2(i, j) = 0;
        dq_target_2(i, j) = dq_target_1(i, end);
        q_target_2(i, j) = q_iniziale(i) + (1/2) * accelerazione(i) * t_accelerazione^2 + accelerazione(i) * t_accelerazione * (p_2(j) - t_accelerazione);
    end
end

% ------------------------------ 3) FASE DI DECELERAZIONE COSTANTE ------------------------------
NPoints = 125; %numero di punti per la discretizzazione del tempo
p_3 = (t_finale - t_accelerazione):t_accelerazione/NPoints:t_finale;

for i = 1:6
    for j = 1:length(p_3)
        ddq_target_3(i, j) = -accelerazione(i);
        dq_target_3(i, j) = -accelerazione(i)*(p_3(j) - t_finale);
        q_target_3(i, j) = q_finale(i) - (1/2) * accelerazione(i) * (p_3(j)^2 + t_finale^2) + accelerazione(i) * t_finale * p_3(j);
    end
end

p_totale = [p_1, p_2, p_3];
ddq_totale = [ddq_target_1, ddq_target_2, ddq_target_3];
dq_totale = [dq_target_1, dq_target_2, dq_target_3];
q_totale = [q_target_1, q_target_2, q_target_3];

figure(1)
subplot(3,1,1) %ultimo 1 serve a specificare tipo di plot. 
plot(p_totale, ddq_totale(:, :))
xlabel("Time [s]")
ylabel("Accelerazione [rad/s^2]")

subplot(3,1,2) %ultimo 1 serve a specificare tipo di plot. 
plot(p_totale, dq_totale(:, :))
xlabel("Time [s]")
ylabel("Velocità [rad/s]")

subplot(3, 1, 3) %ultimo 1 serve a specificare tipo di plot. 
plot(p_totale, q_totale(:, :))
xlabel("Time [s]")
ylabel("Posizione")

l = legend({'Giunto 1', 'Giunto 2', 'Giunto 3', 'Giunto 4', 'Giunto 5', 'Giunto 6'}); %si crei una legenda! Per andare a distinguere tutti i giunti...
newPosition = [0.95, 0.4, 0.025, 0.2]; %per andare a posizionare la legenda
set(l, 'Position', newPosition);

% ------------------------------ CONTROLLO! ------------------------------

%definiamo i parametri del controllore
K_p = [600; 600; 600; 600; 600; 600] .* eye(6); %creiamo matrice 6*6 con elementi sulla diagonale, ogni giunto è disaccoppiato
K_d = [300; 300; 300; 300; 300; 300] .* eye(6); %guadagno derivativo

posizione_iniziale_colonna = [0; 0; 0.3; 0; pi/2; 0]; %orientamento definito attraverso la convenzione ZYZ
posizione_iniziale = posizione_iniziale_colonna.';

%si effettui la cinematica inversa per ottenere le configurazioni di giunto iniziale
T_iniziale = transl(posizione_iniziale(1:3)) * eul2tr(posizione_iniziale(4:6));

giunto_iniziale = ikunc(Puma_560, T_iniziale);

giunto_pos_attuale = giunto_iniziale.'; %si configuri la posizione iniziale
giunto_vel_attuale = [0; 0; 0; 0; 0; 0]; %si configuri la velocità iniziale
tempo_simulazione = 0; %il tempo della simulazione inizia a 0
giunti_pos_storia = giunto_pos_attuale;
tempo_storia = tempo_simulazione;

for i = 1:size(q_totale, 2)%inizialimo il ciclo principale della nostra simulazione. Il 2 indica il numero di colonne. Scorre tutti i valori della traiettoria
    %ad ogni instante di tempo, controlliamo i termini desiderati della traiettoria
    giunto_pos_desiderata = q_totale(:, i);
    giunto_vel_desiderata = dq_totale(:,i);
    giunto_acc_desiderata = ddq_totale(:, i);

    giunto_acc_attuale = giunto_acc_desiderata + K_d * giunto_vel_desiderata ...
    + K_p * giunto_pos_desiderata - K_d * giunto_vel_attuale - K_p * giunto_pos_attuale;

    %aggiorniamo la simulazione

    %disp(t); %mostra il valore del tempo durante la simulazione

    giunto_vel_attuale = giunto_vel_attuale + giunto_acc_attuale * step_time; %aggiorniamo posizione e velocità secondo il tempo di ciclo
    giunto_pos_attuale = giunto_pos_attuale + giunto_vel_attuale * step_time;
    tempo_simulazione = tempo_simulazione + step_time; %aggiorniamo il tempo
    giunti_pos_storia = [giunti_pos_storia, giunto_pos_attuale]; %aggiorniamo la storia
    tempo_storia = [tempo_storia, tempo_simulazione];
end

tempo_storia(size(q_totale, 2)) = [];

%plottiamo i risultati
figure(2)
for i = 1:6
    subplot(6,1,i)
    plot(tempo_storia, q_totale(i, :)) %plottiamo la traiettoria desiderata per ciascun giunto separatamente
    hold on
    vettore = [1:7:length(giunti_pos_storia(i,:))]; %creiamo un vettore. la traiettoria vera del robot la facciamo come sequenza di marker, prendiamo un marker ogni 25
    plot(tempo_storia(vettore), giunti_pos_storia(i, vettore), 'or') %lo stampiamo con il marker rosso
    xlabel("Tempo [s]");
    ylabel("Posizione [rad]");
    title(strcat("Joint", num2str(i))); %strcat concatena, num2str converte da numero a stringa
end

%%
figure(3)
for i = 1:length(q_totale) %ad ogni ciclo, plottiamo il nostro modello
    Puma_560.plot(q_totale(:, i).') %accediamo a tutti i giunti allo step i
end







