%Punto 4
close all;
clear;
clc;

%si abbiano i seguenti parametri/vincoli di input

Puma_560 = carica_Puma_560();

punto_pick = [-0.5, 0.4, 0.3, 0, pi, pi/2]; %punto di pick nello spazio operativo
punto_place = [0.5, -0.4, 0.3, 0, pi, pi/2]; %punto di place nello spazio operativo

T_pick = transl(punto_pick(1:3)) * eul2tr(punto_pick(4:6));
T_place = transl(punto_place(1:3)) * eul2tr(punto_place(4:6));

q_pick = ikunc(Puma_560, T_pick);
q_place = ikunc(Puma_560, T_place);

posizione_ostacolo = [0, 0, 1];
raggio_ostacolo = 0.5; %[m]

%qualche parametro per la definizione della traiettoria semicircolare
centro_traiettoria = [0, 0, 0.5];
raggio_traiettoria = 0.64;
angolo_arco_circonferenza = pi;
arco_circonferenza = raggio_traiettoria * angolo_arco_circonferenza;

altezza_sicurezza = 0.2; %[m]

punto_pick_safe = punto_pick;
punto_pick_safe(3) = punto_pick_safe(3) + altezza_sicurezza;

punto_place_safe = punto_place;
punto_place_safe(3) = punto_place_safe(3) + altezza_sicurezza;

%tempi [s]
tempo_pick = 5;
tempo_place = 3;
tempo_movimentazione = 2;
tempo_transizione = 6; %movimento tra pick e place

%creazione oggetto sferico

[X, Y, Z] = sphere;
X_sfera = X * raggio_ostacolo;
Y_sfera = Y * raggio_ostacolo;
Z_sfera = Z * raggio_ostacolo;

surf(X_sfera + 0, Y_sfera + 0, Z_sfera + 1)
axis equal

% ------------------------------ 1) PUNTO_PICK_SAFE - PUNTO_PICK ------------------------------

%si effettui una traiettoria rettilinea attraverso la rappresentazione parametrica di un segmento

numero_di_punti_1 = 15;

s_posizione_iniziale = 0;
s_posizione_finale = norm(punto_pick - punto_pick_safe);

%si generi il profilo s
[p1, F] = polynomial_3_modificata(tempo_movimentazione, numero_di_punti_1, [s_posizione_iniziale, 0.0, s_posizione_finale, 0.0]);
s_posizioni = F(1, :); %evoluzione di s. Estraiamo la posizione dalla polinomiale

%l'evoluzione della posizione dell'EE può essere calcolata con l'equazione parametrica del segmento, per ogni valore s che abbiamo generato
for i = 1:length(s_posizioni)
    insieme_x_EE(i, 1:3) = punto_pick_safe(1:3) + (s_posizioni(i) / norm(punto_pick(1:3) - punto_pick_safe(1:3))) ...
    * (punto_pick(1:3) - punto_pick_safe(1:3)); %vettore delle posizioni, in cui si ha colonna x y z
end

%l'orientamento dell'EE deve rimanere fisso
for i = 1:length(insieme_x_EE)
    insieme_x_EE(i, 4:6) = punto_pick(4:6);
end

hold on
scatter3(insieme_x_EE(2:end, 1), insieme_x_EE(2:end, 2), insieme_x_EE(2:end, 3), 'ob') %va a graficare con una successione di marker. Si vanno a plottare successioni di posizioni di coordinate x y z (il primo lo facciamo di un altro colore)
scatter3(insieme_x_EE(1, 1), insieme_x_EE(1, 2), insieme_x_EE(1, 3), 'or') 
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
xlim([-1.0, 1.0])
ylim([-1.0, 1.0])
zlim([-1.0, 1.0])

%si ricavino le matrici di trasformazione dell'EE

for i = 1:length(insieme_x_EE)
    pose(:, :, i) = transl(insieme_x_EE(i, 1:3)) * eul2tr(insieme_x_EE(i, 4:6));
end

%si convertano nello spazio di giunto attraverso la cinematica inversa
q_1 = ikunc(Puma_560, pose);

% ------------------------------ 2) PUNTO_PICK - PUNTO_PICK ------------------------------

%si lavori ora direttamente nello spazio di giunto

numero_di_punti_2 = 15;

for i = 1:6 %per ogni giunto
    [p2, F] = polynomial_3_modificata(tempo_pick, numero_di_punti_2, [q_pick(i), 0.0, q_pick(i), 0.0]);
    q_2(:, i) = F(1, :);
end

% ------------------------------ 3) PUNTO_PICK - PUNTO_PICK_SAFE ------------------------------

%si utilizzi la rappresentazione parametrica di un segmento

numero_di_punti_3 = 15;

s_posizione_iniziale = 0;
s_posizione_finale = norm(punto_pick_safe - punto_pick);

%si generi il profilo s
[p3, F] = polynomial_3_modificata(tempo_movimentazione, numero_di_punti_3, [s_posizione_iniziale, 0.0, s_posizione_finale, 0.0]);
s_posizioni = F(1, :); %evoluzione di s. Estraiamo la posizione dalla polinomiale

%l'evoluzione della posizione dell'EE può essere calcolata con l'equazione parametrica del segmento, per ogni valore s che abbiamo generato
for i = 1:length(s_posizioni)
    insieme_x_EE(i, 1:3) = punto_pick(1:3) + (s_posizioni(i) / norm(punto_pick_safe(1:3) - punto_pick(1:3))) * (punto_pick_safe(1:3) - punto_pick(1:3)); %vettore delle posizioni, in cui si ha colonna x y z
end

%l'orientamento dell'EE deve rimanere fisso
for i = 1:length(insieme_x_EE)
    insieme_x_EE(i, 4:6) = punto_pick(4:6);
end

%si ricavino le matrici di trasformazione dell'EE

for i = 1:length(insieme_x_EE)
    pose(:, :, i) = transl(insieme_x_EE(i, 1:3)) * eul2tr(insieme_x_EE(i, 4:6));
end

%si convertano nello spazio di giunto attraverso la cinematica inversa
q_3 = ikunc(Puma_560, pose);

% ------------------------------ 4) PUNTO_PICK_SAFE - PUNTO_PLACE_SAFE ------------------------------

%si utilizzi la primitiva di movimento della circonferenza

%si definisca come evolve il parametro s
s_pos_start = 0;
s_pos_end = arco_circonferenza;

%si generi ora il profilo di s
[p, F] = polynomial_5(tempo_transizione, [s_pos_start, 0.0, 0.0, s_pos_end, 0.0, 0.0]);
s_pos = F(1, :); %evoluzione di s. Estraiamo la posizione dalla polinomiale

%l'evoluzione della posizione può essere calcolata con l'equazione parametrica della circonferenza, per ogni valore s che abbiamo generato
for i=1:length(s_pos)
    insieme_x_EE(i, 1) = raggio_traiettoria * cos(s_pos(i)/raggio_traiettoria);
    insieme_x_EE(i, 2) = raggio_traiettoria * sin(s_pos(i)/raggio_traiettoria);
    insieme_x_EE(i, 3) = centro_traiettoria(3);
end

%si ruoti di 119° il SdR della traiettoria circolare
insieme_x_EE(:, 1:3) = insieme_x_EE(:, 1:3) * rotz((73*pi)/60);

%aggiungiamo l'orientamento
for i = 1:length(insieme_x_EE)
    insieme_x_EE(i, 4:6) = [0, pi, pi/2];
end

hold on
scatter3(insieme_x_EE(2:end, 1), insieme_x_EE(2:end, 2), insieme_x_EE(2:end, 3), 'ob') %va a graficare con una successione di marker. Si vanno a plottare successioni di posizioni di coordinate x y z (il primo lo facciamo di un altro colore)
scatter3(insieme_x_EE(1, 1), insieme_x_EE(1, 2), insieme_x_EE(1, 3), 'or') 
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
xlim([-1.0, 1.0])
ylim([-1.0, 1.0])
zlim([-1.0, 1.0])

%si individuino le varie matrici di trasformazione/pose
for i = 1:length(insieme_x_EE)
    trasformazioni(:, :, i) = transl(insieme_x_EE(i, 1:3)) * eul2tr(insieme_x_EE(i, 4:6));
end

q_4 = ikunc(Puma_560, trasformazioni);

% ------------------------------ 5) PUNTO_PLACE_SAFE - PUNTO_PLACE ------------------------------

%si utilizzi la rappresentazione parametrica di un segmento

numero_di_punti_5 = 15;

s_posizione_iniziale = 0;
s_posizione_finale = norm(punto_place - punto_place_safe);

%si generi il profilo s
[p5, F] = polynomial_3_modificata(tempo_movimentazione, numero_di_punti_5, [s_posizione_iniziale, 0.0, s_posizione_finale, 0.0]);
s_posizioni = F(1, :); %evoluzione di s. Estraiamo la posizione dalla polinomiale


insieme_x_EE = [];
%l'evoluzione della posizione dell'EE può essere calcolata con l'equazione parametrica del segmento, per ogni valore s che abbiamo generato
for i = 1:length(s_posizioni)
    insieme_x_EE(i, 1:3) = punto_place_safe(1:3) + (s_posizioni(i) / norm(punto_place(1:3) - punto_place_safe(1:3))) * (punto_place(1:3) - punto_place_safe(1:3)); %vettore delle posizioni, in cui si ha colonna x y z
end

%l'orientamento dell'EE deve rimanere fisso
for i = 1:length(insieme_x_EE)
    insieme_x_EE(i, 4:6) = punto_pick(4:6);
end

hold on
scatter3(insieme_x_EE(2:end, 1), insieme_x_EE(2:end, 2), insieme_x_EE(2:end, 3), 'ob') %va a graficare con una successione di marker. Si vanno a plottare successioni di posizioni di coordinate x y z (il primo lo facciamo di un altro colore)
scatter3(insieme_x_EE(1, 1), insieme_x_EE(1, 2), insieme_x_EE(1, 3), 'or') 
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
xlim([-1.0, 1.0])
ylim([-1.0, 1.0])
zlim([-1.0, 1.0])

%si ricavino le matrici di trasformazione dell'EE

for i = 1:length(insieme_x_EE)
    pose(:, :, i) = transl(insieme_x_EE(i, 1:3)) * eul2tr(insieme_x_EE(i, 4:6));
end

%si convertano nello spazio di giunto attraverso la cinematica inversa
q_5 = ikunc(Puma_560, pose);

% ------------------------------ 6) PUNTO_PLACE - PUNTO_PLACE ------------------------------

%si lavori ora nello spazio di giunto

numero_di_punti_6 = 15;

for i = 1:6 %per ogni giunto
    [p6, F] = polynomial_3_modificata(tempo_place, numero_di_punti_6, [q_place(i), 0.0, q_place(i), 0.0]);
    q_6(:, i) = F(1, :);
end

q_target = [q_1; q_2; q_3; q_4; q_5; q_6];

%%
for i = 1:length(q_target)
    Puma_560.plot(q_target(i, :));
end