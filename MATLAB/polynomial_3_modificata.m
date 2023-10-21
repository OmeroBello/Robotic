function [p, F] = polynomial_3_modificata(T, NPoints, Q0) %p dà punti che descrivono asse x (tempo), mentre F è vettore di vettori, con evoluzione posizione, velocità, accelerazione, jerk (variazioni di accelerazione). T tempo, Q0 condizioni al contorno
    f_0 = Q0(1); %posizione iniziale
    df_0 = Q0(2); %velocità iniziale
    f_T = Q0(3); %posizione finale
    df_T = Q0(4); %velocità finale

    a_0 = f_0;
    a_1 = df_0;
    a_2 = (3*(f_T - f_0) - T * (2*df_0 + df_T)) / T^2;
    a_3 = - (2 * (f_T - f_0) - T * (df_0 + df_T)) / T^3;

    p = 0:T/NPoints:T; %discretizzazione del tempo

    f = a_0 + a_1 * p + a_2 * p.*p + a_3 * p.*p.*p; %il punto . prima della moltiplicazione significa moltiplicare ogni valore del vettore
    df = a_1 + 2*a_2*p+3*a_3*p.*p;
    ddf = 2*a_2 + 6*a_3*p;
    dddf = 6*a_3*ones(1, length(ddf)); %non dipende dal tempo, ma comunque lo vogliamo rendere un vettore, di dimensione del vettore p

    F = [f;df;ddf;dddf]; %è una matrice!