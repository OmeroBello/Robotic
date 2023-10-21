function [p, F] = polynomial_5(T, Q0) %p dà punti che descrivono asse x (tempo), mentre F è vettore di vettori, con evoluzione posizione, velocità, accelerazione, jerk (variazioni di accelerazione). T tempo, Q0 condizioni al contorno
    f_0 = Q0(1); %posizione iniziale
    df_0 = Q0(2); %velocità iniziale
    dff_0 = Q0(3); %accelerazione iniziale
    f_T = Q0(4); %posizione finale
    df_T = Q0(5); %velcoità finale
    dff_T = Q0(6); %accelerazione finale

    a_0 = f_0;
    a_1 = df_0;
    a_2 = dff_0/2;
    a_3 = - (20*f_0 - 20*f_T + 3*dff_0*T^2 - dff_T*T^2 + 12*T*df_0 + 8*T*df_T)/ (2*T^3);
    a_4 = - (-30*f_0 + 30*f_T - 16*df_0*T - 14*df_T*T - 3*dff_0*T^2 + 2*dff_T*T^2) / (2*T^4);
    a_5 = - (-12*f_0 - 12*f_T + 6*df_0*T + 6*df_T*T - dff_0*T^2 + dff_T*T^2) / (2*T^5);

    NPoints = 75; %numeri di punti per discretizzare il tutto
    p = 0:T/NPoints:T; %discretizzazione del tempo

    f = a_0 + a_1 * p + a_2 * p.*p + a_3 * p.*p.*p + a_4 * p.*p.*p.*p + a_5 * p.*p.*p.*p.*p; %il punto . prima della moltiplicazione significa moltiplicare ogni valore del vettore
    df = a_1 + 2*a_2*p +3*a_3*p.*p + 4*a_4*p.*p.*p + 5*a_5*p.*p.*p.*p;
    ddf = 2*a_2 + 6*a_3*p + 12*a_4*p.*p + 20*a_5*p.*p.*p;
    dddf = 6*a_3 + 24*a_4*p + 60*a_5*p.*p; %lo vogliamo rendere un vettore

    F = [f;df;ddf;dddf]; %è una matrice!