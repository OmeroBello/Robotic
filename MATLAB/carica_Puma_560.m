function Modello_Puma_560 = carica_Puma_560()
    
    %parametri di Denavit-Hartenberg
    
    d = [0, 0, 0.1501, 0.4331, 0, 0]; %vettore dei parametri d (dei parametri di DH) di ciascun link
    a = [0, 0.4323, 0, 0, 0, 0]; %vettore dei parametri a (dei parametri di DH) di ciascun link
    alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0]; %vettore dei parametri alpha (dei parametri di DH) di ciascun giunto
    offset = [0, 0, 0, 0, 0, 0]; %vettore dei parametri theta (dei parametri di DH) iniziali
    % di ciascun giunto (sarebbe il nostro offset dell'esercitazione, forse)
    
    %parametri dinamici
    
    %parametri di massa per ogni link
    m = [0, 17.4000, 4.8000, 0.8200, 0.3400, 0.0900];
    
    %coordinate dei centri di massa di ogni link [m]
    r_x = [0, -0.3638, -0.0203, 0, 0, 0];
    r_y = [0, 0.0060, -0.0141, 0.0190, 0, 0];
    r_z = [0, 0.2275, 0.0700, 0, 0, 0.0320];

    %inseriamo i momenti d'inerzia lungo gli assi principali
    I_xx = [0, 0.1300, 0.0660, 0.0018, 0.0003, 0.15e-3]; %momento d'inerzia di ogni link lungo l'asse x
    I_yy = [0.3500, 0.5240, 0.0860, 0.0013, 0.0004, 0.15e-3]; %momento d'inerzia di ogni link lungo l'asse y
    I_zz = [0, 0.5390, 0.0125, 0.0018, 0.0003, 0.04e-3]; %momento d'inerzia di ogni link lungo l'asse z

    %fine parametri

    %specifichiamo, per ogni link, i parametri di DH e dinamici appena inseriti
    for i = 1:6 %definiamo ciascun link
        L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', offset(i), 'revolute', ... %revolute per indicare giunti rotoidali
            'm', m(i), 'r', [r_x(i), r_y(i), r_z(i)], ...
            'I', [I_xx(i), I_yy(i), I_zz(i)]);
    end

    Modello_Puma_560 = SerialLink(L, 'name', 'Puma 560'); %crea la catena cinematica completa prendendo tutti i link a nostra disposizione