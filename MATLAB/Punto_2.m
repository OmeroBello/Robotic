%Punto 2
close all;
clear;
clc;

%componenti dell'asse
r_x = 0.2639;
r_y = 0.7037;
r_z = 0.6597;

theta_asse_angolo = (3*pi)/7; %angolo di rotazione attorno all'asse

%prima di tutto, si trovino gli angoli di rotazione delle rotazioni elementari attorno agli assi Z e Y, alpha e beta

alpha = asin(r_y/(sqrt(r_x^2 + r_y^2)));
beta = acos(r_z);

%successimanete, si indivui la matrice di rotazione complessiva, andando a premoltiplicare le varie rotazioni elementari che compongono la
%convenzione asse angolo

R = rotz(alpha)*roty(beta)*rotz(theta_asse_angolo)*roty(-beta)*rotz(-alpha)

%infine, da tale matrice di rotazione, si vadano ad inviduare i suoi tre angoli di Eulero con convenzione ZYZ

%prima di tutto, si cerchi di capire se sen(theta) (con theta angolo della convenzione ZYZ) è positivo o negativo.

sen_theta = sqrt(R(1,3)^2 + R(2,3)^2);

%essendo positivo, si proceda con i calcoli per l'ottenimento di phi, theta, psi della convenzione ZYZ, nel caso sen(theta) > 0

phi = atan2(R(2,3),R(1,3))
theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2),R(3,3))
psi = atan2(R(3,2),-R(3,1))

%per scaramanzia, si ricavi nuovamente la matrice di rotazione, questa volta attraverso gli angoli di Eulero, e si controlli che risulti uguale
%alla prima matrice di rotazione. La si calcoli andando a postmoltiplicare le matrici di rotazione elementari attorno a Z, Y, Z

R_ZYZ = rotz(phi)*roty(theta)*rotz(psi)

%risultano uguali