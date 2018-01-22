close all
clear all

%-------------------------------------------------------------------------%
% 
% To obtain Gravitation Field (raw format):
% 1) get the Total Field for your location from here:
%    http://www.ngdc.noaa.gov/geomag-web (tab Magneti3c Field)
%    es. Total Field = 47,241.3 nT
% 2) Convert this values to Gauss (1nT = 10E-5G)
%    es. Total Field = 47,241.3 nT = 0.47241G
% 3) Convert Total Field to Raw value Total Field, which is the
%    Raw Gravitation Field we are searching for
%    Read your magnetometer datasheet and find your gain value,
%    Which should be the same of the collected raw points
%    es. on HMC5883L, given +_ 1.3 Ga as Sensor Field Range settings
%        Gain (LSB/Gauss) = 1090
%        Raw Total Field = Gain * Total Field
%        0.47241 * 1090 = ~515LSB
% 
% Say in Kelowna:
% 1)	The Total Field: 55,092.4 nT
% 2)	Covert this values to Gauss: 55,092.4 nT = 0.551 G
% 3)	Convert Total Field to Raw Value Total Field, essentially the raw Gravitation Field
% a.	On AK8963, given  4,800,000 nT = 48 G
% b.	Gain = 1/[0.15(?T/ LSB)] = 6.67(LSB/?T) = 0.00667(LSB/nT) = 667(LSB/Gauss)
        %(0.6?/LSB is for 14-bit, 0.15?/LSB is for 16-bit, the magnetometer
        %use 16-bit, see MPU9250.h for detals)
% c.	Raw Total Field = Gain * Total Field = 0.551 * 667 = 367.5170LSB
% 4)	The value read from the magnetometer we used is the magnetometer values in milliGauss
%
%-------------------------------------------------------------------------%

mag_raw = 367.5170; % the unit is in LSB, this value is 367.5170 (16-bit, 92 for 14-bit) for Kelowna

% read data obtained from the magnetometer
f_id = fopen('With_Iron_3cm.txt','r'); 
sample = fscanf(f_id,'%f');
data_mG = vec2mat(sample,3); % the unit is in mGauss
gain = 0.667; % the unit is in LSB/mGauss
data_raw = gain .* data_mG;

% write data_raw to text file
f_id2 = fopen('With_Iron_3cm_raw.txt', 'w'); 
for i=1:size(data_raw,1)
   fprintf(f_id2, '%f ', data_raw(i,:));
   fprintf(f_id2, '\n');
end
fclose(f_id2);

% Build the design matrix D
column_num = length(data_raw);

D(1,:) = data_raw(:,1).^2; % x^2
D(2,:) = data_raw(:,2).^2; % y^2
D(3,:) = data_raw(:,3).^2; % z^2
D(4,:) = 2 .* data_raw(:,2) .* data_raw(:,3); % 2yz
D(5,:) = 2 .* data_raw(:,1) .* data_raw(:,3); % 2xz
D(6,:) = 2 .* data_raw(:,1) .* data_raw(:,2); % 2xy
D(7,:) = 2 .* data_raw(:,1); % 2x
D(8,:) = 2 .* data_raw(:,2); % 2y
D(9,:) = 2 .* data_raw(:,3); % 2z
D(10,:) = ones([1,column_num]); % 1

% Create S matrix
S = D * transpose(D);

% Split the S matrix to 4 sub-matrices S11, S12, S21, and S22
% S21 and transpose(S12)
for m=1:1:6 % row
    for n=1:1:6 %column
        S11(m,n)=S(m,n); % S11
    end
end

for m=1:1:6 
    for n=1:1:4 %column
        S12(m,n)=S(m,n+6); % S12
    end
end  

S21 = transpose(S12); % S21

for m=1:1:4 
    for n=1:1:4 %column
        S22(m,n)=S(m+6,n+6); % S22
    end
end  

% Find the inverse matrix of S22
S22_inv = inv(S22);

% Calculate matrix SS
SS = S11 - S12 * S22_inv * S21;

% Constraint Matrix C
% C = [-1,1,1,0,0,0;1,-1,1,0,0,0;1,1,-1,0,0,0;0,0,0,-4,0,0;0,0,0,0,-4,0;0,0,0,0,0,-4];
% normalized C:
C = [0,0.5,0.5,0,0,0;0.5,0,0.5,0,0,0;0.5,0.5,0,0,0,0;0,0,0,-0.25,0,0;0,0,0,0,-0.25,0;0,0,0,0,0,-0.25];

% Calculate inverse C matrix
C_inv = inv(C);

% Calculate matrix E
E = C * SS;

% Find eigenvalue of matirix E, E*V - V*D = [0]
e = eig(E); % eigenvalue
[V,D] = eig(E); % eigenvector is V

% Find the zero-based position of the only positive eigenvalue. 
% The associated eigenvector will be in the corresponding column of matrix 
maxvalue_e = e(1,1);
for k = 1:1:6 % row of e
    if e(k,1)>0
        maxvalue_e = e(k,1);
        index = k;
        break
    end
end

% Extract the associated eigenvector V1
V1 = V(:,index);

% Check the sign of V1, if it is negative, flip the sign of the eigenvector
if V1(1,1) < 0
    V1 = -1 .* V1;
end

% Calculate V2
V2 = (S22_inv * S21) * V1;

% Setup matrix Ellipsoid, which is what we want!!!
for i = 1:1:6
    for j = 7:1:10
        Ellipsoid(i,1) = V1(i,1);
        Ellipsoid(j,1) = -1 .* V2(j-6,1);
    end
end

% Normalize the Ellipsoid matrix (normalize coefficient of the ellipsoid to prevent trivial results)
Ellipsoid = Ellipsoid ./ norm(Ellipsoid(:));

%-------------------------------------------------------------------------%
% Calculate combined bias
% Setup Q matrix
Q = [Ellipsoid(1,1),Ellipsoid(6,1),Ellipsoid(5,1);Ellipsoid(6,1),Ellipsoid(2,1),Ellipsoid(4,1);Ellipsoid(5,1),Ellipsoid(4,1),Ellipsoid(3,1)];

% Setup U matrix
U = [Ellipsoid(7,1);Ellipsoid(8,1);Ellipsoid(9,1)];

% Calculate inverse Q matrix
Q_inv = inv(Q);

% Calculate B matrix
B = Q_inv * U;

% Combined Bias
x_bias = -B(1,1);
y_bias = -B(2,1);
z_bias = -B(3,1);
bias = repmat([x_bias, y_bias, z_bias],length(data_raw),1); % Hard iron offset

%-------------------------------------------------------------------------%
% Calculate correction matrix A_inv
J = Ellipsoid(10,1);
A_inv = (Q^(0.5)).*((mag_raw)./(sqrt(transpose(B)*Q*B-J)));

%-------------------------------------------------------------------------%
% Calibrated Data
data_cal = transpose(A_inv * transpose(data_raw - bias)); % the matrix dimensions must match 

f_id3 = fopen('With_Iron_3cm_cal.txt', 'w'); % write data_cal to txt file
for i=1:size(data_cal,1)
   fprintf(f_id3, '%f ', data_cal(i,:));
   fprintf(f_id3, '\n');
end
fclose(f_id3);

%-------------------------------------------------------------------------%
% Plot Data
x = data_raw(:,1);
y = data_raw(:,2);
z = data_raw(:,3);
x_cal = data_cal(:,1);
y_cal = data_cal(:,2);
z_cal = data_cal(:,3);

A = Ellipsoid(1,1);
B = Ellipsoid(2,1);
C = Ellipsoid(3,1);
D = Ellipsoid(6,1);
E = Ellipsoid(5,1);
F = Ellipsoid(4,1);
G = Ellipsoid(7,1);
H = Ellipsoid(8,1);
I = Ellipsoid(9,1);
J = Ellipsoid(10,1);

x_c = bias(1,1);
y_c = bias(1,2);
z_c = bias(1,3);
x_r = mag_raw/A_inv(1,1);
y_r = mag_raw/A_inv(2,2);
z_r = mag_raw/A_inv(3,3);
[j,k,l] = ellipsoid(x_c,y_c,z_c,x_r,y_r,z_r,50);

% x_c = ((2*D)*(2*H)-2*B*(2*G))/(4*A*B-(2*D)^2);
% y_c = ((2*D)*(2*G)-2*A*(2*H))/(4*A*B-(2*D)^2)
% y_c2 = ((2*F)*(2*I)-2*C*(2*H))/(4*B*C-(2*F)^2)
% z_c = ((2*F)*(2*H)-2*B*(2*I))/(4*B*C-(2*F)^2);

% myColors = ones(1, length(x));
myColors(1,length(x)) = 1; 
myColors2(1,length(x_cal)) = 0;
% change the "2" to other number to change color:
% -1:yellow; -2:magenta; 0:green; 1:cyan; 2:blue
% http://stackoverflow.com/questions/18646879/matlab-using-scatter3-and-mesh-at-the-same-time
x(isnan(myColors)) = NaN;
x_cal(isnan(myColors2)) = NaN;
[a,b,c] = sphere(50); % 50 is the mesh density

figure(1)
scatter3(x,y,z,10, myColors, 'filled'); % "100" is the size of the bubble
get(gcf, 'Renderer');
hold on;
surf(a*mag_raw+x_bias, b*mag_raw+y_bias, c*mag_raw+z_bias, 'FaceColor', [1 0 0],'EdgeAlpha',0.2,'FaceAlpha', 0.1);
hold off;
legend('Uncalibrated Results','Local Magnetic Field');

figure(2)
scatter3(x_cal,y_cal,z_cal,10, myColors2, 'filled');
hold on;
surf(a*mag_raw, b*mag_raw, c*mag_raw, 'FaceColor', [1 0 0],'EdgeAlpha',0.2,'FaceAlpha', 0.1);
hold off;
legend('Calibrated Results','Local Magnetic Field');

figure(3)
surf(j,k,l,'FaceColor', [1 1 0],'EdgeAlpha',0.2,'FaceAlpha', 0.2);
hold on
surf(a*mag_raw, b*mag_raw, c*mag_raw, 'FaceColor', [1 0 0],'EdgeAlpha',0.2,'FaceAlpha', 0.2);
hold off
legend('Uncalibrated Results (Ellipsoid)','Calibrated Results (Sphere)');
