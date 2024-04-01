clear; clc;

A = [2, -11, 12, 31;
     -3, 13, -11, -33;
     4, -25, 14, 51;
     -3, 16, -11, -36];
 
B = [-1, -4;
     1, 6;
     -1, -11;
     1, 7];
 
u_openLoop = [1;
              1];
          
[eigen_vectors, eigen_values] = eig(A)

x = [1;
     -1;
     0;
     2];
 
data1 = x(1);
data2 = x(2);
data3 = x(3);
data4 = x(4);

% Inicio de bucle
t= 0.0;
dt = 0.01;
t_array = t;
while t <= 10
    
    if t < dt
        x = x + (A*x)*dt;
    else
        x = x + (A*x + B*u_openLoop) * dt;
    end
    
    data1 = [data1 x(1)];
    data2 = [data2 x(2)];
    data3 = [data3 x(3)];
    data4 = [data4 x(4)];
    t_array = [t_array t];
    
    t = t + dt;
end

figure
plot(t_array, data1,t_array, data2, t_array, data3, t_array, data4)