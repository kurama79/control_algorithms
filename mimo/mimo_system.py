# Tarea 1: Estabilizacion de un sistema lineal de multiples entradas

import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt 

# Funcion para graficar la respuesta de los estados en Lazo Abierto
def open_loop_plot(t_array, data1, data2, data3, data4):

	fig1, ax1 = plt.subplots()
	ax1.set_title('Respuesta del sistema en Lazo Abierto')
	ax1.set_ylim(-20, 30)
	ax1.set_xlim(-0.01, 10)

	ax1.plot(t_array, data1)
	ax1.plot(t_array, data2)
	ax1.plot(t_array, data3)
	ax1.plot(t_array, data4)

	# plt.show()

# Funcion para graficar la respuesta de los estados en Lazo Cerrado
def close_loop_plot(t_array, data1, data2, data3, data4):

	fig2, ax2 = plt.subplots()
	ax2.set_title('Respuesta del sistema en Lazo Cerrado (retro de estados)')
	ax2.set_ylim(-25, 35)
	ax2.set_xlim(-0.01, 10)

	ax2.plot(t_array, data1)
	ax2.plot(t_array, data2)
	ax2.plot(t_array, data3)
	ax2.plot(t_array, data4)

# Funcion para graficar la respuesta de los controles
def control_plot(t_array, u1, u2):

	fig3, ax3 = plt.subplots()
	ax3.set_title('Respuesta de los controles u1 y u2')
	ax3.set_ylim(-30, 10)
	ax3.set_xlim(-0.01, 10)

	ax3.plot(t_array, u1)
	ax3.plot(t_array, u2)

	plt.show()

# Funcion principal...
if __name__ == '__main__':

	A = np.array([[  2, -11,  12,  31],
			 	  [ -3,  13, -11, -33],
			 	  [  4, -25,  14,  51],
			 	  [ -3,  16, -11, -36]])

	B = np.array([[ -1,  -4],
			 	  [  1,   6],
			 	  [ -1, -11],
			 	  [  1,   7]])

	# Lazo abierto.....

	u_openLoop = np.array([[0],
						   [0]])

	eigen_values, eigen_vectors = LA.eig(A) 

	print('Los valores propios del sistema son: ', eigen_values, '\n')

	# Condiciones iniciales
	x = np.array([[ 1],
				  [-1],
				  [ 0], 
				  [ 2]])

	xc = x

	openLoopData1 = [x[0][0]]
	openLoopData2 = [x[1][0]]
	openLoopData3 = [x[2][0]]
	openLoopData4 = [x[3][0]]

	closeLoopData1 = [xc[0][0]]
	closeLoopData2 = [xc[1][0]]
	closeLoopData3 = [xc[2][0]]
	closeLoopData4 = [xc[3][0]]

	# Verificando controlabilidad
	b_rank = LA.matrix_rank(B) # Rango columna de B
	a_rank = LA.matrix_rank(A) # Rango de A
	print('El rango de B es: ', b_rank, '\n')

	B1 = B
	B2 = A @ B
	B3 = A @ B2
	B4 = A @ B3
	C = np.concatenate((B1, B2, B3, B4), axis=1)
	C_rank = LA.matrix_rank(C)
	print('La matriz de controlabilidad del sistema:\n', C, '\n Su rango es: ', C_rank, '\n')

	m_1 = b_rank # Indices de controlabilidad
	m_2 = C_rank - m_1
	print('Los indices de controlabilidad del sistema son: \n m_1: ', m_1, '\n m_2: ', m_2, '\n')
	m = max([m_1, m_2])
	print('Por lo que m_1 + m_2 = ', m_1 + m_2, '\n Y su indice de controlabilidad es: ', m, '\n')

	if C_rank == len(A): # Verificando si es controlable
		print('El sistema SI es controlable!')
	else:
		print('El sistema NO es controlable')

	# Transformacion a forma canonica
	c1 = np.array([[B[:,0][0]], [B[:,0][1]], [B[:,0][2]], [B[:,0][3]]])
	c2 = A @ c1
	c3 = np.array([[B[:,1][0]], [B[:,1][1]], [B[:,1][2]], [B[:,1][3]]])
	c4 = A @ c3
	C_simply = np.concatenate((c1, c2, c3, c4), axis=1) # Obtenemos la matriz de transformacion

	M = LA.inv(C_simply)
	# M = np.transpose(M)
	e1 = M[1]
	e2 = M[3]
	print('Matriz M para obtener T', M)

	T =  np.array([e1,
				   e1 @ A,
				   e2,
				   e2 @ A])
	T_rank = LA.matrix_rank(T)
	print('La matriz de transformacion es:\n',LA.inv(T), '\n Su rango es: ', T_rank, '\n')
	
	# Obtenemos la forma Canonica Controlador
	Ac = T @ A @ LA.inv(T)
	Bc = T @ B
	print('Matriz A en forma canonica:\n', Ac, '\n')
	print('Matriz B en forma canonica:\n', Bc, '\n')

	eigen_values_c, eigen_vectors_c = LA.eig(Ac)
	print('Lo valores propios del sistema transformado son: \n', eigen_values_c)

	# Diseño de controlador (Bass-Gura)
	a1 = np.poly(eigen_values[0:2])[::-1] # Coeficientes del polinomio caracteristico
	a2 = np.poly(eigen_values[2::])[::-1]
	alpha1 = np.poly((-3.+3.j, -3.-3.j))[::-1] # Polos deseados
	alpha2 = np.poly((-5., -5.))[::-1]

	k_B = np.array([Bc[1],
					Bc[3]])

	pol_diff = np.array([[alpha1[0] - a1[0], alpha1[1] - a1[1], 0, 0],
						 [0, 0, alpha2[0] - a2[0], alpha2[1] - a2[1]]])

	K = k_B @ pol_diff
	print('La matriz de ganacia K es: ', K, '\n')
	
	# Comprobacion de K
	K = K @ T
	check_mat = A - (B @ K)
	check_poly = np.poly(check_mat)
	check_eigen, vec = LA.eig(check_mat)
	print('Los valores propios y polinomio del nuevo sistema:\n\t eigen_values = ', check_eigen)
	print('\tCoeficientes de polinomio = ', check_poly)

	# Control
	u_closeLoop = -K @ xc
	u1_data = [u_closeLoop[0][0]]
	u2_data = [u_closeLoop[1][0]]

	# Inicio de ciclo
	t = 0.0
	dt = 0.01
	t_array = [t]
	while t <= 10:

		# Sistema en Lazo Abierto
		if t < dt:
			x = x + (A@x)*dt

		else:
			x = x + (A@x + B@u_openLoop)*dt

		openLoopData1.append(x[0][0])
		openLoopData2.append(x[1][0])
		openLoopData3.append(x[2][0])
		openLoopData4.append(x[3][0])

		# Sistema en Lazo Cerrado
		xc = xc + (A@xc + B@u_closeLoop) * dt
		u_closeLoop = -K @ xc

		closeLoopData1.append(xc[0][0])
		closeLoopData2.append(xc[1][0])
		closeLoopData3.append(xc[2][0])
		closeLoopData4.append(xc[3][0])
		u1_data.append(u_closeLoop[0][0])
		u2_data.append(u_closeLoop[1][0])

		# Tiempo
		t_array.append(t)
		t += dt

	open_loop_plot(t_array, openLoopData1, openLoopData2, openLoopData3, openLoopData4)
	close_loop_plot(t_array, closeLoopData1, closeLoopData2, closeLoopData3, closeLoopData4)
	control_plot(t_array, u1_data, u2_data)