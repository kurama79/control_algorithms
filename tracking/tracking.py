# Tarea 1: Estabilizacion de un sistema lineal de multiples entradas

import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt 

# Funcion para graficar la respuesta de los estados en Lazo Abierto
def open_loop_plot(t_array, data1, data2, data3, data4):

	fig1, ax1 = plt.subplots()
	ax1.set_title('Respuesta del sistema en Lazo Abierto')
	ax1.set_ylim(-50, 50)
	ax1.set_xlim(-0.01, 10)

	ax1.plot(t_array, data1, label='x1')
	ax1.plot(t_array, data2, label='x2')
	ax1.plot(t_array, data3, label='x3')
	ax1.plot(t_array, data4, label='x4')

	ax1.legend()

# Funcion para graficar la respuesta de los estados en Lazo Cerrado
def close_loop_plot(t_array, data1, data2, data3, data4):

	fig2, ax2 = plt.subplots()
	ax2.set_title('Respuesta del sistema controlado')
	ax2.set_ylim(-2, 2)
	ax2.set_xlim(-0.01, 10)

	ax2.plot(t_array, data1, label='xc1')
	ax2.plot(t_array, data2, label='xc2')
	ax2.plot(t_array, data3, label='xc3')
	ax2.plot(t_array, data4, label='xc4')

	ax2.legend()

# Funcion para graficar la respuesta de los controles
def control_plot(t_array, u1, u2):

	fig3, ax3 = plt.subplots()
	ax3.set_title('Respuesta de los controles u1 y u2')
	ax3.set_ylim(-20, 4)
	ax3.set_xlim(-0.01, 10)

	ax3.plot(t_array, u1, label='u1')
	ax3.plot(t_array, u2, label='u2')

	ax3.legend()

# Funcion para graficar la respuesta de los errores
def error_plot(t_array, e1, e2, e3, e4):

	fig4, ax4 = plt.subplots()
	ax4.set_title('Respuesta de los errores')
	ax4.set_ylim(-10, 2)
	ax4.set_xlim(-0.01, 10)

	ax4.plot(t_array, e1, label='e1')
	ax4.plot(t_array, e2, label='e2')
	ax4.plot(t_array, e3, label='e3')
	ax4.plot(t_array, e4, label='e4')

	ax4.legend()

# Funcion para graficar la respuesta de las salidas y referencias
def output_plot(t_array, y1, y2, ref1, ref2):

	fig5, ax5 = plt.subplots()
	ax5.set_title('Respuesta de las salidas y referecnias')
	ax5.set_ylim(-2, 1.5)
	ax5.set_xlim(-0.01, 10)

	ax5.plot(t_array, y1, label='y1')
	ax5.plot(t_array, y2, label='y2')
	ax5.plot(t_array, ref1, '--', label='ref1')
	ax5.plot(t_array, ref2, '--', label='ref2')

	ax5.legend()

# Funcion principal...
if __name__ == '__main__':

	A = np.array([[  1,  -1,   0,   0],
			 	  [  0,   2,   0,   0],
			 	  [  0,   0,   2,   1],
			 	  [  0,   0,   0,  -1]])

	B = np.array([[  0,   1],
			 	  [ -1,   0],
			 	  [ -1,   1],
			 	  [  1,   2]])

	C = np.array([[ 1,  1, -1,  0],
				  [-2,  1,  0,  1]])

	# Condiciones iniciales
	t = 0.0

	x = np.array([[ 1],
				  [-1],
				  [ 0], 
				  [ 2]])

	ref = np.array([[1],
					[0],
					[np.sin(2*t)],
					[2*np.cos(2*t)]])

	dref = np.array([[0],
					 [0],
					 [2*np.cos(2*t)],
					 [-4*np.sin(2*t)]])

	xe = x

	# Obtenemos la matriz de transformacion por medio de la matriz de observabilidad, reorganizada
	c1 = C[0]
	c2 = c1 @ A
	c3 = C[1]
	c4 = c3 @ A
	T = np.array([c1, c2, c3, c4]) # Obtenemos la matriz de transformacion

	print('La matriz de transformacion es:\n',LA.inv(T), '\n')
	
	# Obtenemos la forma Canonica Controlador
	Ae = T @ A @ LA.inv(T)
	Be = T @ B
	errors = T @ xe - ref
	ref_e = Ae @ ref
	print('Matriz Ae en forma para el error:\n', Ae, '\n')
	print('Matriz Be en forma para el error:\n', Be, '\n')
	print('Vector de errores "e":\n', errors, '\n')
	print('Vector de perturbaciones por referecnias "yr_e":\n', ref_e, '\n')

	# Diseño de controlador por control auxiliar
	Ae_simply = np.array([Ae[1],
						  Ae[3]])
	Br = np.array([Be[1],
				   Be[3]])
	dref_simply = np.array([dref[1],
				   			dref[3]])
	invBr = LA.inv(Br)

	print('Matriz simple Ae en forma para el error:\n', Ae_simply, '\n')
	print('Matriz simple Br en forma para el error:\n', Br, '\n')
	print('Vector simple de referencias dinamica "dyr":\n', dref_simply, '\n')
	print('El determinante de Br es: ', LA.det(Br), '\n')
	print('Su inversa es:\n', invBr, '\n')
	
	# new_error_1 = np.array([[-1, 0], # Primeros polos propuestos
	# 					   [0, -2]])
	# new_error_1 = np.array([[-1.8, 0], # Mejora para controles auxiliares
	# 					  [0, -1.7]])
	new_error_1 = np.array([[-10, 0], # Mejora para reubicacion de polos
						  [0, -15]])
	poly_new_error_1 = np.poly(new_error_1)
	# new_error_2 = np.array([[-3, 0], # Primeros polos propuestos
	# 					    [0, -4]])
	# new_error_2 = np.array([[-1.5, 0], # Mejora para controles auxiliares
	# 					  [0, -3]])
	new_error_2 = np.array([[-7, 0], # Mejora para reubicacion de polos
						  [0, -8]])
	poly_new_error_2 = np.poly(new_error_2)
	print('El nuevo polinomio (parte 1) de la dinamica propuesta es: ', poly_new_error_1)
	print('El nuevo polinomio (parte 2) de la dinamica propuesta es: ', poly_new_error_2, '\n')

	K_error = np.array([poly_new_error_1[1], poly_new_error_1[2], poly_new_error_2[1], poly_new_error_2[2]])
	# K_error = np.array([2, 3, 4, 5])
	print('La ganncias K para la nueva dinamica son: ', K_error, '\n')

	v_1 = -K_error[0]*errors[0] - K_error[1]*errors[1] # Controles auxiliares
	v_2 = -K_error[2]*errors[2] - K_error[3]*errors[3]
	V = np.array([[v_1],
				  [v_2]])
	print('Los controles auxiliares son:\n', V, '\n')

	# Diseño de controlador (Bass-Gura)
	eigen_values, eigen_vectors = LA.eig(Ae) 
	print('Los valores propios del sistema son: ', eigen_values, '\n')

	a1 = np.poly(eigen_values[0:2])[::-1] # Coeficientes del polinomio caracteristico
	a2 = np.poly(eigen_values[2::])[::-1]
	alpha1 = poly_new_error_1[::-1] # Polos deseados
	alpha2 = poly_new_error_2[::-1]

	k_B = invBr
	pol_diff = np.array([[alpha1[0] - a1[0], alpha1[1] - a1[1], 0, 0],
						 [0, 0, alpha2[0] - a2[0], alpha2[1] - a2[1]]])

	K = k_B @ pol_diff
	print('La matriz de ganacia K es: ', K, '\n')

	# Arreglos para guardar datos
	x1_data = []
	x2_data = []
	x3_data = []
	x4_data = []

	x1_c_data = []
	x2_c_data = []
	x3_c_data = []
	x4_c_data = []

	y1_data = []
	y2_data = []

	ref_1_data = []
	ref_2_data = []

	e_1_data = []
	e_2_data = []
	e_3_data = []
	e_4_data = []

	u1_data = []
	u2_data = []

	# Inicio de ciclo
	dt = 0.01
	t_array = []
	while t <= 10:

		# Sistema en Lazo Abierto
		x = x + (A@x)*dt

		x1_data.append(x[0][0])
		x2_data.append(x[1][0])
		x3_data.append(x[2][0])
		x4_data.append(x[3][0])

		# Obtenemos controles
		ref = np.array([[1],
						[0],
						[np.sin(2*t)],
						[2*np.cos(2*t)]])

		ref_1_data.append(ref[0][0])
		ref_2_data.append(ref[2][0])

		dref = np.array([[0],
						 [0],
					 	 [2*np.cos(2*t)],
					 	 [-4*np.sin(2*t)]])

		errors = T @ xe - ref

		e_1_data.append(errors[0][0])
		e_2_data.append(errors[1][0])
		e_3_data.append(errors[2][0])
		e_4_data.append(errors[3][0])

		ref_T = Ae @ ref

		v_1 = -K_error[0]*errors[0] - K_error[1]*errors[1] # Controles auxiliares
		v_2 = -K_error[2]*errors[2] - K_error[3]*errors[3]

		# Control por controles auxiliares
		fila1 = -Ae[1]@errors - ref_T[1] + v_1 + dref[1]
		fila2 = -Ae[3]@errors - ref_T[3] + v_2 + dref[3]

		# u = invBr @ np.array([fila1, fila2])

		# Control por Bass-Gura
		u = -K @ errors + invBr @ np.array([[-ref_T[1][0] + dref[1][0]], [-ref_T[3][0] + dref[3][0]]])

		u1_data.append(u[0][0])
		u2_data.append(u[1][0])

		# Sistema en Lazo Cerrado
		xe = xe + (A@xe + B@u) * dt

		x1_c_data.append(xe[0][0])
		x2_c_data.append(xe[1][0])
		x3_c_data.append(xe[2][0])
		x4_c_data.append(xe[3][0])

		# Salidas del sistema en lazo cerrado
		y = C @ xe

		y1_data.append(y[0][0])
		y2_data.append(y[1][0])

		# Tiempo
		t_array.append(t)
		t += dt

	open_loop_plot(t_array, x1_data, x2_data, x3_data, x4_data)
	close_loop_plot(t_array, x1_c_data, x2_c_data, x3_c_data, x4_c_data)
	control_plot(t_array, u1_data, u2_data)
	error_plot(t_array, e_1_data, e_2_data, e_3_data, e_4_data)
	output_plot(t_array, y1_data, y2_data, ref_1_data, ref_2_data)
	plt.show()