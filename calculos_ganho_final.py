import numpy as np
import control as ct

print("--- PROJETO DE CONTROLADOR PID - BARATINHA (PARTE B) ---")

# ==========================================
# 1. DEFINIÇÃO DA PLANTA G(z) (Dados do utilizador)
# ==========================================
# G(z) = (0.3246 * z) / (z^2 - 1.999z + 0.9985)
K_planta = 0.3246
den_p1 = -1.999
den_p0 = 0.9985

print(f"Planta G(z) utilizada: {K_planta}z / (z^2 {den_p1}z + {den_p0})")

# ==========================================
# 2. CÁLCULO DO POLINÔMIO DESEJADO
# ==========================================
# Requisitos
Ts = 0.01
Mp_target = 0.30  # 30%
ts_target = 3.0   # 3 segundos

# Cálculos de Zeta e Wn
ln_mp = np.log(Mp_target)
zeta = np.abs(ln_mp) / np.sqrt(np.pi**2 + ln_mp**2)
wn = 4 / (zeta * ts_target)

# Ajuste de segurança (fator 1.2 conforme boa prática)
zeta_proj = 0.8 # Arredondando para cima para garantir menos oscilação
wn_proj = wn * 1.2 

# Polos Dominantes (s -> z)
s_real = -zeta_proj * wn_proj
s_imag = wn_proj * np.sqrt(1 - zeta_proj**2)
z1 = np.exp((s_real + 1j*s_imag) * Ts)
z2 = np.exp((s_real - 1j*s_imag) * Ts)

# Polo Auxiliar (rápido)
# Parte real 5x maior que a dos dominantes
z3 = np.exp(5 * s_real * Ts)

print(f"\nPolos Desejados em Z:")
print(f"z1: {z1:.4f}")
print(f"z2: {z2:.4f}")
print(f"z3: {z3:.4f}")

# Polinômio Desejado: (z-z1)(z-z2)(z-z3) = z^3 + d2*z^2 + d1*z + d0
poly_desejado = np.poly([z1, z2, z3])
d2_alvo = poly_desejado[1].real
d1_alvo = poly_desejado[2].real
d0_alvo = poly_desejado[3].real

print(f"\nPolinômio Desejado (D(z)): z^3 + ({d2_alvo:.4f})z^2 + ({d1_alvo:.4f})z + ({d0_alvo:.4f})")

# ==========================================
# 3. MONTAGEM E RESOLUÇÃO DO SISTEMA LINEAR
# ==========================================
# A Equação Característica da Malha Fechada é derivada expandindo 1 + C(z)G(z) = 0
# Após álgebra, obtemos os coeficientes em função de Kp, Ki, Kd:
# Coef z^2:  -2.999 + K(Kp + Ki + Kd)  = d2_alvo
# Coef z^1:   2.9975 + K(-Kp - 2Kd)    = d1_alvo
# Coef z^0:  -0.9985 + K(Kd)           = d0_alvo
# (Onde K é o ganho do numerador da planta = 0.3246)

# Termos constantes da planta (que passam para o lado direito 'b')
# O denominador da planta em malha aberta expandido com o integrador (z-1) gera:
# z^3 - 2.999z^2 + 2.9975z - 0.9985
cte_z2 = den_p1 - 1       # -1.999 - 1 = -2.999
cte_z1 = den_p0 - den_p1  # 0.9985 - (-1.999) = 2.9975
cte_z0 = -den_p0          # -0.9985

# Montagem da Matriz A [Kp, Ki, Kd]
# Linha 1 (z^2): K*Kp + K*Ki + K*Kd
# Linha 2 (z^1): -K*Kp + 0   - 2K*Kd
# Linha 3 (z^0): 0     + 0   + K*Kd
K = K_planta

A = np.array([
    [K,  K,  K],
    [-K, 0, -2*K],
    [0,  0,  K]
])

# Montagem do Vetor b (Alvo - Constantes da Planta)
b = np.array([
    d2_alvo - cte_z2,
    d1_alvo - cte_z1,
    d0_alvo - cte_z0
])

print("\n--- RESOLUÇÃO DO SISTEMA ---")
print("Matriz A:\n", A)
print("Vetor b:\n", b)

# Resolve x = inv(A) * b
ganhos = np.linalg.solve(A, b)
Kp, Ki, Kd = ganhos

print("\n=== GANHOS CALCULADOS ===")
print(f"Kp = {Kp:.5f}")
print(f"Ki = {Ki:.5f}")
print(f"Kd = {Kd:.5f}")

# ==========================================
# 4. VERIFICAÇÃO RÁPIDA (STEP RESPONSE)
# ==========================================
# Validar se os ganhos realmente estabilizam
# Controlador C(z)
num_pid = [Kp+Kd+Ki, -(Kp+2*Kd), Kd]
den_pid = [1, -1, 0] # z(z-1)
C_z = ct.TransferFunction(num_pid, den_pid, Ts)

# Planta G(z)
G_z = ct.TransferFunction([K_planta, 0], [1, den_p1, den_p0], Ts)

# Malha Fechada
H_z = ct.feedback(C_z * G_z, 1)

print("\nPolos da Malha Fechada (Validação):")
print(ct.poles(H_z))