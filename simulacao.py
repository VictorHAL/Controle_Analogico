import numpy as np
import control as ct
import matplotlib.pyplot as plt

# ==========================================
# 1. CONFIGURAÇÃO (SEUS DADOS CALCULADOS)
# ==========================================
Ts = 0.01
# Ganhos obtidos na Parte B
'''
=== GANHOS CALCULADOS ===
Kp = 0.02698
Ki = 0.00075
Kd = 0.52282
'''
Kp = 0.01633
Ki = 0.00052
Kd = 0.35789

print(f"--- SIMULAÇÃO PARTE C ---")
print(f"Ganhos: Kp={Kp}, Ki={Ki}, Kd={Kd}")

# ==========================================
# 2. DEFINIÇÃO DAS FUNÇÕES DE TRANSFERÊNCIA
# ==========================================

# Planta G(z) - Do teu resultado da Parte A
# G(z) = 0.3246z / (z^2 - 1.999z + 0.9985)
num_g = [0.3246, 0]
den_g = [1, -1.999, 0.9985]
G_z = ct.TransferFunction(num_g, den_g, Ts)

# Controlador C(z) - PID na forma posicional
# C(z) = ( Kp(z-1)z + Ki(z^2) + Kd(z-1)^2 ) / ( z(z-1) )
# Numerador expandido: (Kp + Ki + Kd)z^2 - (Kp + 2Kd)z + Kd
num_c = [ (Kp + Ki + Kd), -(Kp + 2*Kd), Kd ]
den_c = [1, -1, 0] # z(z-1)
C_z = ct.TransferFunction(num_c, den_c, Ts)

# Malha Aberta: L(z) = C(z) * G(z)
L_z = C_z * G_z

# Malha Fechada (Saída vs Referência): T(z) = L(z) / (1 + L(z))
T_z = ct.feedback(L_z, 1)

# Função de Transferência do Controle (U vs Referência): U(z)/R(z) = C(z) / (1 + C(z)G(z))
# Isso nos mostra o esforço que o motor faz para um degrau de erro
U_z = ct.feedback(C_z, G_z)

# ==========================================
# 3. GERAÇÃO DOS GRÁFICOS
# ==========================================
plt.figure(figsize=(10, 12))

# --- GRÁFICO 1: LUGAR DAS RAÍZES (ROOT LOCUS) ---
plt.subplot(3, 1, 1)
ct.root_locus(L_z, grid=True)
# Desenha Círculo Unitário
theta = np.linspace(0, 2*np.pi, 100)
plt.plot(np.cos(theta), np.sin(theta), 'r--', label='Estabilidade (Círculo Unitário)')
plt.title("1. Lugar das Raízes (Root Locus)")
plt.legend()

# --- GRÁFICO 2: RESPOSTA AO DEGRAU (SAÍDA) ---
plt.subplot(3, 1, 2)
time, yout = ct.step_response(T_z, T=4.0) # Simula 4 segundos
plt.plot(time, yout, 'b', linewidth=2, label='Resposta do Robô')
plt.axhline(1.0, color='k', linestyle='--', label='Setpoint (1.0)')
plt.axhline(1.3, color='r', linestyle=':', label='Limite Máximo (30%)')
plt.title("2. Resposta ao Degrau (Posição)")
plt.ylabel("Posição")
plt.xlabel("Tempo (s)")
plt.legend()
plt.grid(True)

# Verificação automática do Sobressinal
max_y = np.max(yout)
sobressinal = (max_y - 1.0) * 100
print(f"Sobressinal Máximo: {sobressinal:.2f}% (Meta: <= 30%)")

# --- GRÁFICO 3: SINAL DE CONTROLE u[k] ---
plt.subplot(3, 1, 3)
# Simulamos o esforço de controle para um erro degrau de 1 unidade (ex: 1 cm ou 1 metro)
time_u, u_out = ct.step_response(U_z, T=4.0)

# Supondo que o degrau seja de 10cm (exemplo prático), multiplicamos a resposta unitária por 10
# Mas para análise puramente teórica, usamos o unitário.
plt.plot(time_u, u_out, 'g', linewidth=2, label='Esforço de Controle (PWM/Tensão)')
plt.title("3. Sinal de Controle u[k] (Esforço do Motor)")
plt.ylabel("Amplitude do Controle")
plt.xlabel("Tempo (s)")
plt.grid(True)

# Análise de Pico de Controle
max_u = np.max(np.abs(u_out))
print(f"Pico do Sinal de Controle: {max_u:.2f}")
print("NOTA: Se este valor for muito alto, o motor vai saturar (cortar o pico).")

plt.tight_layout()
plt.show()