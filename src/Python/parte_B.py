#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PARTE B - PROJETO POR ALOCACAO DE POLOS
Trabalho Vivencial - Controle Digital - UNIFOR

CORRIGIDO: Polinomio e de ordem 4, nao 3
"""

import numpy as np
import control as ct
import matplotlib.pyplot as plt
import os

# ============================================================
# CONFIGURAÇÃO DE DIRETÓRIOS
# ============================================================
RESULTS_DIR = r"C:\Users\Calil\Documents\VIDA\Faculdade\2025.2\Controle Digital\AV3\Controle_Analogico\Results\Results-ParteB"
os.makedirs(RESULTS_DIR, exist_ok=True)

print("="*80)
print("PARTE B - CALCULO DOS GANHOS PID POR ALOCACAO DE POLOS")
print("="*80)
print(f"\nResultados serao salvos em:")
print(f"  {RESULTS_DIR}")
print("="*80)

# ============================================================
# GANHOS OBTIDOS DO ALGORITMO GENETICO
# ============================================================
KP_GA = 0.01072208
KI_GA = 0.00014477
KD_GA = 0.37626039

print(f"\nGANHOS DO ALGORITMO GENETICO:")
print(f"  Kp = {KP_GA:.8f}")
print(f"  Ki = {KI_GA:.8f}")
print(f"  Kd = {KD_GA:.8f}")

# ============================================================
# ESPECIFICACOES DE PROJETO
# ============================================================
MP_MAX = 0.30
TS_MAX = 3.0
TS_A = 0.01

# ============================================================
# DADOS DA PLANTA
# ============================================================
B1 = 0.3246
A1 = -1.999
A0 = 0.9985

num_gz = [B1, 0]
den_gz = [1, A1, A0]
Gz = ct.tf(num_gz, den_gz, TS_A)

print(f"\nDados da Planta:")
print(f"  G(z) = {B1}z / (z^2 {A1}z + {A0})")
print(f"  Ts = {TS_A}s")

# ============================================================
# [1] REQUISITOS DE DESEMPENHO
# ============================================================
print("\n" + "="*80)
print("[1] REQUISITOS DE DESEMPENHO")
print("="*80)

zeta_min = np.abs(np.log(MP_MAX)) / np.sqrt(np.pi**2 + np.log(MP_MAX)**2)
print(f"\nRestricao de Mp <= {MP_MAX*100:.0f}%:")
print(f"  zeta_min = {zeta_min:.4f}")

print(f"\nRestricao de ts < {TS_MAX:.1f}s:")
print(f"  wn_min(zeta) = 4 / (zeta x {TS_MAX})")

# ============================================================
# [2] POLINOMIO CARACTERISTICO CORRETO (ORDEM 4)
# ============================================================
print("\n" + "="*80)
print("[2] POLINOMIO CARACTERISTICO (ORDEM 4 - CORRIGIDO)")
print("="*80)

Kp, Ki, Kd = KP_GA, KI_GA, KD_GA

# C(z) e G(z)
num_C = [Kp + Ki + Kd, -(Kp + 2*Kd), Kd]
den_C = [1, -1, 0]
Cz = ct.tf(num_C, den_C, TS_A)

# C(z)G(z)
num_cg = np.polymul(num_C, [B1, 0])
den_cg = np.polymul(den_C, [1, A1, A0])

# 1 + C(z)G(z) = 0
poly_char = den_cg + np.pad(num_cg, (0, len(den_cg) - len(num_cg)), 'constant')

# Normalizar
poly_norm = poly_char / poly_char[0]

print(f"\nPolinomio Caracteristico:")
print(f"  z^4 + ({poly_norm[1]:.8f})z^3 + ({poly_norm[2]:.8f})z^2 + ({poly_norm[3]:.8f})z + ({poly_norm[4]:.8f})")

a3 = poly_norm[1]
a2 = poly_norm[2]
a1 = poly_norm[3]
a0 = poly_norm[4]

# ============================================================
# [3] CRITERIO DE JURY (ORDEM 4)
# ============================================================
print("\n" + "="*80)
print("[3] VERIFICACAO DE ESTABILIDADE (Criterio de Jury n=4)")
print("="*80)

# Condicoes Necessarias
P_1 = 1 + a3 + a2 + a1 + a0
P_minus1 = 1 - a3 + a2 - a1 + a0

cond1 = P_1 > 0
cond2 = P_minus1 > 0  # Para n par, P(-1) > 0
cond3 = np.abs(a0) < 1

# Tabela de Jury - Linha 1
b3 = 1 - a0**2
b2 = a3 - a0*a1
b1 = a2 - a0*a2
b0 = a1 - a0*a3

# Condicao: |b3| > |b0|
cond4 = np.abs(b3) > np.abs(b0)

# Tabela de Jury - Linha 2 (se necessario)
if cond4:
    c2 = b3**2 - b0**2
    c1 = b3*b2 - b0*b1
    c0 = b3*b1 - b0*b2
    
    cond5 = np.abs(c2) > np.abs(c0)
else:
    cond5 = False

jury_estavel = cond1 and cond2 and cond3 and cond4 and cond5

print(f"Condicoes Necessarias:")
print(f"  1) P(1) > 0?      {cond1} ({P_1:.8f})")
print(f"  2) P(-1) > 0?     {cond2} ({P_minus1:.8f})")
print(f"  3) |a0| < 1?      {cond3} ({np.abs(a0):.8f})")

print(f"\nTabela de Jury - Linha 1:")
print(f"  b3 = {b3:.8f}")
print(f"  b0 = {b0:.8f}")
print(f"  4) |b3| > |b0|?   {cond4}")

if cond4:
    print(f"\nTabela de Jury - Linha 2:")
    print(f"  c2 = {c2:.8f}")
    print(f"  c0 = {c0:.8f}")
    print(f"  5) |c2| > |c0|?   {cond5}")

print(f"\nSTATUS: {'ESTAVEL' if jury_estavel else 'INSTAVEL'}")

# ============================================================
# [4] VALIDACAO DIRETA (POLOS)
# ============================================================
print("\n" + "="*80)
print("[4] VALIDACAO DIRETA - POLOS")
print("="*80)

sys_mf = ct.feedback(Cz * Gz, 1)
polos = ct.poles(sys_mf)

print(f"\nPolos em Malha Fechada:")
for i, p in enumerate(polos):
    print(f"  z{i+1} = {p.real:.8f} + j{p.imag:.8f}  |z| = {np.abs(p):.8f}")

max_pole = np.max(np.abs(polos))
estavel_polos = max_pole < 1.0

print(f"\nMaior magnitude: {max_pole:.8f}")
print(f"Sistema estavel: {estavel_polos}")

# ============================================================
# VALIDACAO POR SIMULACAO
# ============================================================
print("\n" + "="*80)
print("VALIDACAO POR SIMULACAO")
print("="*80)

t = np.arange(0, 5, TS_A)
t, y = ct.step_response(sys_mf, T=t)

yss = y[-1]
Mp = (np.max(y) - yss) / yss if yss > 0 else 0
erro = np.abs(y - yss)
idx = np.where(erro <= 0.02 * yss)[0]
ts = t[idx[0]] if len(idx) > 0 else 0

print(f"  Mp Real = {Mp*100:.2f}%  (Meta: <= {MP_MAX*100:.0f}%)")
print(f"  ts Real = {ts:.4f}s   (Meta: < {TS_MAX:.1f}s)")

atende = Mp <= MP_MAX and ts < TS_MAX and estavel_polos

# ============================================================
# SALVAR RESULTADOS EM TEXTO
# ============================================================
txt_path = os.path.join(RESULTS_DIR, 'resultados_parte_b.txt')
with open(txt_path, 'w', encoding='utf-8') as f:
    f.write("PARTE B - RESULTADOS DO PROJETO POR ALOCACAO DE POLOS\n")
    f.write("="*80 + "\n\n")
    
    f.write("GANHOS PID (DO ALGORITMO GENETICO):\n")
    f.write(f"  Kp = {Kp:.8f}\n")
    f.write(f"  Ki = {Ki:.8f}\n")
    f.write(f"  Kd = {Kd:.8f}\n\n")
    
    f.write("POLINOMIO CARACTERISTICO (ORDEM 4):\n")
    f.write(f"  z^4 + ({a3:.8f})z^3 + ({a2:.8f})z^2 + ({a1:.8f})z + ({a0:.8f})\n\n")
    
    f.write("DESEMPENHO:\n")
    f.write(f"  Mp  = {Mp*100:.2f}%\n")
    f.write(f"  ts  = {ts:.4f}s\n")
    f.write(f"  yss = {yss:.4f}\n\n")
    
    f.write("CRITERIO DE JURY (n=4):\n")
    f.write(f"  P(1) > 0?      {cond1} ({P_1:.6f})\n")
    f.write(f"  P(-1) > 0?     {cond2} ({P_minus1:.6f})\n")
    f.write(f"  |a0| < 1?      {cond3} ({np.abs(a0):.6f})\n")
    f.write(f"  |b3| > |b0|?   {cond4}\n")
    if cond4:
        f.write(f"  |c2| > |c0|?   {cond5}\n")
    f.write(f"  Jury: {jury_estavel}\n\n")
    
    f.write("POLOS:\n")
    for i, p in enumerate(polos):
        f.write(f"  z{i+1} = {p.real:.8f} + j{p.imag:.8f}  |z| = {np.abs(p):.8f}\n")
    f.write(f"  Max |z| = {max_pole:.8f}\n")
    f.write(f"  Estavel (polos): {estavel_polos}\n\n")
    
    f.write("VALIDACAO:\n")
    f.write(f"  Mp OK:   {Mp <= MP_MAX}\n")
    f.write(f"  ts OK:   {ts < TS_MAX}\n")
    f.write(f"  Estavel: {estavel_polos}\n")
    f.write(f"  Status:  {'SUCESSO' if atende else 'FALHA'}\n")

print(f"\nResultados salvos em: {txt_path}")

# ============================================================
# GRAFICOS
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

# Resposta ao degrau
ax1 = axes[0]
ax1.plot(t, y, 'b-', linewidth=2.5, label='y(t)')
ax1.axhline(1.0, color='r', linestyle='--', label='Setpoint')
ax1.axhline(1+MP_MAX, color='g', linestyle=':', label=f'Limite Mp')
ax1.set_xlabel('Tempo (s)')
ax1.set_ylabel('Posicao')
ax1.set_title(f'Resposta ao Degrau\nMp={Mp*100:.1f}%, ts={ts:.2f}s', fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Diagrama de polos
ax2 = axes[1]
theta = np.linspace(0, 2*np.pi, 100)
ax2.plot(np.cos(theta), np.sin(theta), 'k--', alpha=0.4, label='Circulo Unitario')
ax2.plot(polos.real, polos.imag, 'bo', markersize=10, label='Polos MF')
ax2.set_xlabel('Real')
ax2.set_ylabel('Imaginario')
ax2.set_title('Mapa de Polos z', fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend()
ax2.axis('equal')

plt.tight_layout()
graph_path = os.path.join(RESULTS_DIR, 'parte_b_resultado_final.png')
plt.savefig(graph_path, dpi=300)
print(f"Grafico salvo em: {graph_path}")

plt.close()

print(f"\n{'SUCESSO TOTAL!' if atende else 'ATENCAO: Verifique os requisitos'}")
print("="*80)