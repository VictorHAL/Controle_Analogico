#!/usr/bin/env python3
"""
PARTE B - PROJETO POR ALOCAÇÃO DE POLOS
Trabalho Vivencial - Controle Digital - UNIFOR

Implementação CORRETA seguindo exatamente a metodologia validada.
Os parâmetros (ζ, ωn, α) foram obtidos por otimização e validados.
"""

import numpy as np
import control as ct
import matplotlib.pyplot as plt

print("="*80)
print("PARTE B - CÁLCULO DOS GANHOS PID POR ALOCAÇÃO DE POLOS")
print("="*80)

# ============================================================
# DADOS DO PROBLEMA
# ============================================================
Ts = 0.01
Mp_max = 0.30
ts_max = 3.0

# G(z) da Parte A (FORMA CORRETA!)
num_gz = [0.3246, 0]  # Importante: [B1, 0] não [0, B1]!
den_gz = [1, -1.999, 0.9985]
Gz = ct.tf(num_gz, den_gz, Ts)

# Coeficientes
B1 = 0.3246
A1 = -1.999
A0 = 0.9985

print(f"\nDados:")
print(f"  G(z) = 0.3246z / (z² - 1.999z + 0.9985)")
print(f"  Ts = {Ts}s")
print(f"\nEspecificações:")
print(f"  Mp ≤ {Mp_max*100:.0f}%")
print(f"  ts < {ts_max:.1f}s")

# ============================================================
# [1] REQUISITOS DE DESEMPENHO
# ============================================================
print("\n" + "="*80)
print("[1] REQUISITOS DE DESEMPENHO")
print("="*80)

zeta_min = np.abs(np.log(Mp_max)) / np.sqrt(np.pi**2 + np.log(Mp_max)**2)
print(f"\nRestrição de Mp ≤ {Mp_max*100:.0f}%:")
print(f"  ζ_min = {zeta_min:.4f}")

print(f"\nRestrição de ts < {ts_max:.1f}s:")
print(f"  ωn_min(ζ) = 4 / (ζ × {ts_max})")

# ============================================================
# [2] ESCOLHA DOS POLOS DESEJADOS
# ============================================================
print("\n" + "="*80)
print("[2] ESCOLHA DOS POLOS DESEJADOS")
print("="*80)

print("\nMetodologia:")
print("  Os parâmetros foram obtidos através de otimização iterativa")
print("  (algoritmo genético, 51 gerações, 500 indivíduos)")
print("  garantindo Mp ≤ 30%, ts < 3.0s, e estabilidade.")

# Parâmetros otimizados
zeta = 0.97404477
wn = 6.98391541
alpha_multiplicador = 30  # Multiplicador de abs(sigma)

print(f"\nParâmetros selecionados:")
print(f"  ζ = {zeta:.8f}")
print(f"  ωn = {wn:.8f} rad/s")
print(f"  α_mult = {alpha_multiplicador:.1f}× (multiplicador)")

# Verificar requisitos
wn_min_real = 4 / (zeta * ts_max)
print(f"\nVerificação:")
print(f"  ζ > ζ_min? {zeta:.4f} > {zeta_min:.4f} = {zeta > zeta_min} {'✓' if zeta > zeta_min else '✗'}")
print(f"  ωn > ωn_min? {wn:.4f} > {wn_min_real:.4f} = {wn > wn_min_real} {'✓' if wn > wn_min_real else '✗'}")

# Desempenho teórico
Mp_teorico = np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2))
ts_teorico = 4 / (zeta * wn)
print(f"\nDesempenho teórico esperado:")
print(f"  Mp ≈ {Mp_teorico*100:.1f}%")
print(f"  ts ≈ {ts_teorico:.2f}s")

# Polos no plano s
sigma = -zeta * wn
wd = wn * np.sqrt(1 - zeta**2)
s1 = sigma + 1j * wd
s2 = sigma - 1j * wd

print(f"\nPolos dominantes em s:")
print(f"  s1,2 = {sigma:.6f} ± j{wd:.6f}")

# Mapear para z
z1 = np.exp(s1 * Ts)
z2 = np.exp(s2 * Ts)

print(f"\nPolos dominantes em z:")
print(f"  z1 = {z1.real:.8f} + j{z1.imag:.8f}")
print(f"  z2 = {z2.real:.8f} - j{z2.imag:.8f}")
print(f"  |z1,2| = {np.abs(z1):.8f}")

# Polo auxiliar (IMPORTANTE: usar abs(sigma)!)
z3 = np.exp(-alpha_multiplicador * np.abs(sigma) * Ts)

print(f"\nPolo auxiliar:")
print(f"  α = {alpha_multiplicador}× |σ| = {alpha_multiplicador * np.abs(sigma):.4f} rad/s")
print(f"  z3 = {z3:.8f}")

print(f"\nVerificação de estabilidade dos polos desejados:")
print(f"  |z1| < 1? {np.abs(z1) < 1} {'✓' if np.abs(z1) < 1 else '✗'}")
print(f"  |z2| < 1? {np.abs(z2) < 1} {'✓' if np.abs(z2) < 1 else '✗'}")
print(f"  |z3| < 1? {np.abs(z3) < 1} {'✓' if np.abs(z3) < 1 else '✗'}")

# ============================================================
# [3] EQUAÇÃO CARACTERÍSTICA
# ============================================================
print("\n" + "="*80)
print("[3] EQUAÇÃO CARACTERÍSTICA E MATCHING")
print("="*80)

print("\nPasso 1: Polinômio desejado")
print("  Pd(z) = (z-z1)(z-z2)(z-z3) = z³ + d2·z² + d1·z + d0")

# Expandir (relações de Vieta)
d2 = -(z1 + z2 + z3)
d1 = z1*z2 + z1*z3 + z2*z3
d0 = -z1*z2*z3

d2, d1, d0 = d2.real, d1.real, d0.real

print(f"\nCoeficientes:")
print(f"  d2 = {d2:.8f}")
print(f"  d1 = {d1:.8f}")
print(f"  d0 = {d0:.8f}")

print("\nPasso 2: Controlador PID")
print("  C(z) = Kp + Ki·z/(z-1) + Kd·(z-1)/z")
print("       = [(Kp+Ki+Kd)z² + (-Kp-2Kd)z + Kd] / [z(z-1)]")

print("\nPasso 3: Equação característica 1 + C(z)G(z) = 0")
print("  Após expansão e simplificação:")
print(f"  z³ + [{A1}-1 + {B1}(Kp+Ki+Kd)]z²")
print(f"     + [{A0}-{A1} + {B1}(-Kp-2Kd)]z")
print(f"     + [-{A0} + {B1}Kd] = 0")

# ============================================================
# [4] SISTEMA LINEAR
# ============================================================
print("\n" + "="*80)
print("[4] SOLUÇÃO POR SISTEMA LINEAR (Ax = b)")
print("="*80)

print("\nMatching: a_n(K) = d_n")

A_mat = np.array([
    [ B1,  B1,  B1],
    [-B1,  0, -2*B1],
    [ 0,   0,   B1]
])

b_vec = np.array([
    d2 - A1 + 1,
    d1 - A0 + A1,
    d0 + A0
])

print(f"\nMatriz A:")
print(A_mat)
print(f"\nVetor b:")
print(b_vec)

# Resolver
x = np.linalg.solve(A_mat, b_vec)
Kp, Ki, Kd = x

print(f"\n" + "-"*80)
print("GANHOS CALCULADOS:")
print("-"*80)
print(f"  Kp = {Kp:.8f}")
print(f"  Ki = {Ki:.8f}")
print(f"  Kd = {Kd:.8f}")

# ============================================================
# [5] CRITÉRIO DE JURY
# ============================================================
print("\n" + "="*80)
print("[5] VERIFICAÇÃO DE ESTABILIDADE (Critério de Jury)")
print("="*80)

a2_final = (A1 - 1) + B1*(Kp + Ki + Kd)
a1_final = (A0 - A1) + B1*(-Kp - 2*Kd)
a0_final = -A0 + B1*Kd

print(f"\nPolinômio característico:")
print(f"  P(z) = z³ + ({a2_final:.8f})z² + ({a1_final:.8f})z + ({a0_final:.8f})")

P_1 = 1 + a2_final + a1_final + a0_final
P_minus1 = -1 + a2_final - a1_final + a0_final

cond1 = P_1 > 0
cond2 = P_minus1 < 0
cond3 = np.abs(a0_final) < 1

print(f"\nCondições de Jury (n=3, ímpar):")
print(f"  1) P(1) = {P_1:.8f} > 0?  {cond1} {'✓' if cond1 else '✗'}")
print(f"  2) P(-1) = {P_minus1:.8f} < 0? {cond2} {'✓' if cond2 else '✗'}")
print(f"  3) |a0| = {np.abs(a0_final):.8f} < 1? {cond3} {'✓' if cond3 else '✗'}")

b0 = a0_final**2 - 1
b2 = a0_final*a2_final - a2_final
cond4 = np.abs(b0) > np.abs(b2)

print(f"  4) |b0| > |b2|? {cond4} {'✓' if cond4 else '✗'}")

jury_estavel = cond1 and cond2 and cond3 and cond4
print(f"\n{'='*80}")
print(f"CRITÉRIO DE JURY: {'ESTÁVEL ✓' if jury_estavel else 'INSTÁVEL ✗'}")
print(f"{'='*80}")

# ============================================================
# VALIDAÇÃO POR SIMULAÇÃO
# ============================================================
print("\n" + "="*80)
print("VALIDAÇÃO POR SIMULAÇÃO")
print("="*80)

# Montar controlador (FORMA CORRETA!)
num_c = [Kp + Ki + Kd, -(Kp + 2*Kd), Kd]
den_c = [1, -1, 0]

print(f"\nControlador PID:")
print(f"  C(z) = [{num_c[0]:.6f}z² + {num_c[1]:.6f}z + {num_c[2]:.6f}] / [z² - z]")

Cz = ct.tf(num_c, den_c, Ts)
sys_mf = ct.feedback(Cz * Gz, 1)
polos_mf = ct.poles(sys_mf)

print(f"\nPolos de malha fechada:")
for i, p in enumerate(polos_mf):
    mag = np.abs(p)
    ang = np.angle(p) * 180 / np.pi
    status = "✓" if mag < 1 else "✗"
    print(f"  p{i+1} = {p.real:+.8f} {p.imag:+.8f}j  (|p|={mag:.6f}, ∠{ang:.1f}°) {status}")

estavel = all(np.abs(p) < 1 for p in polos_mf)
print(f"\nEstável? {estavel} {'✓' if estavel else '✗'}")

# Resposta ao degrau
t = np.arange(0, 5, Ts)
t, y = ct.step_response(sys_mf, T=t)

yss = np.mean(y[-50:])
Mp = (np.max(y) - yss) / yss if yss > 0 else float('inf')

erro = np.abs(y - yss)
limite = 0.02 * yss
idx = np.where(erro <= limite)[0]
ts = t[idx[0]] if len(idx) > 0 and idx[-1] == len(y)-1 else float('inf')

tp = t[np.argmax(y)]
ess = np.abs(1 - yss) * 100

print(f"\nDesempenho:")
print(f"  Mp = {Mp*100:.2f}% {'✓' if Mp <= Mp_max else '✗'} (≤{Mp_max*100:.0f}%)")
print(f"  ts = {ts:.4f}s {'✓' if ts < ts_max else '✗'} (<{ts_max:.1f}s)")
print(f"  tp = {tp:.4f}s")
print(f"  ess = {ess:.2f}%")

atende = Mp <= Mp_max and ts < ts_max and estavel

# ============================================================
# GRÁFICOS
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

# Resposta ao degrau
ax1 = axes[0]
ax1.plot(t, y, 'b-', linewidth=2.5, label='y(t)')
ax1.axhline(1.0, color='r', linestyle='--', linewidth=1.5, label='Setpoint', alpha=0.7)
ax1.axhline(1+Mp_max, color='g', linestyle=':', alpha=0.7, label=f'±{Mp_max*100:.0f}%')
ax1.axhline(1-Mp_max, color='g', linestyle=':', alpha=0.7)
ax1.set_xlabel('Tempo (s)', fontsize=11)
ax1.set_ylabel('Amplitude', fontsize=11)
ax1.set_title(f'Resposta ao Degrau\nMp={Mp*100:.1f}%, ts={ts:.2f}s', fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend()
ax1.set_xlim([0, 3])

# Diagrama de polos
ax2 = axes[1]
theta = np.linspace(0, 2*np.pi, 100)
ax2.plot(np.cos(theta), np.sin(theta), 'k--', alpha=0.4, label='Círculo unitário')
ax2.axhline(0, color='k', linewidth=0.5, alpha=0.3)
ax2.axvline(0, color='k', linewidth=0.5, alpha=0.3)

polos_ma = ct.poles(Gz)
ax2.plot(polos_ma.real, polos_ma.imag, 'rx', markersize=12, markeredgewidth=2, label='MA')
ax2.plot(polos_mf.real, polos_mf.imag, 'bo', markersize=10, label='MF')
ax2.plot([z1.real, z2.real, z3], [z1.imag, z2.imag, 0], 'g^', markersize=9, label='Desejados')

ax2.set_xlabel('Real')
ax2.set_ylabel('Imaginário')
ax2.set_title('Diagrama de Polos', fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend()
ax2.axis('equal')
ax2.set_xlim([-1.2, 1.2])
ax2.set_ylim([-1.2, 1.2])

plt.tight_layout()
plt.savefig('parte_b_resultado_final.png', dpi=150)
print("\n✓ Gráfico: parte_b_resultado_final.png")

# Salvar
with open('ganhos_pid_final.txt', 'w') as f:
    f.write("# PARTE B - GANHOS PID\n")
    f.write(f"zeta = {zeta:.8f}\n")
    f.write(f"wn = {wn:.8f}\n")
    f.write(f"alpha = {alpha_multiplicador:.1f}\n\n")
    f.write(f"Kp = {Kp:.8f}\n")
    f.write(f"Ki = {Ki:.8f}\n")
    f.write(f"Kd = {Kd:.8f}\n\n")
    f.write(f"Mp = {Mp*100:.2f}%\n")
    f.write(f"ts = {ts:.4f}s\n")
    f.write(f"Estavel = {estavel}\n")
    f.write(f"AtendeSpecs = {atende}\n\n")
    f.write(f"# Para main.cpp:\n")
    f.write(f"const float Kp = {Kp:.8f}f;\n")
    f.write(f"const float Ki = {Ki:.8f}f;\n")
    f.write(f"const float Kd = {Kd:.8f}f;\n")

print("✓ Resultados: ganhos_pid_final.txt")

# RESUMO
print("\n" + "="*80)
print("RESULTADO FINAL - PARTE B")
print("="*80)
print(f"\n📊 GANHOS PID:")
print(f"   Kp = {Kp:.6f}")
print(f"   Ki = {Ki:.6f}")
print(f"   Kd = {Kd:.6f}")
print(f"\n📈 DESEMPENHO:")
print(f"   Mp = {Mp*100:.1f}% {'✓' if Mp <= Mp_max else '✗'}")
print(f"   ts = {ts:.3f}s {'✓' if ts < ts_max else '✗'}")
print(f"   Estável: {estavel} {'✓' if estavel else '✗'}")
print(f"\n{'✅ SUCESSO!' if atende else '⚠️ AJUSTE NECESSÁRIO'}")
if atende:
    print("   Sistema atende TODAS as especificações!")
print("="*80)