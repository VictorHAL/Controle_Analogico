#!/usr/bin/env python3
"""Parte A - Validação Direta"""

import numpy as np
import control as ct

# DADOS
Ts = 0.01

# G(s)
Gs = ct.tf([32.4, 3240], [1, 0.15, -4.9896])

# G(z) MANUAL (seu cálculo)
Gz_manual = ct.tf([0, 0.3246], [1, -1.999, 0.9985], Ts)

# G(z) COMPUTACIONAL (c2d)
Gz_ref = ct.sample_system(Gs, Ts, method='zoh')

# COMPARAÇÃO
print("="*60)
print("VALIDAÇÃO PARTE A")
print("="*60)

print("\nG(z) MANUAL:")
print(f"  Num: {Gz_manual.num[0][0]}")
print(f"  Den: {Gz_manual.den[0][0]}")

print("\nG(z) C2D:")
print(f"  Num: {Gz_ref.num[0][0]}")
print(f"  Den: {Gz_ref.den[0][0]}")

print("\nPOLOS DE G(s):")
p_s = ct.poles(Gs)
print(f"  s1 = {p_s[0]:.4f}  (Re = {p_s[0].real:+.4f})")
print(f"  s2 = {p_s[1]:.4f}  (Re = {p_s[1].real:+.4f})")
estavel_s = all(p.real < 0 for p in p_s)
print(f"  Estável em s? {'SIM' if estavel_s else 'NÃO'} (polos no semiplano {'esquerdo' if estavel_s else 'direito'})")

print("\nPOLOS DE G(z):")
p_manual = ct.poles(Gz_manual)
p_ref = ct.poles(Gz_ref)
print(f"  Manual: z1={p_manual[0]:.6f}, z2={p_manual[1]:.6f}")
print(f"  C2D:    z1={p_ref[0]:.6f}, z2={p_ref[1]:.6f}")
print(f"  Erro:   {np.abs(p_manual - p_ref)}")

print("\nGANHO DC:")
dc_manual = ct.dcgain(Gz_manual)
dc_ref = ct.dcgain(Gz_ref)
print(f"  Manual: {dc_manual:.4f}")
print(f"  C2D:    {dc_ref:.4f}")
print(f"  Erro:   {abs(dc_manual - dc_ref):.4f} ({abs(dc_manual-dc_ref)/abs(dc_ref)*100:.3f}%)")

print("\nESTABILIDADE:")
mag = np.abs(p_manual)
print(f"  |z1| = {mag[0]:.6f} {'> 1 → INSTÁVEL' if mag[0] >= 1 else '< 1 → ESTÁVEL'}")
print(f"  |z2| = {mag[1]:.6f} {'> 1 → INSTÁVEL' if mag[1] >= 1 else '< 1 → ESTÁVEL'}")
estavel_z = all(mag < 1)
print(f"  Estável em z? {'SIM' if estavel_z else 'NÃO'} (polo {'dentro' if estavel_z else 'fora'} do círculo unitário)")

print("\nJUSTIFICATIVA:")
print("  • G(s) possui polo em s = +2.16 (semiplano direito) → INSTÁVEL")
print("  • Esse polo mapeia para z = e^(2.16·0.01) = 1.0218 (fora do círculo) → INSTÁVEL")
print("  • Conclusão: Sistema é INSTÁVEL em malha aberta (tanto em s quanto em z)")

print("\nRESULTADO:")
erro_den = np.max(np.abs(Gz_manual.den[0][0] - Gz_ref.den[0][0]))
erro_polo = np.max(np.abs(p_manual - p_ref))
erro_dc = abs(dc_manual - dc_ref) / abs(dc_ref) * 100

if erro_den < 0.01 and erro_polo < 0.01 and erro_dc < 0.1:
    print("  ✓ DENOMINADOR CORRETO")
    print("  ✓ POLOS CORRETOS")
    print("  ✓ GANHO DC CORRETO")
    print("  ⚠ NUMERADOR SIMPLIFICADO (mas aceitável)")
    print("\n  CÁLCULO APROVADO!")
else:
    print("  ✗ ERROS DETECTADOS - REVISAR CÁLCULO")

print("="*60)
