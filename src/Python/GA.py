#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Algoritmo Genético para Otimização de Controlador PID Digital
"""

import numpy as np
import control as ct
import matplotlib.pyplot as plt
import random
import time
import os

# ============================================================
# CONFIGURAÇÃO DE DIRETÓRIOS
# ============================================================
RESULTS_DIR = r"C:\Users\Calil\Documents\VIDA\Faculdade\2025.2\Controle Digital\AV3\Controle_Analogico\Results\algoritmo_genetico"
os.makedirs(RESULTS_DIR, exist_ok=True)

# ============================================================
# PARÂMETROS DO ALGORITMO GENÉTICO
# ============================================================
NUM_GENERATIONS = 20
POPULATION_SIZE = 1000
MUTATION_RATE = 0.25
ELITISM_COUNT = 15
TOURNAMENT_SIZE = 5
TOURNAMENT_POOL_SIZE = 300

# Critério de convergência
CONVERGENCE_GENERATION = 30
CONVERGENCE_VALID_COUNT = 300

# ============================================================
# RANGES DE PROJETO
# ============================================================
ZETA_RANGE = (0.45, 0.95)
WN_RANGE = (2.0, 10.0)
ALPHA_RANGE = (5.0, 12.0)

# ============================================================
# LIMITES DE GANHOS
# ============================================================
KP_MAX = 200.0
KI_MAX = 100.0
KD_MAX = 0.8

# ============================================================
# SISTEMA E ESPECIFICAÇÕES
# ============================================================
Ts = 0.01  # Período de amostragem (100 Hz)
B1, A1, A0 = 0.3246, -1.999, 0.9985  # Coeficientes da planta discreta
Gz = ct.tf([B1, 0], [1, A1, A0], Ts)

MP_MIN = 0.15
MP_MAX = 0.25
TS_MIN = 2.2
TS_MAX = 2.9
UNDERSHOOT_MAX = 0.03

TS_CRITERION = 0.02  # Banda de settling (±2%)

print("="*70)
print("ALGORITMO GENETICO - OTIMIZACAO PID")
print("="*70)
print(f"Populacao: {POPULATION_SIZE} individuos")
print(f"Geracoes: {NUM_GENERATIONS}")
print(f"\nEspecificacoes:")
print(f"  {MP_MIN*100:.0f}% <= Mp <= {MP_MAX*100:.0f}%")
print(f"  {TS_MIN:.1f}s <= ts <= {TS_MAX:.1f}s")
print(f"  Undershoot < {UNDERSHOOT_MAX*100:.1f}%")
print(f"\nRanges de projeto:")
print(f"  Zeta: [{ZETA_RANGE[0]}, {ZETA_RANGE[1]}]")
print(f"  Wn: [{WN_RANGE[0]}, {WN_RANGE[1]}]")
print(f"  Alpha: [{ALPHA_RANGE[0]}, {ALPHA_RANGE[1]}]")
print(f"\nLimites de ganhos:")
print(f"  Kp_max: {KP_MAX}, Ki_max: {KI_MAX}, Kd_max: {KD_MAX}")
print(f"\nResultados serao salvos em:")
print(f"  {RESULTS_DIR}")
print("-"*70)

# ============================================================
# CLASSE INDIVÍDUO
# ============================================================
class Individual:
    def __init__(self, genes=None):
        if genes is not None:
            self.genes = genes
        else:
            self.genes = [
                random.uniform(*ZETA_RANGE),
                random.uniform(*WN_RANGE),
                random.uniform(*ALPHA_RANGE)
            ]
        
        self.fitness = -1e9
        self.stats = {}
        self.gains = (0, 0, 0)
        self.is_valid = False
    
    def calculate_fitness(self):
        zeta, wn, alpha = self.genes
        
        try:
            sigma = -zeta * wn
            wd = wn * np.sqrt(1 - zeta**2)
            
            z1 = np.exp((sigma + 1j*wd) * Ts)
            z2 = np.exp((sigma - 1j*wd) * Ts)
            z3 = np.exp(-alpha * abs(sigma) * Ts)
            
            d2 = -(z1 + z2 + z3).real
            d1 = (z1*z2 + z1*z3 + z2*z3).real
            d0 = -(z1*z2*z3).real
            
            A = np.array([
                [B1,  B1,  B1],
                [-B1, 0,   -2*B1],
                [0,   0,    B1]
            ])
            
            b = np.array([
                d2 - A1 + 1,
                d1 - A0 + A1,
                d0 + A0
            ])
            
            Kp, Ki, Kd = np.linalg.solve(A, b)
            self.gains = (Kp, Ki, Kd)
            
            if abs(Kp) > KP_MAX or abs(Ki) > KI_MAX or abs(Kd) > KD_MAX:
                self.fitness = -50000
                return
            
            num_C = [Kp + Ki + Kd, -(Kp + 2*Kd), Kd]
            den_C = [1, -1, 0]
            Cz = ct.tf(num_C, den_C, Ts)
            
            sys_mf = ct.feedback(Cz * Gz, 1)
            
            polos = ct.poles(sys_mf)
            max_pole_mag = np.max(np.abs(polos))
            
            if max_pole_mag >= 0.999:
                self.fitness = -10000
                return
            
            t = np.arange(0, 12, Ts)
            t, y = ct.step_response(sys_mf, T=t)
            
            n_buffer = max(int(len(y) * 0.2), 50)
            yss = np.mean(y[-n_buffer:])
            
            if not (0.92 < yss < 1.08):
                self.fitness = -5000
                return
            
            ymax = np.max(y)
            idx_max = np.argmax(y)
            Mp = (ymax - yss) / yss if ymax > yss else 0
            
            if idx_max < len(y) - 10:
                y_after_peak = y[idx_max:]
                ymin_after_peak = np.min(y_after_peak)
                undershoot = (yss - ymin_after_peak) / yss if ymin_after_peak < yss else 0
            else:
                undershoot = 0
            
            erro = np.abs(y - yss)
            limite = TS_CRITERION * abs(yss)
            idx_fora = np.where(erro > limite)[0]
            
            if len(idx_fora) > 0:
                ts = t[idx_fora[-1]]
            else:
                ts = 0
            
            self.stats = {
                'Mp': Mp,
                'ts': ts,
                'yss': yss,
                'undershoot': undershoot,
                'max_pole_mag': max_pole_mag
            }
            
            score = 10000
            
            if Mp > MP_MAX:
                excesso = Mp - MP_MAX
                score -= excesso * 120000
            elif Mp < MP_MIN:
                falta = MP_MIN - Mp
                score -= falta * 120000
            else:
                score += 6000
                dist = abs(Mp - (MP_MIN + MP_MAX)/2)
                score += ((MP_MAX - MP_MIN)/2 - dist) * 3000
            
            if ts > TS_MAX:
                excesso = ts - TS_MAX
                score -= excesso * 80000
            elif ts < TS_MIN:
                falta = TS_MIN - ts
                score -= falta * 30000
            else:
                score += 8000
                dist_centro = abs(ts - (TS_MIN + TS_MAX)/2)
                score += ((TS_MAX - TS_MIN)/2 - dist_centro) * 5000
            
            if undershoot > UNDERSHOOT_MAX:
                excesso = undershoot - UNDERSHOOT_MAX
                score -= excesso * 180000
            else:
                score += (UNDERSHOOT_MAX - undershoot) * 6000
            
            penalty_ganhos = 0.02 * (abs(Kp) + abs(Ki) + abs(Kd))
            score -= penalty_ganhos
            
            margem = 1.0 - max_pole_mag
            score += margem * 2000
            
            self.fitness = score
            self.is_valid = (MP_MIN <= Mp <= MP_MAX and 
                           TS_MIN <= ts <= TS_MAX and 
                           undershoot <= UNDERSHOOT_MAX)
            
        except Exception as e:
            self.fitness = -100000
            self.is_valid = False

# ============================================================
# OPERADORES GENÉTICOS
# ============================================================
def crossover(parent1, parent2):
    child_genes = []
    for i in range(3):
        weight = random.random()
        gene = weight * parent1.genes[i] + (1 - weight) * parent2.genes[i]
        child_genes.append(gene)
    return Individual(child_genes)

def mutate(individual):
    if random.random() < MUTATION_RATE:
        gene_idx = random.randint(0, 2)
        perturbation = random.uniform(0.90, 1.10)
        individual.genes[gene_idx] *= perturbation
        
        if gene_idx == 0:
            individual.genes[0] = np.clip(individual.genes[0], *ZETA_RANGE)
        elif gene_idx == 1:
            individual.genes[1] = np.clip(individual.genes[1], *WN_RANGE)
        else:
            individual.genes[2] = np.clip(individual.genes[2], *ALPHA_RANGE)

def tournament_selection(population):
    candidates = random.sample(population[:TOURNAMENT_POOL_SIZE], TOURNAMENT_SIZE)
    return max(candidates, key=lambda ind: ind.fitness)

# ============================================================
# EXECUÇÃO
# ============================================================
print("\nIniciando evolucao...")
start_time = time.time()

population = [Individual() for _ in range(POPULATION_SIZE)]
best_fitness_history = []

for generation in range(1, NUM_GENERATIONS + 1):
    for individual in population:
        individual.calculate_fitness()
    
    population.sort(key=lambda ind: ind.fitness, reverse=True)
    best = population[0]
    best_fitness_history.append(best.fitness)
    
    valid_count = sum(1 for ind in population if ind.is_valid)
    
    if generation % 10 == 0 or generation == 1:
        mp_pct = best.stats.get('Mp', 0) * 100
        ts_val = best.stats.get('ts', 0)
        us_pct = best.stats.get('undershoot', 0) * 100
        
        mp_ok = MP_MIN <= best.stats.get('Mp', 0) <= MP_MAX
        ts_ok = TS_MIN <= ts_val <= TS_MAX
        us_ok = best.stats.get('undershoot', 0) <= UNDERSHOOT_MAX
        
        status_mp = "OK" if mp_ok else "X"
        status_ts = "OK" if ts_ok else "X"
        status_us = "OK" if us_ok else "X"
        
        print(f"Gen {generation:3d} | "
              f"Score: {best.fitness:9.0f} | "
              f"Mp: {mp_pct:5.1f}% [{status_mp}] | "
              f"ts: {ts_val:5.2f}s [{status_ts}] | "
              f"US: {us_pct:4.1f}% [{status_us}]")
    
    if generation > CONVERGENCE_GENERATION and valid_count > CONVERGENCE_VALID_COUNT:
        print(f"\nConvergencia alcancada na geracao {generation}!")
        break
    
    new_population = []
    new_population.extend(population[:ELITISM_COUNT])
    
    while len(new_population) < POPULATION_SIZE:
        parent1 = tournament_selection(population)
        parent2 = tournament_selection(population)
        child = crossover(parent1, parent2)
        mutate(child)
        new_population.append(child)
    
    population = new_population

elapsed_time = time.time() - start_time

# ============================================================
# RESULTADO FINAL
# ============================================================
best_individual = population[0]
zeta_best, wn_best, alpha_best = best_individual.genes
Kp_best, Ki_best, Kd_best = best_individual.gains

mp_final = best_individual.stats['Mp'] * 100
ts_final = best_individual.stats['ts']
us_final = best_individual.stats['undershoot'] * 100

print("\n" + "="*70)
print("RESULTADO FINAL")
print("="*70)
print(f"Parametros: zeta={zeta_best:.8f}, wn={wn_best:.8f}, alpha={alpha_best:.8f}")
print(f"Ganhos PID: Kp={Kp_best:.8f}, Ki={Ki_best:.8f}, Kd={Kd_best:.8f}")
print(f"Desempenho: Mp={mp_final:.2f}%, ts={ts_final:.4f}s, Undershoot={us_final:.2f}%")

mp_check = MP_MIN * 100 <= mp_final <= MP_MAX * 100
ts_check = TS_MIN <= ts_final <= TS_MAX
us_check = us_final <= UNDERSHOOT_MAX * 100

if mp_check and ts_check and us_check:
    print(f"STATUS: SUCESSO TOTAL!")
else:
    print(f"STATUS: PARCIAL")

# ============================================================
# SALVAR
# ============================================================
txt_path = os.path.join(RESULTS_DIR, 'resultados_ga.txt')
with open(txt_path, 'w', encoding='utf-8') as f:
    f.write("RESULTADOS DO ALGORITMO GENETICO\n")
    f.write("="*70 + "\n\n")
    f.write(f"PARAMETROS DE PROJETO:\n")
    f.write(f"  zeta = {zeta_best:.8f}\n")
    f.write(f"  wn   = {wn_best:.8f}\n")
    f.write(f"  alfa = {alpha_best:.8f}\n\n")
    f.write(f"GANHOS PID:\n")
    f.write(f"  Kp = {Kp_best:.8f}\n")
    f.write(f"  Ki = {Ki_best:.8f}\n")
    f.write(f"  Kd = {Kd_best:.8f}\n\n")
    f.write(f"DESEMPENHO:\n")
    f.write(f"  Mp  = {mp_final:.2f}%\n")
    f.write(f"  ts  = {ts_final:.4f}s\n")
    f.write(f"  yss = {best_individual.stats['yss']:.4f}\n")
    f.write(f"  Undershoot = {us_final:.2f}%\n\n")
    f.write(f"VALIDACAO:\n")
    f.write(f"  Mp OK: {mp_check}\n")
    f.write(f"  ts OK: {ts_check}\n")
    f.write(f"  Undershoot OK: {us_check}\n")

print(f"\nResultados salvos em: {txt_path}")

# ============================================================
# GRÁFICO
# ============================================================
num_C = [Kp_best + Ki_best + Kd_best, -(Kp_best + 2*Kd_best), Kd_best]
den_C = [1, -1, 0]
sys_mf = ct.feedback(ct.tf(num_C, den_C, Ts) * Gz, 1)

t_plot = np.arange(0, 8, Ts)
t_plot, y_plot = ct.step_response(sys_mf, T=t_plot)

plt.figure(figsize=(12, 6))
plt.plot(t_plot, y_plot, 'b-', linewidth=2.5, label='Resposta y(t)')
plt.axhline(1.0, color='black', linestyle='--', linewidth=1.5, label='Setpoint', alpha=0.7)

plt.axvspan(TS_MIN, TS_MAX, color='green', alpha=0.15, label=f'Alvo Tempo ({TS_MIN}-{TS_MAX}s)')
plt.axhspan(1.0 + MP_MIN, 1.0 + MP_MAX, color='red', alpha=0.15, label=f'Alvo Mp ({MP_MIN*100:.0f}-{MP_MAX*100:.0f}%)')
plt.axhline(1.0 + MP_MIN, color='r', linestyle=':', alpha=0.6)
plt.axhline(1.0 + MP_MAX, color='r', linestyle=':', alpha=0.6)
plt.axhline(1.0 - UNDERSHOOT_MAX, color='orange', linestyle=':', linewidth=1.5, alpha=0.8, label=f'Limite US ({UNDERSHOOT_MAX*100:.1f}%)')

plt.axhspan(1.0 - TS_CRITERION, 1.0 + TS_CRITERION, color='gray', alpha=0.1, label=f'Banda ±{TS_CRITERION*100:.0f}%')

plt.title(f'Resposta Final\nts={ts_final:.2f}s, Mp={mp_final:.1f}%, US={us_final:.1f}%', fontsize=14, fontweight='bold')
plt.xlabel('Tempo (s)', fontsize=12)
plt.ylabel('Amplitude', fontsize=12)
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)
plt.xlim(0, 8)

graph_path = os.path.join(RESULTS_DIR, 'resposta_otimizada.png')
plt.savefig(graph_path, dpi=300, bbox_inches='tight')
print(f"Grafico salvo em: {graph_path}")

plt.close()

print("\nOtimizacao concluida!")