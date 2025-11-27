#include <Arduino.h>
#include <Baratinha.h>
#include <math.h>

Baratinha bra;

namespace {
  // Periodo de controle - CORRIGIDO para 0.01s (100 Hz)
  const float Ts = 0.01f;
  
  // GANHOS PID - PREENCHER COM VALORES DO GA
  const float Kp = 0.0f;  // Substituir
  const float Ki = 0.0f;  // Substituir
  const float Kd = 0.0f;  // Substituir
  
  // Setpoint (distancia alvo em mm)
  const float setpoint = 100.0f;  // 10 cm
  
  // Variaveis de controle
  float erro_integral = 0.0f;
  float erro_anterior = 0.0f;
  float derivada_filtrada = 0.0f;
  
  // Anti-windup
  const float INTEGRAL_MAX = 1000.0f;
  const float INTEGRAL_MIN = -1000.0f;
  
  // Filtro derivativo
  const float ALPHA_FILTRO = 0.1f;
  
  // Saturacao PWM
  const float PWM_MAX = 255.0f;
  const float PWM_MIN = -255.0f;
  
  // Seguranca
  const float DIST_MIN = 30.0f;  // 3 cm
}

float saturar(float valor, float minimo, float maximo) {
  if (valor > maximo) return maximo;
  if (valor < minimo) return minimo;
  return valor;
}

void setup() {
  bra.recoveryMode();
  bra.setupAll();
  bra.setControlInterval(Ts);
  bra.awaitStart();
  
  // Reset inicial
  erro_integral = 0.0f;
  erro_anterior = 0.0f;
  derivada_filtrada = 0.0f;
}

void loop() {
  bra.updateStartStop();
  
  if (!bra.isRunning()) {
    bra.stop();
    erro_integral = 0.0f;
    erro_anterior = 0.0f;
    derivada_filtrada = 0.0f;
    return;
  }
  
  if (!bra.controlTickDue()) return;
  
  // Leitura do sensor
  float distancia = bra.readDistance();
  
  if (distancia <= 0 || distancia > 2000) {
    bra.stop();
    return;
  }
  
  // Seguranca
  if (distancia < DIST_MIN) {
    bra.stop();
    return;
  }
  
  // Erro
  float erro = setpoint - distancia;
  
  // Termo P
  float termo_p = Kp * erro;
  
  // Termo I (com anti-windup)
  erro_integral += erro * Ts;
  erro_integral = saturar(erro_integral, INTEGRAL_MIN, INTEGRAL_MAX);
  float termo_i = Ki * erro_integral;
  
  // Termo D (com filtro)
  float derivada = (erro - erro_anterior) / Ts;
  derivada_filtrada = ALPHA_FILTRO * derivada + (1.0f - ALPHA_FILTRO) * derivada_filtrada;
  float termo_d = Kd * derivada_filtrada;
  
  // Controle total
  float u = termo_p + termo_i + termo_d;
  float u_sat = saturar(u, PWM_MIN, PWM_MAX);

  /*float u = termo_p + termo_i + termo_d;
float u_sat = saturar(u, -255.0f, 255.0f);
bra.move1D((int)u_sat); */
  
  // Anti-windup: compensar integral se saturou
  if (u != u_sat && Ki != 0) {
    float excesso = u - u_sat;
    erro_integral -= (excesso / Ki) * 0.5f;
  }
  
  // Aplicar aos motores
  bra.move1D((int)u_sat);
  
  // Atualizar estado
  erro_anterior = erro;
}