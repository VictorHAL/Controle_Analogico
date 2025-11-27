#include <Arduino.h>
#include <Baratinha.h>
#include <math.h>

Baratinha bra; // Instancia unica do robo Baratinha

namespace { // Escopo anonimo para constantes e variaveis globais
  const float Ts = 0.01f; // Periodo de controle (0.01s = 10ms) [cite: 13]
  
  // =========================================================
  // AJUSTE AQUI OS GANHOS CALCULADOS NA SIMULAÇÃO (PARTE C)
  // =========================================================
  // Insira os valores que deram o gráfico estável (sem passar 30% de overshoot)
  const float Kp = 0.04134; 
  const float Ki = 0.00097;
  const float Kd = 0.67774;

  // Setpoint: Distância desejada do obstáculo (em CM)
  const float SETPOINT_CM = 15.0f; // Ex: Manter 15cm de distância

  // Variáveis de memória do PID
  float erro_anterior = 0.0f;
  float integral = 0.0f;
  float derivativo_filtrado = 0.0f;
  
  // Parâmetro do filtro do derivativo (suavização de ruído) 
  // Valor entre 0.0 e 1.0. (0.7 significa que mantemos 70% do histórico e aceitamos 30% da novidade)
  const float alpha = 0.7f; 

  // Limites do PWM (O método move1D aceita int, assumindo max 255 ou 100 dependendo da lib)
  // Vamos assumir padrão Arduino (255). Se o robô for muito rápido, baixa para 150 ou 200.
  const float PWM_MAX = 255.0f;
  const float PWM_MIN = -255.0f;
}

void setup() {
  bra.recoveryMode(); // Modo de recuperacao
  bra.setupAll();     // Configuracao completa do hardware padrao
  
  bra.setControlInterval(Ts); // Define o periodo de controle
  bra.awaitStart();           // Aguarda o toque no botao
  
  bra.println("--- INICIANDO CONTROLE PID DISCRETO ---");
}

void loop() {
  bra.updateStartStop();          // Atualiza o estado de start/stop
  if (!bra.isRunning()) {
    // Se o robô for pausado pelo botão, resetamos a integral para não acumular erro parado
    integral = 0.0f;
    erro_anterior = 0.0f;
    bra.stop();
    return; 
  }
  
  if (!bra.controlTickDue()) return; // Garante a execução exata no tempo Ts

  // ============================================================
  // ALGORITMO DE CONTROLE (Executado a cada 0.01s)
  // ============================================================

  // 1. LEITURA E CONVERSÃO [cite: 100]
  float leitura_mm = bra.readDistance();
  float distancia_cm = leitura_mm / 10.0f; // Converte mm para cm

  // 2. SEGURANÇA 
  // Se a distância for menor que 3cm (30mm), para tudo.
  if (distancia_cm < 3.0f) {
    bra.stop();
    integral = 0.0f; // Reset do integrador
    bra.println("ALERTA: Muito perto! Parada de emergencia.");
    return;
  }

  // 3. CÁLCULO DO ERRO
  // Lógica: Se Distancia > Setpoint (Longe) -> Erro positivo -> Motor anda para frente (+)
  // Se Distancia < Setpoint (Perto) -> Erro negativo -> Motor anda para trás (-)
  float erro = distancia_cm - SETPOINT_CM;

  // 4. PID (Forma Posicional) [cite: 14, 91]
  
  // Termo Proporcional
  float P = Kp * erro;

  // Termo Integral
  integral += erro; 
  float I = Ki * integral;

  // Termo Derivativo com Filtro 
  // D_raw = (erro - erro_anterior);
  // Filtro: Y[k] = alpha*Y[k-1] + (1-alpha)*X[k]
  float D_raw = (erro - erro_anterior);
  derivativo_filtrado = (alpha * derivativo_filtrado) + ((1.0f - alpha) * D_raw);
  float D = Kd * derivativo_filtrado;

  // Sinal de Controle (u)
  float u = P + I + D;

  // 5. SATURAÇÃO E ANTI-WINDUP [cite: 93]
  // Se o sinal exceder o físico do motor, cortamos e impedimos o integrador de crescer
  if (u > PWM_MAX) {
    u = PWM_MAX;
    integral -= erro; // Clamping (Anti-windup): desfaz a soma da integral
  } else if (u < PWM_MIN) {
    u = PWM_MIN;
    integral -= erro; // Clamping
  }

  // 6. ATUAÇÃO
  bra.move1D((int)u);

  // 7. ATUALIZAÇÃO PARA O PRÓXIMO CICLO
  erro_anterior = erro;

  // Telemetria para o Serial Plotter (opcional, ajuda a calibrar)
  // Formato: Distancia, Setpoint, PWM
  bra.printf("Dist:%.2f Set:%.2f PWM:%.0f\n", distancia_cm, SETPOINT_CM, u);
}