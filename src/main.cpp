#include <Arduino.h>
#include <Baratinha.h>

Baratinha bra;

namespace {

  /// Período de amostragem do controlador (100 Hz)
  const float Ts = 0.01f;
  
  /// Ganhos do controlador PID digital (calculados via algoritmo genético)
  const float Kp = 0.010806919f;
  const float Ki = 0.00014681f;
  const float Kd = 0.38340489f;



  /// Distância alvo em milímetros (10 cm)
  const float setpoint = 100.0f;
  
  float erro_integral = 0.0f;      ///< Acumulador do termo integral
  float erro_anterior = 0.0f;      ///< Erro da iteração anterior (para cálculo da derivada)
  float derivada_filtrada = 0.0f;  ///< Derivada com filtro passa-baixas aplicado
  

  const float INTEGRAL_MAX = 65.0f;  ///< Limite superior anti-windup
  const float INTEGRAL_MIN = -65.0f; ///< Limite inferior anti-windup
  
  /// Coeficiente do filtro para derivada (filtro passa-baixas exponencial)
  const float ALPHA_FILTRO = 1.0f;
  
  const float PWM_MAX = 255.0f;  ///< Limite superior do PWM dos motores
  const float PWM_MIN = -255.0f; ///< Limite inferior do PWM dos motores
  
  const float DIST_MIN = 30.0f;  ///< Distância mínima de segurança (3 cm)
}

// ============================================================
// FUNÇÕES AUXILIARES
// ============================================================

/**
 * @brief Limita um valor entre um mínimo e um máximo
 * @param valor Valor a ser saturado
 * @param minimo Limite inferior
 * @param maximo Limite superior
 * @return Valor saturado
 */
inline float saturar(float valor, float minimo, float maximo) {
  if (valor > maximo) return maximo;
  if (valor < minimo) return minimo;
  return valor;
}

/**
 * @brief Reseta todas as variáveis de estado do controlador
 * @note Deve ser chamada ao parar o robô
 */
void resetar_controle() {
  erro_integral = 0.0f;
  erro_anterior = 0.0f;
  derivada_filtrada = 0.0f;
}

// ============================================================
// CONFIGURAÇÃO INICIAL
// ============================================================

void setup() {
  bra.recoveryMode();
  bra.setupAll();
  bra.setControlInterval(Ts);
  bra.awaitStart();
}

// ============================================================
// LOOP PRINCIPAL DE CONTROLE
// ============================================================

void loop() {
  bra.updateStartStop();
  
  if (!bra.isRunning()) {
    bra.stop();
    resetar_controle();
    return;
  }
  
  if (!bra.controlTickDue()) return;
  
  // ============================================================
  // AQUISIÇÃO E VALIDAÇÃO DA LEITURA DO SENSOR
  // ============================================================
  
  float distancia = bra.readDistance();

  
  // Segurança: para imediatamente se muito próximo do obstáculo
  if (distancia < DIST_MIN) {
    bra.stop();
    return;
  }
  
  // ============================================================
  // CÁLCULO DO CONTROLADOR PID
  // ============================================================
  
  // Cálculo do erro com sinal correto:
  // Se robô está longe (dist > setpoint) → erro positivo → avança
  // Se robô está perto (dist < setpoint) → erro negativo → recua
  float erro = distancia - setpoint;
  
  // --- Termo Proporcional ---
  float termo_p = Kp * erro;
  
  // --- Termo Integral com Anti-Windup ---
  erro_integral = saturar(erro_integral + erro * Ts, INTEGRAL_MIN, INTEGRAL_MAX);
  float termo_i = Ki * erro_integral;
  
  // --- Termo Derivativo com Filtro Passa-Baixas ---
  float derivada = (erro - erro_anterior) / Ts;


  float termo_d = ALPHA_FILTRO * derivada_filtrada + ((1.0f - ALPHA_FILTRO) * Kd * derivada);
  
  // --- Sinal de Controle Total ---
  float u = termo_p + termo_i + termo_d;
  float u_sat = saturar(u, -1, 1);

  int velPID = u_sat * 255.0f;
  
  // --- Anti-Windup Condicional ---
  if (u*255 != velPID && Ki != 0.0f) {
    erro_integral -= ((u*255 - velPID) / Ki) * 0.5f;
  }
  
  // ============================================================
  // APLICAÇÃO DO SINAL DE CONTROLE AOS MOTORES
  // ============================================================
  
  bra.move1D(velPID);
  erro_anterior = erro;
  derivada_filtrada = termo_d;
}
