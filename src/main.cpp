#include <Arduino.h>
#include <Baratinha.h>
#include <math.h>

Baratinha bra; // Instancia unica do robo Baratinha

namespace { // Escopo anonimo para constantes locais
  const float Ts = 0.01f; // Periodo de controle em segundos


}

void setup() {
  bra.recoveryMode(); // Modo de recuperacao (verifica botao pressionado no inicio para iniciar o setup de rede wifi)
  bra.setupAll(); // Configuracao completa do hardware padrao


  bra.setControlInterval(Ts); // Define o periodo de controle
  bra.awaitStart(); // Aguarda o toque no botao para iniciar a execucao
}

void loop() {
  bra.updateStartStop(); // Atualiza o estado de start/stop baseado no botao
  if (!bra.isRunning()) return; // Sai se nao estiver em modo de execucao
  if (!bra.controlTickDue()) return; // Sai se nao for hora do proximo ciclo de controle

  // restante do controle aqui:
}

/*
Métodos úteis da classe Baratinha:
- bool setupAll() : Inicializa todos os componentes de hardware.
- void setControlInterval(float seconds) : Define o intervalo de controle em segundos.
- bool controlTickDue() : Verifica se é hora de executar o próximo ciclo de controle
- void println(const char* msg) : Imprime uma mensagem na porta serial.
- void printf(const char* format, ...) : Imprime uma mensagem formatada na porta serial
- bool isRunning() : Verifica se o robô está em modo de execução.
- void updateStartStop() : Atualiza o estado de start/stop com base no botão.
- void move1D(int pwm, bool light = false) : Move o robô em uma direção com velocidade especificada.
- void stop() : Para os motores imediatamente.
- float readDistance() : Lê a distância do sensor ToF em milímetros.
*/