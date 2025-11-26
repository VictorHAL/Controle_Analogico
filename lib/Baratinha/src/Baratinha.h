#pragma once

#include <Arduino.h>
#include <FastLED.h>
#include <VL53L0X.h>
#include <driver/gpio.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_wifi.h>

/// @brief Classe utilitaria que concentra toda a interacao com o robo Baratinha:
/// sensores, motores, LEDs, botao start/stop e telemetria.
class Baratinha final {
public:
  /// Quantidade fixa de LEDs enderecaveis no chassi.
  static constexpr uint8_t kNumLeds = 4;

  /// Intervalo padrao (em microssegundos) para o laco de controle de 100 Hz.
  static constexpr uint32_t kDefaultControlIntervalUs = 10000;

  Baratinha();

  /// Inicializa a porta serial (conveniente para logs e telemetria).
  void beginSerial(uint32_t baud = 115200);

  /// Executa todos os setups essenciais do robo.
  bool setupAll(uint32_t serialBaud = 115200,
                uint8_t ledBrightness = 100,
                uint32_t pwmFrequency = 20000,
                uint8_t pwmResolutionBits = 8,
                uint8_t tofSda = 2,
                uint8_t tofScl = 4,
                uint16_t tofTimeoutMs = 500);

  /// Configura e inicia o sensor ToF (VL53L0X).
  /// @return true se o sensor respondeu corretamente.
  bool setupTOF(uint8_t sda = 2, uint8_t scl = 4, uint16_t timeoutMs = 500);

  /// Configura a matriz de LEDs enderecaveis e limpa o estado inicial.
  void setupLeds(uint8_t brightness = 100);

  /// Configura os canais PWM e os pinos utilizados pelos motores.
  void setupMotors(uint32_t pwmFrequency = 20000, uint8_t pwmResolutionBits = 8);

  /// Configura o botao analogico de start/stop.
  void setupButtons();

  /// Executa a animacao de boot e aguarda o toque no botao para liberar o robo.
  void awaitStart(uint16_t pollDelayMs = 100);

  /// Le o botao e alterna o estado de execucao quando houver borda de subida.
  void updateStartStop();

  /// Indica se o robo esta em modo "rodando".
  bool isRunning() const;

  /// Forca o estado de start/stop manualmente.
  void setRunning(bool running);

  /// Verifica se ja se passou o intervalo configurado para o controle.
  bool controlTickDue();

  /// Ajusta o periodo entre chamadas de controle (em microssegundos).
  void setControlIntervalUs(uint32_t intervalUs);

  /// Ajusta o periodo entre chamadas de controle (em segundos).
  void setControlInterval(float seconds);

  /// Consulta o periodo atual do controle em microssegundos.
  uint32_t controlIntervalUs() const;

  /// Consulta o periodo atual do controle em segundos.
  float controlPeriodSeconds() const;

  /// Leitura bruta do sensor ToF em milimetros.
  float readDistance();

  /// Movimento unidimensional (mesma velocidade nos dois motores).
  /// @param pwm Faixa sugerida: -255 (re) a +255 (frente).
  void move1D(int pwm, bool light = false);

  void move(int pwmE, int pwmD);

  /// Para os motores imediatamente.
  void stop();

  /// Ajusta cor individual ou global dos LEDs (usar 'a' para todos).
  void setColor(char ledId, uint8_t h, uint8_t s, uint8_t v);

  /// Liga ou desliga a telemetria serial.
  void enableTelemetry(bool enable);

  /// Define o divisor para envio de telemetria (1 = toda iteracao de controle).
  void setTelemetryDivider(uint8_t divider);

  /// Configura OTA no modo station (conectando em uma rede WiFi existente).
  /// @return true se conectou com sucesso.
  bool setupOTAStation(const char* ssid, const char* password);

  /// Configura OTA no modo Access Point (cria rede propria, usando nomes padrao se necessario).
  /// @return true se o AP foi iniciado com sucesso.
  bool setupOTAAccessPoint(const char* ssid, const char* password);

  /// Liga ou desliga o processamento do OTA.
  void enableOTA(bool enable);

  /// Habilita/desabilita o servidor Telnet para telemetria.
  void enableTelnetTelemetry(bool enable, uint16_t port = 23);

  /// Replica Serial.print em todos os canais (USB + Telnet).
  template <typename T>
  void print(const T& value) {
    Serial.print(value);
    if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
      _telnetClient.print(value);
    }
  }

  /// Versao de print com argumento extra (base/digitos).
  template <typename T>
  void print(const T& value, int format) {
    Serial.print(value, format);
    if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
      _telnetClient.print(value, format);
    }
  }

  /// Versao com quebra de linha automatica.
  template <typename T>
  void println(const T& value) {
    Serial.println(value);
    if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
      _telnetClient.println(value);
    }
  }

  /// println com argumento extra (base/digitos).
  template <typename T>
  void println(const T& value, int format) {
    Serial.println(value, format);
    if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
      _telnetClient.println(value, format);
    }
  }

  /// Equivalente a Serial.println() sem argumentos.
  void println();

  /// printf compartilhado (via Serial e Telnet).
  void printf(const char* format, ...);

  /// Define credenciais para o modo station (opcional).
  void setStationCredentials(const char* ssid, const char* password);

  /// Define credenciais para o modo Access Point (opcional).
  void setAccessPointCredentials(const char* ssid, const char* password);

  /// Escolhe qual modo tentar primeiro (false = preferir station).
  void setPreferAccessPoint(bool prefer);

  /// Define o hostname usado pelo mDNS (padrao: "baratinha").
  void setHostname(const char* hostname);

  /// Permite forcar o modo de recuperacao manualmente.
  void recoveryMode();

  /// Emite a linha de telemetria com o formato esperado pelas ferramentas atuais.
  void emitTelemetry(float setpoint, float measurement, float error,
                     float controlRaw, float controlLimited,
                     float pTerm, float iTerm, float dTerm);

private:
  static constexpr gpio_num_t kIn1 = GPIO_NUM_39;
  static constexpr gpio_num_t kIn2 = GPIO_NUM_45;
  static constexpr gpio_num_t kIn3 = GPIO_NUM_40;
  static constexpr gpio_num_t kIn4 = GPIO_NUM_41;
  static constexpr gpio_num_t kPwmM1Pin = GPIO_NUM_46;
  static constexpr gpio_num_t kPwmM2Pin = GPIO_NUM_42;
  static constexpr gpio_num_t kButtonPin = GPIO_NUM_17;
  static constexpr uint8_t kLedDataPin = 48;
  static constexpr uint8_t kPwmChannelM1 = 0;
  static constexpr uint8_t kPwmChannelM2 = 1;
  static constexpr int kButtonLowThreshold = 30;
  static constexpr int kButtonHighThreshold = 100;

  
  void motorE_PWM(int vel);
  void motorD_PWM(int vel);
  void applyBootAnimation();
  void applyResumeAnimation();
  void applyPauseIndicator();
  void updateStartStopState(int rawValue);
  int readButtonRaw() const;
  void setAllLeds(uint8_t h, uint8_t s, uint8_t v);
  void resetControlTick();
  void processOTA();
  void processTelnet();
  void startTelnetServer(uint16_t port);
  void startMDNS();
  void configureOTAHandlers();
  void showOTAStartAnimation();
  void showOTAProgressAnimation(unsigned int progress, unsigned int total);
  void showOTAEndAnimation();
  void showOTAFailAnimation();
  void checkRecoveryMode();
  void startConfigPortal();
  void handleConfigPage();
  void handleConfigSave();
  void handleCloneRequest();
  void stopConfigPortal();
  bool parseMacAddress(const String& mac, uint8_t out[6]);
  String getConnectedStationMac() const;
  static String formatMac(const uint8_t* mac);
  void loadPreferences();
  void savePreferences();
  void showStationConnectingFrame(bool bright);
  void showStationConnectedAnimation();
  void showAPTransitionAnimation();
  void showAPReadyAnimation();
  void broadcastRaw(const char* message);
  bool autoConfigureOTA();
  bool configureOTA(bool preferAccessPoint,
                    const char* staSsid,
                    const char* staPassword,
                    const char* apSsid,
                    const char* apPassword);

  VL53L0X _tof;
  CRGB _leds[kNumLeds];
  bool _tofReady;
  bool _running;
  bool _telemetryEnabled;
  uint8_t _telemetryDivider;
  uint8_t _telemetryCounter;
  int _buttonLast;
  int _buttonCurrent;
  uint32_t _lastControlMicros;
  uint32_t _controlIntervalUs;
  bool _otaEnabled;
  bool _otaConfigured;
  WiFiServer _telnetServer;
  WiFiClient _telnetClient;
  bool _telnetEnabled;
  bool _telnetServerActive;
  uint16_t _telnetPort;
  bool _otaInProgress;
  uint8_t _otaAnimationIndex;
  String _staSsid;
  String _staPassword;
  String _apSsid;
  String _apPassword;
  bool _preferAccessPoint;
  bool _mdnsRunning;
  String _mdnsHostname;
  bool _recoveryForced;
  WebServer* _configServer = nullptr;
  bool _configPortalRunning = false;
  DNSServer _dnsServer;
  bool _dnsRunning = false;
  Preferences _prefs;
  bool _prefsReady = false;

  String _staMacToSave;

};
