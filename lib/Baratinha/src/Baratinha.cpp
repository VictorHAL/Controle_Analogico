#include "Baratinha.h"

#include <Wire.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <stdarg.h>
#include <stdio.h>
#include <cstring>

namespace {
constexpr char kDefaultApSsid[] = "Baratinha-OTA";
constexpr char kDefaultApPassword[] = "12345678";
constexpr char kDefaultHostname[] = "baratinha";
constexpr char kRecoveryApSsid[] = "Baratinha-Config";
}

Baratinha::Baratinha()
    : _tof(),
      _leds{},
      _tofReady(false),
      _running(true),
      _telemetryEnabled(false),
      _telemetryDivider(5),
      _telemetryCounter(0),
      _buttonLast(0),
      _buttonCurrent(0),
      _lastControlMicros(0),
      _controlIntervalUs(kDefaultControlIntervalUs),
      _otaEnabled(true),
      _otaConfigured(false),
      _telnetServer(23),
      _telnetClient(),
      _telnetEnabled(true),
      _telnetServerActive(false),
      _telnetPort(23),
      _otaInProgress(false),
      _otaAnimationIndex(0),
      _staSsid(),
      _staPassword(),
      _apSsid(kDefaultApSsid),
      _apPassword(kDefaultApPassword),
      _preferAccessPoint(false),
      _mdnsRunning(false),
      _mdnsHostname(kDefaultHostname),
      _recoveryForced(false),
      _configServer(nullptr),
      _configPortalRunning(false),
      _dnsRunning(false),
      _prefs(),
      _prefsReady(false),
      _staMacToSave("") {}

void Baratinha::beginSerial(uint32_t baud) {
  Serial.begin(baud);
}

bool Baratinha::setupAll(uint32_t serialBaud,
                         uint8_t ledBrightness,
                         uint32_t pwmFrequency,
                         uint8_t pwmResolutionBits,
                         uint8_t tofSda,
                         uint8_t tofScl,
                         uint16_t tofTimeoutMs) {
  beginSerial(serialBaud);
  loadPreferences();
  Serial.println(F("[Baratinha] Iniciando setup completo."));

  setupLeds(ledBrightness);
  Serial.println(F("[Baratinha] LEDs configurados."));

  setupMotors(pwmFrequency, pwmResolutionBits);
  Serial.println(F("[Baratinha] Motores configurados."));

  setupButtons();
  Serial.println(F("[Baratinha] Botao pronto."));
  if (!_recoveryForced) {
    checkRecoveryMode();
  }

  if (!setupTOF(tofSda, tofScl, tofTimeoutMs)) {
    Serial.println(F("[Baratinha] ERRO: Sensor ToF nao respondeu."));
    setAllLeds(0, 255, 40);
    FastLED.show();
    if (_otaEnabled) {
      autoConfigureOTA();
    }
    return false;
  }

  Serial.println(F("[Baratinha] Sensor ToF pronto."));
  setAllLeds(96, 255, 80);
  FastLED.show();
  Serial.println(F("[Baratinha] Setup concluido."));

  if (_otaEnabled) {
    autoConfigureOTA();
  }
  return true;
}

bool Baratinha::setupTOF(uint8_t sda, uint8_t scl, uint16_t timeoutMs) {
  Wire.begin(sda, scl);
  _tof.setTimeout(timeoutMs);
  if (!_tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    _tofReady = false;
    return false;
  }

  _tof.startContinuous();
  _tofReady = true;
  return true;
}

void Baratinha::setupLeds(uint8_t brightness) {
  FastLED.addLeds<WS2812B, kLedDataPin, RGB>(_leds, kNumLeds);
  FastLED.setBrightness(brightness);
  FastLED.clear();
  FastLED.show();
}

void Baratinha::setupMotors(uint32_t pwmFrequency, uint8_t pwmResolutionBits) {
  ledcSetup(kPwmChannelM1, pwmFrequency, pwmResolutionBits);
  ledcAttachPin(kPwmM1Pin, kPwmChannelM1);

  ledcSetup(kPwmChannelM2, pwmFrequency, pwmResolutionBits);
  ledcAttachPin(kPwmM2Pin, kPwmChannelM2);

  gpio_set_direction(kIn1, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(kIn2, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(kIn3, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(kIn4, GPIO_MODE_INPUT_OUTPUT);
}

void Baratinha::setupButtons() {
  gpio_set_direction(kButtonPin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(kButtonPin, GPIO_PULLDOWN_ONLY);
  gpio_pulldown_en(kButtonPin);

  _buttonCurrent = readButtonRaw();
  _buttonLast = _buttonCurrent;
}

void Baratinha::awaitStart(uint16_t pollDelayMs) {
  applyBootAnimation();

  Serial.println(F("Aguardando inicio"));
  setAllLeds(0, 255, 30);
  FastLED.show();

  _buttonCurrent = readButtonRaw();
  _buttonLast = _buttonCurrent;

  while (!(_buttonLast < kButtonLowThreshold && _buttonCurrent > kButtonHighThreshold)) {
    delay(pollDelayMs);
    _buttonLast = _buttonCurrent;
    _buttonCurrent = readButtonRaw();
    Serial.println(_buttonCurrent);
    processOTA();
  }

  applyResumeAnimation();
  Serial.println(F("Iniciou"));

  setAllLeds(120, 255, 30);
  FastLED.show();
  Serial.println(F("Fim setup."));

  _running = true;
  resetControlTick();
}

void Baratinha::updateStartStop() {
  _buttonLast = _buttonCurrent;
  _buttonCurrent = readButtonRaw();
  updateStartStopState(_buttonCurrent);
  processOTA();
}

bool Baratinha::isRunning() const {
  return _running;
}

void Baratinha::setRunning(bool running) {
  if (_running == running) {
    return;
  }

  _running = running;
  if (_running) {
    applyResumeAnimation();
    Serial.println(F("Despausou"));
    resetControlTick();
  } else {
    applyPauseIndicator();
    stop();
    Serial.println(F("Pausou"));
  }
}

bool Baratinha::controlTickDue() {
  const uint32_t now = micros();

  if ((now - _lastControlMicros) >= _controlIntervalUs) {
    // Ajusta o contador interno para evitar drift excessivo.
    _lastControlMicros += _controlIntervalUs;
    if ((now - _lastControlMicros) >= _controlIntervalUs) {
      _lastControlMicros = now;
    }
    return true;
  }
  return false;
}

void Baratinha::setControlIntervalUs(uint32_t intervalUs) {
  if (intervalUs == 0) {
    intervalUs = kDefaultControlIntervalUs;
  }
  _controlIntervalUs = intervalUs;
  resetControlTick();
}

void Baratinha::setControlInterval(float seconds) {
  if (!(seconds > 0.0f)) {
    setControlIntervalUs(kDefaultControlIntervalUs);
    return;
  }
  const uint32_t intervalUs = static_cast<uint32_t>(seconds * 1000000.0f);
  setControlIntervalUs(intervalUs);
}

uint32_t Baratinha::controlIntervalUs() const {
  return _controlIntervalUs;
}

float Baratinha::controlPeriodSeconds() const {
  return static_cast<float>(_controlIntervalUs) / 1000000.0f;
}

float Baratinha::readDistance() {
  if (!_tofReady) {
    return 0;
  }
  float distance = static_cast<float>(_tof.readRangeContinuousMillimeters());
  return distance < 0.0f ? 0.0f : distance;
}

void Baratinha::move1D(int pwm, bool light) {
  const int magnitude = abs(pwm);
  const int bright = (magnitude < 20) ? 0 : magnitude;

  if (light) {
    if (pwm < 0) {
      _leds[0] = CHSV(200, 255, bright);
      _leds[3] = CHSV(200, 255, 0);
      _leds[1] = CHSV(200, 255, bright);
      _leds[2] = CHSV(200, 255, 0);
    } else {
      _leds[0] = CHSV(100, 255, 0);
      _leds[3] = CHSV(100, 255, bright);
      _leds[1] = CHSV(100, 255, 0);
      _leds[2] = CHSV(100, 255, bright);
    }
    FastLED.show();
  }

  motorE_PWM(pwm);
  motorD_PWM(pwm);
}

void Baratinha::move(int pwmE, int pwmD) {

  motorE_PWM(pwmE);
  motorD_PWM(pwmD);
}


void Baratinha::stop() {
  motorE_PWM(0);
  motorD_PWM(0);
}

void Baratinha::setColor(char ledId, uint8_t h, uint8_t s, uint8_t v) {
  switch (ledId) {
    case '0':
    case 0:
      _leds[0] = CHSV(h, s, v);
      break;
    case '1':
    case 1:
      _leds[1] = CHSV(h, s, v);
      break;
    case '2':
    case 2:
      _leds[2] = CHSV(h, s, v);
      break;
    case '3':
    case 3:
      _leds[3] = CHSV(h, s, v);
      break;
    case 'a':
    default:
      setAllLeds(h, s, v);
      break;
  }

  FastLED.show();
}

void Baratinha::enableTelemetry(bool enable) {
  _telemetryEnabled = enable;
}

void Baratinha::setTelemetryDivider(uint8_t divider) {
  if (divider == 0) {
    divider = 1;
  }
  _telemetryDivider = divider;
  _telemetryCounter = 0;
}

bool Baratinha::setupOTAStation(const char* ssid, const char* password) {
  _otaConfigured = false;
  if (ssid == nullptr || ssid[0] == '\0') {
    Serial.println(F("Credenciais invalidas para OTA station."));
    return false;
  }

  bool pulse = false;
  showStationConnectingFrame(pulse);

  WiFi.mode(WIFI_STA);

  // --- ADICIONE ESTE BLOCO DE CÓDIGO AQUI ---
  if (_staMacToSave.length() == 17) {
    uint8_t parsed[6];
    if (parseMacAddress(_staMacToSave, parsed)) {
        Serial.print(F("[Baratinha] Aplicando MAC clonado (STA): "));
        Serial.println(_staMacToSave);
        esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, parsed);
        if (err != ESP_OK) {
          Serial.print(F("[Baratinha] ERRO ao aplicar MAC: "));
          Serial.println(err);
        }
    } else {
        Serial.println(F("[Baratinha] Falha ao parsear MAC, usando de fabrica."));
    }
  }
  // --- FIM DO BLOCO ADICIONADO ---



  if (_mdnsHostname.length()) {
    WiFi.setHostname(_mdnsHostname.c_str());
  }
  if (password != nullptr) {
    WiFi.begin(ssid, password);
  } else {
    WiFi.begin(ssid);
  }
  Serial.printf("Conectando a rede WiFi SSID: %s\n", ssid);
  Serial.printf("Senha: %s\n", (password != nullptr) ? password : "<nenhuma>");
  Serial.print(F("[Baratinha] Conectando! MAC Address em uso (STA): "));
  Serial.println(WiFi.macAddress());


  Serial.print(F("Conectando OTA (station)"));
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 20000) {
    delay(500);
    Serial.print('.');
    pulse = !pulse;
    showStationConnectingFrame(pulse);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Falha ao conectar a rede WiFi para OTA."));
    return false;
  }

  showStationConnectedAnimation();

  Serial.print(F("[Baratinha] Conectado! MAC Address em uso (STA): "));
  Serial.println(WiFi.macAddress());

  configureOTAHandlers();
  ArduinoOTA.begin();
  _otaConfigured = true;
  startTelnetServer(_telnetPort);
  startMDNS();

  Serial.print(F("OTA pronto. IP: "));
  Serial.println(WiFi.localIP());
  return true;
}

bool Baratinha::setupOTAAccessPoint(const char* ssid, const char* password) {
  _otaConfigured = false;
  const char* apSsid = (ssid != nullptr && ssid[0] != '\0') ? ssid : kDefaultApSsid;
  const char* apPassword = (password != nullptr && password[0] != '\0') ? password : kDefaultApPassword;

  showAPTransitionAnimation();  
  // --- ADICIONE ESTAS 2 LINHAS ---
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  WiFi.mode(WIFI_AP);

  // --- ADICIONE ESTE BLOCO DE CÓDIGO AQUI ---
  if (_staMacToSave.length() == 17) {
    uint8_t parsed[6];
    if (parseMacAddress(_staMacToSave, parsed)) {
        Serial.print(F("[Baratinha] Aplicando MAC clonado (AP): "));
        Serial.println(_staMacToSave);
        esp_err_t err = esp_wifi_set_mac(WIFI_IF_AP, parsed); // <--- Note o WIFI_IF_AP
        if (err != ESP_OK) {
          Serial.print(F("[Baratinha] ERRO ao aplicar MAC (AP): "));
          Serial.println(err);
        }
    } else {
        Serial.println(F("[Baratinha] Falha ao parsear MAC, usando de fabrica."));
    }
  }
  // --- FIM DO BLOCO ADICIONADO ---

  if (_mdnsHostname.length()) {
    WiFi.softAPsetHostname(_mdnsHostname.c_str());
  }
  if (!WiFi.softAP(apSsid, apPassword)) {
    Serial.println(F("Falha ao iniciar access point para OTA."));
    return false;
  }

  showAPReadyAnimation();

  configureOTAHandlers();
  ArduinoOTA.begin();
  _otaConfigured = true;
  startTelnetServer(_telnetPort);
  startMDNS();

  Serial.print(F("OTA AP pronto. IP: "));
  Serial.println(WiFi.softAPIP());
  return true;
}

void Baratinha::enableOTA(bool enable) {
  if (_otaEnabled == enable) {
    return;
  }
  _otaEnabled = enable;
  if (!enable) {
    _otaInProgress = false;
    _otaConfigured = false;
    if (_telnetClient) {
      _telnetClient.stop();
    }
    if (_telnetServerActive) {
      _telnetServer.stop();
      _telnetServerActive = false;
    }
    if (_mdnsRunning) {
      MDNS.end();
      _mdnsRunning = false;
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  } else {
    autoConfigureOTA();
  }
}

void Baratinha::enableTelnetTelemetry(bool enable, uint16_t port) {
  _telnetEnabled = enable;
  _telnetPort = port;

  if (!enable) {
    if (_telnetClient) {
      _telnetClient.stop();
    }
    if (_telnetServerActive) {
      _telnetServer.stop();
      _telnetServerActive = false;
    }
    return;
  }

  if (_otaConfigured) {
    startTelnetServer(_telnetPort);
  }
}

void Baratinha::setStationCredentials(const char* ssid, const char* password) {
  _staSsid = (ssid != nullptr) ? ssid : "";
  _staPassword = (password != nullptr) ? password : "";
}

void Baratinha::setAccessPointCredentials(const char* ssid, const char* password) {
  _apSsid = (ssid != nullptr && ssid[0] != '\0') ? ssid : kDefaultApSsid;
  _apPassword = (password != nullptr && password[0] != '\0') ? password : kDefaultApPassword;
}

void Baratinha::setPreferAccessPoint(bool prefer) {
  _preferAccessPoint = prefer;
}

void Baratinha::checkRecoveryMode() {
  const int raw = readButtonRaw();
  if (raw > kButtonHighThreshold) {
    if (!_recoveryForced) {
      Serial.println(F("[Baratinha] Recovery mode detectado (botao pressionado)."));
    }
    setupLeds();
    _recoveryForced = true;
    _preferAccessPoint = true;
    setAccessPointCredentials(nullptr, nullptr);
    setHostname(kDefaultHostname);
    startConfigPortal();
    return;
  }

  _recoveryForced = false;

  
  if (!_preferAccessPoint && _staSsid.isEmpty()) {
    Serial.println(F("[Baratinha] Preferencia station sem credenciais. Forcando AP."));
    _preferAccessPoint = true;
  }
}

void Baratinha::setHostname(const char* hostname) {
  if (hostname == nullptr || hostname[0] == '\0') {
    _mdnsHostname = kDefaultHostname;
  } else {
    _mdnsHostname = hostname;
    _mdnsHostname.toLowerCase();
    _mdnsHostname.replace(" ", "-");
  }

  if (_mdnsRunning) {
    startMDNS();
  }
}

void Baratinha::recoveryMode() {
  setupLeds();
  loadPreferences();
  checkRecoveryMode();
}

void Baratinha::println() {
  Serial.println();
  if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
    _telnetClient.println();
  }
}

void Baratinha::printf(const char* format, ...) {
  if (format == nullptr) {
    return;
  }

  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  broadcastRaw(buffer);
}

bool Baratinha::configureOTA(bool preferAccessPoint,
                             const char* staSsid,
                             const char* staPassword,
                             const char* apSsid,
                             const char* apPassword) {
  bool success = false;
  const bool hasStationCreds = (staSsid != nullptr && staSsid[0] != '\0');

  if (preferAccessPoint) {
    println(F("Preferencia: Access Point."));
    success = setupOTAAccessPoint(apSsid, apPassword);
    if (!success && hasStationCreds) {
      println(F("Falha no modo AP. Tentando conectar em modo station..."));
      success = setupOTAStation(staSsid, staPassword);
    }
  } else {
    println(F("Preferencia: Station."));
    if (hasStationCreds) {
      success = setupOTAStation(staSsid, staPassword);
    } else {
      println(F("Credenciais de station ausentes."));
    }
    if (!success) {
      println(F("Alternando para Access Point."));
      success = setupOTAAccessPoint(apSsid, apPassword);
    }
  }

  if (!success) {
    println(F("Nao foi possivel iniciar OTA em nenhum modo."));
  }
  return success;
}

void Baratinha::configureOTAHandlers() {
  ArduinoOTA.onStart([this]() {
    Serial.println(F("OTA comecou"));
    _otaInProgress = true;
    _otaAnimationIndex = 0;
    showOTAStartAnimation();
  });
  ArduinoOTA.onEnd([this]() {
    Serial.println(F("OTA finalizado"));
    _otaInProgress = false;
    showOTAEndAnimation();
  });
  ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
    const unsigned int percent = (total == 0) ? 0 : (progress * 100U / total);
    Serial.print(F("OTA progresso: "));
    Serial.print(percent);
    Serial.println(F("%"));
    showOTAProgressAnimation(progress, total);
  });
  ArduinoOTA.onError([this](ota_error_t error) {
    Serial.print(F("Erro OTA: "));
    Serial.println(static_cast<int>(error));
    _otaInProgress = false;
    showOTAFailAnimation();
  });
}

void Baratinha::emitTelemetry(float setpoint, float measurement, float error,
                              float controlRaw, float controlLimited,
                              float pTerm, float iTerm, float dTerm) {
  if (!_telemetryEnabled) {
    return;
  }

  if (++_telemetryCounter < _telemetryDivider) {
    return;
  }
  _telemetryCounter = 0;

  auto emitTo = [&](Print& out) {
    out.print(millis());
    out.print(',');
    out.print(setpoint);
    out.print(',');
    out.print(measurement);
    out.print(',');
    out.print(error);
    out.print(',');
    out.print(controlRaw);
    out.print(',');
    out.print(controlLimited);
    out.print(',');
    out.print(pTerm);
    out.print(',');
    out.print(iTerm);
    out.print(',');
    out.println(dTerm);
  };

  emitTo(Serial);
  if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
    emitTo(_telnetClient);
  }
}

void Baratinha::broadcastRaw(const char* message) {
  if (message == nullptr) {
    return;
  }

  Serial.print(message);
  if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
    _telnetClient.print(message);
  }
}

bool Baratinha::autoConfigureOTA() {
  if (!_otaEnabled) {
    return false;
  }
  if (_otaConfigured) {
    return true;
  }

  const char* staSsid = _staSsid.length() ? _staSsid.c_str() : nullptr;
  const char* staPassword = _staPassword.length() ? _staPassword.c_str() : nullptr;
  const char* apSsid = _apSsid.length() ? _apSsid.c_str() : kDefaultApSsid;
  const char* apPassword = _apPassword.length() ? _apPassword.c_str() : kDefaultApPassword;

  return configureOTA(_preferAccessPoint, staSsid, staPassword, apSsid, apPassword);
}

void Baratinha::motorE_PWM(int vel) {
  int duty = vel;
  if (vel < 0) {
    gpio_set_pull_mode(kIn1, GPIO_PULLDOWN_ONLY);
    gpio_set_level(kIn2, 1);
    duty = -vel;
  } else {
    gpio_set_pull_mode(kIn1, GPIO_PULLUP_ONLY);
    gpio_set_level(kIn2, 0);
  }

  duty = constrain(duty, 0, 255);
  ledcWrite(kPwmChannelM1, duty);
}

void Baratinha::motorD_PWM(int vel) {
  int duty = vel;
  if (vel < 0) {
    gpio_set_pull_mode(kIn4, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(kIn3, GPIO_PULLDOWN_ONLY);
    duty = -vel;
  } else {
    gpio_set_pull_mode(kIn4, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(kIn3, GPIO_PULLUP_ONLY);
  }

  duty = constrain(duty, 0, 255);
  ledcWrite(kPwmChannelM2, duty);
}

void Baratinha::applyBootAnimation() {
  setAllLeds(120, 255, 100);
  FastLED.show();
  delay(300);
  processOTA();

  const uint8_t sequence[][3] = {
      {60, 255, 0},
      {60, 255, 100},
      {60, 255, 0},
      {60, 255, 50},
      {60, 255, 0},
      {120, 255, 50},
  };

  const uint16_t delays[] = {170, 170, 170, 170, 170, 500};

  for (size_t i = 0; i < (sizeof(sequence) / sizeof(sequence[0])); ++i) {
    setAllLeds(sequence[i][0], sequence[i][1], sequence[i][2]);
    FastLED.show();
    delay(delays[i]);
    processOTA();
  }
}

void Baratinha::applyResumeAnimation() {
  const uint8_t resumeSequence[][3] = {
      {120, 255, 50},
      {60, 255, 0},
      {60, 255, 50},
      {60, 255, 0},
      {60, 255, 50},
      {60, 255, 0},
      {120, 255, 50},
  };

  const uint16_t resumeDelays[] = {150, 150, 150, 150, 150, 150, 500};

  for (size_t i = 0; i < (sizeof(resumeSequence) / sizeof(resumeSequence[0])); ++i) {
    setAllLeds(resumeSequence[i][0], resumeSequence[i][1], resumeSequence[i][2]);
    FastLED.show();
    delay(resumeDelays[i]);
    processOTA();
  }
}

void Baratinha::applyPauseIndicator() {
  setAllLeds(0, 255, 30);
  FastLED.show();
}

void Baratinha::updateStartStopState(int rawValue) {
  if (_buttonLast < kButtonLowThreshold && rawValue > kButtonHighThreshold) {
    setRunning(!_running);
  }
}

int Baratinha::readButtonRaw() const {
  return analogRead(kButtonPin);
}

void Baratinha::setAllLeds(uint8_t h, uint8_t s, uint8_t v) {
  for (uint8_t i = 0; i < kNumLeds; ++i) {
    _leds[i] = CHSV(h, s, v);
  }
}

void Baratinha::resetControlTick() {
  _lastControlMicros = micros();
}

void Baratinha::processOTA() {
  if (_otaEnabled && _otaConfigured) {
    ArduinoOTA.handle();
  }
  processTelnet();
}

void Baratinha::processTelnet() {
  if (!_telnetEnabled || !_telnetServerActive) {
    return;
  }

  if (!_telnetClient || !_telnetClient.connected()) {
    if (_telnetClient) {
      _telnetClient.stop();
    }
    WiFiClient incoming = _telnetServer.available();
    if (incoming && incoming.connected()) {
      _telnetClient = incoming;
      _telnetClient.println(F("Bem-vindo ao console Telnet da Baratinha!"));
      _telnetClient.println(F("Telemetria sera exibida aqui quando habilitada."));
    }
  } else {
    while (_telnetClient.available() > 0) {
      _telnetClient.read();  // descarta comandos por enquanto
    }
  }
}

void Baratinha::startTelnetServer(uint16_t port) {
  if (!_telnetEnabled) {
    return;
  }

  if (_telnetClient) {
    _telnetClient.stop();
  }

  _telnetServer.stop();
  _telnetServer = WiFiServer(port);
  _telnetServer.begin();
  _telnetServer.setNoDelay(true);
  _telnetServerActive = true;

  Serial.print(F("Telnet ativo na porta "));
  Serial.println(port);
}

void Baratinha::startMDNS() {
  if (!_otaEnabled) {
    return;
  }

  if (_mdnsRunning) {
    MDNS.end();
    _mdnsRunning = false;
  }

  const char* host = _mdnsHostname.length() ? _mdnsHostname.c_str() : kDefaultHostname;
  if (MDNS.begin(host)) {
    _mdnsRunning = true;
    MDNS.addService("telnet", "tcp", _telnetPort);
    MDNS.addService("arduino", "tcp", 3232);
    Serial.print(F("mDNS ativo em http://"));
    Serial.print(host);
    Serial.println(F(".local"));
  } else {
    Serial.println(F("Falha ao iniciar mDNS."));
  }
}

void Baratinha::showOTAStartAnimation() {
  for (uint8_t wave = 0; wave < 12; ++wave) {
    for (uint8_t i = 0; i < kNumLeds; ++i) {
      const uint8_t brightness = (i == (wave % kNumLeds)) ? 200 : 15;
      _leds[i] = CHSV(160, 255, brightness);
    }
    FastLED.show();
    delay(60);
  }
}

void Baratinha::showOTAProgressAnimation(unsigned int progress, unsigned int total) {
  if (!_otaInProgress) {
    return;
  }
  _otaAnimationIndex = (_otaAnimationIndex + 1) % kNumLeds;

  for (uint8_t i = 0; i < kNumLeds; ++i) {
    const uint8_t offset = (i + kNumLeds - _otaAnimationIndex) % kNumLeds;
    uint8_t brightness = 30;
    if (offset == 0) {
      brightness = 200;
    } else if (offset == 1 || offset == (kNumLeds - 1)) {
      brightness = 120;
    }
    _leds[i] = CHSV(160, 255, brightness);
  }

  FastLED.show();
}

void Baratinha::showOTAEndAnimation() {
  for (uint8_t pulse = 0; pulse < 3; ++pulse) {
    setAllLeds(96, 255, 200);
    FastLED.show();
    delay(120);
    setAllLeds(96, 255, 20);
    FastLED.show();
    delay(120);
  }
}

void Baratinha::showOTAFailAnimation() {
  for (uint8_t blink = 0; blink < 6; ++blink) {
    setAllLeds(0, 255, 200);
    FastLED.show();
    delay(120);
    setAllLeds(0, 255, 0);
    FastLED.show();
    delay(90);
  }
}

void Baratinha::showStationConnectingFrame(bool bright) {
  for (uint8_t i = 0; i < kNumLeds; ++i) {
    bool phase = (i % 2 == 0);
    uint8_t value = (phase == bright) ? 180 : 50;
    _leds[i] = CHSV(60, 200, value);
    FastLED.show();
  }
  FastLED.show();
}

void Baratinha::showStationConnectedAnimation() {
  for (uint8_t pulse = 0; pulse < 3; ++pulse) {
    setAllLeds(120, 180, 200);
    FastLED.show();
    delay(80);
    setAllLeds(120, 180, 60);
    FastLED.show();
    delay(80);
    setAllLeds(120, 180, 60);
    FastLED.show();
    delay(80);
    setAllLeds(0, 0, 0);
    FastLED.show();
  }
}

void Baratinha::showAPTransitionAnimation() {
  setAllLeds(25, 180, 150);
  FastLED.show();
}

void Baratinha::showAPReadyAnimation() {
  for (uint8_t pulse = 0; pulse < 2; ++pulse) {
    setAllLeds(240, 100, 200);
    FastLED.show();
    delay(120);
    setAllLeds(240, 100, 80);
    FastLED.show();
    delay(120);
  }
}

void Baratinha::startConfigPortal() {
  loadPreferences();
  const char* apSsid = kRecoveryApSsid;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid);
  setAllLeds(40, 160, 120);
  FastLED.show();

  IPAddress ip = WiFi.softAPIP();
  Serial.print(F("[Baratinha] Portal de configuracao em http://"));
  Serial.print(ip);
  Serial.println('/');

  _dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  _dnsServer.start(53, "*", ip);
  _dnsRunning = true;

  if (!_configServer) {
    _configServer = new WebServer(80);
  }
  auto renderPage = [this]() { handleConfigPage(); };
  auto redirectCaptive = [this]() { handleConfigPage(); };

  _configServer->on("/", renderPage);
  _configServer->on("/save", HTTP_POST, [this]() { handleConfigSave(); });
  _configServer->on("/clone", HTTP_GET, [this]() { handleCloneRequest(); });
  _configServer->on("/generate_204", redirectCaptive);
  _configServer->on("/gen_204", redirectCaptive);
  _configServer->on("/hotspot-detect.html", renderPage);
  _configServer->on("/connecttest.txt", redirectCaptive);
  _configServer->on("/ncsi.txt", redirectCaptive);
  _configServer->on("/fwlink", redirectCaptive);
  _configServer->onNotFound(renderPage);
  _configServer->begin();
  _configPortalRunning = true;

  while (_configPortalRunning) {
    if (_dnsRunning) {
      _dnsServer.processNextRequest();
    }
    _configServer->handleClient();
    delay(10);
  }

  stopConfigPortal();
}

void Baratinha::handleConfigPage() {
  const bool preferAP = _preferAccessPoint;
  const String host = _mdnsHostname.length() ? _mdnsHostname : kDefaultHostname;

  const String staSsidValue = _staSsid;
  const String staPassValue = _staPassword;
  const String apSsidValue = _apSsid.length() ? _apSsid : String(kDefaultApSsid);
  const String apPassValue = _apPassword;
  const String staSsidPlaceholder = staSsidValue.length() ? staSsidValue : String("MinhaRede");
  const String staPassPlaceholder = staPassValue.length() ? staPassValue : String("********");
  const String apSsidPlaceholder = apSsidValue;
  const String apPassPlaceholder = apPassValue.length() ? apPassValue : String("12345678");

  String html = F(
      "<!DOCTYPE html><html lang='pt-br'><head><meta charset='utf-8'/>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
      "<title>Configurar Baratinha</title>"
     "<style>body{font-family:'Segoe UI',sans-serif;background:#0f172a;color:#e2e8f0;"
      "margin:0;padding:0;}"
      "*,*::before,*::after{box-sizing:border-box;}"
      "margin:0;padding:0;}main{max-width:900px;margin:0 auto;padding:32px;}h1{font-size:2rem;"
      "margin-bottom:24px;}form{background:#1e293b;border-radius:18px;padding:32px;"
      "box-shadow:0 15px 35px rgba(0,0,0,.35);}label{display:block;margin-bottom:6px;font-weight:600;}"
      "input,select{width:100%;padding:12px;border-radius:10px;border:1px solid #334155;"
      "background:#0f172a;color:#e2e8f0;margin-bottom:16px;}input[type=checkbox]{width:auto;margin-right:8px;}"
      ".row{display:flex;gap:16px;flex-wrap:wrap;}.row>div{flex:1;min-width:250px;}"
      "button{width:100%;padding:14px;border:none;border-radius:12px;background:#6366f1;"
      "color:#fff;font-weight:600;font-size:1rem;cursor:pointer;transition:.2s;}button:hover{background:#4f46e5;}"
      ".pass-group{display:flex;align-items:center;gap:8px;} .pass-group input{flex:1;margin-bottom:0;}"
      ".pass-group button{width:auto;padding:10px 12px;border-radius:10px;background:#334155;color:#cbd5f5;}"
      ".pass-group button:hover{background:#475569;}"
      ".mac-row{display:flex;gap:10px;align-items:stretch;}"
      ".mac-row input{flex:1;margin-bottom:0;}"
      ".mac-row .ghost{display:flex;align-items:center;justify-content:center;}"
      ".mac-check{margin:8px 0 16px 0;}"
      ".mac-check label{display:flex;align-items:center;gap:8px;}"

      ".ghost{background:#0f172a;border:1px solid #334155;color:#e2e8f0;width:auto;padding:10px 14px;border-radius:10px;}"
      ".ghost:hover{background:#1f2937;}"
      ".tag{display:inline-block;background:#334155;color:#cbd5f5;padding:4px 10px;border-radius:999px;"
      "font-size:.85rem;margin-left:8px;}fieldset{border:1px solid #334155;border-radius:14px;margin-bottom:20px;"
      "padding:16px;}legend{padding:0 10px;}p{line-height:1.5;color:#94a3b8;}</style></head><body>"
      "<main><h1>Configurar Baratinha<span class='tag'>Recovery Mode</span></h1>"
      "<p>Mantenha esta pagina aberta ate salvar suas preferencias. "
      "O robo reiniciara automaticamente com as configuracoes escolhidas.</p>"
      "<form action='/save' method='post'>");

  html += F("<label>Modo de operacao</label><select name='mode' id='modeSelect' onchange='updateMode()'>");
  html += preferAP ? F("<option value='ap' selected>Access Point</option><option value='station'>Station</option>")
                   : F("<option value='ap'>Access Point</option><option value='station' selected>Station</option>");
  html += F("</select>");

  html += F("<fieldset id='stationFields'><legend>Rede Station</legend>"
            "<label>SSID</label><input name='staSsid' value='");
  html += staSsidValue;
  html += F("' placeholder='");
  html += staSsidPlaceholder;
  html += F("'/><label>Senha</label><div class='pass-group'><input name='staPassword' id='staPassword' type='password' value='");
  html += staPassValue;
  html += F("' placeholder='");
  html += staPassPlaceholder;
  html += F("'/><button type='button' onclick=\"toggleVisibility('staPassword', this)\">&#128065;</button></div>"
            "<label><input type='checkbox' id='staOpen' name='staOpen' ");
  if (_staPassword.isEmpty()) {
    html += F("checked");
  }
  html += F(">Rede sem senha</label></fieldset>");

  html += F("<fieldset id='apFields'><legend>Rede Access Point</legend>"
            "<label>SSID</label><input name='apSsid' value='");
  html += apSsidValue;
  html += F("' placeholder='");
  html += apSsidPlaceholder;
  html += F("'/><label>Senha</label><div class='pass-group'><input name='apPassword' id='apPassword' type='password' value='");
  html += apPassValue;
  html += F("' placeholder='");
  html += apPassPlaceholder;
  html += F("'/><button type='button' onclick=\"toggleVisibility('apPassword', this)\">&#128065;</button></div>"
            "<label><input type='checkbox' id='apOpen' name='apOpen' ");
  if (_apPassword.length() < 8) {
    html += F("checked");
  }
  html += F(">Rede sem senha</label></fieldset>");

  html += F("<div class='row'><div><label>Hostname (mDNS)</label>"
          "<input name='hostname' value='");
  html += host;
  html += F("' placeholder='baratinha'/></div>"
            "<div><label>MAC personalizado</label>"
            "<div class='mac-row'><input name='macAddress' id='macField' placeholder='AA:BB:CC:DD:EE:FF'/>"
            "<button type='button' class='ghost' onclick='cloneMacFromClient(event)'>Clonar MAC</button></div></div></div>"
            "<div class='mac-check'><label><input type='checkbox' name='cloneMac'>Aplicar MAC personalizado em STA/AP</label></div>"
            "<button type='submit'>Salvar e Reiniciar</button></form>"
            "<p>Dica: conecte-se ao SSID exibido e acesse <strong>http://");
  html += WiFi.softAPIP().toString();
  html += F("</strong> para retornar a esta pagina.</p>"
            "</main><script>"
            "function toggleVisibility(id,btn){const input=document.getElementById(id);"
            "if(!input)return;const isPass=input.type==='password';input.type=isPass?'text':'password';"
            "btn.innerText=isPass?'\\uD83D\\uDD12':'\\u{1F441}';}"
            "function updateMode(){const sel=document.getElementById('modeSelect');if(!sel)return;"
            "const showStation=sel.value==='station';document.getElementById('stationFields').style.display=showStation?'block':'none';"
            "document.getElementById('apFields').style.display=showStation?'none':'block';}"
            "async function cloneMacFromClient(event){event.preventDefault();try{const res=await fetch('/clone',{cache:'no-store'});"
            "const data=await res.json();if(data.mac){document.getElementById('macField').value=data.mac;}"
            "alert(data.message||'MAC atualizado');}catch(e){alert('Nao foi possivel clonar MAC.');}}"

            "document.addEventListener('DOMContentLoaded',()=>{updateMode();"
            "[['staPassword','staOpen'],['apPassword','apOpen']].forEach(pair=>{const input=document.getElementById(pair[0]);"
            "const check=document.getElementById(pair[1]);if(!input||!check)return;"
            "input.addEventListener('input',()=>{if(input.value.length>0){check.checked=false;}else{check.checked=true;}});"
            "if(input.value.length>0){check.checked=false;}else{check.checked=true;}});});"
            "</script></body></html>");

  if (_configServer) {
    _configServer->send(200, "text/html", html);
  }
}

void Baratinha::handleConfigSave() {
  // --- 1. Coleta todos os dados do formulário ---
  const String mode = (_configServer && _configServer->hasArg("mode")) ? _configServer->arg("mode") : "ap";
  const bool preferAP = (mode == "ap");

  // Atualiza as variáveis de classe
  setPreferAccessPoint(preferAP);

  if (_configServer && _configServer->hasArg("staSsid")) {
      String staSsid = _configServer->arg("staSsid");
      String staPassword = _configServer->hasArg("staOpen") ? "" : _configServer->arg("staPassword");
      setStationCredentials(staSsid.c_str(), staPassword.c_str());
  }

  if (_configServer && _configServer->hasArg("apSsid")) {
      String apSsid = _configServer->arg("apSsid");
      String apPassword = _configServer->hasArg("apOpen") ? "" : _configServer->arg("apPassword");
      setAccessPointCredentials(apSsid.c_str(), apPassword.c_str());
  }

  if (_configServer && _configServer->hasArg("hostname")) {
    setHostname(_configServer->arg("hostname").c_str());
  }


  // Lógica do MAC: Apenas atualiza a variável _staMacToSave
  if (_configServer && _configServer->hasArg("cloneMac")) {
      String mac = _configServer->arg("macAddress");
      uint8_t parsed[6];
      if (parseMacAddress(mac, parsed)) {
          Serial.print(F("[Baratinha] Portal: MAC '"));
          Serial.print(mac);
          Serial.println(F("' sera salvo."));
          _staMacToSave = mac; // Guarda na variável de classe
      } else {
          Serial.println(F("[Baratinha] Portal: MAC invalido, nao sera salvo."));
          _staMacToSave = ""; 
      }
  } else {
      Serial.println(F("[Baratinha] Portal: Checkbox do MAC nao marcado. Limpando MAC salvo."));
      _staMacToSave = ""; // Limpa se o usuário não marcou
  }

  // --- 2. Envia a resposta HTTP (antes de reiniciar) ---

  String response = F("<!DOCTYPE html><html><head><meta charset='utf-8'/>"
                      "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
                      "<style>body{background:#0f172a;color:#e2e8f0;font-family:'Segoe UI',sans-serif;"
                      "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;}"
                      "div{background:#1e293b;padding:32px;border-radius:18px;text-align:center;"
                      "box-shadow:0 20px 45px rgba(0, 0, 0, 0.45);}h2{margin-bottom:16px;}p{color:#94a3b8;}</style>"
                      "</head><body><div><h2>Configuracoes salvas!</h2>"
                      "<p>O robo continuara o boot com os novos parametros.</p>"
                      "<p>Voce pode fechar esta pagina.</p></div></body></html>");
  if (_configServer) {
    _configServer->send(200, "text/html", response);
  }

  // --- 3. Salva TUDO e Reinicia ---
  _configPortalRunning = false;
  savePreferences();

  // GARANTE que os prints saiam antes de reiniciar
  Serial.flush(); // Garante que os prints saiam
  delay(200);
  
  // Força a reinicialização para aplicar o MAC no próximo boot
  Serial.println(F("[Baratinha] Portal: Configuracoes salvas. Reiniciando..."));
  ESP.restart();
}

void Baratinha::handleCloneRequest() {
  String payload;
  const String mac = getConnectedStationMac();
  if (mac.length()) {
    payload = "{\"mac\":\"" + mac + "\",\"message\":\"MAC copiado do dispositivo conectado.\"}";
  } else {
    payload = "{\"mac\":\"\",\"message\":\"Nenhum dispositivo conectado ao AP.\"}";
  }

  if (_configServer) {
    _configServer->send(200, "application/json", payload);
  }

   String json = "{\"mac\":\"AA:BB:CC:DD:EE:FF\",\"message\":\"MAC clonado\"}";
  _configServer->send(200, "application/json", json);
}

void Baratinha::stopConfigPortal() {
  if (_configServer) {
    _configServer->stop();
  }
  if (_dnsRunning) {
    _dnsServer.stop();
    _dnsRunning = false;
  }
  delete _configServer;
  _configServer = nullptr;
  _configPortalRunning = false;
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

bool Baratinha::parseMacAddress(const String& mac, uint8_t out[6]) {
  if (mac.length() < 17) {
    return false;
  }
  unsigned int bytes[6];
  if (sscanf(mac.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
             &bytes[0], &bytes[1], &bytes[2],
             &bytes[3], &bytes[4], &bytes[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; ++i) {
    out[i] = static_cast<uint8_t>(bytes[i]);
  }
  return true;
}

String Baratinha::getConnectedStationMac() const {
  wifi_sta_list_t wifiStaList;
  memset(&wifiStaList, 0, sizeof(wifi_sta_list_t));
  if (esp_wifi_ap_get_sta_list(&wifiStaList) != ESP_OK || wifiStaList.num == 0) {
    return String();
  }
  return formatMac(wifiStaList.sta[0].mac);
}

String Baratinha::formatMac(const uint8_t* mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buffer);
}

void Baratinha::loadPreferences() {
  if (_prefsReady) {
    _prefs.end(); // Fecha, caso esteja aberto por engano
    _prefsReady = false;
  }
  // Abre NVS
  if (!_prefs.begin("baracfg", false)) { 
      Serial.println(F("[Baratinha] NVS indisponivel."));
      return;
  }
  _prefsReady = true;

  // --- Aplicação do MAC (ANTER de tudo) ---
  String mac = _prefs.getString("staMac", "");
  _staMacToSave = mac; // Apenas carrega o valor salvo na nossa var de classe
  
  if (mac.length() == 17) { 
      Serial.print(F("[Baratinha] NVS: MAC a ser usado: "));
      Serial.println(mac);
  } else {
      Serial.println(F("[Baratinha] NVS: Nenhum MAC clonado salvo. Usando MAC de fabrica."));
  }
    // --- Fim da aplicação do MAC ---

  _preferAccessPoint = _prefs.getBool("preferAP", _preferAccessPoint);
  _staSsid = _prefs.getString("staSsid", _staSsid);
  _staPassword = _prefs.getString("staPass", _staPassword);
  _apSsid = _prefs.getString("apSsid", _apSsid);
  _apPassword = _prefs.getString("apPass", _apPassword);
  _mdnsHostname = _prefs.getString("hostname", _mdnsHostname.length() ? _mdnsHostname : kDefaultHostname);
  _prefs.end(); // Fecha NVS
  _prefsReady = false;
}

void Baratinha::savePreferences() {
    // Abre NVS para escrita
    if (!_prefs.begin("baracfg", false)) { 
        Serial.println(F("[Baratinha] Falha ao ABRIR NVS para salvar."));
        return;
    }
    _prefsReady = true; 

    Serial.println(F("[Baratinha] Salvando preferencias na NVS..."));

    _prefs.putBool("preferAP", _preferAccessPoint);
    _prefs.putString("staSsid", _staSsid);
    _prefs.putString("staPass", _staPassword);
    _prefs.putString("apSsid", _apSsid);
    _prefs.putString("apPass", _apPassword);
    _prefs.putString("hostname", _mdnsHostname);

    // Salva o MAC que foi colocado em _staMacToSave
    _prefs.putString("staMac", _staMacToSave); 
    Serial.print(F("[Baratinha] ...MAC a salvar: "));
    Serial.println(_staMacToSave.length() ? _staMacToSave : "NENHUM");

    // Fecha (commita) NVS
    _prefs.end(); 
    
    Serial.println(F("[Baratinha] Preferencias salvas e commitadas."));
    _prefsReady = false; // Marca como "fechado"
}
