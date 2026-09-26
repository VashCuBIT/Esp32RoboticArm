#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <driver/i2s.h>
#include <Preferences.h>
#include <ESPmDNS.h>

/*
  ==============================================================================
  BRAÇO ROBÓTICO ESP32 - FIRMWARE V8.1 FAST MOTION & ULTRA STABLE WI-FI
  ==============================================================================
  - Arquitetura de 7 Servos Motores com Conversor de Nível Lógico SN74HCT245N
  - Servos MG90S com dinâmica rápida acelerada (fator de velocidade 2.0x)
  - Servos MG996R com controle suave e alto torque (fatores 1.25x - 1.35x)
  - Acumulador fracionário de graus com integração temporal de alta precisão (4ms)
  - Áudio I2S: 2x Microfones INMP441 (Entrada Estéreo) + DAC MAX98357A (Saída)
  - API REST HTTP completa, suporte a JSON bruto e Telemetria em tempo real
  - Persistência não-volátil em Flash NVS (Preferences) de Home, Pinos e Limites
  - Wi-Fi Ultra-Estável: Modem Sleep desativado, controle fino de TX Power (17dBm),
    gerenciamento inteligente anti-conflito de canais e fallback automático de AP.
  ==============================================================================

  Mapeamento Padrão de Pinos (Conforme Descritivo Técnico - Esquema v8):
  ------------------------------------------------------------------------------
  Servos MG90S (Buffer SN74HCT245N - 3.3V -> 5V):
    - D13 -> Garra Abertura  (HCT245 A1-IN > B1-OUT, pinos 2 e 18)
    - D14 -> Garra Rotação   (HCT245 A2-IN > B2-OUT, pinos 3 e 17)
    - D18 -> Ombro Slave     (HCT245 A6-IN > B6-OUT, pinos 7 e 13)
    - D19 -> Punho           (HCT245 A7-IN > B7-OUT, pinos 8 e 12)

  Servos MG996R (Buffer SN74HCT245N - 3.3V -> 5V):
    - D25 -> Base Rotação    (HCT245 A5-IN > B5-OUT, pinos 6 e 14)
    - D26 -> Cotovelo        (HCT245 A4-IN > B4-OUT, pinos 5 e 15)
    - D27 -> Ombro Master    (HCT245 A3-IN > B3-OUT, pinos 4 e 16)

  Microfones INMP441 (I2S Canal 0 - Entrada):
    - WS  -> GPIO 33
    - SCK -> GPIO 32
    - SD  -> GPIO 35 (Entrada direta)

  Amplificador MAX98357A (I2S Canal 1 - Saída):
    - LRC  -> GPIO 21
    - BCLK -> GPIO 22
    - DIN  -> GPIO 23
  ==============================================================================
*/

// ==============================================================================
// 1. CONFIGURAÇÕES DE REDE WI-FI & SERVIDOR WEB
// ==============================================================================

// Modo Station (Conexão a um roteador ou hotspot do celular/notebook)
const char* STA_SSID     = "EsseAKI";
const char* STA_PASSWORD = "trapboost";
const bool  USAR_IP_ESTATICO_STA = false;

// Se true: Desativa o AP próprio quando conectado ao Hotspot para dedicar 100%
// do rádio e antena à conexão principal, eliminando saltos de canal de rádio
// e desconexões periódicas a cada poucos segundos. Caso o Hotspot caia, o AP reativa.
const bool PRIORIZAR_ESTABILIDADE_STA = true;

// Definições de IP Estático (utilizadas se USAR_IP_ESTATICO_STA = true)
IPAddress ESP32_IP_FIXO(192, 168, 1, 150);
IPAddress ESP32_GATEWAY(192, 168, 1, 1);
IPAddress ESP32_SUBNET(255, 255, 255, 0);
IPAddress ESP32_DNS(8, 8, 8, 8);

// Modo Access Point (Rede direta de contingência criada pelo ESP32)
const char* AP_SSID     = "BRACO_ESP32_AP";
const char* AP_PASSWORD = "12345678";
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

// Servidor HTTP REST na porta padrão 80
WebServer server(80);

// ==============================================================================
// 2. PARÃ‚METROS GERAIS DE CINEMÃTICA E DINÃ‚MICA DE MOVIMENTO
// ==============================================================================

// FrequÃªncia padrÃ£o de PWM dos servomotores analÃ³gicos/digitais
const int FREQUENCIA_SERVO_HZ = 50;

// Velocidade base da cinemÃ¡tica em graus por segundo (Â°/s)
// v8.0 anterior: 60 Â°/s  ->  v8.1 Fast Motion: 120 Â°/s
const int VELOCIDADE_PADRAO = 120;

// Velocidade de retorno Ã  posiÃ§Ã£o Home
const int VELOCIDADE_HOME = 100;

// Limite mÃ¡ximo permitido para comandos de velocidade vindos da API
const int VELOCIDADE_COMANDO_MAX = 300;

// Teto mÃ¡ximo para velocidade efetiva computada por software (Â°/s).
// O limite fÃ­sico real dependerÃ¡ da tensÃ£o de alimentaÃ§Ã£o e torque da carga.
const float VELOCIDADE_EFETIVA_MAX = 300.0f;

// Intervalo mÃ­nimo de atualizaÃ§Ã£o da cinemÃ¡tica em milissegundos (4ms = 250 Hz).
// O sinal PWM continua a 50 Hz, mas o cÃ¡lculo de interpolaÃ§Ã£o ocorre com alta taxa.
const unsigned long INTERVALO_CONTROLE_MS = 4;

// Janela mÃ¡xima de tempo para cÃ¡lculo de passo (clamp anti-windup de 40ms).
// Evita que pausas causadas por requisiÃ§Ãµes HTTP ou Ã¡udio I2S faÃ§am o braÃ§o saltar bruscamente.
const unsigned long TEMPO_MAX_CALCULO_MS = 40;

// HabilitaÃ§Ã£o global do sistema mecÃ¢nico (quando falso, desconecta saÃ­das PWM)
bool sistemaHabilitado = true;

// O motor auxiliar do ombro (Ombro Slave) estÃ¡ montado mecanicamente espelhado
const bool OMBRO_INVERTIDO = true;

// Offset angular opcional de calibraÃ§Ã£o fina entre os dois motores do ombro
int offsetOmbroSlave = 0;

// ==============================================================================
// 3. ESTRUTURAS DE DADOS DOS MOTORES E POSIÃ‡ÃƒO HOME
// ==============================================================================

// Identificadores numÃ©ricos dos 7 servomotores
enum MotorId {
  GARRA_ABERTURA = 0,
  GARRA_ROTACAO,
  PUNHO,
  COTOVELO,
  OMBRO_MASTER,
  OMBRO_SLAVE,
  BASE_ROTACAO,
  TOTAL_MOTORES
};

// Estrutura com estado dinÃ¢mico, limites e parÃ¢metros cinemÃ¡ticos de cada motor
struct Motor {
  Servo driver;                  // InstÃ¢ncia da biblioteca ESP32Servo
  const char* nome;              // Nome textual do atuador para APIs e logs
  uint8_t pino;                  // GPIO associado
  int anguloMinimo;              // Limite angular inferior de seguranÃ§a (graus)
  int anguloMaximo;              // Limite angular superior de seguranÃ§a (graus)
  int pulsoMinimoUs;             // Largura mÃ­nima do pulso PWM em microssegundos (us)
  int pulsoMaximoUs;             // Largura mÃ¡xima do pulso PWM em microssegundos (us)
  bool anexado;                  // Indica se o canal PWM estÃ¡ ativo no pino
  int anguloAtual;               // PosiÃ§Ã£o angular instantÃ¢nea inteira
  int anguloAlvo;                // PosiÃ§Ã£o angular de destino comandada
  int velocidade;                // Velocidade base comandada (Â°/s)
  float fatorVelocidade;         // Multiplicador individual de velocidade (dinÃ¢mica rÃ¡pida)
  float acumuladorGraus;         // Integrador fracionÃ¡rio de graus para interpolaÃ§Ã£o contÃ­nua
  unsigned long ultimoPassoMs;   // Marca temporal do Ãºltimo cÃ¡lculo de cinemÃ¡tica
};

// Estrutura para armazenamento dos Ã¢ngulos da posiÃ§Ã£o de repouso (Home)
struct HomeConfig {
  int base_rotacao;
  int ombro;
  int cotovelo;
  int punho;
  int garra_rotacao;
  int garra_abertura;
} posHome = { 90, 90, 90, 90, 90, 90 };

// Tabela de inicializaÃ§Ã£o dos 7 motores com calibraÃ§Ãµes de velocidade e pulsos
Motor motores[TOTAL_MOTORES] = {
  // --- Servos MG90S Independentes (Garra e Punho) ---
  // Operam com alta agilidade e peso leve, utilizando fator 2.0x (atÃ© 240-300 Â°/s efetivos)
  { Servo(), "garra_abertura", 13, 45, 135, 1000, 2000, false, 90, 90, VELOCIDADE_PADRAO, 2.0f,  0.0f, 0 },
  { Servo(), "garra_rotacao",  14,  0, 180, 1000, 2000, false, 90, 90, VELOCIDADE_PADRAO, 2.0f,  0.0f, 0 },
  { Servo(), "punho",          19, 35, 145, 1000, 2000, false, 90, 90, VELOCIDADE_PADRAO, 2.0f,  0.0f, 0 },

  // --- Servo MG996R Cotovelo ---
  // Movimenta massa intermediÃ¡ria com alta precisÃ£o e torque
  { Servo(), "cotovelo",       26, 25, 105,  500, 2500, false, 90, 90, VELOCIDADE_PADRAO, 1.35f, 0.0f, 0 },

  // --- Ombro Master (MG996R) + Slave (MG90S) ---
  // O par conjugado deve obrigatoriamente possuir fatores idÃªnticos para manter sincronia mecÃ¢nica
  { Servo(), "ombro_master",   27, 35, 145,  500, 2500, false, 90, 90, VELOCIDADE_PADRAO, 1.25f, 0.0f, 0 },
  { Servo(), "ombro_slave",    18, 35, 145,  500, 2500, false, 90, 90, VELOCIDADE_PADRAO, 1.25f, 0.0f, 0 },

  // --- Servo MG996R Base GiratÃ³ria ---
  // Suporta a inÃ©rcia de todo o braÃ§o articulado
  { Servo(), "base_rotacao",   25, 15, 165,  500, 2500, false, 90, 90, VELOCIDADE_PADRAO, 1.35f, 0.0f, 0 }
};

// DeclaraÃ§Ã£o prÃ©via da funÃ§Ã£o de sincronizaÃ§Ã£o do ombro
int calcularAnguloOmbroSlave(int anguloMaster);

// ==============================================================================
// 4. PERSISTÃŠNCIA EM MEMÃ“RIA FLASH NÃƒO-VOLÃTIL (NVS / PREFERENCES)
// ==============================================================================

Preferences prefs;

// Carrega posiÃ§Ãµes Home, limites angulares e pinos salvos na Flash
void carregarConfiguracoesNVS() {
  prefs.begin("braco_cfg", false);

  posHome.base_rotacao   = prefs.getInt("home_b",  posHome.base_rotacao);
  posHome.ombro          = prefs.getInt("home_o",  posHome.ombro);
  posHome.cotovelo       = prefs.getInt("home_c",  posHome.cotovelo);
  posHome.punho          = prefs.getInt("home_p",  posHome.punho);
  posHome.garra_rotacao  = prefs.getInt("home_gr", posHome.garra_rotacao);
  posHome.garra_abertura = prefs.getInt("home_ga", posHome.garra_abertura);

  for (int i = 0; i < TOTAL_MOTORES; i++) {
    char kMin[16], kMax[16], kPin[16];
    snprintf(kMin, sizeof(kMin), "lim_min_%d", i);
    snprintf(kMax, sizeof(kMax), "lim_max_%d", i);
    snprintf(kPin, sizeof(kPin), "pin_%d", i);

    motores[i].anguloMinimo = prefs.getInt(kMin, motores[i].anguloMinimo);
    motores[i].anguloMaximo = prefs.getInt(kMax, motores[i].anguloMaximo);

    int p = prefs.getInt(kPin, -1);
    if (p >= 0 && p <= 39) {
      motores[i].pino = (uint8_t)p;
    }
  }

  prefs.end();

  // Inicializa o estado angular sincronizado com a posiÃ§Ã£o Home carregada
  motores[BASE_ROTACAO].anguloAtual   = posHome.base_rotacao;
  motores[BASE_ROTACAO].anguloAlvo    = posHome.base_rotacao;

  motores[OMBRO_MASTER].anguloAtual   = posHome.ombro;
  motores[OMBRO_MASTER].anguloAlvo    = posHome.ombro;

  motores[OMBRO_SLAVE].anguloAtual    = calcularAnguloOmbroSlave(posHome.ombro);
  motores[OMBRO_SLAVE].anguloAlvo     = motores[OMBRO_SLAVE].anguloAtual;

  motores[COTOVELO].anguloAtual       = posHome.cotovelo;
  motores[COTOVELO].anguloAlvo        = posHome.cotovelo;

  motores[PUNHO].anguloAtual          = posHome.punho;
  motores[PUNHO].anguloAlvo           = posHome.punho;

  motores[GARRA_ROTACAO].anguloAtual  = posHome.garra_rotacao;
  motores[GARRA_ROTACAO].anguloAlvo   = posHome.garra_rotacao;

  motores[GARRA_ABERTURA].anguloAtual = posHome.garra_abertura;
  motores[GARRA_ABERTURA].anguloAlvo  = posHome.garra_abertura;

  Serial.printf("[NVS] Home Carregada: B=%d O=%d C=%d P=%d GR=%d GA=%d\n",
    posHome.base_rotacao, posHome.ombro, posHome.cotovelo,
    posHome.punho, posHome.garra_rotacao, posHome.garra_abertura);
}

// Salva as coordenadas da posiÃ§Ã£o Home atual na Flash
void salvarHomeNVS() {
  prefs.begin("braco_cfg", false);
  prefs.putInt("home_b",  posHome.base_rotacao);
  prefs.putInt("home_o",  posHome.ombro);
  prefs.putInt("home_c",  posHome.cotovelo);
  prefs.putInt("home_p",  posHome.punho);
  prefs.putInt("home_gr", posHome.garra_rotacao);
  prefs.putInt("home_ga", posHome.garra_abertura);
  prefs.end();
  Serial.println("[NVS] Coordenadas Home salvas com sucesso.");
}

// Salva os limites angulares de seguranÃ§a na Flash
void salvarLimitesNVS() {
  prefs.begin("braco_cfg", false);
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    char kMin[16], kMax[16];
    snprintf(kMin, sizeof(kMin), "lim_min_%d", i);
    snprintf(kMax, sizeof(kMax), "lim_max_%d", i);
    prefs.putInt(kMin, motores[i].anguloMinimo);
    prefs.putInt(kMax, motores[i].anguloMaximo);
  }
  prefs.end();
  Serial.println("[NVS] Limites angulares salvos com sucesso.");
}

// Salva a tabela de pinos GPIO reconfigurados na Flash
void salvarPinosNVS() {
  prefs.begin("braco_cfg", false);
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    char kPin[16];
    snprintf(kPin, sizeof(kPin), "pin_%d", i);
    prefs.putInt(kPin, motores[i].pino);
  }
  prefs.end();
  Serial.println("[NVS] Mapeamento de pinos salvo com sucesso.");
}

// ==============================================================================
// 5. SUBSISTEMA DE ÃUDIO DIGITAL I2S (INMP441 E MAX98357A)
// ==============================================================================

#define I2S_SAMPLE_RATE 16000

// Canal I2S 0: Microfones EstÃ©reo INMP441 (Entrada RX)
#define I2S_MIC_PORT     I2S_NUM_0
#define PIN_I2S_MIC_WS   33
#define PIN_I2S_MIC_SCK  32
#define PIN_I2S_MIC_SD   35

// Canal I2S 1: Amplificador MAX98357A (SaÃ­da TX)
#define I2S_SPK_PORT     I2S_NUM_1
#define PIN_I2S_SPK_LRC  21
#define PIN_I2S_SPK_BCLK 22
#define PIN_I2S_SPK_DIN  23

bool i2sIniciado = false;

// Inicializa e aloca os drivers de barramento I2S para Ã¡udio bidirecional
void configurarI2S() {
  // ConfiguraÃ§Ã£o do receptor de microfone I2S
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t mic_pins = {
    .bck_io_num = PIN_I2S_MIC_SCK,
    .ws_io_num = PIN_I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_MIC_SD
  };

  esp_err_t errMic = i2s_driver_install(I2S_MIC_PORT, &i2s_mic_config, 0, NULL);
  if (errMic == ESP_OK) {
    i2s_set_pin(I2S_MIC_PORT, &mic_pins);
  }

  // ConfiguraÃ§Ã£o do transmissor para alto-falante I2S
  i2s_config_t i2s_spk_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t spk_pins = {
    .bck_io_num = PIN_I2S_SPK_BCLK,
    .ws_io_num = PIN_I2S_SPK_LRC,
    .data_out_num = PIN_I2S_SPK_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t errSpk = i2s_driver_install(I2S_SPK_PORT, &i2s_spk_config, 0, NULL);
  if (errSpk == ESP_OK) {
    i2s_set_pin(I2S_SPK_PORT, &spk_pins);
  }

  i2sIniciado = (errMic == ESP_OK && errSpk == ESP_OK);
  Serial.printf("[I2S] Drivers inicializados: Mic=%d, Spk=%d\n", errMic, errSpk);
}

// ==============================================================================
// 6. CONTROLE DE BAIXO NÃVEL E GERAÃ‡ÃƒO DE SINAIS PWM
// ==============================================================================

// Garante que o Ã¢ngulo fornecido permaneÃ§a estritamente dentro da faixa segura do motor
int limitarAngulo(int motorId, int angulo) {
  return constrain(angulo, motores[motorId].anguloMinimo, motores[motorId].anguloMaximo);
}

// Mapeia o Ã¢ngulo em graus para o pulso PWM em microssegundos (us)
int converterAnguloParaPulso(int motorId, int angulo) {
  angulo = limitarAngulo(motorId, angulo);
  // O mapeamento do pulso PWM (ex: 500us a 2500us) corresponde Ã  amplitude fÃ­sica total (0 a 180 graus).
  // NÃƒO se deve usar anguloMinimo e anguloMaximo aqui, pois eles representam limites de restriÃ§Ã£o lÃ³gica (software clamping),
  // e usÃ¡-los no mapeamento causaria a re-escala (distorÃ§Ã£o) do movimento fÃ­sico.
  return map(angulo,
             0, 180,
             motores[motorId].pulsoMinimoUs, motores[motorId].pulsoMaximoUs);
}

// Escreve a largura de pulso diretamente no hardware do temporizador do ESP32
void escreverAngulo(int motorId, int angulo) {
  int pulso = converterAnguloParaPulso(motorId, angulo);
  motores[motorId].driver.writeMicroseconds(pulso);
}

// Anexa o sinal PWM no GPIO apenas quando necessÃ¡rio, evitando estresse em repouso
void anexarMotorSeNecessario(int motorId, int primeiroAngulo) {
  if (motores[motorId].anexado) return;

  primeiroAngulo = limitarAngulo(motorId, primeiroAngulo);
  motores[motorId].driver.setPeriodHertz(FREQUENCIA_SERVO_HZ);
  motores[motorId].driver.attach(motores[motorId].pino, 500, 2500);

  motores[motorId].anexado = true;
  motores[motorId].anguloAtual = primeiroAngulo;
  motores[motorId].anguloAlvo = primeiroAngulo;
  motores[motorId].acumuladorGraus = 0.0f;
  motores[motorId].ultimoPassoMs = millis();

  escreverAngulo(motorId, primeiroAngulo);
}

// Define o destino e velocidade de um motor com inicializaÃ§Ã£o suave
void definirAlvoMotor(int motorId, int angulo, int velocidade) {
  angulo = limitarAngulo(motorId, angulo);
  velocidade = constrain(velocidade, 1, VELOCIDADE_COMANDO_MAX);

  // Anexa na posiÃ§Ã£o atual conhecida antes de iniciar a rampa de movimento
  if (!motores[motorId].anexado) {
    anexarMotorSeNecessario(motorId, motores[motorId].anguloAtual);
  }

  motores[motorId].anguloAlvo = angulo;
  motores[motorId].velocidade = velocidade;
}

// ==============================================================================
// 7. CINEMÃTICA ACOPLADA DO OMBRO (MASTER MG996R + SLAVE MG90S)
// ==============================================================================

// Calcula a posiÃ§Ã£o espelhada e compensada para o motor auxiliar do ombro
int calcularAnguloOmbroSlave(int anguloMaster) {
  int anguloSlave = OMBRO_INVERTIDO ? (180 - anguloMaster) : anguloMaster;
  anguloSlave += offsetOmbroSlave;
  return limitarAngulo(OMBRO_SLAVE, anguloSlave);
}

// Coordena o par do ombro assegurando que nenhum motor exceda seu limite mecÃ¢nico
void definirAlvoOmbro(int angulo, int velocidade) {
  int effMin = max(motores[OMBRO_MASTER].anguloMinimo,
                   180 + offsetOmbroSlave - motores[OMBRO_SLAVE].anguloMaximo);
  int effMax = min(motores[OMBRO_MASTER].anguloMaximo,
                   180 + offsetOmbroSlave - motores[OMBRO_SLAVE].anguloMinimo);

  if (effMin > effMax) {
    effMin = motores[OMBRO_MASTER].anguloMinimo;
    effMax = motores[OMBRO_MASTER].anguloMaximo;
  }

  int master = constrain(angulo, effMin, effMax);
  int slave  = calcularAnguloOmbroSlave(master);

  definirAlvoMotor(OMBRO_MASTER, master, velocidade);
  definirAlvoMotor(OMBRO_SLAVE,  slave,  velocidade);
}

// ==============================================================================
// 8. MOTOR DE TRAJETÃ“RIA RÃPIDO COM INTEGRAÃ‡ÃƒO FRACIONÃRIA DE GRAUS
// ==============================================================================

// Executado no loop() principal para computar passos de movimento contÃ­nuo
void atualizarMovimentos() {
  unsigned long agora = millis();

  for (int i = 0; i < TOTAL_MOTORES; i++) {
    // Pula motores desanexados
    if (!motores[i].anexado) {
      continue;
    }

    // Se jÃ¡ atingiu a meta, zera o acumulador e segue
    if (motores[i].anguloAtual == motores[i].anguloAlvo) {
      motores[i].acumuladorGraus = 0.0f;
      continue;
    }

    // Verifica se decorreu a janela mÃ­nima de controle (4ms = 250 Hz)
    unsigned long decorrido = agora - motores[i].ultimoPassoMs;
    if (decorrido < INTERVALO_CONTROLE_MS) {
      continue;
    }

    // Limita delta temporal para evitar trancos apÃ³s bloqueios momentÃ¢neos de rede
    if (decorrido > TEMPO_MAX_CALCULO_MS) {
      decorrido = TEMPO_MAX_CALCULO_MS;
    }

    motores[i].ultimoPassoMs = agora;

    // Calcula velocidade efetiva com o multiplicador individual
    float velocidadeEfetiva = (float)motores[i].velocidade * motores[i].fatorVelocidade;
    if (velocidadeEfetiva < 1.0f) {
      velocidadeEfetiva = 1.0f;
    }
    if (velocidadeEfetiva > VELOCIDADE_EFETIVA_MAX) {
      velocidadeEfetiva = VELOCIDADE_EFETIVA_MAX;
    }

    // IntegraÃ§Ã£o: deslocamento (graus) = velocidade (Â°/s) * tempo (s)
    float deslocamento = velocidadeEfetiva * ((float)decorrido / 1000.0f);
    motores[i].acumuladorGraus += deslocamento;

    // Apenas aplica o passo inteiro quando houver pelo menos 1 grau acumulado
    int passo = (int)motores[i].acumuladorGraus;
    if (passo < 1) {
      continue;
    }

    motores[i].acumuladorGraus -= (float)passo;

    int distancia = abs(motores[i].anguloAlvo - motores[i].anguloAtual);
    if (passo > distancia) {
      passo = distancia;
    }

    // Aplica o passo na direÃ§Ã£o do alvo
    if (motores[i].anguloAtual < motores[i].anguloAlvo) {
      motores[i].anguloAtual += passo;
    } else {
      motores[i].anguloAtual -= passo;
    }

    // Garantia de convergÃªncia exata ao chegar no alvo
    if (abs(motores[i].anguloAlvo - motores[i].anguloAtual) <= 0) {
      motores[i].anguloAtual = motores[i].anguloAlvo;
      motores[i].acumuladorGraus = 0.0f;
    }

    // Atualiza fisicamente o pulso PWM
    escreverAngulo(i, motores[i].anguloAtual);
  }
}

// Interrompe imediatamente qualquer movimento em andamento fixando o alvo no ponto atual
void pararMovimentos() {
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (motores[i].anexado) {
      motores[i].anguloAlvo = motores[i].anguloAtual;
      motores[i].acumuladorGraus = 0.0f;
    }
  }
}

// Desativa os sinais PWM de todos os servos para eliminar consumo e calor
void desanexarTodos() {
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (motores[i].anexado) {
      motores[i].driver.detach();
      motores[i].anexado = false;
    }
    motores[i].acumuladorGraus = 0.0f;
    pinMode(motores[i].pino, OUTPUT);
    digitalWrite(motores[i].pino, LOW);
  }
}

// Move todos os atuadores suavemente para as posiÃ§Ãµes Home cadastradas
void anexarTodosNaHome() {
  definirAlvoMotor(BASE_ROTACAO,   posHome.base_rotacao,   VELOCIDADE_HOME);
  definirAlvoOmbro(posHome.ombro,                          VELOCIDADE_HOME);
  definirAlvoMotor(COTOVELO,       posHome.cotovelo,       VELOCIDADE_HOME);
  definirAlvoMotor(PUNHO,          posHome.punho,          VELOCIDADE_HOME);
  definirAlvoMotor(GARRA_ROTACAO,  posHome.garra_rotacao,  VELOCIDADE_HOME);
  definirAlvoMotor(GARRA_ABERTURA, posHome.garra_abertura, VELOCIDADE_HOME);
}

// Reassocia dinamicamente um servo a um novo pino GPIO sem reiniciar o firmware
void remapearPino(int motorId, uint8_t novoPino) {
  if (motorId < 0 || motorId >= TOTAL_MOTORES) return;
  if (motores[motorId].pino == novoPino) return;

  if (motores[motorId].anexado) {
    motores[motorId].driver.detach();
    motores[motorId].anexado = false;
  }

  motores[motorId].pino = novoPino;
  pinMode(novoPino, OUTPUT);
  digitalWrite(novoPino, LOW);

  anexarMotorSeNecessario(motorId, motores[motorId].anguloAtual);
}

// ==============================================================================
// 9. FORMATAÃ‡ÃƒO DE CABEÃ‡ALHOS CORS E RESPOSTAS JSON
// ==============================================================================

void adicionarCors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Content-Length");
}

void enviarJson(int statusHttp, const String& json) {
  adicionarCors();
  server.send(statusHttp, "application/json", json);
}

// ==============================================================================
// 10. ROTAS DA API HTTP REST & TELEMETRIA
// ==============================================================================

// Telemetria ultra-rÃ¡pida (30-50 Hz) otimizada para sincronizaÃ§Ã£o com render 3D
void handleTelemetry() {
  adicionarCors();

  bool emMovimento = false;
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (motores[i].anexado && motores[i].anguloAtual != motores[i].anguloAlvo) {
      emMovimento = true;
      break;
    }
  }

  char buf[160];
  snprintf(buf, sizeof(buf),
    "{\"sucesso\":true,\"b\":%d,\"o\":%d,\"c\":%d,\"p\":%d,\"gr\":%d,\"ga\":%d,\"m\":%d}",
    motores[BASE_ROTACAO].anexado   ? motores[BASE_ROTACAO].anguloAtual   : 90,
    motores[OMBRO_MASTER].anexado   ? motores[OMBRO_MASTER].anguloAtual   : 90,
    motores[COTOVELO].anexado       ? motores[COTOVELO].anguloAtual       : 90,
    motores[PUNHO].anexado          ? motores[PUNHO].anguloAtual          : 90,
    motores[GARRA_ROTACAO].anexado  ? motores[GARRA_ROTACAO].anguloAtual  : 90,
    motores[GARRA_ABERTURA].anexado ? motores[GARRA_ABERTURA].anguloAtual : 90,
    emMovimento ? 1 : 0
  );

  server.send(200, "application/json", buf);
}

// Retorna o status completo do firmware, rede, Ã¡udio e atuadores
void handleStatus() {
  String json = "{";
  json += "\"sucesso\":true,";
  json += "\"sistema\":\"braco_robotico_v8_1_fast\",";
  json += "\"habilitado\":" + String(sistemaHabilitado ? "true" : "false") + ",";
  json += "\"i2s_ativo\":" + String(i2sIniciado ? "true" : "false") + ",";
  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"ip_sta\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "desconectado") + "\",";
  json += "\"ip_ativo\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\",";
  json += "\"velocidade_padrao\":" + String(VELOCIDADE_PADRAO) + ",";
  json += "\"motores\":[";

  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (i > 0) json += ",";
    json += "{\"nome\":\"" + String(motores[i].nome) + "\",";
    json += "\"pino\":" + String(motores[i].pino) + ",";
    json += "\"anexado\":" + String(motores[i].anexado ? "true" : "false") + ",";
    json += "\"atual\":" + String(motores[i].anguloAtual) + ",";
    json += "\"alvo\":" + String(motores[i].anguloAlvo) + ",";
    json += "\"velocidade\":" + String(motores[i].velocidade) + ",";
    json += "\"fator\":" + String(motores[i].fatorVelocidade, 2) + "}";
  }

  json += "]}";
  enviarJson(200, json);
}

// Rota de movimentaÃ§Ã£o dos eixos (suporta query params, form-data ou JSON no body)
void handleMove() {
  if (!sistemaHabilitado) {
    enviarJson(409, "{\"sucesso\":false,\"erro\":\"SYSTEM_DISABLED\"}");
    return;
  }

  int velocidade = VELOCIDADE_PADRAO;
  if (server.hasArg("speed")) {
    velocidade = server.arg("speed").toInt();
  }
  velocidade = constrain(velocidade, 1, VELOCIDADE_COMANDO_MAX);

  // ParÃ¢metros via Query String ou Form-Data
  if (server.hasArg("base_rotacao"))   definirAlvoMotor(BASE_ROTACAO,   server.arg("base_rotacao").toInt(),   velocidade);
  if (server.hasArg("ombro"))          definirAlvoOmbro(server.arg("ombro").toInt(), velocidade);
  if (server.hasArg("cotovelo"))       definirAlvoMotor(COTOVELO,       server.arg("cotovelo").toInt(),       velocidade);
  if (server.hasArg("punho"))          definirAlvoMotor(PUNHO,          server.arg("punho").toInt(),          velocidade);
  if (server.hasArg("garra_rotacao"))  definirAlvoMotor(GARRA_ROTACAO,  server.arg("garra_rotacao").toInt(),  velocidade);
  if (server.hasArg("garra_abertura")) definirAlvoMotor(GARRA_ABERTURA, server.arg("garra_abertura").toInt(), velocidade);

  // Suporte a Payload JSON Bruto
  if (server.hasArg("plain")) {
    String corpo = server.arg("plain");
    auto extrair = [&](const char* chave) -> int {
      int idx = corpo.indexOf(chave);
      if (idx < 0) return -999;
      int col = corpo.indexOf(":", idx);
      if (col < 0) return -999;
      int fim = corpo.indexOf(",", col);
      if (fim < 0) fim = corpo.indexOf("}", col);
      if (fim < 0) fim = corpo.length();
      String val = corpo.substring(col + 1, fim);
      val.trim();
      return val.toInt();
    };

    int sp = extrair("\"speed\"");
    if (sp > 0) {
      velocidade = constrain(sp, 1, VELOCIDADE_COMANDO_MAX);
    }

    int v = extrair("\"base_rotacao\"");   if (v != -999) definirAlvoMotor(BASE_ROTACAO, v, velocidade);
    v = extrair("\"ombro\"");              if (v != -999) definirAlvoOmbro(v, velocidade);
    v = extrair("\"cotovelo\"");           if (v != -999) definirAlvoMotor(COTOVELO, v, velocidade);
    v = extrair("\"punho\"");              if (v != -999) definirAlvoMotor(PUNHO, v, velocidade);
    v = extrair("\"garra_rotacao\"");      if (v != -999) definirAlvoMotor(GARRA_ROTACAO, v, velocidade);
    v = extrair("\"garra_abertura\"");     if (v != -999) definirAlvoMotor(GARRA_ABERTURA, v, velocidade);
  }

  enviarJson(200, "{\"sucesso\":true,\"mensagem\":\"Movimento atualizado\"}");
}

// Dispara o movimento de todos os servos para a posiÃ§Ã£o Home
void handleHome() {
  if (!sistemaHabilitado) {
    enviarJson(409, "{\"sucesso\":false,\"erro\":\"SYSTEM_DISABLED\"}");
    return;
  }
  anexarTodosNaHome();
  enviarJson(200, "{\"sucesso\":true,\"mensagem\":\"Movendo para Home\"}");
}

// Consulta ou redefine os Ã¢ngulos da posiÃ§Ã£o Home
void handleHomeConfig() {
  if (server.method() == HTTP_POST) {
    if (server.hasArg("base_rotacao"))   posHome.base_rotacao   = server.arg("base_rotacao").toInt();
    if (server.hasArg("ombro"))          posHome.ombro          = server.arg("ombro").toInt();
    if (server.hasArg("cotovelo"))       posHome.cotovelo       = server.arg("cotovelo").toInt();
    if (server.hasArg("punho"))          posHome.punho          = server.arg("punho").toInt();
    if (server.hasArg("garra_rotacao"))  posHome.garra_rotacao  = server.arg("garra_rotacao").toInt();
    if (server.hasArg("garra_abertura")) posHome.garra_abertura = server.arg("garra_abertura").toInt();

    if (server.hasArg("plain")) {
      String corpo = server.arg("plain");
      auto extrair = [&](const char* chave) -> int {
        int idx = corpo.indexOf(chave);
        if (idx < 0) return -999;
        int col = corpo.indexOf(":", idx);
        if (col < 0) return -999;
        int fim = corpo.indexOf(",", col);
        if (fim < 0) fim = corpo.indexOf("}", col);
        if (fim < 0) fim = corpo.length();
        String val = corpo.substring(col + 1, fim);
        val.trim();
        return val.toInt();
      };

      int v = extrair("\"base_rotacao\"");   if (v != -999) posHome.base_rotacao = v;
      v = extrair("\"ombro\"");              if (v != -999) posHome.ombro = v;
      v = extrair("\"cotovelo\"");           if (v != -999) posHome.cotovelo = v;
      v = extrair("\"punho\"");              if (v != -999) posHome.punho = v;
      v = extrair("\"garra_rotacao\"");      if (v != -999) posHome.garra_rotacao = v;
      v = extrair("\"garra_abertura\"");     if (v != -999) posHome.garra_abertura = v;
    }

    salvarHomeNVS();
  }

  String json = "{";
  json += "\"base_rotacao\":"   + String(posHome.base_rotacao)   + ",";
  json += "\"ombro\":"          + String(posHome.ombro)          + ",";
  json += "\"cotovelo\":"       + String(posHome.cotovelo)       + ",";
  json += "\"punho\":"          + String(posHome.punho)          + ",";
  json += "\"garra_rotacao\":"  + String(posHome.garra_rotacao)  + ",";
  json += "\"garra_abertura\":" + String(posHome.garra_abertura);
  json += "}";

  enviarJson(200, json);
}

// Consulta ou reconfigura o mapeamento de pinos GPIO dos servos
void handlePins() {
  if (server.method() == HTTP_POST) {
    if (server.hasArg("garra_abertura")) remapearPino(GARRA_ABERTURA, server.arg("garra_abertura").toInt());
    if (server.hasArg("garra_rotacao"))  remapearPino(GARRA_ROTACAO,  server.arg("garra_rotacao").toInt());
    if (server.hasArg("ombro_slave"))    remapearPino(OMBRO_SLAVE,    server.arg("ombro_slave").toInt());
    if (server.hasArg("punho"))          remapearPino(PUNHO,          server.arg("punho").toInt());
    if (server.hasArg("base_rotacao"))   remapearPino(BASE_ROTACAO,   server.arg("base_rotacao").toInt());
    if (server.hasArg("cotovelo"))       remapearPino(COTOVELO,       server.arg("cotovelo").toInt());
    if (server.hasArg("ombro_master"))   remapearPino(OMBRO_MASTER,   server.arg("ombro_master").toInt());

    if (server.hasArg("plain")) {
      String corpo = server.arg("plain");
      auto extrair = [&](const char* chave) -> int {
        int idx = corpo.indexOf(chave);
        if (idx < 0) return -999;
        int col = corpo.indexOf(":", idx);
        if (col < 0) return -999;
        int fim = corpo.indexOf(",", col);
        if (fim < 0) fim = corpo.indexOf("}", col);
        if (fim < 0) fim = corpo.length();
        String val = corpo.substring(col + 1, fim);
        val.trim();
        return val.toInt();
      };

      int v = extrair("\"garra_abertura\""); if (v != -999) remapearPino(GARRA_ABERTURA, v);
      v = extrair("\"garra_rotacao\"");      if (v != -999) remapearPino(GARRA_ROTACAO, v);
      v = extrair("\"ombro_slave\"");        if (v != -999) remapearPino(OMBRO_SLAVE, v);
      v = extrair("\"punho\"");              if (v != -999) remapearPino(PUNHO, v);
      v = extrair("\"base_rotacao\"");       if (v != -999) remapearPino(BASE_ROTACAO, v);
      v = extrair("\"cotovelo\"");           if (v != -999) remapearPino(COTOVELO, v);
      v = extrair("\"ombro_master\"");       if (v != -999) remapearPino(OMBRO_MASTER, v);
    }

    salvarPinosNVS();
  }

  String json = "{";
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (i > 0) json += ",";
    json += "\"" + String(motores[i].nome) + "\":" + String(motores[i].pino);
  }
  json += "}";

  enviarJson(200, json);
}

// Consulta ou redefine os limites angulares mÃ­nimo e mÃ¡ximo de cada atuador
void handleLimits() {
  if (server.method() == HTTP_POST) {
    auto atualizarLimite = [&](int motorId, const char* minKey, const char* maxKey) {
      if (server.hasArg(minKey)) {
        motores[motorId].anguloMinimo = constrain(server.arg(minKey).toInt(), 0, 180);
      }
      if (server.hasArg(maxKey)) {
        motores[motorId].anguloMaximo = constrain(server.arg(maxKey).toInt(), 0, 180);
      }
      if (motores[motorId].anguloMinimo > motores[motorId].anguloMaximo) {
        int temp = motores[motorId].anguloMinimo;
        motores[motorId].anguloMinimo = motores[motorId].anguloMaximo;
        motores[motorId].anguloMaximo = temp;
      }
    };

    atualizarLimite(GARRA_ABERTURA, "garra_abertura_min", "garra_abertura_max");
    atualizarLimite(GARRA_ROTACAO,  "garra_rotacao_min",  "garra_rotacao_max");
    atualizarLimite(PUNHO,          "punho_min",          "punho_max");
    atualizarLimite(COTOVELO,       "cotovelo_min",       "cotovelo_max");
    atualizarLimite(OMBRO_MASTER,   "ombro_master_min",   "ombro_master_max");
    atualizarLimite(OMBRO_SLAVE,    "ombro_slave_min",    "ombro_slave_max");
    atualizarLimite(BASE_ROTACAO,   "base_rotacao_min",   "base_rotacao_max");

    if (server.hasArg("plain")) {
      String corpo = server.arg("plain");
      auto extrair = [&](const char* chave) -> int {
        int idx = corpo.indexOf(chave);
        if (idx < 0) return -999;
        int col = corpo.indexOf(":", idx);
        if (col < 0) return -999;
        int fim = corpo.indexOf(",", col);
        if (fim < 0) fim = corpo.indexOf("}", col);
        if (fim < 0) fim = corpo.length();
        String val = corpo.substring(col + 1, fim);
        val.trim();
        return val.toInt();
      };

      for (int i = 0; i < TOTAL_MOTORES; i++) {
        String kMin = String("\"") + motores[i].nome + "_min\"";
        String kMax = String("\"") + motores[i].nome + "_max\"";
        int vMin = extrair(kMin.c_str());
        int vMax = extrair(kMax.c_str());

        if (vMin != -999) motores[i].anguloMinimo = constrain(vMin, 0, 180);
        if (vMax != -999) motores[i].anguloMaximo = constrain(vMax, 0, 180);

        if (motores[i].anguloMinimo > motores[i].anguloMaximo) {
          int t = motores[i].anguloMinimo;
          motores[i].anguloMinimo = motores[i].anguloMaximo;
          motores[i].anguloMaximo = t;
        }
      }
    }

    salvarLimitesNVS();
  }

  String json = "{\"sucesso\":true,\"limits\":{";
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (i > 0) json += ",";
    json += "\"" + String(motores[i].nome) + "\":{";
    json += "\"min\":" + String(motores[i].anguloMinimo) + ",";
    json += "\"max\":" + String(motores[i].anguloMaximo) + "}";
  }
  json += "}}";

  enviarJson(200, json);
}

// Callback de recepÃ§Ã£o de streaming de Ã¡udio para o alto-falante MAX98357A via I2S
void handleAudioUpload() {
  if (!i2sIniciado) return;

  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_WRITE && upload.currentSize > 0) {
    size_t bytesEscritos = 0;
    esp_err_t resultado = i2s_write(
      I2S_SPK_PORT,
      upload.buf,
      upload.currentSize,
      &bytesEscritos,
      portMAX_DELAY
    );

    if (resultado != ESP_OK || bytesEscritos != upload.currentSize) {
      Serial.printf("[Audio TX] Falha I2S. erro=%d recebido=%u escrito=%u\n",
        resultado, (unsigned int)upload.currentSize, (unsigned int)bytesEscritos);
    }
  }
}

// FinalizaÃ§Ã£o da requisiÃ§Ã£o de streaming de Ã¡udio
void handleAudioStream() {
  if (!i2sIniciado) {
    enviarJson(500, "{\"sucesso\":false,\"erro\":\"I2S_NOT_READY\"}");
    return;
  }
  enviarJson(200, "{\"sucesso\":true,\"mensagem\":\"Audio recebido\"}");
}

// Captura pacote PCM bruto de 16 kHz dos microfones INMP441 e envia via HTTP
void handleMicCapture() {
  if (!i2sIniciado) {
    enviarJson(500, "{\"sucesso\":false,\"erro\":\"I2S_NOT_READY\"}");
    return;
  }

  const size_t BUFFER_SIZE = 16000; // ~0.25 segundos de Ã¡udio estÃ©reo 16-bit
  uint8_t* audioBuf = (uint8_t*)malloc(BUFFER_SIZE);
  if (!audioBuf) {
    enviarJson(500, "{\"sucesso\":false,\"erro\":\"OUT_OF_MEMORY\"}");
    return;
  }

  size_t bytesLidos = 0;
  esp_err_t resultado = i2s_read(
    I2S_MIC_PORT,
    audioBuf,
    BUFFER_SIZE,
    &bytesLidos,
    pdMS_TO_TICKS(600)
  );

  if (resultado != ESP_OK || bytesLidos == 0) {
    Serial.printf("[Audio RX] Erro ao ler microfones. erro=%d bytes=%u\n",
      resultado, (unsigned int)bytesLidos);
    free(audioBuf);
    enviarJson(500, "{\"sucesso\":false,\"erro\":\"MIC_READ_FAILED\"}");
    return;
  }

  adicionarCors();
  server.sendHeader("Content-Disposition", "attachment; filename=\"mic.raw\"");
  server.setContentLength(bytesLidos);
  server.send(200, "application/octet-stream", "");
  server.sendContent(reinterpret_cast<const char*>(audioBuf), bytesLidos);

  free(audioBuf);
}

// Manipulador de preflight OPTIONS para compatibilidade CORS com navegadores
void handleOptions() {
  adicionarCors();
  server.send(204);
}

// Registra todas as rotas da API REST no servidor WebServer
void configurarRotas() {
  // Telemetria e DiagnÃ³stico
  server.on("/telemetry",   HTTP_GET, handleTelemetry);
  server.on("/status",      HTTP_GET, handleStatus);

  // Controle de Movimento
  server.on("/move",        HTTP_GET,  handleMove);
  server.on("/move",        HTTP_POST, handleMove);
  server.on("/home",        HTTP_GET,  handleHome);
  server.on("/home",        HTTP_POST, handleHome);
  server.on("/home/config", HTTP_GET,  handleHomeConfig);
  server.on("/home/config", HTTP_POST, handleHomeConfig);

  // ConfiguraÃ§Ã£o de Pinos e Limites Angulares
  server.on("/pins",        HTTP_GET,  handlePins);
  server.on("/pins",        HTTP_POST, handlePins);
  server.on("/limits",      HTTP_GET,  handleLimits);
  server.on("/limits",      HTTP_POST, handleLimits);

  // Gerenciamento de AlimentaÃ§Ã£o e EmergÃªncia
  server.on("/habilitar",   HTTP_GET, []() {
    sistemaHabilitado = true;
    enviarJson(200, "{\"sucesso\":true}");
  });

  server.on("/desabilitar", HTTP_GET, []() {
    sistemaHabilitado = false;
    desanexarTodos();
    enviarJson(200, "{\"sucesso\":true}");
  });

  server.on("/stop", HTTP_GET, []() {
    pararMovimentos();
    enviarJson(200, "{\"sucesso\":true}");
  });

  // Ãudio I2S (Voz & Microfone)
  server.on("/audio/stream", HTTP_POST, handleAudioStream, handleAudioUpload);
  server.on("/mic/capture",  HTTP_GET,  handleMicCapture);

  // Rota de Erro 404 e Tratamento OPTIONS
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      handleOptions();
    } else {
      enviarJson(404, "{\"sucesso\":false,\"erro\":\"NOT_FOUND\"}");
    }
  });
}

// ==============================================================================
// 11. INICIALIZACAO DO SISTEMA (SETUP)
// ==============================================================================

unsigned long ultimoStatusSerialMs = 0;
unsigned long ultimoMomentoConectadoMs = 0;
unsigned long ultimaTentativaReconexaoMs = 0;
bool apAtivo = false;
wl_status_t ultimoStatusWifi = WL_IDLE_STATUS;

const char* obterNomeMotivoDesconexao(uint8_t motivo) {
  switch (motivo) {
    case 1:   return "UNSPECIFIED (Nao especificado)";
    case 2:   return "AUTH_EXPIRE (Autenticacao expirou)";
    case 3:   return "AUTH_LEAVE (Desconectado/Deauthed)";
    case 4:   return "ASSOC_EXPIRE (Associacao expirou)";
    case 8:   return "ASSOC_LEAVE (Desconectado pelo roteador/celular)";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT (Timeout de chave WPA2 - desative WPA3/PMF no celular)";
    case 200: return "BEACON_TIMEOUT (Perda de sinal / Beacons perdidos por Modem Sleep)";
    case 201: return "NO_AP_FOUND (Rede nao encontrada - verifique se Hotspot esta em 2.4 GHz)";
    case 202: return "AUTH_FAIL (Senha Wi-Fi incorreta)";
    case 203: return "ASSOC_FAIL (Falha de associacao com a rede)";
    case 204: return "HANDSHAKE_TIMEOUT (Timeout de aperto de mao WPA)";
    default:  return "OUTRO_MOTIVO";
  }
}

void imprimirStatusRede() {
  Serial.println("\n------------------------------------------------------------");
  Serial.println("  >>> DIAGNOSTICO DE REDE - BRACO ROBOTICO ESP32 <<<");
  Serial.println("------------------------------------------------------------");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("  [Wi-Fi STA] STATUS: CONECTADO COM SUCESSO!");
    Serial.printf ("  [Wi-Fi STA] Rede / Hotspot: %s\n", STA_SSID);
    Serial.printf ("  [Wi-Fi STA] IP ATRIBUIDO:   %s  <--- USE ESTE IP NA INTERFACE\n", WiFi.localIP().toString().c_str());
    Serial.printf ("  [Wi-Fi STA] Gateway:        %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf ("  [Wi-Fi STA] Sinal (RSSI):   %d dBm\n", WiFi.RSSI());
    Serial.printf ("  [Wi-Fi STA] Canal de Radio: %d\n", WiFi.channel());
  } else {
    Serial.println("  [Wi-Fi STA] STATUS: NAO CONECTADO AO HOTSPOT / ROTEADOR");
    Serial.printf ("  [Wi-Fi STA] SSID Configurado: %s\n", STA_SSID);
    Serial.printf ("  [Wi-Fi STA] Codigo de Status: %d\n", (int)WiFi.status());
    Serial.println("  [DICA 1]: O Hotspot do seu celular DEVE estar na banda 2.4 GHz!");
    Serial.println("  [DICA 2]: No iPhone, ative 'Maximizar Compatibilidade'.");
    Serial.println("  [DICA 3]: No Android, configure a banda do Ponto de Acesso para 2.4 GHz.");
  }
  Serial.println("  ----------------------------------------------------------");
  if (apAtivo) {
    Serial.println("  [Wi-Fi AP]  Ponto de Acesso de Contingencia (ATIVO):");
    Serial.printf ("  [Wi-Fi AP]  Nome da Rede (SSID): %s\n", AP_SSID);
    Serial.printf ("  [Wi-Fi AP]  Senha:               %s\n", AP_PASSWORD);
    Serial.printf ("  [Wi-Fi AP]  IP do AP:            %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("  [Wi-Fi AP]  Ponto de Acesso: EM ESPERA (Desativado para estabilidade do STA)");
    Serial.println("              (Sera ativado automaticamente se o Hotspot cair por > 8s)");
  }
  Serial.println("  ----------------------------------------------------------");
  Serial.println("  [mDNS]      Nome de Host: http://braco-esp32.local");
  Serial.println("  (Dica: Pressione ENTER no Serial Monitor para ver este status a qualquer momento)");
  Serial.println("------------------------------------------------------------\n");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n============================================================");
  Serial.println("  BRACO ROBOTICO ESP32 - FIRMWARE V8.1 FAST MOTION & STABLE ");
  Serial.println("============================================================");

  // 1. Carrega parametros salvos na NVS Flash
  carregarConfiguracoesNVS();

  // 2. Inicializa barramento de audio I2S
  configurarI2S();

  // 3. Inicializacao Wi-Fi Ultra-Estavel (Sem Modem Sleep e sem Brownout)
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);                 // Desativa Modem Sleep (elimina quedas a cada 10-30s)
  esp_wifi_set_ps(WIFI_PS_NONE);        // Forca 100% de atividade no nivel do driver ESP-IDF
  WiFi.setTxPower(WIFI_POWER_17dBm);    // 17dBm estabiliza a tensao e evita picos de corrente

  #ifdef ARDUINO_EVENT_WIFI_STA_DISCONNECTED
  WiFi.onEvent([](WiFiEvent_t evento, WiFiEventInfo_t info) {
    if (evento == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      uint8_t motivo = info.wifi_sta_disconnected.reason;
      Serial.printf("\n[Wi-Fi Evento] Queda detectada! Codigo: %d -> %s\n", motivo, obterNomeMotivoDesconexao(motivo));
    } else if (evento == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("\n[Wi-Fi Evento] >>> CONEXAO ESTABELECIDA! IP ATRIBUIDO: %s <<<\n", WiFi.localIP().toString().c_str());
    }
  });
  #endif

  // Inicia conexao Station (Hotspot / Roteador)
  bool staConectou = false;
  if (strlen(STA_SSID) > 0 && strcmp(STA_SSID, "SUA_REDE_WIFI") != 0) {
    if (USAR_IP_ESTATICO_STA) {
      WiFi.config(ESP32_IP_FIXO, ESP32_GATEWAY, ESP32_SUBNET, ESP32_DNS);
    }
    WiFi.setAutoReconnect(true);
    WiFi.begin(STA_SSID, STA_PASSWORD);
    Serial.printf("[Wi-Fi STA] Conectando ao Hotspot '%s'...\n", STA_SSID);

    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 25) {
      delay(400);
      Serial.print(".");
      tentativas++;
    }
    Serial.println();
    staConectou = (WiFi.status() == WL_CONNECTED);
  }

  // Se conectou ao Hotspot com sucesso e priorizamos estabilidade,
  // desligamos os beacons do AP local para evitar salto constante de canais.
  if (staConectou) {
    ultimoMomentoConectadoMs = millis();
    if (PRIORIZAR_ESTABILIDADE_STA) {
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      apAtivo = false;
      Serial.println("[Wi-Fi STA] Modo Puro Estacao ativo: 100% do radio dedicado ao Hotspot (Estabilidade maxima).");
    } else {
      WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
      WiFi.softAP(AP_SSID, AP_PASSWORD, WiFi.channel());
      apAtivo = true;
    }
  } else {
    // Se nao conectou, sobe o AP de contingencia imediatamente
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    apAtivo = true;
    Serial.println("[Wi-Fi AP] Hotspot nao encontrado de imediato. AP de contingencia ativado (192.168.4.1).");
  }

  // 4. Imprime o relatorio completo de status e IPs
  imprimirStatusRede();

  // 5. Inicia o respondedor mDNS
  if (MDNS.begin("braco-esp32")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[mDNS] Disponivel em: http://braco-esp32.local");
  }

  // 6. Configura e inicializa o servidor HTTP REST
  configurarRotas();
  server.begin();
  Serial.println("[HTTP] Servidor REST ativo na porta 80.");

  // 7. Informacoes de Dinamica de Movimento
  Serial.println("[MOVIMENTO] Controle rapido habilitado (v8.1 Fast Motion).");
  Serial.printf("[MOVIMENTO] Velocidade padrao: %d graus/s\n", VELOCIDADE_PADRAO);
}

// ==============================================================================
// 12. LOOP PRINCIPAL (LOOP)
// ==============================================================================

void loop() {
  server.handleClient();
  atualizarMovimentos();

  unsigned long agora = millis();

  // 1. Gerenciamento inteligente e resiliente da conexao Wi-Fi
  if (WiFi.status() == WL_CONNECTED) {
    ultimoMomentoConectadoMs = agora;

    // Se o AP de contingencia ainda estiver ativo e priorizamos estabilidade, desliga para eliminar conflito de canais
    if (apAtivo && PRIORIZAR_ESTABILIDADE_STA) {
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      apAtivo = false;
      Serial.println("[Wi-Fi AP] AP de contingencia em espera (100% da banda e radio dedicados ao Hotspot).");
    }
  } else {
    // Se ficar desconectado por mais de 8 segundos, sobe o AP de contingencia para o usuario nao ficar sem acesso
    if (!apAtivo && (agora - ultimoMomentoConectadoMs > 8000)) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
      WiFi.softAP(AP_SSID, AP_PASSWORD);
      apAtivo = true;
      Serial.println("[Wi-Fi AP] Hotspot desconectado: Ativando AP de Contingencia 'BRACO_ESP32_AP' (192.168.4.1)...");
    }

    // Tenta reconectar a cada 5 segundos de forma nao-bloqueante
    if (agora - ultimaTentativaReconexaoMs > 5000) {
      ultimaTentativaReconexaoMs = agora;
      Serial.printf("[Wi-Fi STA] Tentando reconectar ao Hotspot '%s'...\n", STA_SSID);
      WiFi.reconnect();
    }
  }

  // 2. Notificacao de alteracao de status na Serial
  wl_status_t statusAtual = WiFi.status();
  if (statusAtual != ultimoStatusWifi) {
    ultimoStatusWifi = statusAtual;
    if (statusAtual == WL_CONNECTED) {
      Serial.printf("\n[Wi-Fi] >>> CONECTADO COM SUCESSO! IP ATRIBUIDO: %s <<<\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.printf("\n[Wi-Fi] Desconectado da rede (Status: %d). Mantendo tentativas em background...\n", (int)statusAtual);
    }
  }

  // 3. Se o usuario digitar algo ou pressionar ENTER no Serial Monitor, reimprime o status completo!
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    imprimirStatusRede();
  }

  // 4. Heartbeat periodico a cada 10 segundos na Serial se desconectado
  if (WiFi.status() != WL_CONNECTED && (agora - ultimoStatusSerialMs > 10000)) {
    ultimoStatusSerialMs = agora;
    Serial.printf("[Wi-Fi STA] Aguardando conexao em '%s'... (AP de Contingencia: %s)\n", STA_SSID, apAtivo ? WiFi.softAPIP().toString().c_str() : "Inativo");
  }
}

