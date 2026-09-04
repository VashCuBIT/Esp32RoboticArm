#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

/*
  ============================================================
  API DE TESTES DO BRACO ROBOTICO
  ESP32 + COMANDOS DIRETOS PELO NAVEGADOR
  ============================================================

  Esta versao foi criada para testes e demonstracoes.

  Comunicacao:
  Navegador -> requisicao HTTP GET -> ESP32 -> PWM -> HCT245 -> servos

  Exemplos:
  http://192.168.4.1/status
  http://192.168.4.1/habilitar
  http://192.168.4.1/servo?motor=punho&angle=90
  http://192.168.4.1/desabilitar

  IMPORTANTE:
  - Nenhum servo e inicializado automaticamente no setup.
  - Nenhum movimento e executado ao ligar.
  - O sistema precisa ser habilitado antes dos movimentos.
*/

// ============================================================
// CONFIGURACAO DO WI-FI
// ============================================================

const char* AP_SSID = "BRACO_TESTE";
const char* AP_PASSWORD = "12345678";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

WebServer server(80);

// ============================================================
// CONFIGURACAO GERAL
// ============================================================

const int FREQUENCIA_SERVO_HZ = 50;
const int VELOCIDADE_PADRAO = 50;

bool sistemaHabilitado = false;

// Os dois servos do ombro ficam de frente um para o outro.
const bool OMBRO_INVERTIDO = true;

// Ajuste fino para alinhar mecanicamente o servo slave.
const int OFFSET_OMBRO_SLAVE = 0;

// ============================================================
// PINOS DOS SERVOS
// ============================================================

const int PINO_GARRA_ABERTURA = 13;
const int PINO_GARRA_ROTACAO  = 14;
const int PINO_PUNHO          = 19;

const int PINO_COTOVELO       = 26;
const int PINO_OMBRO_MASTER   = 27;
const int PINO_OMBRO_SLAVE    = 18;
const int PINO_BASE_ROTACAO   = 25;

// ============================================================
// OBJETOS DOS SERVOS
// ============================================================

Servo servoGarraAbertura;
Servo servoGarraRotacao;
Servo servoPunho;
Servo servoCotovelo;
Servo servoOmbroMaster;
Servo servoOmbroSlave;
Servo servoBaseRotacao;

// ============================================================
// IDENTIFICADORES
// ============================================================

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

// ============================================================
// ESTRUTURA DOS MOTORES
// ============================================================

struct Motor {
  Servo* driver;

  const char* nome;
  uint8_t pino;

  int anguloMinimo;
  int anguloMaximo;

  int pulsoMinimoUs;
  int pulsoMaximoUs;

  bool anexado;

  int anguloAtual;
  int anguloAlvo;
  int velocidade;

  unsigned long ultimoPassoMs;
};

// Os MG90S usam inicialmente uma faixa conservadora de pulso.
// Os valores podem ser calibrados posteriormente.

Motor motores[TOTAL_MOTORES] = {
  {
    &servoGarraAbertura,
    "garra_abertura",
    PINO_GARRA_ABERTURA,
    45,
    135,
    1000,
    2000,
    false,
    90,
    90,
    VELOCIDADE_PADRAO,
    0
  },
  {
    &servoGarraRotacao,
    "garra_rotacao",
    PINO_GARRA_ROTACAO,
    0,
    180,
    1000,
    2000,
    false,
    90,
    90,
    VELOCIDADE_PADRAO,
    0
  },
  {
    &servoPunho,
    "punho",
    PINO_PUNHO,
    0,
    180,
    1000,
    2000,
    false,
    90,
    90,
    VELOCIDADE_PADRAO,
    0
  },
  {
    &servoCotovelo,
    "cotovelo",
    PINO_COTOVELO,
    0,
    180,
    500,
    2500,
    false,
    90,
    90,
    VELOCIDADE_PADRAO,
    0
  },
  {
    &servoOmbroMaster,
    "ombro_master",
    PINO_OMBRO_MASTER,
    0,
    180,
    500,
    2500,
    false,
    90,
    90,
    VELOCIDADE_PADRAO,
    0
  },
  {
    &servoOmbroSlave,
    "ombro_slave",
    PINO_OMBRO_SLAVE,
    0,
    180,
    500,
    2500,
    false,
    90,
    90,
    VELOCIDADE_PADRAO,
    0
  },
  {
    &servoBaseRotacao,
    "base_rotacao",
    PINO_BASE_ROTACAO,
    0,
    180,
    500,
    2500,
    false,
    90,
    90,
    VELOCIDADE_PADRAO,
    0
  }
};

// ============================================================
// POSICAO CENTRAL DE TESTE
// ============================================================
//
// Esta ainda nao e a posicao de descanso definitiva.
// Estes valores servem apenas como referencia durante os testes.

const int CENTRO_GARRA_ABERTURA = 90;
const int CENTRO_GARRA_ROTACAO  = 90;
const int CENTRO_PUNHO          = 90;
const int CENTRO_COTOVELO       = 90;
const int CENTRO_OMBRO          = 90;
const int CENTRO_BASE_ROTACAO   = 90;

// ============================================================
// FUNCOES HTTP
// ============================================================

void adicionarCors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void enviarJson(int statusHttp, const String& json) {
  adicionarCors();
  server.send(statusHttp, "application/json", json);
}

void enviarErro(
  int statusHttp,
  const String& codigo,
  const String& mensagem
) {
  String json = "{";

  json += "\"sucesso\":false,";
  json += "\"erro\":\"" + codigo + "\",";
  json += "\"mensagem\":\"" + mensagem + "\"";

  json += "}";

  enviarJson(statusHttp, json);
}

void enviarSucesso(
  const String& mensagem,
  int statusHttp = 200
) {
  String json = "{";

  json += "\"sucesso\":true,";
  json += "\"mensagem\":\"" + mensagem + "\"";

  json += "}";

  enviarJson(statusHttp, json);
}

// ============================================================
// VALIDACAO DE NUMEROS
// ============================================================

bool textoEhNumero(const String& texto) {
  if (texto.length() == 0) {
    return false;
  }

  for (unsigned int i = 0; i < texto.length(); i++) {
    if (!isDigit(texto[i])) {
      return false;
    }
  }

  return true;
}

bool lerNumero(
  const String& texto,
  int minimo,
  int maximo,
  int& resultado
) {
  if (!textoEhNumero(texto)) {
    return false;
  }

  resultado = texto.toInt();

  return resultado >= minimo && resultado <= maximo;
}

// ============================================================
// CONTROLE DOS SERVOS
// ============================================================

int limitarAngulo(int motorId, int angulo) {
  return constrain(
    angulo,
    motores[motorId].anguloMinimo,
    motores[motorId].anguloMaximo
  );
}

int converterAnguloParaPulso(int motorId, int angulo) {
  angulo = limitarAngulo(motorId, angulo);

  return map(
    angulo,
    motores[motorId].anguloMinimo,
    motores[motorId].anguloMaximo,
    motores[motorId].pulsoMinimoUs,
    motores[motorId].pulsoMaximoUs
  );
}

void escreverAngulo(int motorId, int angulo) {
  int pulso = converterAnguloParaPulso(motorId, angulo);

  motores[motorId].driver->writeMicroseconds(pulso);
}

void anexarMotorSeNecessario(
  int motorId,
  int primeiroAngulo
) {
  if (motores[motorId].anexado) {
    return;
  }

  primeiroAngulo = limitarAngulo(
    motorId,
    primeiroAngulo
  );

  motores[motorId].driver->setPeriodHertz(
    FREQUENCIA_SERVO_HZ
  );

  motores[motorId].driver->attach(
    motores[motorId].pino,
    500,
    2500
  );

  motores[motorId].anexado = true;
  motores[motorId].anguloAtual = primeiroAngulo;
  motores[motorId].anguloAlvo = primeiroAngulo;
  motores[motorId].ultimoPassoMs = millis();

  escreverAngulo(
    motorId,
    primeiroAngulo
  );

  Serial.print("Servo anexado: ");
  Serial.print(motores[motorId].nome);
  Serial.print(" | GPIO ");
  Serial.print(motores[motorId].pino);
  Serial.print(" | Angulo ");
  Serial.println(primeiroAngulo);
}

void definirAlvoMotor(
  int motorId,
  int angulo,
  int velocidade
) {
  angulo = limitarAngulo(
    motorId,
    angulo
  );

  velocidade = constrain(
    velocidade,
    1,
    180
  );

  if (!motores[motorId].anexado) {
    /*
      No primeiro comando, o servo recebe diretamente
      a posicao solicitada.

      Como os servos nao possuem sensores de retorno,
      o ESP32 nao conhece a posicao fisica anterior.
    */
    anexarMotorSeNecessario(
      motorId,
      angulo
    );

    return;
  }

  motores[motorId].anguloAlvo = angulo;
  motores[motorId].velocidade = velocidade;
}

int calcularAnguloOmbroSlave(int anguloMaster) {
  int anguloSlave;

  if (OMBRO_INVERTIDO) {
    anguloSlave = 180 - anguloMaster;
  } else {
    anguloSlave = anguloMaster;
  }

  anguloSlave += OFFSET_OMBRO_SLAVE;

  return limitarAngulo(
    OMBRO_SLAVE,
    anguloSlave
  );
}

void definirAlvoOmbro(
  int angulo,
  int velocidade
) {
  int master = limitarAngulo(
    OMBRO_MASTER,
    angulo
  );

  int slave = calcularAnguloOmbroSlave(master);

  definirAlvoMotor(
    OMBRO_MASTER,
    master,
    velocidade
  );

  definirAlvoMotor(
    OMBRO_SLAVE,
    slave,
    velocidade
  );
}

int encontrarMotorPublico(String nome) {
  nome.trim();
  nome.toLowerCase();

  if (nome == "garra_abertura" || nome == "garra") {
    return GARRA_ABERTURA;
  }

  if (nome == "garra_rotacao") {
    return GARRA_ROTACAO;
  }

  if (nome == "punho") {
    return PUNHO;
  }

  if (nome == "cotovelo") {
    return COTOVELO;
  }

  if (nome == "base" || nome == "base_rotacao") {
    return BASE_ROTACAO;
  }

  return -1;
}

bool obterLimitesPublicos(
  const String& nome,
  int& minimo,
  int& maximo
) {
  if (nome == "ombro") {
    minimo = motores[OMBRO_MASTER].anguloMinimo;
    maximo = motores[OMBRO_MASTER].anguloMaximo;

    return true;
  }

  int motorId = encontrarMotorPublico(nome);

  if (motorId < 0) {
    return false;
  }

  minimo = motores[motorId].anguloMinimo;
  maximo = motores[motorId].anguloMaximo;

  return true;
}

bool aplicarComando(
  String nome,
  int angulo,
  int velocidade,
  String& erro
) {
  nome.trim();
  nome.toLowerCase();

  int minimo;
  int maximo;

  if (!obterLimitesPublicos(nome, minimo, maximo)) {
    erro = "Servo desconhecido: " + nome;
    return false;
  }

  if (angulo < minimo || angulo > maximo) {
    erro =
      "Angulo invalido para " +
      nome +
      ". Limite: " +
      String(minimo) +
      " a " +
      String(maximo);

    return false;
  }

  if (nome == "ombro") {
    definirAlvoOmbro(
      angulo,
      velocidade
    );

    return true;
  }

  int motorId = encontrarMotorPublico(nome);

  definirAlvoMotor(
    motorId,
    angulo,
    velocidade
  );

  return true;
}

void atualizarMovimentos() {
  unsigned long agora = millis();

  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (!motores[i].anexado) {
      continue;
    }

    if (
      motores[i].anguloAtual ==
      motores[i].anguloAlvo
    ) {
      continue;
    }

    int velocidade = max(
      1,
      motores[i].velocidade
    );

    unsigned long intervalo =
      max(
        5UL,
        1000UL / (unsigned long)velocidade
      );

    if (
      agora - motores[i].ultimoPassoMs <
      intervalo
    ) {
      continue;
    }

    motores[i].ultimoPassoMs = agora;

    if (
      motores[i].anguloAtual <
      motores[i].anguloAlvo
    ) {
      motores[i].anguloAtual++;
    } else {
      motores[i].anguloAtual--;
    }

    escreverAngulo(
      i,
      motores[i].anguloAtual
    );
  }
}

void pararMovimentos() {
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (motores[i].anexado) {
      motores[i].anguloAlvo =
        motores[i].anguloAtual;
    }
  }
}

void desanexarTodos() {
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (motores[i].anexado) {
      motores[i].driver->detach();
    }

    motores[i].anexado = false;
    motores[i].anguloAtual = 90;
    motores[i].anguloAlvo = 90;

    pinMode(
      motores[i].pino,
      OUTPUT
    );

    digitalWrite(
      motores[i].pino,
      LOW
    );
  }
}

void prepararPinosSemPWM() {
  for (int i = 0; i < TOTAL_MOTORES; i++) {
    pinMode(
      motores[i].pino,
      OUTPUT
    );

    digitalWrite(
      motores[i].pino,
      LOW
    );
  }
}

// ============================================================
// STATUS
// ============================================================

String gerarStatusJson() {
  String json = "{";

  json += "\"sucesso\":true,";
  json += "\"sistema\":\"braco_robotico_teste\",";
  json += "\"versao\":\"teste_get_v1\",";
  json += "\"ip\":\"";
  json += WiFi.softAPIP().toString();
  json += "\",";
  json += "\"habilitado\":";
  json += sistemaHabilitado ? "true" : "false";
  json += ",";
  json += "\"tempo_ativo_ms\":";
  json += String(millis());
  json += ",";
  json += "\"motores\":[";

  for (int i = 0; i < TOTAL_MOTORES; i++) {
    if (i > 0) {
      json += ",";
    }

    json += "{";

    json += "\"nome\":\"";
    json += motores[i].nome;
    json += "\",";

    json += "\"pino\":";
    json += String(motores[i].pino);
    json += ",";

    json += "\"anexado\":";
    json += motores[i].anexado ? "true" : "false";
    json += ",";

    json += "\"minimo\":";
    json += String(motores[i].anguloMinimo);
    json += ",";

    json += "\"maximo\":";
    json += String(motores[i].anguloMaximo);
    json += ",";

    json += "\"atual\":";

    if (motores[i].anexado) {
      json += String(motores[i].anguloAtual);
    } else {
      json += "null";
    }

    json += ",";

    json += "\"alvo\":";

    if (motores[i].anexado) {
      json += String(motores[i].anguloAlvo);
    } else {
      json += "null";
    }

    json += "}";

  }

  json += "]";
  json += "}";

  return json;
}

// ============================================================
// PAGINA INICIAL
// ============================================================

const char PAGINA_INICIAL[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <title>API de Testes do Braço Robótico</title>

  <style>
    body {
      max-width: 850px;
      margin: 40px auto;
      padding: 0 20px;
      font-family: Arial, sans-serif;
      background: #10131a;
      color: #f1f4f8;
    }

    h1 {
      color: #8ea6ff;
    }

    h2 {
      margin-top: 32px;
      color: #b5c2ff;
    }

    p {
      line-height: 1.6;
    }

    code {
      display: block;
      margin: 8px 0;
      padding: 12px;
      overflow-wrap: anywhere;
      border: 1px solid #32394c;
      border-radius: 8px;
      background: #1a1f2b;
    }

    a {
      color: #9db3ff;
    }

    .aviso {
      padding: 14px;
      border-left: 4px solid #ffc857;
      background: #242113;
    }
  </style>
</head>

<body>
  <h1>API de Testes do Braço Robótico</h1>

  <p>
    Esta página permite testar os servos diretamente pelo navegador.
    A versão foi criada para validação da comunicação com o ESP32.
  </p>

  <div class="aviso">
    O sistema precisa ser habilitado antes dos movimentos.
    Nenhum servo é inicializado automaticamente.
  </div>

  <h2>Sistema</h2>

  <code>
    <a href="/status">/status</a>
  </code>

  <code>
    <a href="/habilitar">/habilitar</a>
  </code>

  <code>
    <a href="/desabilitar">/desabilitar</a>
  </code>

  <code>
    <a href="/stop">/stop</a>
  </code>

  <code>
    <a href="/centro">/centro</a>
  </code>

  <h2>Movimentos individuais</h2>

  <code>
    <a href="/servo?motor=garra_abertura&angle=90">
      /servo?motor=garra_abertura&amp;angle=90
    </a>
  </code>

  <code>
    <a href="/servo?motor=garra_rotacao&angle=90">
      /servo?motor=garra_rotacao&amp;angle=90
    </a>
  </code>

  <code>
    <a href="/servo?motor=punho&angle=90">
      /servo?motor=punho&amp;angle=90
    </a>
  </code>

  <code>
    <a href="/servo?motor=cotovelo&angle=90">
      /servo?motor=cotovelo&amp;angle=90
    </a>
  </code>

  <code>
    <a href="/servo?motor=ombro&angle=90">
      /servo?motor=ombro&amp;angle=90
    </a>
  </code>

  <code>
    <a href="/servo?motor=base_rotacao&angle=90">
      /servo?motor=base_rotacao&amp;angle=90
    </a>
  </code>

  <h2>Movimento conjunto</h2>

  <code>
    <a href="/move?garra_abertura=90&garra_rotacao=90&punho=90&cotovelo=90&ombro=90&base_rotacao=90">
      /move?garra_abertura=90&amp;garra_rotacao=90&amp;punho=90&amp;cotovelo=90&amp;ombro=90&amp;base_rotacao=90
    </a>
  </code>
</body>
</html>
)HTML";

// ============================================================
// ROTAS
// ============================================================

void handleRoot() {
  server.send_P(
    200,
    "text/html; charset=utf-8",
    PAGINA_INICIAL
  );
}

void handleStatus() {
  enviarJson(
    200,
    gerarStatusJson()
  );
}

void handleHabilitar() {
  sistemaHabilitado = true;

  enviarSucesso(
    "Sistema habilitado. Os servos aguardam comandos."
  );
}

void handleDesabilitar() {
  pararMovimentos();
  desanexarTodos();

  sistemaHabilitado = false;

  enviarSucesso(
    "Sistema desabilitado e sinais PWM removidos."
  );
}

void handleStop() {
  pararMovimentos();

  enviarSucesso(
    "Movimentos interrompidos."
  );
}

void handleCentro() {
  if (!sistemaHabilitado) {
    enviarErro(
      409,
      "SYSTEM_DISABLED",
      "Habilite o sistema antes de movimentar os servos."
    );

    return;
  }

  String erro;

  aplicarComando(
    "garra_abertura",
    CENTRO_GARRA_ABERTURA,
    35,
    erro
  );

  aplicarComando(
    "garra_rotacao",
    CENTRO_GARRA_ROTACAO,
    35,
    erro
  );

  aplicarComando(
    "punho",
    CENTRO_PUNHO,
    35,
    erro
  );

  aplicarComando(
    "cotovelo",
    CENTRO_COTOVELO,
    35,
    erro
  );

  aplicarComando(
    "ombro",
    CENTRO_OMBRO,
    35,
    erro
  );

  aplicarComando(
    "base_rotacao",
    CENTRO_BASE_ROTACAO,
    35,
    erro
  );

  enviarSucesso(
    "Movimento para a posicao central iniciado.",
    202
  );
}

void handleServo() {
  if (!sistemaHabilitado) {
    enviarErro(
      409,
      "SYSTEM_DISABLED",
      "Habilite o sistema usando /habilitar."
    );

    return;
  }

  if (
    !server.hasArg("motor") ||
    !server.hasArg("angle")
  ) {
    enviarErro(
      400,
      "MISSING_PARAMETERS",
      "Use /servo?motor=nome&angle=90"
    );

    return;
  }

  String nome = server.arg("motor");

  int angulo;

  if (
    !lerNumero(
      server.arg("angle"),
      0,
      180,
      angulo
    )
  ) {
    enviarErro(
      400,
      "INVALID_ANGLE",
      "O angulo deve ser um numero entre 0 e 180."
    );

    return;
  }

  int velocidade = VELOCIDADE_PADRAO;

  if (server.hasArg("speed")) {
    if (
      !lerNumero(
        server.arg("speed"),
        1,
        180,
        velocidade
      )
    ) {
      enviarErro(
        400,
        "INVALID_SPEED",
        "A velocidade deve estar entre 1 e 180."
      );

      return;
    }
  }

  String erro;

  if (
    !aplicarComando(
      nome,
      angulo,
      velocidade,
      erro
    )
  ) {
    enviarErro(
      400,
      "INVALID_COMMAND",
      erro
    );

    return;
  }

  String json = "{";

  json += "\"sucesso\":true,";
  json += "\"mensagem\":\"Comando recebido\",";
  json += "\"motor\":\"" + nome + "\",";
  json += "\"angulo\":" + String(angulo) + ",";
  json += "\"velocidade\":" + String(velocidade);

  json += "}";

  enviarJson(
    202,
    json
  );
}

void handleMove() {
  if (!sistemaHabilitado) {
    enviarErro(
      409,
      "SYSTEM_DISABLED",
      "Habilite o sistema usando /habilitar."
    );

    return;
  }

  int velocidade = VELOCIDADE_PADRAO;

  if (server.hasArg("speed")) {
    if (
      !lerNumero(
        server.arg("speed"),
        1,
        180,
        velocidade
      )
    ) {
      enviarErro(
        400,
        "INVALID_SPEED",
        "Velocidade invalida."
      );

      return;
    }
  }

  const char* nomes[] = {
    "garra_abertura",
    "garra_rotacao",
    "punho",
    "cotovelo",
    "ombro",
    "base_rotacao"
  };

  const int quantidadeNomes =
    sizeof(nomes) / sizeof(nomes[0]);

  bool encontrouComando = false;

  // Primeiro valida todos os parametros.
  for (int i = 0; i < quantidadeNomes; i++) {
    String nome = nomes[i];

    if (!server.hasArg(nome)) {
      continue;
    }

    encontrouComando = true;

    int angulo;

    if (
      !lerNumero(
        server.arg(nome),
        0,
        180,
        angulo
      )
    ) {
      enviarErro(
        400,
        "INVALID_ANGLE",
        "Angulo invalido para " + nome
      );

      return;
    }

    int minimo;
    int maximo;

    if (
      !obterLimitesPublicos(
        nome,
        minimo,
        maximo
      )
    ) {
      enviarErro(
        400,
        "UNKNOWN_SERVO",
        "Servo desconhecido: " + nome
      );

      return;
    }

    if (angulo < minimo || angulo > maximo) {
      enviarErro(
        400,
        "ANGLE_OUT_OF_RANGE",
        "Angulo fora do limite para " + nome
      );

      return;
    }
  }

  if (!encontrouComando) {
    enviarErro(
      400,
      "EMPTY_COMMAND",
      "Nenhuma posicao foi informada."
    );

    return;
  }

  // Depois aplica todos os parametros validados.
  for (int i = 0; i < quantidadeNomes; i++) {
    String nome = nomes[i];

    if (!server.hasArg(nome)) {
      continue;
    }

    int angulo = server.arg(nome).toInt();
    String erro;

    aplicarComando(
      nome,
      angulo,
      velocidade,
      erro
    );
  }

  enviarSucesso(
    "Movimento conjunto iniciado.",
    202
  );
}

void handleOptions() {
  adicionarCors();
  server.send(204);
}

void handleNotFound() {
  if (server.method() == HTTP_OPTIONS) {
    handleOptions();
    return;
  }

  enviarErro(
    404,
    "NOT_FOUND",
    "Rota nao encontrada."
  );
}

// ============================================================
// WI-FI
// ============================================================

void iniciarWiFi() {
  Serial.println("Inicializando rede Wi-Fi de testes...");

  WiFi.mode(WIFI_OFF);
  delay(300);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  bool configuracaoOk = WiFi.softAPConfig(
    AP_IP,
    AP_GATEWAY,
    AP_SUBNET
  );

  if (!configuracaoOk) {
    Serial.println(
      "Aviso: falha ao configurar o IP fixo."
    );
  }

  bool redeCriada = WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    6,
    false,
    4
  );

  if (!redeCriada) {
    Serial.println(
      "ERRO: nao foi possivel criar a rede Wi-Fi."
    );

    return;
  }

  Serial.println("Rede criada com sucesso.");

  Serial.print("SSID: ");
  Serial.println(AP_SSID);

  Serial.print("Senha: ");
  Serial.println(AP_PASSWORD);

  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

// ============================================================
// CONFIGURACAO DAS ROTAS
// ============================================================

void configurarRotas() {
  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/status",
    HTTP_GET,
    handleStatus
  );

  server.on(
    "/habilitar",
    HTTP_GET,
    handleHabilitar
  );

  server.on(
    "/desabilitar",
    HTTP_GET,
    handleDesabilitar
  );

  server.on(
    "/stop",
    HTTP_GET,
    handleStop
  );

  server.on(
    "/centro",
    HTTP_GET,
    handleCentro
  );

  server.on(
    "/home",
    HTTP_GET,
    handleCentro
  );

  server.on(
    "/servo",
    HTTP_GET,
    handleServo
  );

  server.on(
    "/move",
    HTTP_GET,
    handleMove
  );

  server.onNotFound(handleNotFound);
}

// ============================================================
// SETUP E LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("API DE TESTES DO BRACO ROBOTICO");
  Serial.println("========================================");

  /*
    Nenhum servo e anexado nesta etapa.
    Nenhuma posicao e enviada automaticamente.
  */
  prepararPinosSemPWM();

  iniciarWiFi();
  configurarRotas();

  server.begin();

  Serial.println("Servidor HTTP iniciado.");
  Serial.println("Acesse no navegador:");
  Serial.println("http://192.168.4.1");
}

void loop() {
  server.handleClient();
  atualizarMovimentos();
}