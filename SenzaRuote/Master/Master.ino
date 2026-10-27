#include <WiFi.h>
#include <esp_now.h>
#include <NewPing.h>

// ─── PIN SENSORE ULTRASUONI ────────────────────────────
#define trigPin 12
#define echoPin 14

// ─── ESP-NOW ───────────────────────────────────────────
uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x8A, 0x17, 0xB4};

typedef struct {
  int32_t velocita;
  int32_t distanza;
} Dati;

Dati dati;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Invio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ─── PARAMETRI PID ─────────────────────────────────────
double Kp = 2.5;
double Ki = 0.1;
double Kd = 0.05;

// ─── VARIABILI PID ─────────────────────────────────────
double distanza_desiderata  = 10.0;
double distanza_misurata    = 0.0;
double errore_proporzionale = 0.0;
double errore_integrale     = 0.0;
double errore_derivativo    = 0.0;
double ultimo_errore        = 0.0;
double uscitaPWM            = 0.0;
int    input_utente         = 200;

// ─── TIMING ────────────────────────────────────────────
const unsigned long Tempo_di_campionamento = 500;
unsigned long lastTime      = 0;
double dt                   = 0.0;
bool campionamento_pronto   = false;

// ─── SONAR ─────────────────────────────────────────────
NewPing sonar(trigPin, echoPin, 400);

// ═══════════════════════════════════════════════════════
// MISURA DISTANZA E TEMPO
// ═══════════════════════════════════════════════════════
void misura_distanze_e_tempi() {
  unsigned long ora = millis();

  if (ora - lastTime < Tempo_di_campionamento) {
    campionamento_pronto = false;
    return;
  }

  dt       = (ora - lastTime) / 1000.0;
  lastTime = ora;
  campionamento_pronto = true;

  distanza_misurata = sonar.ping_cm();

  Serial.print("[SONAR] distanza = ");
  Serial.print(distanza_misurata);
  Serial.print(" cm | dt = ");
  Serial.println(dt);
}

// ═══════════════════════════════════════════════════════
// CALCOLO PID
// ═══════════════════════════════════════════════════════
double calcoloPID() {

  // Nessun ostacolo → cruise control
  if (distanza_misurata == 0) {
    errore_integrale = 0.0;
    ultimo_errore    = 0.0;
    return (double)input_utente;
  }

  errore_proporzionale = distanza_misurata - distanza_desiderata;

  double errore_integrale_temp = errore_integrale + errore_proporzionale * dt;

  errore_derivativo = (errore_proporzionale - ultimo_errore) / dt;
  ultimo_errore = errore_proporzionale;

  double uscita = Kp * errore_proporzionale
                + Ki * errore_integrale_temp
                + Kd * errore_derivativo;

  double uscita_saturata = constrain(uscita, 0.0, (double)input_utente);

  // Anti-windup
  if (uscita_saturata == uscita) {
    errore_integrale = errore_integrale_temp;
  }

  return uscita_saturata;
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  lastTime = millis();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init fallita!");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Aggiunta peer fallita!");
    return;
  }

  Serial.println("Setup completato.");
}

// ═══════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════
void loop() {

  misura_distanze_e_tempi();

  if (!campionamento_pronto) return;

  uscitaPWM = calcoloPID();
  int PWM_EFFETTIVI = (int)uscitaPWM;

  // Debug seriale
  Serial.print("[PID] errore=");
  Serial.print(errore_proporzionale);
  Serial.print(" | integrale=");
  Serial.print(errore_integrale);
  Serial.print(" | PWM=");
  Serial.println(PWM_EFFETTIVI);

  // Invio ESP-NOW
  dati.velocita = PWM_EFFETTIVI;
  dati.distanza = (int32_t)distanza_misurata;
  esp_now_send(receiverMAC, (uint8_t *)&dati, sizeof(dati));

  Serial.print("[ESP-NOW] velocita=");
  Serial.print(dati.velocita);
  Serial.print(" | distanza=");
  Serial.println(dati.distanza);
}