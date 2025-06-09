//
// Created By eloig 
//

// *******************************************************//
//          Projet Systeme Bouclé ECE BORDEAUX            //
// *******************************************************//

// ================= LIBRAIRIES ================= //
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>


// ================= SERV WIFI POUR SE CO ================= //
const char* ssid = "Drawbot_WIFI";
const char* password = "12345678";
WebServer server(80);


// ================= On def tous les PINS ================= //
// Moteur Droit
#define EN_D 23
#define IN_1_D 19
#define IN_2_D 18


// Moteur Gauche
#define EN_G 4
#define IN_1_G 17
#define IN_2_G 16


// Adresse I2C du LIS3MDL
#define ADDR_MAG 0x1E


// Registres du LIS3MDL
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define OUT_X_L   0x28

//Pour le cercle
#define NB_POINTS_CERCLE 32   // Plus il y a de points, plus c'est fluide mais on reste a 32 ça bug trop
struct Point {
    float x;
    float y;
};
Point pointsCercle[NB_POINTS_CERCLE];


#define conversion_number 36 


// ================= ENCODEURS POUR ESCALIER ================= //
#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;


// ================= CONSTANTES PHYSIQUES ================= //
const float DIAMETRE_ROUE_CM = 9.0;
const float EMPATTEMENT_CM = 10.5; // calibré après bcp trop d'essai
const float TICKS_PAR_TOUR = 830.0;
const float PERIMETRE_ROUE = PI * DIAMETRE_ROUE_CM;


// ================= FONCTIONS MOTEUR ================= //
void avancer(int pwm);
void reculer(int pwm);
void tournerDroite(int pwm);
void tournerGauche(int pwm);
void stopMoteurs();


// ================= SEQUENCES ================= //
void sequence1();
void sequence2();
void sequence3();


// ISR encodeur gauche
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_G_CH_B)) leftEncoderCount++;
  else leftEncoderCount--;
}

// ISR encodeur droit (En inverse car moteur droit est inversé.. j'ai juste inversé les cables mais on est restés comme ça ..)
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_D_CH_B)) rightEncoderCount--;
  else rightEncoderCount++;
}


// Fonction utilitaire : retourne le cap (degrés 0-360, nord=0) pour la boussole
const float OFFSET_NORD = 255.0; // reglé au degré près à la main pour tomber  pile sur 0° Nord  


float lireCapNord() {
  int16_t mx, my, mz;
  Wire.beginTransmission(ADDR_MAG);
  Wire.write(OUT_X_L | 0x80);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR_MAG, 6);

  if (Wire.available() == 6) {
    uint8_t xL = Wire.read();
    uint8_t xH = Wire.read();
    uint8_t yL = Wire.read();
    uint8_t yH = Wire.read();
    uint8_t zL = Wire.read();
    uint8_t zH = Wire.read();
    mx = (int16_t)(xH << 8 | xL);
    my = (int16_t)(yH << 8 | yL);
    mz = (int16_t)(zH << 8 | zL);

    float mx_offset = (-645 -2930) / 2.0;
    float my_offset = (1390 -1180) / 2.0;
    float mx_corr = mx - mx_offset;
    float my_corr = my - my_offset;

    float heading = atan2(my_corr, mx_corr) * 180.0 / PI;
    heading = 325.0 - heading;

    // === Correction offset nord ===
    heading = heading - OFFSET_NORD;
    heading = fmod(heading + 360.0, 360.0);

    return heading;
  }
  return -1; // Erreur de lecture
}


class Wheel {
    public:
    volatile long int target_ticks;
    volatile long int nbr_ticks;
    volatile long int error;
    volatile long int last_error;
    short int speed;
};
Wheel wheel_left;
Wheel wheel_right;

float kp_dist = 2, ki_dist = 0, kd_dist = 0.9;
double P_dist = 0, I_dist = 0, D_dist = 0;

float kp_diff = 1.2, ki_diff = 0, kd_diff = 0.2;
float P_diff = 0, I_diff = 0, D_diff = 0;
float pid_distance(int distance_cm);


void generePointsCercle(float rayon_cm) {
    float r = rayon_cm / 100.0; // conversion en mètres 
    for (int i = 0; i < NB_POINTS_CERCLE; i++) {
        float theta = 2.0 * PI * i / NB_POINTS_CERCLE;
        pointsCercle[i].x = r * cos(theta);
        pointsCercle[i].y = r * sin(theta);
    }
}


// ================= HTML POUR INTERFACE DU CONTROL PANEL ================= //
void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <title>DRAWBOT - Contrôle Web</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <!-- Bootstrap CSS -->
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.10.5/font/bootstrap-icons.css">
  <style>
    body { background: #f0f0f0; }
    .pad-row { display: flex; justify-content: center; align-items: center; margin-bottom: 1rem; }
    .btn-pad {
      background: #007179; color: white; border: none; font-size: 2rem;
      border-radius: 1rem; margin: .2rem .7rem; min-width: 75px; min-height: 75px; transition: .2s;
      display: flex; align-items: center; justify-content: center;
    }
    .btn-pad:hover { background: #00aeba; }
    .btn-pad.stop { background: #ec2424; }
    .btn-pad.stop:hover { background: #b61d1d; }
    .btn-pad.reset { background: #888; font-size: 1.5rem;}
    .btn-pad.reset:hover { background: #444;}
    .pad-container { max-width: 370px; margin: 48px auto; }
    h1 { color: #007179; text-align: center; margin-top: 2rem; font-size: 2.5rem;}
    .seq-title { text-align: center; margin-top: 3rem; font-size: 1.3rem; color: #444; }
    .seq-row { display: flex; justify-content: center; gap: 1.5rem; margin-top: 1.5rem; }
    .btn-seq {
      background: #234086; color: #fff; font-size: 1.1rem; border: none; border-radius: .6rem;
      padding: .8rem 2.2rem; font-weight: 500; transition: .2s;
    }
    .btn-seq:hover { background: #506fc2; }
  </style>
  <script>
    function action(path) { fetch(path); }
  </script>
</head>
<body>
  <h1>DRAWBOT – Contrôle Web</h1>
  <div class="pad-container">
    <!-- Ligne Avancer -->
    <div class="pad-row">
      <button class="btn-pad" onclick="action('/avancer')"><i class="bi bi-arrow-up"></i></button>
    </div>
    <!-- Ligne Gauche / Stop / Droite -->
    <div class="pad-row">
      <button class="btn-pad" onclick="action('/gauche')"><i class="bi bi-arrow-left"></i></button>
      <button class="btn-pad stop" onclick="action('/stop')"><i class="bi bi-square-fill"></i></button>
      <button class="btn-pad" onclick="action('/droite')"><i class="bi bi-arrow-right"></i></button>
    </div>
    <!-- Ligne Reculer / Reset -->
    <div class="pad-row">
      <button class="btn-pad" onclick="action('/reculer')"><i class="bi bi-arrow-down"></i></button>
      <button class="btn-pad reset" onclick="action('/reset')"><i class="bi bi-arrow-counterclockwise"></i></button>
    </div>
  </div>
  <div class="seq-title">Lancer une séquence :</div>
  <div class="seq-row">
    <button class="btn-seq" onclick="action('/start_sequence1')"></i>Séquence 1</button>
    <button class="btn-seq" onclick="action('/start_sequence2')"></i>Séquence 2</button>
    <button class="btn-seq" onclick="action('/start_sequence3')"></i>Séquence 3</button>
  </div>
</body>
</html>
    )rawliteral";
    server.send(200, "text/html", html);
}


// ================= HANDLERS SEQUENCES ================= //
void handleSeq1() { sequence1(); server.send(200, "text/plain", "Séquence 1 lancée !"); }
void handleSeq2() { sequence2(); server.send(200, "text/plain", "Séquence 2 lancée !"); }
void handleSeq3() { sequence3(); server.send(200, "text/plain", "Séquence 3 lancée !"); }


// ================= HANDLERS MOUVEMENTS ================= //
void handleAvancer()  { avancer(130);   server.send(200, "text/plain", "Avance !"); }
void handleReculer()  { reculer(130);   server.send(200, "text/plain", "Recule !"); }
void handleDroite()   { tournerDroite(130); server.send(200, "text/plain", "Droite !"); }
void handleGauche()   { tournerGauche(130); server.send(200, "text/plain", "Gauche !"); }
void handleStop()     { stopMoteurs();  server.send(200, "text/plain", "Stop !"); }
void handleReset()    { stopMoteurs();  server.send(200, "text/plain", "Reset !"); }

// ================= SETUP ================= //
void setup() {
  Serial.begin(115200);

  // ==== 1. INITIALISATION I2C et LIS3MDL AVANT LE WIFI ====
  Wire.begin(); 

  // --- Config LIS3MDL ---
  Wire.beginTransmission(ADDR_MAG);
  Wire.write(CTRL_REG1);
  Wire.write(0b11110000); // Turbo performance apparemment, 80 Hz 
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(CTRL_REG2);
  Wire.write(0b00000000); // ±4 gauss
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(CTRL_REG3);
  Wire.write(0b00000000); // Mode continu
  Wire.endTransmission();

  // ==== 2. Initialisation des pins moteurs ====
  pinMode(EN_D, OUTPUT); pinMode(IN_1_D, OUTPUT); pinMode(IN_2_D, OUTPUT);
  pinMode(EN_G, OUTPUT); pinMode(IN_1_G, OUTPUT); pinMode(IN_2_G, OUTPUT);

  // -- Initialisation des pins encodeur
  pinMode(ENC_G_CH_A, INPUT_PULLUP);
  pinMode(ENC_G_CH_B, INPUT_PULLUP);
  pinMode(ENC_D_CH_A, INPUT_PULLUP);
  pinMode(ENC_D_CH_B, INPUT_PULLUP);

  // -- Attache les interruptions
  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), rightEncoderISR, RISING);

  // ==== 3. APRES TOUT ÇA, INITIALISATION WIFI ====
  WiFi.softAP(ssid, password);
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.softAPIP());

  // ==== 4. Serveur Web ====
  server.on("/", handleRoot);
  server.on("/avancer", handleAvancer);
  server.on("/reculer", handleReculer);
  server.on("/gauche", handleGauche);
  server.on("/droite", handleDroite);
  server.on("/stop", handleStop);
  server.on("/reset", handleReset);
  server.on("/start_sequence1", handleSeq1);
  server.on("/start_sequence2", handleSeq2);
  server.on("/start_sequence3", handleSeq3);

  server.begin();
  Serial.println("Serv web ok ");
}


void loop() {
  server.handleClient();
}


// ================= LES FONCTIONS ================= //
void avancer(int pwm) {
  // Moteur gauche avance
  digitalWrite(IN_1_G, HIGH);
  digitalWrite(IN_2_G, LOW);
  // Moteur droit avance (INVERSE du coup car tjrs les cables inversés)
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, HIGH);

  analogWrite(EN_D, pwm);
  analogWrite(EN_G, pwm);
}

void reculer(int pwm) {
  // Moteur gauche recule
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, HIGH);
  // Moteur droit recule (INVERSE)
  digitalWrite(IN_1_D, HIGH);
  digitalWrite(IN_2_D, LOW);

  analogWrite(EN_D, pwm);
  analogWrite(EN_G, pwm);
}

void tournerDroite(int pwm) {
  // Gauche avance, droite recule
  digitalWrite(IN_1_G, HIGH);
  digitalWrite(IN_2_G, LOW);
  digitalWrite(IN_1_D, HIGH);
  digitalWrite(IN_2_D, LOW);

  analogWrite(EN_D, pwm);
  analogWrite(EN_G, pwm);
}

void tournerGauche(int pwm) {
  // Gauche recule, droite avance
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, HIGH);
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, HIGH);

  analogWrite(EN_D, pwm);
  analogWrite(EN_G, pwm);
}

void stopMoteurs() {
  analogWrite(EN_D, 0);
  analogWrite(EN_G, 0);
}


void avancerDistance(float cm, int pwm = 130) {
  const float ticksParTour = 830.0;
  const float perimRoue = PI * 9.0;
  long ticksCible = (long)(cm * ticksParTour / perimRoue);

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Avance tout droit
  digitalWrite(IN_1_G, HIGH);  digitalWrite(IN_2_G, LOW);
  digitalWrite(IN_1_D, LOW);   digitalWrite(IN_2_D, HIGH);
  analogWrite(EN_G, pwm);
  analogWrite(EN_D, pwm);

  unsigned long t0 = millis();
  while ((abs(leftEncoderCount) < ticksCible || abs(rightEncoderCount) < ticksCible) && millis() - t0 < 8000) {
    delay(5);
  }

  analogWrite(EN_G, 0);
  analogWrite(EN_D, 0);
  delay(150);
}


void tournerAngle(float degre, int pwm = 130) {
  const float ticksPour360 = 625.0; // J'ai mesuré à la main après 25 tentatives de calibrage !! Mais tour précis  
  long ticksCible = (long)(ticksPour360 * abs(degre) / 360.0);

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  if (degre > 0) {
    // Tourner à gauche
    digitalWrite(IN_1_G, LOW);  digitalWrite(IN_2_G, HIGH);  // gauche recule
    digitalWrite(IN_1_D, LOW);  digitalWrite(IN_2_D, HIGH);  // droite avance (INVERSE)
  } else {
    // Tourner à droite
    digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);   // gauche avance
    digitalWrite(IN_1_D, HIGH); digitalWrite(IN_2_D, LOW);   // droite recule (INVERSE)
  }
  analogWrite(EN_G, pwm);
  analogWrite(EN_D, pwm);

  unsigned long t0 = millis();
  while ((abs(leftEncoderCount) < ticksCible || abs(rightEncoderCount) < ticksCible) && millis() - t0 < 5000) {
    delay(5);
  }
  analogWrite(EN_G, 0);
  analogWrite(EN_D, 0);
  delay(150);
}


void avancerEtTourner(float distance_cm, float angle_deg, int pwm = 200) {
    // Avance sur une petite distance
    avancerDistance(distance_cm, pwm);
    // Tourne de l’angle indiqué
    if (angle_deg > 1) {
        tournerGauche(pwm);
        delay((int)(abs(angle_deg) * 4)); // 4ms par degré (faut continuer à l'ajuster c'est pas exactement bon)
        stopMoteurs();
    }
    else if (angle_deg < -1) {
        tournerDroite(pwm);
        delay((int)(abs(angle_deg) * 4));
        stopMoteurs();
    }
}


// ==================== TESTS MOTEUR – MODES PWM ====================

// Coupe les moteurs
void stopMoteursPWM() {
  analogWrite(EN_G, 0);
  analogWrite(EN_D, 0);
}

// Avance tout droit jusqu'à targetTicks (encodeur gauche), PWM fixe
void avancerTicksConstante(long targetTicks, int pwm) {
  leftEncoderCount = 0;
  digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  digitalWrite(IN_1_D, LOW);  digitalWrite(IN_2_D, HIGH);

  analogWrite(EN_G, pwm);
  analogWrite(EN_D, pwm);

  while (abs(leftEncoderCount) < targetTicks) {
    delay(1);
  }
  stopMoteursPWM();
}

// Avance en augmentant la PWM DROITE de +10 tous les 100 ticks (après 70)
void avancerAvecRampDroite(long targetTicks, int pwmG_init, int pwmD_init) {
  leftEncoderCount = 0;
  int currentPwmG = pwmG_init;
  int currentPwmD = pwmD_init;
  long nextThreshold = 70;

  digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  digitalWrite(IN_1_D, LOW);  digitalWrite(IN_2_D, HIGH);

  analogWrite(EN_G, currentPwmG);
  analogWrite(EN_D, currentPwmD);

  while (abs(leftEncoderCount) < targetTicks) {
    if (abs(leftEncoderCount) >= nextThreshold) {
      currentPwmD = min(currentPwmD + 6, 170);
      analogWrite(EN_D, currentPwmD);
      nextThreshold += 100;
    }
    delay(1);
  }
  stopMoteursPWM();
}

// Avance en augmentant la PWM GAUCHE de +35 tous les 100 ticks (après 70)
void avancerAvecRampGauche(long targetTicks, int pwmG_init, int pwmD_init) {
  rightEncoderCount = 0;
  int currentPwmG = pwmG_init;
  int currentPwmD = pwmD_init;
  long nextThreshold = 70;

  digitalWrite(IN_1_G, HIGH); digitalWrite(IN_2_G, LOW);
  digitalWrite(IN_1_D, LOW);  digitalWrite(IN_2_D, HIGH);

  analogWrite(EN_G, currentPwmG);
  analogWrite(EN_D, currentPwmD);

  while (abs(rightEncoderCount) < targetTicks) {
    if (abs(rightEncoderCount) >= nextThreshold) {
      currentPwmG = min(currentPwmG + 35, 255);
      analogWrite(EN_G, currentPwmG);
      nextThreshold += 100;
    }
    delay(1);
  }
  stopMoteursPWM();
}


// ==================== LES SEQUENCES ==================== //
// ========== SEQUENCE 1 ========== //
void sequence1() {
  Serial.println("Séquence 1 : TEST MOTEUR avec rampes");

  // 1) PWM constante sur les deux moteurs
  avancerTicksConstante(480, 180);
  delay(1000);

  // 2) Ramp gauche (PWM gauche augmente)
  avancerAvecRampGauche(160, 10, 200);
  delay(1000);

  // 3) Ramp droite (PWM droite augmente)
  avancerAvecRampDroite(1250, 200, 80);
  delay(1000);

  stopMoteursPWM();
  Serial.println("Fin du test moteur !");
}



// ========== SEQUENCE 2 ========== //
void sequence2() {
    Serial.println("Séquence 2 : Cercle");

    float rayon_cm = 10;   
    generePointsCercle(rayon_cm);

    for (int i = 0; i < NB_POINTS_CERCLE; i++) {
        // Calcul du prochain point relatif
        int j = (i + 1) % NB_POINTS_CERCLE;
        float dx = pointsCercle[j].x - pointsCercle[i].x;
        float dy = pointsCercle[j].y - pointsCercle[i].y;

        // Calcul de la distance à parcourir 
        float dist = sqrt(dx*dx + dy*dy) * 100.0; // repasse en cm
        // Calcul de l’angle à tourner 
        float angle = atan2(dy, dx) * 180.0 / PI;
        // On veut juste tourner de angle / NB_POINTS_CERCLE à chaque fois
        float angle_per_step = 360.0 / NB_POINTS_CERCLE;

        // Avance et tourne
        avancerEtTourner(dist, angle_per_step, 200);
        delay(30); // Micro-pause entre chaque étape
    }

    stopMoteurs();
    Serial.println("Cercle terminé !");
}



// ========== SEQUENCE 3 ========== //
void sequence3() {
  Serial.println("Séquence 3 : On cherche le Nord ");
  stopMoteurs();
  delay(200);

  // Étape 1 : Faire un tour complet sur place 360
  Serial.println("Tour complet sur place avant recherche du Nord...");
  tournerDroite(180);  // On peut changer la puissance
  delay(1700);         // À calibrer selon pour  vitesse de rotation
  stopMoteurs();
  delay(300);

  // Étape 2 : Recherche du Nord
  float heading = lireCapNord();
  if (heading < 0) {
    Serial.println("Erreur du capteur ");
    return;
  }

  const float tolerance = 5.0; // ±5° autour du nord
  int tries = 0, triesMax = 300;

  while (tries++ < triesMax) {
    heading = lireCapNord();
    Serial.print("Heading: ");
    Serial.println(heading);

    float diff = heading;
    if (diff > 180) diff -= 360;

    if (abs(diff) < tolerance) {
      break;
    } else if (diff > 0) {
      tournerDroite(130);
    } else {
      tournerGauche(130);
    }
    delay(30);
    stopMoteurs();
    delay(30);
  }
  stopMoteurs();
  Serial.println("Robot vers le NORD !");
}


