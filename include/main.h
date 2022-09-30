/**
 *@brief Fichier d'en-tête pour le main.cpp. Ce fichier contient des variables et les prototypes de fonctions.
 *@file main.h
 *@date septembre 2021
**/

/*--------------------------------------------------------------- Libraires ---------------------------------------------------------------*/
#include <EEPROM.h>  // Librairie pour utiliser la mémoire EEPROM de la carte Arduino (https://www.arduino.cc/en/Reference/EEPROM)
#include <Nextion.h> // Voir le lien suivant: https://randomnerdtutorials.com/nextion-display-with-arduino-getting-started/ qui explique
                     // bien la librairie Nextion.h pour l'écran tactile
#include <SPI.h>     //(https://www.arduino.cc/en/reference/SPI)
#include <HX711.h>   // Librairie pour lire le load cell du Zenith. Load cell permet de mesurer le poids du patient
                     // lorsqu'il est complètement soulevé (ne touche plus au sol). HX711 est un amplificateur de signal pour la load cell
                     // (pont de Wheatstone). Documentation: https://www.robotshop.com/ca/en/hx711-load-cell-amplifier.html
#include <Adafruit_BNO055.h> // Libraire pour l'orientation des 9 axes de la nouvelle manette (https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
#include <Arduino.h> // Librairie pour l'utilisation de la carte Arduino
#include <Wire.h> //Libairie pour l'utilisation de I2C
#include <Servo.h> //Libraire pour l'utilisation du servomoteur pour la calibration du IMU (BNO055)
#include <string.h> 
//#include <EnableInterrupt.h> //Librairie pour l'utilisation des interruptions externes 
#include <encoder.h> //Librairie pour l'utilisation des encodeurs
//#include <QuadratureEncoder.h> //Librairie pour l'utilisation des encodeurs

/*---------------------------------------------------------- Variables et objets ----------------------------------------------------------*/
/*--- ENCODEURS DE POSITION DES VÉRINS ---*/
const byte EncoderVerinGauche_A = 48;
const byte EncoderVerinGauche_B = 49;
const byte EncoderVerinDroit_A = 48;
const byte EncoderVerinDroit_B = 49;


signed long encoder1count = 0; 
signed long encoder2count = 0;
long encoder1Reading = 0;
long encoder2Reading = 0;
long encoder1ReadingPrev = 0;
long encoder2ReadingPrev = 0;

Encoder EncoderVerinGauche(EncoderVerinGauche_A, EncoderVerinGauche_B);
Encoder EncoderVerinDroit(EncoderVerinDroit_A, EncoderVerinDroit_B);

/*--- ENCODEURS AUX ROUES ---*/
const byte EncoderRW_A = 18; //Pin de l'encodeur de la roue droite
const byte EncoderRW_B = 15; 
const byte EncoderLW_A = 19; //Pin de l'encodeur de la roue gauche
const byte EncoderLW_B = 14; 

Encoder EncoderRoueGauche(EncoderLW_A, EncoderLW_B);
Encoder EncoderRoueDroite(EncoderRW_A, EncoderRW_B);

#define EncoderHoleQuantity 2400
#define M_S_TO_KM_H 3.6

volatile int EncoderStepL;
volatile int EncoderStepR;
long oldEncoder3Steps = 0;
long newEncoder4Steps;
long oldEncoder4Steps = 0;
unsigned long startTime=0;
unsigned long endTime =0;
float temp=0;
float vitesseDroiteReelle=0;
float vitesseGaucheReelle=0;
float vitesseDesireeDroite = 0;
float vitesseDesireeGauche = 0;
float rayonRoue = 0.1778; //mètres
long acquisitionTimeRight = 1000;
unsigned long lastacquisitionTimeRight = 0;
long acquisitionTimeLeft = 1000;
unsigned long lastacquisitionTimeLeft = 0;
unsigned long TimeDebouceL = 0;
unsigned long TimeDebouceR = 0;


/*--- INTERNAL MESURMENTS UNIT ---*/
#define RadToDeg  180 / PI
#define DegToRad  PI / 180

String ID = "1A0B&";
String IDattendu = "2B1C";

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
adafruit_bno055_offsets_t allOffsets;

// Servo servoCalibration;
// int servoPos =0;

/*--- EEPROM ---*/
int eepromPoidsOffsetAddr = 0;

/*--- DÉPLACEMENT ---*/
int directiontemp = 0;  // Variable temporaire
int modetemp = 0;       // Variable temporaire
uint32_t speedcoef = 0; // Vitesse de déplacement, variant entre 0 et 10 (valeur par défaut)
float coefVitesse = 0;  // Coefficient de vitesse, variant entre 0 et 10 (valeur par défaut)
int JogX;               // Axe X issue du joystick de la manette
int JogY;               // Axe Y issue du joystick de la manette
int jogCoef = 19.1;     // Multiple de speedcoef
float pwmG_Value = 0;   // Valeur PWM moteur côté batterie déduite de JoxX et JogY
float pwmD_Value = 0;   // Valeur PWM moteur côté contrôleur déduite de JoxX et JogY
int PWMG = 0;           // PWM visé pour moteur côté batterie
int PWMD = 0;           // PWM visé pour moteur côté contrôleur
int PWMG_reel = 0;    // PWM réellement envoyé au moteur côté batterie
int PWMD_reel = 0;    // PWM réellement envoyé au moteur côté contrôleur
int diff_PWMG = 0;          // Coefficient de variation du PWM pour limiter le jerk du côté batterie
int diff_PWMD = 0;          // Coefficient de variation du PWM pour limiter le jerk du côté contrôleur
unsigned long jerkTimer = millis();
unsigned long jerkTime = 20;    // Délai entre chaque variation du PWM pour limiter le jerk
bool wheelstopped = 1; // Appareil immobilisé ?
#define ACCELMAX 3

/*--- LEVAGE ---*/
int liftCoef = 255;   // Vitesse de levage maximale, sur 255
int PWMVG = 0;        // PWM Vérin côté batterie
int PWMVD = 0;        // PWM Vérin côté contrôleur
int PWM_VG_reel = 0; // PWM réellement envoyé au Vérin côté batterie
int PWM_VD_reel = 0; // PWM réellement envoyé au Vérin côté contrôleur
int diff_PWM_VG = 0;       // Coefficient de variation du PWM pour limiter le jerk du côté batterie
int diff_PWM_VD = 0;       // Coefficient de variation du PWM pour limiter le jerk du côté contrôleur
bool btnreleased = 0; // Bouton commande manuelle du levage relâchée ?
int lmtTrigg = 0;     // Nombre de détection de fin de course consécutives des vérins
int referenceBouton = 0;

/*Asservissement PID*/
int e = 0;                  // Erreur asservissement vérin (écart entre les compteurs effet Hall gauche vs droit)
int eMin = 1;               // Erreur minimale tenue en compte (dead zone)
double lastError;           // Buffer pour calcul de la dérivée
double cumError, rateError; // Intégrale et dérivée du PID
int rateErrorMax = 10;      // Valeur maxi de dérivée du PID
float kp = 1;               // Coefficient proportionnel
float ki = 0;               // Coefficient intégral
float kd = 3;               // Coefficient dérivée
double output;              // Écart appliqué entre les PWMs des vérins gauche et droit
int outputMax = 60;         // Valeur maxi de output (anti windup)
float kMax = 5;             // Valeur maxi des boutons scroll d'ajustement des coefficient du PID, sur écran
bool PIDset = 0;            // Les boutons scroll ont été mis à jour ?
HX711 scale;                // Objet pour amplificateur cellule de charge

/*Asservissement PID Moteur*/
float eM = 0;                  // Erreur asservissement vérin (écart entre les compteurs effet Hall gauche vs droit)
float eMinM = 0.2;               // Erreur minimale tenue en compte (dead zone)
double lastErrorM_R, lastErrorM_L;           // Buffer pour calcul de la dérivée
double cumErrorM_R, rateErrorM_R, cumErrorM_L, rateErrorM_L; // Intégrale et dérivée du PID
int rateErrorMaxM = 10;      // Valeur maxi de dérivée du PID
float kpM = 3;               // Coefficient proportionnel
float kiM = 0;               // Coefficient intégral
float kdM = 0;               // Coefficient dérivée
double outputM;              // Écart appliqué entre les PWMs des vérins gauche et droit
int outputMaxM = 25;         // Valeur maxi de output (anti windup)

/*--- MAINTIEN ---*/
uint32_t pcmaintien = 50; // Pourcentage de maintien entre 0 et 100 (valeur par défaut)
float POIDS = 0;          // Valeur pesée, sans offset d'appliqué
float poids;              // Poids soutenu visé
int poidsOffset;          // Offset appliqué au poids, selon calibration ou valeur enregistére en EEPROM
int percu;                // Poids perçu par l'utilisateur

/*--- Ressorts ---*/
int R917 = 6; // poids soutenu équivalent (kg) pour chaque ressort 0917, 0919 etc.
int R919 = 2;
int R921 = 11;
int R925 = 20;
int R928 = 32;
int R929 = 50;
int rcomb[64]; // Array pour poids de toutes les combinaisons de ressort

/*--- Fourchettes ---*/
int Rconfig[8] = {127, 140, 158, 175, 190, 206, 220, 240}; // Positions fourchette côté contrôleur
int rconfig[8] = {127, 140, 150, 163, 176, 188, 200, 220}; // Positions fourchette côté batterie
int Rkg[8];                                                // Arrays pour poids soutenu à chaque configuration fourchette côté contrôleur
int rkg[8];                                                // Arrays pour poids soutenu à chaque configuration fourchette côté batterie
int PWMFG = 0;                                             // PWM Fourchette côté contrôleur (PWMFC)
int PWMFD = 0;                                             // PWM Fourchette côté batterie (PWMFB)

/*--- BATTERIE ---*/
int battLevel = 100;            // Charge batterie (valeur par défaut)
const int battSize = 100;       // %
int batt[battSize];             // Array pour faire une moyenne
byte battPos = 0;               // Position du curseur dans le array batt
long battSum = 0;               // Somme de l'array
float battAverage = 0;          // Moyenne de l'array - charge affichée de la batterie
int battErr = 0;                // Compte de fois où la valeur de la charge diffère de plus de 1
int battLevelNintyfivePc = 815; // Équivalent de 95% de charge dans l'algorithme
int battLevelTwentyPc = 785;    // Équivalent de 20% de charge dans l'algorithme
unsigned long battTimer = millis();
unsigned long battDelay = 0;             // Délai entre mesure de la charge, 0 pour remplir vite le array, devient 1000 après

/*--- Manette ---*/
const unsigned int MAX_INPUT = 10;
char chaine[MAX_INPUT];
int signal_verin;
int signal_x;
int signal_y;
int signal_Joystick;
bool manette_en_cours = 0;         // La manette est en train de communiquer avec le contrôleur

/*--- Signal BLE manette ---*/
char Data[100];   // Char buffer pour enregistrer la communication en AT commande
char RAW[3];      // Char array pour lecture de la valeur du signal BLE
int INDEX;        // Curseur pour array RAW
char Value = '-'; // Symbol à détecter dans l'array Data signifiant le début de la valeur du signal BLE

/*--- POIGNÉES ---*/
int pwmg = 0;             // Signal PWM brut provenant de la lecture analogique de la gachette côté batterie
int pwmd = 0;             // Signal PWM brut provenant de la lecture analogique de la gachette côté contrôleur
int pwm_min = 0;          // Signal PWM brut des deux gachettes si le mode Solidaire est sélectionné
int dir;                  // Variable qui definit si les roues motrices vont vers l'avant (HIGH) ou l'arriere (LOW)
int mode;                 // Condition pour choisir le mode d'avance du Zenith -- HIGH = indépendantes et LOW = solidaires
int activ_poignees = LOW; // Condition pour activer les poignees avec l'ecran
int nmode = 1;            // Test pour le mode de contrôle des poignées

/*--- ÉCRAN ---*/
char chr[4] = {0}; // Char buffer pour communication avec l'écran
byte npage = 1;    // Numéro de la page active pour l'écran
int nPagelue = 0;     // Numéro de page lu depuis l'écran

// déclaration des objets Nextion (page_id, component_id, "component_name")
// Page 0 (Calibration)
// NexText nopageDebut = NexText(0, 5, "nopage" );                     // Texte invisible qui indique le numéro de page
// NexButton bOk = NexButton(0, 6, "goInterface");                             // Bouton pour aller à la page principale (interface)
// NexProgressBar calibrationProgressBar = NexProgressBar(0, 1, "progressBar");   // Barre de progression qui affiche le statut de calibration
// NexText tStatut = NexText(0, 3, "tStatut");                         // Texte qui affiche le statut de calibration (en cours ou terminée)

// Page 1 (Interface)
NexText nopageDebut = NexText(0, 16, "nopage");                // Texte invisible qui indique le numéro de page
NexButton next = NexButton(0, 48, "next");                     // Bouton pour aller à la page secondaire (modes)
NexButton bOn = NexButton(0, 22, "bOn");                       // Bouton on pour activer les poignées
NexButton bOff = NexButton(0, 23, "bOff");                     // Bouton off pour désactiver les poignées
NexText bStat = NexText(0, 24, "bStat");                       // Texte qui affiche si les poignées sont activées ou non
NexText poidspatient = NexText(0, 8, "poidspatient");          // Texte qui affiche le poids du patient
NexButton msgpoids = NexButton(0, 15, "msgpoids");             // Message d'erreur sur le poids
NexButton balance = NexButton(0, 14, "balance");               // Bouton pour la pesée automatique par la load cell
NexVariable PoucMaintien = NexVariable(0,35,"PoucMaintien");   // Variable qui indique le pourcentage de maintien désiré
NexButton majMaintien = NexButton(0, 30, "majMaintien");       // Bouton de mise à jour des ressorts de maintien
NexButton msgmaintien = NexButton(0, 12, "msgmaintien");       // Message d'erreur pour les fourchettes des ressorts
NexText poidspercu = NexText(0,26, "poidspercu");              // Texte qui indique le poids perçu par le patient
NexVariable Vitesse = NexVariable(0,46,"Vitesse");             // Variable qui indique la vitesse de déplacement du Zénith
NexButton majVitesse = NexButton(0,38,"majVitesse");           // Bouton de mise à jour de la vitesse du Zénith
NexButton msgvitesse = NexButton(0,49,"msgvitesse");           // Texte pour valider que la vitesse a été mise à jour
NexButton msgbatterie = NexButton(0, 13, "msgbatterie");       // Message d'erreur pour la batterie
NexProgressBar BatteryLevel = NexProgressBar(0, 11, "blevel"); // Barre de progression qui affiche le niveau de la batterie

// Page 2 (Clavier numérique)
NexButton ok = NexButton(1, 4, "ok");  // Bouton pour envoyer le poids inscrit manuellement

// Page 3 (Modes de déplacement (controle direct ou inverse des moteurs)
NexText nopageModes = NexText(2,13,"nopage"); // Texte invisible qui indique le numéro de page
NexButton prev = NexButton(2,15,"prev");      // Bouton pour aller à la page principale
NexButton bMode1 = NexButton(2,5,"bMode1");   // Bouton pour activer le mode 1 de contrôle des poignées
NexButton bMode2 = NexButton(2,6,"bMode2");   // Bouton pour activer le mode 3 de contrôle des poignées
NexButton bMode3 = NexButton(2,7,"bMode3");   // Bouton pour activer le mode 3 de contrôle des poignées
NexButton bMode4 = NexButton(2,8,"bMode4");   // Bouton pour activer le mode 4 de contrôle des poignées
NexText MODE = NexText(2, 4, "MODE");         // Texte qui affiche le mode actuel de contrôle des poignées

// Array des objets Nextion à surveiller
NexTouch *nex_listen_list[] = {
    &majMaintien,
    &balance,
    &ok,
    &bOn,
    &bOff,
    &poidspatient,
    &msgmaintien,
    &msgpoids,
    &msgbatterie,
    &balance,
    &next,
    &poidspercu,
    &PoucMaintien,
    &Vitesse,
    &majVitesse,
    &msgvitesse, 
    &bMode1,
    &bMode2,
    &bMode3,
    &bMode4,
    NULL};

/*------------------------------------------------------------------ PINS -----------------------------------------------------------------*/
/*--- Déplacement ---*/
const byte mg = 3;   // Moteur côté batterie
const byte md = 10;  // Moteur côté contrôleur
const byte dmg = 33; // Direction pour Moteur côté batterie
const byte dmd = 9;  // Direction pour Moteur côté contrôleur
const byte bkg = 35; // Frein électromagnétique du moteur côté batterie
const byte bkd = 8;  // Frein électromagnétique du moteur côté contrôleur

/*--- Levage ---*/
const byte updn = A0;            // Interrupteur monter-descendre pour contrôle manuel du levage
const byte vg = 5;               // Vérin côté batterie
const byte vd = 6;               // Vérin côté contrôleur
const byte vdg = 43;             // Direction Vérin côté batterie
const byte vdd = 46;             // Direction Vérin côté contrôleur
const byte slaveSelectEnc1 = 44; // Pin Slave du quad encoder counter 1
const byte slaveSelectEnc2 = 42; // Pin Slave du quad encoder counter 2
const byte dat = 24;             // Pin data du load cell amp
const byte clk = 22;             // Pin clock du load cell amp
const byte LimitSwitchBD = 29;   // Limit switch bas côté controller
const byte LimitSwitchHD = 31;   // Limit switch haut côté controller
const byte LimitSwitchBG = 40;   // Limit switch bas côté batterie
const byte LimitSwitchHG = 41;   // Limit switch haut côté batterie

/*--- Maintien ---*/
const byte fg = 11;  // Fourchette côté batterie
const byte fd = 12;  // Fourchette côté contrôleur
const byte mag = 45; // Électroaimant pour retenue durant configuration des fourchettes
const byte lmt = 47; // Limit switch pour position angulaire de l'arbre principal

/*--- Batterie ---*/
const byte bat = A1; // Pin pour lire la capacité de la batterie

/*--- Poignées ---*/
const byte potg = A11; // Potentiomètre de la gachette de la poignée côté batterie pour contrôler la vitesse des roues motrices côté batterie
const byte potd = A12; // Potentiomètre de la gachette de la poignée côté contrôleur pour contrôler la vitesse des roues motrices côté contrôleur
const byte modePoignees= 7;   // Switch pour décider le mode d'avance du Zenith à partir des gachettes (roues motrices indépendantes ou solidaires)
const byte sens = 2;  // Bouton pour alterner entre la marche avant ou arrière des roues motrices
const byte ledRouge = 34; // LED qui indique que le Zénith se déplace vers l'arrière
const byte ledJaune = 32; // LED qui indique que le Zénith se déplace vers l'avant
const byte ledBleue = 28; // LED qui indique que les poignées sont en mode indépendantes
const byte ledVerte = 30; // LED qui indique que les poignées sont en mode solidaires

/*-------------------------------------------------------- Prototypes de Fonctions --------------------------------------------------------*/
/*--- DÉPLACEMENT ---*/
void riseBreaks();     // Désactiver les freins des roues motrices
void Break();  // Actionner les freins des roues motrices
void asserMoteurs();
void motorDriveCalc(); // Définit les PWMs finaux envoyés aux roues motrices selon les commandes de la manette ou des poignées
void machine_stop();   // Arrête le Zenith
void acquisitionEncoderL();
void acquisitionEncoderR();
void encoderSpeedCalc();
void MAJ_PWM();

/*--- INTERNAL MESUREMENTS UNIT ---*/
float lectureAngle();
void envoieValeurs();
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);

/*--- LEVAGE ---*/
void peserPatient();    // Pesée automatique du patient par la load cell
void asserVerins();
void consigneVerins();
void verinDriveCalc();
void setPIDValues();
void PIDScrollValues();
void verinManuel();     // Commande des verins de levage avec l'interrupteur noir du côté contrôleur du Zenith
void LoadCellCalib();   // Calibration de la load cell

/*--- MAINTIEN ---*/
void springCalc(); // Calcul des combinaisons de ressorts pour trouver la combinaison optimale pour le poids à compenser
void calcPoids();
void fourchetteCongif();
void lmtWait(int lap);
void setFourchette();

/*--- BATTERIE ---*/
void checkBatt();

/*--- MANETTE ---*/
void process_data(String data); // Génère les PWMs des moteurs selon les inputs de la manette
void processIncomingByte();     // Fonction appelée dans le Loop pour les moteurs
void serialFlush();
void checkSignal();
void receiveEvent(int howMany);

/*--- POIGNÉES ---*/
void ContrlPoignees(); // Fonction qui définit PWMD et PWMG selon les commandes des poignées
int smoothing(const byte analogPin);
void closeLED();

/*--- ÉCRAN ---*/
void tracePoids();
void checkpage();

//Fonction quand un bouton est appuyé sur l'écran
void nextPopCallback(void *ptr);
void prevPopCallback(void *ptr);
void majMaintienPopCallback(void *ptr);
void majVitessePopCallback(void *ptr);
void okPopCallback(void *ptr);
void balancePopCallback(void *ptr);
void bOnPopCallback(void *ptr);
void bOffPopCallback(void *ptr);
void bMode1PopCallback(void*ptr);
void bMode2PopCallback(void*ptr);
void bMode3PopCallback(void*ptr);
void bMode4PopCallback(void*ptr);