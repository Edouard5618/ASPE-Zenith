/**
 *@brief Fichier d'en-tête pour le main.cpp. Ce fichier contient des variables et les prototypes de fonctions.
 *@file main.h
 *@date octobre 2022
 **/

/*--------------------------------------------------------------- Libraires ---------------------------------------------------------------*/
#include <EEPROM.h>  // Librairie pour utiliser la mémoire EEPROM de la carte Arduino (https://www.arduino.cc/en/Reference/EEPROM)
#include <Nextion.h> // Voir le lien suivant: https://randomnerdtutorials.com/nextion-display-with-arduino-getting-started/ qui explique
                     // bien la librairie Nextion.h pour l'écran tactile
#include <SPI.h>     //(https://www.arduino.cc/en/reference/SPI)
#include <HX711.h>   // Librairie pour lire le load cell du Zenith. Load cell permet de mesurer le poids du patient
                     // lorsqu'il est complètement soulevé (ne touche plus au sol). HX711 est un amplificateur de signal pour la load cell
                     // (pont de Wheatstone). Documentation: https://www.robotshop.com/ca/en/hx711-load-cell-amplifier.html
#include <Arduino.h> // Librairie pour l'utilisation de la carte Arduino
#include <Wire.h>    //Libairie pour l'utilisation de I2C
#include <Servo.h>   //Libraire pour l'utilisation du servomoteur pour la calibration du IMU (BNO055)
#include <string.h>
#include <encoder.h> //Librairie pour l'utilisation des encodeurs

/*------------------------------------------------------------------ PINS -----------------------------------------------------------------*/
/*--- ENCODEURS DE POSITION DES VÉRINS ---*/
const byte EVGA = 2;
const byte EVGB = 53;
const byte EVDA = 3;
const byte EVDB = 4;

/*--- ENCODEURS AUX ROUES ---*/
const byte ERGA = 19;
const byte ERGB = 15;
const byte ERDA = 18;
const byte ERDB = 14;

/*--- Déplacement ---*/
const byte mg = 7;   // Moteur côté batterie
const byte md = 10;  // Moteur côté contrôleur
const byte dmg = 35; // Direction pour Moteur côté batterie
const byte dmd = 9;  // Direction pour Moteur côté contrôleur
const byte bkg = 33; // Frein électromagnétique du moteur côté batterie
const byte bkd = 8;  // Frein électromagnétique du moteur côté contrôleur

/*--- Levage ---*/
const byte updn = A0;  // Interrupteur monter-descendre pour contrôle manuel du levage
const byte vg = 5;     // Vérin côté batterie
const byte vd = 6;     // Vérin côté contrôleur
const byte dvg = 43;   // Direction Vérin côté batterie
const byte dvd = 46;   // Direction Vérin côté contrôleur
const byte dat = 24;   // Pin data du load cell amp
const byte clk = 22;   // Pin clock du load cell amp
const byte LMTHG = 41; // Limit switch haut côté batterie
const byte LMTBG = 40; // Limit switch bas côté batterie
const byte LMTHD = 25; // Limit switch haut côté controller
const byte LMTBD = 23; // Limit switch bas côté controller

/*--- Maintien ---*/
const byte fg = 12;  // Fourchette côté batterie
const byte fd = 11;  // Fourchette côté contrôleur
const byte mag = 45; // Électroaimant pour retenue durant configuration des fourchettes
const byte lmt = 47; // Limit switch pour position angulaire de l'arbre principal

/*--- Batterie ---*/
const byte bat = A1; // Pin pour lire la capacité de la batterie

/*--- Poignées ---*/
const byte potg = A12; // Potentiomètre de la gachette de la poignée côté batterie pour contrôler la vitesse des roues motrices côté batterie
const byte potd = A11; // Potentiomètre de la gachette de la poignée côté contrôleur pour contrôler la vitesse des roues motrices côté contrôleur
const byte Mode = 44;  // Switch pour décider le mode d'avance du Zenith à partir des gachettes (roues motrices indépendantes ou solidaires)
const byte sens = 48;  // Bouton pour alterner entre la marche avant ou arrière des roues motrices
const byte LEDR = 28;  // LED rouge qui indique que le Zénith se déplace vers l'arrière
const byte LEDJ = 30;  // LED jaune qui indique que le Zénith se déplace vers l'avant
const byte LEDB = 34;  // LED bleue qui indique que les poignées sont en mode indépendantes
const byte LEDV = 32;  // LED verte qui indique que les poignées sont en mode solidaires

/*---------------------------------------------------------- Variables et objets ----------------------------------------------------------*/
/*--- ENCODEURS DE POSITION DES VÉRINS ---*/
Encoder EncodeurVerinGauche(EVGA, EVGB);
Encoder EncodeurVerinDroit(EVDA, EVDB);

/*--- ENCODEURS AUX ROUES ---*/
#define phaseEncodeurRoue 2400 // Nombre de pulse par tour
#define M_S_TO_KM_H 3.6        // Vitesse mètre par seconde en kilomètres par heure
#define rayonRoue 0.1778       // En mètre
Encoder EncodeurRoueGauche(ERGA, ERGB);
Encoder EncodeurRoueDroite(ERDA, ERDB);
float vitesseDroiteReelle = 0;  // Vitesse mesurée par l'encodeur
float vitesseGaucheReelle = 0;  // Vitesse mesurée par l'encodeur
float vitesseDesireeDroite = 0; // Vitesse désirée par la manette ou les poignées
float vitesseDesireeGauche = 0; // Vitesse désirée par la manette ou les poignées
unsigned long tempsAcquisitionDroit = 0;
unsigned long ancienTempsAcquisitionDroit = 0;
unsigned long tempsAcquisitionGauche = 0;
unsigned long ancienTempsAcquisitionGauche = 0;
int32_t ancienPulseDroit = 0;
int32_t ancienPulseGauche = 0;
int32_t pulseParcouruDroit = 0;
int32_t pulseParcouruGauche = 0;

/*--- EEPROM ---*/
int eepromPoidsOffsetAddr = 0;
int eepromCorrSecuriteVitesseAddr = 1;
int eepromCorrDeviationAddr = 2;
int eepromCorrAccelerationAddr = 3;

/*--- DÉPLACEMENT ---*/
uint32_t CorrSecuriteVitesse = 100; // Correction de sécurité de vitesse
uint32_t CorrDeviation = 100;       // Correction de déviation
uint32_t CorrAcceleration = 100;    // Correction d'acceleration
float PWM_MAX = 0;                  // PWM maximum pour les moteurs
float PWM_MAX_AJUST = 1;            // PWM maximum ajusté pour les moteurs
float ACCELMAX = 5;                 // Acceleration ou déceleration maximale
#define jerkTime 55                 // Délai entre chaque variation du PWM pour limiter le jerk
#define CorrectionJog 0.90          // Valeur appliquée au moteur droit pour que le Zénith avance droit
uint32_t speedcoef = 0;             // Vitesse de déplacement, variant entre 0 et 10 (valeur par défaut)
float coefVitesse = 0;              // Coefficient de vitesse, variant entre 0 et 10 (valeur par défaut)
int JogX;                           // Axe X issue du joystick de la manette
int JogY;                           // Axe Y issue du joystick de la manette
float pwmG_Value = 0;               // Valeur PWM moteur côté batterie déduite de JoxX et JogY
float pwmD_Value = 0;               // Valeur PWM moteur côté contrôleur déduite de JoxX et JogY
float PWMG = 0;                     // PWM visé pour moteur côté batterie
float PWMD = 0;                     // PWM visé pour moteur côté contrôleur
int PWMG_reel = 0;                  // PWM réellement envoyé au moteur côté batterie
int PWMD_reel = 0;                  // PWM réellement envoyé au moteur côté contrôleur
int diff_PWMG = 0;                  // Coefficient de variation du PWM pour limiter le jerk du côté batterie
int diff_PWMD = 0;                  // Coefficient de variation du PWM pour limiter le jerk du côté contrôleur
unsigned long jerkTimer = millis(); // Timer entre chaque variation des PWM
bool wheelstopped = 1;              // Appareil immobilisé ?

/*--- LEVAGE ---*/
#define liftCoef 255     // Vitesse de levage maximale, sur 255
#define ACCELMAXVERIN 35 // Acceleration ou déceleration maximale des vérins
int PWMVG = 0;           // PWM Vérin côté batterie
int PWMVD = 0;           // PWM Vérin côté contrôleur
int PWM_VG_reel = 0;     // PWM réellement envoyé au Vérin côté batterie
int PWM_VD_reel = 0;     // PWM réellement envoyé au Vérin côté contrôleur
int diff_PWM_VG = 0;     // Coefficient de variation du PWM pour limiter le jerk du côté batterie
int diff_PWM_VD = 0;     // Coefficient de variation du PWM pour limiter le jerk du côté contrôleur
bool btnreleased = 0;    // Bouton commande manuelle du levage relâchée ?
int lmtTrigg = 0;        // Nombre de détection de fin de course consécutives des vérins
int referenceBouton = 0; // Valeur de référence

/*Asservissement PID Verin*/
#define eMin 25             // Erreur minimale tenue en compte (dead zone)
#define rateErrorMax 10     // Valeur maxi de dérivée du PID
#define outputMax 30        // Valeur maxi de output (anti windup)
#define kMax 5              // Valeur maxi des boutons scroll d'ajustement des coefficient du PID, sur écran
#define kp 0.10             // Coefficient proportionnel
#define ki 0.0              // Coefficient intégral
#define kd 0.0              // Coefficient dérivée
int e = 0;                  // Erreur asservissement vérin (écart entre les compteurs effet Hall gauche vs droit)
double lastError;           // Buffer pour calcul de la dérivée
double cumError, rateError; // Intégrale et dérivée du PID
double output;              // Écart appliqué entre les PWMs des vérins gauche et droit

/*Asservissement PID Moteur*/
#define eMinM 0.02                 // Erreur minimale tenue en compte (dead zone)
#define rateErrorMaxM 10           // Valeur maxi de dérivée du PID
#define kpM 1.9                    // Coefficient proportionnel
#define kiM 0.0                    // Coefficient intégral
#define kdM 1.0                    // Coefficient dérivée
#define outputMaxM 6               // Valeur maxi de output (anti windup)
float eM = 0;                      // Erreur asservissement vérin (écart entre les compteurs effet Hall gauche vs droit)
double lastErrorM_R, lastErrorM_L; // Buffer pour calcul de la dérivée
double cumErrorM_R, rateErrorM_R;  // Intégrale et dérivée du PID
double cumErrorM_L, rateErrorM_L;  // Intégrale et dérivée du PID
double outputM;                    // Écart appliqué entre les PWMs des vérins gauche et droit

/*--- MAINTIEN ---*/
uint32_t pcmaintien = 50; // Pourcentage de maintien entre 0 et 100 (valeur par défaut)
float POIDS = 0;          // Valeur pesée, sans offset d'appliqué
float poids;              // Poids soutenu visé
int poidsOffset;          // Offset appliqué au poids, selon calibration ou valeur enregistére en EEPROM
int percu;                // Poids perçu par l'utilisateur
HX711 scale;              // Objet pour amplificateur cellule de charge

/*--- Ressorts ---*/
#define R917 6  // Poids soutenu équivalent (kg)
#define R919 2  // Poids soutenu équivalent (kg)
#define R921 11 // Poids soutenu équivalent (kg)
#define R925 20 // Poids soutenu équivalent (kg)
#define R928 32 // Poids soutenu équivalent (kg)
#define R929 50 // Poids soutenu équivalent (kg)
int rcomb[64];  // Array pour poids de toutes les combinaisons de ressort

/*--- Fourchettes ---*/
int Rconfig[8] = {127, 140, 158, 175, 190, 206, 220, 240}; // Positions fourchette côté contrôleur
int rconfig[8] = {127, 140, 150, 163, 176, 188, 200, 220}; // Positions fourchette côté batterie
int Rkg[8];                                                // Arrays pour poids soutenu à chaque configuration fourchette côté contrôleur
int rkg[8];                                                // Arrays pour poids soutenu à chaque configuration fourchette côté batterie
int PWMFG = 0;                                             // PWM Fourchette côté contrôleur (PWMFC)
int PWMFD = 0;                                             // PWM Fourchette côté batterie (PWMFB)

/*--- BATTERIE ---*/
int battLevel = 100;                // Charge batterie (valeur par défaut)
const int battSize = 100;           // %
int batt[battSize];                 // Array pour faire une moyenne
byte battPos = 0;                   // Position du curseur dans le array batt
long battSum = 0;                   // Somme de l'array
float battAverage = 0;              // Moyenne de l'array - charge affichée de la batterie
int battErr = 0;                    // Compte de fois où la valeur de la charge diffère de plus de 1
int battLevelNintyfivePc = 815;     // Équivalent de 95% de charge dans l'algorithme
int battLevelTwentyPc = 785;        // Équivalent de 20% de charge dans l'algorithme
unsigned long battTimer = millis(); // Timer pour limiter la vitesse de rafraichissement
unsigned long battDelay = 0;        // Délai entre mesure de la charge, 0 pour remplir vite le array, devient 1000 après

/*--- Manette ---*/
#define TempsSansCommMax 500
int signal_verin;
int signal_x;
int signal_y;
int signal_Joystick;
bool manette_en_cours = 0; // La manette est en train de communiquer avec le contrôleur
unsigned long DerniereComm = 0;

/*--- Signal BLE manette ---*/
#define IDattendu "2B1C" // ID de la manette
char Data[100];          // Char buffer pour enregistrer la communication en AT commande

/*--- POIGNÉES ---*/
int pwmg = 0;             // Signal PWM brut provenant de la lecture analogique de la gachette côté batterie
int pwmd = 0;             // Signal PWM brut provenant de la lecture analogique de la gachette côté contrôleur
int pwm_min = 0;          // Signal PWM brut des deux gachettes si le mode Solidaire est sélectionné
bool dir;                 // Variable qui definit si les roues motrices vont vers l'avant (HIGH) ou l'arriere (LOW)
int mode;                 // Condition pour choisir le mode d'avance du Zenith -- HIGH = indépendantes et LOW = solidaires
int activ_poignees = LOW; // Condition pour activer les poignees avec l'ecran
int nmode = 1;            // Test pour le mode de contrôle des poignées

/*--- ÉCRAN ---*/
char chr[4] = {0}; // Char buffer pour communication avec l'écran
byte npage = 1;    // Numéro de la page active pour l'écran
int nPagelue = 0;  // Numéro de page lu depuis l'écran
unsigned long timerMsgMaintien;
unsigned long timerMsgVitesse;
unsigned long timerMsgPoids;
bool boolMsgMaintien;
bool boolMsgVitesse;
bool boolMsgPoids;
bool PremierChangementP2 = true;
bool PremierChangementP3 = true;
#define TempsMaxMsgMaintien 10000
#define TempsMaxMsgVitesse 10000
#define TempsMaxMsgPoids 10000

// déclaration des objets Nextion (page_id, component_id, "component_name")
// Page 0 (Calibration)
// NexText nopageDebut = NexText(0, 5, "nopage" );                     // Texte invisible qui indique le numéro de page
// NexButton bOk = NexButton(0, 6, "goInterface");                             // Bouton pour aller à la page principale (interface)
// NexProgressBar calibrationProgressBar = NexProgressBar(0, 1, "progressBar");   // Barre de progression qui affiche le statut de calibration
// NexText tStatut = NexText(0, 3, "tStatut");                         // Texte qui affiche le statut de calibration (en cours ou terminée)

// Page 1 (Interface)
NexText nopageDebut = NexText(0, 12, "nopage");                // Texte invisible qui indique le numéro de page
NexButton next = NexButton(0, 43, "next");                     // Bouton pour aller à la page secondaire (modes)
NexButton bOn = NexButton(0, 17, "bOn");                       // Bouton on pour activer les poignées
NexButton bOff = NexButton(0, 18, "bOff");                     // Bouton off pour désactiver les poignées
NexText bStat = NexText(0, 19, "bStat");                       // Texte qui affiche si les poignées sont activées ou non
NexText poidspatient = NexText(0, 4, "poidspatient");          // Texte qui affiche le poids du patient
NexButton msgpoids = NexButton(0, 11, "msgpoids");             // Message d'erreur sur le poids
NexButton balance = NexButton(0, 10, "balance");               // Bouton pour la pesée automatique par la load cell
NexVariable PoucMaintien = NexVariable(0, 30, "PoucMaintien"); // Variable qui indique le pourcentage de maintien désiré
NexButton majMaintien = NexButton(0, 25, "majMaintien");       // Bouton de mise à jour des ressorts de maintien
NexButton msgmaintien = NexButton(0, 8, "msgmaintien");        // Message d'erreur pour les fourchettes des ressorts
NexText poidspercu = NexText(0, 21, "poidspercu");             // Texte qui indique le poids perçu par le patient
NexVariable Vitesse = NexVariable(0, 41, "Vitesse");           // Variable qui indique la vitesse de déplacement du Zénith
NexButton majVitesse = NexButton(0, 33, "majVitesse");         // Bouton de mise à jour de la vitesse du Zénith
NexButton msgvitesse = NexButton(0, 44, "msgvitesse");         // Texte pour valider que la vitesse a été mise à jour
NexButton msgErreur = NexButton(0, 45, "msgErreur");
NexButton msgDisManette = NexButton(0, 46, "msgDisManette");
NexButton msgbatterie = NexButton(0, 9, "msgbatterie"); // Message d'erreur pour la batterie
NexPicture ImgNext = NexPicture(0, 14, "ImgNext");      // Image pour aller à la page secondaire (modes)

NexProgressBar BatteryLevel = NexProgressBar(0, 7, "blevel"); // Barre de progression qui affiche le niveau de la batterie

// Page 2 (Clavier numérique)
NexButton ok = NexButton(1, 4, "ok"); // Bouton pour envoyer le poids inscrit manuellement

// Page 3 (Modes de déplacement (controle direct ou inverse des moteurs)
NexText nopageModes = NexText(2, 13, "nopage");     // Texte invisible qui indique le numéro de page
NexButton prev = NexButton(2, 15, "prev");          // Bouton pour aller à la page principale
NexPicture ImgPrev = NexPicture(2, 14, "ImgPrev");  // Image pour aller à la page principale
NexButton next2 = NexButton(2, 16, "next2");        // Bouton pour aller à la page secondaire (Réglages)
NexPicture ImgNext2 = NexPicture(2, 17, "ImgNext"); // Image pour aller à la page secondaire (Réglages)
NexButton bMode1 = NexButton(2, 5, "bMode1");       // Bouton pour activer le mode 1 de contrôle des poignées
NexButton bMode2 = NexButton(2, 6, "bMode2");       // Bouton pour activer le mode 3 de contrôle des poignées
NexButton bMode3 = NexButton(2, 7, "bMode3");       // Bouton pour activer le mode 3 de contrôle des poignées
NexButton bMode4 = NexButton(2, 8, "bMode4");       // Bouton pour activer le mode 4 de contrôle des poignées
NexText MODE = NexText(2, 4, "MODE");               // Texte qui affiche le mode actuel de contrôle des poignées

// Page 4 (Réglages avancés)
NexText nopageAvance = NexText(3, 25, "nopageAvance"); // Texte invisible qui indique le numéro de page
NexButton prev2 = NexButton(3, 16, "prev2");           // Bouton pour aller à la page principale
NexPicture ImgPrev2 = NexPicture(3, 17, "ImgPrev");    // Image pour aller à la page principale
NexSlider SliderVit = NexSlider(3, 5, "SliderVit");    // Slider pour régler la vitesse du Zénith
NexSlider SliderDev = NexSlider(3, 7, "SliderDev");    // Slider pour régler la déviation du Zénith
NexSlider SliderAcc = NexSlider(3, 6, "SliderAcc");    // Slider pour régler l'acceleration du Zénith
NexVariable Vit = NexVariable(3, 26, "Vit");           // Variable qui indique la vitesse de déplacement du Zénith
NexVariable Dev = NexVariable(3, 28, "Dev");           // Variable qui indique la déviation de déplacement du Zénith
NexVariable Acc = NexVariable(3, 27, "Acc");           // Variable qui indique l'acceleration de déplacement du Zénith
NexNumber ValVit = NexNumber(3, 12, "ValVit");         // Texte qui indique la vitesse de déplacement du Zénith
NexNumber ValDev = NexNumber(3, 13, "ValDev");         // Texte qui indique la déviation de déplacement du Zénith
NexNumber ValAcc = NexNumber(3, 14, "ValAcc");         // Texte qui indique l'acceleration de déplacement du Zénith
NexPicture pVit = NexPicture(3, 2, "pVit");            // Image qui indique la vitesse de déplacement du Zénith
NexPicture pDev = NexPicture(3, 3, "pDev");            // Image qui indique la déviation de déplacement du Zénith
NexPicture pAcc = NexPicture(3, 4, "pAcc");            // Image qui indique l'acceleration de déplacement du Zénith

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
    &next2,
    &poidspercu,
    &PoucMaintien,
    &Vitesse,
    &majVitesse,
    &msgvitesse,
    &msgErreur,
    &msgDisManette,
    &bMode1,
    &bMode2,
    &bMode3,
    &bMode4,
    &prev,
    &prev2,
    &ImgNext,
    &ImgNext2,
    &ImgPrev,
    &ImgPrev2,
    &SliderVit,
    &SliderDev,
    &SliderAcc,
    &pVit,
    &pDev,
    &pAcc,
    NULL};

/*-------------------------------------------------------- Prototypes de Fonctions --------------------------------------------------------*/
/*--- DÉPLACEMENT ---*/
void relacherFrein();   // Désactiver les freins des roues motrices
void Frein();           // Actionner les freins des roues motrices
void asserMoteurs();    // Asservissement des moteurs des roues
void accelMoteurs();    // Définit les PWMs finaux envoyés aux roues motrices selon les commandes de la manette ou des poignées
void machine_stop();    // Arrête le Zenith
void vitesseEncodeur(); // Calculs de vitesse du Zénith
void MAJ_PWM();         // Met à jour tous les PWM

/*--- LEVAGE ---*/
void peserPatient();   // Pesée automatique du patient par la load cell
void asserVerins();    // Asservissement des vérins
void consigneVerins(); // Vérification des limit-switch et de l'asservissement
void accelVerin();     // Acceleration des vérins
void verinManuel();    // Commande des verins de levage avec l'interrupteur noir du côté contrôleur du Zenith
void LoadCellCalib();  // Calibration de la load cell

/*--- MAINTIEN ---*/
void springCalc();       // Calcul des combinaisons de ressorts pour trouver la combinaison optimale pour le poids à compenser
void calcPoids();        // Calculer poids soutenu visé
void fourchetteCongif(); // Calculer la position nécessaire des fourchettes
void lmtWait(int lap);   // Afficher un message à l'écran si les fourchettes sont obstruées
void setFourchette();    // Placer les fourchettes à la bonne position

/*--- BATTERIE ---*/
void checkBatt(); // Vérification du niveau de la batterie

/*--- MANETTE ---*/
void traitementDonnesManette(String data); // Génère les PWMs des moteurs selon les inputs de la manette
void CommManette();                        // Fonction appelée dans le Loop pour communiquer avec les moteurs
void receiveEvent(int howMany);            // Fonction appelée lorsqu'une donnée est reçue par le bus I2C
void SecuriteManette();

/*--- POIGNÉES ---*/
void ContrlPoignees(); // Fonction qui définit PWMD et PWMG selon les commandes des poignées
void fermerLED();      // Fermer toutes les LED

/*--- ÉCRAN ---*/
void checkMsg();
void checkpage();
void nextPopCallback(void *ptr);
void next2PopCallback(void *ptr);
void prevPopCallback(void *ptr);
void prev2PopCallback(void *ptr);
void majMaintienPopCallback(void *ptr);
void majVitessePopCallback(void *ptr);
void okPopCallback(void *ptr);
void balancePopCallback(void *ptr);
void bOnPopCallback(void *ptr);
void bOffPopCallback(void *ptr);
void bMode1PopCallback(void *ptr);
void bMode2PopCallback(void *ptr);
void bMode3PopCallback(void *ptr);
void bMode4PopCallback(void *ptr);
void SliderVitPopCallback(void *ptr);
void SliderDevPopCallback(void *ptr);
void SliderAccPopCallback(void *ptr);
void pVitPopCallback(void *ptr);
void pDevPopCallback(void *ptr);
void pAccPopCallback(void *ptr);