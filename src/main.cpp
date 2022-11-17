/* Code principal du Zenith, téléchargé sur le Arduino Mega. Comprend les fonctions qui prennent en charge tout les sous-systèmes de la machine,
 * sauf pour le GUI de l'écran en temps que tel. Pour modifier le GUI de l'écran, se référer au programme Nextion "GUI écran Zénith".
 */
#include "main.h"

//-------------------------------------------------------------------------------------------------------------
// DEBUT SETUP ET LOOP
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup()
{
  // Initialisation des ports série
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial1.begin(115200);
  Serial2.begin(115200);
  SPI.begin();
  Wire.begin(4);
  Wire.onReceive(receiveEvent);

  // //EEPROM
  poidsOffset = 0; // EEPROM.read(eepromPoidsOffsetAddr) - 128; // Lecture EEPROM
  CorrDeviation = EEPROM.read(eepromCorrDeviationAddr); // Lecture EEPROM
  CorrAcceleration = EEPROM.read(eepromCorrAccelerationAddr); // Lecture EEPROM

  // // Augmentation de la frequence PWM pour les moteurs et les verins
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;

  // // pinMode
  pinMode(mg, OUTPUT);
  pinMode(md, OUTPUT);
  pinMode(bkd, OUTPUT);
  pinMode(bkg, OUTPUT);
  pinMode(vg, OUTPUT);
  pinMode(vd, OUTPUT);
  pinMode(dvd, OUTPUT);
  pinMode(dvg, OUTPUT);
  pinMode(dmd, OUTPUT);
  pinMode(dmg, OUTPUT);
  pinMode(mag, OUTPUT);
  pinMode(updn, INPUT);
  pinMode(LMTBD, INPUT_PULLUP);
  pinMode(LMTBG, INPUT_PULLUP);
  pinMode(LMTHD, INPUT_PULLUP);
  pinMode(LMTHG, INPUT_PULLUP);
  pinMode(lmt, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sens, INPUT_PULLUP);
  pinMode(Mode, INPUT_PULLUP);
  pinMode(potd, INPUT_PULLUP);
  pinMode(potg, INPUT_PULLUP);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDJ, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDV, OUTPUT);

  // // Établit la liste des items Nextion à surveiller (items qui peuvent être appuyés) pour l'écran
  majMaintien.attachPop(majMaintienPopCallback, &majMaintien);
  majVitesse.attachPop(majVitessePopCallback, &majVitesse);
  ok.attachPop(okPopCallback, &ok);
  balance.attachPop(balancePopCallback, &balance);
  next.attachPop(nextPopCallback);
  ImgNext.attachPop(nextPopCallback);
  next2.attachPop(next2PopCallback);
  ImgNext2.attachPop(next2PopCallback);
  prev.attachPop(prevPopCallback);
  ImgPrev.attachPop(prevPopCallback);
  prev2.attachPop(prev2PopCallback);
  ImgPrev2.attachPop(prev2PopCallback);
  bOn.attachPop(bOnPopCallback, &bOn);
  bOff.attachPop(bOffPopCallback, &bOff);
  bMode1.attachPop(bMode1PopCallback, &bMode1);
  bMode2.attachPop(bMode2PopCallback, &bMode2);
  bMode3.attachPop(bMode3PopCallback, &bMode3);
  bMode4.attachPop(bMode4PopCallback, &bMode4);
  SliderDev.attachPop(SliderDevPopCallback, &SliderDev);
  SliderAcc.attachPop(SliderAccPopCallback, &SliderAcc);
  HeurePlus.attachPop(HeurePlusPopCallback, &HeurePlus);
  HeureMoins.attachPop(HeureMoinsPopCallback, &HeureMoins);
  MinutePlus.attachPop(MinutePlusPopCallback, &MinutePlus);
  MinuteMoins.attachPop(MinuteMoinsPopCallback, &MinuteMoins);
  bStat.setText("Statut: off");
  nexInit(); // Initialisation de la librairie de l'Écran
  
  // // Initialisation de l'horloge
  rtc.begin();
  now = rtc.now();
  delay(100);
  Minute.setValue(now.minute());
  Heure.setValue(now.hour());

  // // Initialisation du compteur des encodeurs quad, vérins
  EncodeurVerinGauche.readAndReset(); // Clear Encodeur
  EncodeurVerinDroit.readAndReset();  // Clear Encodeur
  EncodeurRoueDroite.readAndReset();
  EncodeurRoueGauche.readAndReset();

  // // Initialisation de la balance
  scale.begin(dat, clk);    // Pins pour le Output des donnees et la Clock.
  scale.set_scale(5000); // Scale trouver apres la calibration des donnees
  scale.tare();             // Mise à zéro de la pesée
  scale.power_down();
  springCalc(); // Calcul des combinaisons de ressorts

  Serial.println("Start"); // Checkup série

} // fin setup
void loop()
{
  nexLoop(nex_listen_list); // Écoute les items Nextion

  /**** Mise à jour des PWM ****/
  if ((millis() - jerkTimer) > jerkTime)
  { // limitation de la fréquence de m-à-j des PWMs
    checkBatt();              //Vérification de l'état de la batterie
    vitesseEncodeur();        // Vérifier la vitesse du Zénith
    asserMoteurs();           // Asservissement des moteurs des roues
    accelMoteurs();           // Accélération des moteurs des roues
    verinManuel();            // Vérifier le signal du bouton manuel du vérin
    consigneVerins();         // Calculer la consigne des vérins
    accelVerin();             // Accélération des vérins
    CommManette();            // Vérifier si la manette est en communication  
    MAJ_PWM();                // M-à-j des PWMs
    fermerLED();              // Fermer les LED si les poignées ne sont pas activées sur l'écran.
    majHorloge();             // Mise à jour de l'horloge

    

    /*Serial.print("  PWMG_Reel: ");
    Serial.print(PWMG_reel);
    Serial.print("  PWM_MAX: ");
    Serial.print(PWM_MAX);
    Serial.print("  PWMG: ");
    Serial.print(PWMG);
    Serial.print("  Vitesse desiree gauche: ");
    Serial.print(vitesseDesireeGauche);
    Serial.print("  Vitesse relle gauche: ");
    Serial.print(vitesseReelleGauche);  
    Serial.print("  Vitesse desiree Droite: ");
    Serial.print(vitesseDesireeDroite);
    Serial.print("  Vitesse relle Droite: ");
    Serial.println(vitesseReelleDroite);*/
    



    jerkTimer = millis();     // Reset du timer
  }
  checkMsg();         // Enlever les messages superflus à l'écran après 10s d'apparition
  SecuriteManette();  // MachineStop si la manette se déconnecte pendant une communication
} // fin loop

//-------------------------------------------------------------------------------------------------------------
// DEBUT DES FONCTIONS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*--- DÉPLACEMENT ---*/
void relacherFrein()
{
  wheelstopped = 0;
  digitalWrite(bkd, HIGH);
  digitalWrite(bkg, HIGH);
}
void Frein()
{
  machine_stop();
  wheelstopped = 1;
  digitalWrite(bkd, LOW);
  digitalWrite(bkg, LOW);
}
void asserMoteurs()
{
  //*** Vérification des consignes de vitesse ***//
    if (signal_verin) // Si la manette est en communication
    {
      vitesseDesireeDroite = -coefVitesse * pwmD_Value / 255.0;
      vitesseDesireeGauche = -coefVitesse * pwmG_Value / 255.0;
    }
    else if (activ_poignees == HIGH) // Sinon si les poignées sont activées
    {
      ContrlPoignees();
    }
    else // Sinon tout éteindre
    {
      vitesseDesireeDroite = 0;
      vitesseDesireeGauche = 0;
      PWMG = 0;
      PWMD = 0;
      return;
    }

  //*** PID pour côté droit ***//
    eM = -vitesseDesireeDroite * (CorrectionJog*(float)CorrDeviation/100) + vitesseReelleDroite;
    if (abs(eM) >= eMinM)
    {
      rateErrorM_R = (eM - lastErrorM_R); // Dérivée
      LimiterDouble(&rateErrorM_R, rateErrorMaxM);

      outputM = kpM * eM + kdM * rateErrorM_R; // PID output
      LimiterDouble(&outputM, outputMaxM);
      PWMD += outputM ;
      lastErrorM_R = eM; // Remember current error

      if (abs(vitesseDesireeDroite) < 0.05)  //Ajouter une deadzone
        PWMD = 0;
      if (vitesseDesireeDroite > 0.1 && PWMD > -PWM_MIN && PWMD < 0) //Améliorer l'acceleration à basse vitesse
        PWMD = -PWM_MIN;
      else if (vitesseDesireeDroite < -0.1 && PWMD < PWM_MIN && PWMD > 0)
        PWMD = PWM_MIN;
    }
    else
    {
      rateErrorM_R = 0;
      outputM = 0;
    }

  //*** PID pour côté gauche ***//
    eM = -vitesseDesireeGauche + vitesseReelleGauche;
    if (abs(eM) >= eMinM)
    {
      rateErrorM_L = (eM - lastErrorM_L); // Dérivée
      LimiterDouble(&rateErrorM_L, rateErrorMaxM);

      outputM = kpM * eM + kdM * rateErrorM_L; // PID output
      LimiterDouble(&outputM, outputMaxM);
      PWMG += outputM;
      lastErrorM_L = eM; // Remember current error

      if (abs(vitesseDesireeGauche) < 0.05) //Ajouter une deadzone
        PWMG = 0;
      if (vitesseDesireeGauche > 0.1 && PWMG > -10 && PWMG < 0) //Améliorer l'acceleration à basse vitesse
        PWMG = -10;
      else if (vitesseDesireeGauche < -0.1 && PWMG < 10 && PWMG > 0)
        PWMG = 10;
    }
    else
    {
      rateErrorM_L = 0;
      outputM = 0;
    }
} // Fin asserMoteurs
void accelMoteurs()
{
  if (activ_poignees == HIGH)
  {
    mode = abs(digitalRead(Mode));
    dir = abs(1 - digitalRead(sens));
  }
  // Moteur côté controlleur  ------------------------------------------------------
  // Plage d'acceleration du moteur
  diff_PWMD = (PWMD - PWMD_reel);
  LimiterInt(&diff_PWMD, ACCELMAX);
  PWMD_reel += diff_PWMD;

  // Décision logique pour la direction des roues motrices
  if (PWMD_reel < 0)
    digitalWrite(dmd, LOW);
  if (PWMD_reel > 0)
    digitalWrite(dmd, HIGH);

  // Moteur côté batterie ------------------------------------------------------
  // Plage d'acceleration du moteur côté batterie
  diff_PWMG = (PWMG - PWMG_reel);
  LimiterInt(&diff_PWMG, ACCELMAX);
  PWMG_reel += diff_PWMG;

  // Décision logique pour la direction des roues motrices
  if (PWMG_reel < 0)
    digitalWrite(dmg, LOW);
  if (PWMG_reel > 0)
    digitalWrite(dmg, HIGH);

  // Frein
  if ((!wheelstopped && !activ_poignees && !manette_en_cours && millis() - DerniereComm > 1250) || (!wheelstopped && signal_Joystick)) // Actionne les freins des roues motrices si celles-ci ne sont pas déjà arrêtées et qu'il n'y a plus de commandes provenant de la manette ET des poignées.
    Frein();
  else if ((wheelstopped && (activ_poignees || manette_en_cours)) && !signal_Joystick) // Désactive les freins des roues motrices si les roues sont déjà arrêtées et que le Arduino reçoit des commandes provenant des poignées OU de la manette.
    relacherFrein();
} // fin accelMoteurs
void machine_stop()
{
  PWMD = 0;
  PWMG = 0;
  PWMD_reel = 0;
  PWMG_reel = 0;
  PWMVG = 0;
  PWMVD = 0;
  PWM_VG_reel = 0;
  PWM_VD_reel = 0;
  MAJ_PWM();
}
void vitesseEncodeur()
{
  //*** Vitesse côté droit ***//
  tempsAcquisitionDroit = millis() - ancienTempsAcquisitionDroit;
  ancienTempsAcquisitionDroit = millis();
  pulseParcouruDroit = EncodeurRoueDroite.read() - ancienPulseDroit;
  ancienPulseDroit = EncodeurRoueDroite.read();

if(vitesseDesireeDroite >= 0 ){
  if(pulseParcouruDroit < 80*vitesseDesireeDroite && pulseParcouruDroit > 20*vitesseReelleDroite) // Il y a des valeurs abbérantes qui sont filtrées
    vitesseReelleDroite = (pulseParcouruDroit * 6.283185 * rayonRoue / (phaseEncodeurRoue * tempsAcquisitionDroit / 1000)) * M_S_TO_KM_H; // Vitesse en km/h
}
if(vitesseDesireeDroite < 0){
  if(pulseParcouruDroit > 80*vitesseDesireeDroite && pulseParcouruDroit < 20*vitesseReelleDroite) // Il y a des valeurs abbérantes qui sont filtrées
    vitesseReelleDroite = (pulseParcouruDroit * 6.283185 * rayonRoue / (phaseEncodeurRoue * tempsAcquisitionDroit / 1000)) * M_S_TO_KM_H; // Vitesse en km/h
}


  //*** Vitesse côté gauche ***//
  tempsAcquisitionGauche = millis() - ancienTempsAcquisitionGauche;
  ancienTempsAcquisitionGauche = millis();
  pulseParcouruGauche = -EncodeurRoueGauche.read() - ancienPulseGauche;
  ancienPulseGauche = -EncodeurRoueGauche.read();

if(vitesseDesireeGauche >= 0){
  if(pulseParcouruGauche < 80*vitesseDesireeGauche && pulseParcouruGauche > 20*vitesseReelleGauche) // Il y a des valeurs abbérantes qui sont filtrées
    vitesseReelleGauche = (pulseParcouruGauche * 6.283185 * rayonRoue / (phaseEncodeurRoue * tempsAcquisitionGauche / 1000)) * M_S_TO_KM_H; // Vitesse en km/h
  else
    Serial.print("*");
}

if(vitesseDesireeGauche < 0){
  if(pulseParcouruGauche > 80*vitesseDesireeGauche && pulseParcouruGauche < 20*vitesseReelleGauche) // Il y a des valeurs abbérantes qui sont filtrées
    vitesseReelleGauche = (pulseParcouruGauche * 6.283185 * rayonRoue / (phaseEncodeurRoue * tempsAcquisitionGauche / 1000)) * M_S_TO_KM_H; // Vitesse en km/h
  else
    Serial.print("*");
}



  Serial.print("  pulseParcouruGauche: ");
  Serial.print(pulseParcouruGauche);
  Serial.print(" VitesseReelleGauche: ");
  Serial.print(vitesseReelleGauche);
  Serial.print("  Max: ");
  Serial.print(80*vitesseDesireeGauche);
  Serial.print("  Min: ");
  Serial.println(20*vitesseReelleGauche);

 /* Serial.print("  pulseParcouruDroit: ");
  Serial.print(pulseParcouruDroit);
  Serial.print(" VitesseReelleDroite: ");
  Serial.println(vitesseReelleDroite);*/


}
void MAJ_PWM()
{
  if (abs(PWMD_reel) > PWM_MAX || abs(PWMG_reel) > PWM_MAX || abs(PWMG) > PWM_MAX || abs(PWMD) > PWM_MAX) // Condition de sécurité
  {
    machine_stop();
    Serial.println("MACHINE STOP - Vitesse trop elevee");
  }

  analogWrite(mg, abs(PWMG_reel));
  analogWrite(md, abs(PWMD_reel));
  analogWrite(vg, abs(PWM_VG_reel));
  analogWrite(vd, abs(PWM_VD_reel));
}

/*--- LEVAGE ---*/
void peserPatient()
{
  scale.power_up();
  POIDS = scale.get_units(40); // Lecture des données du HX711  //* 0.0001855; 4
  scale.power_down();
}
void asserVerins()
{
  //Vérifier nécessité de eMin
  e = -(float)EncodeurVerinGauche.read() + (float)EncodeurVerinDroit.read();
  if (abs(e) >= eMin)
  {
    rateError = (e - lastError); // Dérivée
    LimiterDouble(&rateError, rateErrorMax);

    output = kp * e + kd * rateError; // PID output
    LimiterDouble(&output, outputMax);
    lastError = e; // Remember current error

    if (PWMVG || PWMVD)
    {
      PWMVG -= output;
      if (PWMVG < -liftCoef)
      {
        PWMVD -= PWMVG + liftCoef;
        PWMVG = -liftCoef;
      }
      else if (PWMVG > liftCoef)
      {
        PWMVD -= PWMVG - liftCoef;
        PWMVG = liftCoef;
      }
    }
    else
    {
      rateError = 0;
    }
  }

  
} // Fin asserVerins
void consigneVerins()
{
  // Asservir les vérins si la vitesse est suffisante
  if ((abs(PWM_VG_reel) > 40) || (abs(PWM_VD_reel) > 40))
  {
    asserVerins();
  }

  // Vérification des limit switch et changement du PWM en conséquence
  if (digitalRead(LMTBG) == HIGH)
  {
    EncodeurVerinGauche.readAndReset(); // Reset de l'encodeur si la limite est atteinte
    if (PWMVG > 0)
    {
      PWMVG = 0;
      PWM_VG_reel = 0;
    }
  }
  if (digitalRead(LMTBD) == HIGH)
  {
    EncodeurVerinDroit.readAndReset(); // Reset de l'encodeur si la limite est atteinte
    if (PWMVD > 0)
    {
      PWMVD = 0;
      PWM_VD_reel = 0;
    }
  }
  if (digitalRead(LMTHG) == HIGH)
  {
    if (PWMVG < 0)
    {
      PWMVG = 0;
      PWM_VG_reel = 0;
    }
  }
  if (digitalRead(LMTHD) == HIGH)
  {
    if (PWMVD < 0)
    {
      PWMVD = 0;
      PWM_VD_reel = 0;
    }
  }
} // Fin consigneVerins
void accelVerin()
{
  // Accélération du vérin gauche
  diff_PWM_VG = (PWMVG - PWM_VG_reel);
  LimiterInt(&diff_PWM_VG, ACCELMAXVERIN);
  PWM_VG_reel += diff_PWM_VG;

  if (PWM_VG_reel < 0)
    digitalWrite(dvg, LOW);
  else
    digitalWrite(dvg, HIGH);

  // Accélération du vérin droit
  diff_PWM_VD = (PWMVD - PWM_VD_reel);
  LimiterInt(&diff_PWM_VD, ACCELMAXVERIN);
  PWM_VD_reel += diff_PWM_VD;

  if (PWM_VD_reel < 0)
    digitalWrite(dvd, LOW);
  else
    digitalWrite(dvd, HIGH);
    
} // Fin accelVerin
void verinManuel()
{
  if (!signal_verin) // Si la manette n'est pas en communication
  {
    int btnmanuel = analogRead(updn);
    if (btnmanuel <= 100) //UpDn Pas appuyé
    {
        PWMVG = 0;
        PWMVD = 0;
    }
    else if (btnmanuel > 300 && btnmanuel < 700) //UpDn vers le bas
    {
      PWMVG = liftCoef;
      PWMVD = liftCoef;
      Serial.println("down");
    }
    else if( btnmanuel >= 1000) //UpDn vers le haut
    {
      PWMVG = -liftCoef;
      PWMVD = -liftCoef;
      Serial.println("up");
    }
  }
}
void LoadCellCalib()
{
  //---------------------------------------- Calibration du loadCell ----------------------------------------
  scale.set_scale(5164.67);
  scale.tare();
  Serial.println("Mettre poids");
  delay(5000);
  Serial.println("Debut mesure");
  Serial.print("poids = ");
  Serial.println(scale.get_units(40));
  //--------------------------------------------------------------------------------------------------------
}

/*--- MAINTIEN ---*/
void springCalc()
{
  // Combinaisons pour chaque position des fourchettes
  Rkg[0] = R925 + R928 + R929;
  Rkg[1] = 0;
  Rkg[2] = R925 + R929;
  Rkg[3] = R928;
  Rkg[4] = R925 + R928;
  Rkg[5] = R929;
  Rkg[6] = R928 + R929;
  Rkg[7] = R925;
  rkg[0] = R917 + R919 + R921;
  rkg[1] = 0;
  rkg[2] = R919 + R921;
  rkg[3] = R917;
  rkg[4] = R917 + R919;
  rkg[5] = R921;
  rkg[6] = R917 + R921;
  rkg[7] = R919;
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      rcomb[8 * i + j] = rkg[i] + Rkg[j];
    }
  }
}
void calcPoids()
{
  poids = (POIDS + poidsOffset) * pcmaintien;
  poids /= 100;
  Serial.print("poids dans calcPoids : ");
  Serial.println(poids);
  if (poids == -10)
  {
    poids = 0;
  }
  else if (poids < R919)
  {
    poids = R919;
  }
}
void fourchetteConfig()
{
  int n = 0;
  int diff;
  int smaller = 200;
  for (int i = 0; i < 64; i++)
  {
    diff = abs(poids - rcomb[i]);
    if (diff < smaller)
    {
      smaller = diff;
      n = i;
    }
  }
  percu = (POIDS + poidsOffset) - (Rkg[n % 8] + rkg[n / 8]);
  char temp[10] = {0};
  itoa(percu, temp, 10);
  poidspercu.setText(temp);
  PWMFG = rconfig[n / 8];
  PWMFD = Rconfig[n % 8];
  Serial.print("Poids: ");
  Serial.print(poids);
  Serial.print(" kg -> [ Gauche: ");
  Serial.print(PWMFG);
  Serial.print(", Droit: ");
  Serial.print(PWMFD);
  Serial.print(" ] -> avec ");
  Serial.print(rcomb[n]);
  Serial.print(" kg (ecart de ");
  Serial.print(smaller);
  Serial.println(" kg)");
}
void lmtWait(int lap)
{
  unsigned long lapStart = millis();
  while ((millis() - lapStart) < (unsigned int)lap)
  {
    delay(1);

    if (digitalRead(lmt) == HIGH)
    {
      Serial.println("Err. Obstruction Soudaine Fourchette");
      msgmaintien.setText("Obstruction soudaine : tenir le ceintre pendant toute la configuration");
      MsgObstruction = true;
      break;
    }
  }
}
void setFourchette()
{
  byte lmtstatus = 0;
  delay(1000);
  for (int i = 1; i < 4; i++)
  {
    digitalWrite(mag, HIGH);
    delay(200);
    lmtstatus = digitalRead(lmt);

    if (lmtstatus == LOW)
    {
      break;
    }
    else
    {
      digitalWrite(mag, LOW);
      delay(1000);
    }
  }
  if (lmtstatus == LOW)
  {
    Serial.print("Call fourchette Gauche: ");
    Serial.print(PWMFG);
    Serial.print(", Droite: ");
    Serial.println(PWMFD);
    if (PWMFG > 257)
    {
      analogWrite(fg, PWMFG - 30);
    }
    else
    {
      analogWrite(fg, PWMFG + 30);
    }
    if (PWMFD > 257)
    {
      analogWrite(fd, PWMFD - 30);
    }
    else
    {
      analogWrite(fd, PWMFD + 30);
    }
    lmtWait(1200);
    analogWrite(fg, PWMFG);
    analogWrite(fd, PWMFD);
    lmtWait(5000);
  }
  else
  {
    Serial.println("Err. Obstruction Fourchette");
    msgmaintien.setText("Obstruction : soulever le ceintre et essayer de nouveau");
    MsgObstruction = true;
  }
  digitalWrite(mag, LOW);


  timerMsgMaintien = millis();
  boolMsgMaintien = true;

  if(!MsgObstruction)
  {
    msgmaintien.setText("Calibration terminee");
  }
  MsgObstruction = false;

} // Fin setFourchette

/*--- BATTERIE ---*/
void checkBatt()
{
  if ((millis() - battTimer) > battDelay)
  { // limitation de la fréquence de m-à-j de la charge

  if ((PWMG_reel + PWMD_reel + PWM_VG_reel + PWM_VD_reel) == 0)
  {
    battSum = 0;
    batt[battPos % battSize] = analogRead(bat);
    for (int i = 0; i < battSize; i++)
    {
      battSum += batt[i];
    }
    if (battPos > 2 * battSize)
    {
      battDelay = 10000;
      battAverage = battSum / battSize;
      battAverage = 75 / (battLevelNintyfivePc - battLevelTwentyPc) * (battAverage - battLevelTwentyPc) + 20;
      if (battAverage > (battLevel + 1))
      {
        battErr++;
        if (battErr > 3)
        {
          battErr = 3;
        }
      }
      else
      {
        battErr = 0;
      }
      if (((battAverage < battLevel) || (battErr == 3)) && (battAverage >= 20) && (battAverage <= 90))
      { // Charge approximative
        battLevel = battAverage * 1.1;
      }
      else if (((battAverage < battLevel) || (battErr == 3)) && (battAverage >= 90))
      { // Batterie pleine charge
        battLevel = 100;
      }
      else if (battAverage < -75)
      { // Niveau critique !!!
        battLevel = 5;
        Serial.println("Batterie niveau critique !");
        msgbatterie.setText("Niveau de charge critque");
      }
      else if (battAverage < -20)
      { // Charger la batterie
        battLevel = 10;
        msgbatterie.setText("Charger la batterie");
      }
      else if ((battAverage < battLevel) || (battErr == 3))
      {
        battLevel = 15;
      }
      BatteryLevel.setValue(battLevel);
    }
    battPos++;
  }
  battTimer = millis();
  }
} // Fin checkBatt

/*--- MANETTE ---*/
void traitementDonnesManette(String data)
{
  signal_verin = (data.substring(0, 1)).toInt();
  signal_x = map((data.substring(1, 5)).toInt(), 1000, 2024, 255, -255);
  signal_y = map((data.substring(5, 9)).toInt(), 1000, 2024, 255, -255);
  signal_Joystick = (data.substring(9, 10)).toInt();  // Bouton joystick non appuyé == 1 , appuyé == 0
  
  if (signal_Joystick == 1)  
  {
    signal_Joystick = 0;                  // Bouton joystick non appuyé == 0 , appuyé == 1
  }
  else
  {
    signal_Joystick = 1;
    signal_x = 0;
    signal_y = 0;
    ChangementVitesseManette();
  }

  pwmD_Value = -(double)(signal_x + signal_y);
  pwmG_Value = (double)(-signal_x + signal_y);
  LimiterDouble(&pwmD_Value, 255);
  LimiterDouble(&pwmG_Value, 255);

  if (pwmD_Value < 15 && pwmD_Value > -15) // Dead zone
    pwmD_Value = 0;
  else if (pwmG_Value < 15 && pwmG_Value > -15) // Dead zone
    pwmG_Value = 0;

  if (abs(pwmG_Value - pwmD_Value) < 15) // Faire avancer le Zénith plus droit
    pwmG_Value = pwmD_Value;

  // Ralentir la rotation
  if ((pwmG_Value > 200 && pwmD_Value < -200) || (pwmG_Value < -200 && pwmD_Value > 200))
  {
    pwmG_Value = pwmG_Value / 1.75;
    pwmD_Value = pwmD_Value / 1.75;
  }


if(signal_Joystick == 0)
{
  DescenteRapideVerin();

  if (signal_verin == 1)
  { // AUCUNE COMMANDE DE VERIN
    PWMVG = 0;
    PWMVD = 0;
  }
  else if (signal_verin == 2)
  { // VERIN UP
    PWMD = 0;
    PWMG = 0;
    PWMVG = -liftCoef;
    PWMVD = -liftCoef;
    DescenteRapide = false;
    // Serial.println("UP");
  }
  else if (signal_verin == 3)
  { // VERIN DOWN
    PWMD = 0;
    PWMG = 0;
    PWMVG = liftCoef;
    PWMVD = liftCoef;
    // Serial.println("DOWN");
  }

  if (DescenteRapide)
  {
    PWMD = 0;
    PWMG = 0;
    PWMVG = liftCoef;
    PWMVD = liftCoef;
  }
}
  if (signal_verin == 0)
  { // FIN DE LA COMMUNICATION
    manette_en_cours = false;
    PWMD = 0;
    PWMG = 0;
    PWMVG = 0;
    PWMVD = 0;
    DescenteRapide = false;
  }

} // Fin traitementDonnesManette
void CommManette()
{
  if (Wire.available())
  {
    String IDcommande = "";
    String commandeS = "";

    bool IDdone = false;

    while (Wire.available()) // loop through all but the last
    {
      char c = Wire.read(); // receive byte as a character
      if (c == '&')
        IDdone = (true);
      else if (!IDdone)
        IDcommande = IDcommande + c;
      else
        commandeS = commandeS + c;
    }
    if (String(IDcommande) == String(IDattendu))
    {
      if (commandeS.length() == 12 || commandeS.length() == 11)
      {
        manette_en_cours = true;
        DerniereComm = millis();
        traitementDonnesManette(commandeS);
      }
    }
    Wire.flush();
  }
}
void receiveEvent(int howMany)
{
  //**** NE PAS SUPPRIMER LA FONCTION  ****//
  // Necessaire pour la communication I2C
}
void SecuriteManette()
{

  if( millis() - DerniereComm > TempsSansCommMax && signal_verin != 0)
  {
    while (millis() - DerniereComm > TempsSansCommMax && signal_verin != 0)
    {
    machine_stop();
    CommManette();

    Serial.println("    PERTE DE COMMUNICATION AVEC LA MANETTE");

    msgErreur.setText("Erreur: ");
    msgDisManette.setText("Manette deconnectee");
    delay(50);
    }
    msgErreur.setText(" ");
    msgDisManette.setText(" ");
  }
}
void ChangementVitesseManette()
{
    if (signal_verin == 2 && coefVitesse < 4.5 && millis()-TempsChangementVitesse > 250){
      coefVitesse += 0.25;
      speedcoef = coefVitesse*100;
      PWM_MAX = 30+35*coefVitesse;
      Vitesse.setValue(speedcoef);
      VitesseFloat.setValue(speedcoef);
      TempsChangementVitesse = millis();
    }
    else if (signal_verin == 3 && coefVitesse > 0 && millis()-TempsChangementVitesse > 250){
      coefVitesse -= 0.25;
      speedcoef = coefVitesse*100;
      PWM_MAX = 30+35*coefVitesse;
      Vitesse.setValue(speedcoef);
      VitesseFloat.setValue(speedcoef);
      TempsChangementVitesse = millis();
    }
}
void DescenteRapideVerin()
{
  if(signal_verin == 3)
    DescenteTimer1 = millis();

  if(signal_verin == 1 && (millis() - DescenteTimer1) < 500)
    DescenteTimer2 = millis();

  if(signal_verin == 3 && (millis() - DescenteTimer2) < 500)
    DescenteRapide = true;
}

/*--- POIGNÉES ---*/
void ContrlPoignees()
{
  // Lecture des gachettes aux poignées (potentiomètre). Plage de conversion entre 610-350 pour gauche et 900-670 (branché à l'envers) pour s'adapter à la rotation limitée des gachettes. Finit à 100/970 (et non 0/1023)
  // pour éviter des signaux parasites si la gachette n'est pas parfaitement dépressée.
  pwmg = map(analogRead(potg), 160, 32, 0, 255);
  if (pwmg < 50) // Si la valeur du potentiomètre sort de la plage
    pwmg = 0;
  else if (pwmg > 255)
    pwmg = 255;

  pwmd = map(analogRead(potd), 135, 300, 0, 255);
  if (pwmd < 50) // Si la valeur du potentiomètre sort de la plage
    pwmd = 0;
  else if (pwmd > 255)
    pwmd = 255;


  if (abs(pwmg - pwmd) < 15) // Faire avancer le Zénith plus droit
    pwmg = pwmd;

  if (dir == ARRIERE)
  {
    pwmg = -pwmg;
    pwmd = -pwmd;
  }

  if (signal_Joystick) // Si le joystick est appuyé, les gachettes sont ignorées
  {
    pwmg = 0;
    pwmd = 0;
  }

  // Mode d'avance du Zenith: roues solidaires (HIGH) ou indépendantes (LOW)
  if (mode == INDEPENDANT)
  {
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDV, LOW);

    if (nmode == 1)
    { // Algo selon le numéro de mode de contrôle

      vitesseDesireeGauche = ((float)pwmg / 255.0 * coefVitesse);
      vitesseDesireeDroite = ((float)pwmd / 255.0 * coefVitesse);
    }
    else if (nmode == 2)
    {
      if (dir == AVANT)
      {
        vitesseDesireeGauche = ((float)pwmd / 255.0 * coefVitesse);
        vitesseDesireeDroite = ((float)pwmg / 255.0 * coefVitesse);
      }
      else
      {
        vitesseDesireeGauche = ((float)pwmg / 255.0 * coefVitesse);
        vitesseDesireeDroite = ((float)pwmd / 255.0 * coefVitesse);
      }
    }
    else if (nmode == 3)
    {
      vitesseDesireeGauche = ((float)pwmd / 255.0 * coefVitesse);
      vitesseDesireeDroite = ((float)pwmg / 255.0 * coefVitesse);
    }
    else if (nmode == 4)
    {
      if (dir == AVANT)
      {
        vitesseDesireeGauche = ((float)pwmg / 255.0 * coefVitesse);
        vitesseDesireeDroite = ((float)pwmd / 255.0 * coefVitesse);
      }
      else
      {
        vitesseDesireeGauche = ((float)pwmd / 255.0 * coefVitesse);
        vitesseDesireeDroite = ((float)pwmg / 255.0 * coefVitesse);
      }
    }
  }
  else if (mode == SOLIDAIRE)
  {
    
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDV, HIGH);

    if (dir == AVANT)
      pwm_min = min(pwmg, pwmd); // Mode solidaire contrôle les deux roues à la même vitesse selon la gâchette la moins pressée.
    else 
      pwm_min = max(pwmg, pwmd); // Mode solidaire contrôle les deux roues à la même vitesse selon la gâchette la moins pressée.
     
                               // Adapté pour les patients hémiplégiques (moitié gauche/droite du corps avec moins de contrôle moteur et/ou force).
    vitesseDesireeGauche = ((float)pwm_min / 255.0 * coefVitesse);
    vitesseDesireeDroite = ((float)pwm_min / 255.0 * coefVitesse);
  }

  if (dir == AVANT)
  {
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDJ, HIGH);
  }
  else
  {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDJ, LOW);
  }
}
void fermerLED()
{
  if (activ_poignees == LOW) // Lit seulement les valeurs PWM des poignées si les poignées sont activées sur l'écran.
  {
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDV, LOW);
  digitalWrite(LEDJ, LOW);
  }
  else
  {
    if (mode == INDEPENDANT)
    {
      digitalWrite(LEDB, HIGH);
      digitalWrite(LEDV, LOW);
    }
    if (mode == SOLIDAIRE)
    {
      digitalWrite(LEDB, LOW);
      digitalWrite(LEDV, HIGH);
    }
    if (dir == AVANT)
    {
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDJ, HIGH);
    }
    if (dir == ARRIERE)
    {
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDJ, LOW);
    }
  }
}

/*--- ÉCRAN ---*/
void checkMsg()
{
  if (millis() - timerMsgMaintien > TempsMaxMsgMaintien && boolMsgMaintien)
    {
      msgmaintien.setText(" ");
      boolMsgMaintien = false;
    }

  if (millis() - timerMsgVitesse > TempsMaxMsgVitesse && boolMsgVitesse)
    {
      msgvitesse.setText(" ");
      boolMsgVitesse = false;
    }

  if (millis() - timerMsgPoids > TempsMaxMsgPoids && boolMsgPoids)
    {
      msgpoids.setText(" ");
      boolMsgPoids = false;
    }

}
/*void checkpage()
{
  chr[0] = {0};
  memset(chr, 0, 3);
  nopageModes.getText(chr, 3);
  nPagelue = atoi(chr);
  if (nPagelue == 2)
  {
    npage = 2;
  }
  else
  {
    chr[0] = {0};
    memset(chr, 0, 3);
    nopageDebut.getText(chr, 3);
    nPagelue = atoi(chr);
    if (nPagelue == 1)
    {
      npage = 1;
    }
    else
    {
      Serial.println("Erreur checkpage");
    }
  }
} // Fin checkpage*/
void nextPopCallback(void *ptr)
{

  if (nmode == 1)
  {
    MODE.setText("1");
    bMode1.Set_background_color_bco(3463);
  }
  else if (nmode == 2)
  {
    MODE.setText("2");
    bMode2.Set_background_color_bco(3463);
  }
  else if (nmode == 3)
  {
    MODE.setText("3");
    bMode3.Set_background_color_bco(3463);
  }
  else if (nmode == 4)
  {
    MODE.setText("4");
    bMode4.Set_background_color_bco(3463);
  }
  else
  {
    Serial.println("Erreur nextPopCallback");
  }


  Serial.println("nextPopCallback");
  npage = 2;
  //delay(200);
}
void next2PopCallback(void *ptr)
{
  Serial.println("next2PopCallback");
  npage = 3;

  if (PremierChangementP3 == true)
  {
    SliderDev.setValue(CorrDeviation);
    SliderAcc.setValue(CorrAcceleration);
    ValDev.setValue(CorrDeviation);
    ValAcc.setValue(CorrAcceleration);
    ACCELMAX *= (float)CorrAcceleration / 100.0;
    PremierChangementP3 = false;
  }

  DateTime now = rtc.now();
  delay(100);
  HeureP3.setValue(now.hour());
  MinuteP3.setValue(now.minute());
  //delay(200);
}
void prevPopCallback(void *ptr)
{

  DateTime now = rtc.now();
  delay(100);
  Heure.setValue(now.hour());
  Minute.setValue(now.minute());

  Serial.println("prevPopCallback");
  npage = 0;
  //delay(200);
}
void prev2PopCallback(void *ptr)
{
  Serial.println("prev2PopCallback");
  npage = 2;

  if (nmode == 1)
  {
    MODE.setText("1");
    bMode1.Set_background_color_bco(3463);
  }
  else if (nmode == 2)
  {
    MODE.setText("2");
    bMode2.Set_background_color_bco(3463);
  }
  else if (nmode == 3)
  {
    MODE.setText("3");
    bMode3.Set_background_color_bco(3463);
  }
  else if (nmode == 4)
  {
    MODE.setText("4");
    bMode4.Set_background_color_bco(3463);
  }
  else
  {
    Serial.println("Erreur nextPopCallback");
  }

  //delay(200);
}
void majMaintienPopCallback(void *ptr)
{
  if (wheelstopped)
  {
    Serial.println("Sequence fourchette");
    msgmaintien.setText("Calibration en cours...");
    PoucMaintien.getValue(&pcmaintien);
    calcPoids();
    fourchetteConfig();
    setFourchette();
  }
  else
  {
    Serial.println("Err. Freins releves !");
    msgmaintien.setText("Freins releves !");
  }
}
void majVitessePopCallback(void *ptr)
{
  Vitesse.getValue(&speedcoef);
  coefVitesse = (float)speedcoef / 100.0;
  PWM_MAX = 30+35*coefVitesse;


  timerMsgVitesse = millis();
  boolMsgVitesse = true;
  msgvitesse.setText("La vitesse a ete mise a jour");
}
void okPopCallback(void *ptr)
{
  chr[0] = 0;
  Serial.print("ok ");
  delay(500);
  //checkpage();
  memset(chr, 0, 3);
  poidspatient.getText(chr, 3);
  Serial.println(npage);
  if (npage == 5)
  {
    poids = atoi(chr);
    poidsOffset = poids - POIDS;
    EEPROM.write(eepromPoidsOffsetAddr, poidsOffset + 128);
    chr[0] = 0;
    memset(chr, 0, 3);
    itoa((int)poidsOffset, chr, 10);
    msgpoids.setText(chr);
    Serial.print("Nouveau Offset: ");
    Serial.println(poidsOffset);
    POIDS = poids;
  }
  else
  {
    POIDS = atoi(chr);
  }
}
void balancePopCallback(void *ptr)
{
  msgpoids.setText("Pesee en cours...");
  chr[0] = 0;
  Serial.println("balancePopCallback");
  peserPatient();
  if (POIDS < 1)
  {
    POIDS = 0;
    msgpoids.setText("Erreur : poids negatif");
  }
  else
  {
    msgpoids.setText("Pesee terminee");
  }
  timerMsgPoids = millis();
  boolMsgPoids = true;
  memset(chr, 0, 3);
  itoa((int)(POIDS + poidsOffset), chr, 10);
  poidspatient.setText(chr);
}
void bOnPopCallback(void *ptr)
{
  bStat.setText("Statut: on");
  activ_poignees = HIGH;
}
void bOffPopCallback(void *ptr)
{
  bStat.setText("Statut: off");
  activ_poignees = LOW;
  pwmg = 0;
  pwmd = 0;
}
void bMode1PopCallback(void *ptr)
{
  nmode = 1;
  MODE.setText("1");
  bMode1.Set_background_color_bco(3463);
  bMode2.Set_background_color_bco(50712);
  bMode3.Set_background_color_bco(50712);
  bMode4.Set_background_color_bco(50712);
  activ_poignees = HIGH;
}
void bMode2PopCallback(void *ptr)
{
  nmode = 2;
  MODE.setText("2");
  bMode1.Set_background_color_bco(50712);
  bMode2.Set_background_color_bco(3463);
  bMode3.Set_background_color_bco(50712);
  bMode4.Set_background_color_bco(50712);
  activ_poignees = HIGH;
}
void bMode3PopCallback(void *ptr)
{
  nmode = 3;
  MODE.setText("3");
  bMode1.Set_background_color_bco(50712);
  bMode2.Set_background_color_bco(50712);
  bMode3.Set_background_color_bco(3463);
  bMode4.Set_background_color_bco(50712);
  activ_poignees = HIGH;
}
void bMode4PopCallback(void *ptr)
{
  nmode = 4;
  MODE.setText("4");
  bMode1.Set_background_color_bco(50712);
  bMode2.Set_background_color_bco(50712);
  bMode3.Set_background_color_bco(50712);
  bMode4.Set_background_color_bco(3463);
  activ_poignees = HIGH;
}
void SliderDevPopCallback(void *ptr)
{
  Dev.getValue(&CorrDeviation);
  EEPROM.write(eepromCorrDeviationAddr, CorrDeviation);
  Serial.print("Deviation: ");
  Serial.println(CorrDeviation);
}
void SliderAccPopCallback(void *ptr)
{
  Acc.getValue(&CorrAcceleration);
  ACCELMAX *= (float)CorrAcceleration / 100.0;
  EEPROM.write(eepromCorrAccelerationAddr, CorrAcceleration);
  Serial.print("Acceleration: ");
  Serial.println(CorrAcceleration);
}
void pDevPopCallback(void *ptr)
{
  Dev.getValue(&CorrDeviation);
  EEPROM.write(eepromCorrDeviationAddr, CorrDeviation);
  Serial.print("Deviation: ");
  Serial.println(CorrDeviation);
}
void pAccPopCallback(void *ptr)
{
  Acc.getValue(&CorrAcceleration);
  ACCELMAX *= (float)CorrAcceleration / 100.0;
  EEPROM.write(eepromCorrAccelerationAddr, CorrAcceleration);
  Serial.print("Acceleration: ");
  Serial.println(CorrAcceleration);
}
void majHorloge(){
  now = rtc.now();

  if (now.second() == 0 && !TempsDejaChange && (now.second() - DernierTemps == 1 || now.second() - DernierTemps == 0 || DernierTemps - now.second() == 59))
  {
    Heure.setValue(now.hour());
    Minute.setValue(now.minute());
    HeureP3.setValue(now.hour());
    MinuteP3.setValue(now.minute());
    TempsDejaChange = true;
  }
  if (now.second() != 0 && TempsDejaChange)
    TempsDejaChange = false;

  DernierTemps = now.second();
}
void HeurePlusPopCallback(void *ptr)
{
  now = rtc.now();
  if (now.hour() < 23)
  {
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour() + 1, now.minute(), now.second()));
    now = rtc.now();
    Heure.setValue(now.hour());
    HeureP3.setValue(now.hour());
  }

}
void HeureMoinsPopCallback(void *ptr)
{
  now = rtc.now();

  if (now.hour() > 0)
  {
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour() - 1, now.minute(), now.second()));
    now = rtc.now();
    Heure.setValue(now.hour());
    HeureP3.setValue(now.hour());
  }
}
void MinutePlusPopCallback(void *ptr)
{
  now = rtc.now();
  if (now.minute() < 59)
  {
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute() + 1, now.second()));
    now = rtc.now();
    MinuteP3.setValue(now.minute());
    Minute.setValue(now.minute());
  }
}
void MinuteMoinsPopCallback(void *ptr)
{

  now = rtc.now();
  if (now.minute() > 0)
  {
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute() - 1, now.second()));
    now = rtc.now();
    MinuteP3.setValue(now.minute());
    Minute.setValue(now.minute());
  }
}

//Autres fonctions
void LimiterDouble(double *nombre, int max)
{
  if (*nombre < -max)
    *nombre = -max;
  else if (*nombre > max)
    *nombre = max;
}
void LimiterInt(int *nombre, int max)
{
  if (*nombre < -max)
    *nombre = -max;
  else if (*nombre > max)
    *nombre = max;
}