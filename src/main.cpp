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
  Wire.onReceive(receiveEvent); // register event


  // //EEPROM
  poidsOffset = 0; //EEPROM.read(eepromPoidsOffsetAddr) - 128; // Lecture EEPROM

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
  pinMode(vdd, OUTPUT);
  pinMode(vdg, OUTPUT);
  pinMode(dmd, OUTPUT);
  pinMode(dmg, OUTPUT);
  pinMode(mag, OUTPUT);
  pinMode(updn, INPUT);
  pinMode(LimitSwitchBD,INPUT_PULLUP);
  pinMode(LimitSwitchBG,INPUT_PULLUP);
  pinMode(LimitSwitchHD,INPUT_PULLUP);
  pinMode(LimitSwitchHG,INPUT_PULLUP);
  pinMode(lmt, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(boutr, INPUT_PULLUP);
  pinMode(swit, INPUT_PULLUP);
  pinMode(potd, INPUT_PULLUP);
  pinMode(potg, INPUT_PULLUP);
  pinMode(ledBleue, OUTPUT);
  pinMode(ledJaune, OUTPUT);
  pinMode(ledRouge, OUTPUT);
  pinMode(ledVerte, OUTPUT);
  pinMode(EncoderRW, INPUT_PULLUP);
  pinMode(EncoderLW, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(EncoderRW),acquisitionEncoderR,FALLING);
 // attachInterrupt(digitalPinToInterrupt(EncoderLW),acquisitionEncoderL,FALLING);
  //enableInterrupt(EncoderRW, acquisitionEncoderR, FALLING);
  //enableInterrupt(EncoderLW, acquisitionEncoderL, FALLING);

  // pinMode(30, OUTPUT);
  
  // // Établit la liste des items Nextion à surveiller (items qui peuvent être appuyés) pour l'écran
  majMaintien.attachPop(majMaintienPopCallback, &majMaintien);
  majVitesse.attachPop(majVitessePopCallback, &majVitesse);
  ok.attachPop(okPopCallback, &ok);
  balance.attachPop(balancePopCallback, &balance);
  next.attachPop(nextPopCallback);
  prev.attachPop(prevPopCallback);
  bOn.attachPop(bOnPopCallback, &bOn);
  bOff.attachPop(bOffPopCallback, &bOff);
  bMode1.attachPop(bMode1PopCallback, &bMode1);
  bMode2.attachPop(bMode2PopCallback, &bMode2);
  bMode3.attachPop(bMode3PopCallback, &bMode3);
  bMode4.attachPop(bMode4PopCallback, &bMode4);

  // // Initialisation du compteur des encodeurs quad, vérins
  EncoderVerinGauche.setEncoderCount(0); // Clear Encoder
  EncoderVerinDroit.setEncoderCount(0); // Clear Encoder
  referenceBouton = analogRead(updn);
  nexInit();                    // Initialisation de la librairie de l'Écran
  delay(10);
  
  Serial.println("Start"); // Checkup série
  
  scale.begin(dat, clk);    //Pins pour le Output des donnees et la Clock.
  scale.set_scale(5164.67); //Scale trouver apres la calibration des donnees
  scale.tare();             // Mise à zéro de la pesée
  scale.power_down();

  springCalc(); // Calcul des combinaisons de ressorts

  bStat.setText("Statut: off");
} // fin setup
void loop()
{
  nexLoop(nex_listen_list); // Écoute les items Nextion

  Serial.print(" EncoderVerinGauche: ");
  Serial.print(EncoderVerinGauche.getEncoderCount());
  
  Serial.print("     EncoderVerinDroit: ");
  Serial.println(EncoderVerinDroit.getEncoderCount());  



  

  switch (npage)
  {       // Algo selon le numéro de page de l'écran
  case 1: // Interface normale
    
    if ((millis() - jerkTimer) > jerkTime)
    { // limitation de la fréquence de m-à-j des PWMs
      encoderSpeedCalc();
      motorDriveCalc();
      consigneVerins();
      verinDriveCalc();
      jerkTimer = millis();
    }
    if ((millis() - battTimer) > battDelay)
    { // limitation de la fréquence de m-à-j de la charge
      checkBatt();
      battTimer = millis();
    }
    // M-à-j des PWMs
    analogWrite(mg, abs(PWMG_reel));
    analogWrite(md, abs(PWMD_reel));
    analogWrite(vg, abs(PWM_VG_reel));
    analogWrite(vd, abs(PWM_VD_reel));
   if(Wire.available() > 0)
   {
     envoieValeurs();
     processIncomingByte();
   }
   else if (watchdog)
   {
     if ((millis() - watchdogTimer) > watchdogDelay)
     { // Manette déconnectée ?
       machine_stop();
     }
   }
   else
   {
     if (activ_poignees == HIGH)
     { // Lit seulement les valeurs PWM des poignées si les poignées sont activées sur l'écran.
       ContrlPoignees();
     }
     else
     {
       digitalWrite(ledBleue, LOW);
       digitalWrite(ledRouge, LOW);
       digitalWrite(ledVerte, LOW);
       digitalWrite(ledJaune, LOW);
     }
     verinManuel();
   }
   
    break;
  case 2: // Case 2 est utilisee pour faire des tests

    if ((millis() - jerkTimer) > jerkTime)
    { // limitation de la fréquence de m-à-j des PWMs
      encoderSpeedCalc();
      motorDriveCalc();
      consigneVerins();
      verinDriveCalc();
      jerkTimer = millis();
    }
    if ((millis() - battTimer) > battDelay)
    { // limitation de la fréquence de m-à-j de la charge
      checkBatt();
      battTimer = millis();
    }
    // M-à-j des PWMs
    analogWrite(mg, abs(PWMG_reel));
    analogWrite(md, abs(PWMD_reel));
    analogWrite(vg, abs(PWM_VG_reel));
    analogWrite(vd, abs(PWM_VD_reel));

   if(Wire.available() > 0)
   {
     envoieValeurs();
     processIncomingByte();
   }
   else if (watchdog)
   {
     if ((millis() - watchdogTimer) > watchdogDelay)
     { // Manette déconnectée ?
       machine_stop();
     }
   }
   else
   {
     if (activ_poignees == HIGH)
     { // Lit seulement les valeurs PWM des poignées si les poignées sont activées sur l'écran
       ContrlPoignees();
     }
     else
     {
       digitalWrite(ledBleue, LOW);
       digitalWrite(ledRouge, LOW);
       digitalWrite(ledVerte, LOW);
       digitalWrite(ledJaune, LOW);
     }
     verinManuel();
   }

    //---------------------------------------- Calibration du loadCell ----------------------------------------
    // scale.set_scale(5164.67);
    // scale.tare();
    // Serial.println("Mettre poids");
    // delay(5000);
    // Serial.println("Debut mesure");
    // Serial.print("poids = ");
    // Serial.println(scale.get_units(40));
    //--------------------------------------------------------------------------------------------------------

    break;
  default:
    checkpage();
    break;
  }
} // fin loop

void receiveEvent(int howmany)
{
  //rien
}

//-------------------------------------------------------------------------------------------------------------
// DEBUT DES FONCTIONS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*--- DÉPLACEMENT ---*/
void riseBreaks()
{ 
  wheelstopped = 0;
  digitalWrite(bkd, HIGH);
  digitalWrite(bkg, HIGH);
}
void Break()
{ 
  wheelstopped = 1;
  digitalWrite(bkd, LOW);
  digitalWrite(bkg, LOW);
}
void asserMoteurs()
{
  // À compléter pour l'algorithme de contrôle PID
  // eM = vitesseDroiteReelle - vitesseDesiree;
  // if (abs(eM) >= eMinM)
  // {
  //   cumErrorM = eM + lastErrorM;    // Integrale
  //   rateErrorM = (eM - lastErrorM); // Dérivée
  //   if (rateErrorM > rateErrorMaxM)
  //   {
  //     rateErrorM = rateErrorMaxM;
  //   }
  //   outputM = kpM * eM + kiM * cumErrorM + kdM * rateErrorM; // PID output
  //   if (outputM > outputMaxM)
  //   {
  //     outputM = outputMaxM;
  //   }
  //   else if (outputM < -outputMaxM)
  //   {
  //     outputM = -outputMaxM;
  //   }
  //   lastErrorM = eM; // Remember current error
  //   PWMD += outputM;
  // }
  // else
  // {
  //   rateErrorM = 0;
  // }
  } // Fin asserMoteurs 
void motorDriveCalc()
{
  if (activ_poignees == HIGH){
    mode = abs(1-digitalRead(swit));
    dir = abs(1-digitalRead(boutr));
    delay(15);
  }
  // Moteur côté controlleur  ------------------------------------------------------
  // Plage d'acceleration du moteur
  diff_PWMD = (PWMD - PWMD_reel);
  if (diff_PWMD > ACCELMAX)
    PWMD_reel += ACCELMAX;
  else if (diff_PWMD < -ACCELMAX)
    PWMD_reel += -ACCELMAX;
  else
    PWMD_reel += diff_PWMD;

  // Décision logique pour la direction des roues motrices, PWMD_reel vient de la manette et dir vient des poignées.
  if ((PWMD_reel < 0) || ((dir == HIGH) && activ_poignees ))
    digitalWrite(dmd, LOW);
  if ((PWMD_reel > 0 && manette_en_cours) || (dir == LOW && activ_poignees && !manette_en_cours))
    digitalWrite(dmd, HIGH);

  // Moteur côté batterie ------------------------------------------------------
  // Plage d'acceleration du moteur côté batterie 
  diff_PWMG = (PWMG - PWMG_reel);
  if (diff_PWMG > ACCELMAX)
    PWMG_reel += ACCELMAX;
  else if (diff_PWMG < -ACCELMAX)
    PWMG_reel += -ACCELMAX;
  else
    PWMG_reel += diff_PWMG;
  
  // Décision logique pour la direction des roues motrices, PWMG_reel vient de la manette et dir vient des poignées.
  if ((PWMG_reel < 0) || ((dir == HIGH) && activ_poignees))
    digitalWrite(dmg, LOW);
  if ((PWMG_reel > 0 && manette_en_cours) || (dir == LOW && activ_poignees && !manette_en_cours))
    digitalWrite(dmg, HIGH);

  // Frein  
  if (!wheelstopped && !activ_poignees && !manette_en_cours)    // Actionne les freins des roues motrices si celles-ci ne sont pas déjà arrêtées et qu'il n'y a plus de commandes provenant de la manette ET des poignées.
    Break();
  else if (wheelstopped && (activ_poignees || manette_en_cours))    // Désactive les freins des roues motrices si les roues sont déjà arrêtées et que le Arduino reçoit des commandes provenant des poignées OU de la manette.
    riseBreaks();
} // fin motorDriveCalc
void machine_stop() 
{
  PWMD = 0;
  PWMG = 0;
  PWMVG = 0;
  PWMVD = 0;
  watchdog = 0;
}

void acquisitionEncoderL()
{
  if (millis()-TimeDebouceL > 7)
    EncoderStepL ++;

  TimeDebouceL = millis();
}
void acquisitionEncoderR()
{
  if (millis()-TimeDebouceR > 7)
    EncoderStepR ++;

  TimeDebouceR = millis();
}
void encoderSpeedCalc(){
    if(EncoderStepR>8){
      acquisitionTimeR = millis() - lastAcquisitionTimeR;
      vitesseDroiteReelle = EncoderStepR*6.283185*rayonRoue/(EncoderHoleQuantity*acquisitionTimeR/1000); //  m/s
      vitesseDroiteReelle = (1/vitesseDroiteReelle)*3; //secondes / m

        /*Serial.print("Vitesse Droite reelle: ");
        Serial.print(vitesseDroiteReelle);*/
        lastAcquisitionTimeR = millis();
        noInterrupts();
        EncoderStepR=0;
        interrupts();
    }

      if(EncoderStepL>8){
      acquisitionTimeL = millis() - lastAcquisitionTimeL;
      vitesseGaucheReelle = EncoderStepL*6.283185*rayonRoue/(EncoderHoleQuantity*acquisitionTimeL/1000); //  m/s
      vitesseGaucheReelle = (1/vitesseGaucheReelle)*3; //secondes / m

        /*Serial.print("                  Vitesse Gauche reelle: ");
        Serial.println(vitesseGaucheReelle);*/

      lastAcquisitionTimeL = millis();
      noInterrupts();
      EncoderStepL=0;
      interrupts();
    }

}

/*--- INTERNAL MESUREMENT UNIT ---*/
float lectureAngle()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();
  float w, x, y, z, coef, p[3], p_prime[3], pitch;

  pitch = abs(euler.y()) * DegToRad;

  w = quat.w();
  x = quat.x();
  y = quat.y();
  z = quat.z();
  coef = 2 / (w * w + x * x + y * y + z * z);

  p[0] = cos(pitch);
  p[1] = 0;
  p[2] = sin(pitch);

  p_prime[0] = p[0] + coef * (y * (w * p[2] + x * p[1] - y * p[0]) - z * (w * p[1] + z * p[0] - x * p[2]));
  p_prime[1] = p[1] + coef * (z * (w * p[0] + y * p[2] - z * p[1]) - x * (w * p[2] + x * p[1] - y * p[0]));
  p_prime[2] = p[2] + coef * (x * (w * p[1] + z * p[0] - x * p[2]) - y * (w * p[0] + y * p[2] - z * p[1]));

  return atan2(p_prime[1], p_prime[0]);
}
void envoieValeurs()
{
  //float angle = lectureAngle()*1000;
  // Serial.print(ID);
  // Serial.println(angle);
  //delay(60);
}
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
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
  e = encoder2Reading - encoder1Reading;
  if (abs(e) >= eMin)
  {
    cumError = e + lastError;    // Integrale
    rateError = (e - lastError); // Dérivée
    if (rateError > rateErrorMax)
    {
      rateError = rateErrorMax;
    }
    output = kp * e + ki * cumError + kd * rateError; // PID output
    if (output > outputMax)
    {
      output = outputMax;
    }
    else if (output < -outputMax)
    {
      output = -outputMax;
    }
    lastError = e; // Remember current error
    if (PWMVG || PWMVD)
    {
      PWMVG += output;
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
  //Asservir les vérins si la vitesse est suffisante
  if ((abs(PWM_VG_reel) > 40) || (abs(PWM_VD_reel) > 40))
  {
      asserVerins();
  }

  //Vérification des limit switch et changement du PWM en conséquence
  if (digitalRead(LimitSwitchBG) == HIGH)
  {
    EncoderVerinGauche.setEncoderCount(0); // Reset de l'encodeur si la limite est atteinte
    if (PWMVG > 0)
    {
      PWMVG = 0;
      PWM_VG_reel = 0;
    }
  }
  if (digitalRead(LimitSwitchBD) == HIGH)
  {
    EncoderVerinDroit.setEncoderCount(0); // Reset de l'encodeur si la limite est atteinte
    if (PWMVD > 0)
    {
      PWMVD = 0;
      PWM_VD_reel = 0;
    }
  }
if (digitalRead(LimitSwitchHG) == HIGH)
{
  if (PWMVG < 0)
  {
    PWMVG = 0;
    PWM_VG_reel = 0;
  }

}
if (digitalRead(LimitSwitchHD) == HIGH)
{
  if (PWMVD < 0)
  {
    PWMVD = 0;
    PWM_VD_reel = 0;
  }

}
} // Fin consigneVerins
void verinDriveCalc()
{
  diff_PWM_VG = (PWMVG - PWM_VG_reel) / 2;
  if (diff_PWM_VG > 10)
  {
    diff_PWM_VG = 10;
    PWM_VG_reel += diff_PWM_VG;
  }
  else if (diff_PWM_VG < -10)
  {
    diff_PWM_VG = -10;
    PWM_VG_reel += diff_PWM_VG;
  }
  else if ((abs(diff_PWM_VG) < 2) && (abs(PWMVG) < 2)) //Vérifier la nécessité de cette condition
  {
    diff_PWM_VG = 0;
    PWM_VG_reel = 0;
  }
  else
  {
    PWM_VG_reel += diff_PWM_VG;
  }

  if (PWM_VG_reel < 0)
  {
    digitalWrite(vdg, LOW);
    digitalWrite(vdd, LOW);
  }
  else
  {
    digitalWrite(vdg, HIGH);
    digitalWrite(vdd, HIGH);
  }
  diff_PWM_VD = (PWMVD - PWM_VD_reel) / 2;
  if (diff_PWM_VD > 10)
  {
    diff_PWM_VD = 10;
    PWM_VD_reel += diff_PWM_VD;
  }
  else if (diff_PWM_VD < -10)
  {
    diff_PWM_VD = -10;
    PWM_VD_reel += diff_PWM_VD;
  }
  else if ((abs(diff_PWM_VD) < 2) && (abs(PWMVD) < 2)) //Vérifier la nécessité de cette condition
  {
    diff_PWM_VD = 0;
    PWM_VD_reel = 0;
  }
  else
  {
    PWM_VD_reel += diff_PWM_VD;
  }
  if (PWM_VD_reel < 0)
  {
    digitalWrite(vdg, LOW);
    digitalWrite(vdd, LOW);
  }
  else
  {
    digitalWrite(vdg, HIGH);
    digitalWrite(vdd, HIGH);
  }
} // Fin verinDriveCalc
void verinManuel()
{ 
  int btnmanuel = analogRead(updn);
  if (btnmanuel >= (referenceBouton-100))
  {
    if (btnreleased == 0)
    {
      PWMVG = 0;
      PWMVD = 0;
      btnreleased = 1;
    }
  }
  else if (btnmanuel > (100))
  {
    PWMVG = liftCoef;
    PWMVD = liftCoef;
    btnreleased = 0;
    Serial.println("down");
  }
  else
  {
    PWMVG = -liftCoef;
    PWMVD = -liftCoef;
    btnreleased = 0;
    Serial.println("up");
  }
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
  if (poids < R919)
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
    if (digitalRead(lmt) == HIGH)
    {
      Serial.println("Err. Obstruction Soudaine Fourchette");
      msgmaintien.setText("Obstruction soudaine : tenir le ceintre pendant toute la configuration");
      break;
    }
    else
    {
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
    Serial.println(lmtstatus);
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
  }
  digitalWrite(mag, LOW);
} // Fin setFourchette

/*--- BATTERIE ---*/
void checkBatt()
{
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
} // Fin checkBatt

/*--- MANETTE ---*/
void process_data (String data) 
{
  signal_verin = (data.substring(0,1)).toInt();
  signal_x = map((data.substring(1,5)).toInt(),1000,2024,255,-255);
  signal_y = map((data.substring(5,9)).toInt(),1000,2024,255,-255);

  pwmD_Value = -(float)(signal_x + signal_y);
  pwmG_Value = (float)(-signal_x + signal_y);
  if (pwmD_Value > 255)
    pwmD_Value = 255;
  else if (pwmD_Value < -255)
    pwmD_Value = -255;
  if (pwmG_Value > 255)
    pwmG_Value = 255;
  else if (pwmG_Value < -255)
    pwmG_Value = -255;

  pwmD_Value = pwmD_Value / 512 * speedcoef * jogCoef; // Scale 255 selon limite jogCoef, pour PWM
  pwmG_Value = pwmG_Value / 512 * speedcoef * jogCoef;
  PWMD = int(pwmD_Value);
  PWMG = int(pwmG_Value);
  
  if(signal_verin == 0) {  // FIN DE LA COMMUNICATION
    manette_en_cours = false;
    PWMD = 0;
    PWMG = 0;
    PWMVG = 0;
    PWMVD = 0;
  }
  else if(signal_verin == 1) { // AUCUNE COMMANDE DE VERIN
    PWMVG = 0;
    PWMVD = 0;
  }
  else if (signal_verin == 2) { //VERIN UP
  PWMD = 0;
  PWMG = 0;
  PWMVG = -liftCoef;
  PWMVD = -liftCoef;
  //Serial.println("UP");
  }
    else if (signal_verin == 3) { //VERIN DOWN
  PWMD = 0;
  PWMG = 0;
  PWMVG = liftCoef;
  PWMVD = liftCoef;
  //Serial.println("DOWN");
  }

  
  watchdog = 1;
  watchdogTimer = millis();
}  // Fin process_data
void processIncomingByte ()
{
  String IDcommande = ""; 
  String commandeS = "" ;

  bool IDdone = false;

  while(0 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    if(c == '&')
      IDdone = (true);
    else if (!IDdone)
      IDcommande = IDcommande +c;
    else
      commandeS = commandeS + c;
  }
  if(String(IDcommande) == String(IDattendu))
  { 
    manette_en_cours = true;
    process_data(commandeS);
  }
  Wire.flush();
}

/*--- POIGNÉES ---*/
int smoothing(const byte analogPin)
{
  int array = 0;

  for(int i = 0; i < 8; i++)
  {
    array += analogRead(analogPin);
    delay(5);
  }

  return (array/8);
}
void ContrlPoignees()
{                       
  bStat.setText("Statut: on");
  // Lecture des gachettes aux poignées (potentiomètre). Plage de conversion entre 610-350 pour gauche et 900-670 (branché à l'envers) pour s'adapter à la rotation limitée des gachettes. Finit à 100/970 (et non 0/1023)
  // pour éviter des signaux parasites si la gachette n'est pas parfaitement dépressée.
  pwmg = map(smoothing(potg), 160, 0, 0, 255);
  if (pwmg < 50) // Si la valeur du potentiomètre sort de la plage
    pwmg = 0;
  else if (pwmg > 255)
    pwmg = 255;

  pwmd = map(smoothing(potd), 135, 320, 0, 255);
   if (pwmd < 50) // Si la valeur du potentiomètre sort de la plage
    pwmd = 0;
  else if (pwmd > 255)
    pwmd = 255;

  // Mode d'avance du Zenith: roues solidaires (HIGH) ou indépendantes (LOW)
  if (mode == HIGH)
  {
    digitalWrite(ledBleue, HIGH);
    digitalWrite(ledVerte, LOW); 

    if (nmode == 1)
    {       // Algo selon le numéro de mode de contrôle
        PWMG = int ((pwmg*speedcoef)/16);
        PWMD = int ((pwmd*speedcoef)/16);
    }
    else if (nmode == 2)
    {
      if (dir==HIGH)
      {
       PWMG = int ((pwmd*speedcoef)/16);
       PWMD = int ((pwmg*speedcoef)/16);
      }
      else
      {
       PWMG = int ((pwmg*speedcoef)/16);
       PWMD = int ((pwmd*speedcoef)/16);
      }
    }
    else if (nmode == 3)
    {
       PWMG = int ((pwmd*speedcoef)/16);
       PWMD = int ((pwmg*speedcoef)/16);
    }
    else if (nmode == 4)
    {
      if (dir==HIGH)
      {
       PWMG = int ((pwmg*speedcoef)/16);
       PWMD = int ((pwmd*speedcoef)/16);
      }
      else
      {
       PWMG = int ((pwmd*speedcoef)/16);
       PWMD = int ((pwmg*speedcoef)/16);
      }
    }
    
  }
  else if (mode == LOW)
  {
    digitalWrite(ledBleue, LOW);
    digitalWrite(ledVerte, HIGH);

    pwm_min = min(pwmg, pwmd); // Mode solidaire contrôle les deux roues à la même vitesse selon la gâchette la moins pressée.
                               // Adapté pour les patients hémiplégiques (moitié gauche/droite du corps avec moins de contrôle moteur et/ou force).
    PWMG = int((pwm_min * speedcoef) / 16);
    PWMD = int((pwm_min * speedcoef) / 16);
  }

  if(dir == HIGH)
  {
    digitalWrite(ledRouge, LOW);
    digitalWrite(ledJaune, HIGH);
  }
  else
  {
    digitalWrite(ledRouge, HIGH);
    digitalWrite(ledJaune, LOW); 
  }
}

/*--- ÉCRAN ---*/
void checkpage()
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
} // Fin checkpage
void nextPopCallback(void *ptr)
{
  Serial.println("nextPopCallback");
  npage = 2;
  delay(200);
}
void prevPopCallback(void *ptr)
{
  Serial.println("prevPopCallback");
  npage = 0;
  delay(200);
}
void majMaintienPopCallback(void *ptr)
{
  if (wheelstopped)
  {
    Serial.println("Sequence fourchette");
    PoucMaintien.getValue(&pcmaintien);
    calcPoids();
    fourchetteConfig();
    msgmaintien.setText("");
    setFourchette();
  }
  else
  {
    Serial.println("Err. Freins releves !");
    msgmaintien.setText("Freins releves !");
  }
}
void majVitessePopCallback(void*ptr)
{
  Vitesse.getValue(&speedcoef);
  speedcoef /= 5.75;  // Facteur arbitraire ajouté pour limiter la vitesse du Zénith, qui était dangereux à vitesse max
  vitesseDesiree = speedcoef;
  msgvitesse.setText("La vitesse a ete mise a jour"); 
}
void okPopCallback(void *ptr)
{
  chr[0] = 0;
  Serial.print("ok ");
  delay(500);
  checkpage();
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
    msgpoids.setText("");
  }
  memset(chr, 0, 3);
  itoa((int)(POIDS + poidsOffset), chr, 10);
  poidspatient.setText(chr);
}
void bOnPopCallback(void *ptr)
{
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
 activ_poignees = HIGH;
}
void bMode2PopCallback(void *ptr)
{
 nmode = 2;
 MODE.setText("2");
 activ_poignees = HIGH;
}
void bMode3PopCallback(void *ptr)
{
 nmode = 3;
 MODE.setText("3");
 activ_poignees = HIGH;
}
void bMode4PopCallback(void *ptr)
{
 nmode = 4;
 MODE.setText("4");
 activ_poignees = HIGH;
}