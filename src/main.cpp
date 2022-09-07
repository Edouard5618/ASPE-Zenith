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
  attachInterrupt(digitalPinToInterrupt(EncoderRW),acquisitionEncoder,FALLING);
  attachInterrupt(digitalPinToInterrupt(EncoderLW),acquisitionEncoder,FALLING);
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
  Encoder1.initEncoder();
  Encoder2.initEncoder();
  Encoder1.clearEncoderCount(); // Clear Encoder
  Encoder2.clearEncoderCount();
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
  nexLoop(nex_listen_list);

  switch (npage)
  {       // Algo selon le numéro de page de l'écran
  case 1: // Interface normale

    if ((millis() - jerkTimer) > jerkTime)
    { // limitation de la fréquence de m-à-j des PWMs
      vitesseDroiteReelle = newEncoder3Steps*6.283185*rayonRoue*3.6/(EncoderHoleQuantity*acquisitionTime/1000);
      Serial.println(newEncoder3Steps);
      newEncoder3Steps=0;
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
    analogWrite(mg, abs(brightnessG));
    analogWrite(md, abs(brightnessD));
    analogWrite(vg, abs(brightnessVG));
    analogWrite(vd, abs(brightnessVD));
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
      vitesseDroiteReelle = newEncoder3Steps*6.283185*rayonRoue*3.6/(EncoderHoleQuantity*acquisitionTime/1000);
      Serial.println(newEncoder3Steps);
      newEncoder3Steps=0;
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
    analogWrite(mg, abs(brightnessG));
    analogWrite(md, abs(brightnessD));
    analogWrite(vg, abs(brightnessVG));
    analogWrite(vd, abs(brightnessVD));

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
void releaseBreaks()
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

    directiontemp = dir; 

    if(directiontemp != dir){
      delay(25);
      directiontemp = digitalRead(swit);

      if(directiontemp != dir){
        dir = directiontemp;
      }
    }

    modetemp = mode;

    if (modetemp != mode)
    {
      delay(25);
      modetemp = digitalRead(boutr);

      if (modetemp != mode)
      {
        mode = modetemp;
      }
    }

    delay(15);
  }
  // Moteur côté controleur (Même fonction que moteur côté batterie, donc seule la section côté contrôleur sera commentée)
  windD = (PWMD - brightnessD) / 2;
  if (windD > 3)
  {
    windD = 3; // Limite le gain positif du PWM côté contrôleur (brightnessD) à 3.
    brightnessD += windD;
  }
  else if (windD < -3)
  {
    windD = -3; // Limite le gain négatif du PWM côté contrôleur (brightnessD) à 3.
    brightnessD += windD;
  }
  else if ((abs(windD) < 1) && (abs(PWMD) < 1))
  {
    windD = 0;
    brightnessD = 0;
  }
  else
  {
    brightnessD += windD;
  }
  if ((brightnessD < 0) || ((dir == HIGH) && (commande == 500000000)))
  { // Décision logique pour la direction des roues motrices, brightnessD vient de la manette et dir vient des poignées.
    digitalWrite(dmd, LOW);
  }
  else
  {
    digitalWrite(dmd, HIGH);
  }
  // Moteur côté batterie
  windG = (PWMG - brightnessG) / 2;
  if (windG > 3)
  {
    windG = 3;
    brightnessG += windG;
  }
  else if (windG < -3)
  {
    windG = -3;
    brightnessG += windG;
  }
  else if ((abs(windG) < 1) && (abs(PWMG) < 1))
  {
    windG = 0;
    brightnessG = 0;
  }
  else
  {
    brightnessG += windG;
  }
  if ((brightnessG < 0) || ((dir == HIGH) && (commande == 500000000)))
  {
    digitalWrite(dmg, LOW);
  }
  else
  {
    digitalWrite(dmg, HIGH);
  }
  // Frein  
  if (!wheelstopped && (((abs(windG) + abs(brightnessG) + abs(windD + brightnessD)) == 0) || (commande >= 200000000) || (commande < 100000000)) && (commande != 500000000))
  {
    // Actionne les freins des roues motrices si celles-ci ne sont pas déjà arrêtées et qu'il n'y a plus de commandes provenant de la manette ET des poignées.
    releaseBreaks();
  }
  else if (wheelstopped && ((abs(windG) + abs(brightnessG) + abs(windD) + abs(brightnessD)) != 0) && (((commande >= 100000000) && (commande < 200000000)) || (commande == 500000000)))
  {
    // Désactive les freins des roues motrices si les roues sont déjà arrêtées et que le Arduino reçoit des commandes provenant des poignées OU de la manette.
    riseBreaks();
  }
} // fin motorDriveCalc
void machine_stop() 
{
  PWMD = 0;
  PWMG = 0;
  PWMVG = 0;
  PWMVD = 0;
  watchdog = 0;
}

void acquisitionEncoder()
{
  newEncoder3Steps = newEncoder3Steps+1;
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
  float angle = lectureAngle()*1000;
  // Serial.print(ID);
  // Serial.println(angle);
  delay(60);
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
  encoder1ReadingPrev = encoder1Reading;
  encoder2ReadingPrev = encoder2Reading;
  encoder1Reading = Encoder1.readEncoder();
  encoder2Reading = Encoder2.readEncoder();
  if ((abs(brightnessVG) > 40) || (abs(brightnessVD) > 40))
  {
    if (!abs(encoder1ReadingPrev - encoder1Reading) && !abs(encoder2ReadingPrev - encoder2Reading))
    {
      if (lmtTrigg > 10)
      {
        Encoder1.clearEncoderCount();
        Encoder2.clearEncoderCount();
        encoder1ReadingPrev = 0;
        encoder2ReadingPrev = 0;
      }
      else
      {
        lmtTrigg++;
      }
    }
    else
    {
      lmtTrigg = 0;
      asserVerins();
    }
  }
}
void verinDriveCalc()
{
  windVG = (PWMVG - brightnessVG) / 2;
  if (windVG > 10)
  {
    windVG = 10;
    brightnessVG += windVG;
  }
  else if (windVG < -10)
  {
    windVG = -10;
    brightnessVG += windVG;
  }
  else if ((abs(windVG) < 2) && (abs(PWMVG) < 2))
  {
    windVG = 0;
    brightnessVG = 0;
  }
  else
  {
    brightnessVG += windVG;
  }
  if (brightnessVG < 0)
  {
    digitalWrite(vdg, LOW);
    digitalWrite(vdd, LOW);
  }
  else
  {
    digitalWrite(vdg, HIGH);
    digitalWrite(vdd, HIGH);
  }
  windVD = (PWMVD - brightnessVD) / 2;
  if (windVD > 10)
  {
    windVD = 10;
    brightnessVD += windVD;
  }
  else if (windVD < -10)
  {
    windVD = -10;
    brightnessVD += windVD;
  }
  else if ((abs(windVD) < 2) && (abs(PWMVD) < 2))
  {
    windVD = 0;
    brightnessVD = 0;
  }
  else
  {
    brightnessVD += windVD;
  }
  if (brightnessVD < 0)
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
  if (btnmanuel >= (referenceBouton-10))
  {
    if (btnreleased == 0)
    {
      PWMVG = 0;
      PWMVD = 0;
      btnreleased = 1;
    }
  }
  else if (btnmanuel > (10))
  {
    PWMVG = liftCoef;
    PWMVD = liftCoef;
    btnreleased = 0;
  }
  else
  {
    PWMVG = -liftCoef;
    PWMVD = -liftCoef;
    btnreleased = 0;
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
  long lapStart = millis();
  while ((millis() - lapStart) < lap)
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
  if ((brightnessG + brightnessD + brightnessVG + brightnessVD) == 0)
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
  commande = data.toInt();

  if ((commande < 200000000) && (commande > 100000000)) {
    // riseBreaks();
    PWMVG = 0;
    PWMVD = 0;
    // JogX et JogY sont les valeurs du joystick de la manette, sur 255 signe
    JogX = (commande % 100000000) / 10000; // Recuperer la valeur vrx
    JogX -= 1511; // Soustraire le 1000 ajoute et centrer sur 511
    JogY = commande % 10000;
    JogY -= 1511;
    // Rotation pi/4 pour vitesses moteur ** Utiliser abs pour le PWM
    pwmD_Value = -0.707 * float(JogX + JogY);
    pwmG_Value = 0.707 * float(JogY - JogX);
    pwmD_Value = pwmD_Value / 512 * speedcoef * jogCoef; // Scale 255 selon limite jogCoef, pour PWM
    pwmG_Value = pwmG_Value / 512 * speedcoef * jogCoef;
    PWMD = int(pwmD_Value);
    PWMG = int(pwmG_Value);

    Serial2.print(PWMD);
    Serial2.print(",");
    Serial2.println(PWMG);

  } else if (commande == 200000000) {
    PWMD = 0;
    PWMG = 0;
    PWMVG = liftCoef;
    PWMVD = liftCoef;

  } else if (commande == 300000000) {
    PWMD = 0;
    PWMG = 0;
    PWMVG = -liftCoef;
    PWMVD = -liftCoef;
  } else {
    //Err
    Serial.println("Erreur 200");
    // String msgString = "Erreur 200, " + String(commande);
    // msgspeed.setText(msgString.c_str());
    machine_stop();
  }
  watchdog = 1;
  watchdogTimer = millis();
}  // Fin process_data
void processIncomingByte ()
{
  String IDcommande = ""; 
  String commandeS = "" ;
  //angle; 

  bool IDdone = false;

  while(0 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    if(c == '&')
    {
      IDdone = (true);
    }
    else if (!IDdone)
    {
      IDcommande = IDcommande +c;
    }
    else
    {
      commandeS = commandeS + c;
    }
  }
  if(String(IDcommande) == String(IDattendu))
  { 
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
  commande = 500000000; // Sert à désactiver les freins électroniques aux roues motrices
  // Lecture des gachettes aux poignées (potentiomètre). Plage de conversion entre 610-350 pour gauche et 900-670 (branché à l'envers) pour s'adapter à la rotation limitée des gachettes. Finit à 100/970 (et non 0/1023)
  // pour éviter des signaux parasites si la gachette n'est pas parfaitement dépressée.
  pwmg = map(smoothing(potg), 145, 0, 0, 255);
  //Serial.println("PWMG: "+ pwmg);
  if (pwmg < 50) // Si la valeur du potentiomètre sort de la plage
  { 
    pwmg = 0;
  }
  else if (pwmg > 255)
  {
    pwmg = 255;
  }

  pwmd = map(smoothing(potd), 135, 320, 0, 255);

   if (pwmd < 50) // Si la valeur du potentiomètre sort de la plage
  { 
    pwmd = 0;
  }
  else if (pwmd > 255)
  {
    pwmd = 255;
  }
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
  //Serial.println(PWMG);

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
  commande = 0;
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