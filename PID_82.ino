// Program sterujący zaworem centralnego ogrzewania  - Sylwester Kilian K.2016.S
// Biblioteka PID_v1I.h dostosowana do pracy w przerwaniu - ze standardowej biblioteki PID_v1.h
#include <PID_v1I.h> // zmodyfikowana biblioteka funkcji PID - działająca w przerwaniu
#include <TimerOne.h> // oryginalna biblioteka funkcji Timera1
//definicje stałych i zmiennych  ===============================================================================================================================================

// UWAGA!!! wszystkie temperatury podawać pomnożone przez 10

//*************************************************************************************************************************
volatile const double Kdo = 1.4;      // |1.45| współczynnik domu - zależne od parametrów cieplnych budynku (0.8 do 1.5)
volatile const double Tko = 30;       // |30| dodatkowa korekta do ch-ki domu - zależne od parametrów cieplnych budynku (0 do 40)
//****************************************************************************************************************************
// ustawienia PID  ...........................................................................................................
volatile const int dTagg = 40;        // |40| maksymalny bład powyżej którego parametry PID zostana przełaczone na agresywne
volatile const int kReg = 84;         // |84| korekta wyjscia PID dla błędu 0
volatile const double aggKp = 1;      // |1| ustawienia agresywne wsp. PID
volatile const double aggKi = 0;      // |0|
volatile const double aggKd = 50;     // |50|
volatile const double consKp = 0.45;  // |0.45| ustawienia normalne  wsp. PID
volatile const double consKi = 0.002; // |0.002|
volatile const double consKd = 60;    // |60|
double Setpoint = 200, Input = 200, Output = 0; // parametry  zmiennych funkcji PID
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE); // definicja parametrów funkcji PID
//      Stałe     ................................................................................................................
volatile const int dTpod = 15;                         // |17| graniczna odchyłka temperatury pokojowej  w dół, poniżej której nastąpi wymuszono korekta o Tkopo
volatile const int dTpog = 5;                          // |5| graniczna odchyłka temperatury pokojowej  w górę, powyżej której nastąpi wymuszone korekta o -Tkopo
volatile const int Tkopo = 50;                         // |50| wielkość korekty temperatury po przekroczeniu granicznej temperatury pokojowej w dół, lub w górę -2 stopnie
volatile const int Tod = 1366;                         // [4096-2730 = 1366] stała maksymalnej temp. przetwornika ADC przy założeniu, że Vref = 4,096 - w tym konkretnym modelu
const int Tzamax = 850, Totmax = 170 ;                 // |850|,|170| temperatury alarmowe: zasilania, otoczenia
const int kdTpowmax = 3;                               // |3| współczynnik, przez który dzielimy Setpoint -  wynik określa dopuszczalną różnicę między zasilaniem a powrotem
const int dTpowmin = 15;                               // |20| minimalna różnica Tza i Tpow aby pracowała pompa, gdy nie grzeje
const int Tzamin = 300;                                // |300| minimalna temp. zasilania przy której włączy się pompa - tylko piec
const int Ti = 20, Tp = 20 ;                           // |200|, |200| czas impulsu , przerwy dla diod
const byte  Jdl = 5, Jdh = 100;                        // |5|, |200| jasność diod mormalna i gdy korekta od temp. pokojowej
const unsigned int  Tb0 = 0, Tb1 = 9999 ;              // |0|, |9999| zdefiniowane (charakterystyczne) czasy buzera alarmu: 0-wył, 9999-ciągły
//      Zmienne    ................................................................................................................
volatile int Tot = 0, Tza = 400, Tpow = 300, Tpowd = 200; // zmienne temperatury: otoczenia, zasilania, powrotu, powrotu długookresowa
volatile int dTpow = 100, dTpowd = 150;                // zmienne różnicy zasilsnia i powrotu
volatile int Tpo = 200, Tpozad = 200, Tdo = 0;         // zmienne temperatury pokoju, zadanej temperatury pokoju i dodatkowej temp. pokoju
volatile int Tpopo = 0;                                // zmienna poprawki od granicznej temp. pokoju
volatile double  Tots = 0, Tzas = 300,  Tpos = 200 , Tpows = 200, Tpowds = 200, Tdos = 0, m = 0, n = 0, o = 0, p = 0; //zmienne do obliczanie średnich z temperatur i czasu
volatile int Reg = kReg, Bla = 0;                      // znormowane wyjście PID (0-200), błąd temperatury sterowania,
volatile byte Fpwm = 1;                                // flaga uruchomienia PWM
volatile byte Fpid = 0;                                // flaga obliczenia PID
volatile byte Fco = 0, Fcop = 0, Fcob = 0;             // flaga przełacznika C.O.: 1 gdy C.O. , 0 gdy piec ; flaga C.O pomocnicza przy rozruchu ; flaga buzzera
volatile int Th = 10 , Tl = 10  , L_H = 0;             // Th - zmienna "czasu" załączenia wyjścia PWM  , Tl - zmienna "czasu" wyłączenia wyjscia PWM, L_H - zmiennna wyjścia PWM
volatile byte Fala = 0;                                // flaga wysyłania alarmu
volatile byte Falc = 0;                                // flaga alarmu czujników temperatury
volatile unsigned long nT10 = 0;                       // zmienna liczby przerwań co 10 ms
unsigned long Tpi = 0, Tii = 0 ;                       // zmienna  "czasu" świecenia diod: przerwy, impulsu
unsigned long T5 = 0, T10 = 0, T45 = 0;                // zmienne odliczania czasu 5, 10, 30 minut
unsigned long TFco = 0;                                // zmienna czasu wstępnego załączenia grzałki zaworu C.O.
int  Lii = 5;                                          // zmienna liczby impulsów LED
byte Zol = 100, Zie = 255, Fp = 1, Jd = Jdl;           // zmienne zapalenia żółtej, zielonej, flaga przerwy, jasności świecenia diod
byte Buz = 0;                                          // zmienna stanu Buzzera
byte St = 1;                                           // bit startowy - jednokrotnie przy uruchomieniu
unsigned int T1ib = 20, T1pb = 40, T2ib = 10, T2pb = 200; //zmienne czasów buzera alarmu
unsigned int Isys = 0; byte Psys = 0;                  // zmienne dla diody systemowej
//    Przypisenie wejść i wyjść    .................................................................................................
const int TOT_pin = A1, TZA_pin = A0, TPO_pin = A3, TPOW_pin = A2, TDO_pin = A4; // piny odczytu temperatury: otoczenia, zasilania, pokojowej, powrotu, dodatkowej
const int L_H_pin = 2 ;                                // pin wyjściowy PWM
const int ZIE_pin = 3, ZOL_pin = 5;                    // piny diod
const int BUZ_pin = 4;                                 // pin buzzera
const int PUMP_pin = 7;                                // pin pompy
const int ALA_pin = 6;                                 // pin alarmu
const int SYS_pin = 13;                                // pin diody systemowej
const int CO_pin = 8;                                  // pin przełącznika C.O. / piec

// koniec definicji ===========================================================================================================================================
// ustawienia =================================================================================================================================================
void setup() {
  myPID.SetTunings(consKp, consKi, consKd);                    // ustawienie parametrów PID
  myPID.SetOutputLimits(-80, 85);                              // |-80,85| Zakres wyjścia PID
  myPID.SetSampleTime(2000);                                   // [2000]ustalenie okresu PID w ms. Uwaga, musi być taki jaki wynika z czasu i ilości przerwań
  myPID.SetMode(AUTOMATIC);                                    //załączenie funkcji PID
  Timer1.initialize(10000);                                    //[10000] inicjalizacja Timera 1 - przerwanie co 10 ms
  Timer1.attachInterrupt(PWM);                                 // przypisanie procedury PWM() do przerwania od Timera 1
  pinMode(L_H_pin, OUTPUT); digitalWrite(L_H_pin, HIGH);       // aktywacja wyjścia PWM
  pinMode(ZIE_pin, OUTPUT); analogWrite(ZIE_pin, Jdh);         // aktywacja wyjść dla diod
  pinMode(ZOL_pin, OUTPUT); analogWrite(ZOL_pin, Jdh);         //  wyjść dla diod
  pinMode(BUZ_pin, OUTPUT); digitalWrite(BUZ_pin, HIGH);       // aktywacja wyjścia buzzera
  pinMode(PUMP_pin, OUTPUT); digitalWrite(PUMP_pin, HIGH);     // aktywacja wyjścia pompy
  pinMode(ALA_pin, OUTPUT); digitalWrite(ALA_pin, HIGH);       // aktywacja wyjścia alarmowego
  pinMode(SYS_pin, OUTPUT); digitalWrite(SYS_pin, HIGH);       // aktywacja wyjścia dla diody systemowej
  pinMode(CO_pin, INPUT);                                      // aktywacja wejścia przełacznika C.O./piec
  analogReference(EXTERNAL);                                   // ustawienie napięcia odniesienia na zewnętrzne (4,096V)

  // Serial.begin(9600);                                       //aktywacja portu szeregowego - tylko do testów
}
// koniec ustawień ============================================================================================================================================
//Pętla główna ================================================================================================================================================
void loop()
{
  ++Isys ;              // miganie diodą systemową - 1sek świecenia to 0,1 msek pętli głównej
  if (Isys == 10000) {
    Psys = (Psys + 1) % 2;
    digitalWrite(SYS_pin, Psys);
    Isys = 0;
  }
  // obsługa startu ...........................................................................................................................................
  if ( St == 1) {       // start systemu - efekty audiowizualne
    delay(2500);
    digitalWrite(BUZ_pin, LOW);
    analogWrite(ZIE_pin, Jdl);
    analogWrite(ZOL_pin, Jdl);
    delay(2500);
    Jd = Jdh;
    Tot = map (analogRead(TOT_pin), 0, 1023, -2730, Tod);// wstępny odczyt temperatur
    Tza = map (analogRead(TZA_pin), 0, 1023, -2730, Tod);
    Tpo =  map (analogRead(TPO_pin), 0, 1023, -2730, Tod);
    Tpow = map (analogRead(TPOW_pin), 0, 1023, -2730, Tod);
    Tdo = 200 - map (analogRead(TDO_pin), 0, 1023, -2730, Tod);
    delay(500);
    Tpozad = 200 + Tdo;
    Tpowd = Tpow;
    dTpow = Tza - Tpow;
    dTpowd = dTpow;
    St = 0;
  }
  // wydruki testowe - obowiązkowo wyłacz po testach...........................................................................................................
  /*
    Serial.print(F("dTpowd=   "));
    Serial.println( dTpowd);
    Serial.println(".............");
    Serial.print(F("Tpow=   "));
    Serial.println( Tpow);
    Serial.print(F("Tza=   "));
    Serial.println(Tza);
    Serial.print(F("Tot=   "));
    Serial.println(Tot);
    Serial.print(F("Tpozad=   "));
    Serial.println(Tpozad);
    Serial.print(F("Tpo=   "));
    Serial.println(Tpo);
    Serial.println(".............");
    Serial.print(F("Setpoint=   "));
    Serial.println(Setpoint);
    Serial.print(F("Reg=   "));
    Serial.println(Reg);
    Serial.print(F("Tod==   "));
    Serial.println(Tod);
    Serial.print(F("Tpopo=   "));
    Serial.println(Tpopo);
    delay(10);
  */
  // odczyty wstępne do alarmów .........................................................................................................................................
  Fco = digitalRead(CO_pin);  // odczyt stanu przełacznika C.O. - piec
  dTpow = Tza - Tpow;        // różnica miedzy zasilaniem a powrotem
  dTpowd = Tza - Tpowd;      // różnica miedzy zasilaniem a powrotem, średnia z 5-ciu minut
  // obsługa alarmów ....................................................................................................................................................
  if (Buz == 0) {
    Jd = Jdh;
    Fala = 0;
    Falc = 0;
    T1ib = Tb0;
    T1pb = Tb0;
    T2ib = Tb0;
    T2pb = Tb0;
    if (Fco == 1 && ((Tot >= Totmax && Tpopo <= 0) || Tza >= Tzamax || Bla < -70 || Setpoint < Tpo + 70 || Falc == 1))  { // zamknięcie zaworu - patrz PWM()- tylko C.O.
      Fpwm = 0;
      Jd = Jdl;                                                          // diody świecą słabo
    }
    else {
      if ( Fco == 1 && Tot < Totmax && Tza < Tzamax  && Bla >= -70 &&  Setpoint >= (Tpo + 70) && Falc == 0) { // otwarcie zaworu - tylko C.O.
        if (Fpwm == 0 && Fcop == 1) {                                     // jeżeli uruchomienie po alarmie to zeruj PID
          noInterrupts();
          delay(25);
          myPID.SetMode(MANUAL);
          Input = Tza;
          Setpoint = (Tpozad + Tpopo - Tot) * Kdo + Tpozad + Tko;
          Output = 0;
          myPID.SetMode(AUTOMATIC);
          interrupts();
        }
        Fpwm = 1;                    // otwarcie zaworu (patrz PWM())
        Jd = Jdh;                    // diody świecą jasno
      }
    }
    if (Tpopo < 0) {
      T1ib = 10; T1pb = 10; T2ib = 40; T2pb = 2000;                  // gdy temp. pomieszczenia za wysoka o dTpog to  buzzer [. -]
    }
    if (Tpopo > 0) {
      T1ib = 40; T1pb = 10; T2ib = 10; T2pb = 2000;                  // gdy temp. pomieszczenia za niska o dTpod to  buzzer [- .]
    }
    if ( Bla < -70) {
      T1ib = 40; T1pb = 10; T2ib = 40; T2pb = 500;                   // gdy temp. zasilania o wiecej niż 7 stopni większa od zadanej to  buzzer [- -]
      if (Fco == 1) {                                                // dla C.O. dodatkowo alarm
        Fala = 1;
      }
    }
    if (dTpowd <= dTpowmin && Bla > 50 && Fpwm == 1 && Fco == 1) {   // jeżeli włączony a  nie grzeje przez 10 minut to  buzzer [-] i alarm, tylko dla C.O.
      if (nT10 > T10 + 60000) {
        T1ib = 60; T1pb = 1000; T2ib = Tb0;
      }
    }
    else {
      T10 = nT10;     // zerowanie licznika 10 minut
    }

    if (dTpow >= Tza * 1.5 / kdTpowmax ) { // gdy za duża różnica między zasilaniem a powrotem (krótko)  to buzzer [.] i alarm
      T1ib = 10; T1pb = 1500; T2ib = Tb0; Fala = 1;
    }
    if (dTpowd >= Tza / kdTpowmax ) { // gdy za duża różnica między zasilaniem a powrotem  przez 45 min. to buzzer [..] i alarm
      if (nT10 > T45 + 270000) {
        T1ib = 10; T1pb = 10; T2ib = 10; T2pb = 500; Fala = 1;
      }
    }
    else {
      T45 = nT10;      // zerowanie licznika 45 minut
    }
    if (Tot >= Totmax && Tpopo <= 0) {                                   // gdy temperatura otoczenia przekroczy max i nie trzeba grzać - ciągły buzzer
      T1ib = Tb1;
    }
    if (Tza >= Tzamax)  {
      T1ib = 20; T1pb = 10; T2ib = 20; T2pb = 10; Fala = 1;              // gdy zasilanie przekroczy max - buzzer i alarm
    }
    // Alarm od przerwy/zwarcia czujników temperatury  .................................................................................................................
    if ( Tot > 500 || Tot < -500 || Tpo > 500 || Tpo < 0 || Tza > 1200 || Tza < 0 || Tpow > 1200 || Tpow < 0 || Tdo > 100 || Tdo < -100) {
      Falc = 1;
      Fala = 1;
      analogWrite(ZOL_pin, 0);
      analogWrite(ZIE_pin, 0);
      delay(100);
      analogWrite(ZOL_pin, 255);
      analogWrite(ZIE_pin, 255);
      delay(100);
      analogWrite(ZOL_pin, 0);
      analogWrite(ZIE_pin, 0);
      T1ib = 10; T1pb = 10; T2ib = 10; T2pb = 10;
    }
  }
  // Obsługa przełącznika C.O. - PIEC ................................................................................................................................
  if (Fcob != Fco) {
    delay(1300);
    Buz = 0;
    T1ib = 300; T1pb = 300; T2ib = Tb0; T2pb = Tb0;               //  jednokrotny buzzer po zmianie stanu przełacznika C.O. / piec
    Fcob = Fco;
    if (Fco == 1) {
      Fcop = 0;
      TFco = nT10;
    }
  }
  if (Fco == 1 && Fcop == 0) {
    if ((nT10 < 12000 + TFco) || (dTpow > dTpowmin)) {    // gdy rozruch C.O. to załącz ciągłe grzanie zaworu C.O. przez minimum 2 min. aż do momentu gdy zawór się zamknie
      Fpwm = 0;
      Jd = 0;
      if (Buz == 0) {
        T1ib = 10; T1pb = 50; T2ib = Tb0; T2pb = Tb0;
      }
    }                                              // zakończenie wstępnego grzania zaworu C.O.
    else {
      Fcop = 1;
      delay(1300);
      Jd = Jdh;
      Buz = 0;
      T1ib = 300; T1pb = 300; T2ib = Tb0; T2pb = Tb0;     //  jednokrotny długi buzzer po zakończeniu wstępnego grzania zaworu C.O.

    }
  }
  //obsługa pompy ..............................................................................................................................................................
  if (Fco == 1 && (Fpwm == 0 && dTpow <= dTpowmin && Fcop == 1 && Falc == 0 && digitalRead (PUMP_pin) == HIGH))  { // gdy nie grzeje, nie jest w fazie rozruchu, sprawne czujniki i zasilanie prawie jak powrót to wyłącz pompę - dla C.O.
    digitalWrite (PUMP_pin, LOW);
  }
  if (Fco == 1 && ((dTpow > dTpowmin + 10  || Fpwm == 1 || Fcop == 0 || Falc == 1)  && digitalRead (PUMP_pin) == LOW)) { // gdy zasilanie wyższe od powrotu, grzeje, niesprawne czujniki lub rozruch, to załącz pompę - dla C.O.
    digitalWrite (PUMP_pin, HIGH);
  }
  if (Fco == 0 && (dTpow < dTpowmin)  &&  digitalRead (PUMP_pin) == HIGH) { // gdy  zasilanie prawie jak powrót   to wyłącz pompę - dla pieca
    digitalWrite (PUMP_pin, LOW);
  }
  if ( Fco == 0 && (Tza > Tzamin || dTpow > dTpowmin + 10) && digitalRead (PUMP_pin) == LOW) { // gdy temp. pieca większa od Tzamin i piec grzeje to załącz pompę - dla pieca
    digitalWrite (PUMP_pin, HIGH);
  }
  // wywołania podprogramów .......................................................................................................................................
  Buzzer();   // wywołanie Buzzera
  if (Falc == 0) {
    LEDi();     // wywołanie generacji błysków LED
  }
}
// koniec pętli głownej =======================================================================================================================================
// procedura przerwania ========================================================================================================================================
void PWM()
{
  ++nT10;            // zwiększenie licznika przerwań o 1 (10ms)
  // obliczenia PID ..................................................................................................................................................
  // uśrednienie odczytów temperatur
  Fpid = 0;
  Tots += analogRead(TOT_pin); // Pomiar napięć z czujników temperatury i dodatkowej z  potencjometru
  Tzas += analogRead(TZA_pin) ;
  Tpos += analogRead(TPO_pin);
  Tpows += analogRead(TPOW_pin);
  Tdos += analogRead(TDO_pin);
  ++m;
  if (m == 200) {       //po dwóch sekundach
    Tza = map (Tzas / 200, 0, 1023, -2730, Tod); Tzas = 0; //średnia z 2 sekund
    ++n;
    if (n == 5) {       // po 10-ciu sekundach
      Tpowds +=  Tpows;
      Tpow = map (Tpows / 1000, 0, 1023, -2730, Tod); Tpows = 0;

      ++o;
      if (o == 6) {    // po 1 minucie
        Tot = map (Tots / 6000, 0, 1023, -2730, Tod); Tots = 0;
        Tpo = map (Tpos / 6000, 0, 1023, -2730, Tod); Tpos = 0;
        Tpowd = map (Tpowds / 6000, 0, 1023, -2730, Tod); Tpowds = 0;
        Tdo =  200 - map (Tdos / 6000, 0, 1023, -2730, Tod); Tdos = 0; //przeskalowanane do 20 stopni. Uwaga - ujemna dodaje, dodatnia odejmuje
        Tpozad = 200 + Tdo;
        // sprawdzenie czy nie przkroczona graniczna temperatura  w pokoju ...........................................................................................
        if (( Tpozad - Tpo ) >= dTpod ) { // jeżeli poniżej zakładanej temp. pokoju o dTpod to grzej o Tkopo -2 stopnie więcej
          Tpopo = Tkopo - 20;
        }
        if ((Tpo - Tpozad ) >= dTpog ) { // jeżeli powyżej zakładanej temp. pokoju o dTpog to grzej o Tkopo mniej
          Tpopo = -Tkopo;
        }
        if (((5 + Tpozad - Tpo ) <= dTpod ) && ((Tpo - Tpozad + 5 ) <= dTpog )) { // jezeli w granicach to wyłącz korektę
          Tpopo = 0;
        }
        o = 0;
      }
      n = 0;
    }
    //  koniec sprawdzenia czy nie przkroczona graniczna temperatura  w pokoju .....................................................................................
    // obliczenia PID ............................................................................................................................................
    Input = Tza; // podstawienie temperatury zasilania dla funkcji PID
    Setpoint = (Tpozad + Tpopo - Tot) * Kdo + Tpozad + Tko; // obliczenie temperatury zadanej(Setpoint)
    myPID.Compute(); //odczyt PID
    Fpid = 1;
    Reg = Output + kReg;  //odczyt i przeskalowanie wyjścia z PID
    Bla = (Setpoint - Input); //aktualny błąd w dziesiątych częściach stopnia
    //zmiana parametrów PID w zależności od wielkości błędu
    if (abs(Bla) <= dTagg) {
      myPID.SetTunings(consKp, consKi, consKd); // ustawiamy normalne parametry PID
    }
    else {
      myPID.SetTunings(aggKp, aggKi, aggKd); // ustawiamy agresywne parametry PID
    }
    m = 0; // zerowanie licznika przerwań
  }
  // koniec obliczeń PID ...........................................................................................................................................
  //PWM dla zaworu .................................................................................................................................................
  if (Fpwm == 1) {      //gdy nie jest wymuszone zamknięcie zaworu to nim reguluj
    if (L_H == 1) {     //gdy trwa impuls
      if (Th > 1) {
        --Th;
      }
      else {            // gdy skończył się impuls
        L_H = 0;
        digitalWrite(L_H_pin, LOW);
      }
    }
    else {
      if (Tl > 1) {     // gdy trwa przerwa
        --Tl;
      }
      else {             // gdy skończyła się przerwa
        if (Fpid == 1) { // gdy było obliczenie PID (synchronizacja)
          Tl = 200 - Reg;
          Th = Reg;
          L_H = 1;
          digitalWrite(L_H_pin, HIGH);
        }
      }
    }
  }
  else {
    digitalWrite(L_H_pin, HIGH); // gdy wymuszone zamknięcie zaworu(alarmy) to go zamknij
  }
  // koniec PWM dla zaworu .....................................................................................................................................
}
//koniec procedury przerwania PWM()============================================================================================================================
//Procedura zapalania diod ====================================================================================================================================
void LEDi() {
  if (Zie > 0) {
    Zie = Jd;
  }
  if (Zol > 0) {
    Zol = Jd;
  }
  if ( Lii <= 0) {
    Lii = abs(Bla / 10);
    if (Bla >= 0) {           //gdy temperatura zasilania za niska
      Zol = 0;
      Zie = Jd;
    }
    else {                    //gdy temperatura zasilania za zwysoka
      Zol = Jd ;
      Zie = 0;
    }
    if (Lii < 1) {            //gdy bład mniejszy od 1 to świecą obie
      analogWrite(ZOL_pin, Jd);
      analogWrite(ZIE_pin, Jd);
      Fp = 0;
      Lii = 0;
      return;
    }
    if (Lii > 5 ) {            //gdy bład większy od 5 to świeci ciągle
      analogWrite(ZOL_pin, Zol );
      analogWrite(ZIE_pin, Zie);
      Fp = 0;
      Lii = 0;
      return;
    }
    analogWrite(ZOL_pin, Zol); // gdy Lii z zakresu 1-5 to początek pierwszego impulsu
    analogWrite(ZIE_pin, Zie);
    Fp = 0;
    Tii = nT10;
    return;
  }
  if (Fp == 0) { //trwa impuls
    if (nT10 - Tii >= Ti) {
      analogWrite(ZOL_pin, 0);
      analogWrite(ZIE_pin, 0);
      Fp = 1;
      Tpi = nT10;
      return;
    }
    return;
  }
  if (Fp == 1) {  // trwa przerwa
    if (Lii == 1) {
      if (nT10 - Tpi >= 4 * Tp) { // ostatnia, wydłużona przerwa
        Lii = 0;
        return;
      }
      return;
    }
    if (nT10 - Tpi >= Tp) {       // przerwa między impulsami
      analogWrite(ZOL_pin, Zol);
      analogWrite(ZIE_pin, Zie);
      Fp = 0;
      Tii = nT10;
      --Lii;
      return;
    }
    return;
  }
}
//koniec procedury impulsów diod LED  =========================================================================================================================
// Procedura  dźwięku Buzzer'a ================================================================================================================================
void Buzzer() {
  static unsigned long Tpbp;  // czas przerwy buzzera poczatkowy
  static unsigned long Tibp;  // czas impulsu buzzera poczatkowy
  if (T1ib == Tb0) {                    // bez dźwięku
    Buz = 0;
    digitalWrite(BUZ_pin, LOW);
    digitalWrite(ALA_pin, LOW);
    return;
  }
  if (T1ib == Tb1 ) {                   // ciągły dźwięk
    Buz = 0;
    digitalWrite(BUZ_pin, HIGH);
    digitalWrite(ALA_pin, HIGH && Fala);
    return;
  }
  if ( Buz == 0) {                      // początek pierwszego impulsu
    digitalWrite(BUZ_pin, HIGH);
    digitalWrite(ALA_pin, HIGH && Fala);
    Tibp = nT10;
    Buz = 1; return;
  }
  if (Buz == 1) {
    if (nT10 -   Tibp > T1ib) {        // odliczanie czasu pierwszego impulsu
      digitalWrite(BUZ_pin, LOW);
      digitalWrite(ALA_pin, LOW);
      Tpbp = nT10;
      Buz = 2;
      return;                          // początek pierwszej przerwy
    }
  }
  if (Buz == 2) {
    if (nT10 -   Tpbp > T1pb) {       // odliczanie czasu pierwszej przerwy
      digitalWrite(BUZ_pin, HIGH);
      digitalWrite(ALA_pin,
                   HIGH && Fala); Tibp = nT10;
      if (T2ib == Tb0) {
        Buz = 0;
      }
      else {
        Buz = 3;
      }
      return;                        // poczatek drugiego sygnału
    }
  }
  if (Buz == 3) {
    if (nT10 - Tibp > T2ib) {         // odliczanie czasu drugiego impulsu
      digitalWrite(BUZ_pin, LOW);
      digitalWrite(ALA_pin, LOW);
      Tpbp = nT10; Buz = 4;
      return;                         // poczatek drugiej przerwy
    }
  }
  if (Buz == 4) {
    if (nT10 - Tpbp > T2pb) {         // odliczanie czasu drugiej przerwy
      Buz = 0;
      return;                         // koniec cyklu buzzera
    }
  }
}
// koniec procedury  dźwięku Buzzer'a =========================================================================================================================



