#include "arduinoFFT.h"
#include "LiquidCrystal_I2C.h"

// Canalul ADC pentru citirea semnalelor audio de la microfon
#define CHANNEL A0
// SNUmarul de esantioane pentru FFT
const uint16_t samples = 128;
// Frecventa de esantinare
const double samplingFrequency = 2000;
// Perioada de esantionare in ms
unsigned int sampling_period_us;
// microsecunde
unsigned long microseconds;
// Threshold-ul pentru detectarea frecventei
unsigned int threshold = 200;
// Intervalul de deviatie acceptabil in jurul frecventei tinta
const float frequencyRange = 40.0;
// Dimensiunea pasului pentru luminarea LED-urilor
const float frequencyIncrement = 5.0;

// Partea reala pentru stocarea inputului FFT
double vReal[samples];
// Partea imaginara pentru stocarea inputului FFT
double vImag[samples];

// Interfata I2C pentru displayul LCD-ului
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Pinii GPIO pentru ON/OFF (Intrerupere)
const int onOffButtonPin = 2; 
// Pinii GPIO pentru ciclu (Intrerupere)
const int cycleButtonPin = 3; 

// Starea tunerului (pornit/oprit)
volatile bool isOn = false;
// Indexul pentru frecventa tinta curenta
volatile int targetFrequencyIndex = 0;
// Lista frecventelor tinta pentru fiecare nota in parte
int targetFrequencies[] = { 82, 110, 147, 196, 247, 330 };

// CObiectul FFT creat
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// Constants for printing vector types
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup() {
  // Perioada de esantionare in microsecunde
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  // Comunicarea seriala penrtu debugging
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Ready");

  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Configurarea pinilor GPIO pentru butoane
  pinMode(onOffButtonPin, INPUT_PULLUP);
  pinMode(cycleButtonPin, INPUT_PULLUP);

  // Intreruperi pentru cele 2 butoane pe front crescator
  attachInterrupt(digitalPinToInterrupt(onOffButtonPin), onOffButtonISR, RISING);
  attachInterrupt(digitalPinToInterrupt(cycleButtonPin), cycleButtonISR, RISING);
}

void loop() {
  if (isOn) {
    // Tuner pornit
    // Citeste semnalele audio
    sample();
    delay(50);
  } else {
    // Stinge toate ledurile conectate
    clearLEDs();
    lcd.clear();
    lcd.setCursor(0, 0);
    // Afiseaza mesajul Waiting OFF pentru a semnala ca tunerul e pornit
    lcd.print("Waiting OFF");
    delay(50);
  }
}

// Stinge toate ledurile conectate
void clearLEDs() {
  // Itereaza prin toti pinii de la 5 la 13
  for (int pin = 5; pin <= 13; pin++) {
    digitalWrite(pin, LOW);
  }
}

// Preia detele audio pe care efectueaza FFT
void sample() {
  // Initializeaza timpul de esantionare
  microseconds = micros();
  
  // Citirea esantioanelor audio
  for (int i = 0; i < samples; i++) {
    // Cititrrea valorii ADC la microfon
    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    // Pana cand perioada de esantionare e completa
    while (micros() - microseconds < sampling_period_us) {}
    microseconds += sampling_period_us;
  }

  // Efectuarea FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);

  // Convertirea valorilor complexe la magnitudine
  FFT.complexToMagnitude();
  
  // Filtrarea sunetului (frecvente neimportante)
  for (uint16_t i = 0; i < (samples >> 1); i++) {
    if (vReal[i] < threshold || 
    (vReal[i] > targetFrequencies[targetFrequencyIndex] - frequencyRange && vReal[i] < targetFrequencies[targetFrequencyIndex] + frequencyRange)) {
      vReal[i] = 0.0;
    }
  }
  
  // Frecventa principala
  double x = FFT.majorPeak();
  if (x > 20 && x < 980) {
    Serial.println(x, 6);
    // Actualizarea LED-urilor in functie de frecevnta
    mapFrequencyToLEDs(x); 
    // Actualizarea LCD-ului dupa frecventa
    displayFrequency(targetFrequencies[targetFrequencyIndex]);
  } else {
    Serial.println("Waiting");
  }
}

// Mapeaza frecventa detectata la led-urui
void mapFrequencyToLEDs(double dominantFrequency) {
  // Opreste toate ledurile
  clearLEDs();

  // Deviatia frecventei in functie de target
  float deviation = dominantFrequency + frequencyRange - targetFrequencies[targetFrequencyIndex];
  
  // Index led ce trebuie aprins
  int ledIndex = round(deviation / frequencyIncrement) - 4;
  
  Serial.println("Deviation: ");
  Serial.print(deviation);
  Serial.print(", ledIndex: ");
  Serial.print(ledIndex);

  ledIndex = constrain(ledIndex, 0, 8);
  Serial.print(", constrained ledIndex: ");
  Serial.print(ledIndex);
  Serial.print("\n\n");

  // Determinarea pinului GPIO corespunzator LED-ului
  int pin = ledIndex + 5;
  
  // Aprinderea ledului corespunzator
  digitalWrite(pin, HIGH);
}

// Afiseaza nota + frecventa tinta pe display
void displayFrequency(double frequency) {
  // Sterge afisajul de pe LCD
  lcd.clear();
  
  // Seteaza cursorul la pozitia (0, 0)
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");
  // Frecventa cu 2 zecimale
  lcd.print(frequency, 2);
  lcd.print(" Hz");
  lcd.setCursor(0, 1);
  lcd.print("Note: ");
  
  // Nota muzicala corespunzatoare frecventei
  switch (static_cast<int>(frequency)) {
    case 82:
      lcd.print("E2");
      break;
    case 110:
      lcd.print("A2");
      break;
    case 147:
      lcd.print("D3");
      break;
    case 196:
      lcd.print("G3");
      break;
    case 247:
      lcd.print("B3");
      break;
    case 330:
      lcd.print("E4");
      break;
    default:
      lcd.print("Unknown");
      break;
  }
}

// ISR pentru butonul de ON/OFF
void onOffButtonISR() {
  static unsigned long lastInterruptTimeOnOff = 0;
  unsigned long interruptTime = millis();

  // Daca a trecut suficient timp de la ultima intrerupere
  if (interruptTime - lastInterruptTimeOnOff > 400) {
    // Inverseaza starea tunerului
    isOn = !isOn;
  }

  // Actualizeaza timpul ultimei intreruperi
  lastInterruptTimeOnOff = interruptTime;
}

// ISR pentru butonul de ciclare
void cycleButtonISR() {
  static unsigned long lastInterruptTimeCycle = 0;
  unsigned long interruptTime = millis();

  // Daca a trecut suficient timp de la ultima intrerupere
  if (interruptTime - lastInterruptTimeCycle > 400) {
    // Ciclu prin frecventele tinta
    targetFrequencyIndex = (targetFrequencyIndex + 1) % 6; 
  }

  // Actualizeaza timpul ultimei intreruperi
  lastInterruptTimeCycle = interruptTime;
}

// Functie pentru debugging, afisare informatii importante in conosla
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    double abscissa;
    switch (scaleType) {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    Serial.print(abscissa, 6);
    if (scaleType == SCL_FREQUENCY) Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
