///////////////////////////////////////////////////////////////////Definicje//////////////////////////////////////////////////////////////////////////////////////

#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define SCL_INDEX 0x00                      //
#define SCL_TIME 0x01                       //
#define SCL_FREQUENCY 0x02                  //
#define SCL_PLOT 0x03                       //
#define pixelx 100                          //  szerokość wyświetlacza w pixelach
#define pixely 32                           //  wysokość wyświetlacza w pixelach
#define xrange 20                           //  szerokość wyświetlacza
#define yrange 4                            //  wysokość wyświetlacza
 
const byte adcPin = 0;                      // A0
const uint16_t samples = 512;               // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 2000;     // Will affect timer max value in timer_setup() SYSCLOCK/8/samplingFrequency should be a integer
const int buttonPin = 48;                   // the number of the pushbutton pin
const int ledPin = 13;                      // the number of the LED pin

volatile int resultNumber;                  //
double vReal[samples];                      //
double vImag[samples];                      //
double m0, m1, m2, m3;                      //
double f0, f1, f2, f3;                      //
int ledflag = 0;                            // variable for reading the pushbutton status


////////////////////////////////////////////////////////////Uruchomienie Bibliotek////////////////////////////////////////////////////////////////////////////////

LiquidCrystal_I2C lcd(0x27, 20, 4);         // definiowanie ekranu
arduinoFFT FFT = arduinoFFT();              // uruchominie FFT

//////////////////////////////////////////////////////////// ADC complete ISR/////////////////////////////////////////////////////////////////////////////////////

ISR (ADC_vect){
    vReal[resultNumber++] = ADC;
            
    if(resultNumber == samples)
    {
      ADCSRA = 0;  // turn off ADC
    }
} 

EMPTY_INTERRUPT (TIMER1_COMPB_vect);

////////////////////////////////////////////////////////////SETUP TIMERA//////////////////////////////////////////////////////////////////////////////////////////

void timer_setup(){                                 // reset Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B = bit (CS11) | bit (WGM12);                // CTC, prescaler of 8
  TIMSK1 = bit (OCIE1B);
  OCR1A = ((16000000 / 8) / samplingFrequency) -1;  // sampling frequency = 16/8/250 MHz =  8  KHz 
}

////////////////////////////////////////////////////////////SETUP ADC/////////////////////////////////////////////////////////////////////////////////////////////

void adc_setup() {
  ADCSRA =  bit (ADEN) | bit (ADIE) | bit (ADIF);   // włączenie ADC, przerwanie po zakońćzeniu
  ADCSRA |= bit (ADPS2);                            // Prescaler = 16
  ADMUX = bit (REFS0) | (adcPin & 7);               //
  ADCSRB = bit (ADTS0) | bit (ADTS2);               // Timer/Counter1 Compare Match B trigger source
  ADCSRA |= bit (ADATE);                            // włączanie automatycznego wyzwalania
}

//////////////////////////////////////////////////////Ustawienie części urojonej na Zero//////////////////////////////////////////////////////////////////////////

void zeroI() {
   for (uint16_t i = 0; i < samples; i++)
  {
    vImag[i] = 0.0;                                 //Część urojona musi być wyzerowana w przypadku pętli, aby uniknąć błędnych obliczeń i przepełnienia.
  } 
}

//////////////////////////////////////////////////////Znalezienie 4 Najwyższych Częstotliowości///////////////////////////////////////////////////////////////////

void find4(double *vData, uint16_t bufferSize, uint8_t scaleType){
  for (uint16_t i = 0; i < bufferSize; i++){
    double abscissa;
    /* Print abscissa value */
    switch (scaleType){
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
   
 if(vData[i]>m0){
    f0 = abscissa;
    m0 = vData[i];
    }
      else if(vData[i]>m1){
        f1 = abscissa;
        m1 = vData[i];
        }
          else if(vData[i]>m2){
            f2 = abscissa;
            m2 = vData[i];
          }
            else if(vData[i]>m3){
              f3 = abscissa;
              m3 = vData[i];
             }
  }
}

///////////////////////////////////////////////////////////////Stworzenie Tablicy do rysowania////////////////////////////////////////////////////////////////////

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType){
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
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
    //Serial.print(i);
    //Serial.print(" ");
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
    Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

/////////////////////////////////////////////////////////////////Stworzenie Wyresu na LCD/////////////////////////////////////////////////////////////////////////

void graf(double *vData, uint16_t bufferSize, uint8_t scaleType){
   //double skala[20]={32,16,8,8,4,4,4,4,4,8,8,8,16,16,16,16,16,16,16,32}; //DOmyśna dla duzej FSP
  double skala[20]={1,2,2,2,2,2,2,4,4,4,4,4,4,4,4,8,16,32,54,100}; // Wedlgu oktaw
  double amp = 0;
  double probka = 0;
  uint8_t wysokosc = 4;
  uint8_t wiersz = 3;
  uint8_t kolumna = 0;
  double abscissa;
  for (uint16_t i = 0; i < bufferSize; i++){
    double abscissa;
    /* Print abscissa value */
    switch (scaleType){
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
    if(probka < skala[kolumna]){
      //Serial.print("nr probki ");
      //Serial.print(probka+1);
      //Serial.println("  ");
     // Serial.print(abscissa);
     // Serial.print("Hz ");
     // Serial.print(vData[i]);
     // Serial.print(" amp  ");
      amp = amp + vData[i];
      probka++;
     // Serial.println(amp);
      
    }else{
      //Serial.print("  amp/skala ");
      amp = amp/skala[kolumna];
      //Serial.println(amp);
      uint8_t zmp =  map(amp, 0, 20000, 0, 31);
      //Serial.print(" ZMP ");
      //Serial.println(zmp);
      lcd.draw_vertical_graph(wiersz, kolumna, wysokosc, zmp);
      probka = 0;
      kolumna++;
      amp = 0;
      //Serial.print("wykonane ");
      //Serial.println(kolumna);
      //Serial.println("");
      //Serial.println("");
    }
  }  
}

//////////////////////////////////////////////////////////////////konwersja ADC na dB/////////////////////////////////////////////////////////////////////////////

double dB(double Amp){
  double decybel=0;
  decybel = 20.0*(log10((Amp)/50000.0));
  
  return decybel;
}

/////////////////////////////////////////////////////////////////SETUP LCD////////////////////////////////////////////////////////////////////////////////////////

void lcd_setup(){
  Wire.begin();
  lcd.begin(xrange, yrange);
  lcd.init_bargraph(LCDI2C_VERTICAL_BAR_GRAPH);
  lcd.backlight();
  lcd.clear();
  lcd.clear();
  lcd.print("Henlo");
  lcd.setCursor(2,1);
  lcd.print("Spektrometr Fali");
  lcd.setCursor(5,2);
  lcd.print("Akustycznej");
  lcd.setCursor(1,3);
  lcd.print("by Maciej Silkowski");
  delay(200);
  }
  
//////////////////////////////////////////////////////////////Czyszczenie zmiennej////////////////////////////////////////////////////////////////////////////////

void memclear(){
  m0=0;
  m1=0;
  m2=0;
  m3=0;
  }
  
//////////////////////////////////////////////////////////////////////Setup Programu//////////////////////////////////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  zeroI();
  timer_setup();
  adc_setup();
  lcd_setup();
}

/////////////////////////////////////////////////////////////////////Początek Głównej Funkcji/////////////////////////////////////////////////////////////////////

void loop(){
  // czeka na zapełnienie tablicy
  while (resultNumber < samples){ }

/////////////////////////////////////////////////////////////////////Wypisanie Danych Pomiarowych/////////////////////////////////////////////////////////////////
  
  //Serial.println("Data:");
  //PrintVector(vReal, samples, SCL_TIME);

/////////////////////////////////////////////////////////////////////Transformata Fouriera////////////////////////////////////////////////////////////////////////

  FFT.DCRemoval(vReal, samples); 
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);

//////////////////////////////////////////////////////////////////////Obróbka Danych//////////////////////////////////////////////////////////////////////////////
  
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency); /Znalezienie najwyższej częstotliwości przy wykorzystaniu bilioteki ArduinoFFT
  find4(vReal, (samples >> 1), SCL_FREQUENCY);
  resultNumber = 0;
  
  
//////////////////////////////////////////////////////////////////////Guzik///////////////////////////////////////////////////////////////////////////////////////
  
  if (digitalRead(buttonPin)==HIGH){  // jeśli guzik jest naciśnięty
    if (ledflag==0){                 // a flaga jest == 0
      ledflag=1;                      // zmien ją na 1
      digitalWrite(ledPin,HIGH);      // i włącz LEDe
    }                               // 
    else if(ledflag==1){                            // jeśli nie 
      ledflag=2;                      // zmień na flage na 0
      digitalWrite(ledPin,LOW);       // i wyłącz LEDe
    }
    else{
    ledflag=0;
    digitalWrite(ledPin,HIGH);
    }      
  }
  
/////////////////////////////////////////////////////////////Wyświetlanie/////////////////////////////////////////////////////////////////////////////////////////
  
  if(ledflag == 1){
      lcd.clear();
      lcd.setCursor(0,0); 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("f0=");
      lcd.print(f0);
      lcd.print("Hz");
      //lcd.setCursor(11,0);
      //lcd.print("f0=");
      //lcd.print(x);
      //lcd.print("Hz");
      lcd.setCursor(0,1);
      lcd.print("f1=");
      lcd.print(f1);
      lcd.print("Hz");
      lcd.setCursor(0,2);
      lcd.print("f2=");
      lcd.print(f2);
      lcd.print("Hz");
      lcd.setCursor(0,3);
      lcd.print("f3=");
      lcd.print(f3);
      lcd.print("Hz");
    }
    else if(ledflag == 0){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("f0=");
      lcd.print(f0);
      lcd.print("Hz");
      lcd.setCursor(5,1);
      lcd.print("A0=");
      lcd.print(dB(m0));
      lcd.print(" dB");
      lcd.setCursor(0,2);
      lcd.print("f1=");
      lcd.print(f1);
      lcd.print("Hz");
      lcd.setCursor(5,3);
      lcd.print("A1=");
      lcd.print(dB(m1));
      lcd.print(" dB");
    }
    else if(ledflag == 2){
      lcd.clear();
      graf(vReal, (samples >> 1), SCL_FREQUENCY);
    }

/////////////////////////////////////////////////////////////////Funkcje Końcowe//////////////////////////////////////////////////////////////////////////////////

  memclear();
  delay(100);                      
  zeroI();
  adc_setup();
}

//////////////////////////////////////////////////////////////////////Koniec//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
