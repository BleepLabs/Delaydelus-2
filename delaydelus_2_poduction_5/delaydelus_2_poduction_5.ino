// Delaydelus 2 v5
// by john-mike reed
// bleeplabs.com

#include <Bounce.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <EEPROM.h>

IntervalTimer timer1;


AudioSampler             sample0, sample1, sample2, sample3;
AudioEffectTapeDelay     delay1, delay2;
AudioFilterStateVariable filterdly1, filterdly2, hpf1, hpf2, envfl, envfr;
AudioAnalyzeFollower     follower0;
AudioAnalyzeFollower     follower1;
AudioSynthWaveformSine sinetestl, sinetest2;
AudioInputI2S            i2s_input;          
AudioRecordQueue         queue_left, queue_right;      
AudioMixer4              mixer1, mixer2, mixer3, mixer4, mixer5, mixer6, mixerR, mixerL, mixer_in_left, mixer_in_right, mixer_in_peak, mixerf1, mixerf2, mixerf3, mixerf0, mixertest1, mixertest0;
AudioOutputI2S           i2s_output;        
AudioOutputAnalogStereo  DAC;
AudioAnalyzePeak         peakin, peakright, peakleft;

AudioEffectWaveshaper waveshaper_in_left, waveshaper_in_right, waveshaper_dly1, waveshaper_dly2;

AudioConnection          pc1(sample0, 0, mixer1, 0);
AudioConnection          pc2(sample0, 1, mixer2, 0);

AudioConnection          pc3(sample1, 0, mixer1, 1);
AudioConnection          pc4(sample1, 1, mixer2, 1);

AudioConnection          pc5(sample2, 0, mixer1, 2);
AudioConnection          pc6(sample2, 1, mixer2, 2);

AudioConnection          pc7(sample3, 0, mixer1, 3);
AudioConnection          pc8(sample3, 1, mixer2, 3);

AudioConnection          pc14(mixer1, 0, mixer3, 0);
AudioConnection          pc15(mixer2, 0, mixer4, 0);

AudioConnection          pc141(mixer3, 0, delay1, 0);
AudioConnection          pc1412(delay1, 0, waveshaper_dly1, 0);

AudioConnection          pc152(waveshaper_dly1, 0, filterdly1, 0);
AudioConnection          pc142(filterdly1, 0, hpf1, 0);
AudioConnection          pc1421(hpf1, 2, mixer3, 3);

AudioConnection          pc151(mixer4, 0, delay2, 0);
AudioConnection          pc1512(delay2, 0, waveshaper_dly2, 0);

AudioConnection          pc1513(waveshaper_dly2, 0, filterdly2, 0);
AudioConnection          pc15131(filterdly2, 0, hpf2, 0);
AudioConnection          pc1514(hpf2, 2, mixer4, 3);

AudioConnection          pc9(mixer_in_left, 0, mixer3, 1);

AudioConnection          pc111(mixer_in_left, 0, mixer_in_peak, 0);
AudioConnection          pc112(mixer_in_right, 0, mixer_in_peak, 1);

AudioConnection          pc113(mixer_in_peak, 0, peakin, 0);

AudioConnection          pc10(mixer_in_right, 0, mixer4, 1);

AudioConnection          pca1s(sample0, 0, mixerf0, 0);
AudioConnection          pca2s(sample0, 1, mixerf0, 1);

AudioConnection          pca1(mixerf0, 0, mixer5, 0);
AudioConnection          pca2(mixerf0, 0, mixer6, 0);

AudioConnection          pca3s(sample1, 0, mixerf1, 0);
AudioConnection          pca4s(sample1, 1, mixerf1, 1);

AudioConnection          pca3(mixerf1, 0, mixer5, 1);
AudioConnection          pca4(mixerf1, 0, mixer6, 1);

AudioConnection          pca5s(sample2, 0, mixerf2, 0);
AudioConnection          pca6s(sample2, 1, mixerf2, 1);

AudioConnection          pca5(mixerf2, 0, mixer5, 2);
AudioConnection          pca6(mixerf2, 0, mixer6, 2);

AudioConnection          pca7s(sample3, 0, mixerf3, 0);
AudioConnection          pca8s(sample3, 1, mixerf3, 1);

AudioConnection          pca7(mixerf3, 0, mixer5, 3);
AudioConnection          pca8(mixerf3, 0, mixer6, 3);

AudioConnection          pca11(mixer5, 0, envfl, 0);
AudioConnection          pca111(envfl, 0, follower0, 0);

AudioConnection          pca12(mixer6, 0, envfr, 0);
AudioConnection          pca121(envfr, 0, follower1, 0);

AudioConnection          pca11k(mixer5, 0, peakleft, 0);
AudioConnection          pca12k(mixer6, 0, peakright, 0);

AudioConnection          pcf111(follower1, 0, mixertest1, 0);
AudioConnection          pcf222(follower0, 0, mixertest0, 0);

AudioConnection          pcf1111(sinetestl, 0, mixertest1, 1);
AudioConnection          pcf2222(sinetestl, 0, mixertest0, 1);

AudioConnection          pcf11111(mixertest1, 0, DAC, 1);
AudioConnection          pcf22222(mixertest0, 0, DAC, 0);

AudioConnection          pcin1(i2s_input, 1, mixer_in_left, 0);
AudioConnection          pcin2(waveshaper_in_left, 0, mixer_in_left, 0);

AudioConnection          pcin3(i2s_input, 0, mixer_in_right, 0);
AudioConnection           pcin4(waveshaper_in_right, 0, mixer_in_right, 0);

AudioConnection          pco111(mixer_in_left, 0, queue_left, 0);
AudioConnection          pco112(mixer_in_right, 0, queue_right, 0);


AudioConnection          pco12(mixer3, 0, mixerL, 0);
AudioConnection          pco13(mixer4, 0, mixerR, 0);


AudioConnection          outt2(sinetestl, 0, mixerL, 1);
AudioConnection          outt1(sinetest2, 0, mixerR, 1);

AudioConnection          pco121(mixerL, 0, i2s_output, 0);
AudioConnection          pco131(mixerR, 0, i2s_output, 1);

AudioControlSGTL5000     sgtl5000_1;     //xy=291,525

#define LPOT 32
#define RPOT 35
#define TIMEPOT 16
#define FBPOT 34
#define VOLPOT 33

#define LBUT 8
#define RBUT 10

#define RECBUT 36
#define SHIFTBUT 17
#define CVi1 37
#define CVo1 A21
#define Ti1 25
#define To1 5

#define CVi2 38
#define CVo2 A22
#define Ti2 24
#define To2 21

#define DLY_SYNC_OUT_PIN 20
#define DLY_SYNC_IN_PIN 26

#define left_contact 29
#define right_contact 30

#define DATA_WIDTH 16
#define ploadPin         2
#define srdataPin         4
#define srclockPin         3

#define s2m (1.00/AUDIO_SAMPLE_RATE_EXACT)*1000000.00 ; //2.26666 samples to microseconds =(1/actual sample rate)*1000000;
byte contact_reading[17] = {};

byte left_en_contacts[16] = {};
byte right_en_contacts[16];

byte shiftin_pin_oder[16] = {15, 11, 0, 3, 7,   13, 9, 8, 1, 5,     7, 7, 7, 7, 7, 7};

#define BOUNCE_LOCK_OUT


Bounce buttonRecord = Bounce(RECBUT, 8);
Bounce buttonMode =   Bounce(SHIFTBUT, 8);  // 8 = 8 ms debounce time
Bounce button1Play =   Bounce(LBUT, 8);
Bounce button2Play =   Bounce(RBUT, 8);

#include <Adafruit_NeoPixel.h>
#define LED_PIN  27
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(2, LED_PIN, NEO_GRBW + NEO_KHZ800);


const int FlashChipSelect = 6; // digital pin for flash chip CS pin
byte in8[300];

const int SDchipSelect = 31;    // Audio Shield has SD card CS on pin 10
byte lengthBuff[256] = {};
char NewFile_name[11] = {};


#define FLASH_MOSI_PIN  7
#define FLASH_SCK_PIN   14

int getsize = AUDIO_BLOCK_SAMPLES * 2;

int mode = 0;  // 0=stopped, 1=recording, 2=playing
float norm = 86.1679 * 2; 
File frec;// The file where data is recorded
uint32_t address;
byte erasedb = 0;
uint32_t prev[8];
uint32_t cm ;
#define sfblocks 12 //x4 //26 is about 35 seconds
uint32_t rec_size = (sfblocks * 0x10000); //size in bytes
int16_t rec_target;
uint32_t bankstart[12];
uint32_t samplelen[12];
byte left_voices, right_voices, voices_total;
byte left_playing[4], right_playing[4], prev_poly_sample_sel[5];
byte poly_but_ass[5] = {1, 1, 1, 1};
byte poly_sample_sel[5] = {1, 2, 3, 4};
int sample_sel;
uint32_t foffset = 0;
uint32_t ctrc, ptrc[2];
float left_freq, right_freq;
int16_t fen[8], pfen[8];
int rec_mode;
byte blink[8], rec_timer_latch[4];
float peaklerp[4], incomingpeak[4];
uint32_t rec_timer[4], rec_cm, prev_blink[8];
int16_t cvin2, cvin1;
uint32_t delay_pot_log[1030];
#define DLY_MAX 50000
#define DLY_MAX_OFFSET 11
#define delay_rate_redux 0
short dlyb1[DLY_MAX] = {};
short dlyb2[DLY_MAX] = {};
uint32_t dly_len2, dly_len1, actualdly;
float dly_len2_mod = .999;
uint16_t time_read, prev_time_read;
uint32_t loop_timer[3], loop_cm[3];
byte loop_en[3], loop_latch[3];
byte dly2_set;
uint16_t tpr, prev_trp;
const float fbmax = .75;
byte rec_target_latch = 0;
byte voice_max = 4;
byte left_poly_sel = 0;
byte right_poly_sel = 0;
byte tick;
byte delayt = 5;
byte tout;
byte notify[4], notify_latch[4];
uint32_t notify_cm[4], notify_timer[4];
int32_t inv_delay_length;
float fr1;
float microsdly;
byte rev_en[3];
float prevlf, prevrf;
uint32_t ltgcm[3];
byte ltg[3];
byte plbt[4], pltr[4];
float vol_lerp, vol_read;
int log_pot[1030];
const float vol_max = 4.5;
const float follow_lvl = .8; // reduce to lower envlope follower output level. Increaing it will clip the audio level coming into the follower. followers level is hardcoded
int printme;
uint16_t rawsr;
float ingain;
byte   p_sync_in, sync_in, sync_in_count, sync_in_en;
uint32_t sync_in_buff[5], prev_sync, sync_in_rate;
int32_t sync_dly_rate;
float dlymult, delay_fb;
uint32_t trigt;
int combined_peaks;
byte trig_in[3], prev_trig_in[3];
byte trigtlatch;
const byte trig_out_len_ms = 50;
uint32_t loopt, loopd;
int16_t fh[16];
int16_t prevfh[16];
uint16_t fbt, left_pot_temp, right_pot_temp;
byte wait_to_read[3];
uint32_t wait_to_read_timer[3];
const float lerp_step_size = .03;
const byte wait_to_read_len = 1;   //0 would be no wait it takes 2-3 ms to loop so 1 really jsut mean wait a loop
uint16_t redout, grnout, bluout, whout;
int16_t cv_offset = 0; //for trimmer based one which it needs to be set to 1440
float dly_seperation = 1;
byte button_gate_out[3] = {0, 0, 0};
byte loop_trig_out[3] = {0, 0, 0};
uint32_t mode_timer, mode_cm;
byte mode_timer_latch;
byte sd_green;
byte test_mode = 0;
uint16_t foundhigh[8];

float WAVESHAPE_A[17] = {  //not used
  -0.9408,
  -0.9264,
  -0.8784,
  -0.7808,
  -0.6336,
  -0.512,
  -0.3648,
  -0.1952,
  0,
  0.1952,
  0.36488,
  0.512,
  0.6336,
  0.7808,
  0.8784,
  0.9264,
  0.9408
};

void setup() {
  AudioNoInterrupts();
  delay(10);
  pixels.begin();
  delay(10);
  pixels.setBrightness(255);
  delay(10);

  pixels.setPixelColor(0, 0, 0, 0, 3);
  pixels.show();

  AudioMemory(32);

  pinMode(ploadPin, OUTPUT);
  pinMode(srclockPin, OUTPUT);
  pinMode(srdataPin, INPUT);
  pinMode (0, INPUT_PULLUP);

  pinMode(RECBUT, INPUT_PULLUP);
  pinMode(LBUT, INPUT_PULLUP);
  pinMode(RBUT, INPUT_PULLUP);

  pinMode(SHIFTBUT, INPUT_PULLUP);
  pinMode(Ti1, INPUT_PULLUP);
  pinMode(Ti2, INPUT_PULLUP);

  pinMode(left_contact, OUTPUT);
  pinMode(right_contact, OUTPUT);

  pinMode(DLY_SYNC_OUT_PIN, OUTPUT);
  pinMode(DLY_SYNC_IN_PIN, INPUT_PULLUP);

  pinMode(To1, OUTPUT);
  pinMode(To2, OUTPUT);
  digitalWrite(To1, 1);
  digitalWrite(To2, 1);

  float mfg = .5;
  mixerf0.gain(0, mfg);
  mixerf0.gain(1, mfg);
  mixerf1.gain(0, mfg);
  mixerf1.gain(1, mfg);
  mixerf2.gain(0, mfg);
  mixerf2.gain(1, mfg);
  mixerf3.gain(0, mfg);
  mixerf3.gain(1, mfg);

  float gain1 = .3;
  mixer1.gain(0, gain1);
  mixer2.gain(0, gain1);
  mixer1.gain(1, gain1);
  mixer2.gain(1, gain1);
  mixer1.gain(2, gain1);
  mixer2.gain(2, gain1);
  mixer1.gain(3, gain1);
  mixer2.gain(3, gain1);

  mixer3.gain(0, .5); //sample into delay
  mixer4.gain(0, .5);

  mixer3.gain(1, .125); //from input to delay. input is recorded without attenuation.
  mixer4.gain(1, .125);

  // it seems liek a lot of attenuation at multiple steps and it is but its all there to avoid cliiping and to give plenty of headroom for feedback

  float envgain1 = 0;

  mixer5.gain(0, envgain1);
  mixer6.gain(0, envgain1);
  mixer5.gain(1, envgain1);
  mixer6.gain(1, envgain1);
  mixer5.gain(2, envgain1);
  mixer6.gain(2, envgain1);
  mixer5.gain(3, envgain1);
  mixer6.gain(3, envgain1);
  mixer_in_peak.gain(0, .5);
  mixer_in_peak.gain(1, .5);

  //follower1.exp_set(4092);
  //follower0.exp_set(4092);
  //multipled by level and >>12. 4095 would give a constant level out while 4000 drops very quickly.
  //this nuber is the defualt in the livbrary and doesn't need adjsuting.
  // it's follower1.rate(); that cahnges how long it takes


  follower1.amplitude(2.2); //makeup gain. 1.00 is no gain. 2 fives a good balace between amplifying quite signals but not clipping loud/layered ones.
  follower0.amplitude(2.2);

  filterdly1.frequency(14000);
  filterdly1.resonance(.55);

  filterdly2.frequency(14000);
  filterdly2.resonance(.55);


  envfl.frequency(8000);
  envfl.resonance(.5);

  envfr.frequency(8000);
  envfr.resonance(.5);

  hpf1.frequency(170);
  hpf1.resonance(.55);

  hpf2.frequency(170);
  hpf2.resonance(.55);

  waveshaper_in_left.shape(WAVESHAPE_A, 17);
  waveshaper_in_right.shape(WAVESHAPE_A, 17);

  waveshaper_dly1.shape(WAVESHAPE_A, 17);
  waveshaper_dly2.shape(WAVESHAPE_A, 17);

  mixer_in_left.gain(0, 1);
  mixer_in_right.gain(0, 1);

  delay1.begin(dlyb1, DLY_MAX, 0, delay_rate_redux, 3);
  delay2.begin(dlyb2, DLY_MAX, 0, delay_rate_redux, 3);

  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000_1.lineInLevel(13);
  sgtl5000_1.volume(0.78);
  sgtl5000_1.lineOutLevel(14);


  SPI.setMOSI(FLASH_MOSI_PIN);
  SPI.setSCK(FLASH_SCK_PIN);
  SPI.setCS(6);

  if (!SerialFlash.begin(FlashChipSelect)) {
    while (1) {
      Serial.println("Unable to access SPI Flash chip");
      delay(1000);
      pixels.setPixelColor(0, 20, 0, 0);
      pixels.show();

      delay(1000);
      pixels.setPixelColor(0, 0, 0, 0);
      pixels.show();
    }
  }
  byte sdpresent = 1;
  if (!SD.begin(SDchipSelect)) {
    sdpresent = 0;
    Serial.println("No SD card");
    for (byte wr; wr < 3; wr++) {
      pixels.setPixelColor(0, 43, 17, 0 , 0);
      pixels.show();
      delay(100);
      pixels.setPixelColor(0, 8, 4, 0, 0);
      pixels.show();
      delay(120);
    }
  }

  for (int i = 0; i < 1024; ++i)
  {
    float j = i * (DLY_MAX / 1024.00);
    int32_t logt = (pow(j - (DLY_MAX - DLY_MAX_OFFSET), 3) / (pow((DLY_MAX - DLY_MAX_OFFSET), 2))) + DLY_MAX;
    delay_pot_log[i] = logt;
  }

  for (int i = 0; i < 1024; ++i)
  {
    uint32_t logt = pow(i, 2) / 1024;
    log_pot[i] = logt;
  }



  analogReadResolution(12);
  analogReadAveraging(32);

  DAC.analogReference(EXTERNAL);

  byte shiftb = digitalRead(SHIFTBUT);
  byte recb = digitalRead(RECBUT);
  Serial.println("hey");

  if (recb == 0 && shiftb == 0)
  {
    // delay(100);
    if (recb == 0) {
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      //byte eall = digitalRead(SHIFTBUT);
      byte eall = 1;

      if (eall == 0) {
        SerialFlash.eraseAll();
        unsigned long dotMillis = millis();
        unsigned char dotcount = 0;
        while (SerialFlash.ready() == false) {
          if (millis() - dotMillis > 500) {
            dotMillis = dotMillis + 500;
            Serial.print(".");
            dotcount = dotcount + 1;
            if (dotcount >= 60) {
              Serial.println();
              dotcount = 0;
            }
          }
        }
      }

    }
    pixels.setPixelColor(0, 0, 50, 90);
    pixels.show();
    delay(50);
    //copyfromSD();
    for (byte i = 0; i < 10; i++) {
      copyOneFromSD(i);
      pixels.setPixelColor(0, 50, 0, 0);
      pixels.show();
    }
    delay(100);
  }

  byte rt1 = EEPROM.read(100);
  byte rt2 = EEPROM.read(101);
  cv_offset = ((rt1 << 8) | rt2) * -1;

  if (shiftb == 0)
  {
    int cvot = analogRead(CVi1);
    byte t1 = cvot >> 8;
    byte t2 = cvot;
    EEPROM.write(100, t1);
    EEPROM.write(101, t2);
    cv_offset = cvot * -1;
  }

  if (cv_offset < 10 && cv_offset>-10) {
    pixels.setPixelColor(0, 0, 80, 0, 5);
    pixels.show();
    delay(500);
    pixels.setPixelColor(0, 0, 0, 0, 0);
    pixels.show();
    delay(500);
    pixels.setPixelColor(0, 0, 80, 0, 5);
    pixels.show();
    delay(500);
    pixels.setPixelColor(0, 0, 0, 0, 0);
    pixels.show();
    delay(500);
  }

  Serial.print("cv_offset ");
  Serial.println(cv_offset);
  Serial.println ("banks ");
  for (byte i = 0; i < 10; i++) {
    bankstart[i] = (i * sfblocks * 0x10000 * 4) + foffset;
    samplelen[i] = readlen(i);
    Serial.print(bankstart[i]);
    Serial.print(" ");
    Serial.println(samplelen[i]);

  }

  sample0.begin(1, norm, bankstart[0], samplelen[0]);
  sample1.begin(1, norm, bankstart[1], samplelen[1]);
  sample2.begin(1, norm, bankstart[2], samplelen[2]);
  sample3.begin(1, norm, bankstart[3], samplelen[3]);

  Serial.println("ok ");

  uint16_t tpr = (analogRead(TIMEPOT) - 4095) * -1;

  delay1.length_no_lerp(delay_pot_log[tpr >> 2]);
  delay2.length_no_lerp(delay_pot_log[tpr >> 2]);
  mixertest0.gain(0, 1);
  mixertest0.gain(1, 0);
  mixertest1.gain(0, 1);
  mixertest1.gain(1, 0);
  sinetestl.amplitude(0);
  sinetestl.frequency(0);
  sinetest2.amplitude(0);
  sinetest2.frequency(0);
  AudioInterrupts();

  if (digitalRead(LBUT) == 0 && digitalRead(SHIFTBUT) == 0 && digitalRead(RBUT) == 1 && digitalRead(RECBUT) == 1) {
    test_mode = 1;
    Serial.println(" ~ test_mode ~ ");
  }
  byte r0 = digitalRead(0);
  if (r0 == 0) {
    test_mode = 1;
    Serial.println(" @ test_mode @ ");
  }

  byte bb, rr, gg, tot = 1, tot2 = 1,  tot3 = 1, stepa;
  int aout1, aout2;

  if (test_mode == 1 && sdpresent == 1) {
    File roott = SD.open("/");
    printDirectory(roott, 0);
    delay(1000);
  }

  while (test_mode > 0 ) {

    buttonRecord.update();
    buttonMode.update();
    button1Play.update();
    button2Play.update();
    cm = millis();


    if (button1Play.read() == 0 && button2Play.fallingEdge()) {
      test_mode++;
      if (test_mode > 3) {
        test_mode = 1;
      }
      Serial.print("***test_mode = ");
      Serial.println(test_mode);

    }
    if (test_mode == 2) {
      if (cm - prev[0] > 20) {
        prev[0] = cm;
        Serial.print(analogRead(CVi1));
        Serial.print(" ");
        Serial.println(analogRead(CVi2));
      }
    }
    if (test_mode == 3) {

      int cvot = analogRead(CVi1);
      byte t1 = cvot >> 8;
      byte t2 = cvot;
      EEPROM.write(100, t1);
      EEPROM.write(101, t2);
      cv_offset = cvot * -1;
      Serial.print(" cv_offset");   //SET TO 1440
      Serial.println(cv_offset);
      delay(500);
      test_mode = 1;
    }

    if (test_mode == 1) {
      sinetestl.amplitude(1);
      sinetestl.frequency(220);
      sinetest2.amplitude(1);
      sinetest2.frequency(440);

      if (cm - prev[1] > 5) {
        prev[1] = cm;
        aout1 += 3;
        if (aout1 > 40) {
          aout1 = 0;
        }

        aout2++;
        if (aout2 > 20) {
          aout2 = 0;
        }
        //  analogWrite(CVo2, aout2);
        // analogWrite(CVo1, aout1);

      }
      byte bbb = 10;

      if (cm - prev[2] > (analogRead(TIMEPOT) / 4) + 20) {
        prev[2] = cm;
        stepa++;
        if (stepa == 1) {
          pixels.setPixelColor(0, bbb, 0, 0, 0);
        }
        if (stepa == 2) {
          pixels.setPixelColor(0, 0, bbb, 0, 0 );
        }
        if (stepa == 3) {
          pixels.setPixelColor(0, 0, 0, bbb, 0  );
        }
        if (stepa >= 4) {
          pixels.setPixelColor(0, 0, 0, 0, bbb  );
          stepa = 0;
        }
        pixels.show();
        tot2 = !tot2;
        digitalWrite(To2, tot2);

      }

      if (cm - prev[3] > 500) {
        prev[3] = cm;
        tot = !tot;

        digitalWrite(To1, tot);

      }
      if (cm - prev[0] > 80) {
        prev[0] = cm;
        tot3 = !tot3;
        if (sdpresent == 0) {
          Serial.println("~~~~ NO SD ~~~~");
          Serial.println();

        }
        digitalWrite(DLY_SYNC_OUT_PIN, tot3);

        if (digitalRead(RECBUT) == 0 && digitalRead(SHIFTBUT) == 0) {
          for (byte i = 0; i < 10; i++) {
            copyOneFromSD(i);
            pixels.setPixelColor(0, 50, 0, 0);
            pixels.show();
          }
          delay(100);
        }

        mixertest0.gain(0, 0);
        mixertest0.gain(1, analogRead(VOLPOT) / 3000.00);
        mixertest1.gain(0, 0);
        mixertest1.gain(1, analogRead(VOLPOT) / 3000.00);

        mixerR.gain(1, analogRead(VOLPOT) / 4096.00);
        mixerL.gain(1, analogRead(VOLPOT) / 4096.00);
        mixer3.gain(3, 0);
        mixer4.gain(3, 0);
        mixer3.gain(1, 1);
        mixer4.gain(1, 1);
        mixer_in_left.gain(0, 4);
        mixer_in_right.gain(0, 4);
        gg++;

        //   printMon();
        cv_read();
        read_contacts();
        print_contacts();

        Serial.print("r ");
        Serial.print(buttonRecord.read());
        Serial.print(" m ");
        Serial.println(buttonMode.read());

        Serial.print("L ");
        Serial.print(button1Play.read());
        Serial.print(" R ");
        Serial.println(button2Play.read());
        Serial.println();

        int rcv1 = analogRead(CVi1);
        int rcv2 = analogRead(CVi2);

        int ga = (rcv1 >> 7);
        for (byte i = 0; i < 32; i++) {
          if (i == ga) {
            Serial.print("1");
          }
          else {
            Serial.print("-");
          }
        }
        Serial.println();
        ga = (rcv2 >> 7);
        for (byte i = 0; i < 32; i++) {
          if (i == ga) {
            Serial.print("2");
          }
          else {
            Serial.print("-");
          }
        }
        Serial.println();


        Serial.print(rcv1);
        Serial.print(" ");
        Serial.println(rcv2);



        Serial.print(digitalRead(Ti1));
        Serial.print(" ");
        Serial.print(digitalRead(DLY_SYNC_IN_PIN));
        Serial.print(" ");
        Serial.print(digitalRead(Ti2));
        Serial.println(" ");

        Serial.print(analogRead(LPOT));
        Serial.print(" ");
        Serial.print(analogRead(TIMEPOT));
        Serial.print(" ");
        Serial.print(analogRead(FBPOT));
        Serial.print(" ");
        Serial.println(analogRead(RPOT));
        Serial.print("        ");
        Serial.println(analogRead(VOLPOT));
        Serial.println();

      }
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////




void loop() {
  uint32_t cmm = micros();

  if (cmm - prev[0] >= microsdly - 1) {
    prev[0] = micros();
    tout = !tout;
    digitalWrite(DLY_SYNC_OUT_PIN, tout);
  }

  byte aves = 2;
  p_sync_in = sync_in;
  sync_in = digitalRead(DLY_SYNC_IN_PIN);
  if (p_sync_in == 0  && sync_in == 1) {
    sync_in_en = 1;
    sync_in_buff[sync_in_count] = cmm - prev_sync;
    prev_sync = cmm;

    sync_in_rate = 0;
    for (int i = 0; i < aves; ++i) {
      sync_in_rate += sync_in_buff[i] / aves;
    }

    sync_in_count++;
    if (sync_in_count > aves - 1) {
      sync_in_count = 0;

    }

    sync_dly_rate = (((sync_in_rate / 1000.00) * 22.058 * dlymult) - DLY_MAX) * -1; ////////////////////dlymult here how to make it half speed///////////??????????????
    sync_dly_rate = constrain (sync_dly_rate, 1, DLY_MAX - DLY_MAX_OFFSET) ;
  }




  cm = millis();
  loopd = cm - loopt;
  loopt = cm;

  foundhigh[0] = find_high(0, loopd);


  if (cm - prev[5] > 200 && 1 == 0) {
    prev[5] = cm;

    Serial.println(left_freq);
    Serial.println(right_freq);
    Serial.println();
  }

  if (cm - prev[2] > 1000 && 1 == 0 ) {
    prev[2] = cm;
    printMon();
    print_contacts();

    //  Serial.println(mode_timer);
    //  Serial.println(mode_timer_latch);
  }

  cv_read(); // if we dont read it ever loop smoothign wont work. sounds smart to only read when triggering but doenst work so well.
  read_contacts();

  for (int i = 0; i < 4; i++)
  {
    if (poly_but_ass[i] == 2) {
      mixer6.gain(i, follow_lvl);
    }

    if (poly_but_ass[i] != 2) {
      mixer6.gain(i, 0);
    }

    if (poly_but_ass[i] == 1) {
      mixer5.gain(i, follow_lvl);
    }

    if (poly_but_ass[i] != 1) {
      mixer5.gain(i, 0);
    }

  }

  // sync in


  buttonRecord.update();
  buttonMode.update();
  button1Play.update();
  button2Play.update();

  byte tout1, tout2 = 0;

  button_gate_out[1] = !button1Play.read();
  button_gate_out[2] = !button2Play.read();

  if (ltg[1] > 0) {
    if (cm - ltgcm[1] > trig_out_len_ms) {
      ltg[1] = 0;
      loop_trig_out[1] = 0;
    }
  }

  if (ltg[2] > 0) {
    if (cm - ltgcm[2] > trig_out_len_ms) {
      ltg[2] = 0;
      loop_trig_out[2] = 0;
    }
  }


  for (int i = 0; i < 4; i++) {
    if (poly_but_ass[i] == 1) {
      if (sample_loop_trig(i) != plbt[i]) {
        ltg[1] = 1;
        ltgcm[1] = cm;
        loop_trig_out[1] = 1;

      }
      plbt[i] = sample_loop_trig(i);
    }

    if (poly_but_ass[i] == 2) {
      if (sample_loop_trig(i) != pltr[i]) {
        ltg[2] = 1;
        ltgcm[2] = cm;
        loop_trig_out[2] = 1;

      }
      pltr[i] = sample_loop_trig(i);
    }
  }
  byte tto1 = loop_trig_out[1] + button_gate_out[1] + !trig_in[1];
  if (tto1 > 1) {
    tto1 = 1;
  }

  byte tto2 = loop_trig_out[2] + button_gate_out[2] + !trig_in[2];
  if (tto2 > 1) {
    tto2 = 1;
  }

  digitalWrite(To1, !tto1);
  digitalWrite(To2, !tto2);


  //////////////////////////////////////////////////////// Read triggers

  prev_trig_in[1] = trig_in[1];
  trig_in[1] = digitalRead(Ti1);

  prev_trig_in[2] = trig_in[2];
  trig_in[2] = digitalRead(Ti2);


  if (prev_trig_in[1] == 1 && trig_in[1] == 0 ) {

    if (rec_mode < 1) {
      wait_to_read[1] = 1;
      wait_to_read_timer[1] = cm;
    }
  }

  if (prev_trig_in[2] == 1 && trig_in[2] == 0) {

    if (rec_mode < 1) {
      wait_to_read[2] = 1;
      wait_to_read_timer[2] = cm;
    }
  }

  //wait for cv to settle reading it at the instant the trigger somes in gives inconsistent behavior
  if  (wait_to_read[1] == 1 && cm - wait_to_read_timer[1] > wait_to_read_len) {
    wait_to_read[1] = 0;
    play_lefts();
  }

  if  (wait_to_read[2] == 1 && cm - wait_to_read_timer[2] > wait_to_read_len) {
    wait_to_read[2] = 0;
    play_rights();
  }

  if (rec_mode == -2) {
    if (prev_trig_in[1] == 0 && trig_in[1] == 1 ) {

      for (int i = 0; i < 4; i++) {
        if (poly_but_ass[i] == 1) {
          sample_stop(i);
        }
      }
    }

    if (prev_trig_in[2] == 0 && trig_in[2] == 1) {

      for (int i = 0; i < 4; i++) {
        if (poly_but_ass[i] == 2) {
          sample_stop(i);
        }
      }

    }
  }



  ///////////////////////////////////////////////////////////////led notifications
  if (cm - prev[3] > 42) {
    prev[3] = cm;

    if (notify_latch[0] > 0) {
      notify_timer[0] = millis() - notify_cm[0];
      notify[0] = 30;
      if (notify_timer[0] > 300) {
        notify_latch[0] = 0;
      }
    }

    if (notify_latch[0] == 0) {
      notify[0] = 0;
    }

    pixels.show();
  }

  //led enveloper

  if (cm - prev[1] > 42 ) {
    prev[1] = cm;
    for (int i = 1; i < 4; ++i) {

      if (peaks_avail(i)) {
        incomingpeak[i] = read_peaks(i);

        if (incomingpeak[i] > peaklerp[i]) {
          peaklerp[i] = incomingpeak[i];
        }
      }
      if (peaklerp[i] > .1) {
        peaklerp[i] *= .5;
      }
      if (peaklerp[i] <= .1) {
        peaklerp[i] = 0;
      }
    }
  }

  //  fbt =   log_pot[(digitalSmooth(2, 19 , (analogRead(FBPOT) - 4095) * -1)) >> 2];
  fbt =   (digitalSmooth(2, 19 , (analogRead(FBPOT) - 4095) * -1)) >> 2;
  byte low_cut = 2;
  if (fbt <= low_cut) {
    delay_fb = 0;
    sync_in_en = 0;
  }
  if (fbt > low_cut) {
    fbt = map(fbt, 0, 1023, 0, 1023 * fbmax);
    delay_fb = (fbt / 1023.00 );
  }

  mixer3.gain(3, delay_fb);
  mixer4.gain(3, delay_fb);

  prev_time_read = time_read;
  uint16_t time_read_temp = digitalSmooth(0, 19 , analogRead(TIMEPOT));
  byte time_update = 0;

  if (prev_time_read > time_read_temp + 4 || prev_time_read < time_read_temp - 4) {
    time_read = time_read_temp;
    tpr = (time_read - 4095) * -1;
  }

  if (sync_in_en == 0) {
    dly_len1 = delay_pot_log[tpr >> 2];
    sync_dly_rate = dly_len1;
  }

  if (sync_in_en == 1) {
    float timemult[14] = {2, 1.666, 1.5, 1.333, 1, 0.666, 0.5, 0.333, 0.25, 0.2, 0.16667, 0.125, 0.0625, 0.03125};
    byte msel = tpr / (4095.00 / 14.00);
    dlymult = timemult[msel];

    dly_len1 = sync_dly_rate;
  }

  actualdly = delay1.length(dly_len1 + 1) ;
  delay2.length(dly_len1 + 1);

  inv_delay_length = ((actualdly - DLY_MAX) * -1) + (AUDIO_BLOCK_SAMPLES / 2); //sync pulse is ~20 slow

  microsdly = inv_delay_length * s2m ; //samples to microseconds =(1/44117.65)*1000000;
  fr1 = ((tpr - 4095) * -1) >> 7;
  follower0.rate(fr1); //expects 0-4095 now but really only like 4095-4000
  follower1.rate(fr1);

  vol_read = (digitalSmooth(1, 19 , flip_pot(VOLPOT))) / (4095.00 / vol_max);
  if (vol_read < vol_lerp - lerp_step_size) {
    vol_lerp -= lerp_step_size;
  }
  if (vol_read > vol_lerp + lerp_step_size) {
    vol_lerp += lerp_step_size;
  }

  mixerR.gain(0, vol_lerp);
  mixerL.gain(0, vol_lerp);
  mixerR.gain(1, 0);
  mixerL.gain(1, 0);


  if (buttonRecord.fallingEdge()) {
    mode_timer = 0;
    mode_cm = cm;
    mode_timer_latch = 0;

    rec_timer[0] = 0;
    rec_cm = cm;
    rec_timer_latch[0] = 0;
  }

  if (buttonRecord.read() == 0 && rec_timer_latch[0] == 0) {
    rec_timer[0] = millis() - rec_cm;
  }

  if (rec_timer[0] > 750) {
    rec_timer_latch[0] = 1;
    rec_timer[0] = 0;
    if (rec_mode < 1) {
      rec_mode = 1;
    }
    else {
      rec_mode++;
    }
  }

  if (buttonRecord.risingEdge()) {
    rec_timer[0] = 0;
  }


  if (buttonMode.fallingEdge() && button1Play.read() != 0 && button2Play.read() != 0 ) {

    for (int i = 0; i < 4; i++) {
      sample_stop(i);
    }

    if (rec_mode <= 0) {
      rec_mode--;
      if (rec_mode < -2) {
        rec_mode = 0;
      }
    }

    mode_timer = 0;
    mode_cm = cm;
    mode_timer_latch = 0;
  }

  if (mode_timer > 1000) {
    mode_timer_latch = 1;
    sd_green = 1;
    mode_timer = 0;
  }


  if (buttonMode.read() == 0 && rec_mode == 1) {
    if (mode_timer_latch == 0) {
      mode_timer = millis() - mode_cm;
    }
  }

  if (buttonMode.fallingEdge()) {
    if (rec_mode == 1) { // warning
      //    rec_mode = 0;
    }
  }

  if (button1Play.fallingEdge()) {
    mode_timer = 0;
    mode_cm = cm;
    mode_timer_latch = 0;

    if (rec_mode < 1) {
      play_lefts();
    }
    if (rec_mode == 1) { // warning
      play_lefts();
      rec_mode = 0;
    }

  }

  if (button1Play.read() == 0 && buttonMode.fallingEdge() ) { ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    loop_en[1] = !loop_en[1];
    for (int i = 0; i < 4; i++)
    {
      if (poly_but_ass[i] == 1) {
        sample_loop_enable(i, loop_en[1]);
      }
    }
    if (loop_en[1] == 1) {
      // play_lefts();

    }
    if (loop_en[1] == 0) {
      for (int i = 0; i < 4; i++) {
        if (poly_but_ass[i] == 1) {
          sample_stop(i);
        }
      }
    }

    notify_latch[0] = 1;
    notify_cm[0] = cm;

  }

  if (button2Play.fallingEdge()) {
    mode_timer = 0;
    mode_cm = cm;
    mode_timer_latch = 0;

    if (rec_mode < 1) {
      play_rights();
    }
    if (rec_mode == 1) { // warning
      rec_mode = 0;
      play_rights();
    }
  }
  if (button2Play.read() == 0 && buttonMode.fallingEdge() ) { ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    loop_en[2] = !loop_en[2];
    for (int i = 0; i < 4; i++)
    {
      if (poly_but_ass[i] == 2) {
        sample_loop_enable(i, loop_en[2]);
      }
    }
    if (loop_en[2] == 0) {
      for (int i = 0; i < 4; i++) {
        if (poly_but_ass[i] == 2) {
          sample_stop(i);
        }
      }
    }

    notify_latch[0] = 1;
    notify_cm[0] = cm;
  }


  if (mode == 1) {
    continueRecording();  /////////??????????????
  }



  //////////////////////////////////////////////////////////////////// REC MODES

  if (rec_mode == -2) { // gate passthrough and samples

    if (button1Play.risingEdge()) {
      for (int i = 0; i < 4; i++) {
        if (poly_but_ass[i] == 1) {
          sample_stop(i);
        }
      }
    }

    if (button2Play.risingEdge()) {
      for (int i = 0; i < 4; i++) {
        if (poly_but_ass[i] == 2) {
          sample_stop(i);
        }
      }
    }

    if (button1Play.read() == 0 || trig_in[1] == 0) {
      mixer_in_left.gain(0, 1);
      mixer_in_right.gain(0, 1);
    }

    if (button1Play.read() == 1 && trig_in[1] == 1) {
      mixer_in_left.gain(0, 0);
      mixer_in_right.gain(0, 0);
    }

    const byte bottom = 10;
    int grn ;
    int blue;
    int red = 8;
    int white = 12;

    float lvl1 = (peaklerp[2] + peaklerp[3] + peaklerp[1]);
    float mult = 50;

    grn = 10 ;
    blue = 22 + (lvl1 * mult * 2);
    red = 0 + (lvl1 * mult * .2);
    white = 1;

    if (lvl1 >= .65) {
      grn = 8 + lvl1 * (mult * 1);
      red = 0;
      white =  0;
    }

    grn = constrain(grn , 0, 255);
    blue = constrain(blue, 0, 255);
    red = constrain(red, 0, 255);
    white = constrain(white + notify[0], 0, 255);

    pixels.setPixelColor(0, red, grn, blue, white);

  }

  if (rec_mode == -1) { // passthrough with mute

    if (button1Play.read() == 1) {
      mixer_in_left.gain(0, 1);
      mixer_in_right.gain(0, 1);

    }
    if (button1Play.read() == 0) {
      mixer_in_left.gain(0, 0);
      mixer_in_right.gain(0, 0);
    }

    const byte bottom = 10;
    int grn ;
    int blue;
    int red = 8;
    int white = 12;

    float lvl1 = (peaklerp[2] + peaklerp[3] + peaklerp[1]);
    float mult = 50;

    grn = 0;
    blue = 8 + (lvl1 * mult);
    red = 8 + (lvl1 * mult * .5);
    white =  1;

    if (lvl1 >= .65) {
      blue = 10 + (lvl1 * mult * 1.5);
      white =  0;
    }

    grn = constrain(grn , 0, 255);
    blue = constrain(blue , 0, 255);
    red = constrain(red, 0, 255);
    white = constrain(white + notify[0], 0, 255);

    pixels.setPixelColor(0, red, grn, blue, white);
  }




  if (rec_mode == 0) { // just playing



    mixer_in_left.gain(0, 0);
    mixer_in_right.gain(0, 0);
    const byte bottom = 10;
    int grn ;
    int blue;
    int red = 8;
    int white = 12;

    float lvl1 = (peaklerp[2] + peaklerp[3] + peaklerp[1]);
    float mult = 44;

    grn = 3 + (lvl1 * mult * .5);
    blue = 0;
    red = 17 + (lvl1 * mult);
    white = 5 ;

    if (lvl1 >= .65) {
      //    red = 17 + lvl1 * (mult * 1.5);
    }

    grn = constrain(grn + notify[1], 0, 255);
    blue = constrain(blue  , 0, 255);
    red = constrain(red - notify[1] , 0, 255);
    white = constrain(white + notify[0] - notify[1], 0, 255);

    pixels.setPixelColor(0, red, grn, blue, white);
  }

  if (rec_mode == 1) { // warning

    mixer_in_left.gain(0, 0);
    mixer_in_right.gain(0, 0);

    if (mode_timer_latch == 1) {
      mode_timer = 0;
      mode_cm = cm;
      mode_timer_latch = 0;
      rec_mode = 5;
    }

    loop_latch[1] = 0;
    loop_latch[2] = 0;
    for (int i = 0; i < 4; i++) {
      sample_stop(i);
    }
    if (cm - prev_blink[0] > 250) {
      prev_blink[0] = cm;
      blink[0] = !blink[0];
    }
    pixels.setPixelColor(0, 0, 0, 0, 1 + (blink[0] * 20));

  }

  if (rec_mode == 2) { // erasing
    pixels.setPixelColor(0, 30, 30, 30);

    mixer_in_left.gain(0, 0);
    mixer_in_right.gain(0, 0);
    if (rec_target < 0) {
      rec_mode = 0;
    }

    else {
      eraseBlocks(sfblocks * rec_target * 4, sfblocks * 4);
      rec_mode = 3;
    }

  }

  if (rec_mode == 3) { // waiting and listening

    ingain = (((analogRead(LPOT) - 4095) * -1) / (4095.00 / 1.5)) + .1;

    mixer_in_left.gain(0, ingain);
    mixer_in_right.gain(0, ingain);

    int wp = 0;
    int pklvl = 5 + (peaklerp[1] * 60);
    if (pklvl > 200) {
      pklvl = 200;
    }
    if (peaklerp[1] >= .75) {
      wp = 5 + (peaklerp[1] * 20);
      pklvl /= 4;
    }
    pixels.setPixelColor(0, pklvl, 0, 0, wp);


    if (button1Play.fallingEdge()) {
      startRecording();
      rec_mode = 4;

    }

  }

  if (rec_mode == 4) { // recording and listening



    //loop_en[2]=0;
    if (button1Play.risingEdge()) {
      stopRecording();
      rec_mode = 0;
    }

  }




  if (rec_mode == 5) { // erasing and copy single from sd
    pixels.setPixelColor(0, 30, 0, 0);

    mixer_in_left.gain(0, 0);
    mixer_in_right.gain(0, 0);


    //  eraseBlocks(sfblocks * rec_target * 4, sfblocks * 4);
    AudioNoInterrupts();
    /*
      if (!SD.begin(SDchipSelect)) {
      Serial.println("No SD card");
      for (byte wr; wr < 3; wr++) {
       pixels.setPixelColor(0, 43, 17, 0 , 0);
       pixels.show();
       delay(100);
       pixels.setPixelColor(0, 8, 4, 0, 0);
       pixels.show();
       delay(150);
      }
      }
    */
    copyOneFromSD(rec_target);
    AudioInterrupts();
    rec_mode = 0;
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////// end of loop

void savelen(uint16_t slot, uint32_t lene) {
  int16_t start = slot * 4;
  EEPROM.write(start, lene >> 24);
  EEPROM.write(start + 1, lene >> 16);
  EEPROM.write(start + 2, lene >> 8);
  EEPROM.write(start + 3, lene & 0xFF);
}

uint32_t readlen(uint16_t slot) {
  int16_t start = slot * 4;
  byte b4 = EEPROM.read(start);
  byte b3 = EEPROM.read(start + 1);
  byte b2 = EEPROM.read(start + 2);
  byte b1 = EEPROM.read(start + 3);
  uint32_t lt = (b4 << 24) | (b3 << 16) | (b2 << 8) | (b1 );
  return lt;
}

void copyOneFromSD(byte num) {
  byte sample_num;
  uint32_t filelength;
  File root = SD.open("/");

  while (true) {

    File filetocopy =  root.openNextFile();
    if (! filetocopy) {
      Serial.println(" no file to copy");
      break;
    }

    //Serial.println(filetocopy.name());
    String existingname = filetocopy.name();
    String n = num + 1;
    sample_num = num ;
    String c = filetocopy.name();
    char namearraychar[11];
    byte namearraybyte;
    byte hexarray[11] = {0, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40};
    c.toCharArray(namearraychar, 11);

    for (int j = 0; j < 11 ; j ++)
    {
      //   Serial.print(namearray[j], HEX);
      //  Serial.print(" ");
    }
    //  Serial.println(" ");
    byte load_it = 0;

    if (namearraychar[0] == hexarray[(num + 1)] && 1 == 0) {
      namearraybyte = (num + 1);
      Serial.print(namearraychar[0], HEX);
      Serial.print(" ");
      Serial.print(namearraychar[0]);
      Serial.print(" ");
      Serial.println(num + 1);

    }
    if (num < 9) {
      if (namearraychar[0] == hexarray[(num + 1)] && namearraychar[1] != 0x30) {
        load_it = 1;
      }
    }
    if (num == 9) {
      if (namearraychar[0] == 0x31 && namearraychar[1] == 0x30 ) {
        load_it = 1;
      }
    }
    //  if (c == n + ".WAV" || c == n + ".wav" ) { //must be in caps???
    //  if (namearraybyte == (num + 1)) {
    // if (namearraychar[0] == hexarray[(num + 1)]) {
    if (load_it == 1) {

      Serial.println(c  + " is on card");
      eraseBlocks(sfblocks * sample_num * 4, sfblocks * 4);

      const char *filename = filetocopy.name();
      Serial.print("file length ");
      Serial.print("    ");
      unsigned long filelength = filetocopy.size();
      Serial.print(filelength);

      Serial.print("   sample_num ");
      Serial.println(sample_num);
      uint32_t seekb = sample_num * sfblocks * 0x10000 * 4;
      const byte wav_header_size = 44;
      const int buffsize = 1024;

      byte b1[buffsize];
      byte b2[buffsize * 2];


      filetocopy.seek(0);
      filetocopy.read(b1, wav_header_size);

      byte num_channels = b1[22];
      Serial.print("   num_channels ");
      Serial.print(num_channels);

      byte bits_per_sample = b1[34];
      Serial.print("   bits per sample ");
      Serial.println(bits_per_sample);

      if (num_channels == 10) {

        for (uint32_t i = wav_header_size; i < filelength - buffsize - 16; i += buffsize)
        {
          int k = 0;
          filetocopy.seek(i);
          filetocopy.read(b1, buffsize);

          for (int j = 0; j < (buffsize * 2) ; j += 4)
          {
            b2[j]     = b1[k];
            b2[j + 1] = b1[k + 1];
            b2[j + 2] = 0;
            b2[j + 3] = 0;
            k += 2;
          }

          SerialFlash.write(i + seekb, b2, buffsize * 2);

        }
        samplelen[sample_num] = filelength / 2;
        savelen(sample_num, samplelen[sample_num]);
      }


      if (num_channels == 1) {

        for (uint32_t i = wav_header_size; i < filelength - buffsize - 16; i += buffsize)
        {
          filetocopy.seek(i);
          filetocopy.read(b1, buffsize);
          SerialFlash.write(i + seekb, b1, buffsize);
        }
        samplelen[sample_num] = filelength / 4;
        savelen(sample_num, samplelen[sample_num]);
      }


      if (num_channels == 2) {

        for (uint32_t i = wav_header_size; i < filelength - buffsize - 16; i += buffsize)
        {
          filetocopy.seek(i);
          filetocopy.read(b1, buffsize);
          SerialFlash.write(i + seekb, b1, buffsize);
        }
        samplelen[sample_num] = filelength / 4;
        savelen(sample_num, samplelen[sample_num]);
      }


    }


    filetocopy.close();


  }


  root.close();
  delay(10);
}

void cv_read() {
  cvin1 = read_cv(CVi1);
  // Serial.println(cvin1);
  cvin2 = read_cv(CVi2);
  left_pot_temp =  (digitalSmooth(4, 19 , (analogRead(LPOT) - 4095) * -1));
  right_pot_temp =  (digitalSmooth(3, 19 , (analogRead(RPOT) - 4095) * -1));

  // left_pot_temp =  500;
  // right_pot_temp =  500;

  left_freq = sample_freq(left_pot_temp + cvin1);
  right_freq = sample_freq(right_pot_temp + cvin2);



  for (int i = 0; i < 4; i++)

  {
    if (poly_but_ass[i] == 1) {


      if (left_freq < 0) {
        //  sampler[i]->sample_reverse(1);
        //    sampler[i]->frequency(left_freq * -1);
        sample_set_freq(i, left_freq * -1);
        sample_rev_enable(i, 1);
      }
      if (left_freq >= 0) {
        sample_set_freq(i, left_freq);
        sample_rev_enable(i, 0);
        //   sampler[i]->sample_reverse(0);
        //   sampler[i]->frequency(left_freq);
      }
    }
    if (poly_but_ass[i] == 2) {

      if (right_freq < 0) {
        //    sampler[i]->sample_reverse(1);
        //    sampler[i]->frequency(right_freq * -1);
        sample_set_freq(i, right_freq * -1);
        sample_rev_enable(i, 1);

      }
      if (right_freq >= 0) {
        sample_set_freq(i, right_freq);
        sample_rev_enable(i, 0);

        //  sampler[i]->sample_reverse(0);
        //  sampler[i]->frequency(right_freq);
      }

    }
  }
}


void startRecording() {
  Serial.print("startRecording address:");
  int16_t rec_count = 0;;
  //address = bankstart[sample_sel];
  address = (bankstart[rec_target] / 2) + foffset;
  Serial.println(address);

  queue_left.begin();
  queue_right.begin();

  mode = 1;


}

void continueRecording() {
  byte q1a = queue_left.available();
  byte q2a = queue_right.available();


  if (q1a == 1 && q2a == 1) {

    int16_t buffer_in_left[(getsize * 4) + 10];
    int16_t buffer_in_right[(getsize * 4) + 10];
    int16_t buffer_out[(getsize * 2) + 10];

    memcpy(buffer_in_left, queue_left.readBuffer(), getsize);
    memcpy(buffer_in_right, queue_right.readBuffer(), getsize);


    for (uint32_t i = 0; i < getsize; i ++) {
      uint16_t j = i * 2;

      buffer_out[j] = buffer_in_left[i];
      buffer_out[j + 1] = buffer_in_right[i];

      ///      buffer2[j + 1] = 0; //right channel
    }

    SerialFlash.write(address * 2, buffer_out, getsize * 2);
    queue_left.freeBuffer();
    queue_right.freeBuffer();

    address += getsize;

    if (address > ((rec_size * 2) + (bankstart[rec_target]) / 2)) {
      //  Serial.println("! ");
      stopRecording() ;
    }


  }

}

void stopRecording() {
  samplelen[rec_target] = ((address) - ((bankstart[rec_target] / 2) + foffset)) / 2;
  savelen(rec_target, samplelen[rec_target]);

  Serial.print("stopRecording  ");

  Serial.print(address);

  Serial.print(" "); Serial.println(samplelen[rec_target]);
  queue_right.end();

  queue_left.end();
  if (mode == 1) {
    while (queue_left.available() > 0) {
      queue_left.freeBuffer();
      queue_left.clear();
      queue_right.freeBuffer();
      queue_right.clear();
    }
  }

  //sample0.begin(1, norm, 0, samplelen[0]);
  //sample1.begin(1, norm, rec_target, samplelen[1]);
  rec_mode = 0;
  mode_timer = 0;
  mode_cm = cm;
  mode_timer_latch = 0;
}

uint16_t flip_pot(int pin) {
  uint16_t flip = (analogRead(pin) - 4095) * -1;
  return flip;
}


uint16_t read_cv(int pin) {
  // cv is from 540 - 2900 1375 is zero
  int16_t cvr = (analogRead(pin) + cv_offset); // seems pretty ok from the sq1 and 0coast without scaling
  if (cvr < 6 && cvr > -6) {
    cvr = 0;
  }

  return cvr;
}

float sample_freq(float in) {
  int in_max = 4095;
  float r1 = in;

  if (r1 > in_max) {
    // r1 = in_max;
  }

  const float max_speed_mult = 1.99 ;
  const float low_speed_mult = .05 ;
  const float low_speed_mult_inv = 1 - low_speed_mult ;


  const float maxspeed = norm * max_speed_mult;
  float out;
  const float div4 = (in_max / 4);
  const float div2 = (in_max / 2);

  if (r1 >= div2) {

    float t1 = (r1 - div2);

    out = (t1 * ((max_speed_mult - 1) / div2) ) + 1;
  }

  if (r1 < div2 && r1 > div4) {
    //out = (r1 - div4) / div4;
    out = (((r1 - div4) / div4) * low_speed_mult_inv) + low_speed_mult;
  }

  if (r1 <= div4) {
    out = ((r1 / (div4) - 1) * (max_speed_mult - low_speed_mult)) - low_speed_mult;


  }
  out *= norm;

  if (out > maxspeed) {
    out = maxspeed;

  }

  if (out < maxspeed * -1) {
    out = maxspeed * -1;

  }

  return out;

}

void eraseBlocks(int estart, int elen) {
  byte jj = 0;
  byte hh = 0;
  byte gg = 1;
  Serial.print("erase block:");
  Serial.println(estart);
  for (int i = estart; i < estart + elen; i++) {
    uint32_t eb = (i * 0x10000);
    SerialFlash.eraseBlock(eb);

    int dotcount;
    while (SerialFlash.ready() == false) {
      if (millis() - prev[4] > 1000) {
        prev[4] = millis();
        blink[0] = !blink[0];
      }

      if (millis() - prev[6] > 40) {
        prev[6] = millis();
        jj++;
        if (jj > 20) {
          jj = 0;
          hh = !hh;
          gg = !gg;
        }
        if (sd_green == 1) {
          pixels.setPixelColor(0, (0 * gg) + random(2) + 1, (24 * gg) + random(4) + 1, 4, hh * (8 + random(4)));
        }
        else {
          pixels.setPixelColor(0, (35 * gg) + random(4) + 1, (10 * gg) + random(4) + 1, 0, hh * (8 + random(4)));
        }

        pixels.show();
      }
    }
  }
  sd_green = 0;
  Serial.println(" ");
}


void read_contacts() {
  ctrc = millis();

  if (ctrc - ptrc[0] > 1 && 1 == 1) {
    ptrc[0] = ctrc;
    tick = !tick;

    rec_target = -1;
    rec_target_latch = 0;
    voice_max = 0;
    left_poly_sel = 0;
    right_poly_sel = 0;


    read_shift_regs();

    for (int i = 0; i < 4; i++)
    {
      if (prev_poly_sample_sel[i] != poly_sample_sel[i]) {
        sample_stop(i);
        //   Serial.print("sstop ");
        //   Serial.println(i);
      }

      // prev_poly_sample_sel[i] = poly_sample_sel[i];

      if (poly_but_ass[i] == 1)
      {
        byte cleanup = left_en_contacts[poly_sample_sel[i] - 1] ;
        if (cleanup == 0)
        {
          poly_sample_sel[i] = 0;
          poly_but_ass[i] = 0;
          sample_stop(i);

        }
      }

    }

    for (int i = 0; i < 10; i++)
    {
      byte readleft = left_en_contacts[i];

      if (readleft == 1) {
        if (rec_target_latch == 0) {
          rec_target = i;
          rec_target_latch = 1;
        }

        if (left_poly_sel < 4) {
          left_playing[left_poly_sel] = i + 1;
          left_poly_sel++;
        }

        for (int j = 0; j < 4; ++j)
        {
          byte vl = 0;

          for (int k = 0; k < 4; ++k)
          {
            if (poly_sample_sel[k] == i + 1  && poly_but_ass[k] == 1) {
              vl = 1;
            }
          }

          if (poly_but_ass[j] == 0 && vl == 0) {
            poly_sample_sel[j] = i + 1;
            poly_but_ass[j] = 1;
            vl = 1;
          }
        }

      }

    }


    for (int i = 0; i < 4; i++)
    {
      if (prev_poly_sample_sel[i] != poly_sample_sel[i]) {
        sample_stop(i);
        //   Serial.print("sstop ");
        //   Serial.println(i);
      }

      prev_poly_sample_sel[i] = poly_sample_sel[i];

      if (poly_but_ass[i] == 2)
      {
        byte cleanup = right_en_contacts[poly_sample_sel[i] - 1];
        if (cleanup == 0)
        {
          poly_sample_sel[i] = 0;
          poly_but_ass[i] = 0;
          sample_stop(i);

        }
      }

    }

    for (int i = 0; i < 10; i++)
    {
      byte readright = right_en_contacts[i];

      if (readright == 1) {

        if (right_poly_sel < 4) {
          right_playing[right_poly_sel] = i + 1;
          right_poly_sel++;
        }

        for (int j = 0; j < 4; ++j)
        {
          byte vr = 0;

          for (int k = 0; k < 4; ++k)
          {
            if (poly_sample_sel[k] == i + 1 && poly_but_ass[k] == 2) {
              vr = 1;
            }
          }

          if (poly_but_ass[j] == 0 && vr == 0) {
            poly_sample_sel[j] = i + 1;
            poly_but_ass[j] = 2;
            vr = 1;
          }
        }

      }

    }




  }


}


byte srtick;
void read_shift_regs()
{
  uint16_t bitVal;
  rawsr = 0;
  // srtick = !srtick;

  digitalWrite(left_contact, 0);
  digitalWrite(right_contact, 1);

  digitalWrite(ploadPin, 0);
  digitalWrite(ploadPin, 1);
  byte tempread[16];

  for (int i = 0; i < 16; i++)
  {
    bitVal = digitalRead(srdataPin);
    rawsr |= (bitVal << ((DATA_WIDTH - 1) - i));
    tempread[i] = bitVal;
    digitalWrite(srclockPin, 1);
    digitalWrite(srclockPin, 0);
  }

  digitalWrite(left_contact, 1);
  digitalWrite(right_contact, 1);



  for (int i = 0; i < 10; i++)
  {
    left_en_contacts[i] = tempread[shiftin_pin_oder[i]];
  }

  digitalWrite(left_contact, 1);
  digitalWrite(right_contact, 0);


  digitalWrite(ploadPin, 0);
  digitalWrite(ploadPin, 1);
  //rawsr = 0;

  for (int i = 0; i < 16; i++)
  {
    bitVal = digitalRead(srdataPin);
    //   rawsr |= (bitVal << ((DATA_WIDTH - 1) - i));
    tempread[i] = bitVal;
    digitalWrite(srclockPin, 1);
    digitalWrite(srclockPin, 0);
  }

  digitalWrite(left_contact, 1);
  digitalWrite(right_contact, 1);

  for (int i = 0; i < 10; i++)
  {
    right_en_contacts[i] = tempread[shiftin_pin_oder[i]];
  }



}


void print_contacts() {
  if (ctrc - ptrc[1] > 50 && 1 == 1) {
    ptrc[1] = ctrc;
    //printMon();
    for (int i = 0; i < 10; i++)
    {
      byte z = 0;
      if (left_en_contacts[i] == 1) {
        Serial.print("L");
        z = 1;
      }

      if (right_en_contacts[i] == 1) {
        Serial.print("R");
        z = 1;
      }

      if (z == 0) {
        Serial.print("-");
      }
      Serial.print("\t");

      if (i == 4) {
        Serial.println("");
      }

    }

    Serial.println();
    /*
      Serial.print("poly_sample_sel ");

      for (int i = 0; i < 4; i++)
      {
      Serial.print(poly_sample_sel[i]);
      Serial.print(" ");
      }
      Serial.println();
      Serial.print("poly_but_ass    ");

      for (int i = 0; i < 4; i++)
      {
      Serial.print(poly_but_ass[i]);
      Serial.print(" ");
      }
      byte ppp = 0;
      if (ppp == 1) {
      Serial.println();
      Serial.println();
      for (int i = 0; i < 4; i++)
      {
        Serial.print(left_playing[i]);
        Serial.print(" ");
      }
      Serial.println();

      for (int i = 0; i < 4; i++)
      {
        Serial.print(right_playing[i]);
        Serial.print(" ");
      }
      Serial.println();
      }
      Serial.println();

      Serial.println("rec_target ");

      Serial.println(rec_target);
      Serial.println();
      Serial.println();
    */
  }
}

void printMon() {
  //Serial.print(" ");

  Serial.print(AudioProcessorUsageMax());  //go over 90% and you'll start to glitch out
  Serial.print("  ");
  Serial.print(AudioMemoryUsageMax()); //if this number is bigger than what you entered in  AudioMemory(??) in setup make it bigger
  Serial.print("  ");
  Serial.println(foundhigh[0]); //loop millis

  reset_find_high(0);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
}


void play_lefts() {
  for (int i = 0; i < 4; i++)
  {
    if (poly_but_ass[i] == 1) {
      byte ps = poly_sample_sel[i] - 1;
      sample_play(i, ps);
    }
  }
}

void play_rights() {
  for (int i = 0; i < 4; i++)
  {
    if (poly_but_ass[i] == 2) {
      byte ps = poly_sample_sel[i] - 1;
      sample_play(i, ps);
    }
  }
}


#define maxarrays 10
#define maxsamples 24

uint16_t smoothArray[maxarrays][maxsamples];


int digitalSmooth(int arrayn, int filterSamples, int rawIn) {   // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
  static int sorted[maxarrays][maxsamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  smoothArray[arrayn][i] = rawIn;                 // input new data into the oldest slot



  for (j = 0; j < filterSamples; j++) { // transfer data array into anther array for sorting and averaging
    sorted[arrayn][j] = smoothArray[arrayn][j];
  }

  done = 0;                // flag to know when we're done sorting
  while (done != 1) {      // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++) {
      if (sorted[arrayn][j] > sorted[arrayn][j + 1]) {    // numbers are out of order - swap
        temp = sorted[arrayn][j + 1];
        sorted[arrayn] [j + 1] =  sorted[arrayn][j] ;
        sorted[arrayn] [j] = temp;
        done = 0;
      }
    }
  }


  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1);
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j < top; j++) {
    total += sorted[arrayn][j];  // total remaining indices
    k++;

  }

  return total / k;    // divide by number of samples
}



void printBits(uint16_t myByte) {
  byte cc = 0;

  for (uint16_t mask = 0x8000; mask; mask >>= 1) {
    Serial.print(cc);
    if (mask  & myByte) {
      Serial.print("* ");
    }
    else {
      Serial.print("  ");
    }
    cc++;
  }
  Serial.println("B ");

}

//    sampler[i]->sample_play_loc(bankstart[ps], samplelen[ps]);

void sample_play(byte num, byte loc) {
  switch (num) {
    case 0:
      sample0.sample_play_loc(bankstart[loc], samplelen[loc]);
      break;

    case 1:
      sample1.sample_play_loc(bankstart[loc], samplelen[loc]);
      break;

    case 2:
      sample2.sample_play_loc(bankstart[loc], samplelen[loc]);
      break;

    case 3:
      sample3.sample_play_loc(bankstart[loc], samplelen[loc]);
      break;
  }

}


void sample_stop(byte num) {
  switch (num) {
    case 0:
      sample0.sample_stop();
      break;

    case 1:
      sample1.sample_stop();
      break;

    case 2:
      sample2.sample_stop();
      break;

    case 3:
      sample3.sample_stop();
      break;
  }

}
void sample_set_freq(byte num, float frq) {
  switch (num) {
    case 0:
      sample0.frequency(frq);
      break;

    case 1:
      sample1.frequency(frq);
      break;

    case 2:
      sample2.frequency(frq);
      break;

    case 3:
      sample3.frequency(frq);
      break;
  }

}

void sample_rev_enable(byte num, byte en) {
  switch (num) {
    case 0:
      sample0.sample_reverse(en);
      break;

    case 1:
      sample1.sample_reverse(en);
      break;

    case 2:
      sample2.sample_reverse(en);
      break;

    case 3:
      sample3.sample_reverse(en);
      break;
  }
}
void sample_loop_enable(byte num, byte en) {
  switch (num) {
    case 0:
      sample0.sample_loop(en);
      break;

    case 1:
      sample1.sample_loop(en);
      break;

    case 2:
      sample2.sample_loop(en);
      break;

    case 3:
      sample3.sample_loop(en);
      break;
  }
}

byte sample_loop_trig(byte num) {
  byte out;
  switch (num) {
    case 0:
      out = sample0.loop_trig();
      break;

    case 1:
      out = sample1.loop_trig();
      break;

    case 2:
      out = sample2.loop_trig();
      break;

    case 3:
      out = sample3.loop_trig();
      break;
  }
  return out;
}



float read_peaks(byte num) {
  float out;
  switch (num) {
    case 1:
      out = peakin.read();
      break;
    case 2:
      out = peakleft.read();
      break;

    case 3:
      out = peakright.read();
      break;
    default:
      // statements
      break;
  }
  return out;
}


byte peaks_avail(byte num) {
  byte out;
  switch (num) {
    case 1:
      out = peakin.available();
      break;

    case 2:
      out = peakleft.available();
      break;

    case 3:
      out = peakright.available();
      break;
    default:
      // statements
      break;
  }
  return out;
}



uint16_t reset_find_high (byte n) {
  fh[n] = 0;
}

uint16_t find_high (byte n, int16_t in) {


  if (in > fh[n]) {
    fh[n] = in;
  }

  else {

  }

  return fh[n];


}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      //Serial.println("**nomorefiles**");
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
