//
// ADF5610 controller
//

#include "ADF5610.h"
#include "ArBtn.h"
#include "QEnc.h"
#include "SSD1305_SPI_Adaptor.h"
#include "glcd_fonts.h"
#include "nv_utils.h"

// ADF5610 interface enable pin
#define SEN_PIN 17

// Rotary encoder knob (with button)
#define BT_PIN  4
#define PIN_A   2
#define PIN_B   3

// Display connection
#define CS_PIN  14
#define DC_PIN  15
#define RST_PIN 16

// EEPROM addresses
#define ON_ADDR   0x10
#define FREQ_ADDR 0x20

#define TIMER_PERIOD 500 // usec
#define DEF_FREQ 10000 // MHz
#define UNITS "MHz"

Btn                 g_btn(BT_PIN);
QEnc                g_enc(PIN_A, PIN_B);
SSD1305_SPI_Adaptor g_display(CS_PIN, RST_PIN, DC_PIN);
ADF5610             g_adf(SEN_PIN);

bool     g_initialized = false;
bool     g_locked      = false;
bool     g_out_on      = false;
bool     g_failed      = false;
bool     g_tune        = false;
uint16_t g_freq        = DEF_FREQ; // Mhz
int32_t  g_tune_val    = 0;
uint16_t g_tune_step   = 100; // Possible values: 1,10,100,1000
uint8_t  g_tune_pos    = 2;   // tune_step = 10^tune_pos

#define MAX_STEP 1000
#define ENC_DIV  2 // encoder divider

ISR(TIMER1_OVF_vect)
{
  g_btn.poll();
  g_enc.poll();
}

void timer_init(unsigned us_period)
{
  TCCR1A = 0;  // clear control register A 
  ICR1 = (F_CPU / 2000000) * us_period; // it will count up/down
  TCCR1B = _BV(WGM13) | _BV(CS10); // simmetric PWM, no prescaler
  TIMSK1 = _BV(TOIE1); // enable interrupt
}

void init_nv_params()
{
  bool out_valid  = nv_get(&g_out_on, sizeof(g_out_on), ON_ADDR);
  bool freq_valid = nv_get(&g_freq, sizeof(g_freq), FREQ_ADDR);
  if (!out_valid || !freq_valid)
    g_out_on = false;
}

void adf_set_freq()
{
  if (!g_adf.set_freq(g_freq)) {
    // Invalid frequency. Reset to default.
    g_freq = DEF_FREQ;
    g_out_on = false;
    g_adf.vco_disable();
  }
}

void adf_init()
{
  g_adf.begin();
  if (!g_adf.probe()) {
    g_failed = true;
    return;
  }
  g_adf.init();
  if (!g_out_on)
    g_adf.vco_disable();
  else
    g_adf.vco_enable();
  adf_set_freq();
  g_initialized = true;
}

void adf_poll_status()
{
  g_failed = !g_adf.probe();
  g_locked = g_adf.lock_status();
}

#define STATUS_WIDTH 32

void display_freq()
{
  String sfreq(g_freq);
  sfreq += UNITS;
  int len = sfreq.length();
  struct glcd_patch patches[len] = {};
  if (g_tune) {
    struct glcd_patch tune_marker = {.type = patch_strike, .where = 2, .param = 0xf0};
    patches[len-sizeof(UNITS)-g_tune_pos] = tune_marker;
  }
  glcd_print_str_r_ex(&g_display, STATUS_WIDTH, 1, g_display.width() - STATUS_WIDTH, sfreq.c_str(), &g_font_Tahoma19x20, 1, patches);
}

void display_status()
{
  const char* sta;
  if (!g_initialized || g_failed)
    sta = "Err";
  else if (!g_out_on)
    sta = "Off";
  else if (!g_locked)
    sta = "On";
  else
    sta = "ON";
  glcd_print_str_w(&g_display, 0, 0, STATUS_WIDTH, sta, &g_font_Tahoma15x16, 2);
}

void setup()
{
  Serial.begin(9600);
  init_nv_params();
  g_btn.begin();
  g_enc.begin();
  g_display.begin();
  g_display.init();
  adf_init();
  if (g_initialized) {
    timer_init(TIMER_PERIOD);
    display_freq();
  }
  display_status();
}

void switch_tune_mode()
{
  g_tune = !g_tune;
  if (g_tune)
    g_tune_val = g_enc.value() / ENC_DIV;
  else
    nv_put(&g_freq, sizeof(g_freq), FREQ_ADDR);
  display_freq();
}

void tune_switch_step()
{
  if (g_tune_step == MAX_STEP) {
      g_tune_step = 1;
      g_tune_pos = 0;
  } else {
      g_tune_step *= 10;
      g_tune_pos += 1;
  }
  display_freq();
}

void do_tune()
{
  int32_t val = g_enc.value() / ENC_DIV;
  if (val == g_tune_val)
    return;
  uint16_t freq = g_freq + (val - g_tune_val) * g_tune_step;
  g_tune_val = val;
  if (g_btn.is_pressed())
    return;
  if (freq < FMIN_MHZ)
    freq = FMIN_MHZ;
  if (freq > FMAX_MHZ)
    freq = FMAX_MHZ;
  if (g_freq == freq)
    return;
  g_freq = freq;
  if (g_out_on)
    adf_set_freq();
  display_freq();
}

void switch_output()
{
  g_out_on = !g_out_on;
  if (!g_out_on)
    g_adf.vco_disable();
  else {
    g_adf.vco_enable();
    adf_set_freq();
  }
  nv_put(&g_out_on, sizeof(g_out_on), ON_ADDR);
}

void check_events()
{
  switch(g_btn.get_event()) {
    case bt_released:
      if (!g_tune)
        switch_output();
      else
        tune_switch_step();
      return;
    case bt_long_pressed:
      switch_tune_mode();
      return;
    default:
      ;
  }
  if (g_tune)
    do_tune();
}

void loop() {
  if (g_initialized) {
    check_events();
    adf_poll_status();
    display_status();
  }
  delay(100);
}
