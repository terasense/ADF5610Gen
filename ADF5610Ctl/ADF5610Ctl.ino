//
// ADF5610 controller
//

#include "ADF5610.h"
#include "ArBtn.h"
#include "QEnc.h"
#include "NvTx.h"
#include "SSD1305_SPI_Adaptor.h"
#include "glcd_fonts.h"

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

#define TIMER_PERIOD 500 // usec
#define DEF_FREQ 10000 // MHz
#define UNITS "MHz"

#define BAUDS 9600
#define IDLE_DELAY 50 // msec

static Btn                 g_btn(BT_PIN);
static QEnc                g_enc(PIN_A, PIN_B);
static SSD1305_SPI_Adaptor g_display(CS_PIN, RST_PIN, DC_PIN);
static ADF5610             g_adf(SEN_PIN);

static bool     g_initialized = false;
static bool     g_locked      = false;
static bool     g_out_on      = false;
static bool     g_failed      = false;
static bool     g_remote      = false;
static bool     g_tune        = false;
static uint32_t g_freq        = DEF_FREQ; // Mhz
static uint8_t  g_fmul        = 1;
static int32_t  g_tune_val    = 0;
static uint16_t g_tune_step   = 100; // Possible values: 1,10,100,1000
static uint8_t  g_tune_pos    = 2;   // tune_step = 10^tune_pos

// Assign EEPROM addresses

#define INST_ID 0x1111

NvPlace(g_out_on, 0x10, INST_ID);
NvAfter(g_freq, g_out_on);
NvAfter(g_fmul, g_freq);

#define MAX_STEP 1000 // in freq tuning mode
#define ENC_DIV  2    // encoder divider

ISR(TIMER1_OVF_vect)
{
  g_btn.poll();
  g_enc.poll();
}

static void timer_init(unsigned us_period)
{
  TCCR1A = 0;  // clear control register A 
  ICR1 = (F_CPU / 2000000) * us_period; // it will count up/down
  TCCR1B = _BV(WGM13) | _BV(CS10); // simmetric PWM, no prescaler
  TIMSK1 = _BV(TOIE1); // enable interrupt
}

static void init_nv_params()
{
  bool out_valid  = NvTxGet(g_out_on);
  bool freq_valid = NvTxGet(g_freq);
  bool fmul_valid = NvTxGet(g_fmul);
  if (!out_valid || !freq_valid || !g_fmul) {
    // falback to defaults to be on the safe side
    if (!g_fmul)
      g_fmul = 1;
    g_freq = DEF_FREQ * (uint32_t)g_fmul;
    g_out_on = false;
  }
}

static void adf_set_freq()
{
  if (!g_adf.set_freq(g_freq, g_fmul)) {
    // Invalid frequency. Reset to default.
    g_freq = DEF_FREQ * (uint32_t)g_fmul;
    g_out_on = false;
    g_adf.vco_disable();
  }
}

static void adf_init()
{
  g_adf.begin();
  if (!g_adf.probe()) {
    g_failed = true;
    g_out_on = false;
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

static void adf_poll_status()
{
  g_failed = !g_adf.probe();
  g_locked = g_adf.lock_status();
}

#define STATUS_WIDTH 32

static void display_freq()
{
  String sfreq(g_freq);
  sfreq += UNITS;
  sfreq.remove(8);
  int len = sfreq.length();
  struct glcd_patch patches[len] = {};
  if (g_tune) {
    struct glcd_patch tune_marker = {.type = patch_strike, .where = 2, .param = 0xf0};
    patches[len-sizeof(UNITS)-g_tune_pos] = tune_marker;
  }
  glcd_print_str_r_ex(&g_display, STATUS_WIDTH, 1, g_display.width() - STATUS_WIDTH, sfreq.c_str(), &g_font_Tahoma19x20, 1, patches);
}

static const char* get_status_string()
{
  if (!g_initialized || g_failed)
    return "Err";
  else if (!g_out_on)
    return "Off";
  else if (!g_locked)
    return "On";
  else
    return "ON";
}

static void display_status()
{
  glcd_print_str_w(&g_display, 0, 0, STATUS_WIDTH, get_status_string(), &g_font_Tahoma15x16, 2);
  if (g_remote)
    glcd_print_str_w(&g_display, 0, 2, STATUS_WIDTH, "rem", &g_font_Tahoma15x16, 2);
  else if (g_fmul > 1) {
    String smult('*');
    smult += g_fmul;
    glcd_print_str_w(&g_display, 0, 2, STATUS_WIDTH, smult.c_str(), &g_font_Tahoma12x11Bld, 1);
  } else
    g_display.clear_region(0, 2, STATUS_WIDTH, 2);
}

void setup()
{
  Serial.begin(BAUDS);
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

static void switch_tune_mode()
{
  g_tune = !g_tune;
  if (g_tune)
    g_tune_val = g_enc.value() / ENC_DIV;
  else
    NvTxPut(g_freq);
  display_freq();
}

static void tune_switch_step()
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

static void do_tune()
{
  int32_t val = g_enc.value() / ENC_DIV;
  if (val == g_tune_val)
    return;
  uint32_t freq = g_freq + (val - g_tune_val) * g_tune_step;
  g_tune_val = val;
  if (g_btn.is_pressed())
    return;
  uint32_t limit;
  if (freq < (limit = FMIN_MHZ * (uint32_t)g_fmul))
    freq = limit;
  if (freq > (limit = FMAX_MHZ * (uint32_t)g_fmul))
    freq = limit;
  if (g_freq == freq)
    return;
  g_freq = freq;
  if (g_out_on)
    adf_set_freq();
  display_freq();
}

static void switch_output()
{
  g_out_on = !g_out_on;
  if (!g_out_on)
    g_adf.vco_disable();
  else {
    g_adf.vco_enable();
    adf_set_freq();
  }
  NvTxPut(g_out_on);
}

static void check_events()
{
  if (g_remote) {
    if (g_btn.get_event() == bt_long_pressed) {
      g_remote = false;
    }
    return;
  }
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

static String g_rx_buff;

#define HELP \
"Commands available:\n" \
"i      - Identify. Returns <model> <min MHz> <max MHz>\n" \
"v      - Returns major.minor version followed by build date\n" \
"f<MHz> - Set frequency\n" \
"m<Mul> - Set frequency multiplier\n" \
"o(0|1) - Turns output on (1) or off (0)\n" \
"l(0|1) - Lock (1) or unlock (0). While locked manual controls are disabled\n" \
"         Commands f, m, o, l without parameter return the value currently set\n" \
"p      - Persist current frequency and output status (to be restored on next power on)\n" \
"s      - Returns current status as Off|ON|On|Err. The 'On' status is the same\n" \
"         as 'ON' but with the former the frequency is not locked to desired value\n" \
"?      - This help\n" \
"On success all commands return empty line. On error the line with either !INVALID\n" \
"(command error) or !FAILED (device error) will be returned.\n" \
"Copyright 2019-2020 TeraSense Group, Inc.\n"

#define VERSION "1.0"
#define MODEL "ADF5610"
#define INVAL_RESP "!INVALID"
#define FAIL_RESP  "!FAILED"

static void cli_freq(String &cmd)
{
  if (!cmd.length()) {
    Serial.println(g_freq);
    return;
  }
  uint32_t freq = cmd.toInt();
  if (freq < FMIN_MHZ * (uint32_t)g_fmul || FMAX_MHZ * (uint32_t)g_fmul < freq) {
    Serial.println(INVAL_RESP);
    return;
  }
  g_freq = freq;  
  if (g_out_on)
    adf_set_freq();
  display_freq();
  Serial.println();
}

static void cli_fmul(String &cmd)
{
  if (!cmd.length()) {
    Serial.println(g_fmul);
    return;
  }
  uint8_t mul = cmd.toInt();
  if (!mul) {
    Serial.println(INVAL_RESP);
    return;
  }
  g_freq = (g_freq / g_fmul) * mul;
  g_fmul = mul;
  NvTxPut(g_freq);
  NvTxPut(g_fmul);
  if (g_out_on)
    adf_set_freq();
  display_freq();
  Serial.println();
}

static void cli_outp(String &cmd)
{
  if (!cmd.length()) {
    Serial.println(g_out_on ? '1' : '0');
    return;
  }
  switch (cmd.charAt(0)) {
    case '0':
      if (g_out_on) {
        g_out_on = false;
        g_adf.vco_disable();
      }
      Serial.println();
      break;
    case '1':
      if (g_failed) {
        Serial.println(FAIL_RESP);
        break;
      }
      if (!g_out_on) {
        g_out_on = true;
        g_adf.vco_enable();
        adf_set_freq();
      }
      Serial.println();
      break;
    default:
      Serial.println(INVAL_RESP);
  }
}

static void cli_lock(String &cmd)
{
  if (!cmd.length()) {
    Serial.println(g_remote ? '1' : '0');
    return;
  }
  switch (cmd.charAt(0)) {
    case '0':
      g_remote = false;
      Serial.println();
      break;
    case '1':
      g_remote = true;
      g_tune = false;
      display_freq();
      Serial.println();
      break;
    default:
      Serial.println(INVAL_RESP);
  }
}

static void cli_persist()
{
  NvTxPut(g_freq);
  NvTxPut(g_out_on);
  Serial.println();
}

static void cli_process_cmd(String &cmd)
{
  char tag = cmd.charAt(0);
  cmd.remove(0, 1);
  switch (tag) {
    case '?':
      Serial.println(F(HELP));
      break;
    case 'i':
      Serial.print(MODEL);
      Serial.print(' ');
      Serial.print(FMIN_MHZ);
      Serial.print(' ');
      Serial.println(FMAX_MHZ);
      break;
    case 'v':
      Serial.println(VERSION " " __DATE__);
      break;
    case 's':
      Serial.println(get_status_string());
      break;
    case 'f':
      cli_freq(cmd);
      break;
    case 'm':
      cli_fmul(cmd);
      break;
    case 'o':
      cli_outp(cmd);
      break;
    case 'l':
      cli_lock(cmd);
      break;
    case 'p':
      cli_persist();
      break;
    default:
      Serial.println(INVAL_RESP);
  }
}

#define MAX_CMD_LEN 32

static void serial_process()
{
  char c = 0;
  while (Serial.available()) {
    c = Serial.read();
    if (g_rx_buff.length() <= MAX_CMD_LEN)
      g_rx_buff += c;
  }
  // check command is completed
  if (c != '\n' && c != '\r')
    return;

  // make the copy of the buffer
  String cmd = g_rx_buff;
  g_rx_buff = String();

  if (cmd.length() > MAX_CMD_LEN) {
    Serial.println(INVAL_RESP);
    return;
  }

  // trim white space characters
  cmd.trim();

  // process remaining symbols
  if (cmd.length())
    cli_process_cmd(cmd);
}

void loop() {
  serial_process();
  if (g_initialized) {
    check_events();
    adf_poll_status();
    display_status();
  }
  delay(IDLE_DELAY);
}
