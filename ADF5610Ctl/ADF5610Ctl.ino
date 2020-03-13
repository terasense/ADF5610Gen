//
// ADF5610 controller
//

#include "ADF5610.h"
#include "ArBtn.h"
#include "QEnc.h"
#include "SSD1305_SPI_Adaptor.h"
#include "glcd_fonts.h"
#include "TelNetSrv.h"
#include "NvTx.h"
#include <avr/boot.h>

// ADF5610 interface enable pin
#define SEN_PIN 21

// Rotary encoder knob (with button)
#define BT_PIN  4
#define PIN_A   2
#define PIN_B   3

// Display connection
#define CS_PIN  18
#define DC_PIN  19
#define RST_PIN 20

#define TIMER_PERIOD 500 // usec
#define DEF_FREQ 10000 // MHz

#define BAUDS 9600
#define IDLE_DELAY 50 // msec

#define HOSTNAME "TeraS"
#define DHCP_TOUT 10000
#define TELNET_PORT 23
#define MAX_CMD_LEN 32
#define MAX_PWD_LEN 16

// Uncomment to show multiplier on display
//#define SHOW_MULT

static Btn                 g_btn(BT_PIN);
static QEnc                g_enc(PIN_A, PIN_B);
static SSD1305_SPI_Adaptor g_display(CS_PIN, RST_PIN, DC_PIN);
static ADF5610             g_adf(SEN_PIN);

static bool     g_initialized = false;
static bool     g_eth_present = false;
static bool     g_eth_online  = false;
static bool     g_eth_initing = false;
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

static char     g_pwd[MAX_PWD_LEN];
static bool     g_has_pwd;
static bool     g_auth;

static String    g_rx_buff;
static TelNetSrv g_telnet_srv(TELNET_PORT, MAX_CMD_LEN);

// Assign EEPROM addresses

#define INST_ID 0x1111

NvPlace(g_out_on, 0x10, INST_ID);
NvAfter(g_freq, g_out_on);
NvAfter(g_fmul, g_freq);
NvAfter(g_pwd, g_fmul); 

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
  g_has_pwd = NvTxGet(g_pwd) && g_pwd[0];
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

#define STATUS_WIDTH 45
#define INFO_MARGIN  7

static void display_freq()
{
  String sfreq(g_freq);
  int len = sfreq.length();
  struct glcd_patch patches[len] = {};
  if (g_tune) {
    struct glcd_patch tune_marker = {.type = patch_strike, .where = 2, .param = 0xf0};
    patches[len-1-g_tune_pos] = tune_marker;
  }
  glcd_print_str_r_ex(&g_display, STATUS_WIDTH, 1, g_display.width() - STATUS_WIDTH, sfreq.c_str(), &g_font_Tahoma19x20dg, 1, patches);
}

static const char* get_status_string()
{
  if (!g_initialized || g_failed)
    return "err";
  if (!g_out_on)
    return "off";
  if (!g_locked)
    return "tun";
  return "on";
}

static inline void show_info(const char* str, struct glcd_font const* font)
{
  glcd_print_str_w(&g_display, INFO_MARGIN, 2, STATUS_WIDTH-INFO_MARGIN, str, font, 2);
}

static inline void clear_info()
{
  g_display.clear_region(0, 2, STATUS_WIDTH, 2);
}

static void display_status()
{
  glcd_print_str_w(&g_display, 0, 0, STATUS_WIDTH, get_status_string(), &g_font_Tahoma15x16Small, 2);
  if (g_eth_initing)
    show_info("eth", &g_font_Tahoma15x16Small);
  else if (g_remote)
    show_info("rem", &g_font_Tahoma15x16Small);
#ifdef SHOW_MULT
  else if (g_fmul > 1) {
    String smult('*');
    smult += g_fmul;
    show_info(smult.c_str(), &g_font_Tahoma12x11BldD);
  }
#endif
  else
    clear_info();
}

static void telnet_init()
{
  uint8_t mac[6] = {
    boot_signature_byte_get(0x0E),
    boot_signature_byte_get(0x0F),
    boot_signature_byte_get(0x10),
    boot_signature_byte_get(0x11),
    boot_signature_byte_get(0x12),
    boot_signature_byte_get(0x13)
  };
  mac[0] &= ~1;
  mac[0] |= 2;
  g_eth_initing = true;
  display_status();
  if (Ethernet.begin(mac, HOSTNAME, DHCP_TOUT))
    g_telnet_srv.start();
  g_eth_initing = false;
}

static void on_telnet_connect()
{
  g_auth = false;
}

static void eth_init()
{
  g_telnet_srv.on_connect(on_telnet_connect);
  Ethernet.begin();
  g_eth_present = (Ethernet.hardwareStatus() != EthernetNoHardware);
  if (!g_eth_present)
    return;
  g_eth_online = (Ethernet.linkStatus() == LinkON);
  if (g_eth_online)
    telnet_init();
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
  eth_init();
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

#define HELP \
"Commands available:\r\n" \
"i      - Identify. Returns <model> <min MHz> <max MHz>\r\n" \
"v      - Returns major.minor version followed by build date\r\n" \
"f<MHz> - Set frequency\r\n" \
"m<Mul> - Set frequency multiplier\r\n" \
"o(0|1) - Turns output on (1) or off (0)\r\n" \
"l(0|1) - Lock (1) or unlock (0). While locked manual controls are disabled\r\n" \
"         Commands f, m, o, l without parameter return the value currently set\r\n" \
"p      - Persist current frequency and output status (to be restored on next power on)\r\n" \
"s      - Get current status as off|on|tun|err. The 'tun' (tuning) status is the same\r\n" \
"         as 'on' but with the former the frequency is not locked to desired value\r\n" \
"n      - Get hostname. It may be used to access device by Telnet over the LAN.\r\n" \
"a      - Get current IP address. It is acquired via DHCP on connecting to Ethernet.\r\n" \
"         The reliable network configuration should have IP address statically bound\r\n" \
"         to device MAC address.\r\n" \
"#<PWD> - Authenticate with password\r\n" \
"P<PWD> - Set new password\r\n" \
"?      - This help\r\n" \
"On success all commands return empty line. On error the line with either !INVALID\r\n" \
"(command error) or !FAILED (device error) will be returned.\r\n" \
"Copyright 2019-2020 TeraSense Group, Inc.\r\n"

#define VERSION "1.1"
#define MODEL "ADF5610"
#define INVAL_RESP "!INVALID"
#define FAIL_RESP  "!FAILED"

static void cli_pwd_auth(String &cmd, Print &out)
{
  if (!g_has_pwd) {
    out.println(INVAL_RESP);
    return;
  }
  unsigned pwd_len = cmd.length() + 1;
  if (pwd_len > MAX_PWD_LEN)
    pwd_len = MAX_PWD_LEN;
  g_auth = !memcmp(g_pwd, cmd.c_str(), pwd_len);
  out.println();
}

static void cli_pwd_set(String &cmd, Print &out)
{
  unsigned pwd_len = cmd.length() + 1;
  if (pwd_len > MAX_PWD_LEN)
    pwd_len = MAX_PWD_LEN;
  memcpy(g_pwd, cmd.c_str(), pwd_len);
  NvTxPut(g_pwd);
  g_has_pwd = pwd_len > 1;
  g_auth = g_has_pwd;
  out.println();
}

static void cli_freq(String &cmd, Print &out)
{
  if (!cmd.length()) {
    out.println(g_freq);
    return;
  }
  uint32_t freq = cmd.toInt();
  if (freq < FMIN_MHZ * (uint32_t)g_fmul || FMAX_MHZ * (uint32_t)g_fmul < freq) {
    out.println(INVAL_RESP);
    return;
  }
  g_freq = freq;  
  if (g_out_on)
    adf_set_freq();
  display_freq();
  out.println();
}

static void cli_fmul(String &cmd, Print &out)
{
  if (!cmd.length()) {
    out.println(g_fmul);
    return;
  }
  uint8_t mul = cmd.toInt();
  if (!mul) {
    out.println(INVAL_RESP);
    return;
  }
  g_freq = (g_freq / g_fmul) * mul;
  g_fmul = mul;
  NvTxPut(g_freq);
  NvTxPut(g_fmul);
  if (g_out_on)
    adf_set_freq();
  display_freq();
  out.println();
}

static void cli_outp(String &cmd, Print &out)
{
  if (!cmd.length()) {
    out.println(g_out_on ? '1' : '0');
    return;
  }
  switch (cmd.charAt(0)) {
    case '0':
      if (g_out_on) {
        g_out_on = false;
        g_adf.vco_disable();
      }
      out.println();
      break;
    case '1':
      if (g_failed) {
        out.println(FAIL_RESP);
        break;
      }
      if (!g_out_on) {
        g_out_on = true;
        g_adf.vco_enable();
        adf_set_freq();
      }
      out.println();
      break;
    default:
      out.println(INVAL_RESP);
  }
}

static void cli_lock(String &cmd, Print &out)
{
  if (!cmd.length()) {
    out.println(g_remote ? '1' : '0');
    return;
  }
  switch (cmd.charAt(0)) {
    case '0':
      g_remote = false;
      out.println();
      break;
    case '1':
      g_remote = true;
      g_tune = false;
      display_freq();
      out.println();
      break;
    default:
      out.println(INVAL_RESP);
  }
}

static void cli_persist(Print &out)
{
  NvTxPut(g_freq);
  NvTxPut(g_out_on);
  out.println();
}

static bool cli_chk_access(Print &out)
{
  return !g_has_pwd || g_auth || &out == &Serial;
}

static void cli_process_cmd(String &cmd, Print &out)
{
  char tag = cmd.charAt(0);
  cmd.remove(0, 1);
  switch (tag) {
    case '?':
      out.println(F(HELP));
      return;
    case 'i':
      out.print(MODEL);
      out.print(' ');
      out.print(FMIN_MHZ);
      out.print(' ');
      out.println(FMAX_MHZ);
      return;
    case 'v':
      out.println(VERSION " " __DATE__);
      return;
    case 's':
      out.println(get_status_string());
      return;
    case 'a':
      if (g_eth_online)
        out.println(Ethernet.localIP());
      else
        out.println("0.0.0.0");
      return;
    case 'n': {
      char buff[HOSTNAME_LEN+1] = {};
      Ethernet.getHostname(buff);
      out.println(buff);
      return;
    }
    case '#':
      cli_pwd_auth(cmd, out);
      return;
    default:
      break;
  }
  if (!cli_chk_access(out)) {
    out.println(INVAL_RESP);
    return;    
  }
  switch (tag) {
    case 'f':
      cli_freq(cmd, out);
      break;
    case 'm':
      cli_fmul(cmd, out);
      break;
    case 'o':
      cli_outp(cmd, out);
      break;
    case 'l':
      cli_lock(cmd, out);
      break;
    case 'p':
      cli_persist(out);
      break;
    case 'P':
      cli_pwd_set(cmd, out);
      break;
    default:
      out.println(INVAL_RESP);
  }
}

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

  if (g_rx_buff.length() > MAX_CMD_LEN) {
    Serial.println(INVAL_RESP);
    goto done;
  }

  // trim white space characters
  g_rx_buff.trim();

  // process remaining symbols
  if (g_rx_buff.length())
    cli_process_cmd(g_rx_buff, Serial);

done:
  g_rx_buff = String();
}

static void telnet_process()
{
  g_telnet_srv.serve();

  if (!g_telnet_srv.has_line())
    return;

  String& cmd_buff = g_telnet_srv.get_buff();

  if (cmd_buff.length() > MAX_CMD_LEN) {
    g_telnet_srv.println(INVAL_RESP);
    goto done;
  }

  // trim white space characters
  cmd_buff.trim();

  // process remaining symbols
  if (cmd_buff.length())
    cli_process_cmd(cmd_buff, g_telnet_srv);

done:
  cmd_buff = String();
  g_telnet_srv.flush();
}

static void eth_process()
{
  bool online = (Ethernet.linkStatus() == LinkON);
  if (online != g_eth_online) {
    g_eth_online = online;
    if (online)
      telnet_init();
    else
      g_telnet_srv.reset();
    return;
  }
  if (g_eth_online)
    telnet_process();
}

void loop()
{
  serial_process();
  if (g_eth_present)
    eth_process();
  if (g_initialized) {
    check_events();
    adf_poll_status();
  }
  display_status();
  delay(IDLE_DELAY);
}
