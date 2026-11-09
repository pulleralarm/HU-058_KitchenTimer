#include "aip33628.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>

#ifdef USE_ESP32
#include <soc/gpio_reg.h>
#include <soc/soc.h>
#else // ESP8266
#include <Arduino.h>
#include <esp8266_peri.h>
#endif

namespace esphome {
namespace aip33628 {

static const char *const TAG = "aip33628";
static Aip33628Panel *scan_instance = nullptr;

// Each digit is one driver plus one pair of COM pairs, ten LED positions.
// Driver 1 carries the hours, driver 2 the minutes.
struct Block {
  uint8_t drv;
  uint8_t com_lo;
  uint8_t com_hi;
};
static const Block BLOCKS[4] = {
    {0, 0x30, 0xC0},  // hour tens,   annunciator AM
    {0, 0x03, 0x0C},  // hour ones,   annunciator colon
    {1, 0x30, 0xC0},  // minute tens, annunciator date dash
    {1, 0x03, 0x0C},  // minute ones, annunciator degree mark
};

// Segment position within a block. Side 0 is COM low, side 1 is COM high.
struct SegPos {
  uint8_t side;
  uint8_t led;
};
static const SegPos SEGMAP[9] = {
    {1, 5},  // A top
    {1, 4},  // B top right
    {1, 3},  // C bottom right
    {1, 2},  // D bottom
    {0, 5},  // E bottom left
    {0, 4},  // F top left
    {0, 3},  // G middle
    {1, 1},  // annunciator
    {0, 1},  // second annunciator, block 1 colon only
};
enum { SEG_ANNUN = 7, SEG_ANNUN2 = 8 };

// Physical position of every LED, extracted in docs/led-layout.md. The ids
// are the ones on the board layout map. Order matches SEGMAP.
const PosGeom GEOM[4][9] = {
    {  // block 0, hour tens
        { 1,  19,   8},  // A top
        { 7,  41,  66},  // B top right
        { 6,  40, 194},  // C bottom right
        { 5,  19, 255},  // D bottom
        { 4,   0, 188},  // E bottom left
        { 2,   0,  66},  // F top left
        { 3,  20, 130},  // G middle
        { 8,  20,  65},  // AM mark
        {0, 0, 0},  // unwired
    },
    {  // block 1, hour ones
        { 9,  93,   0},  // A top
        {15, 112,  61},  // B top right
        {14, 111, 198},  // C bottom right
        {13,  90, 246},  // D bottom
        {12,  72, 184},  // E bottom left
        {10,  73,  65},  // F top left
        {11,  92, 124},  // G middle
        {16, 128,  68},  // colon upper
        {18, 129, 189},  // colon lower
    },
    {  // block 2, minute tens
        {19, 163,   5},  // A top
        {25, 185,  67},  // B top right
        {24, 185, 195},  // C bottom right
        {23, 165, 247},  // D bottom
        {22, 145, 188},  // E bottom left
        {20, 143,  70},  // F top left
        {21, 164, 122},  // G middle
        {17, 128, 128},  // date dash
        {0, 0, 0},  // unwired
    },
    {  // block 3, minute ones
        {27, 233,  11},  // A top
        {33, 255,  60},  // B top right
        {32, 255, 190},  // C bottom right
        {31, 234, 249},  // D bottom
        {30, 216, 194},  // E bottom left
        {28, 216,  68},  // F top left
        {29, 235, 128},  // G middle
        {26, 204,  18},  // degree mark
        {0, 0, 0},  // unwired
    },
};

// Seven segment font, bit 0 = A through bit 6 = G. The letters are the subset
// that reads unambiguously on seven segments, so a caller can put a unit or a
// short label in the rightmost position.
static uint8_t glyph(char c) {
  switch (c) {
    case 'A': return 0b1110111;
    case 'b': return 0b1111100;
    case 'C': return 0b0111001;
    case 'c': return 0b1011000;
    case 'd': return 0b1011110;
    case 'E': return 0b1111001;
    case 'F': return 0b1110001;
    case 'G': return 0b0111101;
    case 'g': return 0b1101111;
    case 'H': return 0b1110110;
    case 'h': return 0b1110100;
    case 'I': return 0b0110000;  // left side, so it does not read as a 1
    case 'i': return 0b0010000;
    case 'J': return 0b0011110;
    case 'L': return 0b0111000;
    case 'l': return 0b0110000;
    case 'N': return 0b0110111;  // tall, full height both sides
    case 'n': return 0b1010100;
    case 'O': return 0b0111111;
    case 'o': return 0b1011100;
    case 'P': return 0b1110011;
    case 'q': return 0b1100111;
    case 'r': return 0b1010000;
    case 'S': return 0b1101101;
    case 't': return 0b1111000;
    case 'U': return 0b0111110;
    case 'u': return 0b0011100;
    case 'y': return 0b1101110;
    case '_': return 0b0001000;
    case '=': return 0b1001000;
    case '\'': return 0b0000010;
    case '"': return 0b0100010;
    case '0': return 0b0111111;
    case '1': return 0b0000110;
    case '2': return 0b1011011;
    case '3': return 0b1001111;
    case '4': return 0b1100110;
    case '5': return 0b1101101;
    case '6': return 0b1111101;
    case '7': return 0b0000111;
    case '8': return 0b1111111;
    case '9': return 0b1101111;
    case '-': return 0b1000000;
    case '%': return 0b1010010;
    default:  return 0;
  }
}

// Home Assistant sends gamma encoded values and the panel is linear in
// current and in duty, so both have to be linearized here. 2.8 is the same
// exponent ESPHome uses by default, which is why gamma_correct is set to 1.0
// on the light itself. Applying it in both places would square it.
static const float PANEL_GAMMA = 2.8f;

// Slider position to a current step. A plain gamma curve assumes the output
// can reach zero. This panel bottoms out at IS_MA[0], so a third of the
// slider ends up clamped against that floor with nothing to show for it.
// Interpolating perceived output between the floor and full instead puts all
// sixteen steps across the whole slider.
static uint8_t current_for(float brightness) {
  const float p_min = powf((float) IS_MA[0] / (float) IS_MA[15], 1.0f / PANEL_GAMMA);
  float p = p_min + clamp(brightness, 0.0f, 1.0f) * (1.0f - p_min);
  float want = powf(p, PANEL_GAMMA) * (float) IS_MA[15];
  uint8_t best = 0;
  for (uint8_t i = 1; i < 16; i++) {
    if (fabsf((float) IS_MA[i] - want) < fabsf((float) IS_MA[best] - want))
      best = i;
  }
  return best;
}

// Half brightness keeps the all-white hardware check brief and predictable.
static const float LAMP_BRIGHTNESS = 0.5f;

// Color component to a duty level. Duty is linear light, so the component
// has to be linearized before it is rounded, or every pastel rounds up to a
// saturated color. Pink is the clearest case, and rounds all the way to
// white.
static uint8_t duty_level(float c) {
  return (uint8_t) lroundf(powf(clamp(c, 0.0f, 1.0f), PANEL_GAMMA) * (COLOR_LEVELS - 1));
}

static int com_index(uint8_t cs) {
  for (int i = 0; i < 4; i++) {
    if (COM_SEQ[i] == cs) return i;
  }
  return 0;
}

void Aip33628Panel::setup() {
  for (auto *p : {clk_, data_, data2_}) {
    p->setup();
    p->digital_write(false);
  }
  clk_mask_ = 1u << clk_->get_pin();
  data_mask_ = 1u << data_->get_pin();
  if (clk2_) {
    clk2_->setup();
    clk2_->digital_write(false);
    clk_mask_ |= 1u << clk2_->get_pin();
  }
  data2_mask_ = 1u << data2_->get_pin();

  render_();

#ifdef USE_ESP32
  // A general purpose timer, not esp_timer. The esp_timer task dispatch path
  // runs at task priority on core 0 alongside the WiFi task, which preempts
  // it and stretches whichever COM slot happens to be lit. A 40us sub-frame
  // does not ride that out, so this runs from the interrupt instead.
  gptimer_config_t tcfg = {};
  tcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  tcfg.direction = GPTIMER_COUNT_UP;
  tcfg.resolution_hz = 1000000;  // one tick per microsecond

  gptimer_alarm_config_t acfg = {};
  acfg.alarm_count = UNIT_US;  // fixed, one tick per binary weight unit
  acfg.reload_count = 0;
  acfg.flags.auto_reload_on_alarm = true;

  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = &Aip33628Panel::scan_tick_;

  gptimer_handle_t timer = nullptr;
  if (gptimer_new_timer(&tcfg, &timer) != ESP_OK ||
      gptimer_register_event_callbacks(timer, &cbs, this) != ESP_OK ||
      gptimer_set_alarm_action(timer, &acfg) != ESP_OK ||
      gptimer_enable(timer) != ESP_OK || gptimer_start(timer) != ESP_OK) {
    ESP_LOGE(TAG, "could not start the scan timer");
    this->mark_failed();
  }
#else
  // ESP8266 Timer1 runs from the 80MHz peripheral clock. With TIM_DIV16 it
  // ticks at 5MHz, so one UNIT_US=40us scan tick is 200 timer ticks.
  
  scan_instance = this;
  timer1_disable();
  timer1_detachInterrupt();
  timer1_isr_init();
  timer1_attachInterrupt(&Aip33628Panel::scan_tick_);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(UNIT_US * 5);
#endif
}

void Aip33628Panel::dump_config() {
  ESP_LOGCONFIG(TAG, "AiP33628 panel:");
  LOG_PIN("  CLK: ", clk_);
  LOG_PIN("  DATA: ", data_);
  if (clk2_) LOG_PIN("  CLK_1: ", clk2_);
  LOG_PIN("  DATA_1: ", data2_);
  ESP_LOGCONFIG(TAG, "  Max current: IS 0x%X, %u.%umA per lit sink", max_current_,
                IS_MA[max_current_] / 10, IS_MA[max_current_] % 10);
  const char *fx = effect_ == Effect::CYCLE ? "color cycle"
                   : effect_ == Effect::FLASH ? "flash" : "none";
  const char *sp = spread_ == Spread::DIGIT ? "per digit"
                   : spread_ == Spread::LED ? "per LED" : "whole panel";
  // INFO rather than LOGCONFIG on purpose. The rest of this block is wiring
  // that cannot change, but the effect is live state worth being able to read
  // back, and CONFIG level messages need a DEBUG logger to be visible at all.
  ESP_LOGI(TAG, "  Effect: %s, %s, %.1fs, axis %.0f deg, hue span %.0f deg, flash fade %.2fs",
           fx, sp, effect_speed_, effect_angle_, hue_span_, flash_fade_);
}

// Emit one 30-bit frame to each driver and latch both. Bits are LSB first:
// SS[15:0], CS[7:0], IS[3:0], then two reserved zeros. Data only changes
// while CLK is low. The latch is a DATA rising edge while CLK is held high
// after the last bit, which is the sequence the stock 8051 produces.
//
// CS and IS are common to the two drivers and only SS differs, so one pass
// down the bits clocks both buses. That halves the work outright, and going
// straight to the GPOS/GPOC registers.
// The AiP33628 accepts 30MHz and asks for only 16ns of
// CLK high and low, so explicit delay padding is not required.

#ifdef USE_ESP32
#define PORT_SET(m) REG_WRITE(GPIO_OUT_W1TS_REG, (m))
#define PORT_CLR(m) REG_WRITE(GPIO_OUT_W1TC_REG, (m))
#else // ESP8266
#define PORT_SET(m) GPOS=(m)
#define PORT_CLR(m) GPOC=(m)
#endif

void IRAM_ATTR Aip33628Panel::send_pair_(uint16_t ss1, uint16_t ss2, uint8_t cs, uint8_t is) {
  const uint32_t wire = (uint32_t) IS_WIRE[is & 0xF] << 24;
  uint32_t f1 = (uint32_t) ss1 | ((uint32_t) cs << 16) | wire;
  uint32_t f2 = (uint32_t) ss2 | ((uint32_t) cs << 16) | wire;
  const uint32_t clks = clk_mask_;
  const uint32_t dats = data_mask_ | data2_mask_;

  PORT_CLR(clks | dats);

  for (int i = 0; i < 30; i++) {
    uint32_t set = 0;
    if (f1 & 1)
      set |= data_mask_;
    if (f2 & 1)
      set |= data2_mask_;
    f1 >>= 1;
    f2 >>= 1;
    // Data settles while CLK is low, then one rising edge shifts both buses.
    PORT_CLR(dats & ~set);
    PORT_SET(set);
    PORT_SET(clks);
    if (i < 29)
      PORT_CLR(clks);
  }

  // CLK is still high after bit 29. A DATA rising edge here is the latch.
  PORT_CLR(dats);
  PORT_SET(dats);
  PORT_CLR(dats);
  PORT_CLR(clks);
}

// Walk the schedule the renderer built. The timer runs at a fixed UNIT_US
// and this counts ticks, rather than reprogramming the alarm per step.
// Reprogramming would be fewer interrupts, but an alarm set shorter than the
// counter has already reached never matches, and a single late interrupt
// would then freeze the panel until reboot. A fixed auto-reload alarm cannot
// do that: a late interrupt costs one wobbly sub-frame and nothing more.
//
// Most ticks do nothing. A saturated color collapses to one step per COM
// pair, so fourteen of every fifteen calls are a decrement and a return.
#ifdef USE_ESP32
bool IRAM_ATTR Aip33628Panel::scan_tick_(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata, void *arg) {
  auto *self = static_cast<Aip33628Panel *>(arg);

  if (self->wait_ > 0) {
    self->wait_--;
    return false;
  }
#else
void IRAM_ATTR Aip33628Panel::scan_tick_() {
  auto *self = scan_instance;
  if (self == nullptr)
    return;

  if (self->wait_ > 0) {
    self->wait_--;
    return;  //Void function
  }
#endif

  const ScanBuf &b = self->buf_[self->front_];

  // The renderer can flip the buffer between two steps, and the new schedule
  // may be shorter than the old one, so the index is clamped rather than
  // trusted. Worst case is one odd frame while a slider is moving.
  uint8_t i = self->step_;
  if (i >= b.n)
    i = 0;
  const ScanStep &st = b.step[i];

  self->send_pair_(st.ss[0], st.ss[1], st.cs, b.is);

  self->wait_ = (uint8_t) (st.units - 1);  // this tick is the first of the step
  uint8_t next = (uint8_t) (i + 1);
  self->step_ = next >= b.n ? 0 : next;
#ifdef USE_ESP32
  return false;
#endif
}

void Aip33628Panel::write_pos_(uint8_t block, uint8_t seg, bool on) {
  on_[block][seg] = on;
}

void Aip33628Panel::write_digit_(uint8_t block, char c) {
  uint8_t bits = glyph(c);
  for (int s = 0; s < 7; s++) {
    on_[block][s] = (bits >> s) & 1;
  }
}

// brightness is linear and carries the transition state, so it falls to zero
// on its own during a fade to off. The color components are the normalized
// ratio and do not scale with it. ESPHome guarantees the largest of the three
// is 1, so at least one channel always survives duty_level and a color can
// never round away to nothing.
void Aip33628Panel::set_light(bool on, float r, float g, float b, float brightness) {
  enabled_ = on && brightness > 0.0f;

  // Picking a new color on the master light means the whole panel, so it
  // drops the digit and position tiers. Moving only the brightness slider
  // leaves them alone, which matters because a transition calls this on
  // every step and would otherwise wipe a gradient mid fade.
  bool color_moved = fabsf(r - base_rgb_[0]) > 0.002f || fabsf(g - base_rgb_[1]) > 0.002f ||
                     fabsf(b - base_rgb_[2]) > 0.002f;
  if (color_moved && effect_ == Effect::NONE) {
    for (bool &v : digit_set_)
      v = false;
    clear_positions_();
  }

  base_rgb_[0] = r;
  base_rgb_[1] = g;
  base_rgb_[2] = b;
  requested_current_ = current_for(brightness);
  apply_colors_();
}

// Quantize whatever color each block is currently supposed to be. Everything
// that changes a color goes through here, so there is one place that decides
// what a block ends up at and one place that marks the panel dirty.
void Aip33628Panel::apply_colors_() {
  for (int blk = 0; blk < 4; blk++) {
    for (int seg = 0; seg < 9; seg++) {
      const float *c = pos_set_[blk][seg]  ? pos_rgb_[blk][seg]
                       : digit_set_[blk]   ? digit_rgb_[blk]
                                           : base_rgb_;
      if (enabled_) { // FIXME if nach aussen
        level_[blk][seg][CH_RED] = duty_level(c[0] * envelope_);
        level_[blk][seg][CH_GREEN] = duty_level(c[1] * envelope_);
        level_[blk][seg][CH_BLUE] = duty_level(c[2] * envelope_);
      } else {
        level_[blk][seg][CH_RED] = 0;
        level_[blk][seg][CH_GREEN] = 0;
        level_[blk][seg][CH_BLUE] = 0;
      }
    }
  }
  dirty_ = true;
}

// A digit of -1 sets all four at once, which is what a whole display effect
// wants. Components are taken as given and not normalized: the master light
// arrives already normalized with its magnitude in the current setting, but a
// caller here is asking for one digit to look a particular way next to the
// others, and scaling that back up would throw away the difference.
void Aip33628Panel::set_digit_color(int digit, float r, float g, float b) {
  if (digit < -1 || digit > 3)
    return;
  for (int blk = 0; blk < 4; blk++) {
    if (digit != -1 && digit != blk)
      continue;
    digit_rgb_[blk][0] = clamp(r, 0.0f, 1.0f);
    digit_rgb_[blk][1] = clamp(g, 0.0f, 1.0f);
    digit_rgb_[blk][2] = clamp(b, 0.0f, 1.0f);
    digit_set_[blk] = true;
    // Setting a whole digit drops any per position color inside it. Without
    // this a gradient would sit on top and the digit color would do nothing
    // visible, which reads as the call being ignored.
    for (bool &v : pos_set_[blk])
      v = false;
  }
  apply_colors_();
}

// Hand the whole panel back to the master light, per position overrides
// included. Anything else would leave a gradient stuck on with no obvious way
// to clear it.
void Aip33628Panel::clear_digit_colors() {
  for (bool &v : digit_set_)
    v = false;
  clear_positions_();
  apply_colors_();
}

void Aip33628Panel::clear_positions_() {
  for (auto &blk : pos_set_)
    for (bool &v : blk)
      v = false;
}

// One LED, addressed by the id on the board layout map rather than by block
// and segment, so the numbering here is the same one written on the map.
void Aip33628Panel::set_position_color(int id, float r, float g, float b) {
  for (int blk = 0; blk < 4; blk++) {
    for (int seg = 0; seg < 9; seg++) {
      if (GEOM[blk][seg].id != id)
        continue;
      pos_rgb_[blk][seg][0] = clamp(r, 0.0f, 1.0f);
      pos_rgb_[blk][seg][1] = clamp(g, 0.0f, 1.0f);
      pos_rgb_[blk][seg][2] = clamp(b, 0.0f, 1.0f);
      pos_set_[blk][seg] = true;
      apply_colors_();
      return;
    }
  }
}

// A linear ramp across the panel between two colors. Angle is in degrees, 0
// running left to right and 90 top to bottom, so -45 runs from the bottom
// left corner to the top right.
//
// The ramp is normalized against the LEDs themselves rather than the panel
// outline, so the two colors asked for land exactly on the outermost LEDs
// whichever way the ramp points. Normalizing against the corners instead
// leaves both ends short, because no LED sits in a corner.
void Aip33628Panel::set_gradient(float r0, float g0, float b0, float r1, float g1, float b1,
                                 float angle_deg) {
  const float a = angle_deg * 3.14159265f / 180.0f;
  const float ca = cosf(a), sa = sinf(a);

  float lo = 1e9f, hi = -1e9f;
  for (int blk = 0; blk < 4; blk++) {
    for (int seg = 0; seg < 9; seg++) {
      const PosGeom &g = GEOM[blk][seg];
      if (g.id == 0)
        continue;
      float t = (g.nx / 255.0f) * ca + (g.ny / 255.0f) * sa;
      if (t < lo)
        lo = t;
      if (t > hi)
        hi = t;
    }
  }
  float span = hi - lo;
  if (span < 1e-6f)
    span = 1.0f;

  for (int blk = 0; blk < 4; blk++) {
    for (int seg = 0; seg < 9; seg++) {
      const PosGeom &g = GEOM[blk][seg];
      if (g.id == 0)
        continue;  // nothing wired here
      float t = ((g.nx / 255.0f) * ca + (g.ny / 255.0f) * sa - lo) / span;
      t = clamp(t, 0.0f, 1.0f);
      pos_rgb_[blk][seg][0] = clamp(r0 + (r1 - r0) * t, 0.0f, 1.0f);
      pos_rgb_[blk][seg][1] = clamp(g0 + (g1 - g0) * t, 0.0f, 1.0f);
      pos_rgb_[blk][seg][2] = clamp(b0 + (b1 - b0) * t, 0.0f, 1.0f);
      pos_set_[blk][seg] = true;
    }
  }
  apply_colors_();
}

void Aip33628Panel::show_text(const char *value, Annun annunciators) {
  if (strlen(value) < 4) return;

  if (!memcmp(value,text_,4) && annunciators == annunciators_) return;

  annunciators_ = annunciators;
  memcpy(text_,value,4);

  for (auto &blk : on_) {
    for (bool &v : blk) v = false;
  }

  for (uint8_t n = 0; n<4; ++n) write_digit_(n,text_[n]);
  if (annunciators_ & Annun::AM) write_pos_(0, SEG_ANNUN, true);
  if (annunciators_ & Annun::COL_HIGH) write_pos_(1, SEG_ANNUN, true);
  if (annunciators_ & Annun::COL_LOW) write_pos_(1, SEG_ANNUN2, true);
  if (annunciators_ & Annun::DASH) write_pos_(2, SEG_ANNUN, true);
  if (annunciators_ & Annun::DEGREE) write_pos_(3, SEG_ANNUN, true);

  dirty_ = true;
}

void Aip33628Panel::show_clock_once(time::RealTimeClock *rtc, Annun annunciators, bool twelve_hour, bool online) {
  ESPTime now{};
  char rtext[4];
  bool colon = (millis() / 1000) % 2 == 0;

  if (rtc) now = rtc->now();
  if (!now.is_valid()) {
    show_text("----",annunciators);
    return;
  }

  int hour = now.hour;
  bool pm = hour >= 12;
  if (twelve_hour) {
   hour = hour % 12;
   if (hour == 0) hour = 12;
  }

  // Leading zero stays suppressed in 24 hour mode as well.
  if (hour >= 10) {
    rtext[0] = (char) ('0' + hour / 10);
  } else {
    rtext[0] = ' ';
  }
  rtext[1] = (char) ('0' + hour % 10);
  rtext[2] = (char) ('0' + now.minute / 10);
  rtext[3] = (char) ('0' + now.minute % 10);

  if (twelve_hour && !pm) annunciators = annunciators | Annun::AM;
  if (colon) {
    annunciators = annunciators | Annun::COL_LOW;
    if (online) annunciators = annunciators | Annun::COL_HIGH;
  }
  show_text(rtext,annunciators);
}

void Aip33628Panel::show_temp_once(float temp, const unsigned char unit, Annun annunciators) {
  char tempstr[16];
  sprintf(tempstr, "%3d%c", static_cast<int>(10*temp),unit);
  show_text(tempstr, annunciators | Annun::COL_LOW | Annun::DEGREE);
}

void Aip33628Panel::show_hum_once(float hum, Annun annunciators) {
  char tempstr[16];
  sprintf(tempstr, "%3d%%", static_cast<int>(10*hum));
  show_text(tempstr, annunciators | Annun::COL_LOW);
}


// Every populated position, white, at a fixed brightness, for a few seconds.
// This is a hardware check, so user color and brightness settings do not
// change the result.
void Aip33628Panel::lamp_test() {
  for (auto &blk : on_) {
    for (bool &v : blk) v = true;
  }
  this->dirty_ = true;
}

// Right aligned, no colon. A unit takes the rightmost position and leaves
// three for the number, so 78F and -5C both fit. Without one the number gets
// all four. Out of range values are clamped rather than wrapped, because a
// wrapped temperature is a wrong reading and a clamped one is obviously
// pinned against the end.
//
// The widest values reach the hour tens position. With a unit that only
// happens at three digits or a signed two, and never for a temperature in F.
void Aip33628Panel::draw_number_(int value, char unit) {
  int pos = 3;
  if (unit != '\0' && glyph(unit) != 0) {
    write_digit_(3, unit);
    pos = 2;
    // C and F are temperatures, so light the degree mark ahead of the unit.
    if (unit == 'C' || unit == 'c' || unit == 'F')
      write_pos_(3, SEG_ANNUN, true);
  }

  bool neg = value < 0;
  if (neg) value = -value;

  int room = pos + 1 - (neg ? 1 : 0);  // positions left for digits
  int limit = 1;
  for (int i = 0; i < room; i++) limit *= 10;
  if (value > limit - 1) value = limit - 1;

  do {
    write_digit_(pos--, (char) ('0' + value % 10));
    value /= 10;
  } while (value > 0 && pos >= 0);
  if (neg && pos >= 0) write_digit_(pos, '-');
}

// Full saturation hue to RGB. Effects ride the color wheel rather than the
// master light's color, because a rainbow that keeps the user's tint is not
// a rainbow.
static void hue_rgb(float h, float *out) {
  h -= floorf(h);
  float x = h * 6.0f;
  int i = (int) x;
  float f = x - (float) i;
  switch (i % 6) {
    case 0:  out[0] = 1.0f;    out[1] = f;       out[2] = 0.0f;    break;
    case 1:  out[0] = 1.0f - f; out[1] = 1.0f;   out[2] = 0.0f;    break;
    case 2:  out[0] = 0.0f;    out[1] = 1.0f;    out[2] = f;       break;
    case 3:  out[0] = 0.0f;    out[1] = 1.0f - f; out[2] = 1.0f;   break;
    case 4:  out[0] = f;       out[1] = 0.0f;    out[2] = 1.0f;    break;
    default: out[0] = 1.0f;    out[1] = 0.0f;    out[2] = 1.0f - f; break;
  }
}

// Where every position sits along the effect axis, 0 at the trailing edge
// and 1 at the leading one. Same projection the gradient uses, normalized
// against the LEDs rather than the panel outline for the same reason. Only
// recomputed when the angle changes.
void Aip33628Panel::recompute_axis_() {
  const float a = effect_angle_ * 3.14159265f / 180.0f;
  const float ca = cosf(a), sa = sinf(a);

  float lo = 1e9f, hi = -1e9f;
  for (int blk = 0; blk < 4; blk++) {
    for (int seg = 0; seg < 9; seg++) {
      if (GEOM[blk][seg].id == 0)
        continue;
      float t = (GEOM[blk][seg].nx / 255.0f) * ca + (GEOM[blk][seg].ny / 255.0f) * sa;
      if (t < lo) lo = t;
      if (t > hi) hi = t;
    }
  }
  float span = hi - lo;
  if (span < 1e-6f)
    span = 1.0f;

  float blo = 1e9f, bhi = -1e9f;
  for (int blk = 0; blk < 4; blk++) {
    float sum = 0.0f;
    int n = 0;
    for (int seg = 0; seg < 9; seg++) {
      if (GEOM[blk][seg].id == 0)
        continue;
      float t = ((GEOM[blk][seg].nx / 255.0f) * ca + (GEOM[blk][seg].ny / 255.0f) * sa - lo) / span;
      axis_pos_[blk][seg] = t;
      sum += t;
      n++;
    }
    axis_blk_[blk] = n ? sum / (float) n : 0.0f;
    if (axis_blk_[blk] < blo) blo = axis_blk_[blk];
    if (axis_blk_[blk] > bhi) bhi = axis_blk_[blk];
  }

  // Normalized end to end, same as the per position axis. How much of the
  // wheel that covers is the hue span setting's job, not this one's.
  float bspan = bhi - blo;
  if (bspan < 1e-6f)
    bspan = 1.0f;
  for (int blk = 0; blk < 4; blk++)
    axis_blk_[blk] = (axis_blk_[blk] - blo) / bspan;
}

void Aip33628Panel::set_effect(int mode) {
  Effect want = mode == 1 ? Effect::CYCLE : mode == 2 ? Effect::FLASH : Effect::NONE;
  if (want == effect_)
    return;
  effect_ = want;
  effect_t0_ = millis();
  effect_at_ = 0;
  // Leaving an effect hands the panel back rather than freezing on whatever
  // frame it happened to stop at.
  envelope_ = 1.0f;
  if (want == Effect::NONE)
    clear_positions_();
  if (want == Effect::CYCLE)
    recompute_axis_();
  apply_colors_();
}

void Aip33628Panel::set_effect_speed(float seconds) {
  effect_speed_ = seconds < 0.1f ? 0.1f : (seconds > 600.0f ? 600.0f : seconds);
}

void Aip33628Panel::set_effect_spread(int mode) {
  spread_ = mode == 1 ? Spread::DIGIT : mode == 2 ? Spread::LED : Spread::PANEL;
}

void Aip33628Panel::set_effect_angle(float deg) {
  effect_angle_ = deg;
  recompute_axis_();
}

void Aip33628Panel::set_effect_hue_span(float deg) {
  hue_span_ = deg < 0.0f ? 0.0f : (deg > 360.0f ? 360.0f : deg);
}

void Aip33628Panel::set_flash_fade(float seconds) {
  flash_fade_ = seconds < 0.0f ? 0.0f : (seconds > 300.0f ? 300.0f : seconds);
}

// Advance whichever effect is running. Called from loop() at a fixed cadence
// rather than every pass, since the scan is what the eye sees and a redraw
// faster than about 25Hz buys nothing.
void Aip33628Panel::update_effect_(uint32_t now_ms) {
  float period = effect_speed_;
  float phase = fmodf((float) (now_ms - effect_t0_) / 1000.0f / period, 1.0f);

  if (effect_ == Effect::FLASH) {
    // A trapezoid. Rate and transition time are separate, so 1Hz with a 100ms
    // ramp and 1Hz snapping hard are both reachable. The on and off halves
    // stay even and the ramps eat into them rather than stretching the
    // period, so changing the fade never changes the flash rate.
    const float half = period * 0.5f;
    const float f = flash_fade_ > half ? half : flash_fade_;
    const float t = phase * period;
    float e;
    if (f <= 0.0f) {
      e = t < half ? 1.0f : 0.0f;  // square
    } else if (t < f) {
      e = t / f;
    } else if (t < half) {
      e = 1.0f;
    } else if (t < half + f) {
      e = 1.0f - (t - half) / f;
    } else {
      e = 0.0f;
    }
    envelope_ = e;
    apply_colors_();
    return;
  }

  // How much of the wheel the panel covers end to end. A full turn across
  // four digits packs the whole spectrum into a hand span and reads as noise.
  const float span = hue_span_ / 360.0f;

  for (int blk = 0; blk < 4; blk++) {
    for (int seg = 0; seg < 9; seg++) {
      if (GEOM[blk][seg].id == 0)
        continue;
      float t = spread_ == Spread::PANEL  ? 0.0f
                : spread_ == Spread::DIGIT ? axis_blk_[blk]
                                           : axis_pos_[blk][seg];
      hue_rgb(phase + t * span, pos_rgb_[blk][seg]);
      pos_set_[blk][seg] = true;
    }
  }
  apply_colors_();
}

void Aip33628Panel::loop() {
  uint32_t now_ms = millis();

  if (effect_ != Effect::NONE && (int32_t) (now_ms - effect_at_) >= 0) {
    effect_at_ = now_ms + 40;  // 25Hz, smooth enough for a fade to glide
    update_effect_(now_ms);
  }

  if (!dirty_) return; // FIXME !enabled_
  dirty_ = false;

  render_();
}

// Collapse the panel state into a scan schedule. Each COM pair is split into
// COLOR_BITS binary weighted sub-frames, and a channel at duty level L is lit
// in sub-frame k whenever bit k of L is set. Any level from 0 to
// COLOR_LEVELS - 1 is reachable that way, with no constraint that a dimmer
// channel be a subset of a brighter one.
//
// Per digit color costs nothing here. Each driver and COM pair together
// belong to exactly one block, since the two blocks on a driver sit on
// different COM pairs, so a slot only ever holds one block's color per
// driver and the two drivers carry their own SS word anyway. Four different
// colors reach sixteen steps, which is exactly MAX_STEPS.
void Aip33628Panel::render_() {
  uint16_t sub[2][4][COLOR_BITS] = {};

  for (int blk = 0; blk < 4; blk++) {
    const Block &b = BLOCKS[blk];
    for (int seg = 0; seg < 9; seg++) {
      if (!on_[blk][seg])
        continue;
      const SegPos &sp = SEGMAP[seg];
      int ci = com_index(sp.side ? b.com_hi : b.com_lo);
      int base = 3 * sp.led - 2;  // LED1 is SEG1..SEG3, LED5 is SEG13..SEG15
      for (int ch = 0; ch < 3; ch++) {
        uint8_t lv = level_[blk][seg][ch];
        if (lv == 0)
          continue;
        uint16_t bit = (uint16_t) (1u << (base + ch));
        for (int k = 0; k < COLOR_BITS; k++) {
          if (lv & (1u << k))
            sub[b.drv][ci][k] |= bit;
        }
      }
    }
  }

  // IS is set by the brightness alone and never by what is on screen. Making
  // it depend on the lit sink count changes the brightness of the whole panel
  // every time the colon blinks. The stock firmware held IS fixed across
  // colon on and colon off, and ran 0xF with a white digit lit.
  // The lamp test brings its own current, while max_current_ remains the
  // thermal ceiling for every mode.
  uint8_t want = requested_current_;
  uint8_t is = want < max_current_ ? want : max_current_;

  ScanBuf &b = buf_[front_ ^ 1];
  b.n = 0;
  for (int ci = 0; ci < 4; ci++) {
    for (int k = 0; k < COLOR_BITS; k++) {
      uint16_t s0 = sub[0][ci][k];
      uint16_t s1 = sub[1][ci][k];
      uint8_t units = (uint8_t) (1u << k);
      // Identical neighbors inside one COM pair merge, so a saturated color,
      // where every sub-frame carries the same data, ends up as one step of
      // the full 600us. Merging never crosses a COM boundary, because CS
      // changes there and the drivers have to be re-sent regardless.
      if (k > 0 && b.n > 0 && b.step[b.n - 1].ss[0] == s0 && b.step[b.n - 1].ss[1] == s1) {
        b.step[b.n - 1].units = (uint8_t) (b.step[b.n - 1].units + units);
        continue;
      }
      ScanStep &st = b.step[b.n++];
      st.units = units;
      st.ss[0] = s0;
      st.ss[1] = s1;
      st.cs = COM_SEQ[ci];
    }
  }
  b.is = is;

  // Publish. The barrier keeps the writes above from being reordered past the
  // flip, which is what stops the scan callback seeing a torn pattern while a
  // Home Assistant slider is being dragged.
  __sync_synchronize();
  front_ ^= 1;
}

}  // namespace aip33628
}  // namespace esphome
