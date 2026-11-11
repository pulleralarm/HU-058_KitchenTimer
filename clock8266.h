
enum AipState { CLOCK, SECONDS, TEMP, HUM, CD1, CD2, CD3, EXTVAL };

AipState disp_current{CLOCK}, disp_next{CLOCK};

enum WFState { AUTO, CDDISP, CDSET };

uint32_t wf_autonext; // millis
WFState wf_current(AUTO);
uint16_t wf_countdown; // aktueller Countdown

class Countdown {
  int32_t init_millis_;
  int32_t rest_millis_;
  uint32_t check_millis_;
  bool running_;
  bool buzzing_;

  void update();

 public:
  Countdown(): init_millis_(0), rest_millis_(0), running_(false) {}

  void start();
  void stop();
  void reset();
  void set(int32_t duration) { init_millis_ = rest_millis_ = 1000 * duration; }
  void modify(int32_t seconds); // Update per Tasten
  bool is_running() const { return running_; }
  bool is_alarm();
  bool is_buzzing() const { return buzzing_; }
  void set_buzzing() { buzzing_ = true; }
  int32_t restms() const { return rest_millis_; }
  std::string reststr_ha() const;
  std::string reststr_gui() const;
};

void Countdown::update() {
 if (!running_) return;
 uint32_t m = millis();
 if (m > check_millis_) {
  rest_millis_ -= m - check_millis_;
  check_millis_ = m;
 }
}

void Countdown::start() {
 if (running_) return;
 running_ = true;
 buzzing_ = false;
 check_millis_ = millis();
}

void Countdown::stop() {
 update();
 running_ = false;
 buzzing_ = false;
}

void Countdown::reset() {
 running_ = false;
 buzzing_ = false;
 rest_millis_ = init_millis_;
}

void Countdown::modify(int32_t seconds) {
 if (running_) return;
 rest_millis_ += 1000 * seconds;
 if (rest_millis_ < 0) rest_millis_ = 0;
 if (rest_millis_ > 86400000) rest_millis_ = 86400000;
 init_millis_ = rest_millis_; // Naechter Reset soll auch mit diesem wert starten
}

bool Countdown::is_alarm() {
 if (!running_) return false;
 update();
 return rest_millis_ <= 0;
}
 
std::string Countdown::reststr_ha() const {
 int32_t rest = rest_millis_ / 1000;
 if (rest <= 0) return "00:00:00";
 if (rest > 600) rest -= rest % 60; // >10min: Aufloesung 1min
 else if (rest > 60) rest -= rest % 10; // 1-10min: Aufloesung 10sek
 return str_sprintf("%02" PRIu32 ":%02" PRIu32 ":%02" PRIu32, rest / 3600, rest % 3600 / 60, rest % 60);
}

std::string Countdown::reststr_gui() const {
 int32_t rest = rest_millis_ / 1000;
 if (rest <= 0) return "0000";
 if (rest < 6000) { // 100min
  return str_sprintf("%2" PRIu32 "%02" PRIu32 , rest / 60, rest % 60);
 } // Sonst hh:mm
 rest /= 60;
 return str_sprintf("%02" PRIu32 "%02" PRIu32, rest / 60, rest % 60);
}

Countdown countdowns[3];

uint32_t stringtodelay(std::string s) {
 uint32_t res = 0;
 uint32_t res2;
 s = s.erase(0,s.find_first_not_of(" "));

 std::size_t col;
 col = s.find(':');
 if (col == std::string::npos) { // Als Minuten interpretieren
  res = 60 * atoi(s.c_str());
  goto done;
 }
 res = atoi(s.c_str());
 s = s.substr(col+1); // Nicht sehr effizient aber hier egal
 res2 = atoi(s.c_str());
 col = s.find(':');
 if (col == std::string::npos) { // Als hh:mm interetieren
  res = 3600 * res + 60 * res2;
 } else { // als hh:mm:ss interpretieren
  s = s.substr(col+1);
  uint32_t res3 = atoi(s.c_str());
  res = 3600 * res + 60 * res2 + res3;
 }

done: 
 if (res > 86400) res = 86400;
 return res;
}


std::string delaytostring(uint32_t d) {
 return str_sprintf("%2" PRIu32 ":%02" PRIu32 ":%02" PRIu32, d / 3600, d % 3600 / 60, d % 60);
} 
