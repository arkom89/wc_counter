#include <WiFi.h>
#include <Preferences.h>
#include <FastBot.h>

// ====== ВПИШИ СЮДА ======
static const char* WIFI_SSID = "<SSID>";
static const char* WIFI_PASS = "<PASS>";

#define BOT_TOKEN "<TOKEN>"
static const char* ADMIN_CHAT_ID = "<CHAT_ID>"; // свой chat id строкой
// =========================

// Пины
static const int PIN_COLD = 18;
static const int PIN_HOT  = 23;

// 1 импульс = 10 литров = 0.01 м³
static const uint32_t LITERS_PER_PULSE = 10;

// Фильтр сигнала
static const uint32_t LOW_CONFIRM_MS = 25;   // LOW должен держаться >= 25мс, чтобы засчитать тик
static const uint32_t HIGH_REARM_MS  = 200;  // HIGH должен держаться >= 200мс, чтобы снова разрешить тик

// "Антидребезг" как минимальный интервал между принятыми тиками (в мс)
static const uint32_t DEFAULT_DEBOUNCE_MS = 2000; // для 10л/имп нормально 2000..5000

// Сохранение в NVS
static const uint32_t SAVE_EVERY_PULSES = 3;
static const uint32_t SAVE_EVERY_MS     = 60 * 1000;

FastBot bot(BOT_TOKEN);
Preferences prefs;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// сырые тики
volatile uint64_t cold_pulses = 0;
volatile uint64_t hot_pulses  = 0;

// gate interval
uint32_t debounce_ms = DEFAULT_DEBOUNCE_MS;

// pending из ISR
volatile bool cold_pending = false;
volatile bool hot_pending  = false;
volatile uint32_t cold_pending_at = 0;
volatile uint32_t hot_pending_at  = 0;

// re-arm
volatile bool cold_armed = true;
volatile bool hot_armed  = true;
uint32_t cold_high_since = 0;
uint32_t hot_high_since  = 0;

// последнее принятие
uint32_t cold_last_accept = 0;
uint32_t hot_last_accept  = 0;

// диагностика
volatile uint32_t cold_irq = 0;
volatile uint32_t hot_irq  = 0;

// save bookkeeping
uint64_t last_saved_cold = 0;
uint64_t last_saved_hot  = 0;
uint32_t last_save_ms = 0;

static inline uint32_t nowMs() { return (uint32_t)(micros() / 1000ULL); }

// ====== ISR: только отметить кандидата ======
void IRAM_ATTR isrCold() {
  cold_irq++;
  uint32_t t = nowMs();
  portENTER_CRITICAL_ISR(&mux);
  if (cold_armed && !cold_pending) {
    cold_pending = true;
    cold_pending_at = t;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrHot() {
  hot_irq++;
  uint32_t t = nowMs();
  portENTER_CRITICAL_ISR(&mux);
  if (hot_armed && !hot_pending) {
    hot_pending = true;
    hot_pending_at = t;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// ====== Форматирование показаний в м³ с 3 знаками (как на счётчике) ======
String pulsesToM3String3(uint64_t pulses) {
  // 10 л = 0.01 м³, значит 1 тик = 0.010 м³
  uint64_t liters = pulses * (uint64_t)LITERS_PER_PULSE; // целые литры
  uint64_t m3_int = liters / 1000ULL;
  uint32_t frac3  = (uint32_t)(liters % 1000ULL); // литры как тысячные м³
  char buf[32];
  snprintf(buf, sizeof(buf), "%llu.%03u",
           (unsigned long long)m3_int, (unsigned)frac3);
  return String(buf);
}

// ====== /set: только целое число "сотые м³" ======
// пример: "5464" => 54.64 м³ (последняя цифра литров игнорируется)
uint64_t parseCentiM3ToPulses(const String& s_in) {
  String s = s_in;
  s.trim();
  if (s.length() == 0) return UINT64_MAX;

  for (size_t i = 0; i < s.length(); i++) {
    if (s[i] < '0' || s[i] > '9') return UINT64_MAX;
  }

  // 0.01 м³ = 10 л = 1 тик
  uint64_t centi_m3 = (uint64_t)s.toInt();
  return centi_m3;
}

// ====== NVS ======
void loadState() {
  prefs.begin("water", true);
  cold_pulses = prefs.getULong64("cold", 0);
  hot_pulses  = prefs.getULong64("hot",  0);
  debounce_ms = prefs.getUInt("deb", DEFAULT_DEBOUNCE_MS);
  prefs.end();

  last_saved_cold = cold_pulses;
  last_saved_hot  = hot_pulses;
}

void saveState(bool force) {
  uint32_t now = millis();
  if (!force && (now - last_save_ms) < SAVE_EVERY_MS) return;

  uint64_t c, h;
  portENTER_CRITICAL(&mux);
  c = cold_pulses;
  h = hot_pulses;
  portEXIT_CRITICAL(&mux);

  prefs.begin("water", false);
  prefs.putULong64("cold", c);
  prefs.putULong64("hot",  h);
  prefs.putUInt("deb", debounce_ms);
  prefs.end();

  last_save_ms = now;
  last_saved_cold = c;
  last_saved_hot  = h;
}

// ====== Фильтр одного канала ======
void processChannel(
  int pin,
  volatile bool &pending,
  volatile uint32_t &pending_at,
  volatile bool &armed,
  uint32_t &high_since,
  uint32_t &last_accept,
  volatile uint64_t &pulses
) {
  uint32_t t = nowMs();

  // подтвердить импульс: LOW держится >= LOW_CONFIRM_MS
  if (pending) {
    if (digitalRead(pin) == LOW) {
      // (1) подтверждение LOW
      // (2) gate по времени: не чаще debounce_ms
      if ((uint32_t)(t - pending_at) >= LOW_CONFIRM_MS &&
          (uint32_t)(t - last_accept) >= debounce_ms) {

        portENTER_CRITICAL(&mux);
        pulses++;
        pending = false;
        armed = false; // блокируем до устойчивого HIGH
        portEXIT_CRITICAL(&mux);

        last_accept = t;
      }
    } else {
      // вернулось в HIGH раньше подтверждения => мусор
      pending = false;
    }
  }

  // re-arm после стабильного HIGH
  if (!armed) {
    if (digitalRead(pin) == HIGH) {
      if (high_since == 0) high_since = t;
      else if ((uint32_t)(t - high_since) >= HIGH_REARM_MS) {
        portENTER_CRITICAL(&mux);
        armed = true;
        portEXIT_CRITICAL(&mux);
        high_since = 0;
      }
    } else {
      high_since = 0;
    }
  }
}

// ====== BOT ======
bool isAdmin(const FB_msg& msg) {
  return String(msg.chatID) == String(ADMIN_CHAT_ID);
}

String statusText() {
  uint64_t c, h;
  uint32_t ci, hi;
  portENTER_CRITICAL(&mux);
  c = cold_pulses;
  h = hot_pulses;
  ci = cold_irq;
  hi = hot_irq;
  portEXIT_CRITICAL(&mux);

  String s;
  s.reserve(700);
  s += "Показания (м³):\n";
  s += "Холодная: " + pulsesToM3String3(c) + "\n";
  s += "Горячая : " + pulsesToM3String3(h) + "\n";
  s += "\nPulses: cold=" + String((unsigned long long)c) + " hot=" + String((unsigned long long)h) + "\n";
  s += "deb(ms)=" + String(debounce_ms) + "  LOW>=" + String(LOW_CONFIRM_MS) + "ms  HIGH>=" + String(HIGH_REARM_MS) + "ms\n";
  s += "PIN: cold=" + String(digitalRead(PIN_COLD)) + " hot=" + String(digitalRead(PIN_HOT)) + "\n";
  s += "IRQ: cold=" + String(ci) + " hot=" + String(hi) + "\n";
  return s;
}

void newMsg(FB_msg& msg) {
  if (!isAdmin(msg)) return;

  String t = msg.text;
  t.trim();
  String tl = t; tl.toLowerCase();

  if (tl == "/get" || tl == "get" || tl == "/status" || tl == "status") {
    bot.sendMessage(statusText());
    return;
  }

  if (tl == "/diag" || tl == "diag") {
    bot.sendMessage(statusText());
    return;
  }

  if (tl.startsWith("/deb")) {
    int sp = tl.indexOf(' ');
    if (sp < 0) { bot.sendMessage("Пример: /deb 3000"); return; }
    uint32_t v = (uint32_t)tl.substring(sp + 1).toInt();
    if (v < 50 || v > 20000) { bot.sendMessage("Диапазон: 50..20000"); return; }
    debounce_ms = v;
    saveState(true);
    bot.sendMessage("OK deb(ms)=" + String(debounce_ms));
    return;
  }

  // /set cold 5464  (54.64 м³, только целые)
  if (tl.startsWith("/set")) {
    int p1 = tl.indexOf(' ');
    if (p1 < 0) { bot.sendMessage("Пример: /set cold 5464"); return; }
    int p2 = tl.indexOf(' ', p1 + 1);
    if (p2 < 0) { bot.sendMessage("Пример: /set cold 5464"); return; }

    String who = tl.substring(p1 + 1, p2);
    String val = t.substring(p2 + 1);

    uint64_t pulses = parseCentiM3ToPulses(val);
    if (pulses == UINT64_MAX) {
      bot.sendMessage("Ошибка. Вводи только целое число без точки.\n"
                      "Пример: /set cold 5464  (это 54.64 м³)");
      return;
    }

    portENTER_CRITICAL(&mux);
    if (who == "cold") cold_pulses = pulses;
    else if (who == "hot") hot_pulses = pulses;
    else {
      portEXIT_CRITICAL(&mux);
      bot.sendMessage("Куда set? cold или hot. Пример: /set cold 5464");
      return;
    }
    portEXIT_CRITICAL(&mux);

    saveState(true);
    bot.sendMessage("OK.\n" + statusText());
    return;
  }

  if (tl == "/save" || tl == "save") {
    saveState(true);
    bot.sendMessage("Saved.");
    return;
  }

  bot.sendMessage(
    "Команды:\n"
    "/get или /status — показания\n"
    "/deb N — мин. интервал между тиками (50..20000 мс)\n"
    "/set cold 5464 — установить (54.64 м³), только целые\n"
    "/set hot 2882 — установить (28.82 м³), только целые\n"
    "/diag — диагностика\n"
    "/save — сохранить\n"
  );
}

// ====== WIFI ======
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    if (millis() - t0 > 25000) ESP.restart();
  }
}

// ====== SETUP/LOOP ======
void setup() {
  Serial.begin(115200);

  pinMode(PIN_COLD, INPUT_PULLUP);
  pinMode(PIN_HOT,  INPUT_PULLUP);

  connectWiFi();

  bot.setChatID(ADMIN_CHAT_ID);
  bot.attach(newMsg);

  loadState();

  attachInterrupt(digitalPinToInterrupt(PIN_COLD), isrCold, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HOT),  isrHot,  FALLING);

  bot.sendMessage("Старт.\n" + statusText());
}

void loop() {
  bot.tick();

  processChannel(PIN_COLD, cold_pending, cold_pending_at, cold_armed, cold_high_since, cold_last_accept, cold_pulses);
  processChannel(PIN_HOT,  hot_pending,  hot_pending_at,  hot_armed,  hot_high_since,  hot_last_accept,  hot_pulses);

  // Сохранение по изменениям
  uint64_t c, h;
  portENTER_CRITICAL(&mux);
  c = cold_pulses;
  h = hot_pulses;
  portEXIT_CRITICAL(&mux);

  uint64_t dc = (c > last_saved_cold) ? (c - last_saved_cold) : 0;
  uint64_t dh = (h > last_saved_hot)  ? (h - last_saved_hot)  : 0;

  if ((dc + dh) >= SAVE_EVERY_PULSES) saveState(true);
  else if ((dc + dh) > 0) saveState(false);

  delay(10);
}
