#pragma once
#include <Arduino.h>
#include <map>

struct Coord { float x, y; };

class Cinematica {
public:
  // =================== VALORES-PADRÃO (editáveis no .cpp ou via setters) ===================
  static constexpr int   DEF_DIR1  = 3;
  static constexpr int   DEF_STEP1 = 2;
  static constexpr int   DEF_DIR2  = 5;
  static constexpr int   DEF_STEP2 = 4;
  static constexpr int   DEF_EN    = 8;     // -1 se não quiser controlar ENABLE por pino

  static constexpr float DEF_L1 = 238.0f;   // mm
  static constexpr float DEF_L2 = 207.0f;   // mm
  static constexpr float DEF_TH1_MIN = -70.0f;
  static constexpr float DEF_TH1_MAX =  70.0f;
  static constexpr float DEF_TH2_MIN = -135.0f;
  static constexpr float DEF_TH2_MAX =  135.0f;

  static constexpr float DEF_MOTOR_STEP_DEG = 1.8f;
  static constexpr uint8_t DEF_MICROSTEP_DIV = 16;

  static constexpr unsigned int DEF_PULSE_ON_US  = 5; // A4988 pede >= 1 µs
  static constexpr unsigned int DEF_PULSE_OFF_US = 5;

  // =================== CONSTRUTOR ===================
  // Construtor sem argumentos: já carrega TUDO com os padrões acima e o mapa 1..10
  Cinematica();

  // =================== CICLO DE VIDA ===================
  // Habilita pinos e driver (EN LOW). Se EN_ == -1, só configura pinos.
  void begin(bool enable = true);

  // Liga/desliga driver via EN (LOW=ligado). Ignorado se EN_ == -1.
  void enableDriver(bool enable);

  // =================== MOVIMENTO ===================
  bool goToIndex(uint8_t idx);         // 1..10
  bool goToXY(float x_mm, float y_mm); // coordenada arbitrária

  // =================== AJUSTES EM TEMPO DE EXECUÇÃO ===================
  void setCurrentAnglesDeg(float th1_deg, float th2_deg); // após homing
  void setPulseTimings(unsigned int us_on, unsigned int us_off);
  void setMicrostepDivider(uint8_t divider);              // se trocar hardware depois

  // Sobrescrever configurações (opcional, antes de begin())
  void setPins(int step1, int dir1, int step2, int dir2, int enPin = DEF_EN);
  void setGeometry(float L1_mm, float L2_mm);
  void setLimits(float th1_min, float th1_max, float th2_min, float th2_max);
  void setMotorStep(float motor_step_deg);
  void setMapPoint(uint8_t idx, float x_mm, float y_mm); // muda um ponto do mapa

  // =================== CONSULTAS ===================
  long  currentStepsJoint1() const { return pos1_steps_; }
  long  currentStepsJoint2() const { return pos2_steps_; }
  float currentTheta1Deg()   const { return pos1_steps_ * step_eff_deg_; }
  float currentTheta2Deg()   const { return pos2_steps_ * step_eff_deg_; }

private:
  // ---------- Configurações internas ----------
  // pinos
  int STEP1_ = DEF_STEP1, DIR1_ = DEF_DIR1;
  int STEP2_ = DEF_STEP2, DIR2_ = DEF_DIR2;
  int EN_    = DEF_EN; // -1 desabilita controle por pino

  // braço
  float L1_ = DEF_L1, L2_ = DEF_L2;
  float th1_min_ = DEF_TH1_MIN, th1_max_ = DEF_TH1_MAX;
  float th2_min_ = DEF_TH2_MIN, th2_max_ = DEF_TH2_MAX;

  // motor/step
  float motor_step_deg_ = DEF_MOTOR_STEP_DEG;
  uint8_t microstep_div_ = DEF_MICROSTEP_DIV;  // 1,2,4,8,16
  float step_eff_deg_ = DEF_MOTOR_STEP_DEG / DEF_MICROSTEP_DIV; // graus por passo efetivo

  // timing
  unsigned int us_on_  = DEF_PULSE_ON_US;
  unsigned int us_off_ = DEF_PULSE_OFF_US;

  // estado
  long pos1_steps_ = 0, pos2_steps_ = 0;

  // mapa 1..10
  std::map<int, Coord> points_;

  // ---------- Internos ----------
  bool  solveIK(float x, float y, float& th1_deg_out, float& th2_deg_out);
  long  degToSteps(float deg) const;
  void  moveMotor(long& currentPos, long targetPos, int STEP_PIN, int DIR_PIN);
  static bool inRange(float v, float vmin, float vmax);

  // Inicializa o mapa padrão 1..10
  void loadDefaultPoints();
};