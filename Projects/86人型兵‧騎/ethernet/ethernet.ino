#include <FirmataPlus86.h>

#define CONNECT_86DUINO_METHOD 3 // 0: USB Serial 1: BlueTooth 2: Arduino WiFi Shield 3: Ethernet 4: ESP8266 WiFi 5: ESP8266 AP

char* projectName = "86Humanoid";
char* _ssid = "your SSID";
char* _password = "your password"; // If it is ESP8266 AP mode, the password can not be shorter that 8 chars.
#define BT_ESP8266_Serial         Serial1
#define BT_ESP8266_Serial_Baud    9600

#if CONNECT_86DUINO_METHOD == 2 // Arduino WiFi Shield
    bool _wep = false;
    IPAddress _ip(0, 0, 0, 0);
#elif CONNECT_86DUINO_METHOD == 3 // Ethernet
    #define ETHERNET_DHCP
    #ifndef ETHERNET_DHCP
        IPAddress localIP(192, 168, 4, 100);
        IPAddress subnet(255, 255, 240, 0);
        IPAddress dnsserver(192, 168, 1, 1);
        IPAddress gateway(192, 168, 1, 1);
    #endif
#elif CONNECT_86DUINO_METHOD == 4 || CONNECT_86DUINO_METHOD == 5 // ESP8266
    int ch_pd_pin = 10;
    uint8_t _chl = 11; // channel ID (only for ESP8266 AP mode)
    uint8_t _ecn = 4; // encryption method (only for ESP8266 AP mode)
#endif
//////////////////////////////////////////////////////////////////////////////////

#include <avr/wdt.h>

#include <time.h>
#include <math.h>
#include <Servo86.h>



// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL 1

#define REGISTER_NOT_SPECIFIED -1

#define INTER_PING_INTERVAL 40 // 40 ms.

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/
 

bool checkActiveStart = false;
/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)
unsigned long previousKeepAliveMillis = 0;
unsigned int keepAliveInterval = 0;

boolean isResetting = false;

// test for "w" command to waiting
boolean test_waiting_start = false;
int waiting_id;
int waiting_time;
unsigned long start_waiting_time;

// Ping variables

int numLoops = 0 ;
int pingLoopCounter = 0 ;

uint8_t pingInterval = 33 ;  // Milliseconds between sensor pings (29ms is about the min to avoid
// cross- sensor echo).

#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif


int available_digital_pins[25] = {0, 3, 8, 13, 16, 21, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44};
int busy_id;
int motion_id;
int motion_times;



/*==============================================================================
 * CLASSES
 *============================================================================*/


class Robot86ME {
private:
  double _86ME_var[50];
  bool _86ME_cmd[28];
  double _roll;
  double _pitch;
  double _comp_range;
  double _IMU_val[12];
  double _IMU_Q[4];
  double _omega[2];
  int _IMU_init_status;
  int servo_mask[20];
  Servo used_servos[20];
  unsigned long _TurnLeft_Stage0_frm_time[11];
  unsigned long _TurnLeft_Stage1_frm_time[14];
  unsigned long _TurnLeft_Stage2_frm_time[10];
  unsigned long _TurnRight_Stage0_frm_time[11];
  unsigned long _TurnRight_Stage1_frm_time[14];
  unsigned long _TurnRight_Stage2_frm_time[10];
  unsigned long _WalkBwd_Stage0_frm_time[11];
  unsigned long _WalkBwd_Stage1_frm_time[14];
  unsigned long _WalkBwd_Stage2_frm_time[10];
  unsigned long _WalkFwd_Stage0_frm_time[11];
  unsigned long _WalkFwd_Stage1_frm_time[14];
  unsigned long _WalkFwd_Stage2_frm_time[10];
  unsigned long _WalkLeft_Stage0_frm_time[11];
  unsigned long _WalkLeft_Stage1_frm_time[14];
  unsigned long _WalkLeft_Stage2_frm_time[10];
  unsigned long _WalkRight_Stage0_frm_time[11];
  unsigned long _WalkRight_Stage1_frm_time[14];
  unsigned long _WalkRight_Stage2_frm_time[10];
  unsigned long _bow_frm_time[4];
  unsigned long _grab_frm_time[13];

  enum {_TURNLEFT_STAGE0, _TURNLEFT_STAGE1, _TURNLEFT_STAGE2, _TURNRIGHT_STAGE0, _TURNRIGHT_STAGE1, _TURNRIGHT_STAGE2, _WALKBWD_STAGE0, _WALKBWD_STAGE1, _WALKBWD_STAGE2, _WALKFWD_STAGE0, _WALKFWD_STAGE1, _WALKFWD_STAGE2, _WALKLEFT_STAGE0, _WALKLEFT_STAGE1, _WALKLEFT_STAGE2, _WALKRIGHT_STAGE0, _WALKRIGHT_STAGE1, _WALKRIGHT_STAGE2, _TURNLEFT, _TURNRIGHT, _WALKBWD, _WALKFWD, _WALKLEFT, _WALKRIGHT, _HELLOWORLD, _BOW, _GRAB, _HOME, _NONE};
  int _last_motion[2];
  int _curr_motion[2];
  bool internal_trigger[29];
  bool external_trigger[29];

  ServoOffset offsets;

  ServoFrame _86ME_HOME;
  ServoFrame _86ME_RUN;

  ServoFrame TurnLeft_Stage0_frm[11];
  ServoFrame TurnLeft_Stage1_frm[14];
  ServoFrame TurnLeft_Stage2_frm[10];
  ServoFrame TurnRight_Stage0_frm[11];
  ServoFrame TurnRight_Stage1_frm[14];
  ServoFrame TurnRight_Stage2_frm[10];
  ServoFrame WalkBwd_Stage0_frm[11];
  ServoFrame WalkBwd_Stage1_frm[14];
  ServoFrame WalkBwd_Stage2_frm[10];
  ServoFrame WalkFwd_Stage0_frm[11];
  ServoFrame WalkFwd_Stage1_frm[14];
  ServoFrame WalkFwd_Stage2_frm[10];
  ServoFrame WalkLeft_Stage0_frm[11];
  ServoFrame WalkLeft_Stage1_frm[14];
  ServoFrame WalkLeft_Stage2_frm[10];
  ServoFrame WalkRight_Stage0_frm[11];
  ServoFrame WalkRight_Stage1_frm[14];
  ServoFrame WalkRight_Stage2_frm[10];
  ServoFrame TurnLeft_frm[0];
  ServoFrame TurnRight_frm[0];
  ServoFrame WalkBwd_frm[0];
  ServoFrame WalkFwd_frm[0];
  ServoFrame WalkLeft_frm[0];
  ServoFrame WalkRight_frm[0];
  ServoFrame helloworld_frm[5];
  ServoFrame bow_frm[4];
  ServoFrame grab_frm[13];
  int home_title;
  ServoFrame home_frm[1];

  void updateIMU(void);
  bool isBlocked(int layer);
  void closeTriggers(int layer);
  bool isNoMotion(void);
  void updateCompRange(void);
  void updateCommand(void);
  void updateTrigger(void);
  void TurnLeft_Stage0Update(void);
  void TurnLeft_Stage1Update(void);
  void TurnLeft_Stage2Update(void);
  void TurnRight_Stage0Update(void);
  void TurnRight_Stage1Update(void);
  void TurnRight_Stage2Update(void);
  void WalkBwd_Stage0Update(void);
  void WalkBwd_Stage1Update(void);
  void WalkBwd_Stage2Update(void);
  void WalkFwd_Stage0Update(void);
  void WalkFwd_Stage1Update(void);
  void WalkFwd_Stage2Update(void);
  void WalkLeft_Stage0Update(void);
  void WalkLeft_Stage1Update(void);
  void WalkLeft_Stage2Update(void);
  void WalkRight_Stage0Update(void);
  void WalkRight_Stage1Update(void);
  void WalkRight_Stage2Update(void);
  void TurnLeftUpdate(void);
  void TurnRightUpdate(void);
  void WalkBwdUpdate(void);
  void WalkFwdUpdate(void);
  void WalkLeftUpdate(void);
  void WalkRightUpdate(void);
  void helloworldUpdate(void);
  void bowUpdate(void);
  void grabUpdate(void);
  void homeUpdate(void);
  void update_S(void);
public:
  Robot86ME();
  void begin(bool initIMU);
  void reset(bool initIMU);
  void update(void);
  void TurnLeft_Stage0(int times = 1);
  void TurnLeft_Stage1(int times = 1);
  void TurnLeft_Stage2(int times = 1);
  void TurnRight_Stage0(int times = 1);
  void TurnRight_Stage1(int times = 1);
  void TurnRight_Stage2(int times = 1);
  void WalkBwd_Stage0(int times = 1);
  void WalkBwd_Stage1(int times = 1);
  void WalkBwd_Stage2(int times = 1);
  void WalkFwd_Stage0(int times = 1);
  void WalkFwd_Stage1(int times = 1);
  void WalkFwd_Stage2(int times = 1);
  void WalkLeft_Stage0(int times = 1);
  void WalkLeft_Stage1(int times = 1);
  void WalkLeft_Stage2(int times = 1);
  void WalkRight_Stage0(int times = 1);
  void WalkRight_Stage1(int times = 1);
  void WalkRight_Stage2(int times = 1);
  void TurnLeft(int times = 1);
  void TurnRight(int times = 1);
  void WalkBwd(int times = 1);
  void WalkFwd(int times = 1);
  void WalkLeft(int times = 1);
  void WalkRight(int times = 1);
  void helloworld(int times = 1);
  void bow(int times = 1);
  void grab(int times = 1);
  void home(int times = 1);
};
namespace TurnLeft_Stage0
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace TurnLeft_Stage1
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10, FRAME_11, WAIT_FRAME_11, FRAME_12, WAIT_FRAME_12, FRAME_13, WAIT_FRAME_13};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace TurnLeft_Stage2
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace TurnRight_Stage0
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace TurnRight_Stage1
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10, FRAME_11, WAIT_FRAME_11, FRAME_12, WAIT_FRAME_12, FRAME_13, WAIT_FRAME_13};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace TurnRight_Stage2
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkBwd_Stage0
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkBwd_Stage1
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10, FRAME_11, WAIT_FRAME_11, FRAME_12, WAIT_FRAME_12, FRAME_13, WAIT_FRAME_13};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkBwd_Stage2
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkFwd_Stage0
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkFwd_Stage1
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10, FRAME_11, WAIT_FRAME_11, FRAME_12, WAIT_FRAME_12, FRAME_13, WAIT_FRAME_13};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkFwd_Stage2
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkLeft_Stage0
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkLeft_Stage1
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10, FRAME_11, WAIT_FRAME_11, FRAME_12, WAIT_FRAME_12, FRAME_13, WAIT_FRAME_13};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkLeft_Stage2
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkRight_Stage0
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkRight_Stage1
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10, FRAME_11, WAIT_FRAME_11, FRAME_12, WAIT_FRAME_12, FRAME_13, WAIT_FRAME_13};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkRight_Stage2
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace TurnLeft
{
  enum {IDLE, MOTION_0, WAIT_MOTION_0, MOTION_1, WAIT_MOTION_1, MOTION_2, WAIT_MOTION_2};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace TurnRight
{
  enum {IDLE, MOTION_0, WAIT_MOTION_0, MOTION_1, WAIT_MOTION_1, MOTION_2, WAIT_MOTION_2};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkBwd
{
  enum {IDLE, MOTION_0, WAIT_MOTION_0, MOTION_1, WAIT_MOTION_1, MOTION_2, WAIT_MOTION_2};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkFwd
{
  enum {IDLE, MOTION_0, WAIT_MOTION_0, MOTION_1, WAIT_MOTION_1, MOTION_2, WAIT_MOTION_2};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkLeft
{
  enum {IDLE, MOTION_0, WAIT_MOTION_0, MOTION_1, WAIT_MOTION_1, MOTION_2, WAIT_MOTION_2};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace WalkRight
{
  enum {IDLE, MOTION_0, WAIT_MOTION_0, MOTION_1, WAIT_MOTION_1, MOTION_2, WAIT_MOTION_2};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace helloworld
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
  int mask[20] = { 0xffff, 0xffff, 0xffff, 0xffff, 0, 0, 0, 0, 0, 0, 0xffff, 0xffff, 0xffff, 0xffff, 0, 0, 0, 0, 0, 0};
}
namespace bow
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace grab
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5, FRAME_6, WAIT_FRAME_6, FRAME_7, WAIT_FRAME_7, FRAME_8, WAIT_FRAME_8, FRAME_9, WAIT_FRAME_9, FRAME_10, WAIT_FRAME_10, FRAME_11, WAIT_FRAME_11, FRAME_12, WAIT_FRAME_12};
  int state = IDLE;
  unsigned long time;
  double comp_range = 30;
}
namespace home
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0};
  int state = IDLE;
  unsigned long time;
  double comp_range = 180;
}


/*==============================================================================
 * FUNCTIONS
 *============================================================================*/


Robot86ME::Robot86ME()
: _86ME_var(), _86ME_cmd(), _roll(0), _pitch(0), _comp_range(180), _IMU_val(),
  _IMU_Q{ 1, 0, 0, 0}, _omega(), _IMU_init_status(-1), servo_mask(),
  _TurnLeft_Stage0_frm_time{ 200, 80, 180, 200, 20, 40, 40, 60, 80, 80, 60},
  _TurnLeft_Stage1_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 40, 40, 60, 80, 80, 60},
  _TurnLeft_Stage2_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 60, 60},
  _TurnRight_Stage0_frm_time{ 200, 80, 180, 200, 20, 40, 40, 60, 80, 80, 60},
  _TurnRight_Stage1_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 40, 40, 60, 80, 80, 60},
  _TurnRight_Stage2_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 60, 60},
  _WalkBwd_Stage0_frm_time{ 200, 80, 180, 200, 20, 40, 40, 60, 80, 80, 60},
  _WalkBwd_Stage1_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 40, 40, 60, 80, 80, 60},
  _WalkBwd_Stage2_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 60, 60},
  _WalkFwd_Stage0_frm_time{ 200, 80, 180, 200, 20, 40, 40, 60, 80, 80, 60},
  _WalkFwd_Stage1_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 40, 40, 60, 80, 80, 60},
  _WalkFwd_Stage2_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 60, 60},
  _WalkLeft_Stage0_frm_time{ 200, 80, 180, 200, 20, 40, 40, 60, 80, 80, 60},
  _WalkLeft_Stage1_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 40, 40, 60, 80, 80, 60},
  _WalkLeft_Stage2_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 60, 60},
  _WalkRight_Stage0_frm_time{ 200, 80, 180, 200, 20, 40, 40, 60, 80, 80, 60},
  _WalkRight_Stage1_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 40, 40, 60, 80, 80, 60},
  _WalkRight_Stage2_frm_time{ 20, 40, 40, 60, 80, 80, 60, 20, 60, 60},
  _bow_frm_time{ 500, 500, 700, 700},
  _grab_frm_time{ 500, 500, 500, 300, 300, 300, 300, 500, 300, 300, 300, 300, 500},
  home_title(1),
  _last_motion{ _NONE, _NONE}, _curr_motion{ _NONE, _NONE},
  internal_trigger(), external_trigger()
{ }

bool Robot86ME::isBlocked(int layer)
{
  if(layer == 0)
  {
    if(external_trigger[_TURNLEFT_STAGE0]) return true;
    if(external_trigger[_TURNLEFT_STAGE1]) return true;
    if(external_trigger[_TURNLEFT_STAGE2]) return true;
    if(external_trigger[_TURNRIGHT_STAGE0]) return true;
    if(external_trigger[_TURNRIGHT_STAGE1]) return true;
    if(external_trigger[_TURNRIGHT_STAGE2]) return true;
    if(external_trigger[_WALKBWD_STAGE0]) return true;
    if(external_trigger[_WALKBWD_STAGE1]) return true;
    if(external_trigger[_WALKBWD_STAGE2]) return true;
    if(external_trigger[_WALKFWD_STAGE0]) return true;
    if(external_trigger[_WALKFWD_STAGE1]) return true;
    if(external_trigger[_WALKFWD_STAGE2]) return true;
    if(external_trigger[_WALKLEFT_STAGE0]) return true;
    if(external_trigger[_WALKLEFT_STAGE1]) return true;
    if(external_trigger[_WALKLEFT_STAGE2]) return true;
    if(external_trigger[_WALKRIGHT_STAGE0]) return true;
    if(external_trigger[_WALKRIGHT_STAGE1]) return true;
    if(external_trigger[_WALKRIGHT_STAGE2]) return true;
    if(external_trigger[_TURNLEFT]) return true;
    if(external_trigger[_TURNRIGHT]) return true;
    if(external_trigger[_WALKBWD]) return true;
    if(external_trigger[_WALKFWD]) return true;
    if(external_trigger[_WALKLEFT]) return true;
    if(external_trigger[_WALKRIGHT]) return true;
    if(external_trigger[_BOW]) return true;
    if(external_trigger[_GRAB]) return true;
    if(external_trigger[_HOME]) return true;
  }
  else if(layer == 1)
  {
    if(external_trigger[_HELLOWORLD]) return true;
  }
  return false;
}
void Robot86ME::closeTriggers(int layer)
{
  if(layer == 0)
  {
    external_trigger[_TURNLEFT_STAGE0]= false; internal_trigger[_TURNLEFT_STAGE0]= false;
    external_trigger[_TURNLEFT_STAGE1]= false; internal_trigger[_TURNLEFT_STAGE1]= false;
    external_trigger[_TURNLEFT_STAGE2]= false; internal_trigger[_TURNLEFT_STAGE2]= false;
    external_trigger[_TURNRIGHT_STAGE0]= false; internal_trigger[_TURNRIGHT_STAGE0]= false;
    external_trigger[_TURNRIGHT_STAGE1]= false; internal_trigger[_TURNRIGHT_STAGE1]= false;
    external_trigger[_TURNRIGHT_STAGE2]= false; internal_trigger[_TURNRIGHT_STAGE2]= false;
    external_trigger[_WALKBWD_STAGE0]= false; internal_trigger[_WALKBWD_STAGE0]= false;
    external_trigger[_WALKBWD_STAGE1]= false; internal_trigger[_WALKBWD_STAGE1]= false;
    external_trigger[_WALKBWD_STAGE2]= false; internal_trigger[_WALKBWD_STAGE2]= false;
    external_trigger[_WALKFWD_STAGE0]= false; internal_trigger[_WALKFWD_STAGE0]= false;
    external_trigger[_WALKFWD_STAGE1]= false; internal_trigger[_WALKFWD_STAGE1]= false;
    external_trigger[_WALKFWD_STAGE2]= false; internal_trigger[_WALKFWD_STAGE2]= false;
    external_trigger[_WALKLEFT_STAGE0]= false; internal_trigger[_WALKLEFT_STAGE0]= false;
    external_trigger[_WALKLEFT_STAGE1]= false; internal_trigger[_WALKLEFT_STAGE1]= false;
    external_trigger[_WALKLEFT_STAGE2]= false; internal_trigger[_WALKLEFT_STAGE2]= false;
    external_trigger[_WALKRIGHT_STAGE0]= false; internal_trigger[_WALKRIGHT_STAGE0]= false;
    external_trigger[_WALKRIGHT_STAGE1]= false; internal_trigger[_WALKRIGHT_STAGE1]= false;
    external_trigger[_WALKRIGHT_STAGE2]= false; internal_trigger[_WALKRIGHT_STAGE2]= false;
    external_trigger[_TURNLEFT]= false; internal_trigger[_TURNLEFT]= false;
    external_trigger[_TURNRIGHT]= false; internal_trigger[_TURNRIGHT]= false;
    external_trigger[_WALKBWD]= false; internal_trigger[_WALKBWD]= false;
    external_trigger[_WALKFWD]= false; internal_trigger[_WALKFWD]= false;
    external_trigger[_WALKLEFT]= false; internal_trigger[_WALKLEFT]= false;
    external_trigger[_WALKRIGHT]= false; internal_trigger[_WALKRIGHT]= false;
    external_trigger[_BOW]= false; internal_trigger[_BOW]= false;
    external_trigger[_GRAB]= false; internal_trigger[_GRAB]= false;
    external_trigger[_HOME]= false; internal_trigger[_HOME]= false;
  }
  else if(layer == 1)
  {
    external_trigger[_HELLOWORLD]= false; internal_trigger[_HELLOWORLD]= false;
  }
}
bool Robot86ME::isNoMotion()
{
  if(external_trigger[_TURNLEFT_STAGE0] || internal_trigger[_TURNLEFT_STAGE0] ||
     external_trigger[_TURNLEFT_STAGE1] || internal_trigger[_TURNLEFT_STAGE1] ||
     external_trigger[_TURNLEFT_STAGE2] || internal_trigger[_TURNLEFT_STAGE2] ||
     external_trigger[_TURNRIGHT_STAGE0] || internal_trigger[_TURNRIGHT_STAGE0] ||
     external_trigger[_TURNRIGHT_STAGE1] || internal_trigger[_TURNRIGHT_STAGE1] ||
     external_trigger[_TURNRIGHT_STAGE2] || internal_trigger[_TURNRIGHT_STAGE2] ||
     external_trigger[_WALKBWD_STAGE0] || internal_trigger[_WALKBWD_STAGE0] ||
     external_trigger[_WALKBWD_STAGE1] || internal_trigger[_WALKBWD_STAGE1] ||
     external_trigger[_WALKBWD_STAGE2] || internal_trigger[_WALKBWD_STAGE2] ||
     external_trigger[_WALKFWD_STAGE0] || internal_trigger[_WALKFWD_STAGE0] ||
     external_trigger[_WALKFWD_STAGE1] || internal_trigger[_WALKFWD_STAGE1] ||
     external_trigger[_WALKFWD_STAGE2] || internal_trigger[_WALKFWD_STAGE2] ||
     external_trigger[_WALKLEFT_STAGE0] || internal_trigger[_WALKLEFT_STAGE0] ||
     external_trigger[_WALKLEFT_STAGE1] || internal_trigger[_WALKLEFT_STAGE1] ||
     external_trigger[_WALKLEFT_STAGE2] || internal_trigger[_WALKLEFT_STAGE2] ||
     external_trigger[_WALKRIGHT_STAGE0] || internal_trigger[_WALKRIGHT_STAGE0] ||
     external_trigger[_WALKRIGHT_STAGE1] || internal_trigger[_WALKRIGHT_STAGE1] ||
     external_trigger[_WALKRIGHT_STAGE2] || internal_trigger[_WALKRIGHT_STAGE2] ||
     external_trigger[_TURNLEFT] || internal_trigger[_TURNLEFT] ||
     external_trigger[_TURNRIGHT] || internal_trigger[_TURNRIGHT] ||
     external_trigger[_WALKBWD] || internal_trigger[_WALKBWD] ||
     external_trigger[_WALKFWD] || internal_trigger[_WALKFWD] ||
     external_trigger[_WALKLEFT] || internal_trigger[_WALKLEFT] ||
     external_trigger[_WALKRIGHT] || internal_trigger[_WALKRIGHT] ||
     external_trigger[_HELLOWORLD] || internal_trigger[_HELLOWORLD] ||
     external_trigger[_BOW] || internal_trigger[_BOW] ||
     external_trigger[_GRAB] || internal_trigger[_GRAB] ||
     external_trigger[_HOME] || internal_trigger[_HOME] )
    return false;
  return true;
}
void Robot86ME::updateCompRange()
{
  _comp_range = -1;
  if((external_trigger[_TURNLEFT_STAGE0] || internal_trigger[_TURNLEFT_STAGE0]) && TurnLeft_Stage0::comp_range >= _comp_range)
    _comp_range = TurnLeft_Stage0::comp_range;
  else if((external_trigger[_TURNLEFT_STAGE1] || internal_trigger[_TURNLEFT_STAGE1]) && TurnLeft_Stage1::comp_range >= _comp_range)
    _comp_range = TurnLeft_Stage1::comp_range;
  else if((external_trigger[_TURNLEFT_STAGE2] || internal_trigger[_TURNLEFT_STAGE2]) && TurnLeft_Stage2::comp_range >= _comp_range)
    _comp_range = TurnLeft_Stage2::comp_range;
  else if((external_trigger[_TURNRIGHT_STAGE0] || internal_trigger[_TURNRIGHT_STAGE0]) && TurnRight_Stage0::comp_range >= _comp_range)
    _comp_range = TurnRight_Stage0::comp_range;
  else if((external_trigger[_TURNRIGHT_STAGE1] || internal_trigger[_TURNRIGHT_STAGE1]) && TurnRight_Stage1::comp_range >= _comp_range)
    _comp_range = TurnRight_Stage1::comp_range;
  else if((external_trigger[_TURNRIGHT_STAGE2] || internal_trigger[_TURNRIGHT_STAGE2]) && TurnRight_Stage2::comp_range >= _comp_range)
    _comp_range = TurnRight_Stage2::comp_range;
  else if((external_trigger[_WALKBWD_STAGE0] || internal_trigger[_WALKBWD_STAGE0]) && WalkBwd_Stage0::comp_range >= _comp_range)
    _comp_range = WalkBwd_Stage0::comp_range;
  else if((external_trigger[_WALKBWD_STAGE1] || internal_trigger[_WALKBWD_STAGE1]) && WalkBwd_Stage1::comp_range >= _comp_range)
    _comp_range = WalkBwd_Stage1::comp_range;
  else if((external_trigger[_WALKBWD_STAGE2] || internal_trigger[_WALKBWD_STAGE2]) && WalkBwd_Stage2::comp_range >= _comp_range)
    _comp_range = WalkBwd_Stage2::comp_range;
  else if((external_trigger[_WALKFWD_STAGE0] || internal_trigger[_WALKFWD_STAGE0]) && WalkFwd_Stage0::comp_range >= _comp_range)
    _comp_range = WalkFwd_Stage0::comp_range;
  else if((external_trigger[_WALKFWD_STAGE1] || internal_trigger[_WALKFWD_STAGE1]) && WalkFwd_Stage1::comp_range >= _comp_range)
    _comp_range = WalkFwd_Stage1::comp_range;
  else if((external_trigger[_WALKFWD_STAGE2] || internal_trigger[_WALKFWD_STAGE2]) && WalkFwd_Stage2::comp_range >= _comp_range)
    _comp_range = WalkFwd_Stage2::comp_range;
  else if((external_trigger[_WALKLEFT_STAGE0] || internal_trigger[_WALKLEFT_STAGE0]) && WalkLeft_Stage0::comp_range >= _comp_range)
    _comp_range = WalkLeft_Stage0::comp_range;
  else if((external_trigger[_WALKLEFT_STAGE1] || internal_trigger[_WALKLEFT_STAGE1]) && WalkLeft_Stage1::comp_range >= _comp_range)
    _comp_range = WalkLeft_Stage1::comp_range;
  else if((external_trigger[_WALKLEFT_STAGE2] || internal_trigger[_WALKLEFT_STAGE2]) && WalkLeft_Stage2::comp_range >= _comp_range)
    _comp_range = WalkLeft_Stage2::comp_range;
  else if((external_trigger[_WALKRIGHT_STAGE0] || internal_trigger[_WALKRIGHT_STAGE0]) && WalkRight_Stage0::comp_range >= _comp_range)
    _comp_range = WalkRight_Stage0::comp_range;
  else if((external_trigger[_WALKRIGHT_STAGE1] || internal_trigger[_WALKRIGHT_STAGE1]) && WalkRight_Stage1::comp_range >= _comp_range)
    _comp_range = WalkRight_Stage1::comp_range;
  else if((external_trigger[_WALKRIGHT_STAGE2] || internal_trigger[_WALKRIGHT_STAGE2]) && WalkRight_Stage2::comp_range >= _comp_range)
    _comp_range = WalkRight_Stage2::comp_range;
  else if((external_trigger[_TURNLEFT] || internal_trigger[_TURNLEFT]) && TurnLeft::comp_range >= _comp_range)
    _comp_range = TurnLeft::comp_range;
  else if((external_trigger[_TURNRIGHT] || internal_trigger[_TURNRIGHT]) && TurnRight::comp_range >= _comp_range)
    _comp_range = TurnRight::comp_range;
  else if((external_trigger[_WALKBWD] || internal_trigger[_WALKBWD]) && WalkBwd::comp_range >= _comp_range)
    _comp_range = WalkBwd::comp_range;
  else if((external_trigger[_WALKFWD] || internal_trigger[_WALKFWD]) && WalkFwd::comp_range >= _comp_range)
    _comp_range = WalkFwd::comp_range;
  else if((external_trigger[_WALKLEFT] || internal_trigger[_WALKLEFT]) && WalkLeft::comp_range >= _comp_range)
    _comp_range = WalkLeft::comp_range;
  else if((external_trigger[_WALKRIGHT] || internal_trigger[_WALKRIGHT]) && WalkRight::comp_range >= _comp_range)
    _comp_range = WalkRight::comp_range;
  else if((external_trigger[_HELLOWORLD] || internal_trigger[_HELLOWORLD]) && helloworld::comp_range >= _comp_range)
    _comp_range = helloworld::comp_range;
  else if((external_trigger[_BOW] || internal_trigger[_BOW]) && bow::comp_range >= _comp_range)
    _comp_range = bow::comp_range;
  else if((external_trigger[_GRAB] || internal_trigger[_GRAB]) && grab::comp_range >= _comp_range)
    _comp_range = grab::comp_range;
  else if((external_trigger[_HOME] || internal_trigger[_HOME]) && home::comp_range >= _comp_range)
    _comp_range = home::comp_range;
  else
    _comp_range = 180;
}
void Robot86ME::updateCommand()
{
  if(0) {_86ME_cmd[0] = true;}
  else {_86ME_cmd[0] = false;}
  if(0) {_86ME_cmd[1] = true;}
  else {_86ME_cmd[1] = false;}
  if(0) {_86ME_cmd[2] = true;}
  else {_86ME_cmd[2] = false;}
  if(0) {_86ME_cmd[3] = true;}
  else {_86ME_cmd[3] = false;}
  if(0) {_86ME_cmd[4] = true;}
  else {_86ME_cmd[4] = false;}
  if(0) {_86ME_cmd[5] = true;}
  else {_86ME_cmd[5] = false;}
  if(0) {_86ME_cmd[6] = true;}
  else {_86ME_cmd[6] = false;}
  if(0) {_86ME_cmd[7] = true;}
  else {_86ME_cmd[7] = false;}
  if(0) {_86ME_cmd[8] = true;}
  else {_86ME_cmd[8] = false;}
  if(0) {_86ME_cmd[9] = true;}
  else {_86ME_cmd[9] = false;}
  if(0) {_86ME_cmd[10] = true;}
  else {_86ME_cmd[10] = false;}
  if(0) {_86ME_cmd[11] = true;}
  else {_86ME_cmd[11] = false;}
  if(0) {_86ME_cmd[12] = true;}
  else {_86ME_cmd[12] = false;}
  if(0) {_86ME_cmd[13] = true;}
  else {_86ME_cmd[13] = false;}
  if(0) {_86ME_cmd[14] = true;}
  else {_86ME_cmd[14] = false;}
  if(0) {_86ME_cmd[15] = true;}
  else {_86ME_cmd[15] = false;}
  if(0) {_86ME_cmd[16] = true;}
  else {_86ME_cmd[16] = false;}
  if(0) {_86ME_cmd[17] = true;}
  else {_86ME_cmd[17] = false;}
  if(1) {_86ME_cmd[18] = true;}
  else {_86ME_cmd[18] = false;}
  if(1) {_86ME_cmd[19] = true;}
  else {_86ME_cmd[19] = false;}
  if(1) {_86ME_cmd[20] = true;}
  else {_86ME_cmd[20] = false;}
  if(1) {_86ME_cmd[21] = true;}
  else {_86ME_cmd[21] = false;}
  if(1) {_86ME_cmd[22] = true;}
  else {_86ME_cmd[22] = false;}
  if(1) {_86ME_cmd[23] = true;}
  else {_86ME_cmd[23] = false;}
  if(1) {_86ME_cmd[24] = true;}
  else {_86ME_cmd[24] = false;}
  if(1) {_86ME_cmd[25] = true;}
  else {_86ME_cmd[25] = false;}
  if(0) {_86ME_cmd[26] = true;}
  else {_86ME_cmd[26] = false;}
  if(home_title == 1) {_86ME_cmd[27] = true;}
  else {_86ME_cmd[27] = false;}
}
void Robot86ME::updateTrigger()
{
  if(isBlocked(0)) goto L1;
L0:
  if(_86ME_cmd[0]) {_curr_motion[0] = _TURNLEFT_STAGE0;}
  else if(_86ME_cmd[1]) {_curr_motion[0] = _TURNLEFT_STAGE1;}
  else if(_86ME_cmd[2]) {_curr_motion[0] = _TURNLEFT_STAGE2;}
  else if(_86ME_cmd[3]) {_curr_motion[0] = _TURNRIGHT_STAGE0;}
  else if(_86ME_cmd[4]) {_curr_motion[0] = _TURNRIGHT_STAGE1;}
  else if(_86ME_cmd[5]) {_curr_motion[0] = _TURNRIGHT_STAGE2;}
  else if(_86ME_cmd[6]) {_curr_motion[0] = _WALKBWD_STAGE0;}
  else if(_86ME_cmd[7]) {_curr_motion[0] = _WALKBWD_STAGE1;}
  else if(_86ME_cmd[8]) {_curr_motion[0] = _WALKBWD_STAGE2;}
  else if(_86ME_cmd[9]) {_curr_motion[0] = _WALKFWD_STAGE0;}
  else if(_86ME_cmd[10]) {_curr_motion[0] = _WALKFWD_STAGE1;}
  else if(_86ME_cmd[11]) {_curr_motion[0] = _WALKFWD_STAGE2;}
  else if(_86ME_cmd[12]) {_curr_motion[0] = _WALKLEFT_STAGE0;}
  else if(_86ME_cmd[13]) {_curr_motion[0] = _WALKLEFT_STAGE1;}
  else if(_86ME_cmd[14]) {_curr_motion[0] = _WALKLEFT_STAGE2;}
  else if(_86ME_cmd[15]) {_curr_motion[0] = _WALKRIGHT_STAGE0;}
  else if(_86ME_cmd[16]) {_curr_motion[0] = _WALKRIGHT_STAGE1;}
  else if(_86ME_cmd[17]) {_curr_motion[0] = _WALKRIGHT_STAGE2;}
  else if(_86ME_cmd[18]) {_curr_motion[0] = _TURNLEFT;}
  else if(_86ME_cmd[19]) {_curr_motion[0] = _TURNRIGHT;}
  else if(_86ME_cmd[20]) {_curr_motion[0] = _WALKBWD;}
  else if(_86ME_cmd[21]) {_curr_motion[0] = _WALKFWD;}
  else if(_86ME_cmd[22]) {_curr_motion[0] = _WALKLEFT;}
  else if(_86ME_cmd[23]) {_curr_motion[0] = _WALKRIGHT;}
  else if(_86ME_cmd[25]) {_curr_motion[0] = _BOW;}
  else if(_86ME_cmd[26]) {_curr_motion[0] = _GRAB;}
  else if(_86ME_cmd[27]) {_curr_motion[0] = _HOME;}
  else _curr_motion[0] = _NONE;
  if(_last_motion[0] != _curr_motion[0] && _curr_motion[0] != _NONE)
  {
    closeTriggers(0);
    external_trigger[_curr_motion[0]] = true;
    TurnLeft_Stage0::state = 0;
    TurnLeft_Stage1::state = 0;
    TurnLeft_Stage2::state = 0;
    TurnRight_Stage0::state = 0;
    TurnRight_Stage1::state = 0;
    TurnRight_Stage2::state = 0;
    WalkBwd_Stage0::state = 0;
    WalkBwd_Stage1::state = 0;
    WalkBwd_Stage2::state = 0;
    WalkFwd_Stage0::state = 0;
    WalkFwd_Stage1::state = 0;
    WalkFwd_Stage2::state = 0;
    WalkLeft_Stage0::state = 0;
    WalkLeft_Stage1::state = 0;
    WalkLeft_Stage2::state = 0;
    WalkRight_Stage0::state = 0;
    WalkRight_Stage1::state = 0;
    WalkRight_Stage2::state = 0;
    TurnLeft::state = 0;
    TurnRight::state = 0;
    WalkBwd::state = 0;
    WalkFwd::state = 0;
    WalkLeft::state = 0;
    WalkRight::state = 0;
    bow::state = 0;
    grab::state = 0;
    home::state = 0;
  }
  external_trigger[_curr_motion[0]] = true;
  _last_motion[0] = _curr_motion[0];
L1:
  if(isBlocked(1)) return;
  if(_86ME_cmd[24]) {_curr_motion[1] = _HELLOWORLD;memcpy(servo_mask, helloworld::mask, sizeof(servo_mask));}
  else {_curr_motion[1] = _NONE;  memset(servo_mask, 0, sizeof(servo_mask));}
  if(_last_motion[1] != _curr_motion[1] && _curr_motion[1] != _NONE)
  {
    closeTriggers(1);
    external_trigger[_curr_motion[1]] = true;
    helloworld::state = 0;
  }
  external_trigger[_curr_motion[1]] = true;
  _last_motion[1] = _curr_motion[1];
}
void Robot86ME::TurnLeft_Stage0Update()
{
  switch(TurnLeft_Stage0::state)
  {
  case TurnLeft_Stage0::IDLE:
    if(external_trigger[_TURNLEFT_STAGE0] || internal_trigger[_TURNLEFT_STAGE0]) TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_0;
    else break;
  case TurnLeft_Stage0::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_0;
  case TurnLeft_Stage0::WAIT_FRAME_0:
    if(millis() - TurnLeft_Stage0::time >= 200)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_1;
    break;
  case TurnLeft_Stage0::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_1;
  case TurnLeft_Stage0::WAIT_FRAME_1:
    if(millis() - TurnLeft_Stage0::time >= 80)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_2;
    break;
  case TurnLeft_Stage0::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)180);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_2;
  case TurnLeft_Stage0::WAIT_FRAME_2:
    if(millis() - TurnLeft_Stage0::time >= 180)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_3;
    break;
  case TurnLeft_Stage0::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_3;
  case TurnLeft_Stage0::WAIT_FRAME_3:
    if(millis() - TurnLeft_Stage0::time >= 200)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_4;
    break;
  case TurnLeft_Stage0::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_4;
  case TurnLeft_Stage0::WAIT_FRAME_4:
    if(millis() - TurnLeft_Stage0::time >= 20)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_5;
    break;
  case TurnLeft_Stage0::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_5;
  case TurnLeft_Stage0::WAIT_FRAME_5:
    if(millis() - TurnLeft_Stage0::time >= 40)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_6;
    break;
  case TurnLeft_Stage0::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_6;
  case TurnLeft_Stage0::WAIT_FRAME_6:
    if(millis() - TurnLeft_Stage0::time >= 40)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_7;
    break;
  case TurnLeft_Stage0::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_7;
  case TurnLeft_Stage0::WAIT_FRAME_7:
    if(millis() - TurnLeft_Stage0::time >= 60)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_8;
    break;
  case TurnLeft_Stage0::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_8;
  case TurnLeft_Stage0::WAIT_FRAME_8:
    if(millis() - TurnLeft_Stage0::time >= 80)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_9;
    break;
  case TurnLeft_Stage0::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_9;
  case TurnLeft_Stage0::WAIT_FRAME_9:
    if(millis() - TurnLeft_Stage0::time >= 80)
      TurnLeft_Stage0::state = TurnLeft_Stage0::FRAME_10;
    break;
  case TurnLeft_Stage0::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage0_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage0_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage0_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage0::time = millis();
    TurnLeft_Stage0::state = TurnLeft_Stage0::WAIT_FRAME_10;
  case TurnLeft_Stage0::WAIT_FRAME_10:
    if(millis() - TurnLeft_Stage0::time >= 60)
    {
      TurnLeft_Stage0::state = TurnLeft_Stage0::IDLE;
      internal_trigger[_TURNLEFT_STAGE0] = false;
      external_trigger[_TURNLEFT_STAGE0] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnLeft_Stage0(int times)
{
  _86ME_cmd[0] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[0] = false;
}
void Robot86ME::TurnLeft_Stage1Update()
{
  switch(TurnLeft_Stage1::state)
  {
  case TurnLeft_Stage1::IDLE:
    if(external_trigger[_TURNLEFT_STAGE1] || internal_trigger[_TURNLEFT_STAGE1]) TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_0;
    else break;
  case TurnLeft_Stage1::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_0;
  case TurnLeft_Stage1::WAIT_FRAME_0:
    if(millis() - TurnLeft_Stage1::time >= 20)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_1;
    break;
  case TurnLeft_Stage1::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_1;
  case TurnLeft_Stage1::WAIT_FRAME_1:
    if(millis() - TurnLeft_Stage1::time >= 40)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_2;
    break;
  case TurnLeft_Stage1::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_2;
  case TurnLeft_Stage1::WAIT_FRAME_2:
    if(millis() - TurnLeft_Stage1::time >= 40)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_3;
    break;
  case TurnLeft_Stage1::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_3;
  case TurnLeft_Stage1::WAIT_FRAME_3:
    if(millis() - TurnLeft_Stage1::time >= 60)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_4;
    break;
  case TurnLeft_Stage1::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_4;
  case TurnLeft_Stage1::WAIT_FRAME_4:
    if(millis() - TurnLeft_Stage1::time >= 80)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_5;
    break;
  case TurnLeft_Stage1::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_5;
  case TurnLeft_Stage1::WAIT_FRAME_5:
    if(millis() - TurnLeft_Stage1::time >= 80)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_6;
    break;
  case TurnLeft_Stage1::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_6;
  case TurnLeft_Stage1::WAIT_FRAME_6:
    if(millis() - TurnLeft_Stage1::time >= 60)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_7;
    break;
  case TurnLeft_Stage1::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_7;
  case TurnLeft_Stage1::WAIT_FRAME_7:
    if(millis() - TurnLeft_Stage1::time >= 20)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_8;
    break;
  case TurnLeft_Stage1::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_8;
  case TurnLeft_Stage1::WAIT_FRAME_8:
    if(millis() - TurnLeft_Stage1::time >= 40)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_9;
    break;
  case TurnLeft_Stage1::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_9;
  case TurnLeft_Stage1::WAIT_FRAME_9:
    if(millis() - TurnLeft_Stage1::time >= 40)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_10;
    break;
  case TurnLeft_Stage1::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_10;
  case TurnLeft_Stage1::WAIT_FRAME_10:
    if(millis() - TurnLeft_Stage1::time >= 60)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_11;
    break;
  case TurnLeft_Stage1::FRAME_11:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[11].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[11].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[11].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_11;
  case TurnLeft_Stage1::WAIT_FRAME_11:
    if(millis() - TurnLeft_Stage1::time >= 80)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_12;
    break;
  case TurnLeft_Stage1::FRAME_12:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[12].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[12].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[12].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_12;
  case TurnLeft_Stage1::WAIT_FRAME_12:
    if(millis() - TurnLeft_Stage1::time >= 80)
      TurnLeft_Stage1::state = TurnLeft_Stage1::FRAME_13;
    break;
  case TurnLeft_Stage1::FRAME_13:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage1_frm[13].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage1_frm[13].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage1_frm[13].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage1::time = millis();
    TurnLeft_Stage1::state = TurnLeft_Stage1::WAIT_FRAME_13;
  case TurnLeft_Stage1::WAIT_FRAME_13:
    if(millis() - TurnLeft_Stage1::time >= 60)
    {
      TurnLeft_Stage1::state = TurnLeft_Stage1::IDLE;
      internal_trigger[_TURNLEFT_STAGE1] = false;
      external_trigger[_TURNLEFT_STAGE1] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnLeft_Stage1(int times)
{
  _86ME_cmd[1] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[1] = false;
}
void Robot86ME::TurnLeft_Stage2Update()
{
  switch(TurnLeft_Stage2::state)
  {
  case TurnLeft_Stage2::IDLE:
    if(external_trigger[_TURNLEFT_STAGE2] || internal_trigger[_TURNLEFT_STAGE2]) TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_0;
    else break;
  case TurnLeft_Stage2::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_0;
  case TurnLeft_Stage2::WAIT_FRAME_0:
    if(millis() - TurnLeft_Stage2::time >= 20)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_1;
    break;
  case TurnLeft_Stage2::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_1;
  case TurnLeft_Stage2::WAIT_FRAME_1:
    if(millis() - TurnLeft_Stage2::time >= 40)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_2;
    break;
  case TurnLeft_Stage2::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_2;
  case TurnLeft_Stage2::WAIT_FRAME_2:
    if(millis() - TurnLeft_Stage2::time >= 40)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_3;
    break;
  case TurnLeft_Stage2::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_3;
  case TurnLeft_Stage2::WAIT_FRAME_3:
    if(millis() - TurnLeft_Stage2::time >= 60)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_4;
    break;
  case TurnLeft_Stage2::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_4;
  case TurnLeft_Stage2::WAIT_FRAME_4:
    if(millis() - TurnLeft_Stage2::time >= 80)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_5;
    break;
  case TurnLeft_Stage2::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_5;
  case TurnLeft_Stage2::WAIT_FRAME_5:
    if(millis() - TurnLeft_Stage2::time >= 80)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_6;
    break;
  case TurnLeft_Stage2::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_6;
  case TurnLeft_Stage2::WAIT_FRAME_6:
    if(millis() - TurnLeft_Stage2::time >= 60)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_7;
    break;
  case TurnLeft_Stage2::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_7;
  case TurnLeft_Stage2::WAIT_FRAME_7:
    if(millis() - TurnLeft_Stage2::time >= 20)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_8;
    break;
  case TurnLeft_Stage2::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_8;
  case TurnLeft_Stage2::WAIT_FRAME_8:
    if(millis() - TurnLeft_Stage2::time >= 60)
      TurnLeft_Stage2::state = TurnLeft_Stage2::FRAME_9;
    break;
  case TurnLeft_Stage2::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnLeft_Stage2_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnLeft_Stage2_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnLeft_Stage2_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnLeft_Stage2::time = millis();
    TurnLeft_Stage2::state = TurnLeft_Stage2::WAIT_FRAME_9;
  case TurnLeft_Stage2::WAIT_FRAME_9:
    if(millis() - TurnLeft_Stage2::time >= 60)
    {
      TurnLeft_Stage2::state = TurnLeft_Stage2::IDLE;
      internal_trigger[_TURNLEFT_STAGE2] = false;
      external_trigger[_TURNLEFT_STAGE2] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnLeft_Stage2(int times)
{
  _86ME_cmd[2] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[2] = false;
}
void Robot86ME::TurnRight_Stage0Update()
{
  switch(TurnRight_Stage0::state)
  {
  case TurnRight_Stage0::IDLE:
    if(external_trigger[_TURNRIGHT_STAGE0] || internal_trigger[_TURNRIGHT_STAGE0]) TurnRight_Stage0::state = TurnRight_Stage0::FRAME_0;
    else break;
  case TurnRight_Stage0::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_0;
  case TurnRight_Stage0::WAIT_FRAME_0:
    if(millis() - TurnRight_Stage0::time >= 200)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_1;
    break;
  case TurnRight_Stage0::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_1;
  case TurnRight_Stage0::WAIT_FRAME_1:
    if(millis() - TurnRight_Stage0::time >= 80)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_2;
    break;
  case TurnRight_Stage0::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)180);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_2;
  case TurnRight_Stage0::WAIT_FRAME_2:
    if(millis() - TurnRight_Stage0::time >= 180)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_3;
    break;
  case TurnRight_Stage0::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_3;
  case TurnRight_Stage0::WAIT_FRAME_3:
    if(millis() - TurnRight_Stage0::time >= 200)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_4;
    break;
  case TurnRight_Stage0::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_4;
  case TurnRight_Stage0::WAIT_FRAME_4:
    if(millis() - TurnRight_Stage0::time >= 20)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_5;
    break;
  case TurnRight_Stage0::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_5;
  case TurnRight_Stage0::WAIT_FRAME_5:
    if(millis() - TurnRight_Stage0::time >= 40)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_6;
    break;
  case TurnRight_Stage0::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_6;
  case TurnRight_Stage0::WAIT_FRAME_6:
    if(millis() - TurnRight_Stage0::time >= 40)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_7;
    break;
  case TurnRight_Stage0::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_7;
  case TurnRight_Stage0::WAIT_FRAME_7:
    if(millis() - TurnRight_Stage0::time >= 60)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_8;
    break;
  case TurnRight_Stage0::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_8;
  case TurnRight_Stage0::WAIT_FRAME_8:
    if(millis() - TurnRight_Stage0::time >= 80)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_9;
    break;
  case TurnRight_Stage0::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_9;
  case TurnRight_Stage0::WAIT_FRAME_9:
    if(millis() - TurnRight_Stage0::time >= 80)
      TurnRight_Stage0::state = TurnRight_Stage0::FRAME_10;
    break;
  case TurnRight_Stage0::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage0_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage0_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage0_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage0::time = millis();
    TurnRight_Stage0::state = TurnRight_Stage0::WAIT_FRAME_10;
  case TurnRight_Stage0::WAIT_FRAME_10:
    if(millis() - TurnRight_Stage0::time >= 60)
    {
      TurnRight_Stage0::state = TurnRight_Stage0::IDLE;
      internal_trigger[_TURNRIGHT_STAGE0] = false;
      external_trigger[_TURNRIGHT_STAGE0] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnRight_Stage0(int times)
{
  _86ME_cmd[3] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[3] = false;
}
void Robot86ME::TurnRight_Stage1Update()
{
  switch(TurnRight_Stage1::state)
  {
  case TurnRight_Stage1::IDLE:
    if(external_trigger[_TURNRIGHT_STAGE1] || internal_trigger[_TURNRIGHT_STAGE1]) TurnRight_Stage1::state = TurnRight_Stage1::FRAME_0;
    else break;
  case TurnRight_Stage1::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_0;
  case TurnRight_Stage1::WAIT_FRAME_0:
    if(millis() - TurnRight_Stage1::time >= 20)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_1;
    break;
  case TurnRight_Stage1::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_1;
  case TurnRight_Stage1::WAIT_FRAME_1:
    if(millis() - TurnRight_Stage1::time >= 40)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_2;
    break;
  case TurnRight_Stage1::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_2;
  case TurnRight_Stage1::WAIT_FRAME_2:
    if(millis() - TurnRight_Stage1::time >= 40)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_3;
    break;
  case TurnRight_Stage1::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_3;
  case TurnRight_Stage1::WAIT_FRAME_3:
    if(millis() - TurnRight_Stage1::time >= 60)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_4;
    break;
  case TurnRight_Stage1::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_4;
  case TurnRight_Stage1::WAIT_FRAME_4:
    if(millis() - TurnRight_Stage1::time >= 80)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_5;
    break;
  case TurnRight_Stage1::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_5;
  case TurnRight_Stage1::WAIT_FRAME_5:
    if(millis() - TurnRight_Stage1::time >= 80)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_6;
    break;
  case TurnRight_Stage1::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_6;
  case TurnRight_Stage1::WAIT_FRAME_6:
    if(millis() - TurnRight_Stage1::time >= 60)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_7;
    break;
  case TurnRight_Stage1::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_7;
  case TurnRight_Stage1::WAIT_FRAME_7:
    if(millis() - TurnRight_Stage1::time >= 20)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_8;
    break;
  case TurnRight_Stage1::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_8;
  case TurnRight_Stage1::WAIT_FRAME_8:
    if(millis() - TurnRight_Stage1::time >= 40)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_9;
    break;
  case TurnRight_Stage1::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_9;
  case TurnRight_Stage1::WAIT_FRAME_9:
    if(millis() - TurnRight_Stage1::time >= 40)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_10;
    break;
  case TurnRight_Stage1::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_10;
  case TurnRight_Stage1::WAIT_FRAME_10:
    if(millis() - TurnRight_Stage1::time >= 60)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_11;
    break;
  case TurnRight_Stage1::FRAME_11:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[11].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[11].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[11].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_11;
  case TurnRight_Stage1::WAIT_FRAME_11:
    if(millis() - TurnRight_Stage1::time >= 80)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_12;
    break;
  case TurnRight_Stage1::FRAME_12:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[12].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[12].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[12].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_12;
  case TurnRight_Stage1::WAIT_FRAME_12:
    if(millis() - TurnRight_Stage1::time >= 80)
      TurnRight_Stage1::state = TurnRight_Stage1::FRAME_13;
    break;
  case TurnRight_Stage1::FRAME_13:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage1_frm[13].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage1_frm[13].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage1_frm[13].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage1::time = millis();
    TurnRight_Stage1::state = TurnRight_Stage1::WAIT_FRAME_13;
  case TurnRight_Stage1::WAIT_FRAME_13:
    if(millis() - TurnRight_Stage1::time >= 60)
    {
      TurnRight_Stage1::state = TurnRight_Stage1::IDLE;
      internal_trigger[_TURNRIGHT_STAGE1] = false;
      external_trigger[_TURNRIGHT_STAGE1] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnRight_Stage1(int times)
{
  _86ME_cmd[4] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[4] = false;
}
void Robot86ME::TurnRight_Stage2Update()
{
  switch(TurnRight_Stage2::state)
  {
  case TurnRight_Stage2::IDLE:
    if(external_trigger[_TURNRIGHT_STAGE2] || internal_trigger[_TURNRIGHT_STAGE2]) TurnRight_Stage2::state = TurnRight_Stage2::FRAME_0;
    else break;
  case TurnRight_Stage2::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_0;
  case TurnRight_Stage2::WAIT_FRAME_0:
    if(millis() - TurnRight_Stage2::time >= 20)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_1;
    break;
  case TurnRight_Stage2::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_1;
  case TurnRight_Stage2::WAIT_FRAME_1:
    if(millis() - TurnRight_Stage2::time >= 40)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_2;
    break;
  case TurnRight_Stage2::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_2;
  case TurnRight_Stage2::WAIT_FRAME_2:
    if(millis() - TurnRight_Stage2::time >= 40)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_3;
    break;
  case TurnRight_Stage2::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_3;
  case TurnRight_Stage2::WAIT_FRAME_3:
    if(millis() - TurnRight_Stage2::time >= 60)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_4;
    break;
  case TurnRight_Stage2::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_4;
  case TurnRight_Stage2::WAIT_FRAME_4:
    if(millis() - TurnRight_Stage2::time >= 80)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_5;
    break;
  case TurnRight_Stage2::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_5;
  case TurnRight_Stage2::WAIT_FRAME_5:
    if(millis() - TurnRight_Stage2::time >= 80)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_6;
    break;
  case TurnRight_Stage2::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_6;
  case TurnRight_Stage2::WAIT_FRAME_6:
    if(millis() - TurnRight_Stage2::time >= 60)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_7;
    break;
  case TurnRight_Stage2::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_7;
  case TurnRight_Stage2::WAIT_FRAME_7:
    if(millis() - TurnRight_Stage2::time >= 20)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_8;
    break;
  case TurnRight_Stage2::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_8;
  case TurnRight_Stage2::WAIT_FRAME_8:
    if(millis() - TurnRight_Stage2::time >= 60)
      TurnRight_Stage2::state = TurnRight_Stage2::FRAME_9;
    break;
  case TurnRight_Stage2::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = TurnRight_Stage2_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = TurnRight_Stage2_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = TurnRight_Stage2_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    TurnRight_Stage2::time = millis();
    TurnRight_Stage2::state = TurnRight_Stage2::WAIT_FRAME_9;
  case TurnRight_Stage2::WAIT_FRAME_9:
    if(millis() - TurnRight_Stage2::time >= 60)
    {
      TurnRight_Stage2::state = TurnRight_Stage2::IDLE;
      internal_trigger[_TURNRIGHT_STAGE2] = false;
      external_trigger[_TURNRIGHT_STAGE2] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnRight_Stage2(int times)
{
  _86ME_cmd[5] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[5] = false;
}
void Robot86ME::WalkBwd_Stage0Update()
{
  switch(WalkBwd_Stage0::state)
  {
  case WalkBwd_Stage0::IDLE:
    if(external_trigger[_WALKBWD_STAGE0] || internal_trigger[_WALKBWD_STAGE0]) WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_0;
    else break;
  case WalkBwd_Stage0::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_0;
  case WalkBwd_Stage0::WAIT_FRAME_0:
    if(millis() - WalkBwd_Stage0::time >= 200)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_1;
    break;
  case WalkBwd_Stage0::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_1;
  case WalkBwd_Stage0::WAIT_FRAME_1:
    if(millis() - WalkBwd_Stage0::time >= 80)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_2;
    break;
  case WalkBwd_Stage0::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)180);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_2;
  case WalkBwd_Stage0::WAIT_FRAME_2:
    if(millis() - WalkBwd_Stage0::time >= 180)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_3;
    break;
  case WalkBwd_Stage0::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_3;
  case WalkBwd_Stage0::WAIT_FRAME_3:
    if(millis() - WalkBwd_Stage0::time >= 200)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_4;
    break;
  case WalkBwd_Stage0::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_4;
  case WalkBwd_Stage0::WAIT_FRAME_4:
    if(millis() - WalkBwd_Stage0::time >= 20)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_5;
    break;
  case WalkBwd_Stage0::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_5;
  case WalkBwd_Stage0::WAIT_FRAME_5:
    if(millis() - WalkBwd_Stage0::time >= 40)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_6;
    break;
  case WalkBwd_Stage0::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_6;
  case WalkBwd_Stage0::WAIT_FRAME_6:
    if(millis() - WalkBwd_Stage0::time >= 40)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_7;
    break;
  case WalkBwd_Stage0::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_7;
  case WalkBwd_Stage0::WAIT_FRAME_7:
    if(millis() - WalkBwd_Stage0::time >= 60)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_8;
    break;
  case WalkBwd_Stage0::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_8;
  case WalkBwd_Stage0::WAIT_FRAME_8:
    if(millis() - WalkBwd_Stage0::time >= 80)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_9;
    break;
  case WalkBwd_Stage0::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_9;
  case WalkBwd_Stage0::WAIT_FRAME_9:
    if(millis() - WalkBwd_Stage0::time >= 80)
      WalkBwd_Stage0::state = WalkBwd_Stage0::FRAME_10;
    break;
  case WalkBwd_Stage0::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage0_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage0_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage0_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage0::time = millis();
    WalkBwd_Stage0::state = WalkBwd_Stage0::WAIT_FRAME_10;
  case WalkBwd_Stage0::WAIT_FRAME_10:
    if(millis() - WalkBwd_Stage0::time >= 60)
    {
      WalkBwd_Stage0::state = WalkBwd_Stage0::IDLE;
      internal_trigger[_WALKBWD_STAGE0] = false;
      external_trigger[_WALKBWD_STAGE0] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkBwd_Stage0(int times)
{
  _86ME_cmd[6] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[6] = false;
}
void Robot86ME::WalkBwd_Stage1Update()
{
  switch(WalkBwd_Stage1::state)
  {
  case WalkBwd_Stage1::IDLE:
    if(external_trigger[_WALKBWD_STAGE1] || internal_trigger[_WALKBWD_STAGE1]) WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_0;
    else break;
  case WalkBwd_Stage1::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_0;
  case WalkBwd_Stage1::WAIT_FRAME_0:
    if(millis() - WalkBwd_Stage1::time >= 20)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_1;
    break;
  case WalkBwd_Stage1::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_1;
  case WalkBwd_Stage1::WAIT_FRAME_1:
    if(millis() - WalkBwd_Stage1::time >= 40)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_2;
    break;
  case WalkBwd_Stage1::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_2;
  case WalkBwd_Stage1::WAIT_FRAME_2:
    if(millis() - WalkBwd_Stage1::time >= 40)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_3;
    break;
  case WalkBwd_Stage1::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_3;
  case WalkBwd_Stage1::WAIT_FRAME_3:
    if(millis() - WalkBwd_Stage1::time >= 60)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_4;
    break;
  case WalkBwd_Stage1::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_4;
  case WalkBwd_Stage1::WAIT_FRAME_4:
    if(millis() - WalkBwd_Stage1::time >= 80)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_5;
    break;
  case WalkBwd_Stage1::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_5;
  case WalkBwd_Stage1::WAIT_FRAME_5:
    if(millis() - WalkBwd_Stage1::time >= 80)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_6;
    break;
  case WalkBwd_Stage1::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_6;
  case WalkBwd_Stage1::WAIT_FRAME_6:
    if(millis() - WalkBwd_Stage1::time >= 60)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_7;
    break;
  case WalkBwd_Stage1::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_7;
  case WalkBwd_Stage1::WAIT_FRAME_7:
    if(millis() - WalkBwd_Stage1::time >= 20)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_8;
    break;
  case WalkBwd_Stage1::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_8;
  case WalkBwd_Stage1::WAIT_FRAME_8:
    if(millis() - WalkBwd_Stage1::time >= 40)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_9;
    break;
  case WalkBwd_Stage1::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_9;
  case WalkBwd_Stage1::WAIT_FRAME_9:
    if(millis() - WalkBwd_Stage1::time >= 40)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_10;
    break;
  case WalkBwd_Stage1::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_10;
  case WalkBwd_Stage1::WAIT_FRAME_10:
    if(millis() - WalkBwd_Stage1::time >= 60)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_11;
    break;
  case WalkBwd_Stage1::FRAME_11:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[11].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[11].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[11].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_11;
  case WalkBwd_Stage1::WAIT_FRAME_11:
    if(millis() - WalkBwd_Stage1::time >= 80)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_12;
    break;
  case WalkBwd_Stage1::FRAME_12:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[12].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[12].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[12].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_12;
  case WalkBwd_Stage1::WAIT_FRAME_12:
    if(millis() - WalkBwd_Stage1::time >= 80)
      WalkBwd_Stage1::state = WalkBwd_Stage1::FRAME_13;
    break;
  case WalkBwd_Stage1::FRAME_13:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage1_frm[13].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage1_frm[13].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage1_frm[13].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage1::time = millis();
    WalkBwd_Stage1::state = WalkBwd_Stage1::WAIT_FRAME_13;
  case WalkBwd_Stage1::WAIT_FRAME_13:
    if(millis() - WalkBwd_Stage1::time >= 60)
    {
      WalkBwd_Stage1::state = WalkBwd_Stage1::IDLE;
      internal_trigger[_WALKBWD_STAGE1] = false;
      external_trigger[_WALKBWD_STAGE1] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkBwd_Stage1(int times)
{
  _86ME_cmd[7] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[7] = false;
}
void Robot86ME::WalkBwd_Stage2Update()
{
  switch(WalkBwd_Stage2::state)
  {
  case WalkBwd_Stage2::IDLE:
    if(external_trigger[_WALKBWD_STAGE2] || internal_trigger[_WALKBWD_STAGE2]) WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_0;
    else break;
  case WalkBwd_Stage2::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_0;
  case WalkBwd_Stage2::WAIT_FRAME_0:
    if(millis() - WalkBwd_Stage2::time >= 20)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_1;
    break;
  case WalkBwd_Stage2::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_1;
  case WalkBwd_Stage2::WAIT_FRAME_1:
    if(millis() - WalkBwd_Stage2::time >= 40)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_2;
    break;
  case WalkBwd_Stage2::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_2;
  case WalkBwd_Stage2::WAIT_FRAME_2:
    if(millis() - WalkBwd_Stage2::time >= 40)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_3;
    break;
  case WalkBwd_Stage2::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_3;
  case WalkBwd_Stage2::WAIT_FRAME_3:
    if(millis() - WalkBwd_Stage2::time >= 60)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_4;
    break;
  case WalkBwd_Stage2::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_4;
  case WalkBwd_Stage2::WAIT_FRAME_4:
    if(millis() - WalkBwd_Stage2::time >= 80)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_5;
    break;
  case WalkBwd_Stage2::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_5;
  case WalkBwd_Stage2::WAIT_FRAME_5:
    if(millis() - WalkBwd_Stage2::time >= 80)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_6;
    break;
  case WalkBwd_Stage2::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_6;
  case WalkBwd_Stage2::WAIT_FRAME_6:
    if(millis() - WalkBwd_Stage2::time >= 60)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_7;
    break;
  case WalkBwd_Stage2::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_7;
  case WalkBwd_Stage2::WAIT_FRAME_7:
    if(millis() - WalkBwd_Stage2::time >= 20)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_8;
    break;
  case WalkBwd_Stage2::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_8;
  case WalkBwd_Stage2::WAIT_FRAME_8:
    if(millis() - WalkBwd_Stage2::time >= 60)
      WalkBwd_Stage2::state = WalkBwd_Stage2::FRAME_9;
    break;
  case WalkBwd_Stage2::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkBwd_Stage2_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkBwd_Stage2_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkBwd_Stage2_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkBwd_Stage2::time = millis();
    WalkBwd_Stage2::state = WalkBwd_Stage2::WAIT_FRAME_9;
  case WalkBwd_Stage2::WAIT_FRAME_9:
    if(millis() - WalkBwd_Stage2::time >= 60)
    {
      WalkBwd_Stage2::state = WalkBwd_Stage2::IDLE;
      internal_trigger[_WALKBWD_STAGE2] = false;
      external_trigger[_WALKBWD_STAGE2] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkBwd_Stage2(int times)
{
  _86ME_cmd[8] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[8] = false;
}
void Robot86ME::WalkFwd_Stage0Update()
{
  switch(WalkFwd_Stage0::state)
  {
  case WalkFwd_Stage0::IDLE:
    if(external_trigger[_WALKFWD_STAGE0] || internal_trigger[_WALKFWD_STAGE0]) WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_0;
    else break;
  case WalkFwd_Stage0::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_0;
  case WalkFwd_Stage0::WAIT_FRAME_0:
    if(millis() - WalkFwd_Stage0::time >= 200)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_1;
    break;
  case WalkFwd_Stage0::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_1;
  case WalkFwd_Stage0::WAIT_FRAME_1:
    if(millis() - WalkFwd_Stage0::time >= 80)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_2;
    break;
  case WalkFwd_Stage0::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)180);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_2;
  case WalkFwd_Stage0::WAIT_FRAME_2:
    if(millis() - WalkFwd_Stage0::time >= 180)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_3;
    break;
  case WalkFwd_Stage0::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_3;
  case WalkFwd_Stage0::WAIT_FRAME_3:
    if(millis() - WalkFwd_Stage0::time >= 200)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_4;
    break;
  case WalkFwd_Stage0::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_4;
  case WalkFwd_Stage0::WAIT_FRAME_4:
    if(millis() - WalkFwd_Stage0::time >= 20)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_5;
    break;
  case WalkFwd_Stage0::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_5;
  case WalkFwd_Stage0::WAIT_FRAME_5:
    if(millis() - WalkFwd_Stage0::time >= 40)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_6;
    break;
  case WalkFwd_Stage0::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_6;
  case WalkFwd_Stage0::WAIT_FRAME_6:
    if(millis() - WalkFwd_Stage0::time >= 40)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_7;
    break;
  case WalkFwd_Stage0::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_7;
  case WalkFwd_Stage0::WAIT_FRAME_7:
    if(millis() - WalkFwd_Stage0::time >= 60)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_8;
    break;
  case WalkFwd_Stage0::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_8;
  case WalkFwd_Stage0::WAIT_FRAME_8:
    if(millis() - WalkFwd_Stage0::time >= 80)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_9;
    break;
  case WalkFwd_Stage0::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_9;
  case WalkFwd_Stage0::WAIT_FRAME_9:
    if(millis() - WalkFwd_Stage0::time >= 80)
      WalkFwd_Stage0::state = WalkFwd_Stage0::FRAME_10;
    break;
  case WalkFwd_Stage0::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage0_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage0_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage0_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage0::time = millis();
    WalkFwd_Stage0::state = WalkFwd_Stage0::WAIT_FRAME_10;
  case WalkFwd_Stage0::WAIT_FRAME_10:
    if(millis() - WalkFwd_Stage0::time >= 60)
    {
      WalkFwd_Stage0::state = WalkFwd_Stage0::IDLE;
      internal_trigger[_WALKFWD_STAGE0] = false;
      external_trigger[_WALKFWD_STAGE0] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkFwd_Stage0(int times)
{
  _86ME_cmd[9] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[9] = false;
}
void Robot86ME::WalkFwd_Stage1Update()
{
  switch(WalkFwd_Stage1::state)
  {
  case WalkFwd_Stage1::IDLE:
    if(external_trigger[_WALKFWD_STAGE1] || internal_trigger[_WALKFWD_STAGE1]) WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_0;
    else break;
  case WalkFwd_Stage1::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_0;
  case WalkFwd_Stage1::WAIT_FRAME_0:
    if(millis() - WalkFwd_Stage1::time >= 20)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_1;
    break;
  case WalkFwd_Stage1::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_1;
  case WalkFwd_Stage1::WAIT_FRAME_1:
    if(millis() - WalkFwd_Stage1::time >= 40)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_2;
    break;
  case WalkFwd_Stage1::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_2;
  case WalkFwd_Stage1::WAIT_FRAME_2:
    if(millis() - WalkFwd_Stage1::time >= 40)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_3;
    break;
  case WalkFwd_Stage1::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_3;
  case WalkFwd_Stage1::WAIT_FRAME_3:
    if(millis() - WalkFwd_Stage1::time >= 60)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_4;
    break;
  case WalkFwd_Stage1::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_4;
  case WalkFwd_Stage1::WAIT_FRAME_4:
    if(millis() - WalkFwd_Stage1::time >= 80)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_5;
    break;
  case WalkFwd_Stage1::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_5;
  case WalkFwd_Stage1::WAIT_FRAME_5:
    if(millis() - WalkFwd_Stage1::time >= 80)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_6;
    break;
  case WalkFwd_Stage1::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_6;
  case WalkFwd_Stage1::WAIT_FRAME_6:
    if(millis() - WalkFwd_Stage1::time >= 60)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_7;
    break;
  case WalkFwd_Stage1::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_7;
  case WalkFwd_Stage1::WAIT_FRAME_7:
    if(millis() - WalkFwd_Stage1::time >= 20)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_8;
    break;
  case WalkFwd_Stage1::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_8;
  case WalkFwd_Stage1::WAIT_FRAME_8:
    if(millis() - WalkFwd_Stage1::time >= 40)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_9;
    break;
  case WalkFwd_Stage1::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_9;
  case WalkFwd_Stage1::WAIT_FRAME_9:
    if(millis() - WalkFwd_Stage1::time >= 40)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_10;
    break;
  case WalkFwd_Stage1::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_10;
  case WalkFwd_Stage1::WAIT_FRAME_10:
    if(millis() - WalkFwd_Stage1::time >= 60)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_11;
    break;
  case WalkFwd_Stage1::FRAME_11:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[11].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[11].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[11].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_11;
  case WalkFwd_Stage1::WAIT_FRAME_11:
    if(millis() - WalkFwd_Stage1::time >= 80)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_12;
    break;
  case WalkFwd_Stage1::FRAME_12:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[12].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[12].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[12].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_12;
  case WalkFwd_Stage1::WAIT_FRAME_12:
    if(millis() - WalkFwd_Stage1::time >= 80)
      WalkFwd_Stage1::state = WalkFwd_Stage1::FRAME_13;
    break;
  case WalkFwd_Stage1::FRAME_13:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage1_frm[13].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage1_frm[13].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage1_frm[13].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage1::time = millis();
    WalkFwd_Stage1::state = WalkFwd_Stage1::WAIT_FRAME_13;
  case WalkFwd_Stage1::WAIT_FRAME_13:
    if(millis() - WalkFwd_Stage1::time >= 60)
    {
      WalkFwd_Stage1::state = WalkFwd_Stage1::IDLE;
      internal_trigger[_WALKFWD_STAGE1] = false;
      external_trigger[_WALKFWD_STAGE1] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkFwd_Stage1(int times)
{
  _86ME_cmd[10] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[10] = false;
}
void Robot86ME::WalkFwd_Stage2Update()
{
  switch(WalkFwd_Stage2::state)
  {
  case WalkFwd_Stage2::IDLE:
    if(external_trigger[_WALKFWD_STAGE2] || internal_trigger[_WALKFWD_STAGE2]) WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_0;
    else break;
  case WalkFwd_Stage2::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_0;
  case WalkFwd_Stage2::WAIT_FRAME_0:
    if(millis() - WalkFwd_Stage2::time >= 20)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_1;
    break;
  case WalkFwd_Stage2::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_1;
  case WalkFwd_Stage2::WAIT_FRAME_1:
    if(millis() - WalkFwd_Stage2::time >= 40)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_2;
    break;
  case WalkFwd_Stage2::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_2;
  case WalkFwd_Stage2::WAIT_FRAME_2:
    if(millis() - WalkFwd_Stage2::time >= 40)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_3;
    break;
  case WalkFwd_Stage2::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_3;
  case WalkFwd_Stage2::WAIT_FRAME_3:
    if(millis() - WalkFwd_Stage2::time >= 60)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_4;
    break;
  case WalkFwd_Stage2::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_4;
  case WalkFwd_Stage2::WAIT_FRAME_4:
    if(millis() - WalkFwd_Stage2::time >= 80)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_5;
    break;
  case WalkFwd_Stage2::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_5;
  case WalkFwd_Stage2::WAIT_FRAME_5:
    if(millis() - WalkFwd_Stage2::time >= 80)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_6;
    break;
  case WalkFwd_Stage2::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_6;
  case WalkFwd_Stage2::WAIT_FRAME_6:
    if(millis() - WalkFwd_Stage2::time >= 60)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_7;
    break;
  case WalkFwd_Stage2::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_7;
  case WalkFwd_Stage2::WAIT_FRAME_7:
    if(millis() - WalkFwd_Stage2::time >= 20)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_8;
    break;
  case WalkFwd_Stage2::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_8;
  case WalkFwd_Stage2::WAIT_FRAME_8:
    if(millis() - WalkFwd_Stage2::time >= 60)
      WalkFwd_Stage2::state = WalkFwd_Stage2::FRAME_9;
    break;
  case WalkFwd_Stage2::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkFwd_Stage2_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkFwd_Stage2_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkFwd_Stage2_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkFwd_Stage2::time = millis();
    WalkFwd_Stage2::state = WalkFwd_Stage2::WAIT_FRAME_9;
  case WalkFwd_Stage2::WAIT_FRAME_9:
    if(millis() - WalkFwd_Stage2::time >= 60)
    {
      WalkFwd_Stage2::state = WalkFwd_Stage2::IDLE;
      internal_trigger[_WALKFWD_STAGE2] = false;
      external_trigger[_WALKFWD_STAGE2] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkFwd_Stage2(int times)
{
  _86ME_cmd[11] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[11] = false;
}
void Robot86ME::WalkLeft_Stage0Update()
{
  switch(WalkLeft_Stage0::state)
  {
  case WalkLeft_Stage0::IDLE:
    if(external_trigger[_WALKLEFT_STAGE0] || internal_trigger[_WALKLEFT_STAGE0]) WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_0;
    else break;
  case WalkLeft_Stage0::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_0;
  case WalkLeft_Stage0::WAIT_FRAME_0:
    if(millis() - WalkLeft_Stage0::time >= 200)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_1;
    break;
  case WalkLeft_Stage0::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_1;
  case WalkLeft_Stage0::WAIT_FRAME_1:
    if(millis() - WalkLeft_Stage0::time >= 80)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_2;
    break;
  case WalkLeft_Stage0::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)180);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_2;
  case WalkLeft_Stage0::WAIT_FRAME_2:
    if(millis() - WalkLeft_Stage0::time >= 180)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_3;
    break;
  case WalkLeft_Stage0::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_3;
  case WalkLeft_Stage0::WAIT_FRAME_3:
    if(millis() - WalkLeft_Stage0::time >= 200)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_4;
    break;
  case WalkLeft_Stage0::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_4;
  case WalkLeft_Stage0::WAIT_FRAME_4:
    if(millis() - WalkLeft_Stage0::time >= 20)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_5;
    break;
  case WalkLeft_Stage0::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_5;
  case WalkLeft_Stage0::WAIT_FRAME_5:
    if(millis() - WalkLeft_Stage0::time >= 40)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_6;
    break;
  case WalkLeft_Stage0::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_6;
  case WalkLeft_Stage0::WAIT_FRAME_6:
    if(millis() - WalkLeft_Stage0::time >= 40)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_7;
    break;
  case WalkLeft_Stage0::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_7;
  case WalkLeft_Stage0::WAIT_FRAME_7:
    if(millis() - WalkLeft_Stage0::time >= 60)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_8;
    break;
  case WalkLeft_Stage0::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_8;
  case WalkLeft_Stage0::WAIT_FRAME_8:
    if(millis() - WalkLeft_Stage0::time >= 80)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_9;
    break;
  case WalkLeft_Stage0::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_9;
  case WalkLeft_Stage0::WAIT_FRAME_9:
    if(millis() - WalkLeft_Stage0::time >= 80)
      WalkLeft_Stage0::state = WalkLeft_Stage0::FRAME_10;
    break;
  case WalkLeft_Stage0::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage0_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage0_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage0_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage0::time = millis();
    WalkLeft_Stage0::state = WalkLeft_Stage0::WAIT_FRAME_10;
  case WalkLeft_Stage0::WAIT_FRAME_10:
    if(millis() - WalkLeft_Stage0::time >= 60)
    {
      WalkLeft_Stage0::state = WalkLeft_Stage0::IDLE;
      internal_trigger[_WALKLEFT_STAGE0] = false;
      external_trigger[_WALKLEFT_STAGE0] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkLeft_Stage0(int times)
{
  _86ME_cmd[12] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[12] = false;
}
void Robot86ME::WalkLeft_Stage1Update()
{
  switch(WalkLeft_Stage1::state)
  {
  case WalkLeft_Stage1::IDLE:
    if(external_trigger[_WALKLEFT_STAGE1] || internal_trigger[_WALKLEFT_STAGE1]) WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_0;
    else break;
  case WalkLeft_Stage1::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_0;
  case WalkLeft_Stage1::WAIT_FRAME_0:
    if(millis() - WalkLeft_Stage1::time >= 20)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_1;
    break;
  case WalkLeft_Stage1::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_1;
  case WalkLeft_Stage1::WAIT_FRAME_1:
    if(millis() - WalkLeft_Stage1::time >= 40)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_2;
    break;
  case WalkLeft_Stage1::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_2;
  case WalkLeft_Stage1::WAIT_FRAME_2:
    if(millis() - WalkLeft_Stage1::time >= 40)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_3;
    break;
  case WalkLeft_Stage1::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_3;
  case WalkLeft_Stage1::WAIT_FRAME_3:
    if(millis() - WalkLeft_Stage1::time >= 60)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_4;
    break;
  case WalkLeft_Stage1::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_4;
  case WalkLeft_Stage1::WAIT_FRAME_4:
    if(millis() - WalkLeft_Stage1::time >= 80)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_5;
    break;
  case WalkLeft_Stage1::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_5;
  case WalkLeft_Stage1::WAIT_FRAME_5:
    if(millis() - WalkLeft_Stage1::time >= 80)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_6;
    break;
  case WalkLeft_Stage1::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_6;
  case WalkLeft_Stage1::WAIT_FRAME_6:
    if(millis() - WalkLeft_Stage1::time >= 60)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_7;
    break;
  case WalkLeft_Stage1::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_7;
  case WalkLeft_Stage1::WAIT_FRAME_7:
    if(millis() - WalkLeft_Stage1::time >= 20)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_8;
    break;
  case WalkLeft_Stage1::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_8;
  case WalkLeft_Stage1::WAIT_FRAME_8:
    if(millis() - WalkLeft_Stage1::time >= 40)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_9;
    break;
  case WalkLeft_Stage1::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_9;
  case WalkLeft_Stage1::WAIT_FRAME_9:
    if(millis() - WalkLeft_Stage1::time >= 40)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_10;
    break;
  case WalkLeft_Stage1::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_10;
  case WalkLeft_Stage1::WAIT_FRAME_10:
    if(millis() - WalkLeft_Stage1::time >= 60)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_11;
    break;
  case WalkLeft_Stage1::FRAME_11:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[11].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[11].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[11].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_11;
  case WalkLeft_Stage1::WAIT_FRAME_11:
    if(millis() - WalkLeft_Stage1::time >= 80)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_12;
    break;
  case WalkLeft_Stage1::FRAME_12:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[12].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[12].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[12].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_12;
  case WalkLeft_Stage1::WAIT_FRAME_12:
    if(millis() - WalkLeft_Stage1::time >= 80)
      WalkLeft_Stage1::state = WalkLeft_Stage1::FRAME_13;
    break;
  case WalkLeft_Stage1::FRAME_13:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage1_frm[13].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage1_frm[13].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage1_frm[13].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage1::time = millis();
    WalkLeft_Stage1::state = WalkLeft_Stage1::WAIT_FRAME_13;
  case WalkLeft_Stage1::WAIT_FRAME_13:
    if(millis() - WalkLeft_Stage1::time >= 60)
    {
      WalkLeft_Stage1::state = WalkLeft_Stage1::IDLE;
      internal_trigger[_WALKLEFT_STAGE1] = false;
      external_trigger[_WALKLEFT_STAGE1] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkLeft_Stage1(int times)
{
  _86ME_cmd[13] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[13] = false;
}
void Robot86ME::WalkLeft_Stage2Update()
{
  switch(WalkLeft_Stage2::state)
  {
  case WalkLeft_Stage2::IDLE:
    if(external_trigger[_WALKLEFT_STAGE2] || internal_trigger[_WALKLEFT_STAGE2]) WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_0;
    else break;
  case WalkLeft_Stage2::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_0;
  case WalkLeft_Stage2::WAIT_FRAME_0:
    if(millis() - WalkLeft_Stage2::time >= 20)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_1;
    break;
  case WalkLeft_Stage2::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_1;
  case WalkLeft_Stage2::WAIT_FRAME_1:
    if(millis() - WalkLeft_Stage2::time >= 40)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_2;
    break;
  case WalkLeft_Stage2::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_2;
  case WalkLeft_Stage2::WAIT_FRAME_2:
    if(millis() - WalkLeft_Stage2::time >= 40)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_3;
    break;
  case WalkLeft_Stage2::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_3;
  case WalkLeft_Stage2::WAIT_FRAME_3:
    if(millis() - WalkLeft_Stage2::time >= 60)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_4;
    break;
  case WalkLeft_Stage2::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_4;
  case WalkLeft_Stage2::WAIT_FRAME_4:
    if(millis() - WalkLeft_Stage2::time >= 80)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_5;
    break;
  case WalkLeft_Stage2::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_5;
  case WalkLeft_Stage2::WAIT_FRAME_5:
    if(millis() - WalkLeft_Stage2::time >= 80)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_6;
    break;
  case WalkLeft_Stage2::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_6;
  case WalkLeft_Stage2::WAIT_FRAME_6:
    if(millis() - WalkLeft_Stage2::time >= 60)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_7;
    break;
  case WalkLeft_Stage2::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_7;
  case WalkLeft_Stage2::WAIT_FRAME_7:
    if(millis() - WalkLeft_Stage2::time >= 20)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_8;
    break;
  case WalkLeft_Stage2::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_8;
  case WalkLeft_Stage2::WAIT_FRAME_8:
    if(millis() - WalkLeft_Stage2::time >= 60)
      WalkLeft_Stage2::state = WalkLeft_Stage2::FRAME_9;
    break;
  case WalkLeft_Stage2::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkLeft_Stage2_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkLeft_Stage2_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkLeft_Stage2_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkLeft_Stage2::time = millis();
    WalkLeft_Stage2::state = WalkLeft_Stage2::WAIT_FRAME_9;
  case WalkLeft_Stage2::WAIT_FRAME_9:
    if(millis() - WalkLeft_Stage2::time >= 60)
    {
      WalkLeft_Stage2::state = WalkLeft_Stage2::IDLE;
      internal_trigger[_WALKLEFT_STAGE2] = false;
      external_trigger[_WALKLEFT_STAGE2] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkLeft_Stage2(int times)
{
  _86ME_cmd[14] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[14] = false;
}
void Robot86ME::WalkRight_Stage0Update()
{
  switch(WalkRight_Stage0::state)
  {
  case WalkRight_Stage0::IDLE:
    if(external_trigger[_WALKRIGHT_STAGE0] || internal_trigger[_WALKRIGHT_STAGE0]) WalkRight_Stage0::state = WalkRight_Stage0::FRAME_0;
    else break;
  case WalkRight_Stage0::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_0;
  case WalkRight_Stage0::WAIT_FRAME_0:
    if(millis() - WalkRight_Stage0::time >= 200)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_1;
    break;
  case WalkRight_Stage0::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_1;
  case WalkRight_Stage0::WAIT_FRAME_1:
    if(millis() - WalkRight_Stage0::time >= 80)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_2;
    break;
  case WalkRight_Stage0::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)180);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_2;
  case WalkRight_Stage0::WAIT_FRAME_2:
    if(millis() - WalkRight_Stage0::time >= 180)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_3;
    break;
  case WalkRight_Stage0::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)200);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_3;
  case WalkRight_Stage0::WAIT_FRAME_3:
    if(millis() - WalkRight_Stage0::time >= 200)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_4;
    break;
  case WalkRight_Stage0::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_4;
  case WalkRight_Stage0::WAIT_FRAME_4:
    if(millis() - WalkRight_Stage0::time >= 20)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_5;
    break;
  case WalkRight_Stage0::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_5;
  case WalkRight_Stage0::WAIT_FRAME_5:
    if(millis() - WalkRight_Stage0::time >= 40)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_6;
    break;
  case WalkRight_Stage0::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_6;
  case WalkRight_Stage0::WAIT_FRAME_6:
    if(millis() - WalkRight_Stage0::time >= 40)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_7;
    break;
  case WalkRight_Stage0::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_7;
  case WalkRight_Stage0::WAIT_FRAME_7:
    if(millis() - WalkRight_Stage0::time >= 60)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_8;
    break;
  case WalkRight_Stage0::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_8;
  case WalkRight_Stage0::WAIT_FRAME_8:
    if(millis() - WalkRight_Stage0::time >= 80)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_9;
    break;
  case WalkRight_Stage0::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_9;
  case WalkRight_Stage0::WAIT_FRAME_9:
    if(millis() - WalkRight_Stage0::time >= 80)
      WalkRight_Stage0::state = WalkRight_Stage0::FRAME_10;
    break;
  case WalkRight_Stage0::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage0_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage0_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage0_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage0::time = millis();
    WalkRight_Stage0::state = WalkRight_Stage0::WAIT_FRAME_10;
  case WalkRight_Stage0::WAIT_FRAME_10:
    if(millis() - WalkRight_Stage0::time >= 60)
    {
      WalkRight_Stage0::state = WalkRight_Stage0::IDLE;
      internal_trigger[_WALKRIGHT_STAGE0] = false;
      external_trigger[_WALKRIGHT_STAGE0] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkRight_Stage0(int times)
{
  _86ME_cmd[15] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[15] = false;
}
void Robot86ME::WalkRight_Stage1Update()
{
  switch(WalkRight_Stage1::state)
  {
  case WalkRight_Stage1::IDLE:
    if(external_trigger[_WALKRIGHT_STAGE1] || internal_trigger[_WALKRIGHT_STAGE1]) WalkRight_Stage1::state = WalkRight_Stage1::FRAME_0;
    else break;
  case WalkRight_Stage1::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_0;
  case WalkRight_Stage1::WAIT_FRAME_0:
    if(millis() - WalkRight_Stage1::time >= 20)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_1;
    break;
  case WalkRight_Stage1::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_1;
  case WalkRight_Stage1::WAIT_FRAME_1:
    if(millis() - WalkRight_Stage1::time >= 40)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_2;
    break;
  case WalkRight_Stage1::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_2;
  case WalkRight_Stage1::WAIT_FRAME_2:
    if(millis() - WalkRight_Stage1::time >= 40)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_3;
    break;
  case WalkRight_Stage1::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_3;
  case WalkRight_Stage1::WAIT_FRAME_3:
    if(millis() - WalkRight_Stage1::time >= 60)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_4;
    break;
  case WalkRight_Stage1::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_4;
  case WalkRight_Stage1::WAIT_FRAME_4:
    if(millis() - WalkRight_Stage1::time >= 80)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_5;
    break;
  case WalkRight_Stage1::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_5;
  case WalkRight_Stage1::WAIT_FRAME_5:
    if(millis() - WalkRight_Stage1::time >= 80)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_6;
    break;
  case WalkRight_Stage1::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_6;
  case WalkRight_Stage1::WAIT_FRAME_6:
    if(millis() - WalkRight_Stage1::time >= 60)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_7;
    break;
  case WalkRight_Stage1::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_7;
  case WalkRight_Stage1::WAIT_FRAME_7:
    if(millis() - WalkRight_Stage1::time >= 20)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_8;
    break;
  case WalkRight_Stage1::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_8;
  case WalkRight_Stage1::WAIT_FRAME_8:
    if(millis() - WalkRight_Stage1::time >= 40)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_9;
    break;
  case WalkRight_Stage1::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_9;
  case WalkRight_Stage1::WAIT_FRAME_9:
    if(millis() - WalkRight_Stage1::time >= 40)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_10;
    break;
  case WalkRight_Stage1::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_10;
  case WalkRight_Stage1::WAIT_FRAME_10:
    if(millis() - WalkRight_Stage1::time >= 60)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_11;
    break;
  case WalkRight_Stage1::FRAME_11:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[11].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[11].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[11].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_11;
  case WalkRight_Stage1::WAIT_FRAME_11:
    if(millis() - WalkRight_Stage1::time >= 80)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_12;
    break;
  case WalkRight_Stage1::FRAME_12:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[12].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[12].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[12].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_12;
  case WalkRight_Stage1::WAIT_FRAME_12:
    if(millis() - WalkRight_Stage1::time >= 80)
      WalkRight_Stage1::state = WalkRight_Stage1::FRAME_13;
    break;
  case WalkRight_Stage1::FRAME_13:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage1_frm[13].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage1_frm[13].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage1_frm[13].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage1::time = millis();
    WalkRight_Stage1::state = WalkRight_Stage1::WAIT_FRAME_13;
  case WalkRight_Stage1::WAIT_FRAME_13:
    if(millis() - WalkRight_Stage1::time >= 60)
    {
      WalkRight_Stage1::state = WalkRight_Stage1::IDLE;
      internal_trigger[_WALKRIGHT_STAGE1] = false;
      external_trigger[_WALKRIGHT_STAGE1] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkRight_Stage1(int times)
{
  _86ME_cmd[16] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[16] = false;
}
void Robot86ME::WalkRight_Stage2Update()
{
  switch(WalkRight_Stage2::state)
  {
  case WalkRight_Stage2::IDLE:
    if(external_trigger[_WALKRIGHT_STAGE2] || internal_trigger[_WALKRIGHT_STAGE2]) WalkRight_Stage2::state = WalkRight_Stage2::FRAME_0;
    else break;
  case WalkRight_Stage2::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_0;
  case WalkRight_Stage2::WAIT_FRAME_0:
    if(millis() - WalkRight_Stage2::time >= 20)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_1;
    break;
  case WalkRight_Stage2::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_1;
  case WalkRight_Stage2::WAIT_FRAME_1:
    if(millis() - WalkRight_Stage2::time >= 40)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_2;
    break;
  case WalkRight_Stage2::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)40);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_2;
  case WalkRight_Stage2::WAIT_FRAME_2:
    if(millis() - WalkRight_Stage2::time >= 40)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_3;
    break;
  case WalkRight_Stage2::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_3;
  case WalkRight_Stage2::WAIT_FRAME_3:
    if(millis() - WalkRight_Stage2::time >= 60)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_4;
    break;
  case WalkRight_Stage2::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_4;
  case WalkRight_Stage2::WAIT_FRAME_4:
    if(millis() - WalkRight_Stage2::time >= 80)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_5;
    break;
  case WalkRight_Stage2::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)80);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_5;
  case WalkRight_Stage2::WAIT_FRAME_5:
    if(millis() - WalkRight_Stage2::time >= 80)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_6;
    break;
  case WalkRight_Stage2::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_6;
  case WalkRight_Stage2::WAIT_FRAME_6:
    if(millis() - WalkRight_Stage2::time >= 60)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_7;
    break;
  case WalkRight_Stage2::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)20);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_7;
  case WalkRight_Stage2::WAIT_FRAME_7:
    if(millis() - WalkRight_Stage2::time >= 20)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_8;
    break;
  case WalkRight_Stage2::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_8;
  case WalkRight_Stage2::WAIT_FRAME_8:
    if(millis() - WalkRight_Stage2::time >= 60)
      WalkRight_Stage2::state = WalkRight_Stage2::FRAME_9;
    break;
  case WalkRight_Stage2::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = WalkRight_Stage2_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = WalkRight_Stage2_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = WalkRight_Stage2_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)60);
    WalkRight_Stage2::time = millis();
    WalkRight_Stage2::state = WalkRight_Stage2::WAIT_FRAME_9;
  case WalkRight_Stage2::WAIT_FRAME_9:
    if(millis() - WalkRight_Stage2::time >= 60)
    {
      WalkRight_Stage2::state = WalkRight_Stage2::IDLE;
      internal_trigger[_WALKRIGHT_STAGE2] = false;
      external_trigger[_WALKRIGHT_STAGE2] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkRight_Stage2(int times)
{
  _86ME_cmd[17] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[17] = false;
}
void Robot86ME::TurnLeftUpdate()
{
  switch(TurnLeft::state)
  {
  case TurnLeft::IDLE:
    if(external_trigger[_TURNLEFT] || internal_trigger[_TURNLEFT]) TurnLeft::state = TurnLeft::MOTION_0;
    else break;
  case TurnLeft::MOTION_0:
    TurnLeft::state = TurnLeft::WAIT_MOTION_0;
    internal_trigger[_TURNLEFT_STAGE0] = true;
    TurnLeft_Stage0::state = TurnLeft_Stage0::IDLE;
  case TurnLeft::WAIT_MOTION_0:
    if(!internal_trigger[_TURNLEFT_STAGE0])
      TurnLeft::state = TurnLeft::MOTION_1;
    break;
  case TurnLeft::MOTION_1:
    TurnLeft::state = TurnLeft::WAIT_MOTION_1;
    internal_trigger[_TURNLEFT_STAGE1] = true;
    TurnLeft_Stage1::state = TurnLeft_Stage1::IDLE;
  case TurnLeft::WAIT_MOTION_1:
    if(!internal_trigger[_TURNLEFT_STAGE1])
      TurnLeft::state = TurnLeft::MOTION_2;
    break;
  case TurnLeft::MOTION_2:
    TurnLeft::state = TurnLeft::WAIT_MOTION_2;
    internal_trigger[_TURNLEFT_STAGE2] = true;
    TurnLeft_Stage2::state = TurnLeft_Stage2::IDLE;
  case TurnLeft::WAIT_MOTION_2:
    if(!internal_trigger[_TURNLEFT_STAGE2])
    {
      TurnLeft::state = TurnLeft::IDLE;
      internal_trigger[_TURNLEFT] = false;
      external_trigger[_TURNLEFT] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnLeft(int times)
{
  _86ME_cmd[18] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[18] = false;
}
void Robot86ME::TurnRightUpdate()
{
  switch(TurnRight::state)
  {
  case TurnRight::IDLE:
    if(external_trigger[_TURNRIGHT] || internal_trigger[_TURNRIGHT]) TurnRight::state = TurnRight::MOTION_0;
    else break;
  case TurnRight::MOTION_0:
    TurnRight::state = TurnRight::WAIT_MOTION_0;
    internal_trigger[_TURNRIGHT_STAGE0] = true;
    TurnRight_Stage0::state = TurnRight_Stage0::IDLE;
  case TurnRight::WAIT_MOTION_0:
    if(!internal_trigger[_TURNRIGHT_STAGE0])
      TurnRight::state = TurnRight::MOTION_1;
    break;
  case TurnRight::MOTION_1:
    TurnRight::state = TurnRight::WAIT_MOTION_1;
    internal_trigger[_TURNRIGHT_STAGE1] = true;
    TurnRight_Stage1::state = TurnRight_Stage1::IDLE;
  case TurnRight::WAIT_MOTION_1:
    if(!internal_trigger[_TURNRIGHT_STAGE1])
      TurnRight::state = TurnRight::MOTION_2;
    break;
  case TurnRight::MOTION_2:
    TurnRight::state = TurnRight::WAIT_MOTION_2;
    internal_trigger[_TURNRIGHT_STAGE2] = true;
    TurnRight_Stage2::state = TurnRight_Stage2::IDLE;
  case TurnRight::WAIT_MOTION_2:
    if(!internal_trigger[_TURNRIGHT_STAGE2])
    {
      TurnRight::state = TurnRight::IDLE;
      internal_trigger[_TURNRIGHT] = false;
      external_trigger[_TURNRIGHT] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::TurnRight(int times)
{
  _86ME_cmd[19] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[19] = false;
}
void Robot86ME::WalkBwdUpdate()
{
  switch(WalkBwd::state)
  {
  case WalkBwd::IDLE:
    if(external_trigger[_WALKBWD] || internal_trigger[_WALKBWD]) WalkBwd::state = WalkBwd::MOTION_0;
    else break;
  case WalkBwd::MOTION_0:
    WalkBwd::state = WalkBwd::WAIT_MOTION_0;
    internal_trigger[_WALKBWD_STAGE0] = true;
    WalkBwd_Stage0::state = WalkBwd_Stage0::IDLE;
  case WalkBwd::WAIT_MOTION_0:
    if(!internal_trigger[_WALKBWD_STAGE0])
      WalkBwd::state = WalkBwd::MOTION_1;
    break;
  case WalkBwd::MOTION_1:
    WalkBwd::state = WalkBwd::WAIT_MOTION_1;
    internal_trigger[_WALKBWD_STAGE1] = true;
    WalkBwd_Stage1::state = WalkBwd_Stage1::IDLE;
  case WalkBwd::WAIT_MOTION_1:
    if(!internal_trigger[_WALKBWD_STAGE1])
      WalkBwd::state = WalkBwd::MOTION_2;
    break;
  case WalkBwd::MOTION_2:
    WalkBwd::state = WalkBwd::WAIT_MOTION_2;
    internal_trigger[_WALKBWD_STAGE2] = true;
    WalkBwd_Stage2::state = WalkBwd_Stage2::IDLE;
  case WalkBwd::WAIT_MOTION_2:
    if(!internal_trigger[_WALKBWD_STAGE2])
    {
      WalkBwd::state = WalkBwd::IDLE;
      internal_trigger[_WALKBWD] = false;
      external_trigger[_WALKBWD] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkBwd(int times)
{
  _86ME_cmd[20] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[20] = false;
}
void Robot86ME::WalkFwdUpdate()
{
  switch(WalkFwd::state)
  {
  case WalkFwd::IDLE:
    if(external_trigger[_WALKFWD] || internal_trigger[_WALKFWD]) WalkFwd::state = WalkFwd::MOTION_0;
    else break;
  case WalkFwd::MOTION_0:
    WalkFwd::state = WalkFwd::WAIT_MOTION_0;
    internal_trigger[_WALKFWD_STAGE0] = true;
    WalkFwd_Stage0::state = WalkFwd_Stage0::IDLE;
  case WalkFwd::WAIT_MOTION_0:
    if(!internal_trigger[_WALKFWD_STAGE0])
      WalkFwd::state = WalkFwd::MOTION_1;
    break;
  case WalkFwd::MOTION_1:
    WalkFwd::state = WalkFwd::WAIT_MOTION_1;
    internal_trigger[_WALKFWD_STAGE1] = true;
    WalkFwd_Stage1::state = WalkFwd_Stage1::IDLE;
  case WalkFwd::WAIT_MOTION_1:
    if(!internal_trigger[_WALKFWD_STAGE1])
      WalkFwd::state = WalkFwd::MOTION_2;
    break;
  case WalkFwd::MOTION_2:
    WalkFwd::state = WalkFwd::WAIT_MOTION_2;
    internal_trigger[_WALKFWD_STAGE2] = true;
    WalkFwd_Stage2::state = WalkFwd_Stage2::IDLE;
  case WalkFwd::WAIT_MOTION_2:
    if(!internal_trigger[_WALKFWD_STAGE2])
    {
      WalkFwd::state = WalkFwd::IDLE;
      internal_trigger[_WALKFWD] = false;
      external_trigger[_WALKFWD] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkFwd(int times)
{
  _86ME_cmd[21] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[21] = false;
}
void Robot86ME::WalkLeftUpdate()
{
  switch(WalkLeft::state)
  {
  case WalkLeft::IDLE:
    if(external_trigger[_WALKLEFT] || internal_trigger[_WALKLEFT]) WalkLeft::state = WalkLeft::MOTION_0;
    else break;
  case WalkLeft::MOTION_0:
    WalkLeft::state = WalkLeft::WAIT_MOTION_0;
    internal_trigger[_WALKLEFT_STAGE0] = true;
    WalkLeft_Stage0::state = WalkLeft_Stage0::IDLE;
  case WalkLeft::WAIT_MOTION_0:
    if(!internal_trigger[_WALKLEFT_STAGE0])
      WalkLeft::state = WalkLeft::MOTION_1;
    break;
  case WalkLeft::MOTION_1:
    WalkLeft::state = WalkLeft::WAIT_MOTION_1;
    internal_trigger[_WALKLEFT_STAGE1] = true;
    WalkLeft_Stage1::state = WalkLeft_Stage1::IDLE;
  case WalkLeft::WAIT_MOTION_1:
    if(!internal_trigger[_WALKLEFT_STAGE1])
      WalkLeft::state = WalkLeft::MOTION_2;
    break;
  case WalkLeft::MOTION_2:
    WalkLeft::state = WalkLeft::WAIT_MOTION_2;
    internal_trigger[_WALKLEFT_STAGE2] = true;
    WalkLeft_Stage2::state = WalkLeft_Stage2::IDLE;
  case WalkLeft::WAIT_MOTION_2:
    if(!internal_trigger[_WALKLEFT_STAGE2])
    {
      WalkLeft::state = WalkLeft::IDLE;
      internal_trigger[_WALKLEFT] = false;
      external_trigger[_WALKLEFT] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkLeft(int times)
{
  _86ME_cmd[22] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[22] = false;
}
void Robot86ME::WalkRightUpdate()
{
  switch(WalkRight::state)
  {
  case WalkRight::IDLE:
    if(external_trigger[_WALKRIGHT] || internal_trigger[_WALKRIGHT]) WalkRight::state = WalkRight::MOTION_0;
    else break;
  case WalkRight::MOTION_0:
    WalkRight::state = WalkRight::WAIT_MOTION_0;
    internal_trigger[_WALKRIGHT_STAGE0] = true;
    WalkRight_Stage0::state = WalkRight_Stage0::IDLE;
  case WalkRight::WAIT_MOTION_0:
    if(!internal_trigger[_WALKRIGHT_STAGE0])
      WalkRight::state = WalkRight::MOTION_1;
    break;
  case WalkRight::MOTION_1:
    WalkRight::state = WalkRight::WAIT_MOTION_1;
    internal_trigger[_WALKRIGHT_STAGE1] = true;
    WalkRight_Stage1::state = WalkRight_Stage1::IDLE;
  case WalkRight::WAIT_MOTION_1:
    if(!internal_trigger[_WALKRIGHT_STAGE1])
      WalkRight::state = WalkRight::MOTION_2;
    break;
  case WalkRight::MOTION_2:
    WalkRight::state = WalkRight::WAIT_MOTION_2;
    internal_trigger[_WALKRIGHT_STAGE2] = true;
    WalkRight_Stage2::state = WalkRight_Stage2::IDLE;
  case WalkRight::WAIT_MOTION_2:
    if(!internal_trigger[_WALKRIGHT_STAGE2])
    {
      WalkRight::state = WalkRight::IDLE;
      internal_trigger[_WALKRIGHT] = false;
      external_trigger[_WALKRIGHT] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::WalkRight(int times)
{
  _86ME_cmd[23] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[23] = false;
}
void Robot86ME::helloworldUpdate()
{
  switch(helloworld::state)
  {
  case helloworld::IDLE:
    if(external_trigger[_HELLOWORLD] || internal_trigger[_HELLOWORLD]) helloworld::state = helloworld::FRAME_0;
    else break;
  case helloworld::FRAME_0:
    for(int i = 20; i-- > 0; )
      _86ME_RUN.positions[i] = helloworld_frm[0].positions[i] & servo_mask[i];
    _86ME_RUN.playPositions((unsigned long)350);
    helloworld::time = millis();
    helloworld::state = helloworld::WAIT_FRAME_0;
  case helloworld::WAIT_FRAME_0:
    if(millis() - helloworld::time >= 350)
      helloworld::state = helloworld::FRAME_1;
    break;
  case helloworld::FRAME_1:
    for(int i = 20; i-- > 0; )
      _86ME_RUN.positions[i] = helloworld_frm[1].positions[i] & servo_mask[i];
    _86ME_RUN.playPositions((unsigned long)350);
    helloworld::time = millis();
    helloworld::state = helloworld::WAIT_FRAME_1;
  case helloworld::WAIT_FRAME_1:
    if(millis() - helloworld::time >= 350)
      helloworld::state = helloworld::FRAME_2;
    break;
  case helloworld::FRAME_2:
    for(int i = 20; i-- > 0; )
      _86ME_RUN.positions[i] = helloworld_frm[2].positions[i] & servo_mask[i];
    _86ME_RUN.playPositions((unsigned long)350);
    helloworld::time = millis();
    helloworld::state = helloworld::WAIT_FRAME_2;
  case helloworld::WAIT_FRAME_2:
    if(millis() - helloworld::time >= 350)
      helloworld::state = helloworld::FRAME_3;
    break;
  case helloworld::FRAME_3:
    for(int i = 20; i-- > 0; )
      _86ME_RUN.positions[i] = helloworld_frm[3].positions[i] & servo_mask[i];
    _86ME_RUN.playPositions((unsigned long)350);
    helloworld::time = millis();
    helloworld::state = helloworld::WAIT_FRAME_3;
  case helloworld::WAIT_FRAME_3:
    if(millis() - helloworld::time >= 350)
      helloworld::state = helloworld::FRAME_4;
    break;
  case helloworld::FRAME_4:
    for(int i = 20; i-- > 0; )
      _86ME_RUN.positions[i] = helloworld_frm[4].positions[i] & servo_mask[i];
    _86ME_RUN.playPositions((unsigned long)600);
    helloworld::time = millis();
    helloworld::state = helloworld::WAIT_FRAME_4;
  case helloworld::WAIT_FRAME_4:
    if(millis() - helloworld::time >= 600)
    {
      helloworld::state = helloworld::IDLE;
      internal_trigger[_HELLOWORLD] = false;
      external_trigger[_HELLOWORLD] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::helloworld(int times)
{
  _86ME_cmd[24] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[24] = false;
}
void Robot86ME::bowUpdate()
{
  switch(bow::state)
  {
  case bow::IDLE:
    if(external_trigger[_BOW] || internal_trigger[_BOW]) bow::state = bow::FRAME_0;
    else break;
  case bow::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = bow_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = bow_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = bow_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)500);
    bow::time = millis();
    bow::state = bow::WAIT_FRAME_0;
  case bow::WAIT_FRAME_0:
    if(millis() - bow::time >= 500)
      bow::state = bow::FRAME_1;
    break;
  case bow::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = bow_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = bow_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = bow_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)500);
    bow::time = millis();
    bow::state = bow::WAIT_FRAME_1;
  case bow::WAIT_FRAME_1:
    if(millis() - bow::time >= 500)
      bow::state = bow::FRAME_2;
    break;
  case bow::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = bow_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = bow_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = bow_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)700);
    bow::time = millis();
    bow::state = bow::WAIT_FRAME_2;
  case bow::WAIT_FRAME_2:
    if(millis() - bow::time >= 700)
      bow::state = bow::FRAME_3;
    break;
  case bow::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = bow_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = bow_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = bow_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)700);
    bow::time = millis();
    bow::state = bow::WAIT_FRAME_3;
  case bow::WAIT_FRAME_3:
    if(millis() - bow::time >= 700)
    {
      bow::state = bow::IDLE;
      internal_trigger[_BOW] = false;
      external_trigger[_BOW] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::bow(int times)
{
  _86ME_cmd[25] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[25] = false;
}
void Robot86ME::grabUpdate()
{
  switch(grab::state)
  {
  case grab::IDLE:
    if(external_trigger[_GRAB] || internal_trigger[_GRAB]) grab::state = grab::FRAME_0;
    else break;
  case grab::FRAME_0:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[0].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[0].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[0].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)500);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_0;
  case grab::WAIT_FRAME_0:
    if(millis() - grab::time >= 500)
      grab::state = grab::FRAME_1;
    break;
  case grab::FRAME_1:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[1].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[1].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[1].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)500);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_1;
  case grab::WAIT_FRAME_1:
    if(millis() - grab::time >= 500)
      grab::state = grab::FRAME_2;
    break;
  case grab::FRAME_2:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[2].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[2].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[2].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)500);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_2;
  case grab::WAIT_FRAME_2:
    if(millis() - grab::time >= 500)
      grab::state = grab::FRAME_3;
    break;
  case grab::FRAME_3:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[3].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[3].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[3].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_3;
  case grab::WAIT_FRAME_3:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_4;
    break;
  case grab::FRAME_4:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[4].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[4].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[4].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_4;
  case grab::WAIT_FRAME_4:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_5;
    break;
  case grab::FRAME_5:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[5].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[5].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[5].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_5;
  case grab::WAIT_FRAME_5:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_6;
    break;
  case grab::FRAME_6:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[6].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[6].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[6].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_6;
  case grab::WAIT_FRAME_6:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_7;
    break;
  case grab::FRAME_7:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[7].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[7].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[7].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)500);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_7;
  case grab::WAIT_FRAME_7:
    if(millis() - grab::time >= 500)
      grab::state = grab::FRAME_8;
    break;
  case grab::FRAME_8:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[8].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[8].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[8].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_8;
  case grab::WAIT_FRAME_8:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_9;
    break;
  case grab::FRAME_9:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[9].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[9].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[9].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_9;
  case grab::WAIT_FRAME_9:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_10;
    break;
  case grab::FRAME_10:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[10].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[10].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[10].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_10;
  case grab::WAIT_FRAME_10:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_11;
    break;
  case grab::FRAME_11:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[11].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[11].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[11].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)300);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_11;
  case grab::WAIT_FRAME_11:
    if(millis() - grab::time >= 300)
      grab::state = grab::FRAME_12;
    break;
  case grab::FRAME_12:
    for(int i = 20; i-- > 0; )
    {
      _86ME_RUN.positions[i] = grab_frm[12].positions[i] & (~servo_mask[i]);
      _86ME_RUN.accelerations[i][0] = grab_frm[12].accelerations[i][0];
      _86ME_RUN.accelerations[i][1] = grab_frm[12].accelerations[i][1];
    }
    _86ME_RUN.playPositions((unsigned long)500);
    grab::time = millis();
    grab::state = grab::WAIT_FRAME_12;
  case grab::WAIT_FRAME_12:
    if(millis() - grab::time >= 500)
    {
      grab::state = grab::IDLE;
      internal_trigger[_GRAB] = false;
      external_trigger[_GRAB] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::grab(int times)
{
  _86ME_cmd[26] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[26] = false;
}
void Robot86ME::homeUpdate()
{
  switch(home::state)
  {
  case home::IDLE:
    if(external_trigger[_HOME] || internal_trigger[_HOME]) home::state = home::FRAME_0;
    else break;
  case home::FRAME_0:
    for(int i = 20; i-- > 0; )
      _86ME_RUN.positions[i] = home_frm[0].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)1000);
    home::time = millis();
    home::state = home::WAIT_FRAME_0;
  case home::WAIT_FRAME_0:
    if(millis() - home::time >= 1000)
    {
      home::state = home::IDLE;
      internal_trigger[_HOME] = false;
      external_trigger[_HOME] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::home(int times)
{
  _86ME_cmd[27] = true;
  for(int i = 0; i < times; i++)
  {
    do
    {
      update_S();
    } while (!isNoMotion());
  }
  _86ME_cmd[27] = false;
}
void Robot86ME::reset(bool initIMU)
{
  used_servos[0].detach();
  used_servos[1].detach();
  used_servos[2].detach();
  used_servos[3].detach();
  used_servos[4].detach();
  used_servos[5].detach();
  used_servos[6].detach();
  used_servos[7].detach();
  used_servos[8].detach();
  used_servos[9].detach();
  used_servos[10].detach();
  used_servos[11].detach();
  used_servos[12].detach();
  used_servos[13].detach();
  used_servos[14].detach();
  used_servos[15].detach();
  used_servos[16].detach();
  used_servos[17].detach();
  used_servos[18].detach();
  used_servos[19].detach();
  begin(initIMU);
}
void Robot86ME::begin(bool initIMU)
{
  srand(time(NULL));
  used_servos[0].attach(1, 600, 2700);
  used_servos[1].attach(2, 600, 2700);
  used_servos[2].attach(4, 600, 2700);
  used_servos[3].attach(5, 600, 2700);
  used_servos[4].attach(6, 600, 2700);
  used_servos[5].attach(7, 600, 2700);
  used_servos[6].attach(9, 600, 2700);
  used_servos[7].attach(10, 600, 2700);
  used_servos[8].attach(11, 600, 2700);
  used_servos[9].attach(12, 600, 2700);
  used_servos[10].attach(14, 600, 2700);
  used_servos[11].attach(15, 600, 2700);
  used_servos[12].attach(17, 600, 2700);
  used_servos[13].attach(18, 600, 2700);
  used_servos[14].attach(19, 600, 2700);
  used_servos[15].attach(20, 600, 2700);
  used_servos[16].attach(22, 600, 2700);
  used_servos[17].attach(23, 600, 2700);
  used_servos[18].attach(24, 600, 2700);
  used_servos[19].attach(25, 600, 2700);

  offsets.offsets[0] = -180;
  offsets.offsets[1] = -200;
  offsets.offsets[2] = -130;
  offsets.offsets[3] = -256;
  offsets.offsets[4] = -90;
  offsets.offsets[5] = -80;
  offsets.offsets[6] = -110;
  offsets.offsets[7] = 100;
  offsets.offsets[8] = 200;
  offsets.offsets[9] = 80;
  offsets.offsets[10] = 250;
  offsets.offsets[11] = 250;
  offsets.offsets[12] = 255;
  offsets.offsets[13] = 205;
  offsets.offsets[14] = 90;
  offsets.offsets[15] = 140;
  offsets.offsets[16] = 30;
  offsets.offsets[17] = -30;
  offsets.offsets[18] = -220;
  offsets.offsets[19] = -70;

  TurnLeft_Stage0_frm[0].positions[0] = 1500;
  TurnLeft_Stage0_frm[0].positions[1] = 1470;
  TurnLeft_Stage0_frm[0].positions[2] = 1500;
  TurnLeft_Stage0_frm[0].positions[3] = 1500;
  TurnLeft_Stage0_frm[0].positions[4] = 1460;
  TurnLeft_Stage0_frm[0].positions[5] = 1317;
  TurnLeft_Stage0_frm[0].positions[6] = 1507;
  TurnLeft_Stage0_frm[0].positions[7] = 1356;
  TurnLeft_Stage0_frm[0].positions[8] = 1572;
  TurnLeft_Stage0_frm[0].positions[9] = 1507;
  TurnLeft_Stage0_frm[0].positions[10] = 1500;
  TurnLeft_Stage0_frm[0].positions[11] = 1860;
  TurnLeft_Stage0_frm[0].positions[12] = 1590;
  TurnLeft_Stage0_frm[0].positions[13] = 1500;
  TurnLeft_Stage0_frm[0].positions[14] = 1500;
  TurnLeft_Stage0_frm[0].positions[15] = 1693;
  TurnLeft_Stage0_frm[0].positions[16] = 1493;
  TurnLeft_Stage0_frm[0].positions[17] = 1644;
  TurnLeft_Stage0_frm[0].positions[18] = 1428;
  TurnLeft_Stage0_frm[0].positions[19] = 1493;

  TurnLeft_Stage0_frm[1].positions[0] = 1500;
  TurnLeft_Stage0_frm[1].positions[1] = 1470;
  TurnLeft_Stage0_frm[1].positions[2] = 1500;
  TurnLeft_Stage0_frm[1].positions[3] = 1500;
  TurnLeft_Stage0_frm[1].positions[4] = 1500;
  TurnLeft_Stage0_frm[1].positions[5] = 1317;
  TurnLeft_Stage0_frm[1].positions[6] = 1507;
  TurnLeft_Stage0_frm[1].positions[7] = 1356;
  TurnLeft_Stage0_frm[1].positions[8] = 1572;
  TurnLeft_Stage0_frm[1].positions[9] = 1507;
  TurnLeft_Stage0_frm[1].positions[10] = 1500;
  TurnLeft_Stage0_frm[1].positions[11] = 1860;
  TurnLeft_Stage0_frm[1].positions[12] = 1590;
  TurnLeft_Stage0_frm[1].positions[13] = 1500;
  TurnLeft_Stage0_frm[1].positions[14] = 1500;
  TurnLeft_Stage0_frm[1].positions[15] = 1683;
  TurnLeft_Stage0_frm[1].positions[16] = 1493;
  TurnLeft_Stage0_frm[1].positions[17] = 1644;
  TurnLeft_Stage0_frm[1].positions[18] = 1428;
  TurnLeft_Stage0_frm[1].positions[19] = 1493;

  TurnLeft_Stage0_frm[2].positions[0] = 1500;
  TurnLeft_Stage0_frm[2].positions[1] = 1470;
  TurnLeft_Stage0_frm[2].positions[2] = 1500;
  TurnLeft_Stage0_frm[2].positions[3] = 1500;
  TurnLeft_Stage0_frm[2].positions[4] = 1500;
  TurnLeft_Stage0_frm[2].positions[5] = 1318;
  TurnLeft_Stage0_frm[2].positions[6] = 1518;
  TurnLeft_Stage0_frm[2].positions[7] = 1353;
  TurnLeft_Stage0_frm[2].positions[8] = 1571;
  TurnLeft_Stage0_frm[2].positions[9] = 1518;
  TurnLeft_Stage0_frm[2].positions[10] = 1500;
  TurnLeft_Stage0_frm[2].positions[11] = 1860;
  TurnLeft_Stage0_frm[2].positions[12] = 1590;
  TurnLeft_Stage0_frm[2].positions[13] = 1500;
  TurnLeft_Stage0_frm[2].positions[14] = 1500;
  TurnLeft_Stage0_frm[2].positions[15] = 1683;
  TurnLeft_Stage0_frm[2].positions[16] = 1504;
  TurnLeft_Stage0_frm[2].positions[17] = 1645;
  TurnLeft_Stage0_frm[2].positions[18] = 1428;
  TurnLeft_Stage0_frm[2].positions[19] = 1504;

  TurnLeft_Stage0_frm[3].positions[0] = 1500;
  TurnLeft_Stage0_frm[3].positions[1] = 1470;
  TurnLeft_Stage0_frm[3].positions[2] = 1500;
  TurnLeft_Stage0_frm[3].positions[3] = 1500;
  TurnLeft_Stage0_frm[3].positions[4] = 1501;
  TurnLeft_Stage0_frm[3].positions[5] = 1311;
  TurnLeft_Stage0_frm[3].positions[6] = 1585;
  TurnLeft_Stage0_frm[3].positions[7] = 1360;
  TurnLeft_Stage0_frm[3].positions[8] = 1571;
  TurnLeft_Stage0_frm[3].positions[9] = 1585;
  TurnLeft_Stage0_frm[3].positions[10] = 1500;
  TurnLeft_Stage0_frm[3].positions[11] = 1860;
  TurnLeft_Stage0_frm[3].positions[12] = 1590;
  TurnLeft_Stage0_frm[3].positions[13] = 1500;
  TurnLeft_Stage0_frm[3].positions[14] = 1501;
  TurnLeft_Stage0_frm[3].positions[15] = 1697;
  TurnLeft_Stage0_frm[3].positions[16] = 1571;
  TurnLeft_Stage0_frm[3].positions[17] = 1624;
  TurnLeft_Stage0_frm[3].positions[18] = 1421;
  TurnLeft_Stage0_frm[3].positions[19] = 1571;

  TurnLeft_Stage0_frm[4].positions[0] = 1500;
  TurnLeft_Stage0_frm[4].positions[1] = 1470;
  TurnLeft_Stage0_frm[4].positions[2] = 1500;
  TurnLeft_Stage0_frm[4].positions[3] = 1499;
  TurnLeft_Stage0_frm[4].positions[4] = 1500;
  TurnLeft_Stage0_frm[4].positions[5] = 1314;
  TurnLeft_Stage0_frm[4].positions[6] = 1593;
  TurnLeft_Stage0_frm[4].positions[7] = 1352;
  TurnLeft_Stage0_frm[4].positions[8] = 1566;
  TurnLeft_Stage0_frm[4].positions[9] = 1593;
  TurnLeft_Stage0_frm[4].positions[10] = 1500;
  TurnLeft_Stage0_frm[4].positions[11] = 1860;
  TurnLeft_Stage0_frm[4].positions[12] = 1590;
  TurnLeft_Stage0_frm[4].positions[13] = 1499;
  TurnLeft_Stage0_frm[4].positions[14] = 1500;
  TurnLeft_Stage0_frm[4].positions[15] = 1695;
  TurnLeft_Stage0_frm[4].positions[16] = 1580;
  TurnLeft_Stage0_frm[4].positions[17] = 1630;
  TurnLeft_Stage0_frm[4].positions[18] = 1425;
  TurnLeft_Stage0_frm[4].positions[19] = 1580;

  TurnLeft_Stage0_frm[5].positions[0] = 1500;
  TurnLeft_Stage0_frm[5].positions[1] = 1470;
  TurnLeft_Stage0_frm[5].positions[2] = 1500;
  TurnLeft_Stage0_frm[5].positions[3] = 1498;
  TurnLeft_Stage0_frm[5].positions[4] = 1496;
  TurnLeft_Stage0_frm[5].positions[5] = 1332;
  TurnLeft_Stage0_frm[5].positions[6] = 1610;
  TurnLeft_Stage0_frm[5].positions[7] = 1311;
  TurnLeft_Stage0_frm[5].positions[8] = 1543;
  TurnLeft_Stage0_frm[5].positions[9] = 1610;
  TurnLeft_Stage0_frm[5].positions[10] = 1500;
  TurnLeft_Stage0_frm[5].positions[11] = 1860;
  TurnLeft_Stage0_frm[5].positions[12] = 1590;
  TurnLeft_Stage0_frm[5].positions[13] = 1498;
  TurnLeft_Stage0_frm[5].positions[14] = 1502;
  TurnLeft_Stage0_frm[5].positions[15] = 1700;
  TurnLeft_Stage0_frm[5].positions[16] = 1597;
  TurnLeft_Stage0_frm[5].positions[17] = 1617;
  TurnLeft_Stage0_frm[5].positions[18] = 1423;
  TurnLeft_Stage0_frm[5].positions[19] = 1597;

  TurnLeft_Stage0_frm[6].positions[0] = 1500;
  TurnLeft_Stage0_frm[6].positions[1] = 1470;
  TurnLeft_Stage0_frm[6].positions[2] = 1500;
  TurnLeft_Stage0_frm[6].positions[3] = 1496;
  TurnLeft_Stage0_frm[6].positions[4] = 1499;
  TurnLeft_Stage0_frm[6].positions[5] = 1355;
  TurnLeft_Stage0_frm[6].positions[6] = 1622;
  TurnLeft_Stage0_frm[6].positions[7] = 1261;
  TurnLeft_Stage0_frm[6].positions[8] = 1516;
  TurnLeft_Stage0_frm[6].positions[9] = 1622;
  TurnLeft_Stage0_frm[6].positions[10] = 1500;
  TurnLeft_Stage0_frm[6].positions[11] = 1860;
  TurnLeft_Stage0_frm[6].positions[12] = 1590;
  TurnLeft_Stage0_frm[6].positions[13] = 1496;
  TurnLeft_Stage0_frm[6].positions[14] = 1505;
  TurnLeft_Stage0_frm[6].positions[15] = 1744;
  TurnLeft_Stage0_frm[6].positions[16] = 1613;
  TurnLeft_Stage0_frm[6].positions[17] = 1531;
  TurnLeft_Stage0_frm[6].positions[18] = 1389;
  TurnLeft_Stage0_frm[6].positions[19] = 1613;

  TurnLeft_Stage0_frm[7].positions[0] = 1500;
  TurnLeft_Stage0_frm[7].positions[1] = 1470;
  TurnLeft_Stage0_frm[7].positions[2] = 1500;
  TurnLeft_Stage0_frm[7].positions[3] = 1498;
  TurnLeft_Stage0_frm[7].positions[4] = 1528;
  TurnLeft_Stage0_frm[7].positions[5] = 1364;
  TurnLeft_Stage0_frm[7].positions[6] = 1630;
  TurnLeft_Stage0_frm[7].positions[7] = 1246;
  TurnLeft_Stage0_frm[7].positions[8] = 1510;
  TurnLeft_Stage0_frm[7].positions[9] = 1630;
  TurnLeft_Stage0_frm[7].positions[10] = 1500;
  TurnLeft_Stage0_frm[7].positions[11] = 1860;
  TurnLeft_Stage0_frm[7].positions[12] = 1590;
  TurnLeft_Stage0_frm[7].positions[13] = 1498;
  TurnLeft_Stage0_frm[7].positions[14] = 1482;
  TurnLeft_Stage0_frm[7].positions[15] = 1857;
  TurnLeft_Stage0_frm[7].positions[16] = 1625;
  TurnLeft_Stage0_frm[7].positions[17] = 1342;
  TurnLeft_Stage0_frm[7].positions[18] = 1313;
  TurnLeft_Stage0_frm[7].positions[19] = 1624;

  TurnLeft_Stage0_frm[8].positions[0] = 1500;
  TurnLeft_Stage0_frm[8].positions[1] = 1470;
  TurnLeft_Stage0_frm[8].positions[2] = 1500;
  TurnLeft_Stage0_frm[8].positions[3] = 1502;
  TurnLeft_Stage0_frm[8].positions[4] = 1588;
  TurnLeft_Stage0_frm[8].positions[5] = 1352;
  TurnLeft_Stage0_frm[8].positions[6] = 1612;
  TurnLeft_Stage0_frm[8].positions[7] = 1277;
  TurnLeft_Stage0_frm[8].positions[8] = 1529;
  TurnLeft_Stage0_frm[8].positions[9] = 1612;
  TurnLeft_Stage0_frm[8].positions[10] = 1500;
  TurnLeft_Stage0_frm[8].positions[11] = 1860;
  TurnLeft_Stage0_frm[8].positions[12] = 1590;
  TurnLeft_Stage0_frm[8].positions[13] = 1502;
  TurnLeft_Stage0_frm[8].positions[14] = 1421;
  TurnLeft_Stage0_frm[8].positions[15] = 1869;
  TurnLeft_Stage0_frm[8].positions[16] = 1593;
  TurnLeft_Stage0_frm[8].positions[17] = 1363;
  TurnLeft_Stage0_frm[8].positions[18] = 1340;
  TurnLeft_Stage0_frm[8].positions[19] = 1593;

  TurnLeft_Stage0_frm[9].positions[0] = 1500;
  TurnLeft_Stage0_frm[9].positions[1] = 1470;
  TurnLeft_Stage0_frm[9].positions[2] = 1500;
  TurnLeft_Stage0_frm[9].positions[3] = 1501;
  TurnLeft_Stage0_frm[9].positions[4] = 1611;
  TurnLeft_Stage0_frm[9].positions[5] = 1339;
  TurnLeft_Stage0_frm[9].positions[6] = 1555;
  TurnLeft_Stage0_frm[9].positions[7] = 1289;
  TurnLeft_Stage0_frm[9].positions[8] = 1528;
  TurnLeft_Stage0_frm[9].positions[9] = 1555;
  TurnLeft_Stage0_frm[9].positions[10] = 1500;
  TurnLeft_Stage0_frm[9].positions[11] = 1860;
  TurnLeft_Stage0_frm[9].positions[12] = 1590;
  TurnLeft_Stage0_frm[9].positions[13] = 1501;
  TurnLeft_Stage0_frm[9].positions[14] = 1387;
  TurnLeft_Stage0_frm[9].positions[15] = 1737;
  TurnLeft_Stage0_frm[9].positions[16] = 1528;
  TurnLeft_Stage0_frm[9].positions[17] = 1597;
  TurnLeft_Stage0_frm[9].positions[18] = 1437;
  TurnLeft_Stage0_frm[9].positions[19] = 1528;

  TurnLeft_Stage0_frm[10].positions[0] = 1500;
  TurnLeft_Stage0_frm[10].positions[1] = 1470;
  TurnLeft_Stage0_frm[10].positions[2] = 1500;
  TurnLeft_Stage0_frm[10].positions[3] = 1499;
  TurnLeft_Stage0_frm[10].positions[4] = 1610;
  TurnLeft_Stage0_frm[10].positions[5] = 1325;
  TurnLeft_Stage0_frm[10].positions[6] = 1497;
  TurnLeft_Stage0_frm[10].positions[7] = 1297;
  TurnLeft_Stage0_frm[10].positions[8] = 1522;
  TurnLeft_Stage0_frm[10].positions[9] = 1497;
  TurnLeft_Stage0_frm[10].positions[10] = 1500;
  TurnLeft_Stage0_frm[10].positions[11] = 1860;
  TurnLeft_Stage0_frm[10].positions[12] = 1590;
  TurnLeft_Stage0_frm[10].positions[13] = 1499;
  TurnLeft_Stage0_frm[10].positions[14] = 1393;
  TurnLeft_Stage0_frm[10].positions[15] = 1709;
  TurnLeft_Stage0_frm[10].positions[16] = 1469;
  TurnLeft_Stage0_frm[10].positions[17] = 1623;
  TurnLeft_Stage0_frm[10].positions[18] = 1432;
  TurnLeft_Stage0_frm[10].positions[19] = 1469;

  TurnLeft_Stage1_frm[0].positions[0] = 1500;
  TurnLeft_Stage1_frm[0].positions[1] = 1470;
  TurnLeft_Stage1_frm[0].positions[2] = 1500;
  TurnLeft_Stage1_frm[0].positions[3] = 1498;
  TurnLeft_Stage1_frm[0].positions[4] = 1609;
  TurnLeft_Stage1_frm[0].positions[5] = 1324;
  TurnLeft_Stage1_frm[0].positions[6] = 1477;
  TurnLeft_Stage1_frm[0].positions[7] = 1295;
  TurnLeft_Stage1_frm[0].positions[8] = 1518;
  TurnLeft_Stage1_frm[0].positions[9] = 1477;
  TurnLeft_Stage1_frm[0].positions[10] = 1500;
  TurnLeft_Stage1_frm[0].positions[11] = 1860;
  TurnLeft_Stage1_frm[0].positions[12] = 1590;
  TurnLeft_Stage1_frm[0].positions[13] = 1498;
  TurnLeft_Stage1_frm[0].positions[14] = 1391;
  TurnLeft_Stage1_frm[0].positions[15] = 1703;
  TurnLeft_Stage1_frm[0].positions[16] = 1449;
  TurnLeft_Stage1_frm[0].positions[17] = 1625;
  TurnLeft_Stage1_frm[0].positions[18] = 1428;
  TurnLeft_Stage1_frm[0].positions[19] = 1449;

  TurnLeft_Stage1_frm[1].positions[0] = 1500;
  TurnLeft_Stage1_frm[1].positions[1] = 1470;
  TurnLeft_Stage1_frm[1].positions[2] = 1500;
  TurnLeft_Stage1_frm[1].positions[3] = 1495;
  TurnLeft_Stage1_frm[1].positions[4] = 1613;
  TurnLeft_Stage1_frm[1].positions[5] = 1275;
  TurnLeft_Stage1_frm[1].positions[6] = 1440;
  TurnLeft_Stage1_frm[1].positions[7] = 1368;
  TurnLeft_Stage1_frm[1].positions[8] = 1534;
  TurnLeft_Stage1_frm[1].positions[9] = 1440;
  TurnLeft_Stage1_frm[1].positions[10] = 1500;
  TurnLeft_Stage1_frm[1].positions[11] = 1860;
  TurnLeft_Stage1_frm[1].positions[12] = 1590;
  TurnLeft_Stage1_frm[1].positions[13] = 1495;
  TurnLeft_Stage1_frm[1].positions[14] = 1386;
  TurnLeft_Stage1_frm[1].positions[15] = 1663;
  TurnLeft_Stage1_frm[1].positions[16] = 1416;
  TurnLeft_Stage1_frm[1].positions[17] = 1679;
  TurnLeft_Stage1_frm[1].positions[18] = 1443;
  TurnLeft_Stage1_frm[1].positions[19] = 1416;

  TurnLeft_Stage1_frm[2].positions[0] = 1500;
  TurnLeft_Stage1_frm[2].positions[1] = 1470;
  TurnLeft_Stage1_frm[2].positions[2] = 1500;
  TurnLeft_Stage1_frm[2].positions[3] = 1492;
  TurnLeft_Stage1_frm[2].positions[4] = 1613;
  TurnLeft_Stage1_frm[2].positions[5] = 1212;
  TurnLeft_Stage1_frm[2].positions[6] = 1411;
  TurnLeft_Stage1_frm[2].positions[7] = 1468;
  TurnLeft_Stage1_frm[2].positions[8] = 1555;
  TurnLeft_Stage1_frm[2].positions[9] = 1411;
  TurnLeft_Stage1_frm[2].positions[10] = 1500;
  TurnLeft_Stage1_frm[2].positions[11] = 1860;
  TurnLeft_Stage1_frm[2].positions[12] = 1590;
  TurnLeft_Stage1_frm[2].positions[13] = 1492;
  TurnLeft_Stage1_frm[2].positions[14] = 1391;
  TurnLeft_Stage1_frm[2].positions[15] = 1636;
  TurnLeft_Stage1_frm[2].positions[16] = 1389;
  TurnLeft_Stage1_frm[2].positions[17] = 1715;
  TurnLeft_Stage1_frm[2].positions[18] = 1450;
  TurnLeft_Stage1_frm[2].positions[19] = 1389;

  TurnLeft_Stage1_frm[3].positions[0] = 1500;
  TurnLeft_Stage1_frm[3].positions[1] = 1470;
  TurnLeft_Stage1_frm[3].positions[2] = 1500;
  TurnLeft_Stage1_frm[3].positions[3] = 1497;
  TurnLeft_Stage1_frm[3].positions[4] = 1564;
  TurnLeft_Stage1_frm[3].positions[5] = 1135;
  TurnLeft_Stage1_frm[3].positions[6] = 1380;
  TurnLeft_Stage1_frm[3].positions[7] = 1661;
  TurnLeft_Stage1_frm[3].positions[8] = 1665;
  TurnLeft_Stage1_frm[3].positions[9] = 1381;
  TurnLeft_Stage1_frm[3].positions[10] = 1500;
  TurnLeft_Stage1_frm[3].positions[11] = 1860;
  TurnLeft_Stage1_frm[3].positions[12] = 1590;
  TurnLeft_Stage1_frm[3].positions[13] = 1497;
  TurnLeft_Stage1_frm[3].positions[14] = 1451;
  TurnLeft_Stage1_frm[3].positions[15] = 1626;
  TurnLeft_Stage1_frm[3].positions[16] = 1374;
  TurnLeft_Stage1_frm[3].positions[17] = 1733;
  TurnLeft_Stage1_frm[3].positions[18] = 1459;
  TurnLeft_Stage1_frm[3].positions[19] = 1374;

  TurnLeft_Stage1_frm[4].positions[0] = 1500;
  TurnLeft_Stage1_frm[4].positions[1] = 1470;
  TurnLeft_Stage1_frm[4].positions[2] = 1500;
  TurnLeft_Stage1_frm[4].positions[3] = 1507;
  TurnLeft_Stage1_frm[4].positions[4] = 1452;
  TurnLeft_Stage1_frm[4].positions[5] = 1201;
  TurnLeft_Stage1_frm[4].positions[6] = 1397;
  TurnLeft_Stage1_frm[4].positions[7] = 1652;
  TurnLeft_Stage1_frm[4].positions[8] = 1750;
  TurnLeft_Stage1_frm[4].positions[9] = 1396;
  TurnLeft_Stage1_frm[4].positions[10] = 1500;
  TurnLeft_Stage1_frm[4].positions[11] = 1860;
  TurnLeft_Stage1_frm[4].positions[12] = 1590;
  TurnLeft_Stage1_frm[4].positions[13] = 1507;
  TurnLeft_Stage1_frm[4].positions[14] = 1572;
  TurnLeft_Stage1_frm[4].positions[15] = 1646;
  TurnLeft_Stage1_frm[4].positions[16] = 1399;
  TurnLeft_Stage1_frm[4].positions[17] = 1712;
  TurnLeft_Stage1_frm[4].positions[18] = 1458;
  TurnLeft_Stage1_frm[4].positions[19] = 1399;

  TurnLeft_Stage1_frm[5].positions[0] = 1500;
  TurnLeft_Stage1_frm[5].positions[1] = 1470;
  TurnLeft_Stage1_frm[5].positions[2] = 1500;
  TurnLeft_Stage1_frm[5].positions[3] = 1504;
  TurnLeft_Stage1_frm[5].positions[4] = 1384;
  TurnLeft_Stage1_frm[5].positions[5] = 1322;
  TurnLeft_Stage1_frm[5].positions[6] = 1459;
  TurnLeft_Stage1_frm[5].positions[7] = 1411;
  TurnLeft_Stage1_frm[5].positions[8] = 1643;
  TurnLeft_Stage1_frm[5].positions[9] = 1459;
  TurnLeft_Stage1_frm[5].positions[10] = 1500;
  TurnLeft_Stage1_frm[5].positions[11] = 1860;
  TurnLeft_Stage1_frm[5].positions[12] = 1590;
  TurnLeft_Stage1_frm[5].positions[13] = 1504;
  TurnLeft_Stage1_frm[5].positions[14] = 1615;
  TurnLeft_Stage1_frm[5].positions[15] = 1669;
  TurnLeft_Stage1_frm[5].positions[16] = 1464;
  TurnLeft_Stage1_frm[5].positions[17] = 1653;
  TurnLeft_Stage1_frm[5].positions[18] = 1421;
  TurnLeft_Stage1_frm[5].positions[19] = 1464;

  TurnLeft_Stage1_frm[6].positions[0] = 1500;
  TurnLeft_Stage1_frm[6].positions[1] = 1470;
  TurnLeft_Stage1_frm[6].positions[2] = 1500;
  TurnLeft_Stage1_frm[6].positions[3] = 1499;
  TurnLeft_Stage1_frm[6].positions[4] = 1389;
  TurnLeft_Stage1_frm[6].positions[5] = 1330;
  TurnLeft_Stage1_frm[6].positions[6] = 1521;
  TurnLeft_Stage1_frm[6].positions[7] = 1357;
  TurnLeft_Stage1_frm[6].positions[8] = 1587;
  TurnLeft_Stage1_frm[6].positions[9] = 1521;
  TurnLeft_Stage1_frm[6].positions[10] = 1500;
  TurnLeft_Stage1_frm[6].positions[11] = 1860;
  TurnLeft_Stage1_frm[6].positions[12] = 1590;
  TurnLeft_Stage1_frm[6].positions[13] = 1499;
  TurnLeft_Stage1_frm[6].positions[14] = 1606;
  TurnLeft_Stage1_frm[6].positions[15] = 1663;
  TurnLeft_Stage1_frm[6].positions[16] = 1524;
  TurnLeft_Stage1_frm[6].positions[17] = 1639;
  TurnLeft_Stage1_frm[6].positions[18] = 1402;
  TurnLeft_Stage1_frm[6].positions[19] = 1524;

  TurnLeft_Stage1_frm[7].positions[0] = 1500;
  TurnLeft_Stage1_frm[7].positions[1] = 1470;
  TurnLeft_Stage1_frm[7].positions[2] = 1500;
  TurnLeft_Stage1_frm[7].positions[3] = 1498;
  TurnLeft_Stage1_frm[7].positions[4] = 1387;
  TurnLeft_Stage1_frm[7].positions[5] = 1331;
  TurnLeft_Stage1_frm[7].positions[6] = 1541;
  TurnLeft_Stage1_frm[7].positions[7] = 1344;
  TurnLeft_Stage1_frm[7].positions[8] = 1575;
  TurnLeft_Stage1_frm[7].positions[9] = 1541;
  TurnLeft_Stage1_frm[7].positions[10] = 1500;
  TurnLeft_Stage1_frm[7].positions[11] = 1860;
  TurnLeft_Stage1_frm[7].positions[12] = 1590;
  TurnLeft_Stage1_frm[7].positions[13] = 1498;
  TurnLeft_Stage1_frm[7].positions[14] = 1605;
  TurnLeft_Stage1_frm[7].positions[15] = 1656;
  TurnLeft_Stage1_frm[7].positions[16] = 1544;
  TurnLeft_Stage1_frm[7].positions[17] = 1647;
  TurnLeft_Stage1_frm[7].positions[18] = 1404;
  TurnLeft_Stage1_frm[7].positions[19] = 1544;

  TurnLeft_Stage1_frm[8].positions[0] = 1500;
  TurnLeft_Stage1_frm[8].positions[1] = 1470;
  TurnLeft_Stage1_frm[8].positions[2] = 1500;
  TurnLeft_Stage1_frm[8].positions[3] = 1496;
  TurnLeft_Stage1_frm[8].positions[4] = 1386;
  TurnLeft_Stage1_frm[8].positions[5] = 1337;
  TurnLeft_Stage1_frm[8].positions[6] = 1576;
  TurnLeft_Stage1_frm[8].positions[7] = 1311;
  TurnLeft_Stage1_frm[8].positions[8] = 1548;
  TurnLeft_Stage1_frm[8].positions[9] = 1576;
  TurnLeft_Stage1_frm[8].positions[10] = 1500;
  TurnLeft_Stage1_frm[8].positions[11] = 1860;
  TurnLeft_Stage1_frm[8].positions[12] = 1590;
  TurnLeft_Stage1_frm[8].positions[13] = 1496;
  TurnLeft_Stage1_frm[8].positions[14] = 1615;
  TurnLeft_Stage1_frm[8].positions[15] = 1668;
  TurnLeft_Stage1_frm[8].positions[16] = 1582;
  TurnLeft_Stage1_frm[8].positions[17] = 1612;
  TurnLeft_Stage1_frm[8].positions[18] = 1389;
  TurnLeft_Stage1_frm[8].positions[19] = 1582;

  TurnLeft_Stage1_frm[9].positions[0] = 1500;
  TurnLeft_Stage1_frm[9].positions[1] = 1470;
  TurnLeft_Stage1_frm[9].positions[2] = 1500;
  TurnLeft_Stage1_frm[9].positions[3] = 1495;
  TurnLeft_Stage1_frm[9].positions[4] = 1396;
  TurnLeft_Stage1_frm[9].positions[5] = 1343;
  TurnLeft_Stage1_frm[9].positions[6] = 1602;
  TurnLeft_Stage1_frm[9].positions[7] = 1281;
  TurnLeft_Stage1_frm[9].positions[8] = 1524;
  TurnLeft_Stage1_frm[9].positions[9] = 1602;
  TurnLeft_Stage1_frm[9].positions[10] = 1500;
  TurnLeft_Stage1_frm[9].positions[11] = 1860;
  TurnLeft_Stage1_frm[9].positions[12] = 1590;
  TurnLeft_Stage1_frm[9].positions[13] = 1495;
  TurnLeft_Stage1_frm[9].positions[14] = 1626;
  TurnLeft_Stage1_frm[9].positions[15] = 1719;
  TurnLeft_Stage1_frm[9].positions[16] = 1614;
  TurnLeft_Stage1_frm[9].positions[17] = 1503;
  TurnLeft_Stage1_frm[9].positions[18] = 1348;
  TurnLeft_Stage1_frm[9].positions[19] = 1614;

  TurnLeft_Stage1_frm[10].positions[0] = 1500;
  TurnLeft_Stage1_frm[10].positions[1] = 1470;
  TurnLeft_Stage1_frm[10].positions[2] = 1500;
  TurnLeft_Stage1_frm[10].positions[3] = 1500;
  TurnLeft_Stage1_frm[10].positions[4] = 1455;
  TurnLeft_Stage1_frm[10].positions[5] = 1356;
  TurnLeft_Stage1_frm[10].positions[6] = 1623;
  TurnLeft_Stage1_frm[10].positions[7] = 1253;
  TurnLeft_Stage1_frm[10].positions[8] = 1509;
  TurnLeft_Stage1_frm[10].positions[9] = 1623;
  TurnLeft_Stage1_frm[10].positions[10] = 1500;
  TurnLeft_Stage1_frm[10].positions[11] = 1860;
  TurnLeft_Stage1_frm[10].positions[12] = 1590;
  TurnLeft_Stage1_frm[10].positions[13] = 1500;
  TurnLeft_Stage1_frm[10].positions[14] = 1580;
  TurnLeft_Stage1_frm[10].positions[15] = 1834;
  TurnLeft_Stage1_frm[10].positions[16] = 1635;
  TurnLeft_Stage1_frm[10].positions[17] = 1335;
  TurnLeft_Stage1_frm[10].positions[18] = 1296;
  TurnLeft_Stage1_frm[10].positions[19] = 1635;

  TurnLeft_Stage1_frm[11].positions[0] = 1500;
  TurnLeft_Stage1_frm[11].positions[1] = 1470;
  TurnLeft_Stage1_frm[11].positions[2] = 1500;
  TurnLeft_Stage1_frm[11].positions[3] = 1508;
  TurnLeft_Stage1_frm[11].positions[4] = 1574;
  TurnLeft_Stage1_frm[11].positions[5] = 1354;
  TurnLeft_Stage1_frm[11].positions[6] = 1609;
  TurnLeft_Stage1_frm[11].positions[7] = 1276;
  TurnLeft_Stage1_frm[11].positions[8] = 1530;
  TurnLeft_Stage1_frm[11].positions[9] = 1609;
  TurnLeft_Stage1_frm[11].positions[10] = 1500;
  TurnLeft_Stage1_frm[11].positions[11] = 1860;
  TurnLeft_Stage1_frm[11].positions[12] = 1590;
  TurnLeft_Stage1_frm[11].positions[13] = 1508;
  TurnLeft_Stage1_frm[11].positions[14] = 1457;
  TurnLeft_Stage1_frm[11].positions[15] = 1870;
  TurnLeft_Stage1_frm[11].positions[16] = 1594;
  TurnLeft_Stage1_frm[11].positions[17] = 1368;
  TurnLeft_Stage1_frm[11].positions[18] = 1354;
  TurnLeft_Stage1_frm[11].positions[19] = 1594;

  TurnLeft_Stage1_frm[12].positions[0] = 1500;
  TurnLeft_Stage1_frm[12].positions[1] = 1470;
  TurnLeft_Stage1_frm[12].positions[2] = 1500;
  TurnLeft_Stage1_frm[12].positions[3] = 1505;
  TurnLeft_Stage1_frm[12].positions[4] = 1618;
  TurnLeft_Stage1_frm[12].positions[5] = 1331;
  TurnLeft_Stage1_frm[12].positions[6] = 1548;
  TurnLeft_Stage1_frm[12].positions[7] = 1307;
  TurnLeft_Stage1_frm[12].positions[8] = 1539;
  TurnLeft_Stage1_frm[12].positions[9] = 1548;
  TurnLeft_Stage1_frm[12].positions[10] = 1500;
  TurnLeft_Stage1_frm[12].positions[11] = 1860;
  TurnLeft_Stage1_frm[12].positions[12] = 1590;
  TurnLeft_Stage1_frm[12].positions[13] = 1505;
  TurnLeft_Stage1_frm[12].positions[14] = 1387;
  TurnLeft_Stage1_frm[12].positions[15] = 1744;
  TurnLeft_Stage1_frm[12].positions[16] = 1521;
  TurnLeft_Stage1_frm[12].positions[17] = 1591;
  TurnLeft_Stage1_frm[12].positions[18] = 1442;
  TurnLeft_Stage1_frm[12].positions[19] = 1521;

  TurnLeft_Stage1_frm[13].positions[0] = 1500;
  TurnLeft_Stage1_frm[13].positions[1] = 1470;
  TurnLeft_Stage1_frm[13].positions[2] = 1500;
  TurnLeft_Stage1_frm[13].positions[3] = 1501;
  TurnLeft_Stage1_frm[13].positions[4] = 1606;
  TurnLeft_Stage1_frm[13].positions[5] = 1303;
  TurnLeft_Stage1_frm[13].positions[6] = 1488;
  TurnLeft_Stage1_frm[13].positions[7] = 1336;
  TurnLeft_Stage1_frm[13].positions[8] = 1538;
  TurnLeft_Stage1_frm[13].positions[9] = 1488;
  TurnLeft_Stage1_frm[13].positions[10] = 1500;
  TurnLeft_Stage1_frm[13].positions[11] = 1860;
  TurnLeft_Stage1_frm[13].positions[12] = 1590;
  TurnLeft_Stage1_frm[13].positions[13] = 1501;
  TurnLeft_Stage1_frm[13].positions[14] = 1389;
  TurnLeft_Stage1_frm[13].positions[15] = 1691;
  TurnLeft_Stage1_frm[13].positions[16] = 1460;
  TurnLeft_Stage1_frm[13].positions[17] = 1661;
  TurnLeft_Stage1_frm[13].positions[18] = 1452;
  TurnLeft_Stage1_frm[13].positions[19] = 1460;

  TurnLeft_Stage2_frm[0].positions[0] = 1500;
  TurnLeft_Stage2_frm[0].positions[1] = 1470;
  TurnLeft_Stage2_frm[0].positions[2] = 1500;
  TurnLeft_Stage2_frm[0].positions[3] = 1500;
  TurnLeft_Stage2_frm[0].positions[4] = 1604;
  TurnLeft_Stage2_frm[0].positions[5] = 1303;
  TurnLeft_Stage2_frm[0].positions[6] = 1468;
  TurnLeft_Stage2_frm[0].positions[7] = 1328;
  TurnLeft_Stage2_frm[0].positions[8] = 1531;
  TurnLeft_Stage2_frm[0].positions[9] = 1468;
  TurnLeft_Stage2_frm[0].positions[10] = 1500;
  TurnLeft_Stage2_frm[0].positions[11] = 1860;
  TurnLeft_Stage2_frm[0].positions[12] = 1590;
  TurnLeft_Stage2_frm[0].positions[13] = 1500;
  TurnLeft_Stage2_frm[0].positions[14] = 1387;
  TurnLeft_Stage2_frm[0].positions[15] = 1680;
  TurnLeft_Stage2_frm[0].positions[16] = 1440;
  TurnLeft_Stage2_frm[0].positions[17] = 1673;
  TurnLeft_Stage2_frm[0].positions[18] = 1453;
  TurnLeft_Stage2_frm[0].positions[19] = 1440;

  TurnLeft_Stage2_frm[1].positions[0] = 1500;
  TurnLeft_Stage2_frm[1].positions[1] = 1470;
  TurnLeft_Stage2_frm[1].positions[2] = 1500;
  TurnLeft_Stage2_frm[1].positions[3] = 1499;
  TurnLeft_Stage2_frm[1].positions[4] = 1609;
  TurnLeft_Stage2_frm[1].positions[5] = 1288;
  TurnLeft_Stage2_frm[1].positions[6] = 1432;
  TurnLeft_Stage2_frm[1].positions[7] = 1342;
  TurnLeft_Stage2_frm[1].positions[8] = 1525;
  TurnLeft_Stage2_frm[1].positions[9] = 1432;
  TurnLeft_Stage2_frm[1].positions[10] = 1500;
  TurnLeft_Stage2_frm[1].positions[11] = 1860;
  TurnLeft_Stage2_frm[1].positions[12] = 1590;
  TurnLeft_Stage2_frm[1].positions[13] = 1499;
  TurnLeft_Stage2_frm[1].positions[14] = 1387;
  TurnLeft_Stage2_frm[1].positions[15] = 1657;
  TurnLeft_Stage2_frm[1].positions[16] = 1406;
  TurnLeft_Stage2_frm[1].positions[17] = 1702;
  TurnLeft_Stage2_frm[1].positions[18] = 1458;
  TurnLeft_Stage2_frm[1].positions[19] = 1406;

  TurnLeft_Stage2_frm[2].positions[0] = 1500;
  TurnLeft_Stage2_frm[2].positions[1] = 1470;
  TurnLeft_Stage2_frm[2].positions[2] = 1500;
  TurnLeft_Stage2_frm[2].positions[3] = 1498;
  TurnLeft_Stage2_frm[2].positions[4] = 1615;
  TurnLeft_Stage2_frm[2].positions[5] = 1227;
  TurnLeft_Stage2_frm[2].positions[6] = 1404;
  TurnLeft_Stage2_frm[2].positions[7] = 1446;
  TurnLeft_Stage2_frm[2].positions[8] = 1561;
  TurnLeft_Stage2_frm[2].positions[9] = 1404;
  TurnLeft_Stage2_frm[2].positions[10] = 1500;
  TurnLeft_Stage2_frm[2].positions[11] = 1860;
  TurnLeft_Stage2_frm[2].positions[12] = 1590;
  TurnLeft_Stage2_frm[2].positions[13] = 1498;
  TurnLeft_Stage2_frm[2].positions[14] = 1395;
  TurnLeft_Stage2_frm[2].positions[15] = 1637;
  TurnLeft_Stage2_frm[2].positions[16] = 1379;
  TurnLeft_Stage2_frm[2].positions[17] = 1730;
  TurnLeft_Stage2_frm[2].positions[18] = 1466;
  TurnLeft_Stage2_frm[2].positions[19] = 1379;

  TurnLeft_Stage2_frm[3].positions[0] = 1500;
  TurnLeft_Stage2_frm[3].positions[1] = 1470;
  TurnLeft_Stage2_frm[3].positions[2] = 1500;
  TurnLeft_Stage2_frm[3].positions[3] = 1501;
  TurnLeft_Stage2_frm[3].positions[4] = 1591;
  TurnLeft_Stage2_frm[3].positions[5] = 1129;
  TurnLeft_Stage2_frm[3].positions[6] = 1372;
  TurnLeft_Stage2_frm[3].positions[7] = 1652;
  TurnLeft_Stage2_frm[3].positions[8] = 1665;
  TurnLeft_Stage2_frm[3].positions[9] = 1372;
  TurnLeft_Stage2_frm[3].positions[10] = 1500;
  TurnLeft_Stage2_frm[3].positions[11] = 1860;
  TurnLeft_Stage2_frm[3].positions[12] = 1590;
  TurnLeft_Stage2_frm[3].positions[13] = 1501;
  TurnLeft_Stage2_frm[3].positions[14] = 1426;
  TurnLeft_Stage2_frm[3].positions[15] = 1625;
  TurnLeft_Stage2_frm[3].positions[16] = 1360;
  TurnLeft_Stage2_frm[3].positions[17] = 1749;
  TurnLeft_Stage2_frm[3].positions[18] = 1474;
  TurnLeft_Stage2_frm[3].positions[19] = 1360;

  TurnLeft_Stage2_frm[4].positions[0] = 1500;
  TurnLeft_Stage2_frm[4].positions[1] = 1470;
  TurnLeft_Stage2_frm[4].positions[2] = 1500;
  TurnLeft_Stage2_frm[4].positions[3] = 1505;
  TurnLeft_Stage2_frm[4].positions[4] = 1533;
  TurnLeft_Stage2_frm[4].positions[5] = 1167;
  TurnLeft_Stage2_frm[4].positions[6] = 1376;
  TurnLeft_Stage2_frm[4].positions[7] = 1641;
  TurnLeft_Stage2_frm[4].positions[8] = 1707;
  TurnLeft_Stage2_frm[4].positions[9] = 1376;
  TurnLeft_Stage2_frm[4].positions[10] = 1500;
  TurnLeft_Stage2_frm[4].positions[11] = 1860;
  TurnLeft_Stage2_frm[4].positions[12] = 1590;
  TurnLeft_Stage2_frm[4].positions[13] = 1505;
  TurnLeft_Stage2_frm[4].positions[14] = 1484;
  TurnLeft_Stage2_frm[4].positions[15] = 1637;
  TurnLeft_Stage2_frm[4].positions[16] = 1369;
  TurnLeft_Stage2_frm[4].positions[17] = 1738;
  TurnLeft_Stage2_frm[4].positions[18] = 1475;
  TurnLeft_Stage2_frm[4].positions[19] = 1369;

  TurnLeft_Stage2_frm[5].positions[0] = 1500;
  TurnLeft_Stage2_frm[5].positions[1] = 1470;
  TurnLeft_Stage2_frm[5].positions[2] = 1500;
  TurnLeft_Stage2_frm[5].positions[3] = 1502;
  TurnLeft_Stage2_frm[5].positions[4] = 1498;
  TurnLeft_Stage2_frm[5].positions[5] = 1304;
  TurnLeft_Stage2_frm[5].positions[6] = 1417;
  TurnLeft_Stage2_frm[5].positions[7] = 1386;
  TurnLeft_Stage2_frm[5].positions[8] = 1595;
  TurnLeft_Stage2_frm[5].positions[9] = 1417;
  TurnLeft_Stage2_frm[5].positions[10] = 1500;
  TurnLeft_Stage2_frm[5].positions[11] = 1860;
  TurnLeft_Stage2_frm[5].positions[12] = 1590;
  TurnLeft_Stage2_frm[5].positions[13] = 1502;
  TurnLeft_Stage2_frm[5].positions[14] = 1504;
  TurnLeft_Stage2_frm[5].positions[15] = 1667;
  TurnLeft_Stage2_frm[5].positions[16] = 1404;
  TurnLeft_Stage2_frm[5].positions[17] = 1682;
  TurnLeft_Stage2_frm[5].positions[18] = 1449;
  TurnLeft_Stage2_frm[5].positions[19] = 1404;

  TurnLeft_Stage2_frm[6].positions[0] = 1500;
  TurnLeft_Stage2_frm[6].positions[1] = 1470;
  TurnLeft_Stage2_frm[6].positions[2] = 1500;
  TurnLeft_Stage2_frm[6].positions[3] = 1500;
  TurnLeft_Stage2_frm[6].positions[4] = 1500;
  TurnLeft_Stage2_frm[6].positions[5] = 1311;
  TurnLeft_Stage2_frm[6].positions[6] = 1450;
  TurnLeft_Stage2_frm[6].positions[7] = 1364;
  TurnLeft_Stage2_frm[6].positions[8] = 1576;
  TurnLeft_Stage2_frm[6].positions[9] = 1450;
  TurnLeft_Stage2_frm[6].positions[10] = 1500;
  TurnLeft_Stage2_frm[6].positions[11] = 1860;
  TurnLeft_Stage2_frm[6].positions[12] = 1590;
  TurnLeft_Stage2_frm[6].positions[13] = 1500;
  TurnLeft_Stage2_frm[6].positions[14] = 1500;
  TurnLeft_Stage2_frm[6].positions[15] = 1683;
  TurnLeft_Stage2_frm[6].positions[16] = 1436;
  TurnLeft_Stage2_frm[6].positions[17] = 1648;
  TurnLeft_Stage2_frm[6].positions[18] = 1430;
  TurnLeft_Stage2_frm[6].positions[19] = 1436;

  TurnLeft_Stage2_frm[7].positions[0] = 1500;
  TurnLeft_Stage2_frm[7].positions[1] = 1470;
  TurnLeft_Stage2_frm[7].positions[2] = 1500;
  TurnLeft_Stage2_frm[7].positions[3] = 1500;
  TurnLeft_Stage2_frm[7].positions[4] = 1499;
  TurnLeft_Stage2_frm[7].positions[5] = 1311;
  TurnLeft_Stage2_frm[7].positions[6] = 1460;
  TurnLeft_Stage2_frm[7].positions[7] = 1364;
  TurnLeft_Stage2_frm[7].positions[8] = 1576;
  TurnLeft_Stage2_frm[7].positions[9] = 1460;
  TurnLeft_Stage2_frm[7].positions[10] = 1500;
  TurnLeft_Stage2_frm[7].positions[11] = 1860;
  TurnLeft_Stage2_frm[7].positions[12] = 1590;
  TurnLeft_Stage2_frm[7].positions[13] = 1500;
  TurnLeft_Stage2_frm[7].positions[14] = 1499;
  TurnLeft_Stage2_frm[7].positions[15] = 1684;
  TurnLeft_Stage2_frm[7].positions[16] = 1446;
  TurnLeft_Stage2_frm[7].positions[17] = 1645;
  TurnLeft_Stage2_frm[7].positions[18] = 1429;
  TurnLeft_Stage2_frm[7].positions[19] = 1446;

  TurnLeft_Stage2_frm[8].positions[0] = 1500;
  TurnLeft_Stage2_frm[8].positions[1] = 1470;
  TurnLeft_Stage2_frm[8].positions[2] = 1500;
  TurnLeft_Stage2_frm[8].positions[3] = 1500;
  TurnLeft_Stage2_frm[8].positions[4] = 1500;
  TurnLeft_Stage2_frm[8].positions[5] = 1316;
  TurnLeft_Stage2_frm[8].positions[6] = 1500;
  TurnLeft_Stage2_frm[8].positions[7] = 1356;
  TurnLeft_Stage2_frm[8].positions[8] = 1572;
  TurnLeft_Stage2_frm[8].positions[9] = 1500;
  TurnLeft_Stage2_frm[8].positions[10] = 1500;
  TurnLeft_Stage2_frm[8].positions[11] = 1860;
  TurnLeft_Stage2_frm[8].positions[12] = 1590;
  TurnLeft_Stage2_frm[8].positions[13] = 1500;
  TurnLeft_Stage2_frm[8].positions[14] = 1500;
  TurnLeft_Stage2_frm[8].positions[15] = 1683;
  TurnLeft_Stage2_frm[8].positions[16] = 1486;
  TurnLeft_Stage2_frm[8].positions[17] = 1646;
  TurnLeft_Stage2_frm[8].positions[18] = 1429;
  TurnLeft_Stage2_frm[8].positions[19] = 1486;

  TurnLeft_Stage2_frm[9].positions[0] = 1500;
  TurnLeft_Stage2_frm[9].positions[1] = 1470;
  TurnLeft_Stage2_frm[9].positions[2] = 1500;
  TurnLeft_Stage2_frm[9].positions[3] = 1500;
  TurnLeft_Stage2_frm[9].positions[4] = 1500;
  TurnLeft_Stage2_frm[9].positions[5] = 1316;
  TurnLeft_Stage2_frm[9].positions[6] = 1497;
  TurnLeft_Stage2_frm[9].positions[7] = 1356;
  TurnLeft_Stage2_frm[9].positions[8] = 1572;
  TurnLeft_Stage2_frm[9].positions[9] = 1497;
  TurnLeft_Stage2_frm[9].positions[10] = 1500;
  TurnLeft_Stage2_frm[9].positions[11] = 1860;
  TurnLeft_Stage2_frm[9].positions[12] = 1590;
  TurnLeft_Stage2_frm[9].positions[13] = 1500;
  TurnLeft_Stage2_frm[9].positions[14] = 1500;
  TurnLeft_Stage2_frm[9].positions[15] = 1683;
  TurnLeft_Stage2_frm[9].positions[16] = 1484;
  TurnLeft_Stage2_frm[9].positions[17] = 1646;
  TurnLeft_Stage2_frm[9].positions[18] = 1429;
  TurnLeft_Stage2_frm[9].positions[19] = 1484;

  TurnRight_Stage0_frm[0].positions[0] = 1500;
  TurnRight_Stage0_frm[0].positions[1] = 1470;
  TurnRight_Stage0_frm[0].positions[2] = 1500;
  TurnRight_Stage0_frm[0].positions[3] = 1500;
  TurnRight_Stage0_frm[0].positions[4] = 1500;
  TurnRight_Stage0_frm[0].positions[5] = 1317;
  TurnRight_Stage0_frm[0].positions[6] = 1507;
  TurnRight_Stage0_frm[0].positions[7] = 1356;
  TurnRight_Stage0_frm[0].positions[8] = 1572;
  TurnRight_Stage0_frm[0].positions[9] = 1507;
  TurnRight_Stage0_frm[0].positions[10] = 1500;
  TurnRight_Stage0_frm[0].positions[11] = 1860;
  TurnRight_Stage0_frm[0].positions[12] = 1590;
  TurnRight_Stage0_frm[0].positions[13] = 1500;
  TurnRight_Stage0_frm[0].positions[14] = 1490;
  TurnRight_Stage0_frm[0].positions[15] = 1693;
  TurnRight_Stage0_frm[0].positions[16] = 1493;
  TurnRight_Stage0_frm[0].positions[17] = 1644;
  TurnRight_Stage0_frm[0].positions[18] = 1428;
  TurnRight_Stage0_frm[0].positions[19] = 1493;

  TurnRight_Stage0_frm[1].positions[0] = 1500;
  TurnRight_Stage0_frm[1].positions[1] = 1470;
  TurnRight_Stage0_frm[1].positions[2] = 1500;
  TurnRight_Stage0_frm[1].positions[3] = 1500;
  TurnRight_Stage0_frm[1].positions[4] = 1500;
  TurnRight_Stage0_frm[1].positions[5] = 1317;
  TurnRight_Stage0_frm[1].positions[6] = 1507;
  TurnRight_Stage0_frm[1].positions[7] = 1356;
  TurnRight_Stage0_frm[1].positions[8] = 1572;
  TurnRight_Stage0_frm[1].positions[9] = 1507;
  TurnRight_Stage0_frm[1].positions[10] = 1500;
  TurnRight_Stage0_frm[1].positions[11] = 1860;
  TurnRight_Stage0_frm[1].positions[12] = 1590;
  TurnRight_Stage0_frm[1].positions[13] = 1500;
  TurnRight_Stage0_frm[1].positions[14] = 1500;
  TurnRight_Stage0_frm[1].positions[15] = 1683;
  TurnRight_Stage0_frm[1].positions[16] = 1493;
  TurnRight_Stage0_frm[1].positions[17] = 1644;
  TurnRight_Stage0_frm[1].positions[18] = 1428;
  TurnRight_Stage0_frm[1].positions[19] = 1493;

  TurnRight_Stage0_frm[2].positions[0] = 1500;
  TurnRight_Stage0_frm[2].positions[1] = 1470;
  TurnRight_Stage0_frm[2].positions[2] = 1500;
  TurnRight_Stage0_frm[2].positions[3] = 1500;
  TurnRight_Stage0_frm[2].positions[4] = 1500;
  TurnRight_Stage0_frm[2].positions[5] = 1317;
  TurnRight_Stage0_frm[2].positions[6] = 1496;
  TurnRight_Stage0_frm[2].positions[7] = 1355;
  TurnRight_Stage0_frm[2].positions[8] = 1572;
  TurnRight_Stage0_frm[2].positions[9] = 1496;
  TurnRight_Stage0_frm[2].positions[10] = 1500;
  TurnRight_Stage0_frm[2].positions[11] = 1860;
  TurnRight_Stage0_frm[2].positions[12] = 1590;
  TurnRight_Stage0_frm[2].positions[13] = 1500;
  TurnRight_Stage0_frm[2].positions[14] = 1500;
  TurnRight_Stage0_frm[2].positions[15] = 1682;
  TurnRight_Stage0_frm[2].positions[16] = 1482;
  TurnRight_Stage0_frm[2].positions[17] = 1647;
  TurnRight_Stage0_frm[2].positions[18] = 1429;
  TurnRight_Stage0_frm[2].positions[19] = 1482;

  TurnRight_Stage0_frm[3].positions[0] = 1500;
  TurnRight_Stage0_frm[3].positions[1] = 1470;
  TurnRight_Stage0_frm[3].positions[2] = 1500;
  TurnRight_Stage0_frm[3].positions[3] = 1500;
  TurnRight_Stage0_frm[3].positions[4] = 1499;
  TurnRight_Stage0_frm[3].positions[5] = 1303;
  TurnRight_Stage0_frm[3].positions[6] = 1429;
  TurnRight_Stage0_frm[3].positions[7] = 1376;
  TurnRight_Stage0_frm[3].positions[8] = 1579;
  TurnRight_Stage0_frm[3].positions[9] = 1429;
  TurnRight_Stage0_frm[3].positions[10] = 1500;
  TurnRight_Stage0_frm[3].positions[11] = 1860;
  TurnRight_Stage0_frm[3].positions[12] = 1590;
  TurnRight_Stage0_frm[3].positions[13] = 1500;
  TurnRight_Stage0_frm[3].positions[14] = 1499;
  TurnRight_Stage0_frm[3].positions[15] = 1689;
  TurnRight_Stage0_frm[3].positions[16] = 1415;
  TurnRight_Stage0_frm[3].positions[17] = 1640;
  TurnRight_Stage0_frm[3].positions[18] = 1429;
  TurnRight_Stage0_frm[3].positions[19] = 1415;

  TurnRight_Stage0_frm[4].positions[0] = 1500;
  TurnRight_Stage0_frm[4].positions[1] = 1470;
  TurnRight_Stage0_frm[4].positions[2] = 1500;
  TurnRight_Stage0_frm[4].positions[3] = 1501;
  TurnRight_Stage0_frm[4].positions[4] = 1500;
  TurnRight_Stage0_frm[4].positions[5] = 1305;
  TurnRight_Stage0_frm[4].positions[6] = 1420;
  TurnRight_Stage0_frm[4].positions[7] = 1370;
  TurnRight_Stage0_frm[4].positions[8] = 1575;
  TurnRight_Stage0_frm[4].positions[9] = 1420;
  TurnRight_Stage0_frm[4].positions[10] = 1500;
  TurnRight_Stage0_frm[4].positions[11] = 1860;
  TurnRight_Stage0_frm[4].positions[12] = 1590;
  TurnRight_Stage0_frm[4].positions[13] = 1501;
  TurnRight_Stage0_frm[4].positions[14] = 1500;
  TurnRight_Stage0_frm[4].positions[15] = 1686;
  TurnRight_Stage0_frm[4].positions[16] = 1407;
  TurnRight_Stage0_frm[4].positions[17] = 1648;
  TurnRight_Stage0_frm[4].positions[18] = 1434;
  TurnRight_Stage0_frm[4].positions[19] = 1407;

  TurnRight_Stage0_frm[5].positions[0] = 1500;
  TurnRight_Stage0_frm[5].positions[1] = 1470;
  TurnRight_Stage0_frm[5].positions[2] = 1500;
  TurnRight_Stage0_frm[5].positions[3] = 1502;
  TurnRight_Stage0_frm[5].positions[4] = 1498;
  TurnRight_Stage0_frm[5].positions[5] = 1300;
  TurnRight_Stage0_frm[5].positions[6] = 1403;
  TurnRight_Stage0_frm[5].positions[7] = 1383;
  TurnRight_Stage0_frm[5].positions[8] = 1577;
  TurnRight_Stage0_frm[5].positions[9] = 1403;
  TurnRight_Stage0_frm[5].positions[10] = 1500;
  TurnRight_Stage0_frm[5].positions[11] = 1860;
  TurnRight_Stage0_frm[5].positions[12] = 1590;
  TurnRight_Stage0_frm[5].positions[13] = 1502;
  TurnRight_Stage0_frm[5].positions[14] = 1504;
  TurnRight_Stage0_frm[5].positions[15] = 1668;
  TurnRight_Stage0_frm[5].positions[16] = 1390;
  TurnRight_Stage0_frm[5].positions[17] = 1689;
  TurnRight_Stage0_frm[5].positions[18] = 1457;
  TurnRight_Stage0_frm[5].positions[19] = 1390;

  TurnRight_Stage0_frm[6].positions[0] = 1500;
  TurnRight_Stage0_frm[6].positions[1] = 1470;
  TurnRight_Stage0_frm[6].positions[2] = 1500;
  TurnRight_Stage0_frm[6].positions[3] = 1504;
  TurnRight_Stage0_frm[6].positions[4] = 1495;
  TurnRight_Stage0_frm[6].positions[5] = 1256;
  TurnRight_Stage0_frm[6].positions[6] = 1387;
  TurnRight_Stage0_frm[6].positions[7] = 1469;
  TurnRight_Stage0_frm[6].positions[8] = 1611;
  TurnRight_Stage0_frm[6].positions[9] = 1387;
  TurnRight_Stage0_frm[6].positions[10] = 1500;
  TurnRight_Stage0_frm[6].positions[11] = 1860;
  TurnRight_Stage0_frm[6].positions[12] = 1590;
  TurnRight_Stage0_frm[6].positions[13] = 1504;
  TurnRight_Stage0_frm[6].positions[14] = 1501;
  TurnRight_Stage0_frm[6].positions[15] = 1645;
  TurnRight_Stage0_frm[6].positions[16] = 1378;
  TurnRight_Stage0_frm[6].positions[17] = 1739;
  TurnRight_Stage0_frm[6].positions[18] = 1484;
  TurnRight_Stage0_frm[6].positions[19] = 1378;

  TurnRight_Stage0_frm[7].positions[0] = 1500;
  TurnRight_Stage0_frm[7].positions[1] = 1470;
  TurnRight_Stage0_frm[7].positions[2] = 1500;
  TurnRight_Stage0_frm[7].positions[3] = 1502;
  TurnRight_Stage0_frm[7].positions[4] = 1518;
  TurnRight_Stage0_frm[7].positions[5] = 1143;
  TurnRight_Stage0_frm[7].positions[6] = 1375;
  TurnRight_Stage0_frm[7].positions[7] = 1658;
  TurnRight_Stage0_frm[7].positions[8] = 1687;
  TurnRight_Stage0_frm[7].positions[9] = 1376;
  TurnRight_Stage0_frm[7].positions[10] = 1500;
  TurnRight_Stage0_frm[7].positions[11] = 1860;
  TurnRight_Stage0_frm[7].positions[12] = 1590;
  TurnRight_Stage0_frm[7].positions[13] = 1502;
  TurnRight_Stage0_frm[7].positions[14] = 1472;
  TurnRight_Stage0_frm[7].positions[15] = 1636;
  TurnRight_Stage0_frm[7].positions[16] = 1370;
  TurnRight_Stage0_frm[7].positions[17] = 1754;
  TurnRight_Stage0_frm[7].positions[18] = 1490;
  TurnRight_Stage0_frm[7].positions[19] = 1370;

  TurnRight_Stage0_frm[8].positions[0] = 1500;
  TurnRight_Stage0_frm[8].positions[1] = 1470;
  TurnRight_Stage0_frm[8].positions[2] = 1500;
  TurnRight_Stage0_frm[8].positions[3] = 1498;
  TurnRight_Stage0_frm[8].positions[4] = 1579;
  TurnRight_Stage0_frm[8].positions[5] = 1131;
  TurnRight_Stage0_frm[8].positions[6] = 1407;
  TurnRight_Stage0_frm[8].positions[7] = 1637;
  TurnRight_Stage0_frm[8].positions[8] = 1660;
  TurnRight_Stage0_frm[8].positions[9] = 1407;
  TurnRight_Stage0_frm[8].positions[10] = 1500;
  TurnRight_Stage0_frm[8].positions[11] = 1860;
  TurnRight_Stage0_frm[8].positions[12] = 1590;
  TurnRight_Stage0_frm[8].positions[13] = 1498;
  TurnRight_Stage0_frm[8].positions[14] = 1412;
  TurnRight_Stage0_frm[8].positions[15] = 1648;
  TurnRight_Stage0_frm[8].positions[16] = 1388;
  TurnRight_Stage0_frm[8].positions[17] = 1723;
  TurnRight_Stage0_frm[8].positions[18] = 1471;
  TurnRight_Stage0_frm[8].positions[19] = 1388;

  TurnRight_Stage0_frm[9].positions[0] = 1500;
  TurnRight_Stage0_frm[9].positions[1] = 1470;
  TurnRight_Stage0_frm[9].positions[2] = 1500;
  TurnRight_Stage0_frm[9].positions[3] = 1499;
  TurnRight_Stage0_frm[9].positions[4] = 1613;
  TurnRight_Stage0_frm[9].positions[5] = 1263;
  TurnRight_Stage0_frm[9].positions[6] = 1472;
  TurnRight_Stage0_frm[9].positions[7] = 1403;
  TurnRight_Stage0_frm[9].positions[8] = 1563;
  TurnRight_Stage0_frm[9].positions[9] = 1472;
  TurnRight_Stage0_frm[9].positions[10] = 1500;
  TurnRight_Stage0_frm[9].positions[11] = 1860;
  TurnRight_Stage0_frm[9].positions[12] = 1590;
  TurnRight_Stage0_frm[9].positions[13] = 1499;
  TurnRight_Stage0_frm[9].positions[14] = 1389;
  TurnRight_Stage0_frm[9].positions[15] = 1661;
  TurnRight_Stage0_frm[9].positions[16] = 1445;
  TurnRight_Stage0_frm[9].positions[17] = 1711;
  TurnRight_Stage0_frm[9].positions[18] = 1472;
  TurnRight_Stage0_frm[9].positions[19] = 1445;

  TurnRight_Stage0_frm[10].positions[0] = 1500;
  TurnRight_Stage0_frm[10].positions[1] = 1470;
  TurnRight_Stage0_frm[10].positions[2] = 1500;
  TurnRight_Stage0_frm[10].positions[3] = 1501;
  TurnRight_Stage0_frm[10].positions[4] = 1607;
  TurnRight_Stage0_frm[10].positions[5] = 1291;
  TurnRight_Stage0_frm[10].positions[6] = 1531;
  TurnRight_Stage0_frm[10].positions[7] = 1377;
  TurnRight_Stage0_frm[10].positions[8] = 1568;
  TurnRight_Stage0_frm[10].positions[9] = 1531;
  TurnRight_Stage0_frm[10].positions[10] = 1500;
  TurnRight_Stage0_frm[10].positions[11] = 1860;
  TurnRight_Stage0_frm[10].positions[12] = 1590;
  TurnRight_Stage0_frm[10].positions[13] = 1501;
  TurnRight_Stage0_frm[10].positions[14] = 1390;
  TurnRight_Stage0_frm[10].positions[15] = 1675;
  TurnRight_Stage0_frm[10].positions[16] = 1503;
  TurnRight_Stage0_frm[10].positions[17] = 1703;
  TurnRight_Stage0_frm[10].positions[18] = 1478;
  TurnRight_Stage0_frm[10].positions[19] = 1503;

  TurnRight_Stage1_frm[0].positions[0] = 1500;
  TurnRight_Stage1_frm[0].positions[1] = 1470;
  TurnRight_Stage1_frm[0].positions[2] = 1500;
  TurnRight_Stage1_frm[0].positions[3] = 1502;
  TurnRight_Stage1_frm[0].positions[4] = 1609;
  TurnRight_Stage1_frm[0].positions[5] = 1297;
  TurnRight_Stage1_frm[0].positions[6] = 1551;
  TurnRight_Stage1_frm[0].positions[7] = 1375;
  TurnRight_Stage1_frm[0].positions[8] = 1572;
  TurnRight_Stage1_frm[0].positions[9] = 1551;
  TurnRight_Stage1_frm[0].positions[10] = 1500;
  TurnRight_Stage1_frm[0].positions[11] = 1860;
  TurnRight_Stage1_frm[0].positions[12] = 1590;
  TurnRight_Stage1_frm[0].positions[13] = 1502;
  TurnRight_Stage1_frm[0].positions[14] = 1391;
  TurnRight_Stage1_frm[0].positions[15] = 1676;
  TurnRight_Stage1_frm[0].positions[16] = 1523;
  TurnRight_Stage1_frm[0].positions[17] = 1705;
  TurnRight_Stage1_frm[0].positions[18] = 1482;
  TurnRight_Stage1_frm[0].positions[19] = 1523;

  TurnRight_Stage1_frm[1].positions[0] = 1500;
  TurnRight_Stage1_frm[1].positions[1] = 1470;
  TurnRight_Stage1_frm[1].positions[2] = 1500;
  TurnRight_Stage1_frm[1].positions[3] = 1505;
  TurnRight_Stage1_frm[1].positions[4] = 1614;
  TurnRight_Stage1_frm[1].positions[5] = 1337;
  TurnRight_Stage1_frm[1].positions[6] = 1584;
  TurnRight_Stage1_frm[1].positions[7] = 1321;
  TurnRight_Stage1_frm[1].positions[8] = 1557;
  TurnRight_Stage1_frm[1].positions[9] = 1584;
  TurnRight_Stage1_frm[1].positions[10] = 1500;
  TurnRight_Stage1_frm[1].positions[11] = 1860;
  TurnRight_Stage1_frm[1].positions[12] = 1590;
  TurnRight_Stage1_frm[1].positions[13] = 1505;
  TurnRight_Stage1_frm[1].positions[14] = 1387;
  TurnRight_Stage1_frm[1].positions[15] = 1725;
  TurnRight_Stage1_frm[1].positions[16] = 1560;
  TurnRight_Stage1_frm[1].positions[17] = 1632;
  TurnRight_Stage1_frm[1].positions[18] = 1466;
  TurnRight_Stage1_frm[1].positions[19] = 1560;

  TurnRight_Stage1_frm[2].positions[0] = 1500;
  TurnRight_Stage1_frm[2].positions[1] = 1470;
  TurnRight_Stage1_frm[2].positions[2] = 1500;
  TurnRight_Stage1_frm[2].positions[3] = 1508;
  TurnRight_Stage1_frm[2].positions[4] = 1609;
  TurnRight_Stage1_frm[2].positions[5] = 1364;
  TurnRight_Stage1_frm[2].positions[6] = 1611;
  TurnRight_Stage1_frm[2].positions[7] = 1285;
  TurnRight_Stage1_frm[2].positions[8] = 1550;
  TurnRight_Stage1_frm[2].positions[9] = 1611;
  TurnRight_Stage1_frm[2].positions[10] = 1500;
  TurnRight_Stage1_frm[2].positions[11] = 1860;
  TurnRight_Stage1_frm[2].positions[12] = 1590;
  TurnRight_Stage1_frm[2].positions[13] = 1508;
  TurnRight_Stage1_frm[2].positions[14] = 1387;
  TurnRight_Stage1_frm[2].positions[15] = 1788;
  TurnRight_Stage1_frm[2].positions[16] = 1589;
  TurnRight_Stage1_frm[2].positions[17] = 1532;
  TurnRight_Stage1_frm[2].positions[18] = 1445;
  TurnRight_Stage1_frm[2].positions[19] = 1589;

  TurnRight_Stage1_frm[3].positions[0] = 1500;
  TurnRight_Stage1_frm[3].positions[1] = 1470;
  TurnRight_Stage1_frm[3].positions[2] = 1500;
  TurnRight_Stage1_frm[3].positions[3] = 1503;
  TurnRight_Stage1_frm[3].positions[4] = 1549;
  TurnRight_Stage1_frm[3].positions[5] = 1374;
  TurnRight_Stage1_frm[3].positions[6] = 1626;
  TurnRight_Stage1_frm[3].positions[7] = 1267;
  TurnRight_Stage1_frm[3].positions[8] = 1541;
  TurnRight_Stage1_frm[3].positions[9] = 1626;
  TurnRight_Stage1_frm[3].positions[10] = 1500;
  TurnRight_Stage1_frm[3].positions[11] = 1860;
  TurnRight_Stage1_frm[3].positions[12] = 1590;
  TurnRight_Stage1_frm[3].positions[13] = 1503;
  TurnRight_Stage1_frm[3].positions[14] = 1436;
  TurnRight_Stage1_frm[3].positions[15] = 1865;
  TurnRight_Stage1_frm[3].positions[16] = 1620;
  TurnRight_Stage1_frm[3].positions[17] = 1339;
  TurnRight_Stage1_frm[3].positions[18] = 1335;
  TurnRight_Stage1_frm[3].positions[19] = 1619;

  TurnRight_Stage1_frm[4].positions[0] = 1500;
  TurnRight_Stage1_frm[4].positions[1] = 1470;
  TurnRight_Stage1_frm[4].positions[2] = 1500;
  TurnRight_Stage1_frm[4].positions[3] = 1493;
  TurnRight_Stage1_frm[4].positions[4] = 1428;
  TurnRight_Stage1_frm[4].positions[5] = 1354;
  TurnRight_Stage1_frm[4].positions[6] = 1601;
  TurnRight_Stage1_frm[4].positions[7] = 1288;
  TurnRight_Stage1_frm[4].positions[8] = 1542;
  TurnRight_Stage1_frm[4].positions[9] = 1601;
  TurnRight_Stage1_frm[4].positions[10] = 1500;
  TurnRight_Stage1_frm[4].positions[11] = 1860;
  TurnRight_Stage1_frm[4].positions[12] = 1590;
  TurnRight_Stage1_frm[4].positions[13] = 1493;
  TurnRight_Stage1_frm[4].positions[14] = 1548;
  TurnRight_Stage1_frm[4].positions[15] = 1799;
  TurnRight_Stage1_frm[4].positions[16] = 1603;
  TurnRight_Stage1_frm[4].positions[17] = 1348;
  TurnRight_Stage1_frm[4].positions[18] = 1250;
  TurnRight_Stage1_frm[4].positions[19] = 1604;

  TurnRight_Stage1_frm[5].positions[0] = 1500;
  TurnRight_Stage1_frm[5].positions[1] = 1470;
  TurnRight_Stage1_frm[5].positions[2] = 1500;
  TurnRight_Stage1_frm[5].positions[3] = 1496;
  TurnRight_Stage1_frm[5].positions[4] = 1385;
  TurnRight_Stage1_frm[5].positions[5] = 1331;
  TurnRight_Stage1_frm[5].positions[6] = 1536;
  TurnRight_Stage1_frm[5].positions[7] = 1347;
  TurnRight_Stage1_frm[5].positions[8] = 1579;
  TurnRight_Stage1_frm[5].positions[9] = 1536;
  TurnRight_Stage1_frm[5].positions[10] = 1500;
  TurnRight_Stage1_frm[5].positions[11] = 1860;
  TurnRight_Stage1_frm[5].positions[12] = 1590;
  TurnRight_Stage1_frm[5].positions[13] = 1496;
  TurnRight_Stage1_frm[5].positions[14] = 1616;
  TurnRight_Stage1_frm[5].positions[15] = 1678;
  TurnRight_Stage1_frm[5].positions[16] = 1541;
  TurnRight_Stage1_frm[5].positions[17] = 1589;
  TurnRight_Stage1_frm[5].positions[18] = 1357;
  TurnRight_Stage1_frm[5].positions[19] = 1541;

  TurnRight_Stage1_frm[6].positions[0] = 1500;
  TurnRight_Stage1_frm[6].positions[1] = 1470;
  TurnRight_Stage1_frm[6].positions[2] = 1500;
  TurnRight_Stage1_frm[6].positions[3] = 1501;
  TurnRight_Stage1_frm[6].positions[4] = 1394;
  TurnRight_Stage1_frm[6].positions[5] = 1337;
  TurnRight_Stage1_frm[6].positions[6] = 1476;
  TurnRight_Stage1_frm[6].positions[7] = 1361;
  TurnRight_Stage1_frm[6].positions[8] = 1598;
  TurnRight_Stage1_frm[6].positions[9] = 1476;
  TurnRight_Stage1_frm[6].positions[10] = 1500;
  TurnRight_Stage1_frm[6].positions[11] = 1860;
  TurnRight_Stage1_frm[6].positions[12] = 1590;
  TurnRight_Stage1_frm[6].positions[13] = 1501;
  TurnRight_Stage1_frm[6].positions[14] = 1611;
  TurnRight_Stage1_frm[6].positions[15] = 1670;
  TurnRight_Stage1_frm[6].positions[16] = 1479;
  TurnRight_Stage1_frm[6].positions[17] = 1643;
  TurnRight_Stage1_frm[6].positions[18] = 1413;
  TurnRight_Stage1_frm[6].positions[19] = 1479;

  TurnRight_Stage1_frm[7].positions[0] = 1500;
  TurnRight_Stage1_frm[7].positions[1] = 1470;
  TurnRight_Stage1_frm[7].positions[2] = 1500;
  TurnRight_Stage1_frm[7].positions[3] = 1502;
  TurnRight_Stage1_frm[7].positions[4] = 1395;
  TurnRight_Stage1_frm[7].positions[5] = 1344;
  TurnRight_Stage1_frm[7].positions[6] = 1456;
  TurnRight_Stage1_frm[7].positions[7] = 1353;
  TurnRight_Stage1_frm[7].positions[8] = 1596;
  TurnRight_Stage1_frm[7].positions[9] = 1456;
  TurnRight_Stage1_frm[7].positions[10] = 1500;
  TurnRight_Stage1_frm[7].positions[11] = 1860;
  TurnRight_Stage1_frm[7].positions[12] = 1590;
  TurnRight_Stage1_frm[7].positions[13] = 1502;
  TurnRight_Stage1_frm[7].positions[14] = 1613;
  TurnRight_Stage1_frm[7].positions[15] = 1669;
  TurnRight_Stage1_frm[7].positions[16] = 1459;
  TurnRight_Stage1_frm[7].positions[17] = 1656;
  TurnRight_Stage1_frm[7].positions[18] = 1425;
  TurnRight_Stage1_frm[7].positions[19] = 1459;

  TurnRight_Stage1_frm[8].positions[0] = 1500;
  TurnRight_Stage1_frm[8].positions[1] = 1470;
  TurnRight_Stage1_frm[8].positions[2] = 1500;
  TurnRight_Stage1_frm[8].positions[3] = 1504;
  TurnRight_Stage1_frm[8].positions[4] = 1385;
  TurnRight_Stage1_frm[8].positions[5] = 1332;
  TurnRight_Stage1_frm[8].positions[6] = 1418;
  TurnRight_Stage1_frm[8].positions[7] = 1388;
  TurnRight_Stage1_frm[8].positions[8] = 1611;
  TurnRight_Stage1_frm[8].positions[9] = 1418;
  TurnRight_Stage1_frm[8].positions[10] = 1500;
  TurnRight_Stage1_frm[8].positions[11] = 1860;
  TurnRight_Stage1_frm[8].positions[12] = 1590;
  TurnRight_Stage1_frm[8].positions[13] = 1504;
  TurnRight_Stage1_frm[8].positions[14] = 1614;
  TurnRight_Stage1_frm[8].positions[15] = 1663;
  TurnRight_Stage1_frm[8].positions[16] = 1424;
  TurnRight_Stage1_frm[8].positions[17] = 1689;
  TurnRight_Stage1_frm[8].positions[18] = 1452;
  TurnRight_Stage1_frm[8].positions[19] = 1424;

  TurnRight_Stage1_frm[9].positions[0] = 1500;
  TurnRight_Stage1_frm[9].positions[1] = 1470;
  TurnRight_Stage1_frm[9].positions[2] = 1500;
  TurnRight_Stage1_frm[9].positions[3] = 1505;
  TurnRight_Stage1_frm[9].positions[4] = 1374;
  TurnRight_Stage1_frm[9].positions[5] = 1281;
  TurnRight_Stage1_frm[9].positions[6] = 1386;
  TurnRight_Stage1_frm[9].positions[7] = 1497;
  TurnRight_Stage1_frm[9].positions[8] = 1652;
  TurnRight_Stage1_frm[9].positions[9] = 1386;
  TurnRight_Stage1_frm[9].positions[10] = 1500;
  TurnRight_Stage1_frm[9].positions[11] = 1860;
  TurnRight_Stage1_frm[9].positions[12] = 1590;
  TurnRight_Stage1_frm[9].positions[13] = 1505;
  TurnRight_Stage1_frm[9].positions[14] = 1604;
  TurnRight_Stage1_frm[9].positions[15] = 1657;
  TurnRight_Stage1_frm[9].positions[16] = 1398;
  TurnRight_Stage1_frm[9].positions[17] = 1719;
  TurnRight_Stage1_frm[9].positions[18] = 1476;
  TurnRight_Stage1_frm[9].positions[19] = 1398;

  TurnRight_Stage1_frm[10].positions[0] = 1500;
  TurnRight_Stage1_frm[10].positions[1] = 1470;
  TurnRight_Stage1_frm[10].positions[2] = 1500;
  TurnRight_Stage1_frm[10].positions[3] = 1500;
  TurnRight_Stage1_frm[10].positions[4] = 1420;
  TurnRight_Stage1_frm[10].positions[5] = 1166;
  TurnRight_Stage1_frm[10].positions[6] = 1365;
  TurnRight_Stage1_frm[10].positions[7] = 1665;
  TurnRight_Stage1_frm[10].positions[8] = 1704;
  TurnRight_Stage1_frm[10].positions[9] = 1365;
  TurnRight_Stage1_frm[10].positions[10] = 1500;
  TurnRight_Stage1_frm[10].positions[11] = 1860;
  TurnRight_Stage1_frm[10].positions[12] = 1590;
  TurnRight_Stage1_frm[10].positions[13] = 1500;
  TurnRight_Stage1_frm[10].positions[14] = 1545;
  TurnRight_Stage1_frm[10].positions[15] = 1644;
  TurnRight_Stage1_frm[10].positions[16] = 1377;
  TurnRight_Stage1_frm[10].positions[17] = 1747;
  TurnRight_Stage1_frm[10].positions[18] = 1491;
  TurnRight_Stage1_frm[10].positions[19] = 1377;

  TurnRight_Stage1_frm[11].positions[0] = 1500;
  TurnRight_Stage1_frm[11].positions[1] = 1470;
  TurnRight_Stage1_frm[11].positions[2] = 1500;
  TurnRight_Stage1_frm[11].positions[3] = 1492;
  TurnRight_Stage1_frm[11].positions[4] = 1543;
  TurnRight_Stage1_frm[11].positions[5] = 1130;
  TurnRight_Stage1_frm[11].positions[6] = 1406;
  TurnRight_Stage1_frm[11].positions[7] = 1632;
  TurnRight_Stage1_frm[11].positions[8] = 1646;
  TurnRight_Stage1_frm[11].positions[9] = 1406;
  TurnRight_Stage1_frm[11].positions[10] = 1500;
  TurnRight_Stage1_frm[11].positions[11] = 1860;
  TurnRight_Stage1_frm[11].positions[12] = 1590;
  TurnRight_Stage1_frm[11].positions[13] = 1492;
  TurnRight_Stage1_frm[11].positions[14] = 1426;
  TurnRight_Stage1_frm[11].positions[15] = 1646;
  TurnRight_Stage1_frm[11].positions[16] = 1391;
  TurnRight_Stage1_frm[11].positions[17] = 1724;
  TurnRight_Stage1_frm[11].positions[18] = 1470;
  TurnRight_Stage1_frm[11].positions[19] = 1391;

  TurnRight_Stage1_frm[12].positions[0] = 1500;
  TurnRight_Stage1_frm[12].positions[1] = 1470;
  TurnRight_Stage1_frm[12].positions[2] = 1500;
  TurnRight_Stage1_frm[12].positions[3] = 1495;
  TurnRight_Stage1_frm[12].positions[4] = 1613;
  TurnRight_Stage1_frm[12].positions[5] = 1256;
  TurnRight_Stage1_frm[12].positions[6] = 1479;
  TurnRight_Stage1_frm[12].positions[7] = 1409;
  TurnRight_Stage1_frm[12].positions[8] = 1558;
  TurnRight_Stage1_frm[12].positions[9] = 1479;
  TurnRight_Stage1_frm[12].positions[10] = 1500;
  TurnRight_Stage1_frm[12].positions[11] = 1860;
  TurnRight_Stage1_frm[12].positions[12] = 1590;
  TurnRight_Stage1_frm[12].positions[13] = 1495;
  TurnRight_Stage1_frm[12].positions[14] = 1382;
  TurnRight_Stage1_frm[12].positions[15] = 1669;
  TurnRight_Stage1_frm[12].positions[16] = 1452;
  TurnRight_Stage1_frm[12].positions[17] = 1693;
  TurnRight_Stage1_frm[12].positions[18] = 1461;
  TurnRight_Stage1_frm[12].positions[19] = 1452;

  TurnRight_Stage1_frm[13].positions[0] = 1500;
  TurnRight_Stage1_frm[13].positions[1] = 1470;
  TurnRight_Stage1_frm[13].positions[2] = 1500;
  TurnRight_Stage1_frm[13].positions[3] = 1499;
  TurnRight_Stage1_frm[13].positions[4] = 1611;
  TurnRight_Stage1_frm[13].positions[5] = 1309;
  TurnRight_Stage1_frm[13].positions[6] = 1540;
  TurnRight_Stage1_frm[13].positions[7] = 1339;
  TurnRight_Stage1_frm[13].positions[8] = 1548;
  TurnRight_Stage1_frm[13].positions[9] = 1540;
  TurnRight_Stage1_frm[13].positions[10] = 1500;
  TurnRight_Stage1_frm[13].positions[11] = 1860;
  TurnRight_Stage1_frm[13].positions[12] = 1590;
  TurnRight_Stage1_frm[13].positions[13] = 1499;
  TurnRight_Stage1_frm[13].positions[14] = 1394;
  TurnRight_Stage1_frm[13].positions[15] = 1697;
  TurnRight_Stage1_frm[13].positions[16] = 1512;
  TurnRight_Stage1_frm[13].positions[17] = 1664;
  TurnRight_Stage1_frm[13].positions[18] = 1462;
  TurnRight_Stage1_frm[13].positions[19] = 1512;

  TurnRight_Stage2_frm[0].positions[0] = 1500;
  TurnRight_Stage2_frm[0].positions[1] = 1470;
  TurnRight_Stage2_frm[0].positions[2] = 1500;
  TurnRight_Stage2_frm[0].positions[3] = 1500;
  TurnRight_Stage2_frm[0].positions[4] = 1613;
  TurnRight_Stage2_frm[0].positions[5] = 1320;
  TurnRight_Stage2_frm[0].positions[6] = 1560;
  TurnRight_Stage2_frm[0].positions[7] = 1327;
  TurnRight_Stage2_frm[0].positions[8] = 1547;
  TurnRight_Stage2_frm[0].positions[9] = 1560;
  TurnRight_Stage2_frm[0].positions[10] = 1500;
  TurnRight_Stage2_frm[0].positions[11] = 1860;
  TurnRight_Stage2_frm[0].positions[12] = 1590;
  TurnRight_Stage2_frm[0].positions[13] = 1500;
  TurnRight_Stage2_frm[0].positions[14] = 1396;
  TurnRight_Stage2_frm[0].positions[15] = 1697;
  TurnRight_Stage2_frm[0].positions[16] = 1532;
  TurnRight_Stage2_frm[0].positions[17] = 1672;
  TurnRight_Stage2_frm[0].positions[18] = 1469;
  TurnRight_Stage2_frm[0].positions[19] = 1532;

  TurnRight_Stage2_frm[1].positions[0] = 1500;
  TurnRight_Stage2_frm[1].positions[1] = 1470;
  TurnRight_Stage2_frm[1].positions[2] = 1500;
  TurnRight_Stage2_frm[1].positions[3] = 1501;
  TurnRight_Stage2_frm[1].positions[4] = 1613;
  TurnRight_Stage2_frm[1].positions[5] = 1343;
  TurnRight_Stage2_frm[1].positions[6] = 1594;
  TurnRight_Stage2_frm[1].positions[7] = 1298;
  TurnRight_Stage2_frm[1].positions[8] = 1542;
  TurnRight_Stage2_frm[1].positions[9] = 1594;
  TurnRight_Stage2_frm[1].positions[10] = 1500;
  TurnRight_Stage2_frm[1].positions[11] = 1860;
  TurnRight_Stage2_frm[1].positions[12] = 1590;
  TurnRight_Stage2_frm[1].positions[13] = 1501;
  TurnRight_Stage2_frm[1].positions[14] = 1391;
  TurnRight_Stage2_frm[1].positions[15] = 1712;
  TurnRight_Stage2_frm[1].positions[16] = 1568;
  TurnRight_Stage2_frm[1].positions[17] = 1658;
  TurnRight_Stage2_frm[1].positions[18] = 1475;
  TurnRight_Stage2_frm[1].positions[19] = 1568;

  TurnRight_Stage2_frm[2].positions[0] = 1500;
  TurnRight_Stage2_frm[2].positions[1] = 1470;
  TurnRight_Stage2_frm[2].positions[2] = 1500;
  TurnRight_Stage2_frm[2].positions[3] = 1502;
  TurnRight_Stage2_frm[2].positions[4] = 1605;
  TurnRight_Stage2_frm[2].positions[5] = 1363;
  TurnRight_Stage2_frm[2].positions[6] = 1621;
  TurnRight_Stage2_frm[2].positions[7] = 1270;
  TurnRight_Stage2_frm[2].positions[8] = 1534;
  TurnRight_Stage2_frm[2].positions[9] = 1621;
  TurnRight_Stage2_frm[2].positions[10] = 1500;
  TurnRight_Stage2_frm[2].positions[11] = 1860;
  TurnRight_Stage2_frm[2].positions[12] = 1590;
  TurnRight_Stage2_frm[2].positions[13] = 1502;
  TurnRight_Stage2_frm[2].positions[14] = 1385;
  TurnRight_Stage2_frm[2].positions[15] = 1773;
  TurnRight_Stage2_frm[2].positions[16] = 1596;
  TurnRight_Stage2_frm[2].positions[17] = 1554;
  TurnRight_Stage2_frm[2].positions[18] = 1439;
  TurnRight_Stage2_frm[2].positions[19] = 1596;

  TurnRight_Stage2_frm[3].positions[0] = 1500;
  TurnRight_Stage2_frm[3].positions[1] = 1470;
  TurnRight_Stage2_frm[3].positions[2] = 1500;
  TurnRight_Stage2_frm[3].positions[3] = 1499;
  TurnRight_Stage2_frm[3].positions[4] = 1574;
  TurnRight_Stage2_frm[3].positions[5] = 1375;
  TurnRight_Stage2_frm[3].positions[6] = 1640;
  TurnRight_Stage2_frm[3].positions[7] = 1251;
  TurnRight_Stage2_frm[3].positions[8] = 1526;
  TurnRight_Stage2_frm[3].positions[9] = 1640;
  TurnRight_Stage2_frm[3].positions[10] = 1500;
  TurnRight_Stage2_frm[3].positions[11] = 1860;
  TurnRight_Stage2_frm[3].positions[12] = 1590;
  TurnRight_Stage2_frm[3].positions[13] = 1499;
  TurnRight_Stage2_frm[3].positions[14] = 1409;
  TurnRight_Stage2_frm[3].positions[15] = 1871;
  TurnRight_Stage2_frm[3].positions[16] = 1628;
  TurnRight_Stage2_frm[3].positions[17] = 1348;
  TurnRight_Stage2_frm[3].positions[18] = 1335;
  TurnRight_Stage2_frm[3].positions[19] = 1628;

  TurnRight_Stage2_frm[4].positions[0] = 1500;
  TurnRight_Stage2_frm[4].positions[1] = 1470;
  TurnRight_Stage2_frm[4].positions[2] = 1500;
  TurnRight_Stage2_frm[4].positions[3] = 1495;
  TurnRight_Stage2_frm[4].positions[4] = 1516;
  TurnRight_Stage2_frm[4].positions[5] = 1363;
  TurnRight_Stage2_frm[4].positions[6] = 1631;
  TurnRight_Stage2_frm[4].positions[7] = 1262;
  TurnRight_Stage2_frm[4].positions[8] = 1525;
  TurnRight_Stage2_frm[4].positions[9] = 1631;
  TurnRight_Stage2_frm[4].positions[10] = 1500;
  TurnRight_Stage2_frm[4].positions[11] = 1860;
  TurnRight_Stage2_frm[4].positions[12] = 1590;
  TurnRight_Stage2_frm[4].positions[13] = 1495;
  TurnRight_Stage2_frm[4].positions[14] = 1467;
  TurnRight_Stage2_frm[4].positions[15] = 1833;
  TurnRight_Stage2_frm[4].positions[16] = 1624;
  TurnRight_Stage2_frm[4].positions[17] = 1359;
  TurnRight_Stage2_frm[4].positions[18] = 1293;
  TurnRight_Stage2_frm[4].positions[19] = 1624;

  TurnRight_Stage2_frm[5].positions[0] = 1500;
  TurnRight_Stage2_frm[5].positions[1] = 1470;
  TurnRight_Stage2_frm[5].positions[2] = 1500;
  TurnRight_Stage2_frm[5].positions[3] = 1498;
  TurnRight_Stage2_frm[5].positions[4] = 1496;
  TurnRight_Stage2_frm[5].positions[5] = 1333;
  TurnRight_Stage2_frm[5].positions[6] = 1596;
  TurnRight_Stage2_frm[5].positions[7] = 1318;
  TurnRight_Stage2_frm[5].positions[8] = 1551;
  TurnRight_Stage2_frm[5].positions[9] = 1596;
  TurnRight_Stage2_frm[5].positions[10] = 1500;
  TurnRight_Stage2_frm[5].positions[11] = 1860;
  TurnRight_Stage2_frm[5].positions[12] = 1590;
  TurnRight_Stage2_frm[5].positions[13] = 1498;
  TurnRight_Stage2_frm[5].positions[14] = 1502;
  TurnRight_Stage2_frm[5].positions[15] = 1696;
  TurnRight_Stage2_frm[5].positions[16] = 1583;
  TurnRight_Stage2_frm[5].positions[17] = 1614;
  TurnRight_Stage2_frm[5].positions[18] = 1405;
  TurnRight_Stage2_frm[5].positions[19] = 1583;

  TurnRight_Stage2_frm[6].positions[0] = 1500;
  TurnRight_Stage2_frm[6].positions[1] = 1470;
  TurnRight_Stage2_frm[6].positions[2] = 1500;
  TurnRight_Stage2_frm[6].positions[3] = 1500;
  TurnRight_Stage2_frm[6].positions[4] = 1500;
  TurnRight_Stage2_frm[6].positions[5] = 1317;
  TurnRight_Stage2_frm[6].positions[6] = 1564;
  TurnRight_Stage2_frm[6].positions[7] = 1352;
  TurnRight_Stage2_frm[6].positions[8] = 1570;
  TurnRight_Stage2_frm[6].positions[9] = 1564;
  TurnRight_Stage2_frm[6].positions[10] = 1500;
  TurnRight_Stage2_frm[6].positions[11] = 1860;
  TurnRight_Stage2_frm[6].positions[12] = 1590;
  TurnRight_Stage2_frm[6].positions[13] = 1500;
  TurnRight_Stage2_frm[6].positions[14] = 1500;
  TurnRight_Stage2_frm[6].positions[15] = 1689;
  TurnRight_Stage2_frm[6].positions[16] = 1550;
  TurnRight_Stage2_frm[6].positions[17] = 1636;
  TurnRight_Stage2_frm[6].positions[18] = 1424;
  TurnRight_Stage2_frm[6].positions[19] = 1550;

  TurnRight_Stage2_frm[7].positions[0] = 1500;
  TurnRight_Stage2_frm[7].positions[1] = 1470;
  TurnRight_Stage2_frm[7].positions[2] = 1500;
  TurnRight_Stage2_frm[7].positions[3] = 1500;
  TurnRight_Stage2_frm[7].positions[4] = 1501;
  TurnRight_Stage2_frm[7].positions[5] = 1316;
  TurnRight_Stage2_frm[7].positions[6] = 1554;
  TurnRight_Stage2_frm[7].positions[7] = 1355;
  TurnRight_Stage2_frm[7].positions[8] = 1571;
  TurnRight_Stage2_frm[7].positions[9] = 1554;
  TurnRight_Stage2_frm[7].positions[10] = 1500;
  TurnRight_Stage2_frm[7].positions[11] = 1860;
  TurnRight_Stage2_frm[7].positions[12] = 1590;
  TurnRight_Stage2_frm[7].positions[13] = 1500;
  TurnRight_Stage2_frm[7].positions[14] = 1501;
  TurnRight_Stage2_frm[7].positions[15] = 1689;
  TurnRight_Stage2_frm[7].positions[16] = 1540;
  TurnRight_Stage2_frm[7].positions[17] = 1636;
  TurnRight_Stage2_frm[7].positions[18] = 1424;
  TurnRight_Stage2_frm[7].positions[19] = 1540;

  TurnRight_Stage2_frm[8].positions[0] = 1500;
  TurnRight_Stage2_frm[8].positions[1] = 1470;
  TurnRight_Stage2_frm[8].positions[2] = 1500;
  TurnRight_Stage2_frm[8].positions[3] = 1500;
  TurnRight_Stage2_frm[8].positions[4] = 1500;
  TurnRight_Stage2_frm[8].positions[5] = 1317;
  TurnRight_Stage2_frm[8].positions[6] = 1514;
  TurnRight_Stage2_frm[8].positions[7] = 1354;
  TurnRight_Stage2_frm[8].positions[8] = 1571;
  TurnRight_Stage2_frm[8].positions[9] = 1514;
  TurnRight_Stage2_frm[8].positions[10] = 1500;
  TurnRight_Stage2_frm[8].positions[11] = 1860;
  TurnRight_Stage2_frm[8].positions[12] = 1590;
  TurnRight_Stage2_frm[8].positions[13] = 1500;
  TurnRight_Stage2_frm[8].positions[14] = 1500;
  TurnRight_Stage2_frm[8].positions[15] = 1684;
  TurnRight_Stage2_frm[8].positions[16] = 1500;
  TurnRight_Stage2_frm[8].positions[17] = 1644;
  TurnRight_Stage2_frm[8].positions[18] = 1428;
  TurnRight_Stage2_frm[8].positions[19] = 1500;

  TurnRight_Stage2_frm[9].positions[0] = 1500;
  TurnRight_Stage2_frm[9].positions[1] = 1470;
  TurnRight_Stage2_frm[9].positions[2] = 1500;
  TurnRight_Stage2_frm[9].positions[3] = 1500;
  TurnRight_Stage2_frm[9].positions[4] = 1500;
  TurnRight_Stage2_frm[9].positions[5] = 1317;
  TurnRight_Stage2_frm[9].positions[6] = 1516;
  TurnRight_Stage2_frm[9].positions[7] = 1354;
  TurnRight_Stage2_frm[9].positions[8] = 1571;
  TurnRight_Stage2_frm[9].positions[9] = 1516;
  TurnRight_Stage2_frm[9].positions[10] = 1500;
  TurnRight_Stage2_frm[9].positions[11] = 1860;
  TurnRight_Stage2_frm[9].positions[12] = 1590;
  TurnRight_Stage2_frm[9].positions[13] = 1500;
  TurnRight_Stage2_frm[9].positions[14] = 1500;
  TurnRight_Stage2_frm[9].positions[15] = 1684;
  TurnRight_Stage2_frm[9].positions[16] = 1503;
  TurnRight_Stage2_frm[9].positions[17] = 1644;
  TurnRight_Stage2_frm[9].positions[18] = 1428;
  TurnRight_Stage2_frm[9].positions[19] = 1503;

  WalkBwd_Stage0_frm[0].positions[0] = 1500;
  WalkBwd_Stage0_frm[0].positions[1] = 1470;
  WalkBwd_Stage0_frm[0].positions[2] = 1500;
  WalkBwd_Stage0_frm[0].positions[3] = 1500;
  WalkBwd_Stage0_frm[0].positions[4] = 1500;
  WalkBwd_Stage0_frm[0].positions[5] = 1268;
  WalkBwd_Stage0_frm[0].positions[6] = 1507;
  WalkBwd_Stage0_frm[0].positions[7] = 1451;
  WalkBwd_Stage0_frm[0].positions[8] = 1619;
  WalkBwd_Stage0_frm[0].positions[9] = 1507;
  WalkBwd_Stage0_frm[0].positions[10] = 1500;
  WalkBwd_Stage0_frm[0].positions[11] = 1860;
  WalkBwd_Stage0_frm[0].positions[12] = 1590;
  WalkBwd_Stage0_frm[0].positions[13] = 1500;
  WalkBwd_Stage0_frm[0].positions[14] = 1500;
  WalkBwd_Stage0_frm[0].positions[15] = 1732;
  WalkBwd_Stage0_frm[0].positions[16] = 1493;
  WalkBwd_Stage0_frm[0].positions[17] = 1549;
  WalkBwd_Stage0_frm[0].positions[18] = 1381;
  WalkBwd_Stage0_frm[0].positions[19] = 1493;

  WalkBwd_Stage0_frm[1].positions[0] = 1500;
  WalkBwd_Stage0_frm[1].positions[1] = 1470;
  WalkBwd_Stage0_frm[1].positions[2] = 1500;
  WalkBwd_Stage0_frm[1].positions[3] = 1500;
  WalkBwd_Stage0_frm[1].positions[4] = 1500;
  WalkBwd_Stage0_frm[1].positions[5] = 1268;
  WalkBwd_Stage0_frm[1].positions[6] = 1507;
  WalkBwd_Stage0_frm[1].positions[7] = 1451;
  WalkBwd_Stage0_frm[1].positions[8] = 1619;
  WalkBwd_Stage0_frm[1].positions[9] = 1507;
  WalkBwd_Stage0_frm[1].positions[10] = 1500;
  WalkBwd_Stage0_frm[1].positions[11] = 1860;
  WalkBwd_Stage0_frm[1].positions[12] = 1590;
  WalkBwd_Stage0_frm[1].positions[13] = 1500;
  WalkBwd_Stage0_frm[1].positions[14] = 1500;
  WalkBwd_Stage0_frm[1].positions[15] = 1732;
  WalkBwd_Stage0_frm[1].positions[16] = 1493;
  WalkBwd_Stage0_frm[1].positions[17] = 1549;
  WalkBwd_Stage0_frm[1].positions[18] = 1381;
  WalkBwd_Stage0_frm[1].positions[19] = 1493;

  WalkBwd_Stage0_frm[2].positions[0] = 1500;
  WalkBwd_Stage0_frm[2].positions[1] = 1470;
  WalkBwd_Stage0_frm[2].positions[2] = 1500;
  WalkBwd_Stage0_frm[2].positions[3] = 1500;
  WalkBwd_Stage0_frm[2].positions[4] = 1500;
  WalkBwd_Stage0_frm[2].positions[5] = 1268;
  WalkBwd_Stage0_frm[2].positions[6] = 1499;
  WalkBwd_Stage0_frm[2].positions[7] = 1452;
  WalkBwd_Stage0_frm[2].positions[8] = 1619;
  WalkBwd_Stage0_frm[2].positions[9] = 1499;
  WalkBwd_Stage0_frm[2].positions[10] = 1500;
  WalkBwd_Stage0_frm[2].positions[11] = 1860;
  WalkBwd_Stage0_frm[2].positions[12] = 1590;
  WalkBwd_Stage0_frm[2].positions[13] = 1500;
  WalkBwd_Stage0_frm[2].positions[14] = 1500;
  WalkBwd_Stage0_frm[2].positions[15] = 1732;
  WalkBwd_Stage0_frm[2].positions[16] = 1484;
  WalkBwd_Stage0_frm[2].positions[17] = 1550;
  WalkBwd_Stage0_frm[2].positions[18] = 1381;
  WalkBwd_Stage0_frm[2].positions[19] = 1484;

  WalkBwd_Stage0_frm[3].positions[0] = 1500;
  WalkBwd_Stage0_frm[3].positions[1] = 1470;
  WalkBwd_Stage0_frm[3].positions[2] = 1500;
  WalkBwd_Stage0_frm[3].positions[3] = 1501;
  WalkBwd_Stage0_frm[3].positions[4] = 1500;
  WalkBwd_Stage0_frm[3].positions[5] = 1274;
  WalkBwd_Stage0_frm[3].positions[6] = 1450;
  WalkBwd_Stage0_frm[3].positions[7] = 1436;
  WalkBwd_Stage0_frm[3].positions[8] = 1610;
  WalkBwd_Stage0_frm[3].positions[9] = 1450;
  WalkBwd_Stage0_frm[3].positions[10] = 1500;
  WalkBwd_Stage0_frm[3].positions[11] = 1860;
  WalkBwd_Stage0_frm[3].positions[12] = 1590;
  WalkBwd_Stage0_frm[3].positions[13] = 1501;
  WalkBwd_Stage0_frm[3].positions[14] = 1500;
  WalkBwd_Stage0_frm[3].positions[15] = 1722;
  WalkBwd_Stage0_frm[3].positions[16] = 1436;
  WalkBwd_Stage0_frm[3].positions[17] = 1573;
  WalkBwd_Stage0_frm[3].positions[18] = 1395;
  WalkBwd_Stage0_frm[3].positions[19] = 1436;

  WalkBwd_Stage0_frm[4].positions[0] = 1500;
  WalkBwd_Stage0_frm[4].positions[1] = 1470;
  WalkBwd_Stage0_frm[4].positions[2] = 1500;
  WalkBwd_Stage0_frm[4].positions[3] = 1501;
  WalkBwd_Stage0_frm[4].positions[4] = 1500;
  WalkBwd_Stage0_frm[4].positions[5] = 1275;
  WalkBwd_Stage0_frm[4].positions[6] = 1443;
  WalkBwd_Stage0_frm[4].positions[7] = 1431;
  WalkBwd_Stage0_frm[4].positions[8] = 1607;
  WalkBwd_Stage0_frm[4].positions[9] = 1443;
  WalkBwd_Stage0_frm[4].positions[10] = 1500;
  WalkBwd_Stage0_frm[4].positions[11] = 1860;
  WalkBwd_Stage0_frm[4].positions[12] = 1590;
  WalkBwd_Stage0_frm[4].positions[13] = 1501;
  WalkBwd_Stage0_frm[4].positions[14] = 1500;
  WalkBwd_Stage0_frm[4].positions[15] = 1719;
  WalkBwd_Stage0_frm[4].positions[16] = 1429;
  WalkBwd_Stage0_frm[4].positions[17] = 1579;
  WalkBwd_Stage0_frm[4].positions[18] = 1399;
  WalkBwd_Stage0_frm[4].positions[19] = 1429;

  WalkBwd_Stage0_frm[5].positions[0] = 1500;
  WalkBwd_Stage0_frm[5].positions[1] = 1470;
  WalkBwd_Stage0_frm[5].positions[2] = 1500;
  WalkBwd_Stage0_frm[5].positions[3] = 1504;
  WalkBwd_Stage0_frm[5].positions[4] = 1499;
  WalkBwd_Stage0_frm[5].positions[5] = 1273;
  WalkBwd_Stage0_frm[5].positions[6] = 1429;
  WalkBwd_Stage0_frm[5].positions[7] = 1432;
  WalkBwd_Stage0_frm[5].positions[8] = 1600;
  WalkBwd_Stage0_frm[5].positions[9] = 1429;
  WalkBwd_Stage0_frm[5].positions[10] = 1500;
  WalkBwd_Stage0_frm[5].positions[11] = 1860;
  WalkBwd_Stage0_frm[5].positions[12] = 1590;
  WalkBwd_Stage0_frm[5].positions[13] = 1504;
  WalkBwd_Stage0_frm[5].positions[14] = 1500;
  WalkBwd_Stage0_frm[5].positions[15] = 1715;
  WalkBwd_Stage0_frm[5].positions[16] = 1415;
  WalkBwd_Stage0_frm[5].positions[17] = 1595;
  WalkBwd_Stage0_frm[5].positions[18] = 1409;
  WalkBwd_Stage0_frm[5].positions[19] = 1415;

  WalkBwd_Stage0_frm[6].positions[0] = 1500;
  WalkBwd_Stage0_frm[6].positions[1] = 1470;
  WalkBwd_Stage0_frm[6].positions[2] = 1500;
  WalkBwd_Stage0_frm[6].positions[3] = 1511;
  WalkBwd_Stage0_frm[6].positions[4] = 1499;
  WalkBwd_Stage0_frm[6].positions[5] = 1244;
  WalkBwd_Stage0_frm[6].positions[6] = 1417;
  WalkBwd_Stage0_frm[6].positions[7] = 1496;
  WalkBwd_Stage0_frm[6].positions[8] = 1630;
  WalkBwd_Stage0_frm[6].positions[9] = 1417;
  WalkBwd_Stage0_frm[6].positions[10] = 1500;
  WalkBwd_Stage0_frm[6].positions[11] = 1860;
  WalkBwd_Stage0_frm[6].positions[12] = 1590;
  WalkBwd_Stage0_frm[6].positions[13] = 1511;
  WalkBwd_Stage0_frm[6].positions[14] = 1500;
  WalkBwd_Stage0_frm[6].positions[15] = 1711;
  WalkBwd_Stage0_frm[6].positions[16] = 1404;
  WalkBwd_Stage0_frm[6].positions[17] = 1612;
  WalkBwd_Stage0_frm[6].positions[18] = 1422;
  WalkBwd_Stage0_frm[6].positions[19] = 1404;

  WalkBwd_Stage0_frm[7].positions[0] = 1500;
  WalkBwd_Stage0_frm[7].positions[1] = 1470;
  WalkBwd_Stage0_frm[7].positions[2] = 1500;
  WalkBwd_Stage0_frm[7].positions[3] = 1532;
  WalkBwd_Stage0_frm[7].positions[4] = 1499;
  WalkBwd_Stage0_frm[7].positions[5] = 1168;
  WalkBwd_Stage0_frm[7].positions[6] = 1401;
  WalkBwd_Stage0_frm[7].positions[7] = 1702;
  WalkBwd_Stage0_frm[7].positions[8] = 1766;
  WalkBwd_Stage0_frm[7].positions[9] = 1401;
  WalkBwd_Stage0_frm[7].positions[10] = 1500;
  WalkBwd_Stage0_frm[7].positions[11] = 1860;
  WalkBwd_Stage0_frm[7].positions[12] = 1590;
  WalkBwd_Stage0_frm[7].positions[13] = 1532;
  WalkBwd_Stage0_frm[7].positions[14] = 1500;
  WalkBwd_Stage0_frm[7].positions[15] = 1710;
  WalkBwd_Stage0_frm[7].positions[16] = 1397;
  WalkBwd_Stage0_frm[7].positions[17] = 1635;
  WalkBwd_Stage0_frm[7].positions[18] = 1445;
  WalkBwd_Stage0_frm[7].positions[19] = 1397;

  WalkBwd_Stage0_frm[8].positions[0] = 1500;
  WalkBwd_Stage0_frm[8].positions[1] = 1470;
  WalkBwd_Stage0_frm[8].positions[2] = 1500;
  WalkBwd_Stage0_frm[8].positions[3] = 1574;
  WalkBwd_Stage0_frm[8].positions[4] = 1499;
  WalkBwd_Stage0_frm[8].positions[5] = 1219;
  WalkBwd_Stage0_frm[8].positions[6] = 1411;
  WalkBwd_Stage0_frm[8].positions[7] = 1717;
  WalkBwd_Stage0_frm[8].positions[8] = 1832;
  WalkBwd_Stage0_frm[8].positions[9] = 1411;
  WalkBwd_Stage0_frm[8].positions[10] = 1500;
  WalkBwd_Stage0_frm[8].positions[11] = 1860;
  WalkBwd_Stage0_frm[8].positions[12] = 1590;
  WalkBwd_Stage0_frm[8].positions[13] = 1574;
  WalkBwd_Stage0_frm[8].positions[14] = 1500;
  WalkBwd_Stage0_frm[8].positions[15] = 1730;
  WalkBwd_Stage0_frm[8].positions[16] = 1410;
  WalkBwd_Stage0_frm[8].positions[17] = 1641;
  WalkBwd_Stage0_frm[8].positions[18] = 1470;
  WalkBwd_Stage0_frm[8].positions[19] = 1410;

  WalkBwd_Stage0_frm[9].positions[0] = 1500;
  WalkBwd_Stage0_frm[9].positions[1] = 1470;
  WalkBwd_Stage0_frm[9].positions[2] = 1500;
  WalkBwd_Stage0_frm[9].positions[3] = 1600;
  WalkBwd_Stage0_frm[9].positions[4] = 1499;
  WalkBwd_Stage0_frm[9].positions[5] = 1331;
  WalkBwd_Stage0_frm[9].positions[6] = 1460;
  WalkBwd_Stage0_frm[9].positions[7] = 1522;
  WalkBwd_Stage0_frm[9].positions[8] = 1745;
  WalkBwd_Stage0_frm[9].positions[9] = 1460;
  WalkBwd_Stage0_frm[9].positions[10] = 1500;
  WalkBwd_Stage0_frm[9].positions[11] = 1860;
  WalkBwd_Stage0_frm[9].positions[12] = 1590;
  WalkBwd_Stage0_frm[9].positions[13] = 1600;
  WalkBwd_Stage0_frm[9].positions[14] = 1500;
  WalkBwd_Stage0_frm[9].positions[15] = 1773;
  WalkBwd_Stage0_frm[9].positions[16] = 1454;
  WalkBwd_Stage0_frm[9].positions[17] = 1615;
  WalkBwd_Stage0_frm[9].positions[18] = 1488;
  WalkBwd_Stage0_frm[9].positions[19] = 1454;

  WalkBwd_Stage0_frm[10].positions[0] = 1500;
  WalkBwd_Stage0_frm[10].positions[1] = 1470;
  WalkBwd_Stage0_frm[10].positions[2] = 1500;
  WalkBwd_Stage0_frm[10].positions[3] = 1603;
  WalkBwd_Stage0_frm[10].positions[4] = 1500;
  WalkBwd_Stage0_frm[10].positions[5] = 1316;
  WalkBwd_Stage0_frm[10].positions[6] = 1509;
  WalkBwd_Stage0_frm[10].positions[7] = 1509;
  WalkBwd_Stage0_frm[10].positions[8] = 1725;
  WalkBwd_Stage0_frm[10].positions[9] = 1509;
  WalkBwd_Stage0_frm[10].positions[10] = 1500;
  WalkBwd_Stage0_frm[10].positions[11] = 1860;
  WalkBwd_Stage0_frm[10].positions[12] = 1590;
  WalkBwd_Stage0_frm[10].positions[13] = 1603;
  WalkBwd_Stage0_frm[10].positions[14] = 1500;
  WalkBwd_Stage0_frm[10].positions[15] = 1792;
  WalkBwd_Stage0_frm[10].positions[16] = 1502;
  WalkBwd_Stage0_frm[10].positions[17] = 1627;
  WalkBwd_Stage0_frm[10].positions[18] = 1518;
  WalkBwd_Stage0_frm[10].positions[19] = 1502;

  WalkBwd_Stage1_frm[0].positions[0] = 1500;
  WalkBwd_Stage1_frm[0].positions[1] = 1470;
  WalkBwd_Stage1_frm[0].positions[2] = 1500;
  WalkBwd_Stage1_frm[0].positions[3] = 1603;
  WalkBwd_Stage1_frm[0].positions[4] = 1500;
  WalkBwd_Stage1_frm[0].positions[5] = 1308;
  WalkBwd_Stage1_frm[0].positions[6] = 1526;
  WalkBwd_Stage1_frm[0].positions[7] = 1508;
  WalkBwd_Stage1_frm[0].positions[8] = 1716;
  WalkBwd_Stage1_frm[0].positions[9] = 1526;
  WalkBwd_Stage1_frm[0].positions[10] = 1500;
  WalkBwd_Stage1_frm[0].positions[11] = 1860;
  WalkBwd_Stage1_frm[0].positions[12] = 1590;
  WalkBwd_Stage1_frm[0].positions[13] = 1603;
  WalkBwd_Stage1_frm[0].positions[14] = 1500;
  WalkBwd_Stage1_frm[0].positions[15] = 1789;
  WalkBwd_Stage1_frm[0].positions[16] = 1519;
  WalkBwd_Stage1_frm[0].positions[17] = 1649;
  WalkBwd_Stage1_frm[0].positions[18] = 1538;
  WalkBwd_Stage1_frm[0].positions[19] = 1519;

  WalkBwd_Stage1_frm[1].positions[0] = 1500;
  WalkBwd_Stage1_frm[1].positions[1] = 1470;
  WalkBwd_Stage1_frm[1].positions[2] = 1500;
  WalkBwd_Stage1_frm[1].positions[3] = 1598;
  WalkBwd_Stage1_frm[1].positions[4] = 1500;
  WalkBwd_Stage1_frm[1].positions[5] = 1300;
  WalkBwd_Stage1_frm[1].positions[6] = 1556;
  WalkBwd_Stage1_frm[1].positions[7] = 1487;
  WalkBwd_Stage1_frm[1].positions[8] = 1688;
  WalkBwd_Stage1_frm[1].positions[9] = 1556;
  WalkBwd_Stage1_frm[1].positions[10] = 1500;
  WalkBwd_Stage1_frm[1].positions[11] = 1860;
  WalkBwd_Stage1_frm[1].positions[12] = 1590;
  WalkBwd_Stage1_frm[1].positions[13] = 1598;
  WalkBwd_Stage1_frm[1].positions[14] = 1501;
  WalkBwd_Stage1_frm[1].positions[15] = 1781;
  WalkBwd_Stage1_frm[1].positions[16] = 1550;
  WalkBwd_Stage1_frm[1].positions[17] = 1694;
  WalkBwd_Stage1_frm[1].positions[18] = 1583;
  WalkBwd_Stage1_frm[1].positions[19] = 1550;

  WalkBwd_Stage1_frm[2].positions[0] = 1500;
  WalkBwd_Stage1_frm[2].positions[1] = 1470;
  WalkBwd_Stage1_frm[2].positions[2] = 1500;
  WalkBwd_Stage1_frm[2].positions[3] = 1584;
  WalkBwd_Stage1_frm[2].positions[4] = 1500;
  WalkBwd_Stage1_frm[2].positions[5] = 1301;
  WalkBwd_Stage1_frm[2].positions[6] = 1580;
  WalkBwd_Stage1_frm[2].positions[7] = 1450;
  WalkBwd_Stage1_frm[2].positions[8] = 1651;
  WalkBwd_Stage1_frm[2].positions[9] = 1580;
  WalkBwd_Stage1_frm[2].positions[10] = 1500;
  WalkBwd_Stage1_frm[2].positions[11] = 1860;
  WalkBwd_Stage1_frm[2].positions[12] = 1590;
  WalkBwd_Stage1_frm[2].positions[13] = 1584;
  WalkBwd_Stage1_frm[2].positions[14] = 1503;
  WalkBwd_Stage1_frm[2].positions[15] = 1818;
  WalkBwd_Stage1_frm[2].positions[16] = 1576;
  WalkBwd_Stage1_frm[2].positions[17] = 1622;
  WalkBwd_Stage1_frm[2].positions[18] = 1561;
  WalkBwd_Stage1_frm[2].positions[19] = 1576;

  WalkBwd_Stage1_frm[3].positions[0] = 1500;
  WalkBwd_Stage1_frm[3].positions[1] = 1470;
  WalkBwd_Stage1_frm[3].positions[2] = 1500;
  WalkBwd_Stage1_frm[3].positions[3] = 1540;
  WalkBwd_Stage1_frm[3].positions[4] = 1500;
  WalkBwd_Stage1_frm[3].positions[5] = 1300;
  WalkBwd_Stage1_frm[3].positions[6] = 1598;
  WalkBwd_Stage1_frm[3].positions[7] = 1400;
  WalkBwd_Stage1_frm[3].positions[8] = 1599;
  WalkBwd_Stage1_frm[3].positions[9] = 1598;
  WalkBwd_Stage1_frm[3].positions[10] = 1500;
  WalkBwd_Stage1_frm[3].positions[11] = 1860;
  WalkBwd_Stage1_frm[3].positions[12] = 1590;
  WalkBwd_Stage1_frm[3].positions[13] = 1540;
  WalkBwd_Stage1_frm[3].positions[14] = 1502;
  WalkBwd_Stage1_frm[3].positions[15] = 1912;
  WalkBwd_Stage1_frm[3].positions[16] = 1603;
  WalkBwd_Stage1_frm[3].positions[17] = 1348;
  WalkBwd_Stage1_frm[3].positions[18] = 1369;
  WalkBwd_Stage1_frm[3].positions[19] = 1603;

  WalkBwd_Stage1_frm[4].positions[0] = 1500;
  WalkBwd_Stage1_frm[4].positions[1] = 1470;
  WalkBwd_Stage1_frm[4].positions[2] = 1500;
  WalkBwd_Stage1_frm[4].positions[3] = 1457;
  WalkBwd_Stage1_frm[4].positions[4] = 1500;
  WalkBwd_Stage1_frm[4].positions[5] = 1274;
  WalkBwd_Stage1_frm[4].positions[6] = 1588;
  WalkBwd_Stage1_frm[4].positions[7] = 1378;
  WalkBwd_Stage1_frm[4].positions[8] = 1552;
  WalkBwd_Stage1_frm[4].positions[9] = 1588;
  WalkBwd_Stage1_frm[4].positions[10] = 1500;
  WalkBwd_Stage1_frm[4].positions[11] = 1860;
  WalkBwd_Stage1_frm[4].positions[12] = 1590;
  WalkBwd_Stage1_frm[4].positions[13] = 1457;
  WalkBwd_Stage1_frm[4].positions[14] = 1501;
  WalkBwd_Stage1_frm[4].positions[15] = 1823;
  WalkBwd_Stage1_frm[4].positions[16] = 1597;
  WalkBwd_Stage1_frm[4].positions[17] = 1279;
  WalkBwd_Stage1_frm[4].positions[18] = 1211;
  WalkBwd_Stage1_frm[4].positions[19] = 1597;

  WalkBwd_Stage1_frm[5].positions[0] = 1500;
  WalkBwd_Stage1_frm[5].positions[1] = 1470;
  WalkBwd_Stage1_frm[5].positions[2] = 1500;
  WalkBwd_Stage1_frm[5].positions[3] = 1404;
  WalkBwd_Stage1_frm[5].positions[4] = 1500;
  WalkBwd_Stage1_frm[5].positions[5] = 1227;
  WalkBwd_Stage1_frm[5].positions[6] = 1542;
  WalkBwd_Stage1_frm[5].positions[7] = 1398;
  WalkBwd_Stage1_frm[5].positions[8] = 1525;
  WalkBwd_Stage1_frm[5].positions[9] = 1542;
  WalkBwd_Stage1_frm[5].positions[10] = 1500;
  WalkBwd_Stage1_frm[5].positions[11] = 1860;
  WalkBwd_Stage1_frm[5].positions[12] = 1590;
  WalkBwd_Stage1_frm[5].positions[13] = 1404;
  WalkBwd_Stage1_frm[5].positions[14] = 1501;
  WalkBwd_Stage1_frm[5].positions[15] = 1673;
  WalkBwd_Stage1_frm[5].positions[16] = 1544;
  WalkBwd_Stage1_frm[5].positions[17] = 1466;
  WalkBwd_Stage1_frm[5].positions[18] = 1255;
  WalkBwd_Stage1_frm[5].positions[19] = 1544;

  WalkBwd_Stage1_frm[6].positions[0] = 1500;
  WalkBwd_Stage1_frm[6].positions[1] = 1470;
  WalkBwd_Stage1_frm[6].positions[2] = 1500;
  WalkBwd_Stage1_frm[6].positions[3] = 1397;
  WalkBwd_Stage1_frm[6].positions[4] = 1500;
  WalkBwd_Stage1_frm[6].positions[5] = 1208;
  WalkBwd_Stage1_frm[6].positions[6] = 1491;
  WalkBwd_Stage1_frm[6].positions[7] = 1381;
  WalkBwd_Stage1_frm[6].positions[8] = 1489;
  WalkBwd_Stage1_frm[6].positions[9] = 1491;
  WalkBwd_Stage1_frm[6].positions[10] = 1500;
  WalkBwd_Stage1_frm[6].positions[11] = 1860;
  WalkBwd_Stage1_frm[6].positions[12] = 1590;
  WalkBwd_Stage1_frm[6].positions[13] = 1397;
  WalkBwd_Stage1_frm[6].positions[14] = 1500;
  WalkBwd_Stage1_frm[6].positions[15] = 1680;
  WalkBwd_Stage1_frm[6].positions[16] = 1493;
  WalkBwd_Stage1_frm[6].positions[17] = 1491;
  WalkBwd_Stage1_frm[6].positions[18] = 1271;
  WalkBwd_Stage1_frm[6].positions[19] = 1493;

  WalkBwd_Stage1_frm[7].positions[0] = 1500;
  WalkBwd_Stage1_frm[7].positions[1] = 1470;
  WalkBwd_Stage1_frm[7].positions[2] = 1500;
  WalkBwd_Stage1_frm[7].positions[3] = 1398;
  WalkBwd_Stage1_frm[7].positions[4] = 1500;
  WalkBwd_Stage1_frm[7].positions[5] = 1212;
  WalkBwd_Stage1_frm[7].positions[6] = 1474;
  WalkBwd_Stage1_frm[7].positions[7] = 1357;
  WalkBwd_Stage1_frm[7].positions[8] = 1469;
  WalkBwd_Stage1_frm[7].positions[9] = 1474;
  WalkBwd_Stage1_frm[7].positions[10] = 1500;
  WalkBwd_Stage1_frm[7].positions[11] = 1860;
  WalkBwd_Stage1_frm[7].positions[12] = 1590;
  WalkBwd_Stage1_frm[7].positions[13] = 1398;
  WalkBwd_Stage1_frm[7].positions[14] = 1500;
  WalkBwd_Stage1_frm[7].positions[15] = 1689;
  WalkBwd_Stage1_frm[7].positions[16] = 1475;
  WalkBwd_Stage1_frm[7].positions[17] = 1492;
  WalkBwd_Stage1_frm[7].positions[18] = 1281;
  WalkBwd_Stage1_frm[7].positions[19] = 1475;

  WalkBwd_Stage1_frm[8].positions[0] = 1500;
  WalkBwd_Stage1_frm[8].positions[1] = 1470;
  WalkBwd_Stage1_frm[8].positions[2] = 1500;
  WalkBwd_Stage1_frm[8].positions[3] = 1402;
  WalkBwd_Stage1_frm[8].positions[4] = 1499;
  WalkBwd_Stage1_frm[8].positions[5] = 1222;
  WalkBwd_Stage1_frm[8].positions[6] = 1441;
  WalkBwd_Stage1_frm[8].positions[7] = 1308;
  WalkBwd_Stage1_frm[8].positions[8] = 1420;
  WalkBwd_Stage1_frm[8].positions[9] = 1441;
  WalkBwd_Stage1_frm[8].positions[10] = 1500;
  WalkBwd_Stage1_frm[8].positions[11] = 1860;
  WalkBwd_Stage1_frm[8].positions[12] = 1590;
  WalkBwd_Stage1_frm[8].positions[13] = 1402;
  WalkBwd_Stage1_frm[8].positions[14] = 1500;
  WalkBwd_Stage1_frm[8].positions[15] = 1697;
  WalkBwd_Stage1_frm[8].positions[16] = 1443;
  WalkBwd_Stage1_frm[8].positions[17] = 1513;
  WalkBwd_Stage1_frm[8].positions[18] = 1309;
  WalkBwd_Stage1_frm[8].positions[19] = 1443;

  WalkBwd_Stage1_frm[9].positions[0] = 1500;
  WalkBwd_Stage1_frm[9].positions[1] = 1470;
  WalkBwd_Stage1_frm[9].positions[2] = 1500;
  WalkBwd_Stage1_frm[9].positions[3] = 1416;
  WalkBwd_Stage1_frm[9].positions[4] = 1497;
  WalkBwd_Stage1_frm[9].positions[5] = 1186;
  WalkBwd_Stage1_frm[9].positions[6] = 1414;
  WalkBwd_Stage1_frm[9].positions[7] = 1375;
  WalkBwd_Stage1_frm[9].positions[8] = 1440;
  WalkBwd_Stage1_frm[9].positions[9] = 1414;
  WalkBwd_Stage1_frm[9].positions[10] = 1500;
  WalkBwd_Stage1_frm[9].positions[11] = 1860;
  WalkBwd_Stage1_frm[9].positions[12] = 1590;
  WalkBwd_Stage1_frm[9].positions[13] = 1416;
  WalkBwd_Stage1_frm[9].positions[14] = 1500;
  WalkBwd_Stage1_frm[9].positions[15] = 1696;
  WalkBwd_Stage1_frm[9].positions[16] = 1419;
  WalkBwd_Stage1_frm[9].positions[17] = 1551;
  WalkBwd_Stage1_frm[9].positions[18] = 1347;
  WalkBwd_Stage1_frm[9].positions[19] = 1419;

  WalkBwd_Stage1_frm[10].positions[0] = 1500;
  WalkBwd_Stage1_frm[10].positions[1] = 1470;
  WalkBwd_Stage1_frm[10].positions[2] = 1500;
  WalkBwd_Stage1_frm[10].positions[3] = 1460;
  WalkBwd_Stage1_frm[10].positions[4] = 1498;
  WalkBwd_Stage1_frm[10].positions[5] = 1092;
  WalkBwd_Stage1_frm[10].positions[6] = 1390;
  WalkBwd_Stage1_frm[10].positions[7] = 1650;
  WalkBwd_Stage1_frm[10].positions[8] = 1633;
  WalkBwd_Stage1_frm[10].positions[9] = 1390;
  WalkBwd_Stage1_frm[10].positions[10] = 1500;
  WalkBwd_Stage1_frm[10].positions[11] = 1860;
  WalkBwd_Stage1_frm[10].positions[12] = 1590;
  WalkBwd_Stage1_frm[10].positions[13] = 1460;
  WalkBwd_Stage1_frm[10].positions[14] = 1500;
  WalkBwd_Stage1_frm[10].positions[15] = 1697;
  WalkBwd_Stage1_frm[10].positions[16] = 1400;
  WalkBwd_Stage1_frm[10].positions[17] = 1601;
  WalkBwd_Stage1_frm[10].positions[18] = 1398;
  WalkBwd_Stage1_frm[10].positions[19] = 1400;

  WalkBwd_Stage1_frm[11].positions[0] = 1500;
  WalkBwd_Stage1_frm[11].positions[1] = 1470;
  WalkBwd_Stage1_frm[11].positions[2] = 1500;
  WalkBwd_Stage1_frm[11].positions[3] = 1543;
  WalkBwd_Stage1_frm[11].positions[4] = 1499;
  WalkBwd_Stage1_frm[11].positions[5] = 1180;
  WalkBwd_Stage1_frm[11].positions[6] = 1405;
  WalkBwd_Stage1_frm[11].positions[7] = 1723;
  WalkBwd_Stage1_frm[11].positions[8] = 1793;
  WalkBwd_Stage1_frm[11].positions[9] = 1405;
  WalkBwd_Stage1_frm[11].positions[10] = 1500;
  WalkBwd_Stage1_frm[11].positions[11] = 1860;
  WalkBwd_Stage1_frm[11].positions[12] = 1590;
  WalkBwd_Stage1_frm[11].positions[13] = 1543;
  WalkBwd_Stage1_frm[11].positions[14] = 1500;
  WalkBwd_Stage1_frm[11].positions[15] = 1723;
  WalkBwd_Stage1_frm[11].positions[16] = 1410;
  WalkBwd_Stage1_frm[11].positions[17] = 1621;
  WalkBwd_Stage1_frm[11].positions[18] = 1444;
  WalkBwd_Stage1_frm[11].positions[19] = 1410;

  WalkBwd_Stage1_frm[12].positions[0] = 1500;
  WalkBwd_Stage1_frm[12].positions[1] = 1470;
  WalkBwd_Stage1_frm[12].positions[2] = 1500;
  WalkBwd_Stage1_frm[12].positions[3] = 1596;
  WalkBwd_Stage1_frm[12].positions[4] = 1499;
  WalkBwd_Stage1_frm[12].positions[5] = 1333;
  WalkBwd_Stage1_frm[12].positions[6] = 1461;
  WalkBwd_Stage1_frm[12].positions[7] = 1534;
  WalkBwd_Stage1_frm[12].positions[8] = 1750;
  WalkBwd_Stage1_frm[12].positions[9] = 1461;
  WalkBwd_Stage1_frm[12].positions[10] = 1500;
  WalkBwd_Stage1_frm[12].positions[11] = 1860;
  WalkBwd_Stage1_frm[12].positions[12] = 1590;
  WalkBwd_Stage1_frm[12].positions[13] = 1596;
  WalkBwd_Stage1_frm[12].positions[14] = 1500;
  WalkBwd_Stage1_frm[12].positions[15] = 1771;
  WalkBwd_Stage1_frm[12].positions[16] = 1455;
  WalkBwd_Stage1_frm[12].positions[17] = 1596;
  WalkBwd_Stage1_frm[12].positions[18] = 1467;
  WalkBwd_Stage1_frm[12].positions[19] = 1455;

  WalkBwd_Stage1_frm[13].positions[0] = 1500;
  WalkBwd_Stage1_frm[13].positions[1] = 1470;
  WalkBwd_Stage1_frm[13].positions[2] = 1500;
  WalkBwd_Stage1_frm[13].positions[3] = 1603;
  WalkBwd_Stage1_frm[13].positions[4] = 1500;
  WalkBwd_Stage1_frm[13].positions[5] = 1330;
  WalkBwd_Stage1_frm[13].positions[6] = 1511;
  WalkBwd_Stage1_frm[13].positions[7] = 1506;
  WalkBwd_Stage1_frm[13].positions[8] = 1736;
  WalkBwd_Stage1_frm[13].positions[9] = 1511;
  WalkBwd_Stage1_frm[13].positions[10] = 1500;
  WalkBwd_Stage1_frm[13].positions[11] = 1860;
  WalkBwd_Stage1_frm[13].positions[12] = 1590;
  WalkBwd_Stage1_frm[13].positions[13] = 1603;
  WalkBwd_Stage1_frm[13].positions[14] = 1500;
  WalkBwd_Stage1_frm[13].positions[15] = 1792;
  WalkBwd_Stage1_frm[13].positions[16] = 1505;
  WalkBwd_Stage1_frm[13].positions[17] = 1602;
  WalkBwd_Stage1_frm[13].positions[18] = 1494;
  WalkBwd_Stage1_frm[13].positions[19] = 1505;

  WalkBwd_Stage2_frm[0].positions[0] = 1500;
  WalkBwd_Stage2_frm[0].positions[1] = 1470;
  WalkBwd_Stage2_frm[0].positions[2] = 1500;
  WalkBwd_Stage2_frm[0].positions[3] = 1603;
  WalkBwd_Stage2_frm[0].positions[4] = 1500;
  WalkBwd_Stage2_frm[0].positions[5] = 1323;
  WalkBwd_Stage2_frm[0].positions[6] = 1529;
  WalkBwd_Stage2_frm[0].positions[7] = 1505;
  WalkBwd_Stage2_frm[0].positions[8] = 1728;
  WalkBwd_Stage2_frm[0].positions[9] = 1529;
  WalkBwd_Stage2_frm[0].positions[10] = 1500;
  WalkBwd_Stage2_frm[0].positions[11] = 1860;
  WalkBwd_Stage2_frm[0].positions[12] = 1590;
  WalkBwd_Stage2_frm[0].positions[13] = 1603;
  WalkBwd_Stage2_frm[0].positions[14] = 1500;
  WalkBwd_Stage2_frm[0].positions[15] = 1791;
  WalkBwd_Stage2_frm[0].positions[16] = 1522;
  WalkBwd_Stage2_frm[0].positions[17] = 1619;
  WalkBwd_Stage2_frm[0].positions[18] = 1510;
  WalkBwd_Stage2_frm[0].positions[19] = 1522;

  WalkBwd_Stage2_frm[1].positions[0] = 1500;
  WalkBwd_Stage2_frm[1].positions[1] = 1470;
  WalkBwd_Stage2_frm[1].positions[2] = 1500;
  WalkBwd_Stage2_frm[1].positions[3] = 1601;
  WalkBwd_Stage2_frm[1].positions[4] = 1500;
  WalkBwd_Stage2_frm[1].positions[5] = 1319;
  WalkBwd_Stage2_frm[1].positions[6] = 1561;
  WalkBwd_Stage2_frm[1].positions[7] = 1484;
  WalkBwd_Stage2_frm[1].positions[8] = 1704;
  WalkBwd_Stage2_frm[1].positions[9] = 1561;
  WalkBwd_Stage2_frm[1].positions[10] = 1500;
  WalkBwd_Stage2_frm[1].positions[11] = 1860;
  WalkBwd_Stage2_frm[1].positions[12] = 1590;
  WalkBwd_Stage2_frm[1].positions[13] = 1601;
  WalkBwd_Stage2_frm[1].positions[14] = 1500;
  WalkBwd_Stage2_frm[1].positions[15] = 1779;
  WalkBwd_Stage2_frm[1].positions[16] = 1555;
  WalkBwd_Stage2_frm[1].positions[17] = 1665;
  WalkBwd_Stage2_frm[1].positions[18] = 1549;
  WalkBwd_Stage2_frm[1].positions[19] = 1555;

  WalkBwd_Stage2_frm[2].positions[0] = 1500;
  WalkBwd_Stage2_frm[2].positions[1] = 1470;
  WalkBwd_Stage2_frm[2].positions[2] = 1500;
  WalkBwd_Stage2_frm[2].positions[3] = 1594;
  WalkBwd_Stage2_frm[2].positions[4] = 1500;
  WalkBwd_Stage2_frm[2].positions[5] = 1326;
  WalkBwd_Stage2_frm[2].positions[6] = 1586;
  WalkBwd_Stage2_frm[2].positions[7] = 1447;
  WalkBwd_Stage2_frm[2].positions[8] = 1673;
  WalkBwd_Stage2_frm[2].positions[9] = 1586;
  WalkBwd_Stage2_frm[2].positions[10] = 1500;
  WalkBwd_Stage2_frm[2].positions[11] = 1860;
  WalkBwd_Stage2_frm[2].positions[12] = 1590;
  WalkBwd_Stage2_frm[2].positions[13] = 1594;
  WalkBwd_Stage2_frm[2].positions[14] = 1501;
  WalkBwd_Stage2_frm[2].positions[15] = 1804;
  WalkBwd_Stage2_frm[2].positions[16] = 1582;
  WalkBwd_Stage2_frm[2].positions[17] = 1625;
  WalkBwd_Stage2_frm[2].positions[18] = 1539;
  WalkBwd_Stage2_frm[2].positions[19] = 1582;

  WalkBwd_Stage2_frm[3].positions[0] = 1500;
  WalkBwd_Stage2_frm[3].positions[1] = 1470;
  WalkBwd_Stage2_frm[3].positions[2] = 1500;
  WalkBwd_Stage2_frm[3].positions[3] = 1573;
  WalkBwd_Stage2_frm[3].positions[4] = 1500;
  WalkBwd_Stage2_frm[3].positions[5] = 1333;
  WalkBwd_Stage2_frm[3].positions[6] = 1608;
  WalkBwd_Stage2_frm[3].positions[7] = 1402;
  WalkBwd_Stage2_frm[3].positions[8] = 1635;
  WalkBwd_Stage2_frm[3].positions[9] = 1608;
  WalkBwd_Stage2_frm[3].positions[10] = 1500;
  WalkBwd_Stage2_frm[3].positions[11] = 1860;
  WalkBwd_Stage2_frm[3].positions[12] = 1590;
  WalkBwd_Stage2_frm[3].positions[13] = 1573;
  WalkBwd_Stage2_frm[3].positions[14] = 1501;
  WalkBwd_Stage2_frm[3].positions[15] = 1914;
  WalkBwd_Stage2_frm[3].positions[16] = 1609;
  WalkBwd_Stage2_frm[3].positions[17] = 1375;
  WalkBwd_Stage2_frm[3].positions[18] = 1393;
  WalkBwd_Stage2_frm[3].positions[19] = 1609;

  WalkBwd_Stage2_frm[4].positions[0] = 1500;
  WalkBwd_Stage2_frm[4].positions[1] = 1470;
  WalkBwd_Stage2_frm[4].positions[2] = 1500;
  WalkBwd_Stage2_frm[4].positions[3] = 1531;
  WalkBwd_Stage2_frm[4].positions[4] = 1500;
  WalkBwd_Stage2_frm[4].positions[5] = 1323;
  WalkBwd_Stage2_frm[4].positions[6] = 1609;
  WalkBwd_Stage2_frm[4].positions[7] = 1392;
  WalkBwd_Stage2_frm[4].positions[8] = 1615;
  WalkBwd_Stage2_frm[4].positions[9] = 1609;
  WalkBwd_Stage2_frm[4].positions[10] = 1500;
  WalkBwd_Stage2_frm[4].positions[11] = 1860;
  WalkBwd_Stage2_frm[4].positions[12] = 1590;
  WalkBwd_Stage2_frm[4].positions[13] = 1531;
  WalkBwd_Stage2_frm[4].positions[14] = 1501;
  WalkBwd_Stage2_frm[4].positions[15] = 1880;
  WalkBwd_Stage2_frm[4].positions[16] = 1606;
  WalkBwd_Stage2_frm[4].positions[17] = 1316;
  WalkBwd_Stage2_frm[4].positions[18] = 1301;
  WalkBwd_Stage2_frm[4].positions[19] = 1606;

  WalkBwd_Stage2_frm[5].positions[0] = 1500;
  WalkBwd_Stage2_frm[5].positions[1] = 1470;
  WalkBwd_Stage2_frm[5].positions[2] = 1500;
  WalkBwd_Stage2_frm[5].positions[3] = 1505;
  WalkBwd_Stage2_frm[5].positions[4] = 1500;
  WalkBwd_Stage2_frm[5].positions[5] = 1299;
  WalkBwd_Stage2_frm[5].positions[6] = 1585;
  WalkBwd_Stage2_frm[5].positions[7] = 1418;
  WalkBwd_Stage2_frm[5].positions[8] = 1617;
  WalkBwd_Stage2_frm[5].positions[9] = 1585;
  WalkBwd_Stage2_frm[5].positions[10] = 1500;
  WalkBwd_Stage2_frm[5].positions[11] = 1860;
  WalkBwd_Stage2_frm[5].positions[12] = 1590;
  WalkBwd_Stage2_frm[5].positions[13] = 1505;
  WalkBwd_Stage2_frm[5].positions[14] = 1501;
  WalkBwd_Stage2_frm[5].positions[15] = 1736;
  WalkBwd_Stage2_frm[5].positions[16] = 1572;
  WalkBwd_Stage2_frm[5].positions[17] = 1521;
  WalkBwd_Stage2_frm[5].positions[18] = 1365;
  WalkBwd_Stage2_frm[5].positions[19] = 1572;

  WalkBwd_Stage2_frm[6].positions[0] = 1500;
  WalkBwd_Stage2_frm[6].positions[1] = 1470;
  WalkBwd_Stage2_frm[6].positions[2] = 1500;
  WalkBwd_Stage2_frm[6].positions[3] = 1501;
  WalkBwd_Stage2_frm[6].positions[4] = 1500;
  WalkBwd_Stage2_frm[6].positions[5] = 1284;
  WalkBwd_Stage2_frm[6].positions[6] = 1560;
  WalkBwd_Stage2_frm[6].positions[7] = 1437;
  WalkBwd_Stage2_frm[6].positions[8] = 1621;
  WalkBwd_Stage2_frm[6].positions[9] = 1560;
  WalkBwd_Stage2_frm[6].positions[10] = 1500;
  WalkBwd_Stage2_frm[6].positions[11] = 1860;
  WalkBwd_Stage2_frm[6].positions[12] = 1590;
  WalkBwd_Stage2_frm[6].positions[13] = 1501;
  WalkBwd_Stage2_frm[6].positions[14] = 1500;
  WalkBwd_Stage2_frm[6].positions[15] = 1720;
  WalkBwd_Stage2_frm[6].positions[16] = 1546;
  WalkBwd_Stage2_frm[6].positions[17] = 1555;
  WalkBwd_Stage2_frm[6].positions[18] = 1375;
  WalkBwd_Stage2_frm[6].positions[19] = 1546;

  WalkBwd_Stage2_frm[7].positions[0] = 1500;
  WalkBwd_Stage2_frm[7].positions[1] = 1470;
  WalkBwd_Stage2_frm[7].positions[2] = 1500;
  WalkBwd_Stage2_frm[7].positions[3] = 1500;
  WalkBwd_Stage2_frm[7].positions[4] = 1500;
  WalkBwd_Stage2_frm[7].positions[5] = 1281;
  WalkBwd_Stage2_frm[7].positions[6] = 1551;
  WalkBwd_Stage2_frm[7].positions[7] = 1442;
  WalkBwd_Stage2_frm[7].positions[8] = 1622;
  WalkBwd_Stage2_frm[7].positions[9] = 1551;
  WalkBwd_Stage2_frm[7].positions[10] = 1500;
  WalkBwd_Stage2_frm[7].positions[11] = 1860;
  WalkBwd_Stage2_frm[7].positions[12] = 1590;
  WalkBwd_Stage2_frm[7].positions[13] = 1500;
  WalkBwd_Stage2_frm[7].positions[14] = 1500;
  WalkBwd_Stage2_frm[7].positions[15] = 1723;
  WalkBwd_Stage2_frm[7].positions[16] = 1537;
  WalkBwd_Stage2_frm[7].positions[17] = 1551;
  WalkBwd_Stage2_frm[7].positions[18] = 1374;
  WalkBwd_Stage2_frm[7].positions[19] = 1537;

  WalkBwd_Stage2_frm[8].positions[0] = 1500;
  WalkBwd_Stage2_frm[8].positions[1] = 1470;
  WalkBwd_Stage2_frm[8].positions[2] = 1500;
  WalkBwd_Stage2_frm[8].positions[3] = 1500;
  WalkBwd_Stage2_frm[8].positions[4] = 1500;
  WalkBwd_Stage2_frm[8].positions[5] = 1270;
  WalkBwd_Stage2_frm[8].positions[6] = 1514;
  WalkBwd_Stage2_frm[8].positions[7] = 1452;
  WalkBwd_Stage2_frm[8].positions[8] = 1621;
  WalkBwd_Stage2_frm[8].positions[9] = 1514;
  WalkBwd_Stage2_frm[8].positions[10] = 1500;
  WalkBwd_Stage2_frm[8].positions[11] = 1860;
  WalkBwd_Stage2_frm[8].positions[12] = 1590;
  WalkBwd_Stage2_frm[8].positions[13] = 1500;
  WalkBwd_Stage2_frm[8].positions[14] = 1500;
  WalkBwd_Stage2_frm[8].positions[15] = 1731;
  WalkBwd_Stage2_frm[8].positions[16] = 1500;
  WalkBwd_Stage2_frm[8].positions[17] = 1547;
  WalkBwd_Stage2_frm[8].positions[18] = 1378;
  WalkBwd_Stage2_frm[8].positions[19] = 1500;

  WalkBwd_Stage2_frm[9].positions[0] = 1500;
  WalkBwd_Stage2_frm[9].positions[1] = 1470;
  WalkBwd_Stage2_frm[9].positions[2] = 1500;
  WalkBwd_Stage2_frm[9].positions[3] = 1500;
  WalkBwd_Stage2_frm[9].positions[4] = 1500;
  WalkBwd_Stage2_frm[9].positions[5] = 1269;
  WalkBwd_Stage2_frm[9].positions[6] = 1516;
  WalkBwd_Stage2_frm[9].positions[7] = 1451;
  WalkBwd_Stage2_frm[9].positions[8] = 1620;
  WalkBwd_Stage2_frm[9].positions[9] = 1516;
  WalkBwd_Stage2_frm[9].positions[10] = 1500;
  WalkBwd_Stage2_frm[9].positions[11] = 1860;
  WalkBwd_Stage2_frm[9].positions[12] = 1590;
  WalkBwd_Stage2_frm[9].positions[13] = 1500;
  WalkBwd_Stage2_frm[9].positions[14] = 1500;
  WalkBwd_Stage2_frm[9].positions[15] = 1732;
  WalkBwd_Stage2_frm[9].positions[16] = 1502;
  WalkBwd_Stage2_frm[9].positions[17] = 1548;
  WalkBwd_Stage2_frm[9].positions[18] = 1379;
  WalkBwd_Stage2_frm[9].positions[19] = 1502;

  WalkFwd_Stage0_frm[0].positions[0] = 1500;
  WalkFwd_Stage0_frm[0].positions[1] = 1470;
  WalkFwd_Stage0_frm[0].positions[2] = 1500;
  WalkFwd_Stage0_frm[0].positions[3] = 1500;
  WalkFwd_Stage0_frm[0].positions[4] = 1500;
  WalkFwd_Stage0_frm[0].positions[5] = 1317;
  WalkFwd_Stage0_frm[0].positions[6] = 1507;
  WalkFwd_Stage0_frm[0].positions[7] = 1356;
  WalkFwd_Stage0_frm[0].positions[8] = 1572;
  WalkFwd_Stage0_frm[0].positions[9] = 1507;
  WalkFwd_Stage0_frm[0].positions[10] = 1500;
  WalkFwd_Stage0_frm[0].positions[11] = 1860;
  WalkFwd_Stage0_frm[0].positions[12] = 1590;
  WalkFwd_Stage0_frm[0].positions[13] = 1500;
  WalkFwd_Stage0_frm[0].positions[14] = 1500;
  WalkFwd_Stage0_frm[0].positions[15] = 1683;
  WalkFwd_Stage0_frm[0].positions[16] = 1493;
  WalkFwd_Stage0_frm[0].positions[17] = 1644;
  WalkFwd_Stage0_frm[0].positions[18] = 1428;
  WalkFwd_Stage0_frm[0].positions[19] = 1493;

  WalkFwd_Stage0_frm[1].positions[0] = 1500;
  WalkFwd_Stage0_frm[1].positions[1] = 1470;
  WalkFwd_Stage0_frm[1].positions[2] = 1500;
  WalkFwd_Stage0_frm[1].positions[3] = 1500;
  WalkFwd_Stage0_frm[1].positions[4] = 1500;
  WalkFwd_Stage0_frm[1].positions[5] = 1317;
  WalkFwd_Stage0_frm[1].positions[6] = 1507;
  WalkFwd_Stage0_frm[1].positions[7] = 1356;
  WalkFwd_Stage0_frm[1].positions[8] = 1572;
  WalkFwd_Stage0_frm[1].positions[9] = 1507;
  WalkFwd_Stage0_frm[1].positions[10] = 1500;
  WalkFwd_Stage0_frm[1].positions[11] = 1860;
  WalkFwd_Stage0_frm[1].positions[12] = 1590;
  WalkFwd_Stage0_frm[1].positions[13] = 1500;
  WalkFwd_Stage0_frm[1].positions[14] = 1500;
  WalkFwd_Stage0_frm[1].positions[15] = 1683;
  WalkFwd_Stage0_frm[1].positions[16] = 1493;
  WalkFwd_Stage0_frm[1].positions[17] = 1644;
  WalkFwd_Stage0_frm[1].positions[18] = 1428;
  WalkFwd_Stage0_frm[1].positions[19] = 1493;

  WalkFwd_Stage0_frm[2].positions[0] = 1500;
  WalkFwd_Stage0_frm[2].positions[1] = 1470;
  WalkFwd_Stage0_frm[2].positions[2] = 1500;
  WalkFwd_Stage0_frm[2].positions[3] = 1500;
  WalkFwd_Stage0_frm[2].positions[4] = 1500;
  WalkFwd_Stage0_frm[2].positions[5] = 1317;
  WalkFwd_Stage0_frm[2].positions[6] = 1496;
  WalkFwd_Stage0_frm[2].positions[7] = 1356;
  WalkFwd_Stage0_frm[2].positions[8] = 1573;
  WalkFwd_Stage0_frm[2].positions[9] = 1496;
  WalkFwd_Stage0_frm[2].positions[10] = 1500;
  WalkFwd_Stage0_frm[2].positions[11] = 1860;
  WalkFwd_Stage0_frm[2].positions[12] = 1590;
  WalkFwd_Stage0_frm[2].positions[13] = 1500;
  WalkFwd_Stage0_frm[2].positions[14] = 1500;
  WalkFwd_Stage0_frm[2].positions[15] = 1682;
  WalkFwd_Stage0_frm[2].positions[16] = 1482;
  WalkFwd_Stage0_frm[2].positions[17] = 1646;
  WalkFwd_Stage0_frm[2].positions[18] = 1428;
  WalkFwd_Stage0_frm[2].positions[19] = 1482;

  WalkFwd_Stage0_frm[3].positions[0] = 1500;
  WalkFwd_Stage0_frm[3].positions[1] = 1470;
  WalkFwd_Stage0_frm[3].positions[2] = 1500;
  WalkFwd_Stage0_frm[3].positions[3] = 1499;
  WalkFwd_Stage0_frm[3].positions[4] = 1500;
  WalkFwd_Stage0_frm[3].positions[5] = 1313;
  WalkFwd_Stage0_frm[3].positions[6] = 1429;
  WalkFwd_Stage0_frm[3].positions[7] = 1381;
  WalkFwd_Stage0_frm[3].positions[8] = 1594;
  WalkFwd_Stage0_frm[3].positions[9] = 1429;
  WalkFwd_Stage0_frm[3].positions[10] = 1500;
  WalkFwd_Stage0_frm[3].positions[11] = 1860;
  WalkFwd_Stage0_frm[3].positions[12] = 1590;
  WalkFwd_Stage0_frm[3].positions[13] = 1499;
  WalkFwd_Stage0_frm[3].positions[14] = 1500;
  WalkFwd_Stage0_frm[3].positions[15] = 1679;
  WalkFwd_Stage0_frm[3].positions[16] = 1416;
  WalkFwd_Stage0_frm[3].positions[17] = 1634;
  WalkFwd_Stage0_frm[3].positions[18] = 1413;
  WalkFwd_Stage0_frm[3].positions[19] = 1416;

  WalkFwd_Stage0_frm[4].positions[0] = 1500;
  WalkFwd_Stage0_frm[4].positions[1] = 1470;
  WalkFwd_Stage0_frm[4].positions[2] = 1500;
  WalkFwd_Stage0_frm[4].positions[3] = 1498;
  WalkFwd_Stage0_frm[4].positions[4] = 1500;
  WalkFwd_Stage0_frm[4].positions[5] = 1318;
  WalkFwd_Stage0_frm[4].positions[6] = 1420;
  WalkFwd_Stage0_frm[4].positions[7] = 1376;
  WalkFwd_Stage0_frm[4].positions[8] = 1594;
  WalkFwd_Stage0_frm[4].positions[9] = 1420;
  WalkFwd_Stage0_frm[4].positions[10] = 1500;
  WalkFwd_Stage0_frm[4].positions[11] = 1860;
  WalkFwd_Stage0_frm[4].positions[12] = 1590;
  WalkFwd_Stage0_frm[4].positions[13] = 1498;
  WalkFwd_Stage0_frm[4].positions[14] = 1500;
  WalkFwd_Stage0_frm[4].positions[15] = 1673;
  WalkFwd_Stage0_frm[4].positions[16] = 1407;
  WalkFwd_Stage0_frm[4].positions[17] = 1642;
  WalkFwd_Stage0_frm[4].positions[18] = 1415;
  WalkFwd_Stage0_frm[4].positions[19] = 1407;

  WalkFwd_Stage0_frm[5].positions[0] = 1500;
  WalkFwd_Stage0_frm[5].positions[1] = 1470;
  WalkFwd_Stage0_frm[5].positions[2] = 1500;
  WalkFwd_Stage0_frm[5].positions[3] = 1494;
  WalkFwd_Stage0_frm[5].positions[4] = 1498;
  WalkFwd_Stage0_frm[5].positions[5] = 1299;
  WalkFwd_Stage0_frm[5].positions[6] = 1403;
  WalkFwd_Stage0_frm[5].positions[7] = 1407;
  WalkFwd_Stage0_frm[5].positions[8] = 1594;
  WalkFwd_Stage0_frm[5].positions[9] = 1403;
  WalkFwd_Stage0_frm[5].positions[10] = 1500;
  WalkFwd_Stage0_frm[5].positions[11] = 1860;
  WalkFwd_Stage0_frm[5].positions[12] = 1590;
  WalkFwd_Stage0_frm[5].positions[13] = 1494;
  WalkFwd_Stage0_frm[5].positions[14] = 1500;
  WalkFwd_Stage0_frm[5].positions[15] = 1649;
  WalkFwd_Stage0_frm[5].positions[16] = 1391;
  WalkFwd_Stage0_frm[5].positions[17] = 1679;
  WalkFwd_Stage0_frm[5].positions[18] = 1428;
  WalkFwd_Stage0_frm[5].positions[19] = 1391;

  WalkFwd_Stage0_frm[6].positions[0] = 1500;
  WalkFwd_Stage0_frm[6].positions[1] = 1470;
  WalkFwd_Stage0_frm[6].positions[2] = 1500;
  WalkFwd_Stage0_frm[6].positions[3] = 1485;
  WalkFwd_Stage0_frm[6].positions[4] = 1494;
  WalkFwd_Stage0_frm[6].positions[5] = 1238;
  WalkFwd_Stage0_frm[6].positions[6] = 1387;
  WalkFwd_Stage0_frm[6].positions[7] = 1505;
  WalkFwd_Stage0_frm[6].positions[8] = 1608;
  WalkFwd_Stage0_frm[6].positions[9] = 1387;
  WalkFwd_Stage0_frm[6].positions[10] = 1500;
  WalkFwd_Stage0_frm[6].positions[11] = 1860;
  WalkFwd_Stage0_frm[6].positions[12] = 1590;
  WalkFwd_Stage0_frm[6].positions[13] = 1485;
  WalkFwd_Stage0_frm[6].positions[14] = 1500;
  WalkFwd_Stage0_frm[6].positions[15] = 1621;
  WalkFwd_Stage0_frm[6].positions[16] = 1379;
  WalkFwd_Stage0_frm[6].positions[17] = 1720;
  WalkFwd_Stage0_frm[6].positions[18] = 1441;
  WalkFwd_Stage0_frm[6].positions[19] = 1379;

  WalkFwd_Stage0_frm[7].positions[0] = 1500;
  WalkFwd_Stage0_frm[7].positions[1] = 1470;
  WalkFwd_Stage0_frm[7].positions[2] = 1500;
  WalkFwd_Stage0_frm[7].positions[3] = 1465;
  WalkFwd_Stage0_frm[7].positions[4] = 1491;
  WalkFwd_Stage0_frm[7].positions[5] = 1135;
  WalkFwd_Stage0_frm[7].positions[6] = 1371;
  WalkFwd_Stage0_frm[7].positions[7] = 1658;
  WalkFwd_Stage0_frm[7].positions[8] = 1650;
  WalkFwd_Stage0_frm[7].positions[9] = 1370;
  WalkFwd_Stage0_frm[7].positions[10] = 1500;
  WalkFwd_Stage0_frm[7].positions[11] = 1860;
  WalkFwd_Stage0_frm[7].positions[12] = 1590;
  WalkFwd_Stage0_frm[7].positions[13] = 1465;
  WalkFwd_Stage0_frm[7].positions[14] = 1500;
  WalkFwd_Stage0_frm[7].positions[15] = 1607;
  WalkFwd_Stage0_frm[7].positions[16] = 1372;
  WalkFwd_Stage0_frm[7].positions[17] = 1720;
  WalkFwd_Stage0_frm[7].positions[18] = 1427;
  WalkFwd_Stage0_frm[7].positions[19] = 1372;

  WalkFwd_Stage0_frm[8].positions[0] = 1500;
  WalkFwd_Stage0_frm[8].positions[1] = 1470;
  WalkFwd_Stage0_frm[8].positions[2] = 1500;
  WalkFwd_Stage0_frm[8].positions[3] = 1427;
  WalkFwd_Stage0_frm[8].positions[4] = 1499;
  WalkFwd_Stage0_frm[8].positions[5] = 1128;
  WalkFwd_Stage0_frm[8].positions[6] = 1394;
  WalkFwd_Stage0_frm[8].positions[7] = 1599;
  WalkFwd_Stage0_frm[8].positions[8] = 1623;
  WalkFwd_Stage0_frm[8].positions[9] = 1394;
  WalkFwd_Stage0_frm[8].positions[10] = 1500;
  WalkFwd_Stage0_frm[8].positions[11] = 1860;
  WalkFwd_Stage0_frm[8].positions[12] = 1590;
  WalkFwd_Stage0_frm[8].positions[13] = 1427;
  WalkFwd_Stage0_frm[8].positions[14] = 1500;
  WalkFwd_Stage0_frm[8].positions[15] = 1593;
  WalkFwd_Stage0_frm[8].positions[16] = 1395;
  WalkFwd_Stage0_frm[8].positions[17] = 1696;
  WalkFwd_Stage0_frm[8].positions[18] = 1389;
  WalkFwd_Stage0_frm[8].positions[19] = 1395;

  WalkFwd_Stage0_frm[9].positions[0] = 1500;
  WalkFwd_Stage0_frm[9].positions[1] = 1470;
  WalkFwd_Stage0_frm[9].positions[2] = 1500;
  WalkFwd_Stage0_frm[9].positions[3] = 1400;
  WalkFwd_Stage0_frm[9].positions[4] = 1501;
  WalkFwd_Stage0_frm[9].positions[5] = 1243;
  WalkFwd_Stage0_frm[9].positions[6] = 1459;
  WalkFwd_Stage0_frm[9].positions[7] = 1359;
  WalkFwd_Stage0_frm[9].positions[8] = 1515;
  WalkFwd_Stage0_frm[9].positions[9] = 1459;
  WalkFwd_Stage0_frm[9].positions[10] = 1500;
  WalkFwd_Stage0_frm[9].positions[11] = 1860;
  WalkFwd_Stage0_frm[9].positions[12] = 1590;
  WalkFwd_Stage0_frm[9].positions[13] = 1400;
  WalkFwd_Stage0_frm[9].positions[14] = 1500;
  WalkFwd_Stage0_frm[9].positions[15] = 1606;
  WalkFwd_Stage0_frm[9].positions[16] = 1454;
  WalkFwd_Stage0_frm[9].positions[17] = 1608;
  WalkFwd_Stage0_frm[9].positions[18] = 1313;
  WalkFwd_Stage0_frm[9].positions[19] = 1454;

  WalkFwd_Stage0_frm[10].positions[0] = 1500;
  WalkFwd_Stage0_frm[10].positions[1] = 1470;
  WalkFwd_Stage0_frm[10].positions[2] = 1500;
  WalkFwd_Stage0_frm[10].positions[3] = 1397;
  WalkFwd_Stage0_frm[10].positions[4] = 1500;
  WalkFwd_Stage0_frm[10].positions[5] = 1269;
  WalkFwd_Stage0_frm[10].positions[6] = 1522;
  WalkFwd_Stage0_frm[10].positions[7] = 1347;
  WalkFwd_Stage0_frm[10].positions[8] = 1516;
  WalkFwd_Stage0_frm[10].positions[9] = 1522;
  WalkFwd_Stage0_frm[10].positions[10] = 1500;
  WalkFwd_Stage0_frm[10].positions[11] = 1860;
  WalkFwd_Stage0_frm[10].positions[12] = 1590;
  WalkFwd_Stage0_frm[10].positions[13] = 1397;
  WalkFwd_Stage0_frm[10].positions[14] = 1500;
  WalkFwd_Stage0_frm[10].positions[15] = 1584;
  WalkFwd_Stage0_frm[10].positions[16] = 1515;
  WalkFwd_Stage0_frm[10].positions[17] = 1600;
  WalkFwd_Stage0_frm[10].positions[18] = 1283;
  WalkFwd_Stage0_frm[10].positions[19] = 1515;

  WalkFwd_Stage1_frm[0].positions[0] = 1500;
  WalkFwd_Stage1_frm[0].positions[1] = 1470;
  WalkFwd_Stage1_frm[0].positions[2] = 1500;
  WalkFwd_Stage1_frm[0].positions[3] = 1399;
  WalkFwd_Stage1_frm[0].positions[4] = 1500;
  WalkFwd_Stage1_frm[0].positions[5] = 1277;
  WalkFwd_Stage1_frm[0].positions[6] = 1542;
  WalkFwd_Stage1_frm[0].positions[7] = 1348;
  WalkFwd_Stage1_frm[0].positions[8] = 1525;
  WalkFwd_Stage1_frm[0].positions[9] = 1542;
  WalkFwd_Stage1_frm[0].positions[10] = 1500;
  WalkFwd_Stage1_frm[0].positions[11] = 1860;
  WalkFwd_Stage1_frm[0].positions[12] = 1590;
  WalkFwd_Stage1_frm[0].positions[13] = 1399;
  WalkFwd_Stage1_frm[0].positions[14] = 1500;
  WalkFwd_Stage1_frm[0].positions[15] = 1567;
  WalkFwd_Stage1_frm[0].positions[16] = 1536;
  WalkFwd_Stage1_frm[0].positions[17] = 1617;
  WalkFwd_Stage1_frm[0].positions[18] = 1284;
  WalkFwd_Stage1_frm[0].positions[19] = 1536;

  WalkFwd_Stage1_frm[1].positions[0] = 1500;
  WalkFwd_Stage1_frm[1].positions[1] = 1470;
  WalkFwd_Stage1_frm[1].positions[2] = 1500;
  WalkFwd_Stage1_frm[1].positions[3] = 1407;
  WalkFwd_Stage1_frm[1].positions[4] = 1500;
  WalkFwd_Stage1_frm[1].positions[5] = 1307;
  WalkFwd_Stage1_frm[1].positions[6] = 1578;
  WalkFwd_Stage1_frm[1].positions[7] = 1324;
  WalkFwd_Stage1_frm[1].positions[8] = 1531;
  WalkFwd_Stage1_frm[1].positions[9] = 1578;
  WalkFwd_Stage1_frm[1].positions[10] = 1500;
  WalkFwd_Stage1_frm[1].positions[11] = 1860;
  WalkFwd_Stage1_frm[1].positions[12] = 1590;
  WalkFwd_Stage1_frm[1].positions[13] = 1407;
  WalkFwd_Stage1_frm[1].positions[14] = 1503;
  WalkFwd_Stage1_frm[1].positions[15] = 1595;
  WalkFwd_Stage1_frm[1].positions[16] = 1573;
  WalkFwd_Stage1_frm[1].positions[17] = 1556;
  WalkFwd_Stage1_frm[1].positions[18] = 1276;
  WalkFwd_Stage1_frm[1].positions[19] = 1573;

  WalkFwd_Stage1_frm[2].positions[0] = 1500;
  WalkFwd_Stage1_frm[2].positions[1] = 1470;
  WalkFwd_Stage1_frm[2].positions[2] = 1500;
  WalkFwd_Stage1_frm[2].positions[3] = 1424;
  WalkFwd_Stage1_frm[2].positions[4] = 1500;
  WalkFwd_Stage1_frm[2].positions[5] = 1345;
  WalkFwd_Stage1_frm[2].positions[6] = 1605;
  WalkFwd_Stage1_frm[2].positions[7] = 1284;
  WalkFwd_Stage1_frm[2].positions[8] = 1529;
  WalkFwd_Stage1_frm[2].positions[9] = 1605;
  WalkFwd_Stage1_frm[2].positions[10] = 1500;
  WalkFwd_Stage1_frm[2].positions[11] = 1860;
  WalkFwd_Stage1_frm[2].positions[12] = 1590;
  WalkFwd_Stage1_frm[2].positions[13] = 1424;
  WalkFwd_Stage1_frm[2].positions[14] = 1511;
  WalkFwd_Stage1_frm[2].positions[15] = 1679;
  WalkFwd_Stage1_frm[2].positions[16] = 1604;
  WalkFwd_Stage1_frm[2].positions[17] = 1424;
  WalkFwd_Stage1_frm[2].positions[18] = 1272;
  WalkFwd_Stage1_frm[2].positions[19] = 1604;

  WalkFwd_Stage1_frm[3].positions[0] = 1500;
  WalkFwd_Stage1_frm[3].positions[1] = 1470;
  WalkFwd_Stage1_frm[3].positions[2] = 1500;
  WalkFwd_Stage1_frm[3].positions[3] = 1466;
  WalkFwd_Stage1_frm[3].positions[4] = 1500;
  WalkFwd_Stage1_frm[3].positions[5] = 1377;
  WalkFwd_Stage1_frm[3].positions[6] = 1622;
  WalkFwd_Stage1_frm[3].positions[7] = 1273;
  WalkFwd_Stage1_frm[3].positions[8] = 1550;
  WalkFwd_Stage1_frm[3].positions[9] = 1622;
  WalkFwd_Stage1_frm[3].positions[10] = 1500;
  WalkFwd_Stage1_frm[3].positions[11] = 1860;
  WalkFwd_Stage1_frm[3].positions[12] = 1590;
  WalkFwd_Stage1_frm[3].positions[13] = 1466;
  WalkFwd_Stage1_frm[3].positions[14] = 1518;
  WalkFwd_Stage1_frm[3].positions[15] = 1785;
  WalkFwd_Stage1_frm[3].positions[16] = 1631;
  WalkFwd_Stage1_frm[3].positions[17] = 1305;
  WalkFwd_Stage1_frm[3].positions[18] = 1275;
  WalkFwd_Stage1_frm[3].positions[19] = 1632;

  WalkFwd_Stage1_frm[4].positions[0] = 1500;
  WalkFwd_Stage1_frm[4].positions[1] = 1470;
  WalkFwd_Stage1_frm[4].positions[2] = 1500;
  WalkFwd_Stage1_frm[4].positions[3] = 1541;
  WalkFwd_Stage1_frm[4].positions[4] = 1500;
  WalkFwd_Stage1_frm[4].positions[5] = 1393;
  WalkFwd_Stage1_frm[4].positions[6] = 1602;
  WalkFwd_Stage1_frm[4].positions[7] = 1313;
  WalkFwd_Stage1_frm[4].positions[8] = 1606;
  WalkFwd_Stage1_frm[4].positions[9] = 1602;
  WalkFwd_Stage1_frm[4].positions[10] = 1500;
  WalkFwd_Stage1_frm[4].positions[11] = 1860;
  WalkFwd_Stage1_frm[4].positions[12] = 1590;
  WalkFwd_Stage1_frm[4].positions[13] = 1541;
  WalkFwd_Stage1_frm[4].positions[14] = 1501;
  WalkFwd_Stage1_frm[4].positions[15] = 1846;
  WalkFwd_Stage1_frm[4].positions[16] = 1612;
  WalkFwd_Stage1_frm[4].positions[17] = 1366;
  WalkFwd_Stage1_frm[4].positions[18] = 1320;
  WalkFwd_Stage1_frm[4].positions[19] = 1612;

  WalkFwd_Stage1_frm[5].positions[0] = 1500;
  WalkFwd_Stage1_frm[5].positions[1] = 1470;
  WalkFwd_Stage1_frm[5].positions[2] = 1500;
  WalkFwd_Stage1_frm[5].positions[3] = 1594;
  WalkFwd_Stage1_frm[5].positions[4] = 1500;
  WalkFwd_Stage1_frm[5].positions[5] = 1389;
  WalkFwd_Stage1_frm[5].positions[6] = 1539;
  WalkFwd_Stage1_frm[5].positions[7] = 1396;
  WalkFwd_Stage1_frm[5].positions[8] = 1684;
  WalkFwd_Stage1_frm[5].positions[9] = 1539;
  WalkFwd_Stage1_frm[5].positions[10] = 1500;
  WalkFwd_Stage1_frm[5].positions[11] = 1860;
  WalkFwd_Stage1_frm[5].positions[12] = 1590;
  WalkFwd_Stage1_frm[5].positions[13] = 1594;
  WalkFwd_Stage1_frm[5].positions[14] = 1498;
  WalkFwd_Stage1_frm[5].positions[15] = 1777;
  WalkFwd_Stage1_frm[5].positions[16] = 1541;
  WalkFwd_Stage1_frm[5].positions[17] = 1599;
  WalkFwd_Stage1_frm[5].positions[18] = 1449;
  WalkFwd_Stage1_frm[5].positions[19] = 1541;

  WalkFwd_Stage1_frm[6].positions[0] = 1500;
  WalkFwd_Stage1_frm[6].positions[1] = 1470;
  WalkFwd_Stage1_frm[6].positions[2] = 1500;
  WalkFwd_Stage1_frm[6].positions[3] = 1602;
  WalkFwd_Stage1_frm[6].positions[4] = 1500;
  WalkFwd_Stage1_frm[6].positions[5] = 1418;
  WalkFwd_Stage1_frm[6].positions[6] = 1475;
  WalkFwd_Stage1_frm[6].positions[7] = 1392;
  WalkFwd_Stage1_frm[6].positions[8] = 1710;
  WalkFwd_Stage1_frm[6].positions[9] = 1475;
  WalkFwd_Stage1_frm[6].positions[10] = 1500;
  WalkFwd_Stage1_frm[6].positions[11] = 1860;
  WalkFwd_Stage1_frm[6].positions[12] = 1590;
  WalkFwd_Stage1_frm[6].positions[13] = 1602;
  WalkFwd_Stage1_frm[6].positions[14] = 1500;
  WalkFwd_Stage1_frm[6].positions[15] = 1726;
  WalkFwd_Stage1_frm[6].positions[16] = 1476;
  WalkFwd_Stage1_frm[6].positions[17] = 1668;
  WalkFwd_Stage1_frm[6].positions[18] = 1493;
  WalkFwd_Stage1_frm[6].positions[19] = 1476;

  WalkFwd_Stage1_frm[7].positions[0] = 1500;
  WalkFwd_Stage1_frm[7].positions[1] = 1470;
  WalkFwd_Stage1_frm[7].positions[2] = 1500;
  WalkFwd_Stage1_frm[7].positions[3] = 1601;
  WalkFwd_Stage1_frm[7].positions[4] = 1500;
  WalkFwd_Stage1_frm[7].positions[5] = 1440;
  WalkFwd_Stage1_frm[7].positions[6] = 1453;
  WalkFwd_Stage1_frm[7].positions[7] = 1365;
  WalkFwd_Stage1_frm[7].positions[8] = 1705;
  WalkFwd_Stage1_frm[7].positions[9] = 1453;
  WalkFwd_Stage1_frm[7].positions[10] = 1500;
  WalkFwd_Stage1_frm[7].positions[11] = 1860;
  WalkFwd_Stage1_frm[7].positions[12] = 1590;
  WalkFwd_Stage1_frm[7].positions[13] = 1601;
  WalkFwd_Stage1_frm[7].positions[14] = 1500;
  WalkFwd_Stage1_frm[7].positions[15] = 1714;
  WalkFwd_Stage1_frm[7].positions[16] = 1455;
  WalkFwd_Stage1_frm[7].positions[17] = 1673;
  WalkFwd_Stage1_frm[7].positions[18] = 1487;
  WalkFwd_Stage1_frm[7].positions[19] = 1455;

  WalkFwd_Stage1_frm[8].positions[0] = 1500;
  WalkFwd_Stage1_frm[8].positions[1] = 1470;
  WalkFwd_Stage1_frm[8].positions[2] = 1500;
  WalkFwd_Stage1_frm[8].positions[3] = 1593;
  WalkFwd_Stage1_frm[8].positions[4] = 1497;
  WalkFwd_Stage1_frm[8].positions[5] = 1414;
  WalkFwd_Stage1_frm[8].positions[6] = 1415;
  WalkFwd_Stage1_frm[8].positions[7] = 1423;
  WalkFwd_Stage1_frm[8].positions[8] = 1712;
  WalkFwd_Stage1_frm[8].positions[9] = 1415;
  WalkFwd_Stage1_frm[8].positions[10] = 1500;
  WalkFwd_Stage1_frm[8].positions[11] = 1860;
  WalkFwd_Stage1_frm[8].positions[12] = 1590;
  WalkFwd_Stage1_frm[8].positions[13] = 1593;
  WalkFwd_Stage1_frm[8].positions[14] = 1500;
  WalkFwd_Stage1_frm[8].positions[15] = 1681;
  WalkFwd_Stage1_frm[8].positions[16] = 1418;
  WalkFwd_Stage1_frm[8].positions[17] = 1701;
  WalkFwd_Stage1_frm[8].positions[18] = 1483;
  WalkFwd_Stage1_frm[8].positions[19] = 1418;

  WalkFwd_Stage1_frm[9].positions[0] = 1500;
  WalkFwd_Stage1_frm[9].positions[1] = 1470;
  WalkFwd_Stage1_frm[9].positions[2] = 1500;
  WalkFwd_Stage1_frm[9].positions[3] = 1576;
  WalkFwd_Stage1_frm[9].positions[4] = 1487;
  WalkFwd_Stage1_frm[9].positions[5] = 1325;
  WalkFwd_Stage1_frm[9].positions[6] = 1385;
  WalkFwd_Stage1_frm[9].positions[7] = 1566;
  WalkFwd_Stage1_frm[9].positions[8] = 1722;
  WalkFwd_Stage1_frm[9].positions[9] = 1384;
  WalkFwd_Stage1_frm[9].positions[10] = 1500;
  WalkFwd_Stage1_frm[9].positions[11] = 1860;
  WalkFwd_Stage1_frm[9].positions[12] = 1590;
  WalkFwd_Stage1_frm[9].positions[13] = 1576;
  WalkFwd_Stage1_frm[9].positions[14] = 1500;
  WalkFwd_Stage1_frm[9].positions[15] = 1651;
  WalkFwd_Stage1_frm[9].positions[16] = 1392;
  WalkFwd_Stage1_frm[9].positions[17] = 1725;
  WalkFwd_Stage1_frm[9].positions[18] = 1477;
  WalkFwd_Stage1_frm[9].positions[19] = 1392;

  WalkFwd_Stage1_frm[10].positions[0] = 1500;
  WalkFwd_Stage1_frm[10].positions[1] = 1470;
  WalkFwd_Stage1_frm[10].positions[2] = 1500;
  WalkFwd_Stage1_frm[10].positions[3] = 1534;
  WalkFwd_Stage1_frm[10].positions[4] = 1481;
  WalkFwd_Stage1_frm[10].positions[5] = 1216;
  WalkFwd_Stage1_frm[10].positions[6] = 1362;
  WalkFwd_Stage1_frm[10].positions[7] = 1691;
  WalkFwd_Stage1_frm[10].positions[8] = 1722;
  WalkFwd_Stage1_frm[10].positions[9] = 1361;
  WalkFwd_Stage1_frm[10].positions[10] = 1500;
  WalkFwd_Stage1_frm[10].positions[11] = 1860;
  WalkFwd_Stage1_frm[10].positions[12] = 1590;
  WalkFwd_Stage1_frm[10].positions[13] = 1534;
  WalkFwd_Stage1_frm[10].positions[14] = 1500;
  WalkFwd_Stage1_frm[10].positions[15] = 1623;
  WalkFwd_Stage1_frm[10].positions[16] = 1375;
  WalkFwd_Stage1_frm[10].positions[17] = 1729;
  WalkFwd_Stage1_frm[10].positions[18] = 1452;
  WalkFwd_Stage1_frm[10].positions[19] = 1375;

  WalkFwd_Stage1_frm[11].positions[0] = 1500;
  WalkFwd_Stage1_frm[11].positions[1] = 1470;
  WalkFwd_Stage1_frm[11].positions[2] = 1500;
  WalkFwd_Stage1_frm[11].positions[3] = 1459;
  WalkFwd_Stage1_frm[11].positions[4] = 1499;
  WalkFwd_Stage1_frm[11].positions[5] = 1152;
  WalkFwd_Stage1_frm[11].positions[6] = 1392;
  WalkFwd_Stage1_frm[11].positions[7] = 1634;
  WalkFwd_Stage1_frm[11].positions[8] = 1678;
  WalkFwd_Stage1_frm[11].positions[9] = 1392;
  WalkFwd_Stage1_frm[11].positions[10] = 1500;
  WalkFwd_Stage1_frm[11].positions[11] = 1860;
  WalkFwd_Stage1_frm[11].positions[12] = 1590;
  WalkFwd_Stage1_frm[11].positions[13] = 1459;
  WalkFwd_Stage1_frm[11].positions[14] = 1500;
  WalkFwd_Stage1_frm[11].positions[15] = 1605;
  WalkFwd_Stage1_frm[11].positions[16] = 1397;
  WalkFwd_Stage1_frm[11].positions[17] = 1694;
  WalkFwd_Stage1_frm[11].positions[18] = 1399;
  WalkFwd_Stage1_frm[11].positions[19] = 1397;

  WalkFwd_Stage1_frm[12].positions[0] = 1500;
  WalkFwd_Stage1_frm[12].positions[1] = 1470;
  WalkFwd_Stage1_frm[12].positions[2] = 1500;
  WalkFwd_Stage1_frm[12].positions[3] = 1405;
  WalkFwd_Stage1_frm[12].positions[4] = 1501;
  WalkFwd_Stage1_frm[12].positions[5] = 1227;
  WalkFwd_Stage1_frm[12].positions[6] = 1468;
  WalkFwd_Stage1_frm[12].positions[7] = 1384;
  WalkFwd_Stage1_frm[12].positions[8] = 1538;
  WalkFwd_Stage1_frm[12].positions[9] = 1468;
  WalkFwd_Stage1_frm[12].positions[10] = 1500;
  WalkFwd_Stage1_frm[12].positions[11] = 1860;
  WalkFwd_Stage1_frm[12].positions[12] = 1590;
  WalkFwd_Stage1_frm[12].positions[13] = 1405;
  WalkFwd_Stage1_frm[12].positions[14] = 1500;
  WalkFwd_Stage1_frm[12].positions[15] = 1611;
  WalkFwd_Stage1_frm[12].positions[16] = 1461;
  WalkFwd_Stage1_frm[12].positions[17] = 1615;
  WalkFwd_Stage1_frm[12].positions[18] = 1326;
  WalkFwd_Stage1_frm[12].positions[19] = 1461;

  WalkFwd_Stage1_frm[13].positions[0] = 1500;
  WalkFwd_Stage1_frm[13].positions[1] = 1470;
  WalkFwd_Stage1_frm[13].positions[2] = 1500;
  WalkFwd_Stage1_frm[13].positions[3] = 1397;
  WalkFwd_Stage1_frm[13].positions[4] = 1500;
  WalkFwd_Stage1_frm[13].positions[5] = 1271;
  WalkFwd_Stage1_frm[13].positions[6] = 1532;
  WalkFwd_Stage1_frm[13].positions[7] = 1321;
  WalkFwd_Stage1_frm[13].positions[8] = 1492;
  WalkFwd_Stage1_frm[13].positions[9] = 1532;
  WalkFwd_Stage1_frm[13].positions[10] = 1500;
  WalkFwd_Stage1_frm[13].positions[11] = 1860;
  WalkFwd_Stage1_frm[13].positions[12] = 1590;
  WalkFwd_Stage1_frm[13].positions[13] = 1397;
  WalkFwd_Stage1_frm[13].positions[14] = 1500;
  WalkFwd_Stage1_frm[13].positions[15] = 1598;
  WalkFwd_Stage1_frm[13].positions[16] = 1525;
  WalkFwd_Stage1_frm[13].positions[17] = 1593;
  WalkFwd_Stage1_frm[13].positions[18] = 1292;
  WalkFwd_Stage1_frm[13].positions[19] = 1525;

  WalkFwd_Stage2_frm[0].positions[0] = 1500;
  WalkFwd_Stage2_frm[0].positions[1] = 1470;
  WalkFwd_Stage2_frm[0].positions[2] = 1500;
  WalkFwd_Stage2_frm[0].positions[3] = 1397;
  WalkFwd_Stage2_frm[0].positions[4] = 1500;
  WalkFwd_Stage2_frm[0].positions[5] = 1280;
  WalkFwd_Stage2_frm[0].positions[6] = 1553;
  WalkFwd_Stage2_frm[0].positions[7] = 1316;
  WalkFwd_Stage2_frm[0].positions[8] = 1496;
  WalkFwd_Stage2_frm[0].positions[9] = 1553;
  WalkFwd_Stage2_frm[0].positions[10] = 1500;
  WalkFwd_Stage2_frm[0].positions[11] = 1860;
  WalkFwd_Stage2_frm[0].positions[12] = 1590;
  WalkFwd_Stage2_frm[0].positions[13] = 1397;
  WalkFwd_Stage2_frm[0].positions[14] = 1500;
  WalkFwd_Stage2_frm[0].positions[15] = 1583;
  WalkFwd_Stage2_frm[0].positions[16] = 1546;
  WalkFwd_Stage2_frm[0].positions[17] = 1611;
  WalkFwd_Stage2_frm[0].positions[18] = 1294;
  WalkFwd_Stage2_frm[0].positions[19] = 1546;

  WalkFwd_Stage2_frm[1].positions[0] = 1500;
  WalkFwd_Stage2_frm[1].positions[1] = 1470;
  WalkFwd_Stage2_frm[1].positions[2] = 1500;
  WalkFwd_Stage2_frm[1].positions[3] = 1401;
  WalkFwd_Stage2_frm[1].positions[4] = 1500;
  WalkFwd_Stage2_frm[1].positions[5] = 1313;
  WalkFwd_Stage2_frm[1].positions[6] = 1589;
  WalkFwd_Stage2_frm[1].positions[7] = 1276;
  WalkFwd_Stage2_frm[1].positions[8] = 1489;
  WalkFwd_Stage2_frm[1].positions[9] = 1589;
  WalkFwd_Stage2_frm[1].positions[10] = 1500;
  WalkFwd_Stage2_frm[1].positions[11] = 1860;
  WalkFwd_Stage2_frm[1].positions[12] = 1590;
  WalkFwd_Stage2_frm[1].positions[13] = 1401;
  WalkFwd_Stage2_frm[1].positions[14] = 1502;
  WalkFwd_Stage2_frm[1].positions[15] = 1587;
  WalkFwd_Stage2_frm[1].positions[16] = 1584;
  WalkFwd_Stage2_frm[1].positions[17] = 1593;
  WalkFwd_Stage2_frm[1].positions[18] = 1293;
  WalkFwd_Stage2_frm[1].positions[19] = 1584;

  WalkFwd_Stage2_frm[2].positions[0] = 1500;
  WalkFwd_Stage2_frm[2].positions[1] = 1470;
  WalkFwd_Stage2_frm[2].positions[2] = 1500;
  WalkFwd_Stage2_frm[2].positions[3] = 1410;
  WalkFwd_Stage2_frm[2].positions[4] = 1500;
  WalkFwd_Stage2_frm[2].positions[5] = 1342;
  WalkFwd_Stage2_frm[2].positions[6] = 1617;
  WalkFwd_Stage2_frm[2].positions[7] = 1240;
  WalkFwd_Stage2_frm[2].positions[8] = 1482;
  WalkFwd_Stage2_frm[2].positions[9] = 1617;
  WalkFwd_Stage2_frm[2].positions[10] = 1500;
  WalkFwd_Stage2_frm[2].positions[11] = 1860;
  WalkFwd_Stage2_frm[2].positions[12] = 1590;
  WalkFwd_Stage2_frm[2].positions[13] = 1410;
  WalkFwd_Stage2_frm[2].positions[14] = 1506;
  WalkFwd_Stage2_frm[2].positions[15] = 1648;
  WalkFwd_Stage2_frm[2].positions[16] = 1615;
  WalkFwd_Stage2_frm[2].positions[17] = 1484;
  WalkFwd_Stage2_frm[2].positions[18] = 1267;
  WalkFwd_Stage2_frm[2].positions[19] = 1615;

  WalkFwd_Stage2_frm[3].positions[0] = 1500;
  WalkFwd_Stage2_frm[3].positions[1] = 1470;
  WalkFwd_Stage2_frm[3].positions[2] = 1500;
  WalkFwd_Stage2_frm[3].positions[3] = 1430;
  WalkFwd_Stage2_frm[3].positions[4] = 1500;
  WalkFwd_Stage2_frm[3].positions[5] = 1358;
  WalkFwd_Stage2_frm[3].positions[6] = 1637;
  WalkFwd_Stage2_frm[3].positions[7] = 1233;
  WalkFwd_Stage2_frm[3].positions[8] = 1491;
  WalkFwd_Stage2_frm[3].positions[9] = 1637;
  WalkFwd_Stage2_frm[3].positions[10] = 1500;
  WalkFwd_Stage2_frm[3].positions[11] = 1860;
  WalkFwd_Stage2_frm[3].positions[12] = 1590;
  WalkFwd_Stage2_frm[3].positions[13] = 1430;
  WalkFwd_Stage2_frm[3].positions[14] = 1510;
  WalkFwd_Stage2_frm[3].positions[15] = 1753;
  WalkFwd_Stage2_frm[3].positions[16] = 1643;
  WalkFwd_Stage2_frm[3].positions[17] = 1319;
  WalkFwd_Stage2_frm[3].positions[18] = 1214;
  WalkFwd_Stage2_frm[3].positions[19] = 1643;

  WalkFwd_Stage2_frm[4].positions[0] = 1500;
  WalkFwd_Stage2_frm[4].positions[1] = 1470;
  WalkFwd_Stage2_frm[4].positions[2] = 1500;
  WalkFwd_Stage2_frm[4].positions[3] = 1468;
  WalkFwd_Stage2_frm[4].positions[4] = 1500;
  WalkFwd_Stage2_frm[4].positions[5] = 1359;
  WalkFwd_Stage2_frm[4].positions[6] = 1631;
  WalkFwd_Stage2_frm[4].positions[7] = 1252;
  WalkFwd_Stage2_frm[4].positions[8] = 1511;
  WalkFwd_Stage2_frm[4].positions[9] = 1631;
  WalkFwd_Stage2_frm[4].positions[10] = 1500;
  WalkFwd_Stage2_frm[4].positions[11] = 1860;
  WalkFwd_Stage2_frm[4].positions[12] = 1590;
  WalkFwd_Stage2_frm[4].positions[13] = 1468;
  WalkFwd_Stage2_frm[4].positions[14] = 1501;
  WalkFwd_Stage2_frm[4].positions[15] = 1795;
  WalkFwd_Stage2_frm[4].positions[16] = 1629;
  WalkFwd_Stage2_frm[4].positions[17] = 1347;
  WalkFwd_Stage2_frm[4].positions[18] = 1245;
  WalkFwd_Stage2_frm[4].positions[19] = 1629;

  WalkFwd_Stage2_frm[5].positions[0] = 1500;
  WalkFwd_Stage2_frm[5].positions[1] = 1470;
  WalkFwd_Stage2_frm[5].positions[2] = 1500;
  WalkFwd_Stage2_frm[5].positions[3] = 1495;
  WalkFwd_Stage2_frm[5].positions[4] = 1500;
  WalkFwd_Stage2_frm[5].positions[5] = 1340;
  WalkFwd_Stage2_frm[5].positions[6] = 1596;
  WalkFwd_Stage2_frm[5].positions[7] = 1300;
  WalkFwd_Stage2_frm[5].positions[8] = 1540;
  WalkFwd_Stage2_frm[5].positions[9] = 1596;
  WalkFwd_Stage2_frm[5].positions[10] = 1500;
  WalkFwd_Stage2_frm[5].positions[11] = 1860;
  WalkFwd_Stage2_frm[5].positions[12] = 1590;
  WalkFwd_Stage2_frm[5].positions[13] = 1495;
  WalkFwd_Stage2_frm[5].positions[14] = 1498;
  WalkFwd_Stage2_frm[5].positions[15] = 1705;
  WalkFwd_Stage2_frm[5].positions[16] = 1583;
  WalkFwd_Stage2_frm[5].positions[17] = 1600;
  WalkFwd_Stage2_frm[5].positions[18] = 1392;
  WalkFwd_Stage2_frm[5].positions[19] = 1583;

  WalkFwd_Stage2_frm[6].positions[0] = 1500;
  WalkFwd_Stage2_frm[6].positions[1] = 1470;
  WalkFwd_Stage2_frm[6].positions[2] = 1500;
  WalkFwd_Stage2_frm[6].positions[3] = 1499;
  WalkFwd_Stage2_frm[6].positions[4] = 1500;
  WalkFwd_Stage2_frm[6].positions[5] = 1316;
  WalkFwd_Stage2_frm[6].positions[6] = 1563;
  WalkFwd_Stage2_frm[6].positions[7] = 1349;
  WalkFwd_Stage2_frm[6].positions[8] = 1566;
  WalkFwd_Stage2_frm[6].positions[9] = 1563;
  WalkFwd_Stage2_frm[6].positions[10] = 1500;
  WalkFwd_Stage2_frm[6].positions[11] = 1860;
  WalkFwd_Stage2_frm[6].positions[12] = 1590;
  WalkFwd_Stage2_frm[6].positions[13] = 1499;
  WalkFwd_Stage2_frm[6].positions[14] = 1500;
  WalkFwd_Stage2_frm[6].positions[15] = 1690;
  WalkFwd_Stage2_frm[6].positions[16] = 1550;
  WalkFwd_Stage2_frm[6].positions[17] = 1639;
  WalkFwd_Stage2_frm[6].positions[18] = 1428;
  WalkFwd_Stage2_frm[6].positions[19] = 1550;

  WalkFwd_Stage2_frm[7].positions[0] = 1500;
  WalkFwd_Stage2_frm[7].positions[1] = 1470;
  WalkFwd_Stage2_frm[7].positions[2] = 1500;
  WalkFwd_Stage2_frm[7].positions[3] = 1500;
  WalkFwd_Stage2_frm[7].positions[4] = 1500;
  WalkFwd_Stage2_frm[7].positions[5] = 1314;
  WalkFwd_Stage2_frm[7].positions[6] = 1554;
  WalkFwd_Stage2_frm[7].positions[7] = 1354;
  WalkFwd_Stage2_frm[7].positions[8] = 1569;
  WalkFwd_Stage2_frm[7].positions[9] = 1554;
  WalkFwd_Stage2_frm[7].positions[10] = 1500;
  WalkFwd_Stage2_frm[7].positions[11] = 1860;
  WalkFwd_Stage2_frm[7].positions[12] = 1590;
  WalkFwd_Stage2_frm[7].positions[13] = 1500;
  WalkFwd_Stage2_frm[7].positions[14] = 1500;
  WalkFwd_Stage2_frm[7].positions[15] = 1691;
  WalkFwd_Stage2_frm[7].positions[16] = 1540;
  WalkFwd_Stage2_frm[7].positions[17] = 1636;
  WalkFwd_Stage2_frm[7].positions[18] = 1427;
  WalkFwd_Stage2_frm[7].positions[19] = 1540;

  WalkFwd_Stage2_frm[8].positions[0] = 1500;
  WalkFwd_Stage2_frm[8].positions[1] = 1470;
  WalkFwd_Stage2_frm[8].positions[2] = 1500;
  WalkFwd_Stage2_frm[8].positions[3] = 1500;
  WalkFwd_Stage2_frm[8].positions[4] = 1500;
  WalkFwd_Stage2_frm[8].positions[5] = 1316;
  WalkFwd_Stage2_frm[8].positions[6] = 1514;
  WalkFwd_Stage2_frm[8].positions[7] = 1354;
  WalkFwd_Stage2_frm[8].positions[8] = 1570;
  WalkFwd_Stage2_frm[8].positions[9] = 1514;
  WalkFwd_Stage2_frm[8].positions[10] = 1500;
  WalkFwd_Stage2_frm[8].positions[11] = 1860;
  WalkFwd_Stage2_frm[8].positions[12] = 1590;
  WalkFwd_Stage2_frm[8].positions[13] = 1500;
  WalkFwd_Stage2_frm[8].positions[14] = 1500;
  WalkFwd_Stage2_frm[8].positions[15] = 1685;
  WalkFwd_Stage2_frm[8].positions[16] = 1500;
  WalkFwd_Stage2_frm[8].positions[17] = 1645;
  WalkFwd_Stage2_frm[8].positions[18] = 1429;
  WalkFwd_Stage2_frm[8].positions[19] = 1500;

  WalkFwd_Stage2_frm[9].positions[0] = 1500;
  WalkFwd_Stage2_frm[9].positions[1] = 1470;
  WalkFwd_Stage2_frm[9].positions[2] = 1500;
  WalkFwd_Stage2_frm[9].positions[3] = 1500;
  WalkFwd_Stage2_frm[9].positions[4] = 1500;
  WalkFwd_Stage2_frm[9].positions[5] = 1317;
  WalkFwd_Stage2_frm[9].positions[6] = 1516;
  WalkFwd_Stage2_frm[9].positions[7] = 1354;
  WalkFwd_Stage2_frm[9].positions[8] = 1571;
  WalkFwd_Stage2_frm[9].positions[9] = 1516;
  WalkFwd_Stage2_frm[9].positions[10] = 1500;
  WalkFwd_Stage2_frm[9].positions[11] = 1860;
  WalkFwd_Stage2_frm[9].positions[12] = 1590;
  WalkFwd_Stage2_frm[9].positions[13] = 1500;
  WalkFwd_Stage2_frm[9].positions[14] = 1500;
  WalkFwd_Stage2_frm[9].positions[15] = 1684;
  WalkFwd_Stage2_frm[9].positions[16] = 1503;
  WalkFwd_Stage2_frm[9].positions[17] = 1644;
  WalkFwd_Stage2_frm[9].positions[18] = 1428;
  WalkFwd_Stage2_frm[9].positions[19] = 1503;

  WalkLeft_Stage0_frm[0].positions[0] = 1500;
  WalkLeft_Stage0_frm[0].positions[1] = 1470;
  WalkLeft_Stage0_frm[0].positions[2] = 1500;
  WalkLeft_Stage0_frm[0].positions[3] = 1500;
  WalkLeft_Stage0_frm[0].positions[4] = 1500;
  WalkLeft_Stage0_frm[0].positions[5] = 1317;
  WalkLeft_Stage0_frm[0].positions[6] = 1507;
  WalkLeft_Stage0_frm[0].positions[7] = 1356;
  WalkLeft_Stage0_frm[0].positions[8] = 1572;
  WalkLeft_Stage0_frm[0].positions[9] = 1507;
  WalkLeft_Stage0_frm[0].positions[10] = 1500;
  WalkLeft_Stage0_frm[0].positions[11] = 1860;
  WalkLeft_Stage0_frm[0].positions[12] = 1590;
  WalkLeft_Stage0_frm[0].positions[13] = 1500;
  WalkLeft_Stage0_frm[0].positions[14] = 1500;
  WalkLeft_Stage0_frm[0].positions[15] = 1683;
  WalkLeft_Stage0_frm[0].positions[16] = 1493;
  WalkLeft_Stage0_frm[0].positions[17] = 1644;
  WalkLeft_Stage0_frm[0].positions[18] = 1428;
  WalkLeft_Stage0_frm[0].positions[19] = 1493;

  WalkLeft_Stage0_frm[1].positions[0] = 1500;
  WalkLeft_Stage0_frm[1].positions[1] = 1470;
  WalkLeft_Stage0_frm[1].positions[2] = 1500;
  WalkLeft_Stage0_frm[1].positions[3] = 1500;
  WalkLeft_Stage0_frm[1].positions[4] = 1500;
  WalkLeft_Stage0_frm[1].positions[5] = 1317;
  WalkLeft_Stage0_frm[1].positions[6] = 1507;
  WalkLeft_Stage0_frm[1].positions[7] = 1356;
  WalkLeft_Stage0_frm[1].positions[8] = 1572;
  WalkLeft_Stage0_frm[1].positions[9] = 1507;
  WalkLeft_Stage0_frm[1].positions[10] = 1500;
  WalkLeft_Stage0_frm[1].positions[11] = 1860;
  WalkLeft_Stage0_frm[1].positions[12] = 1590;
  WalkLeft_Stage0_frm[1].positions[13] = 1500;
  WalkLeft_Stage0_frm[1].positions[14] = 1500;
  WalkLeft_Stage0_frm[1].positions[15] = 1683;
  WalkLeft_Stage0_frm[1].positions[16] = 1493;
  WalkLeft_Stage0_frm[1].positions[17] = 1644;
  WalkLeft_Stage0_frm[1].positions[18] = 1428;
  WalkLeft_Stage0_frm[1].positions[19] = 1493;

  WalkLeft_Stage0_frm[2].positions[0] = 1500;
  WalkLeft_Stage0_frm[2].positions[1] = 1470;
  WalkLeft_Stage0_frm[2].positions[2] = 1500;
  WalkLeft_Stage0_frm[2].positions[3] = 1500;
  WalkLeft_Stage0_frm[2].positions[4] = 1500;
  WalkLeft_Stage0_frm[2].positions[5] = 1318;
  WalkLeft_Stage0_frm[2].positions[6] = 1518;
  WalkLeft_Stage0_frm[2].positions[7] = 1353;
  WalkLeft_Stage0_frm[2].positions[8] = 1571;
  WalkLeft_Stage0_frm[2].positions[9] = 1518;
  WalkLeft_Stage0_frm[2].positions[10] = 1500;
  WalkLeft_Stage0_frm[2].positions[11] = 1860;
  WalkLeft_Stage0_frm[2].positions[12] = 1590;
  WalkLeft_Stage0_frm[2].positions[13] = 1500;
  WalkLeft_Stage0_frm[2].positions[14] = 1500;
  WalkLeft_Stage0_frm[2].positions[15] = 1683;
  WalkLeft_Stage0_frm[2].positions[16] = 1504;
  WalkLeft_Stage0_frm[2].positions[17] = 1644;
  WalkLeft_Stage0_frm[2].positions[18] = 1428;
  WalkLeft_Stage0_frm[2].positions[19] = 1504;

  WalkLeft_Stage0_frm[3].positions[0] = 1500;
  WalkLeft_Stage0_frm[3].positions[1] = 1470;
  WalkLeft_Stage0_frm[3].positions[2] = 1500;
  WalkLeft_Stage0_frm[3].positions[3] = 1500;
  WalkLeft_Stage0_frm[3].positions[4] = 1500;
  WalkLeft_Stage0_frm[3].positions[5] = 1315;
  WalkLeft_Stage0_frm[3].positions[6] = 1583;
  WalkLeft_Stage0_frm[3].positions[7] = 1360;
  WalkLeft_Stage0_frm[3].positions[8] = 1574;
  WalkLeft_Stage0_frm[3].positions[9] = 1583;
  WalkLeft_Stage0_frm[3].positions[10] = 1500;
  WalkLeft_Stage0_frm[3].positions[11] = 1860;
  WalkLeft_Stage0_frm[3].positions[12] = 1590;
  WalkLeft_Stage0_frm[3].positions[13] = 1500;
  WalkLeft_Stage0_frm[3].positions[14] = 1500;
  WalkLeft_Stage0_frm[3].positions[15] = 1693;
  WalkLeft_Stage0_frm[3].positions[16] = 1569;
  WalkLeft_Stage0_frm[3].positions[17] = 1625;
  WalkLeft_Stage0_frm[3].positions[18] = 1418;
  WalkLeft_Stage0_frm[3].positions[19] = 1569;

  WalkLeft_Stage0_frm[4].positions[0] = 1500;
  WalkLeft_Stage0_frm[4].positions[1] = 1470;
  WalkLeft_Stage0_frm[4].positions[2] = 1500;
  WalkLeft_Stage0_frm[4].positions[3] = 1501;
  WalkLeft_Stage0_frm[4].positions[4] = 1500;
  WalkLeft_Stage0_frm[4].positions[5] = 1319;
  WalkLeft_Stage0_frm[4].positions[6] = 1592;
  WalkLeft_Stage0_frm[4].positions[7] = 1351;
  WalkLeft_Stage0_frm[4].positions[8] = 1570;
  WalkLeft_Stage0_frm[4].positions[9] = 1592;
  WalkLeft_Stage0_frm[4].positions[10] = 1500;
  WalkLeft_Stage0_frm[4].positions[11] = 1860;
  WalkLeft_Stage0_frm[4].positions[12] = 1590;
  WalkLeft_Stage0_frm[4].positions[13] = 1501;
  WalkLeft_Stage0_frm[4].positions[14] = 1500;
  WalkLeft_Stage0_frm[4].positions[15] = 1690;
  WalkLeft_Stage0_frm[4].positions[16] = 1578;
  WalkLeft_Stage0_frm[4].positions[17] = 1631;
  WalkLeft_Stage0_frm[4].positions[18] = 1421;
  WalkLeft_Stage0_frm[4].positions[19] = 1578;

  WalkLeft_Stage0_frm[5].positions[0] = 1500;
  WalkLeft_Stage0_frm[5].positions[1] = 1470;
  WalkLeft_Stage0_frm[5].positions[2] = 1500;
  WalkLeft_Stage0_frm[5].positions[3] = 1503;
  WalkLeft_Stage0_frm[5].positions[4] = 1500;
  WalkLeft_Stage0_frm[5].positions[5] = 1340;
  WalkLeft_Stage0_frm[5].positions[6] = 1607;
  WalkLeft_Stage0_frm[5].positions[7] = 1310;
  WalkLeft_Stage0_frm[5].positions[8] = 1550;
  WalkLeft_Stage0_frm[5].positions[9] = 1607;
  WalkLeft_Stage0_frm[5].positions[10] = 1500;
  WalkLeft_Stage0_frm[5].positions[11] = 1860;
  WalkLeft_Stage0_frm[5].positions[12] = 1590;
  WalkLeft_Stage0_frm[5].positions[13] = 1503;
  WalkLeft_Stage0_frm[5].positions[14] = 1501;
  WalkLeft_Stage0_frm[5].positions[15] = 1701;
  WalkLeft_Stage0_frm[5].positions[16] = 1593;
  WalkLeft_Stage0_frm[5].positions[17] = 1621;
  WalkLeft_Stage0_frm[5].positions[18] = 1427;
  WalkLeft_Stage0_frm[5].positions[19] = 1593;

  WalkLeft_Stage0_frm[6].positions[0] = 1500;
  WalkLeft_Stage0_frm[6].positions[1] = 1470;
  WalkLeft_Stage0_frm[6].positions[2] = 1500;
  WalkLeft_Stage0_frm[6].positions[3] = 1505;
  WalkLeft_Stage0_frm[6].positions[4] = 1500;
  WalkLeft_Stage0_frm[6].positions[5] = 1363;
  WalkLeft_Stage0_frm[6].positions[6] = 1618;
  WalkLeft_Stage0_frm[6].positions[7] = 1265;
  WalkLeft_Stage0_frm[6].positions[8] = 1528;
  WalkLeft_Stage0_frm[6].positions[9] = 1618;
  WalkLeft_Stage0_frm[6].positions[10] = 1500;
  WalkLeft_Stage0_frm[6].positions[11] = 1860;
  WalkLeft_Stage0_frm[6].positions[12] = 1590;
  WalkLeft_Stage0_frm[6].positions[13] = 1505;
  WalkLeft_Stage0_frm[6].positions[14] = 1503;
  WalkLeft_Stage0_frm[6].positions[15] = 1752;
  WalkLeft_Stage0_frm[6].positions[16] = 1607;
  WalkLeft_Stage0_frm[6].positions[17] = 1536;
  WalkLeft_Stage0_frm[6].positions[18] = 1403;
  WalkLeft_Stage0_frm[6].positions[19] = 1607;

  WalkLeft_Stage0_frm[7].positions[0] = 1500;
  WalkLeft_Stage0_frm[7].positions[1] = 1470;
  WalkLeft_Stage0_frm[7].positions[2] = 1500;
  WalkLeft_Stage0_frm[7].positions[3] = 1504;
  WalkLeft_Stage0_frm[7].positions[4] = 1500;
  WalkLeft_Stage0_frm[7].positions[5] = 1361;
  WalkLeft_Stage0_frm[7].positions[6] = 1621;
  WalkLeft_Stage0_frm[7].positions[7] = 1268;
  WalkLeft_Stage0_frm[7].positions[8] = 1529;
  WalkLeft_Stage0_frm[7].positions[9] = 1621;
  WalkLeft_Stage0_frm[7].positions[10] = 1500;
  WalkLeft_Stage0_frm[7].positions[11] = 1860;
  WalkLeft_Stage0_frm[7].positions[12] = 1590;
  WalkLeft_Stage0_frm[7].positions[13] = 1504;
  WalkLeft_Stage0_frm[7].positions[14] = 1504;
  WalkLeft_Stage0_frm[7].positions[15] = 1844;
  WalkLeft_Stage0_frm[7].positions[16] = 1638;
  WalkLeft_Stage0_frm[7].positions[17] = 1353;
  WalkLeft_Stage0_frm[7].positions[18] = 1316;
  WalkLeft_Stage0_frm[7].positions[19] = 1638;

  WalkLeft_Stage0_frm[8].positions[0] = 1500;
  WalkLeft_Stage0_frm[8].positions[1] = 1470;
  WalkLeft_Stage0_frm[8].positions[2] = 1500;
  WalkLeft_Stage0_frm[8].positions[3] = 1498;
  WalkLeft_Stage0_frm[8].positions[4] = 1500;
  WalkLeft_Stage0_frm[8].positions[5] = 1346;
  WalkLeft_Stage0_frm[8].positions[6] = 1591;
  WalkLeft_Stage0_frm[8].positions[7] = 1298;
  WalkLeft_Stage0_frm[8].positions[8] = 1544;
  WalkLeft_Stage0_frm[8].positions[9] = 1591;
  WalkLeft_Stage0_frm[8].positions[10] = 1500;
  WalkLeft_Stage0_frm[8].positions[11] = 1860;
  WalkLeft_Stage0_frm[8].positions[12] = 1590;
  WalkLeft_Stage0_frm[8].positions[13] = 1498;
  WalkLeft_Stage0_frm[8].positions[14] = 1500;
  WalkLeft_Stage0_frm[8].positions[15] = 1803;
  WalkLeft_Stage0_frm[8].positions[16] = 1647;
  WalkLeft_Stage0_frm[8].positions[17] = 1401;
  WalkLeft_Stage0_frm[8].positions[18] = 1306;
  WalkLeft_Stage0_frm[8].positions[19] = 1647;

  WalkLeft_Stage0_frm[9].positions[0] = 1500;
  WalkLeft_Stage0_frm[9].positions[1] = 1470;
  WalkLeft_Stage0_frm[9].positions[2] = 1500;
  WalkLeft_Stage0_frm[9].positions[3] = 1498;
  WalkLeft_Stage0_frm[9].positions[4] = 1500;
  WalkLeft_Stage0_frm[9].positions[5] = 1306;
  WalkLeft_Stage0_frm[9].positions[6] = 1517;
  WalkLeft_Stage0_frm[9].positions[7] = 1377;
  WalkLeft_Stage0_frm[9].positions[8] = 1583;
  WalkLeft_Stage0_frm[9].positions[9] = 1517;
  WalkLeft_Stage0_frm[9].positions[10] = 1500;
  WalkLeft_Stage0_frm[9].positions[11] = 1860;
  WalkLeft_Stage0_frm[9].positions[12] = 1590;
  WalkLeft_Stage0_frm[9].positions[13] = 1498;
  WalkLeft_Stage0_frm[9].positions[14] = 1499;
  WalkLeft_Stage0_frm[9].positions[15] = 1690;
  WalkLeft_Stage0_frm[9].positions[16] = 1583;
  WalkLeft_Stage0_frm[9].positions[17] = 1622;
  WalkLeft_Stage0_frm[9].positions[18] = 1405;
  WalkLeft_Stage0_frm[9].positions[19] = 1583;

  WalkLeft_Stage0_frm[10].positions[0] = 1500;
  WalkLeft_Stage0_frm[10].positions[1] = 1470;
  WalkLeft_Stage0_frm[10].positions[2] = 1500;
  WalkLeft_Stage0_frm[10].positions[3] = 1499;
  WalkLeft_Stage0_frm[10].positions[4] = 1500;
  WalkLeft_Stage0_frm[10].positions[5] = 1295;
  WalkLeft_Stage0_frm[10].positions[6] = 1441;
  WalkLeft_Stage0_frm[10].positions[7] = 1398;
  WalkLeft_Stage0_frm[10].positions[8] = 1593;
  WalkLeft_Stage0_frm[10].positions[9] = 1441;
  WalkLeft_Stage0_frm[10].positions[10] = 1500;
  WalkLeft_Stage0_frm[10].positions[11] = 1860;
  WalkLeft_Stage0_frm[10].positions[12] = 1590;
  WalkLeft_Stage0_frm[10].positions[13] = 1499;
  WalkLeft_Stage0_frm[10].positions[14] = 1500;
  WalkLeft_Stage0_frm[10].positions[15] = 1716;
  WalkLeft_Stage0_frm[10].positions[16] = 1505;
  WalkLeft_Stage0_frm[10].positions[17] = 1580;
  WalkLeft_Stage0_frm[10].positions[18] = 1396;
  WalkLeft_Stage0_frm[10].positions[19] = 1505;

  WalkLeft_Stage1_frm[0].positions[0] = 1500;
  WalkLeft_Stage1_frm[0].positions[1] = 1470;
  WalkLeft_Stage1_frm[0].positions[2] = 1500;
  WalkLeft_Stage1_frm[0].positions[3] = 1499;
  WalkLeft_Stage1_frm[0].positions[4] = 1500;
  WalkLeft_Stage1_frm[0].positions[5] = 1303;
  WalkLeft_Stage1_frm[0].positions[6] = 1416;
  WalkLeft_Stage1_frm[0].positions[7] = 1382;
  WalkLeft_Stage1_frm[0].positions[8] = 1585;
  WalkLeft_Stage1_frm[0].positions[9] = 1416;
  WalkLeft_Stage1_frm[0].positions[10] = 1500;
  WalkLeft_Stage1_frm[0].positions[11] = 1860;
  WalkLeft_Stage1_frm[0].positions[12] = 1590;
  WalkLeft_Stage1_frm[0].positions[13] = 1499;
  WalkLeft_Stage1_frm[0].positions[14] = 1500;
  WalkLeft_Stage1_frm[0].positions[15] = 1718;
  WalkLeft_Stage1_frm[0].positions[16] = 1480;
  WalkLeft_Stage1_frm[0].positions[17] = 1576;
  WalkLeft_Stage1_frm[0].positions[18] = 1394;
  WalkLeft_Stage1_frm[0].positions[19] = 1480;

  WalkLeft_Stage1_frm[1].positions[0] = 1500;
  WalkLeft_Stage1_frm[1].positions[1] = 1470;
  WalkLeft_Stage1_frm[1].positions[2] = 1500;
  WalkLeft_Stage1_frm[1].positions[3] = 1495;
  WalkLeft_Stage1_frm[1].positions[4] = 1498;
  WalkLeft_Stage1_frm[1].positions[5] = 1315;
  WalkLeft_Stage1_frm[1].positions[6] = 1368;
  WalkLeft_Stage1_frm[1].positions[7] = 1342;
  WalkLeft_Stage1_frm[1].positions[8] = 1547;
  WalkLeft_Stage1_frm[1].positions[9] = 1368;
  WalkLeft_Stage1_frm[1].positions[10] = 1500;
  WalkLeft_Stage1_frm[1].positions[11] = 1860;
  WalkLeft_Stage1_frm[1].positions[12] = 1590;
  WalkLeft_Stage1_frm[1].positions[13] = 1495;
  WalkLeft_Stage1_frm[1].positions[14] = 1500;
  WalkLeft_Stage1_frm[1].positions[15] = 1700;
  WalkLeft_Stage1_frm[1].positions[16] = 1435;
  WalkLeft_Stage1_frm[1].positions[17] = 1612;
  WalkLeft_Stage1_frm[1].positions[18] = 1412;
  WalkLeft_Stage1_frm[1].positions[19] = 1435;

  WalkLeft_Stage1_frm[2].positions[0] = 1500;
  WalkLeft_Stage1_frm[2].positions[1] = 1470;
  WalkLeft_Stage1_frm[2].positions[2] = 1500;
  WalkLeft_Stage1_frm[2].positions[3] = 1491;
  WalkLeft_Stage1_frm[2].positions[4] = 1493;
  WalkLeft_Stage1_frm[2].positions[5] = 1283;
  WalkLeft_Stage1_frm[2].positions[6] = 1331;
  WalkLeft_Stage1_frm[2].positions[7] = 1379;
  WalkLeft_Stage1_frm[2].positions[8] = 1536;
  WalkLeft_Stage1_frm[2].positions[9] = 1331;
  WalkLeft_Stage1_frm[2].positions[10] = 1500;
  WalkLeft_Stage1_frm[2].positions[11] = 1860;
  WalkLeft_Stage1_frm[2].positions[12] = 1590;
  WalkLeft_Stage1_frm[2].positions[13] = 1491;
  WalkLeft_Stage1_frm[2].positions[14] = 1500;
  WalkLeft_Stage1_frm[2].positions[15] = 1660;
  WalkLeft_Stage1_frm[2].positions[16] = 1401;
  WalkLeft_Stage1_frm[2].positions[17] = 1690;
  WalkLeft_Stage1_frm[2].positions[18] = 1450;
  WalkLeft_Stage1_frm[2].positions[19] = 1401;

  WalkLeft_Stage1_frm[3].positions[0] = 1500;
  WalkLeft_Stage1_frm[3].positions[1] = 1470;
  WalkLeft_Stage1_frm[3].positions[2] = 1500;
  WalkLeft_Stage1_frm[3].positions[3] = 1494;
  WalkLeft_Stage1_frm[3].positions[4] = 1491;
  WalkLeft_Stage1_frm[3].positions[5] = 1169;
  WalkLeft_Stage1_frm[3].positions[6] = 1326;
  WalkLeft_Stage1_frm[3].positions[7] = 1609;
  WalkLeft_Stage1_frm[3].positions[8] = 1645;
  WalkLeft_Stage1_frm[3].positions[9] = 1326;
  WalkLeft_Stage1_frm[3].positions[10] = 1500;
  WalkLeft_Stage1_frm[3].positions[11] = 1860;
  WalkLeft_Stage1_frm[3].positions[12] = 1590;
  WalkLeft_Stage1_frm[3].positions[13] = 1494;
  WalkLeft_Stage1_frm[3].positions[14] = 1500;
  WalkLeft_Stage1_frm[3].positions[15] = 1629;
  WalkLeft_Stage1_frm[3].positions[16] = 1375;
  WalkLeft_Stage1_frm[3].positions[17] = 1750;
  WalkLeft_Stage1_frm[3].positions[18] = 1480;
  WalkLeft_Stage1_frm[3].positions[19] = 1375;

  WalkLeft_Stage1_frm[4].positions[0] = 1500;
  WalkLeft_Stage1_frm[4].positions[1] = 1470;
  WalkLeft_Stage1_frm[4].positions[2] = 1500;
  WalkLeft_Stage1_frm[4].positions[3] = 1504;
  WalkLeft_Stage1_frm[4].positions[4] = 1500;
  WalkLeft_Stage1_frm[4].positions[5] = 1166;
  WalkLeft_Stage1_frm[4].positions[6] = 1415;
  WalkLeft_Stage1_frm[4].positions[7] = 1665;
  WalkLeft_Stage1_frm[4].positions[8] = 1728;
  WalkLeft_Stage1_frm[4].positions[9] = 1415;
  WalkLeft_Stage1_frm[4].positions[10] = 1500;
  WalkLeft_Stage1_frm[4].positions[11] = 1860;
  WalkLeft_Stage1_frm[4].positions[12] = 1590;
  WalkLeft_Stage1_frm[4].positions[13] = 1504;
  WalkLeft_Stage1_frm[4].positions[14] = 1500;
  WalkLeft_Stage1_frm[4].positions[15] = 1643;
  WalkLeft_Stage1_frm[4].positions[16] = 1384;
  WalkLeft_Stage1_frm[4].positions[17] = 1724;
  WalkLeft_Stage1_frm[4].positions[18] = 1467;
  WalkLeft_Stage1_frm[4].positions[19] = 1384;

  WalkLeft_Stage1_frm[5].positions[0] = 1500;
  WalkLeft_Stage1_frm[5].positions[1] = 1470;
  WalkLeft_Stage1_frm[5].positions[2] = 1500;
  WalkLeft_Stage1_frm[5].positions[3] = 1504;
  WalkLeft_Stage1_frm[5].positions[4] = 1500;
  WalkLeft_Stage1_frm[5].positions[5] = 1294;
  WalkLeft_Stage1_frm[5].positions[6] = 1505;
  WalkLeft_Stage1_frm[5].positions[7] = 1418;
  WalkLeft_Stage1_frm[5].positions[8] = 1622;
  WalkLeft_Stage1_frm[5].positions[9] = 1505;
  WalkLeft_Stage1_frm[5].positions[10] = 1500;
  WalkLeft_Stage1_frm[5].positions[11] = 1860;
  WalkLeft_Stage1_frm[5].positions[12] = 1590;
  WalkLeft_Stage1_frm[5].positions[13] = 1504;
  WalkLeft_Stage1_frm[5].positions[14] = 1500;
  WalkLeft_Stage1_frm[5].positions[15] = 1665;
  WalkLeft_Stage1_frm[5].positions[16] = 1433;
  WalkLeft_Stage1_frm[5].positions[17] = 1681;
  WalkLeft_Stage1_frm[5].positions[18] = 1446;
  WalkLeft_Stage1_frm[5].positions[19] = 1433;

  WalkLeft_Stage1_frm[6].positions[0] = 1500;
  WalkLeft_Stage1_frm[6].positions[1] = 1470;
  WalkLeft_Stage1_frm[6].positions[2] = 1500;
  WalkLeft_Stage1_frm[6].positions[3] = 1501;
  WalkLeft_Stage1_frm[6].positions[4] = 1500;
  WalkLeft_Stage1_frm[6].positions[5] = 1330;
  WalkLeft_Stage1_frm[6].positions[6] = 1550;
  WalkLeft_Stage1_frm[6].positions[7] = 1330;
  WalkLeft_Stage1_frm[6].positions[8] = 1560;
  WalkLeft_Stage1_frm[6].positions[9] = 1550;
  WalkLeft_Stage1_frm[6].positions[10] = 1500;
  WalkLeft_Stage1_frm[6].positions[11] = 1860;
  WalkLeft_Stage1_frm[6].positions[12] = 1590;
  WalkLeft_Stage1_frm[6].positions[13] = 1501;
  WalkLeft_Stage1_frm[6].positions[14] = 1500;
  WalkLeft_Stage1_frm[6].positions[15] = 1679;
  WalkLeft_Stage1_frm[6].positions[16] = 1482;
  WalkLeft_Stage1_frm[6].positions[17] = 1652;
  WalkLeft_Stage1_frm[6].positions[18] = 1432;
  WalkLeft_Stage1_frm[6].positions[19] = 1482;

  WalkLeft_Stage1_frm[7].positions[0] = 1500;
  WalkLeft_Stage1_frm[7].positions[1] = 1470;
  WalkLeft_Stage1_frm[7].positions[2] = 1500;
  WalkLeft_Stage1_frm[7].positions[3] = 1501;
  WalkLeft_Stage1_frm[7].positions[4] = 1500;
  WalkLeft_Stage1_frm[7].positions[5] = 1336;
  WalkLeft_Stage1_frm[7].positions[6] = 1566;
  WalkLeft_Stage1_frm[7].positions[7] = 1317;
  WalkLeft_Stage1_frm[7].positions[8] = 1553;
  WalkLeft_Stage1_frm[7].positions[9] = 1566;
  WalkLeft_Stage1_frm[7].positions[10] = 1500;
  WalkLeft_Stage1_frm[7].positions[11] = 1860;
  WalkLeft_Stage1_frm[7].positions[12] = 1590;
  WalkLeft_Stage1_frm[7].positions[13] = 1501;
  WalkLeft_Stage1_frm[7].positions[14] = 1500;
  WalkLeft_Stage1_frm[7].positions[15] = 1682;
  WalkLeft_Stage1_frm[7].positions[16] = 1498;
  WalkLeft_Stage1_frm[7].positions[17] = 1646;
  WalkLeft_Stage1_frm[7].positions[18] = 1429;
  WalkLeft_Stage1_frm[7].positions[19] = 1498;

  WalkLeft_Stage1_frm[8].positions[0] = 1500;
  WalkLeft_Stage1_frm[8].positions[1] = 1470;
  WalkLeft_Stage1_frm[8].positions[2] = 1500;
  WalkLeft_Stage1_frm[8].positions[3] = 1505;
  WalkLeft_Stage1_frm[8].positions[4] = 1500;
  WalkLeft_Stage1_frm[8].positions[5] = 1349;
  WalkLeft_Stage1_frm[8].positions[6] = 1594;
  WalkLeft_Stage1_frm[8].positions[7] = 1291;
  WalkLeft_Stage1_frm[8].positions[8] = 1541;
  WalkLeft_Stage1_frm[8].positions[9] = 1594;
  WalkLeft_Stage1_frm[8].positions[10] = 1500;
  WalkLeft_Stage1_frm[8].positions[11] = 1860;
  WalkLeft_Stage1_frm[8].positions[12] = 1590;
  WalkLeft_Stage1_frm[8].positions[13] = 1505;
  WalkLeft_Stage1_frm[8].positions[14] = 1500;
  WalkLeft_Stage1_frm[8].positions[15] = 1721;
  WalkLeft_Stage1_frm[8].positions[16] = 1524;
  WalkLeft_Stage1_frm[8].positions[17] = 1588;
  WalkLeft_Stage1_frm[8].positions[18] = 1419;
  WalkLeft_Stage1_frm[8].positions[19] = 1524;

  WalkLeft_Stage1_frm[9].positions[0] = 1500;
  WalkLeft_Stage1_frm[9].positions[1] = 1470;
  WalkLeft_Stage1_frm[9].positions[2] = 1500;
  WalkLeft_Stage1_frm[9].positions[3] = 1509;
  WalkLeft_Stage1_frm[9].positions[4] = 1500;
  WalkLeft_Stage1_frm[9].positions[5] = 1358;
  WalkLeft_Stage1_frm[9].positions[6] = 1613;
  WalkLeft_Stage1_frm[9].positions[7] = 1275;
  WalkLeft_Stage1_frm[9].positions[8] = 1533;
  WalkLeft_Stage1_frm[9].positions[9] = 1613;
  WalkLeft_Stage1_frm[9].positions[10] = 1500;
  WalkLeft_Stage1_frm[9].positions[11] = 1860;
  WalkLeft_Stage1_frm[9].positions[12] = 1590;
  WalkLeft_Stage1_frm[9].positions[13] = 1509;
  WalkLeft_Stage1_frm[9].positions[14] = 1502;
  WalkLeft_Stage1_frm[9].positions[15] = 1793;
  WalkLeft_Stage1_frm[9].positions[16] = 1546;
  WalkLeft_Stage1_frm[9].positions[17] = 1477;
  WalkLeft_Stage1_frm[9].positions[18] = 1396;
  WalkLeft_Stage1_frm[9].positions[19] = 1546;

  WalkLeft_Stage1_frm[10].positions[0] = 1500;
  WalkLeft_Stage1_frm[10].positions[1] = 1470;
  WalkLeft_Stage1_frm[10].positions[2] = 1500;
  WalkLeft_Stage1_frm[10].positions[3] = 1506;
  WalkLeft_Stage1_frm[10].positions[4] = 1500;
  WalkLeft_Stage1_frm[10].positions[5] = 1361;
  WalkLeft_Stage1_frm[10].positions[6] = 1621;
  WalkLeft_Stage1_frm[10].positions[7] = 1268;
  WalkLeft_Stage1_frm[10].positions[8] = 1529;
  WalkLeft_Stage1_frm[10].positions[9] = 1621;
  WalkLeft_Stage1_frm[10].positions[10] = 1500;
  WalkLeft_Stage1_frm[10].positions[11] = 1860;
  WalkLeft_Stage1_frm[10].positions[12] = 1590;
  WalkLeft_Stage1_frm[10].positions[13] = 1506;
  WalkLeft_Stage1_frm[10].positions[14] = 1505;
  WalkLeft_Stage1_frm[10].positions[15] = 1872;
  WalkLeft_Stage1_frm[10].positions[16] = 1593;
  WalkLeft_Stage1_frm[10].positions[17] = 1317;
  WalkLeft_Stage1_frm[10].positions[18] = 1321;
  WalkLeft_Stage1_frm[10].positions[19] = 1593;

  WalkLeft_Stage1_frm[11].positions[0] = 1500;
  WalkLeft_Stage1_frm[11].positions[1] = 1470;
  WalkLeft_Stage1_frm[11].positions[2] = 1500;
  WalkLeft_Stage1_frm[11].positions[3] = 1496;
  WalkLeft_Stage1_frm[11].positions[4] = 1500;
  WalkLeft_Stage1_frm[11].positions[5] = 1343;
  WalkLeft_Stage1_frm[11].positions[6] = 1589;
  WalkLeft_Stage1_frm[11].positions[7] = 1304;
  WalkLeft_Stage1_frm[11].positions[8] = 1547;
  WalkLeft_Stage1_frm[11].positions[9] = 1589;
  WalkLeft_Stage1_frm[11].positions[10] = 1500;
  WalkLeft_Stage1_frm[11].positions[11] = 1860;
  WalkLeft_Stage1_frm[11].positions[12] = 1590;
  WalkLeft_Stage1_frm[11].positions[13] = 1496;
  WalkLeft_Stage1_frm[11].positions[14] = 1501;
  WalkLeft_Stage1_frm[11].positions[15] = 1810;
  WalkLeft_Stage1_frm[11].positions[16] = 1632;
  WalkLeft_Stage1_frm[11].positions[17] = 1381;
  WalkLeft_Stage1_frm[11].positions[18] = 1294;
  WalkLeft_Stage1_frm[11].positions[19] = 1632;

  WalkLeft_Stage1_frm[12].positions[0] = 1500;
  WalkLeft_Stage1_frm[12].positions[1] = 1470;
  WalkLeft_Stage1_frm[12].positions[2] = 1500;
  WalkLeft_Stage1_frm[12].positions[3] = 1496;
  WalkLeft_Stage1_frm[12].positions[4] = 1500;
  WalkLeft_Stage1_frm[12].positions[5] = 1315;
  WalkLeft_Stage1_frm[12].positions[6] = 1509;
  WalkLeft_Stage1_frm[12].positions[7] = 1359;
  WalkLeft_Stage1_frm[12].positions[8] = 1574;
  WalkLeft_Stage1_frm[12].positions[9] = 1509;
  WalkLeft_Stage1_frm[12].positions[10] = 1500;
  WalkLeft_Stage1_frm[12].positions[11] = 1860;
  WalkLeft_Stage1_frm[12].positions[12] = 1590;
  WalkLeft_Stage1_frm[12].positions[13] = 1496;
  WalkLeft_Stage1_frm[12].positions[14] = 1499;
  WalkLeft_Stage1_frm[12].positions[15] = 1688;
  WalkLeft_Stage1_frm[12].positions[16] = 1577;
  WalkLeft_Stage1_frm[12].positions[17] = 1618;
  WalkLeft_Stage1_frm[12].positions[18] = 1396;
  WalkLeft_Stage1_frm[12].positions[19] = 1577;

  WalkLeft_Stage1_frm[13].positions[0] = 1500;
  WalkLeft_Stage1_frm[13].positions[1] = 1470;
  WalkLeft_Stage1_frm[13].positions[2] = 1500;
  WalkLeft_Stage1_frm[13].positions[3] = 1499;
  WalkLeft_Stage1_frm[13].positions[4] = 1500;
  WalkLeft_Stage1_frm[13].positions[5] = 1315;
  WalkLeft_Stage1_frm[13].positions[6] = 1432;
  WalkLeft_Stage1_frm[13].positions[7] = 1359;
  WalkLeft_Stage1_frm[13].positions[8] = 1574;
  WalkLeft_Stage1_frm[13].positions[9] = 1432;
  WalkLeft_Stage1_frm[13].positions[10] = 1500;
  WalkLeft_Stage1_frm[13].positions[11] = 1860;
  WalkLeft_Stage1_frm[13].positions[12] = 1590;
  WalkLeft_Stage1_frm[13].positions[13] = 1499;
  WalkLeft_Stage1_frm[13].positions[14] = 1500;
  WalkLeft_Stage1_frm[13].positions[15] = 1702;
  WalkLeft_Stage1_frm[13].positions[16] = 1496;
  WalkLeft_Stage1_frm[13].positions[17] = 1608;
  WalkLeft_Stage1_frm[13].positions[18] = 1410;
  WalkLeft_Stage1_frm[13].positions[19] = 1496;

  WalkLeft_Stage2_frm[0].positions[0] = 1500;
  WalkLeft_Stage2_frm[0].positions[1] = 1470;
  WalkLeft_Stage2_frm[0].positions[2] = 1500;
  WalkLeft_Stage2_frm[0].positions[3] = 1499;
  WalkLeft_Stage2_frm[0].positions[4] = 1500;
  WalkLeft_Stage2_frm[0].positions[5] = 1322;
  WalkLeft_Stage2_frm[0].positions[6] = 1407;
  WalkLeft_Stage2_frm[0].positions[7] = 1345;
  WalkLeft_Stage2_frm[0].positions[8] = 1567;
  WalkLeft_Stage2_frm[0].positions[9] = 1407;
  WalkLeft_Stage2_frm[0].positions[10] = 1500;
  WalkLeft_Stage2_frm[0].positions[11] = 1860;
  WalkLeft_Stage2_frm[0].positions[12] = 1590;
  WalkLeft_Stage2_frm[0].positions[13] = 1499;
  WalkLeft_Stage2_frm[0].positions[14] = 1500;
  WalkLeft_Stage2_frm[0].positions[15] = 1706;
  WalkLeft_Stage2_frm[0].positions[16] = 1471;
  WalkLeft_Stage2_frm[0].positions[17] = 1599;
  WalkLeft_Stage2_frm[0].positions[18] = 1406;
  WalkLeft_Stage2_frm[0].positions[19] = 1471;

  WalkLeft_Stage2_frm[1].positions[0] = 1500;
  WalkLeft_Stage2_frm[1].positions[1] = 1470;
  WalkLeft_Stage2_frm[1].positions[2] = 1500;
  WalkLeft_Stage2_frm[1].positions[3] = 1497;
  WalkLeft_Stage2_frm[1].positions[4] = 1499;
  WalkLeft_Stage2_frm[1].positions[5] = 1326;
  WalkLeft_Stage2_frm[1].positions[6] = 1360;
  WalkLeft_Stage2_frm[1].positions[7] = 1328;
  WalkLeft_Stage2_frm[1].positions[8] = 1548;
  WalkLeft_Stage2_frm[1].positions[9] = 1360;
  WalkLeft_Stage2_frm[1].positions[10] = 1500;
  WalkLeft_Stage2_frm[1].positions[11] = 1860;
  WalkLeft_Stage2_frm[1].positions[12] = 1590;
  WalkLeft_Stage2_frm[1].positions[13] = 1497;
  WalkLeft_Stage2_frm[1].positions[14] = 1500;
  WalkLeft_Stage2_frm[1].positions[15] = 1701;
  WalkLeft_Stage2_frm[1].positions[16] = 1425;
  WalkLeft_Stage2_frm[1].positions[17] = 1609;
  WalkLeft_Stage2_frm[1].positions[18] = 1410;
  WalkLeft_Stage2_frm[1].positions[19] = 1425;

  WalkLeft_Stage2_frm[2].positions[0] = 1500;
  WalkLeft_Stage2_frm[2].positions[1] = 1470;
  WalkLeft_Stage2_frm[2].positions[2] = 1500;
  WalkLeft_Stage2_frm[2].positions[3] = 1495;
  WalkLeft_Stage2_frm[2].positions[4] = 1496;
  WalkLeft_Stage2_frm[2].positions[5] = 1292;
  WalkLeft_Stage2_frm[2].positions[6] = 1324;
  WalkLeft_Stage2_frm[2].positions[7] = 1379;
  WalkLeft_Stage2_frm[2].positions[8] = 1556;
  WalkLeft_Stage2_frm[2].positions[9] = 1324;
  WalkLeft_Stage2_frm[2].positions[10] = 1500;
  WalkLeft_Stage2_frm[2].positions[11] = 1860;
  WalkLeft_Stage2_frm[2].positions[12] = 1590;
  WalkLeft_Stage2_frm[2].positions[13] = 1495;
  WalkLeft_Stage2_frm[2].positions[14] = 1500;
  WalkLeft_Stage2_frm[2].positions[15] = 1676;
  WalkLeft_Stage2_frm[2].positions[16] = 1392;
  WalkLeft_Stage2_frm[2].positions[17] = 1659;
  WalkLeft_Stage2_frm[2].positions[18] = 1435;
  WalkLeft_Stage2_frm[2].positions[19] = 1392;

  WalkLeft_Stage2_frm[3].positions[0] = 1500;
  WalkLeft_Stage2_frm[3].positions[1] = 1470;
  WalkLeft_Stage2_frm[3].positions[2] = 1500;
  WalkLeft_Stage2_frm[3].positions[3] = 1496;
  WalkLeft_Stage2_frm[3].positions[4] = 1494;
  WalkLeft_Stage2_frm[3].positions[5] = 1190;
  WalkLeft_Stage2_frm[3].positions[6] = 1306;
  WalkLeft_Stage2_frm[3].positions[7] = 1584;
  WalkLeft_Stage2_frm[3].positions[8] = 1654;
  WalkLeft_Stage2_frm[3].positions[9] = 1306;
  WalkLeft_Stage2_frm[3].positions[10] = 1500;
  WalkLeft_Stage2_frm[3].positions[11] = 1860;
  WalkLeft_Stage2_frm[3].positions[12] = 1590;
  WalkLeft_Stage2_frm[3].positions[13] = 1496;
  WalkLeft_Stage2_frm[3].positions[14] = 1500;
  WalkLeft_Stage2_frm[3].positions[15] = 1629;
  WalkLeft_Stage2_frm[3].positions[16] = 1367;
  WalkLeft_Stage2_frm[3].positions[17] = 1752;
  WalkLeft_Stage2_frm[3].positions[18] = 1480;
  WalkLeft_Stage2_frm[3].positions[19] = 1367;

  WalkLeft_Stage2_frm[4].positions[0] = 1500;
  WalkLeft_Stage2_frm[4].positions[1] = 1470;
  WalkLeft_Stage2_frm[4].positions[2] = 1500;
  WalkLeft_Stage2_frm[4].positions[3] = 1502;
  WalkLeft_Stage2_frm[4].positions[4] = 1500;
  WalkLeft_Stage2_frm[4].positions[5] = 1185;
  WalkLeft_Stage2_frm[4].positions[6] = 1356;
  WalkLeft_Stage2_frm[4].positions[7] = 1622;
  WalkLeft_Stage2_frm[4].positions[8] = 1705;
  WalkLeft_Stage2_frm[4].positions[9] = 1356;
  WalkLeft_Stage2_frm[4].positions[10] = 1500;
  WalkLeft_Stage2_frm[4].positions[11] = 1860;
  WalkLeft_Stage2_frm[4].positions[12] = 1590;
  WalkLeft_Stage2_frm[4].positions[13] = 1502;
  WalkLeft_Stage2_frm[4].positions[14] = 1500;
  WalkLeft_Stage2_frm[4].positions[15] = 1634;
  WalkLeft_Stage2_frm[4].positions[16] = 1371;
  WalkLeft_Stage2_frm[4].positions[17] = 1741;
  WalkLeft_Stage2_frm[4].positions[18] = 1475;
  WalkLeft_Stage2_frm[4].positions[19] = 1371;

  WalkLeft_Stage2_frm[5].positions[0] = 1500;
  WalkLeft_Stage2_frm[5].positions[1] = 1470;
  WalkLeft_Stage2_frm[5].positions[2] = 1500;
  WalkLeft_Stage2_frm[5].positions[3] = 1502;
  WalkLeft_Stage2_frm[5].positions[4] = 1501;
  WalkLeft_Stage2_frm[5].positions[5] = 1307;
  WalkLeft_Stage2_frm[5].positions[6] = 1421;
  WalkLeft_Stage2_frm[5].positions[7] = 1384;
  WalkLeft_Stage2_frm[5].positions[8] = 1598;
  WalkLeft_Stage2_frm[5].positions[9] = 1421;
  WalkLeft_Stage2_frm[5].positions[10] = 1500;
  WalkLeft_Stage2_frm[5].positions[11] = 1860;
  WalkLeft_Stage2_frm[5].positions[12] = 1590;
  WalkLeft_Stage2_frm[5].positions[13] = 1502;
  WalkLeft_Stage2_frm[5].positions[14] = 1500;
  WalkLeft_Stage2_frm[5].positions[15] = 1659;
  WalkLeft_Stage2_frm[5].positions[16] = 1405;
  WalkLeft_Stage2_frm[5].positions[17] = 1692;
  WalkLeft_Stage2_frm[5].positions[18] = 1451;
  WalkLeft_Stage2_frm[5].positions[19] = 1405;

  WalkLeft_Stage2_frm[6].positions[0] = 1500;
  WalkLeft_Stage2_frm[6].positions[1] = 1470;
  WalkLeft_Stage2_frm[6].positions[2] = 1500;
  WalkLeft_Stage2_frm[6].positions[3] = 1500;
  WalkLeft_Stage2_frm[6].positions[4] = 1500;
  WalkLeft_Stage2_frm[6].positions[5] = 1312;
  WalkLeft_Stage2_frm[6].positions[6] = 1452;
  WalkLeft_Stage2_frm[6].positions[7] = 1365;
  WalkLeft_Stage2_frm[6].positions[8] = 1577;
  WalkLeft_Stage2_frm[6].positions[9] = 1452;
  WalkLeft_Stage2_frm[6].positions[10] = 1500;
  WalkLeft_Stage2_frm[6].positions[11] = 1860;
  WalkLeft_Stage2_frm[6].positions[12] = 1590;
  WalkLeft_Stage2_frm[6].positions[13] = 1500;
  WalkLeft_Stage2_frm[6].positions[14] = 1500;
  WalkLeft_Stage2_frm[6].positions[15] = 1682;
  WalkLeft_Stage2_frm[6].positions[16] = 1438;
  WalkLeft_Stage2_frm[6].positions[17] = 1646;
  WalkLeft_Stage2_frm[6].positions[18] = 1428;
  WalkLeft_Stage2_frm[6].positions[19] = 1438;

  WalkLeft_Stage2_frm[7].positions[0] = 1500;
  WalkLeft_Stage2_frm[7].positions[1] = 1470;
  WalkLeft_Stage2_frm[7].positions[2] = 1500;
  WalkLeft_Stage2_frm[7].positions[3] = 1500;
  WalkLeft_Stage2_frm[7].positions[4] = 1500;
  WalkLeft_Stage2_frm[7].positions[5] = 1311;
  WalkLeft_Stage2_frm[7].positions[6] = 1462;
  WalkLeft_Stage2_frm[7].positions[7] = 1368;
  WalkLeft_Stage2_frm[7].positions[8] = 1578;
  WalkLeft_Stage2_frm[7].positions[9] = 1462;
  WalkLeft_Stage2_frm[7].positions[10] = 1500;
  WalkLeft_Stage2_frm[7].positions[11] = 1860;
  WalkLeft_Stage2_frm[7].positions[12] = 1590;
  WalkLeft_Stage2_frm[7].positions[13] = 1500;
  WalkLeft_Stage2_frm[7].positions[14] = 1500;
  WalkLeft_Stage2_frm[7].positions[15] = 1685;
  WalkLeft_Stage2_frm[7].positions[16] = 1448;
  WalkLeft_Stage2_frm[7].positions[17] = 1642;
  WalkLeft_Stage2_frm[7].positions[18] = 1426;
  WalkLeft_Stage2_frm[7].positions[19] = 1448;

  WalkLeft_Stage2_frm[8].positions[0] = 1500;
  WalkLeft_Stage2_frm[8].positions[1] = 1470;
  WalkLeft_Stage2_frm[8].positions[2] = 1500;
  WalkLeft_Stage2_frm[8].positions[3] = 1500;
  WalkLeft_Stage2_frm[8].positions[4] = 1500;
  WalkLeft_Stage2_frm[8].positions[5] = 1316;
  WalkLeft_Stage2_frm[8].positions[6] = 1501;
  WalkLeft_Stage2_frm[8].positions[7] = 1357;
  WalkLeft_Stage2_frm[8].positions[8] = 1573;
  WalkLeft_Stage2_frm[8].positions[9] = 1501;
  WalkLeft_Stage2_frm[8].positions[10] = 1500;
  WalkLeft_Stage2_frm[8].positions[11] = 1860;
  WalkLeft_Stage2_frm[8].positions[12] = 1590;
  WalkLeft_Stage2_frm[8].positions[13] = 1500;
  WalkLeft_Stage2_frm[8].positions[14] = 1500;
  WalkLeft_Stage2_frm[8].positions[15] = 1683;
  WalkLeft_Stage2_frm[8].positions[16] = 1487;
  WalkLeft_Stage2_frm[8].positions[17] = 1645;
  WalkLeft_Stage2_frm[8].positions[18] = 1428;
  WalkLeft_Stage2_frm[8].positions[19] = 1487;

  WalkLeft_Stage2_frm[9].positions[0] = 1500;
  WalkLeft_Stage2_frm[9].positions[1] = 1470;
  WalkLeft_Stage2_frm[9].positions[2] = 1500;
  WalkLeft_Stage2_frm[9].positions[3] = 1500;
  WalkLeft_Stage2_frm[9].positions[4] = 1500;
  WalkLeft_Stage2_frm[9].positions[5] = 1317;
  WalkLeft_Stage2_frm[9].positions[6] = 1498;
  WalkLeft_Stage2_frm[9].positions[7] = 1356;
  WalkLeft_Stage2_frm[9].positions[8] = 1573;
  WalkLeft_Stage2_frm[9].positions[9] = 1498;
  WalkLeft_Stage2_frm[9].positions[10] = 1500;
  WalkLeft_Stage2_frm[9].positions[11] = 1860;
  WalkLeft_Stage2_frm[9].positions[12] = 1590;
  WalkLeft_Stage2_frm[9].positions[13] = 1500;
  WalkLeft_Stage2_frm[9].positions[14] = 1500;
  WalkLeft_Stage2_frm[9].positions[15] = 1682;
  WalkLeft_Stage2_frm[9].positions[16] = 1484;
  WalkLeft_Stage2_frm[9].positions[17] = 1646;
  WalkLeft_Stage2_frm[9].positions[18] = 1428;
  WalkLeft_Stage2_frm[9].positions[19] = 1484;

  WalkRight_Stage0_frm[0].positions[0] = 1500;
  WalkRight_Stage0_frm[0].positions[1] = 1470;
  WalkRight_Stage0_frm[0].positions[2] = 1500;
  WalkRight_Stage0_frm[0].positions[3] = 1500;
  WalkRight_Stage0_frm[0].positions[4] = 1500;
  WalkRight_Stage0_frm[0].positions[5] = 1317;
  WalkRight_Stage0_frm[0].positions[6] = 1507;
  WalkRight_Stage0_frm[0].positions[7] = 1356;
  WalkRight_Stage0_frm[0].positions[8] = 1572;
  WalkRight_Stage0_frm[0].positions[9] = 1507;
  WalkRight_Stage0_frm[0].positions[10] = 1500;
  WalkRight_Stage0_frm[0].positions[11] = 1860;
  WalkRight_Stage0_frm[0].positions[12] = 1590;
  WalkRight_Stage0_frm[0].positions[13] = 1500;
  WalkRight_Stage0_frm[0].positions[14] = 1500;
  WalkRight_Stage0_frm[0].positions[15] = 1683;
  WalkRight_Stage0_frm[0].positions[16] = 1493;
  WalkRight_Stage0_frm[0].positions[17] = 1644;
  WalkRight_Stage0_frm[0].positions[18] = 1428;
  WalkRight_Stage0_frm[0].positions[19] = 1493;

  WalkRight_Stage0_frm[1].positions[0] = 1500;
  WalkRight_Stage0_frm[1].positions[1] = 1470;
  WalkRight_Stage0_frm[1].positions[2] = 1500;
  WalkRight_Stage0_frm[1].positions[3] = 1500;
  WalkRight_Stage0_frm[1].positions[4] = 1500;
  WalkRight_Stage0_frm[1].positions[5] = 1317;
  WalkRight_Stage0_frm[1].positions[6] = 1507;
  WalkRight_Stage0_frm[1].positions[7] = 1356;
  WalkRight_Stage0_frm[1].positions[8] = 1572;
  WalkRight_Stage0_frm[1].positions[9] = 1507;
  WalkRight_Stage0_frm[1].positions[10] = 1500;
  WalkRight_Stage0_frm[1].positions[11] = 1860;
  WalkRight_Stage0_frm[1].positions[12] = 1590;
  WalkRight_Stage0_frm[1].positions[13] = 1500;
  WalkRight_Stage0_frm[1].positions[14] = 1500;
  WalkRight_Stage0_frm[1].positions[15] = 1683;
  WalkRight_Stage0_frm[1].positions[16] = 1493;
  WalkRight_Stage0_frm[1].positions[17] = 1644;
  WalkRight_Stage0_frm[1].positions[18] = 1428;
  WalkRight_Stage0_frm[1].positions[19] = 1493;

  WalkRight_Stage0_frm[2].positions[0] = 1500;
  WalkRight_Stage0_frm[2].positions[1] = 1470;
  WalkRight_Stage0_frm[2].positions[2] = 1500;
  WalkRight_Stage0_frm[2].positions[3] = 1500;
  WalkRight_Stage0_frm[2].positions[4] = 1500;
  WalkRight_Stage0_frm[2].positions[5] = 1317;
  WalkRight_Stage0_frm[2].positions[6] = 1496;
  WalkRight_Stage0_frm[2].positions[7] = 1356;
  WalkRight_Stage0_frm[2].positions[8] = 1572;
  WalkRight_Stage0_frm[2].positions[9] = 1496;
  WalkRight_Stage0_frm[2].positions[10] = 1500;
  WalkRight_Stage0_frm[2].positions[11] = 1860;
  WalkRight_Stage0_frm[2].positions[12] = 1590;
  WalkRight_Stage0_frm[2].positions[13] = 1500;
  WalkRight_Stage0_frm[2].positions[14] = 1500;
  WalkRight_Stage0_frm[2].positions[15] = 1682;
  WalkRight_Stage0_frm[2].positions[16] = 1482;
  WalkRight_Stage0_frm[2].positions[17] = 1647;
  WalkRight_Stage0_frm[2].positions[18] = 1429;
  WalkRight_Stage0_frm[2].positions[19] = 1482;

  WalkRight_Stage0_frm[3].positions[0] = 1500;
  WalkRight_Stage0_frm[3].positions[1] = 1470;
  WalkRight_Stage0_frm[3].positions[2] = 1500;
  WalkRight_Stage0_frm[3].positions[3] = 1500;
  WalkRight_Stage0_frm[3].positions[4] = 1500;
  WalkRight_Stage0_frm[3].positions[5] = 1307;
  WalkRight_Stage0_frm[3].positions[6] = 1431;
  WalkRight_Stage0_frm[3].positions[7] = 1375;
  WalkRight_Stage0_frm[3].positions[8] = 1582;
  WalkRight_Stage0_frm[3].positions[9] = 1431;
  WalkRight_Stage0_frm[3].positions[10] = 1500;
  WalkRight_Stage0_frm[3].positions[11] = 1860;
  WalkRight_Stage0_frm[3].positions[12] = 1590;
  WalkRight_Stage0_frm[3].positions[13] = 1500;
  WalkRight_Stage0_frm[3].positions[14] = 1500;
  WalkRight_Stage0_frm[3].positions[15] = 1685;
  WalkRight_Stage0_frm[3].positions[16] = 1417;
  WalkRight_Stage0_frm[3].positions[17] = 1640;
  WalkRight_Stage0_frm[3].positions[18] = 1426;
  WalkRight_Stage0_frm[3].positions[19] = 1417;

  WalkRight_Stage0_frm[4].positions[0] = 1500;
  WalkRight_Stage0_frm[4].positions[1] = 1470;
  WalkRight_Stage0_frm[4].positions[2] = 1500;
  WalkRight_Stage0_frm[4].positions[3] = 1499;
  WalkRight_Stage0_frm[4].positions[4] = 1500;
  WalkRight_Stage0_frm[4].positions[5] = 1310;
  WalkRight_Stage0_frm[4].positions[6] = 1422;
  WalkRight_Stage0_frm[4].positions[7] = 1369;
  WalkRight_Stage0_frm[4].positions[8] = 1579;
  WalkRight_Stage0_frm[4].positions[9] = 1422;
  WalkRight_Stage0_frm[4].positions[10] = 1500;
  WalkRight_Stage0_frm[4].positions[11] = 1860;
  WalkRight_Stage0_frm[4].positions[12] = 1590;
  WalkRight_Stage0_frm[4].positions[13] = 1499;
  WalkRight_Stage0_frm[4].positions[14] = 1500;
  WalkRight_Stage0_frm[4].positions[15] = 1681;
  WalkRight_Stage0_frm[4].positions[16] = 1408;
  WalkRight_Stage0_frm[4].positions[17] = 1649;
  WalkRight_Stage0_frm[4].positions[18] = 1430;
  WalkRight_Stage0_frm[4].positions[19] = 1408;

  WalkRight_Stage0_frm[5].positions[0] = 1500;
  WalkRight_Stage0_frm[5].positions[1] = 1470;
  WalkRight_Stage0_frm[5].positions[2] = 1500;
  WalkRight_Stage0_frm[5].positions[3] = 1497;
  WalkRight_Stage0_frm[5].positions[4] = 1499;
  WalkRight_Stage0_frm[5].positions[5] = 1299;
  WalkRight_Stage0_frm[5].positions[6] = 1407;
  WalkRight_Stage0_frm[5].positions[7] = 1379;
  WalkRight_Stage0_frm[5].positions[8] = 1573;
  WalkRight_Stage0_frm[5].positions[9] = 1407;
  WalkRight_Stage0_frm[5].positions[10] = 1500;
  WalkRight_Stage0_frm[5].positions[11] = 1860;
  WalkRight_Stage0_frm[5].positions[12] = 1590;
  WalkRight_Stage0_frm[5].positions[13] = 1497;
  WalkRight_Stage0_frm[5].positions[14] = 1500;
  WalkRight_Stage0_frm[5].positions[15] = 1660;
  WalkRight_Stage0_frm[5].positions[16] = 1393;
  WalkRight_Stage0_frm[5].positions[17] = 1690;
  WalkRight_Stage0_frm[5].positions[18] = 1450;
  WalkRight_Stage0_frm[5].positions[19] = 1393;

  WalkRight_Stage0_frm[6].positions[0] = 1500;
  WalkRight_Stage0_frm[6].positions[1] = 1470;
  WalkRight_Stage0_frm[6].positions[2] = 1500;
  WalkRight_Stage0_frm[6].positions[3] = 1495;
  WalkRight_Stage0_frm[6].positions[4] = 1497;
  WalkRight_Stage0_frm[6].positions[5] = 1248;
  WalkRight_Stage0_frm[6].positions[6] = 1393;
  WalkRight_Stage0_frm[6].positions[7] = 1464;
  WalkRight_Stage0_frm[6].positions[8] = 1597;
  WalkRight_Stage0_frm[6].positions[9] = 1393;
  WalkRight_Stage0_frm[6].positions[10] = 1500;
  WalkRight_Stage0_frm[6].positions[11] = 1860;
  WalkRight_Stage0_frm[6].positions[12] = 1590;
  WalkRight_Stage0_frm[6].positions[13] = 1495;
  WalkRight_Stage0_frm[6].positions[14] = 1500;
  WalkRight_Stage0_frm[6].positions[15] = 1637;
  WalkRight_Stage0_frm[6].positions[16] = 1382;
  WalkRight_Stage0_frm[6].positions[17] = 1735;
  WalkRight_Stage0_frm[6].positions[18] = 1472;
  WalkRight_Stage0_frm[6].positions[19] = 1382;

  WalkRight_Stage0_frm[7].positions[0] = 1500;
  WalkRight_Stage0_frm[7].positions[1] = 1470;
  WalkRight_Stage0_frm[7].positions[2] = 1500;
  WalkRight_Stage0_frm[7].positions[3] = 1496;
  WalkRight_Stage0_frm[7].positions[4] = 1496;
  WalkRight_Stage0_frm[7].positions[5] = 1156;
  WalkRight_Stage0_frm[7].positions[6] = 1362;
  WalkRight_Stage0_frm[7].positions[7] = 1647;
  WalkRight_Stage0_frm[7].positions[8] = 1684;
  WalkRight_Stage0_frm[7].positions[9] = 1362;
  WalkRight_Stage0_frm[7].positions[10] = 1500;
  WalkRight_Stage0_frm[7].positions[11] = 1860;
  WalkRight_Stage0_frm[7].positions[12] = 1590;
  WalkRight_Stage0_frm[7].positions[13] = 1496;
  WalkRight_Stage0_frm[7].positions[14] = 1500;
  WalkRight_Stage0_frm[7].positions[15] = 1639;
  WalkRight_Stage0_frm[7].positions[16] = 1379;
  WalkRight_Stage0_frm[7].positions[17] = 1732;
  WalkRight_Stage0_frm[7].positions[18] = 1471;
  WalkRight_Stage0_frm[7].positions[19] = 1379;

  WalkRight_Stage0_frm[8].positions[0] = 1500;
  WalkRight_Stage0_frm[8].positions[1] = 1470;
  WalkRight_Stage0_frm[8].positions[2] = 1500;
  WalkRight_Stage0_frm[8].positions[3] = 1502;
  WalkRight_Stage0_frm[8].positions[4] = 1500;
  WalkRight_Stage0_frm[8].positions[5] = 1197;
  WalkRight_Stage0_frm[8].positions[6] = 1353;
  WalkRight_Stage0_frm[8].positions[7] = 1599;
  WalkRight_Stage0_frm[8].positions[8] = 1694;
  WalkRight_Stage0_frm[8].positions[9] = 1353;
  WalkRight_Stage0_frm[8].positions[10] = 1500;
  WalkRight_Stage0_frm[8].positions[11] = 1860;
  WalkRight_Stage0_frm[8].positions[12] = 1590;
  WalkRight_Stage0_frm[8].positions[13] = 1502;
  WalkRight_Stage0_frm[8].positions[14] = 1500;
  WalkRight_Stage0_frm[8].positions[15] = 1654;
  WalkRight_Stage0_frm[8].positions[16] = 1409;
  WalkRight_Stage0_frm[8].positions[17] = 1702;
  WalkRight_Stage0_frm[8].positions[18] = 1456;
  WalkRight_Stage0_frm[8].positions[19] = 1409;

  WalkRight_Stage0_frm[9].positions[0] = 1500;
  WalkRight_Stage0_frm[9].positions[1] = 1470;
  WalkRight_Stage0_frm[9].positions[2] = 1500;
  WalkRight_Stage0_frm[9].positions[3] = 1502;
  WalkRight_Stage0_frm[9].positions[4] = 1501;
  WalkRight_Stage0_frm[9].positions[5] = 1310;
  WalkRight_Stage0_frm[9].positions[6] = 1417;
  WalkRight_Stage0_frm[9].positions[7] = 1378;
  WalkRight_Stage0_frm[9].positions[8] = 1595;
  WalkRight_Stage0_frm[9].positions[9] = 1417;
  WalkRight_Stage0_frm[9].positions[10] = 1500;
  WalkRight_Stage0_frm[9].positions[11] = 1860;
  WalkRight_Stage0_frm[9].positions[12] = 1590;
  WalkRight_Stage0_frm[9].positions[13] = 1502;
  WalkRight_Stage0_frm[9].positions[14] = 1500;
  WalkRight_Stage0_frm[9].positions[15] = 1694;
  WalkRight_Stage0_frm[9].positions[16] = 1483;
  WalkRight_Stage0_frm[9].positions[17] = 1623;
  WalkRight_Stage0_frm[9].positions[18] = 1417;
  WalkRight_Stage0_frm[9].positions[19] = 1483;

  WalkRight_Stage0_frm[10].positions[0] = 1500;
  WalkRight_Stage0_frm[10].positions[1] = 1470;
  WalkRight_Stage0_frm[10].positions[2] = 1500;
  WalkRight_Stage0_frm[10].positions[3] = 1501;
  WalkRight_Stage0_frm[10].positions[4] = 1500;
  WalkRight_Stage0_frm[10].positions[5] = 1284;
  WalkRight_Stage0_frm[10].positions[6] = 1495;
  WalkRight_Stage0_frm[10].positions[7] = 1420;
  WalkRight_Stage0_frm[10].positions[8] = 1604;
  WalkRight_Stage0_frm[10].positions[9] = 1495;
  WalkRight_Stage0_frm[10].positions[10] = 1500;
  WalkRight_Stage0_frm[10].positions[11] = 1860;
  WalkRight_Stage0_frm[10].positions[12] = 1590;
  WalkRight_Stage0_frm[10].positions[13] = 1501;
  WalkRight_Stage0_frm[10].positions[14] = 1500;
  WalkRight_Stage0_frm[10].positions[15] = 1705;
  WalkRight_Stage0_frm[10].positions[16] = 1559;
  WalkRight_Stage0_frm[10].positions[17] = 1602;
  WalkRight_Stage0_frm[10].positions[18] = 1407;
  WalkRight_Stage0_frm[10].positions[19] = 1559;

  WalkRight_Stage1_frm[0].positions[0] = 1500;
  WalkRight_Stage1_frm[0].positions[1] = 1470;
  WalkRight_Stage1_frm[0].positions[2] = 1500;
  WalkRight_Stage1_frm[0].positions[3] = 1501;
  WalkRight_Stage1_frm[0].positions[4] = 1500;
  WalkRight_Stage1_frm[0].positions[5] = 1282;
  WalkRight_Stage1_frm[0].positions[6] = 1520;
  WalkRight_Stage1_frm[0].positions[7] = 1424;
  WalkRight_Stage1_frm[0].positions[8] = 1606;
  WalkRight_Stage1_frm[0].positions[9] = 1520;
  WalkRight_Stage1_frm[0].positions[10] = 1500;
  WalkRight_Stage1_frm[0].positions[11] = 1860;
  WalkRight_Stage1_frm[0].positions[12] = 1590;
  WalkRight_Stage1_frm[0].positions[13] = 1501;
  WalkRight_Stage1_frm[0].positions[14] = 1500;
  WalkRight_Stage1_frm[0].positions[15] = 1697;
  WalkRight_Stage1_frm[0].positions[16] = 1584;
  WalkRight_Stage1_frm[0].positions[17] = 1618;
  WalkRight_Stage1_frm[0].positions[18] = 1415;
  WalkRight_Stage1_frm[0].positions[19] = 1584;

  WalkRight_Stage1_frm[1].positions[0] = 1500;
  WalkRight_Stage1_frm[1].positions[1] = 1470;
  WalkRight_Stage1_frm[1].positions[2] = 1500;
  WalkRight_Stage1_frm[1].positions[3] = 1505;
  WalkRight_Stage1_frm[1].positions[4] = 1500;
  WalkRight_Stage1_frm[1].positions[5] = 1300;
  WalkRight_Stage1_frm[1].positions[6] = 1565;
  WalkRight_Stage1_frm[1].positions[7] = 1388;
  WalkRight_Stage1_frm[1].positions[8] = 1588;
  WalkRight_Stage1_frm[1].positions[9] = 1565;
  WalkRight_Stage1_frm[1].positions[10] = 1500;
  WalkRight_Stage1_frm[1].positions[11] = 1860;
  WalkRight_Stage1_frm[1].positions[12] = 1590;
  WalkRight_Stage1_frm[1].positions[13] = 1505;
  WalkRight_Stage1_frm[1].positions[14] = 1502;
  WalkRight_Stage1_frm[1].positions[15] = 1685;
  WalkRight_Stage1_frm[1].positions[16] = 1632;
  WalkRight_Stage1_frm[1].positions[17] = 1658;
  WalkRight_Stage1_frm[1].positions[18] = 1453;
  WalkRight_Stage1_frm[1].positions[19] = 1632;

  WalkRight_Stage1_frm[2].positions[0] = 1500;
  WalkRight_Stage1_frm[2].positions[1] = 1470;
  WalkRight_Stage1_frm[2].positions[2] = 1500;
  WalkRight_Stage1_frm[2].positions[3] = 1509;
  WalkRight_Stage1_frm[2].positions[4] = 1500;
  WalkRight_Stage1_frm[2].positions[5] = 1340;
  WalkRight_Stage1_frm[2].positions[6] = 1599;
  WalkRight_Stage1_frm[2].positions[7] = 1310;
  WalkRight_Stage1_frm[2].positions[8] = 1550;
  WalkRight_Stage1_frm[2].positions[9] = 1599;
  WalkRight_Stage1_frm[2].positions[10] = 1500;
  WalkRight_Stage1_frm[2].positions[11] = 1860;
  WalkRight_Stage1_frm[2].positions[12] = 1590;
  WalkRight_Stage1_frm[2].positions[13] = 1509;
  WalkRight_Stage1_frm[2].positions[14] = 1507;
  WalkRight_Stage1_frm[2].positions[15] = 1717;
  WalkRight_Stage1_frm[2].positions[16] = 1669;
  WalkRight_Stage1_frm[2].positions[17] = 1621;
  WalkRight_Stage1_frm[2].positions[18] = 1464;
  WalkRight_Stage1_frm[2].positions[19] = 1669;

  WalkRight_Stage1_frm[3].positions[0] = 1500;
  WalkRight_Stage1_frm[3].positions[1] = 1470;
  WalkRight_Stage1_frm[3].positions[2] = 1500;
  WalkRight_Stage1_frm[3].positions[3] = 1506;
  WalkRight_Stage1_frm[3].positions[4] = 1500;
  WalkRight_Stage1_frm[3].positions[5] = 1371;
  WalkRight_Stage1_frm[3].positions[6] = 1625;
  WalkRight_Stage1_frm[3].positions[7] = 1250;
  WalkRight_Stage1_frm[3].positions[8] = 1520;
  WalkRight_Stage1_frm[3].positions[9] = 1625;
  WalkRight_Stage1_frm[3].positions[10] = 1500;
  WalkRight_Stage1_frm[3].positions[11] = 1860;
  WalkRight_Stage1_frm[3].positions[12] = 1590;
  WalkRight_Stage1_frm[3].positions[13] = 1506;
  WalkRight_Stage1_frm[3].positions[14] = 1509;
  WalkRight_Stage1_frm[3].positions[15] = 1831;
  WalkRight_Stage1_frm[3].positions[16] = 1674;
  WalkRight_Stage1_frm[3].positions[17] = 1391;
  WalkRight_Stage1_frm[3].positions[18] = 1355;
  WalkRight_Stage1_frm[3].positions[19] = 1674;

  WalkRight_Stage1_frm[4].positions[0] = 1500;
  WalkRight_Stage1_frm[4].positions[1] = 1470;
  WalkRight_Stage1_frm[4].positions[2] = 1500;
  WalkRight_Stage1_frm[4].positions[3] = 1496;
  WalkRight_Stage1_frm[4].positions[4] = 1500;
  WalkRight_Stage1_frm[4].positions[5] = 1357;
  WalkRight_Stage1_frm[4].positions[6] = 1616;
  WalkRight_Stage1_frm[4].positions[7] = 1276;
  WalkRight_Stage1_frm[4].positions[8] = 1533;
  WalkRight_Stage1_frm[4].positions[9] = 1616;
  WalkRight_Stage1_frm[4].positions[10] = 1500;
  WalkRight_Stage1_frm[4].positions[11] = 1860;
  WalkRight_Stage1_frm[4].positions[12] = 1590;
  WalkRight_Stage1_frm[4].positions[13] = 1496;
  WalkRight_Stage1_frm[4].positions[14] = 1500;
  WalkRight_Stage1_frm[4].positions[15] = 1834;
  WalkRight_Stage1_frm[4].positions[16] = 1585;
  WalkRight_Stage1_frm[4].positions[17] = 1335;
  WalkRight_Stage1_frm[4].positions[18] = 1272;
  WalkRight_Stage1_frm[4].positions[19] = 1585;

  WalkRight_Stage1_frm[5].positions[0] = 1500;
  WalkRight_Stage1_frm[5].positions[1] = 1470;
  WalkRight_Stage1_frm[5].positions[2] = 1500;
  WalkRight_Stage1_frm[5].positions[3] = 1496;
  WalkRight_Stage1_frm[5].positions[4] = 1500;
  WalkRight_Stage1_frm[5].positions[5] = 1335;
  WalkRight_Stage1_frm[5].positions[6] = 1567;
  WalkRight_Stage1_frm[5].positions[7] = 1319;
  WalkRight_Stage1_frm[5].positions[8] = 1554;
  WalkRight_Stage1_frm[5].positions[9] = 1567;
  WalkRight_Stage1_frm[5].positions[10] = 1500;
  WalkRight_Stage1_frm[5].positions[11] = 1860;
  WalkRight_Stage1_frm[5].positions[12] = 1590;
  WalkRight_Stage1_frm[5].positions[13] = 1496;
  WalkRight_Stage1_frm[5].positions[14] = 1500;
  WalkRight_Stage1_frm[5].positions[15] = 1706;
  WalkRight_Stage1_frm[5].positions[16] = 1495;
  WalkRight_Stage1_frm[5].positions[17] = 1582;
  WalkRight_Stage1_frm[5].positions[18] = 1378;
  WalkRight_Stage1_frm[5].positions[19] = 1495;

  WalkRight_Stage1_frm[6].positions[0] = 1500;
  WalkRight_Stage1_frm[6].positions[1] = 1470;
  WalkRight_Stage1_frm[6].positions[2] = 1500;
  WalkRight_Stage1_frm[6].positions[3] = 1499;
  WalkRight_Stage1_frm[6].positions[4] = 1500;
  WalkRight_Stage1_frm[6].positions[5] = 1321;
  WalkRight_Stage1_frm[6].positions[6] = 1518;
  WalkRight_Stage1_frm[6].positions[7] = 1348;
  WalkRight_Stage1_frm[6].positions[8] = 1568;
  WalkRight_Stage1_frm[6].positions[9] = 1518;
  WalkRight_Stage1_frm[6].positions[10] = 1500;
  WalkRight_Stage1_frm[6].positions[11] = 1860;
  WalkRight_Stage1_frm[6].positions[12] = 1590;
  WalkRight_Stage1_frm[6].positions[13] = 1499;
  WalkRight_Stage1_frm[6].positions[14] = 1500;
  WalkRight_Stage1_frm[6].positions[15] = 1670;
  WalkRight_Stage1_frm[6].positions[16] = 1450;
  WalkRight_Stage1_frm[6].positions[17] = 1670;
  WalkRight_Stage1_frm[6].positions[18] = 1440;
  WalkRight_Stage1_frm[6].positions[19] = 1450;

  WalkRight_Stage1_frm[7].positions[0] = 1500;
  WalkRight_Stage1_frm[7].positions[1] = 1470;
  WalkRight_Stage1_frm[7].positions[2] = 1500;
  WalkRight_Stage1_frm[7].positions[3] = 1499;
  WalkRight_Stage1_frm[7].positions[4] = 1500;
  WalkRight_Stage1_frm[7].positions[5] = 1318;
  WalkRight_Stage1_frm[7].positions[6] = 1502;
  WalkRight_Stage1_frm[7].positions[7] = 1354;
  WalkRight_Stage1_frm[7].positions[8] = 1571;
  WalkRight_Stage1_frm[7].positions[9] = 1502;
  WalkRight_Stage1_frm[7].positions[10] = 1500;
  WalkRight_Stage1_frm[7].positions[11] = 1860;
  WalkRight_Stage1_frm[7].positions[12] = 1590;
  WalkRight_Stage1_frm[7].positions[13] = 1499;
  WalkRight_Stage1_frm[7].positions[14] = 1500;
  WalkRight_Stage1_frm[7].positions[15] = 1664;
  WalkRight_Stage1_frm[7].positions[16] = 1434;
  WalkRight_Stage1_frm[7].positions[17] = 1683;
  WalkRight_Stage1_frm[7].positions[18] = 1447;
  WalkRight_Stage1_frm[7].positions[19] = 1434;

  WalkRight_Stage1_frm[8].positions[0] = 1500;
  WalkRight_Stage1_frm[8].positions[1] = 1470;
  WalkRight_Stage1_frm[8].positions[2] = 1500;
  WalkRight_Stage1_frm[8].positions[3] = 1495;
  WalkRight_Stage1_frm[8].positions[4] = 1500;
  WalkRight_Stage1_frm[8].positions[5] = 1279;
  WalkRight_Stage1_frm[8].positions[6] = 1476;
  WalkRight_Stage1_frm[8].positions[7] = 1412;
  WalkRight_Stage1_frm[8].positions[8] = 1581;
  WalkRight_Stage1_frm[8].positions[9] = 1476;
  WalkRight_Stage1_frm[8].positions[10] = 1500;
  WalkRight_Stage1_frm[8].positions[11] = 1860;
  WalkRight_Stage1_frm[8].positions[12] = 1590;
  WalkRight_Stage1_frm[8].positions[13] = 1495;
  WalkRight_Stage1_frm[8].positions[14] = 1500;
  WalkRight_Stage1_frm[8].positions[15] = 1651;
  WalkRight_Stage1_frm[8].positions[16] = 1406;
  WalkRight_Stage1_frm[8].positions[17] = 1709;
  WalkRight_Stage1_frm[8].positions[18] = 1459;
  WalkRight_Stage1_frm[8].positions[19] = 1406;

  WalkRight_Stage1_frm[9].positions[0] = 1500;
  WalkRight_Stage1_frm[9].positions[1] = 1470;
  WalkRight_Stage1_frm[9].positions[2] = 1500;
  WalkRight_Stage1_frm[9].positions[3] = 1491;
  WalkRight_Stage1_frm[9].positions[4] = 1498;
  WalkRight_Stage1_frm[9].positions[5] = 1207;
  WalkRight_Stage1_frm[9].positions[6] = 1454;
  WalkRight_Stage1_frm[9].positions[7] = 1523;
  WalkRight_Stage1_frm[9].positions[8] = 1604;
  WalkRight_Stage1_frm[9].positions[9] = 1454;
  WalkRight_Stage1_frm[9].positions[10] = 1500;
  WalkRight_Stage1_frm[9].positions[11] = 1860;
  WalkRight_Stage1_frm[9].positions[12] = 1590;
  WalkRight_Stage1_frm[9].positions[13] = 1491;
  WalkRight_Stage1_frm[9].positions[14] = 1500;
  WalkRight_Stage1_frm[9].positions[15] = 1642;
  WalkRight_Stage1_frm[9].positions[16] = 1387;
  WalkRight_Stage1_frm[9].positions[17] = 1725;
  WalkRight_Stage1_frm[9].positions[18] = 1467;
  WalkRight_Stage1_frm[9].positions[19] = 1387;

  WalkRight_Stage1_frm[10].positions[0] = 1500;
  WalkRight_Stage1_frm[10].positions[1] = 1470;
  WalkRight_Stage1_frm[10].positions[2] = 1500;
  WalkRight_Stage1_frm[10].positions[3] = 1494;
  WalkRight_Stage1_frm[10].positions[4] = 1495;
  WalkRight_Stage1_frm[10].positions[5] = 1128;
  WalkRight_Stage1_frm[10].positions[6] = 1407;
  WalkRight_Stage1_frm[10].positions[7] = 1683;
  WalkRight_Stage1_frm[10].positions[8] = 1679;
  WalkRight_Stage1_frm[10].positions[9] = 1407;
  WalkRight_Stage1_frm[10].positions[10] = 1500;
  WalkRight_Stage1_frm[10].positions[11] = 1860;
  WalkRight_Stage1_frm[10].positions[12] = 1590;
  WalkRight_Stage1_frm[10].positions[13] = 1494;
  WalkRight_Stage1_frm[10].positions[14] = 1500;
  WalkRight_Stage1_frm[10].positions[15] = 1639;
  WalkRight_Stage1_frm[10].positions[16] = 1379;
  WalkRight_Stage1_frm[10].positions[17] = 1732;
  WalkRight_Stage1_frm[10].positions[18] = 1471;
  WalkRight_Stage1_frm[10].positions[19] = 1379;

  WalkRight_Stage1_frm[11].positions[0] = 1500;
  WalkRight_Stage1_frm[11].positions[1] = 1470;
  WalkRight_Stage1_frm[11].positions[2] = 1500;
  WalkRight_Stage1_frm[11].positions[3] = 1504;
  WalkRight_Stage1_frm[11].positions[4] = 1499;
  WalkRight_Stage1_frm[11].positions[5] = 1190;
  WalkRight_Stage1_frm[11].positions[6] = 1368;
  WalkRight_Stage1_frm[11].positions[7] = 1619;
  WalkRight_Stage1_frm[11].positions[8] = 1706;
  WalkRight_Stage1_frm[11].positions[9] = 1368;
  WalkRight_Stage1_frm[11].positions[10] = 1500;
  WalkRight_Stage1_frm[11].positions[11] = 1860;
  WalkRight_Stage1_frm[11].positions[12] = 1590;
  WalkRight_Stage1_frm[11].positions[13] = 1504;
  WalkRight_Stage1_frm[11].positions[14] = 1500;
  WalkRight_Stage1_frm[11].positions[15] = 1657;
  WalkRight_Stage1_frm[11].positions[16] = 1411;
  WalkRight_Stage1_frm[11].positions[17] = 1696;
  WalkRight_Stage1_frm[11].positions[18] = 1453;
  WalkRight_Stage1_frm[11].positions[19] = 1411;

  WalkRight_Stage1_frm[12].positions[0] = 1500;
  WalkRight_Stage1_frm[12].positions[1] = 1470;
  WalkRight_Stage1_frm[12].positions[2] = 1500;
  WalkRight_Stage1_frm[12].positions[3] = 1504;
  WalkRight_Stage1_frm[12].positions[4] = 1501;
  WalkRight_Stage1_frm[12].positions[5] = 1312;
  WalkRight_Stage1_frm[12].positions[6] = 1423;
  WalkRight_Stage1_frm[12].positions[7] = 1382;
  WalkRight_Stage1_frm[12].positions[8] = 1604;
  WalkRight_Stage1_frm[12].positions[9] = 1423;
  WalkRight_Stage1_frm[12].positions[10] = 1500;
  WalkRight_Stage1_frm[12].positions[11] = 1860;
  WalkRight_Stage1_frm[12].positions[12] = 1590;
  WalkRight_Stage1_frm[12].positions[13] = 1504;
  WalkRight_Stage1_frm[12].positions[14] = 1500;
  WalkRight_Stage1_frm[12].positions[15] = 1685;
  WalkRight_Stage1_frm[12].positions[16] = 1491;
  WalkRight_Stage1_frm[12].positions[17] = 1641;
  WalkRight_Stage1_frm[12].positions[18] = 1426;
  WalkRight_Stage1_frm[12].positions[19] = 1491;

  WalkRight_Stage1_frm[13].positions[0] = 1500;
  WalkRight_Stage1_frm[13].positions[1] = 1470;
  WalkRight_Stage1_frm[13].positions[2] = 1500;
  WalkRight_Stage1_frm[13].positions[3] = 1501;
  WalkRight_Stage1_frm[13].positions[4] = 1500;
  WalkRight_Stage1_frm[13].positions[5] = 1298;
  WalkRight_Stage1_frm[13].positions[6] = 1504;
  WalkRight_Stage1_frm[13].positions[7] = 1392;
  WalkRight_Stage1_frm[13].positions[8] = 1590;
  WalkRight_Stage1_frm[13].positions[9] = 1504;
  WalkRight_Stage1_frm[13].positions[10] = 1500;
  WalkRight_Stage1_frm[13].positions[11] = 1860;
  WalkRight_Stage1_frm[13].positions[12] = 1590;
  WalkRight_Stage1_frm[13].positions[13] = 1501;
  WalkRight_Stage1_frm[13].positions[14] = 1500;
  WalkRight_Stage1_frm[13].positions[15] = 1685;
  WalkRight_Stage1_frm[13].positions[16] = 1568;
  WalkRight_Stage1_frm[13].positions[17] = 1641;
  WalkRight_Stage1_frm[13].positions[18] = 1426;
  WalkRight_Stage1_frm[13].positions[19] = 1568;

  WalkRight_Stage2_frm[0].positions[0] = 1500;
  WalkRight_Stage2_frm[0].positions[1] = 1470;
  WalkRight_Stage2_frm[0].positions[2] = 1500;
  WalkRight_Stage2_frm[0].positions[3] = 1501;
  WalkRight_Stage2_frm[0].positions[4] = 1500;
  WalkRight_Stage2_frm[0].positions[5] = 1294;
  WalkRight_Stage2_frm[0].positions[6] = 1529;
  WalkRight_Stage2_frm[0].positions[7] = 1401;
  WalkRight_Stage2_frm[0].positions[8] = 1594;
  WalkRight_Stage2_frm[0].positions[9] = 1529;
  WalkRight_Stage2_frm[0].positions[10] = 1500;
  WalkRight_Stage2_frm[0].positions[11] = 1860;
  WalkRight_Stage2_frm[0].positions[12] = 1590;
  WalkRight_Stage2_frm[0].positions[13] = 1501;
  WalkRight_Stage2_frm[0].positions[14] = 1500;
  WalkRight_Stage2_frm[0].positions[15] = 1678;
  WalkRight_Stage2_frm[0].positions[16] = 1593;
  WalkRight_Stage2_frm[0].positions[17] = 1655;
  WalkRight_Stage2_frm[0].positions[18] = 1433;
  WalkRight_Stage2_frm[0].positions[19] = 1593;

  WalkRight_Stage2_frm[1].positions[0] = 1500;
  WalkRight_Stage2_frm[1].positions[1] = 1470;
  WalkRight_Stage2_frm[1].positions[2] = 1500;
  WalkRight_Stage2_frm[1].positions[3] = 1503;
  WalkRight_Stage2_frm[1].positions[4] = 1500;
  WalkRight_Stage2_frm[1].positions[5] = 1299;
  WalkRight_Stage2_frm[1].positions[6] = 1575;
  WalkRight_Stage2_frm[1].positions[7] = 1391;
  WalkRight_Stage2_frm[1].positions[8] = 1590;
  WalkRight_Stage2_frm[1].positions[9] = 1575;
  WalkRight_Stage2_frm[1].positions[10] = 1500;
  WalkRight_Stage2_frm[1].positions[11] = 1860;
  WalkRight_Stage2_frm[1].positions[12] = 1590;
  WalkRight_Stage2_frm[1].positions[13] = 1503;
  WalkRight_Stage2_frm[1].positions[14] = 1501;
  WalkRight_Stage2_frm[1].positions[15] = 1674;
  WalkRight_Stage2_frm[1].positions[16] = 1640;
  WalkRight_Stage2_frm[1].positions[17] = 1672;
  WalkRight_Stage2_frm[1].positions[18] = 1452;
  WalkRight_Stage2_frm[1].positions[19] = 1640;

  WalkRight_Stage2_frm[2].positions[0] = 1500;
  WalkRight_Stage2_frm[2].positions[1] = 1470;
  WalkRight_Stage2_frm[2].positions[2] = 1500;
  WalkRight_Stage2_frm[2].positions[3] = 1505;
  WalkRight_Stage2_frm[2].positions[4] = 1500;
  WalkRight_Stage2_frm[2].positions[5] = 1324;
  WalkRight_Stage2_frm[2].positions[6] = 1608;
  WalkRight_Stage2_frm[2].positions[7] = 1341;
  WalkRight_Stage2_frm[2].positions[8] = 1565;
  WalkRight_Stage2_frm[2].positions[9] = 1608;
  WalkRight_Stage2_frm[2].positions[10] = 1500;
  WalkRight_Stage2_frm[2].positions[11] = 1860;
  WalkRight_Stage2_frm[2].positions[12] = 1590;
  WalkRight_Stage2_frm[2].positions[13] = 1505;
  WalkRight_Stage2_frm[2].positions[14] = 1504;
  WalkRight_Stage2_frm[2].positions[15] = 1708;
  WalkRight_Stage2_frm[2].positions[16] = 1676;
  WalkRight_Stage2_frm[2].positions[17] = 1621;
  WalkRight_Stage2_frm[2].positions[18] = 1444;
  WalkRight_Stage2_frm[2].positions[19] = 1676;

  WalkRight_Stage2_frm[3].positions[0] = 1500;
  WalkRight_Stage2_frm[3].positions[1] = 1470;
  WalkRight_Stage2_frm[3].positions[2] = 1500;
  WalkRight_Stage2_frm[3].positions[3] = 1504;
  WalkRight_Stage2_frm[3].positions[4] = 1500;
  WalkRight_Stage2_frm[3].positions[5] = 1371;
  WalkRight_Stage2_frm[3].positions[6] = 1633;
  WalkRight_Stage2_frm[3].positions[7] = 1248;
  WalkRight_Stage2_frm[3].positions[8] = 1520;
  WalkRight_Stage2_frm[3].positions[9] = 1633;
  WalkRight_Stage2_frm[3].positions[10] = 1500;
  WalkRight_Stage2_frm[3].positions[11] = 1860;
  WalkRight_Stage2_frm[3].positions[12] = 1590;
  WalkRight_Stage2_frm[3].positions[13] = 1504;
  WalkRight_Stage2_frm[3].positions[14] = 1506;
  WalkRight_Stage2_frm[3].positions[15] = 1810;
  WalkRight_Stage2_frm[3].positions[16] = 1694;
  WalkRight_Stage2_frm[3].positions[17] = 1416;
  WalkRight_Stage2_frm[3].positions[18] = 1346;
  WalkRight_Stage2_frm[3].positions[19] = 1694;

  WalkRight_Stage2_frm[4].positions[0] = 1500;
  WalkRight_Stage2_frm[4].positions[1] = 1470;
  WalkRight_Stage2_frm[4].positions[2] = 1500;
  WalkRight_Stage2_frm[4].positions[3] = 1498;
  WalkRight_Stage2_frm[4].positions[4] = 1500;
  WalkRight_Stage2_frm[4].positions[5] = 1366;
  WalkRight_Stage2_frm[4].positions[6] = 1629;
  WalkRight_Stage2_frm[4].positions[7] = 1259;
  WalkRight_Stage2_frm[4].positions[8] = 1525;
  WalkRight_Stage2_frm[4].positions[9] = 1629;
  WalkRight_Stage2_frm[4].positions[10] = 1500;
  WalkRight_Stage2_frm[4].positions[11] = 1860;
  WalkRight_Stage2_frm[4].positions[12] = 1590;
  WalkRight_Stage2_frm[4].positions[13] = 1498;
  WalkRight_Stage2_frm[4].positions[14] = 1500;
  WalkRight_Stage2_frm[4].positions[15] = 1815;
  WalkRight_Stage2_frm[4].positions[16] = 1644;
  WalkRight_Stage2_frm[4].positions[17] = 1378;
  WalkRight_Stage2_frm[4].positions[18] = 1295;
  WalkRight_Stage2_frm[4].positions[19] = 1644;

  WalkRight_Stage2_frm[5].positions[0] = 1500;
  WalkRight_Stage2_frm[5].positions[1] = 1470;
  WalkRight_Stage2_frm[5].positions[2] = 1500;
  WalkRight_Stage2_frm[5].positions[3] = 1498;
  WalkRight_Stage2_frm[5].positions[4] = 1500;
  WalkRight_Stage2_frm[5].positions[5] = 1341;
  WalkRight_Stage2_frm[5].positions[6] = 1595;
  WalkRight_Stage2_frm[5].positions[7] = 1308;
  WalkRight_Stage2_frm[5].positions[8] = 1549;
  WalkRight_Stage2_frm[5].positions[9] = 1595;
  WalkRight_Stage2_frm[5].positions[10] = 1500;
  WalkRight_Stage2_frm[5].positions[11] = 1860;
  WalkRight_Stage2_frm[5].positions[12] = 1590;
  WalkRight_Stage2_frm[5].positions[13] = 1498;
  WalkRight_Stage2_frm[5].positions[14] = 1499;
  WalkRight_Stage2_frm[5].positions[15] = 1693;
  WalkRight_Stage2_frm[5].positions[16] = 1579;
  WalkRight_Stage2_frm[5].positions[17] = 1616;
  WalkRight_Stage2_frm[5].positions[18] = 1402;
  WalkRight_Stage2_frm[5].positions[19] = 1579;

  WalkRight_Stage2_frm[6].positions[0] = 1500;
  WalkRight_Stage2_frm[6].positions[1] = 1470;
  WalkRight_Stage2_frm[6].positions[2] = 1500;
  WalkRight_Stage2_frm[6].positions[3] = 1500;
  WalkRight_Stage2_frm[6].positions[4] = 1500;
  WalkRight_Stage2_frm[6].positions[5] = 1318;
  WalkRight_Stage2_frm[6].positions[6] = 1562;
  WalkRight_Stage2_frm[6].positions[7] = 1354;
  WalkRight_Stage2_frm[6].positions[8] = 1572;
  WalkRight_Stage2_frm[6].positions[9] = 1562;
  WalkRight_Stage2_frm[6].positions[10] = 1500;
  WalkRight_Stage2_frm[6].positions[11] = 1860;
  WalkRight_Stage2_frm[6].positions[12] = 1590;
  WalkRight_Stage2_frm[6].positions[13] = 1500;
  WalkRight_Stage2_frm[6].positions[14] = 1500;
  WalkRight_Stage2_frm[6].positions[15] = 1688;
  WalkRight_Stage2_frm[6].positions[16] = 1548;
  WalkRight_Stage2_frm[6].positions[17] = 1635;
  WalkRight_Stage2_frm[6].positions[18] = 1423;
  WalkRight_Stage2_frm[6].positions[19] = 1548;

  WalkRight_Stage2_frm[7].positions[0] = 1500;
  WalkRight_Stage2_frm[7].positions[1] = 1470;
  WalkRight_Stage2_frm[7].positions[2] = 1500;
  WalkRight_Stage2_frm[7].positions[3] = 1500;
  WalkRight_Stage2_frm[7].positions[4] = 1500;
  WalkRight_Stage2_frm[7].positions[5] = 1315;
  WalkRight_Stage2_frm[7].positions[6] = 1552;
  WalkRight_Stage2_frm[7].positions[7] = 1358;
  WalkRight_Stage2_frm[7].positions[8] = 1574;
  WalkRight_Stage2_frm[7].positions[9] = 1552;
  WalkRight_Stage2_frm[7].positions[10] = 1500;
  WalkRight_Stage2_frm[7].positions[11] = 1860;
  WalkRight_Stage2_frm[7].positions[12] = 1590;
  WalkRight_Stage2_frm[7].positions[13] = 1500;
  WalkRight_Stage2_frm[7].positions[14] = 1500;
  WalkRight_Stage2_frm[7].positions[15] = 1689;
  WalkRight_Stage2_frm[7].positions[16] = 1538;
  WalkRight_Stage2_frm[7].positions[17] = 1632;
  WalkRight_Stage2_frm[7].positions[18] = 1422;
  WalkRight_Stage2_frm[7].positions[19] = 1538;

  WalkRight_Stage2_frm[8].positions[0] = 1500;
  WalkRight_Stage2_frm[8].positions[1] = 1470;
  WalkRight_Stage2_frm[8].positions[2] = 1500;
  WalkRight_Stage2_frm[8].positions[3] = 1500;
  WalkRight_Stage2_frm[8].positions[4] = 1500;
  WalkRight_Stage2_frm[8].positions[5] = 1317;
  WalkRight_Stage2_frm[8].positions[6] = 1513;
  WalkRight_Stage2_frm[8].positions[7] = 1355;
  WalkRight_Stage2_frm[8].positions[8] = 1572;
  WalkRight_Stage2_frm[8].positions[9] = 1513;
  WalkRight_Stage2_frm[8].positions[10] = 1500;
  WalkRight_Stage2_frm[8].positions[11] = 1860;
  WalkRight_Stage2_frm[8].positions[12] = 1590;
  WalkRight_Stage2_frm[8].positions[13] = 1500;
  WalkRight_Stage2_frm[8].positions[14] = 1500;
  WalkRight_Stage2_frm[8].positions[15] = 1684;
  WalkRight_Stage2_frm[8].positions[16] = 1499;
  WalkRight_Stage2_frm[8].positions[17] = 1643;
  WalkRight_Stage2_frm[8].positions[18] = 1427;
  WalkRight_Stage2_frm[8].positions[19] = 1499;

  WalkRight_Stage2_frm[9].positions[0] = 1500;
  WalkRight_Stage2_frm[9].positions[1] = 1470;
  WalkRight_Stage2_frm[9].positions[2] = 1500;
  WalkRight_Stage2_frm[9].positions[3] = 1500;
  WalkRight_Stage2_frm[9].positions[4] = 1500;
  WalkRight_Stage2_frm[9].positions[5] = 1318;
  WalkRight_Stage2_frm[9].positions[6] = 1516;
  WalkRight_Stage2_frm[9].positions[7] = 1354;
  WalkRight_Stage2_frm[9].positions[8] = 1572;
  WalkRight_Stage2_frm[9].positions[9] = 1516;
  WalkRight_Stage2_frm[9].positions[10] = 1500;
  WalkRight_Stage2_frm[9].positions[11] = 1860;
  WalkRight_Stage2_frm[9].positions[12] = 1590;
  WalkRight_Stage2_frm[9].positions[13] = 1500;
  WalkRight_Stage2_frm[9].positions[14] = 1500;
  WalkRight_Stage2_frm[9].positions[15] = 1683;
  WalkRight_Stage2_frm[9].positions[16] = 1502;
  WalkRight_Stage2_frm[9].positions[17] = 1644;
  WalkRight_Stage2_frm[9].positions[18] = 1427;
  WalkRight_Stage2_frm[9].positions[19] = 1502;

  helloworld_frm[0].positions[0] = 1500;
  helloworld_frm[0].positions[1] = 2300;
  helloworld_frm[0].positions[2] = 2075;
  helloworld_frm[0].positions[3] = 1575;
  helloworld_frm[0].positions[4] = 1500;
  helloworld_frm[0].positions[5] = 1480;
  helloworld_frm[0].positions[6] = 1470;
  helloworld_frm[0].positions[7] = 1190;
  helloworld_frm[0].positions[8] = 1500;
  helloworld_frm[0].positions[9] = 1450;
  helloworld_frm[0].positions[10] = 1425;
  helloworld_frm[0].positions[11] = 1250;
  helloworld_frm[0].positions[12] = 1325;
  helloworld_frm[0].positions[13] = 700;
  helloworld_frm[0].positions[14] = 1500;
  helloworld_frm[0].positions[15] = 1470;
  helloworld_frm[0].positions[16] = 1510;
  helloworld_frm[0].positions[17] = 1750;
  helloworld_frm[0].positions[18] = 1405;
  helloworld_frm[0].positions[19] = 1550;

  helloworld_frm[1].positions[0] = 1500;
  helloworld_frm[1].positions[1] = 2300;
  helloworld_frm[1].positions[2] = 2075;
  helloworld_frm[1].positions[3] = 1575;
  helloworld_frm[1].positions[4] = 1500;
  helloworld_frm[1].positions[5] = 1480;
  helloworld_frm[1].positions[6] = 1470;
  helloworld_frm[1].positions[7] = 1190;
  helloworld_frm[1].positions[8] = 1500;
  helloworld_frm[1].positions[9] = 1450;
  helloworld_frm[1].positions[10] = 1200;
  helloworld_frm[1].positions[11] = 1525;
  helloworld_frm[1].positions[12] = 1325;
  helloworld_frm[1].positions[13] = 700;
  helloworld_frm[1].positions[14] = 1500;
  helloworld_frm[1].positions[15] = 1470;
  helloworld_frm[1].positions[16] = 1510;
  helloworld_frm[1].positions[17] = 1750;
  helloworld_frm[1].positions[18] = 1405;
  helloworld_frm[1].positions[19] = 1550;

  helloworld_frm[2].positions[0] = 1500;
  helloworld_frm[2].positions[1] = 2300;
  helloworld_frm[2].positions[2] = 2075;
  helloworld_frm[2].positions[3] = 1575;
  helloworld_frm[2].positions[4] = 1500;
  helloworld_frm[2].positions[5] = 1480;
  helloworld_frm[2].positions[6] = 1470;
  helloworld_frm[2].positions[7] = 1190;
  helloworld_frm[2].positions[8] = 1500;
  helloworld_frm[2].positions[9] = 1450;
  helloworld_frm[2].positions[10] = 1425;
  helloworld_frm[2].positions[11] = 1250;
  helloworld_frm[2].positions[12] = 1325;
  helloworld_frm[2].positions[13] = 700;
  helloworld_frm[2].positions[14] = 1500;
  helloworld_frm[2].positions[15] = 1470;
  helloworld_frm[2].positions[16] = 1510;
  helloworld_frm[2].positions[17] = 1750;
  helloworld_frm[2].positions[18] = 1405;
  helloworld_frm[2].positions[19] = 1550;

  helloworld_frm[3].positions[0] = 1500;
  helloworld_frm[3].positions[1] = 2300;
  helloworld_frm[3].positions[2] = 2075;
  helloworld_frm[3].positions[3] = 1575;
  helloworld_frm[3].positions[4] = 1500;
  helloworld_frm[3].positions[5] = 1480;
  helloworld_frm[3].positions[6] = 1470;
  helloworld_frm[3].positions[7] = 1190;
  helloworld_frm[3].positions[8] = 1500;
  helloworld_frm[3].positions[9] = 1450;
  helloworld_frm[3].positions[10] = 1200;
  helloworld_frm[3].positions[11] = 1525;
  helloworld_frm[3].positions[12] = 1325;
  helloworld_frm[3].positions[13] = 700;
  helloworld_frm[3].positions[14] = 1500;
  helloworld_frm[3].positions[15] = 1470;
  helloworld_frm[3].positions[16] = 1510;
  helloworld_frm[3].positions[17] = 1750;
  helloworld_frm[3].positions[18] = 1405;
  helloworld_frm[3].positions[19] = 1550;

  helloworld_frm[4].positions[0] = 1500;
  helloworld_frm[4].positions[1] = 1470;
  helloworld_frm[4].positions[2] = 1550;
  helloworld_frm[4].positions[3] = 1500;
  helloworld_frm[4].positions[4] = 1500;
  helloworld_frm[4].positions[5] = 1500;
  helloworld_frm[4].positions[6] = 1500;
  helloworld_frm[4].positions[7] = 1100;
  helloworld_frm[4].positions[8] = 1500;
  helloworld_frm[4].positions[9] = 1500;
  helloworld_frm[4].positions[10] = 1500;
  helloworld_frm[4].positions[11] = 1850;
  helloworld_frm[4].positions[12] = 1520;
  helloworld_frm[4].positions[13] = 1500;
  helloworld_frm[4].positions[14] = 1500;
  helloworld_frm[4].positions[15] = 1500;
  helloworld_frm[4].positions[16] = 1500;
  helloworld_frm[4].positions[17] = 1900;
  helloworld_frm[4].positions[18] = 1500;
  helloworld_frm[4].positions[19] = 1500;

  bow_frm[0].positions[0] = 1500;
  bow_frm[0].positions[1] = 1470;
  bow_frm[0].positions[2] = 1750;
  bow_frm[0].positions[3] = 1500;
  bow_frm[0].positions[4] = 1500;
  bow_frm[0].positions[5] = 1480;
  bow_frm[0].positions[6] = 1470;
  bow_frm[0].positions[7] = 1190;
  bow_frm[0].positions[8] = 1500;
  bow_frm[0].positions[9] = 1450;
  bow_frm[0].positions[10] = 1500;
  bow_frm[0].positions[11] = 1850;
  bow_frm[0].positions[12] = 1345;
  bow_frm[0].positions[13] = 1500;
  bow_frm[0].positions[14] = 1500;
  bow_frm[0].positions[15] = 1550;
  bow_frm[0].positions[16] = 1510;
  bow_frm[0].positions[17] = 1750;
  bow_frm[0].positions[18] = 1465;
  bow_frm[0].positions[19] = 1550;

  bow_frm[1].positions[0] = 1500;
  bow_frm[1].positions[1] = 1470;
  bow_frm[1].positions[2] = 1750;
  bow_frm[1].positions[3] = 1975;
  bow_frm[1].positions[4] = 1500;
  bow_frm[1].positions[5] = 1480;
  bow_frm[1].positions[6] = 1470;
  bow_frm[1].positions[7] = 1190;
  bow_frm[1].positions[8] = 1500;
  bow_frm[1].positions[9] = 1450;
  bow_frm[1].positions[10] = 1500;
  bow_frm[1].positions[11] = 1850;
  bow_frm[1].positions[12] = 1200;
  bow_frm[1].positions[13] = 2150;
  bow_frm[1].positions[14] = 1500;
  bow_frm[1].positions[15] = 1550;
  bow_frm[1].positions[16] = 1510;
  bow_frm[1].positions[17] = 1750;
  bow_frm[1].positions[18] = 1465;
  bow_frm[1].positions[19] = 1550;

  bow_frm[2].positions[0] = 1500;
  bow_frm[2].positions[1] = 2175;
  bow_frm[2].positions[2] = 1650;
  bow_frm[2].positions[3] = 2025;
  bow_frm[2].positions[4] = 1500;
  bow_frm[2].positions[5] = 1330;
  bow_frm[2].positions[6] = 1470;
  bow_frm[2].positions[7] = 1190;
  bow_frm[2].positions[8] = 1500;
  bow_frm[2].positions[9] = 1470;
  bow_frm[2].positions[10] = 1500;
  bow_frm[2].positions[11] = 1200;
  bow_frm[2].positions[12] = 1200;
  bow_frm[2].positions[13] = 1950;
  bow_frm[2].positions[14] = 1500;
  bow_frm[2].positions[15] = 1690;
  bow_frm[2].positions[16] = 1520;
  bow_frm[2].positions[17] = 1750;
  bow_frm[2].positions[18] = 1465;
  bow_frm[2].positions[19] = 1520;

  bow_frm[3].positions[0] = 1500;
  bow_frm[3].positions[1] = 1470;
  bow_frm[3].positions[2] = 1625;
  bow_frm[3].positions[3] = 1500;
  bow_frm[3].positions[4] = 1500;
  bow_frm[3].positions[5] = 1480;
  bow_frm[3].positions[6] = 1470;
  bow_frm[3].positions[7] = 1190;
  bow_frm[3].positions[8] = 1500;
  bow_frm[3].positions[9] = 1450;
  bow_frm[3].positions[10] = 1500;
  bow_frm[3].positions[11] = 1850;
  bow_frm[3].positions[12] = 1475;
  bow_frm[3].positions[13] = 1500;
  bow_frm[3].positions[14] = 1500;
  bow_frm[3].positions[15] = 1550;
  bow_frm[3].positions[16] = 1510;
  bow_frm[3].positions[17] = 1750;
  bow_frm[3].positions[18] = 1465;
  bow_frm[3].positions[19] = 1520;

  grab_frm[0].positions[0] = 1500;
  grab_frm[0].positions[1] = 1470;
  grab_frm[0].positions[2] = 1550;
  grab_frm[0].positions[3] = 1500;
  grab_frm[0].positions[4] = 1500;
  grab_frm[0].positions[5] = 1480;
  grab_frm[0].positions[6] = 1470;
  grab_frm[0].positions[7] = 1170;
  grab_frm[0].positions[8] = 1500;
  grab_frm[0].positions[9] = 1450;
  grab_frm[0].positions[10] = 1500;
  grab_frm[0].positions[11] = 1850;
  grab_frm[0].positions[12] = 1520;
  grab_frm[0].positions[13] = 1500;
  grab_frm[0].positions[14] = 1500;
  grab_frm[0].positions[15] = 1550;
  grab_frm[0].positions[16] = 1540;
  grab_frm[0].positions[17] = 1750;
  grab_frm[0].positions[18] = 1465;
  grab_frm[0].positions[19] = 1550;

  grab_frm[1].positions[0] = 1500;
  grab_frm[1].positions[1] = 1470;
  grab_frm[1].positions[2] = 1550;
  grab_frm[1].positions[3] = 1500;
  grab_frm[1].positions[4] = 1500;
  grab_frm[1].positions[5] = 1480;
  grab_frm[1].positions[6] = 1470;
  grab_frm[1].positions[7] = 1170;
  grab_frm[1].positions[8] = 1500;
  grab_frm[1].positions[9] = 1450;
  grab_frm[1].positions[10] = 1500;
  grab_frm[1].positions[11] = 1850;
  grab_frm[1].positions[12] = 1520;
  grab_frm[1].positions[13] = 1500;
  grab_frm[1].positions[14] = 1500;
  grab_frm[1].positions[15] = 1550;
  grab_frm[1].positions[16] = 1540;
  grab_frm[1].positions[17] = 1750;
  grab_frm[1].positions[18] = 1465;
  grab_frm[1].positions[19] = 1550;

  grab_frm[2].positions[0] = 1500;
  grab_frm[2].positions[1] = 1470;
  grab_frm[2].positions[2] = 1550;
  grab_frm[2].positions[3] = 2225;
  grab_frm[2].positions[4] = 1500;
  grab_frm[2].positions[5] = 1480;
  grab_frm[2].positions[6] = 1470;
  grab_frm[2].positions[7] = 1190;
  grab_frm[2].positions[8] = 1500;
  grab_frm[2].positions[9] = 1450;
  grab_frm[2].positions[10] = 1500;
  grab_frm[2].positions[11] = 1850;
  grab_frm[2].positions[12] = 1520;
  grab_frm[2].positions[13] = 1000;
  grab_frm[2].positions[14] = 1500;
  grab_frm[2].positions[15] = 1550;
  grab_frm[2].positions[16] = 1540;
  grab_frm[2].positions[17] = 1750;
  grab_frm[2].positions[18] = 1465;
  grab_frm[2].positions[19] = 1550;

  grab_frm[3].positions[0] = 1800;
  grab_frm[3].positions[1] = 1470;
  grab_frm[3].positions[2] = 1550;
  grab_frm[3].positions[3] = 2225;
  grab_frm[3].positions[4] = 1500;
  grab_frm[3].positions[5] = 1480;
  grab_frm[3].positions[6] = 1470;
  grab_frm[3].positions[7] = 1190;
  grab_frm[3].positions[8] = 1500;
  grab_frm[3].positions[9] = 1450;
  grab_frm[3].positions[10] = 1500;
  grab_frm[3].positions[11] = 1850;
  grab_frm[3].positions[12] = 1520;
  grab_frm[3].positions[13] = 1000;
  grab_frm[3].positions[14] = 1500;
  grab_frm[3].positions[15] = 1550;
  grab_frm[3].positions[16] = 1540;
  grab_frm[3].positions[17] = 1750;
  grab_frm[3].positions[18] = 1465;
  grab_frm[3].positions[19] = 1550;

  grab_frm[4].positions[0] = 1500;
  grab_frm[4].positions[1] = 1470;
  grab_frm[4].positions[2] = 1550;
  grab_frm[4].positions[3] = 2225;
  grab_frm[4].positions[4] = 1500;
  grab_frm[4].positions[5] = 1480;
  grab_frm[4].positions[6] = 1470;
  grab_frm[4].positions[7] = 1190;
  grab_frm[4].positions[8] = 1500;
  grab_frm[4].positions[9] = 1450;
  grab_frm[4].positions[10] = 1500;
  grab_frm[4].positions[11] = 1850;
  grab_frm[4].positions[12] = 1520;
  grab_frm[4].positions[13] = 1000;
  grab_frm[4].positions[14] = 1500;
  grab_frm[4].positions[15] = 1550;
  grab_frm[4].positions[16] = 1540;
  grab_frm[4].positions[17] = 1750;
  grab_frm[4].positions[18] = 1465;
  grab_frm[4].positions[19] = 1550;

  grab_frm[5].positions[0] = 1750;
  grab_frm[5].positions[1] = 1470;
  grab_frm[5].positions[2] = 1550;
  grab_frm[5].positions[3] = 2225;
  grab_frm[5].positions[4] = 1500;
  grab_frm[5].positions[5] = 1480;
  grab_frm[5].positions[6] = 1470;
  grab_frm[5].positions[7] = 1190;
  grab_frm[5].positions[8] = 1500;
  grab_frm[5].positions[9] = 1450;
  grab_frm[5].positions[10] = 1500;
  grab_frm[5].positions[11] = 1850;
  grab_frm[5].positions[12] = 1520;
  grab_frm[5].positions[13] = 1000;
  grab_frm[5].positions[14] = 1500;
  grab_frm[5].positions[15] = 1550;
  grab_frm[5].positions[16] = 1540;
  grab_frm[5].positions[17] = 1750;
  grab_frm[5].positions[18] = 1465;
  grab_frm[5].positions[19] = 1550;

  grab_frm[6].positions[0] = 1500;
  grab_frm[6].positions[1] = 1470;
  grab_frm[6].positions[2] = 1550;
  grab_frm[6].positions[3] = 2225;
  grab_frm[6].positions[4] = 1500;
  grab_frm[6].positions[5] = 1480;
  grab_frm[6].positions[6] = 1470;
  grab_frm[6].positions[7] = 1190;
  grab_frm[6].positions[8] = 1500;
  grab_frm[6].positions[9] = 1450;
  grab_frm[6].positions[10] = 1500;
  grab_frm[6].positions[11] = 1850;
  grab_frm[6].positions[12] = 1520;
  grab_frm[6].positions[13] = 1000;
  grab_frm[6].positions[14] = 1500;
  grab_frm[6].positions[15] = 1550;
  grab_frm[6].positions[16] = 1540;
  grab_frm[6].positions[17] = 1750;
  grab_frm[6].positions[18] = 1465;
  grab_frm[6].positions[19] = 1550;

  grab_frm[7].positions[0] = 1500;
  grab_frm[7].positions[1] = 1470;
  grab_frm[7].positions[2] = 1550;
  grab_frm[7].positions[3] = 1925;
  grab_frm[7].positions[4] = 1500;
  grab_frm[7].positions[5] = 1480;
  grab_frm[7].positions[6] = 1470;
  grab_frm[7].positions[7] = 1190;
  grab_frm[7].positions[8] = 1500;
  grab_frm[7].positions[9] = 1450;
  grab_frm[7].positions[10] = 1500;
  grab_frm[7].positions[11] = 1850;
  grab_frm[7].positions[12] = 1520;
  grab_frm[7].positions[13] = 875;
  grab_frm[7].positions[14] = 1500;
  grab_frm[7].positions[15] = 1550;
  grab_frm[7].positions[16] = 1540;
  grab_frm[7].positions[17] = 1750;
  grab_frm[7].positions[18] = 1465;
  grab_frm[7].positions[19] = 1550;

  grab_frm[8].positions[0] = 1500;
  grab_frm[8].positions[1] = 1470;
  grab_frm[8].positions[2] = 1550;
  grab_frm[8].positions[3] = 1925;
  grab_frm[8].positions[4] = 1500;
  grab_frm[8].positions[5] = 1480;
  grab_frm[8].positions[6] = 1470;
  grab_frm[8].positions[7] = 1190;
  grab_frm[8].positions[8] = 1500;
  grab_frm[8].positions[9] = 1450;
  grab_frm[8].positions[10] = 1200;
  grab_frm[8].positions[11] = 1850;
  grab_frm[8].positions[12] = 1520;
  grab_frm[8].positions[13] = 875;
  grab_frm[8].positions[14] = 1500;
  grab_frm[8].positions[15] = 1550;
  grab_frm[8].positions[16] = 1540;
  grab_frm[8].positions[17] = 1750;
  grab_frm[8].positions[18] = 1465;
  grab_frm[8].positions[19] = 1550;

  grab_frm[9].positions[0] = 1500;
  grab_frm[9].positions[1] = 1470;
  grab_frm[9].positions[2] = 1550;
  grab_frm[9].positions[3] = 1925;
  grab_frm[9].positions[4] = 1500;
  grab_frm[9].positions[5] = 1480;
  grab_frm[9].positions[6] = 1470;
  grab_frm[9].positions[7] = 1190;
  grab_frm[9].positions[8] = 1500;
  grab_frm[9].positions[9] = 1450;
  grab_frm[9].positions[10] = 1495;
  grab_frm[9].positions[11] = 1850;
  grab_frm[9].positions[12] = 1520;
  grab_frm[9].positions[13] = 875;
  grab_frm[9].positions[14] = 1500;
  grab_frm[9].positions[15] = 1550;
  grab_frm[9].positions[16] = 1540;
  grab_frm[9].positions[17] = 1750;
  grab_frm[9].positions[18] = 1465;
  grab_frm[9].positions[19] = 1550;

  grab_frm[10].positions[0] = 1500;
  grab_frm[10].positions[1] = 1470;
  grab_frm[10].positions[2] = 1550;
  grab_frm[10].positions[3] = 1925;
  grab_frm[10].positions[4] = 1500;
  grab_frm[10].positions[5] = 1480;
  grab_frm[10].positions[6] = 1470;
  grab_frm[10].positions[7] = 1190;
  grab_frm[10].positions[8] = 1500;
  grab_frm[10].positions[9] = 1450;
  grab_frm[10].positions[10] = 1225;
  grab_frm[10].positions[11] = 1850;
  grab_frm[10].positions[12] = 1520;
  grab_frm[10].positions[13] = 875;
  grab_frm[10].positions[14] = 1500;
  grab_frm[10].positions[15] = 1550;
  grab_frm[10].positions[16] = 1540;
  grab_frm[10].positions[17] = 1750;
  grab_frm[10].positions[18] = 1465;
  grab_frm[10].positions[19] = 1550;

  grab_frm[11].positions[0] = 1500;
  grab_frm[11].positions[1] = 1470;
  grab_frm[11].positions[2] = 1550;
  grab_frm[11].positions[3] = 1925;
  grab_frm[11].positions[4] = 1500;
  grab_frm[11].positions[5] = 1480;
  grab_frm[11].positions[6] = 1470;
  grab_frm[11].positions[7] = 1190;
  grab_frm[11].positions[8] = 1500;
  grab_frm[11].positions[9] = 1450;
  grab_frm[11].positions[10] = 1475;
  grab_frm[11].positions[11] = 1850;
  grab_frm[11].positions[12] = 1520;
  grab_frm[11].positions[13] = 875;
  grab_frm[11].positions[14] = 1500;
  grab_frm[11].positions[15] = 1550;
  grab_frm[11].positions[16] = 1540;
  grab_frm[11].positions[17] = 1750;
  grab_frm[11].positions[18] = 1465;
  grab_frm[11].positions[19] = 1550;

  grab_frm[12].positions[0] = 1500;
  grab_frm[12].positions[1] = 1470;
  grab_frm[12].positions[2] = 1550;
  grab_frm[12].positions[3] = 1500;
  grab_frm[12].positions[4] = 1500;
  grab_frm[12].positions[5] = 1480;
  grab_frm[12].positions[6] = 1470;
  grab_frm[12].positions[7] = 1190;
  grab_frm[12].positions[8] = 1500;
  grab_frm[12].positions[9] = 1450;
  grab_frm[12].positions[10] = 1500;
  grab_frm[12].positions[11] = 1850;
  grab_frm[12].positions[12] = 1520;
  grab_frm[12].positions[13] = 1500;
  grab_frm[12].positions[14] = 1500;
  grab_frm[12].positions[15] = 1550;
  grab_frm[12].positions[16] = 1540;
  grab_frm[12].positions[17] = 1750;
  grab_frm[12].positions[18] = 1465;
  grab_frm[12].positions[19] = 1550;

  home_frm[0].positions[0] = 1500;
  home_frm[0].positions[1] = 1470;
  home_frm[0].positions[2] = 1500;
  home_frm[0].positions[3] = 1500;
  home_frm[0].positions[4] = 1500;
  home_frm[0].positions[5] = 1500;
  home_frm[0].positions[6] = 1500;
  home_frm[0].positions[7] = 1100;
  home_frm[0].positions[8] = 1500;
  home_frm[0].positions[9] = 1500;
  home_frm[0].positions[10] = 1500;
  home_frm[0].positions[11] = 1850;
  home_frm[0].positions[12] = 1590;
  home_frm[0].positions[13] = 1500;
  home_frm[0].positions[14] = 1500;
  home_frm[0].positions[15] = 1500;
  home_frm[0].positions[16] = 1500;
  home_frm[0].positions[17] = 1900;
  home_frm[0].positions[18] = 1500;
  home_frm[0].positions[19] = 1500;

  _86ME_HOME.positions[0] = 1500;
  _86ME_HOME.positions[1] = 1470;
  _86ME_HOME.positions[2] = 1500;
  _86ME_HOME.positions[3] = 1500;
  _86ME_HOME.positions[4] = 1500;
  _86ME_HOME.positions[5] = 1500;
  _86ME_HOME.positions[6] = 1500;
  _86ME_HOME.positions[7] = 1100;
  _86ME_HOME.positions[8] = 1500;
  _86ME_HOME.positions[9] = 1500;
  _86ME_HOME.positions[10] = 1500;
  _86ME_HOME.positions[11] = 1850;
  _86ME_HOME.positions[12] = 1590;
  _86ME_HOME.positions[13] = 1500;
  _86ME_HOME.positions[14] = 1500;
  _86ME_HOME.positions[15] = 1500;
  _86ME_HOME.positions[16] = 1500;
  _86ME_HOME.positions[17] = 1900;
  _86ME_HOME.positions[18] = 1500;
  _86ME_HOME.positions[19] = 1500;

  servoBeginSplineMotion(CATMULL_ROM, TurnLeft_Stage0_frm, _TurnLeft_Stage0_frm_time, 11);
  servoBeginSplineMotion(CATMULL_ROM, TurnLeft_Stage1_frm, _TurnLeft_Stage1_frm_time, 14);
  servoBeginSplineMotion(CATMULL_ROM, TurnLeft_Stage2_frm, _TurnLeft_Stage2_frm_time, 10);
  servoBeginSplineMotion(CATMULL_ROM, TurnRight_Stage0_frm, _TurnRight_Stage0_frm_time, 11);
  servoBeginSplineMotion(CATMULL_ROM, TurnRight_Stage1_frm, _TurnRight_Stage1_frm_time, 14);
  servoBeginSplineMotion(CATMULL_ROM, TurnRight_Stage2_frm, _TurnRight_Stage2_frm_time, 10);
  servoBeginSplineMotion(CATMULL_ROM, WalkBwd_Stage0_frm, _WalkBwd_Stage0_frm_time, 11);
  servoBeginSplineMotion(CATMULL_ROM, WalkBwd_Stage1_frm, _WalkBwd_Stage1_frm_time, 14);
  servoBeginSplineMotion(CATMULL_ROM, WalkBwd_Stage2_frm, _WalkBwd_Stage2_frm_time, 10);
  servoBeginSplineMotion(CATMULL_ROM, WalkFwd_Stage0_frm, _WalkFwd_Stage0_frm_time, 11);
  servoBeginSplineMotion(CATMULL_ROM, WalkFwd_Stage1_frm, _WalkFwd_Stage1_frm_time, 14);
  servoBeginSplineMotion(CATMULL_ROM, WalkFwd_Stage2_frm, _WalkFwd_Stage2_frm_time, 10);
  servoBeginSplineMotion(CATMULL_ROM, WalkLeft_Stage0_frm, _WalkLeft_Stage0_frm_time, 11);
  servoBeginSplineMotion(CATMULL_ROM, WalkLeft_Stage1_frm, _WalkLeft_Stage1_frm_time, 14);
  servoBeginSplineMotion(CATMULL_ROM, WalkLeft_Stage2_frm, _WalkLeft_Stage2_frm_time, 10);
  servoBeginSplineMotion(CATMULL_ROM, WalkRight_Stage0_frm, _WalkRight_Stage0_frm_time, 11);
  servoBeginSplineMotion(CATMULL_ROM, WalkRight_Stage1_frm, _WalkRight_Stage1_frm_time, 14);
  servoBeginSplineMotion(CATMULL_ROM, WalkRight_Stage2_frm, _WalkRight_Stage2_frm_time, 10);
  servoBeginSplineMotion(CATMULL_ROM, bow_frm, _bow_frm_time, 4);
  servoBeginSplineMotion(CATMULL_ROM, grab_frm, _grab_frm_time, 13);

  offsets.setOffsets();

  _86ME_HOME.playPositions((unsigned long)0);
}

void Robot86ME::update()
{
  updateCommand();
  updateTrigger();
  TurnLeft_Stage0Update();
  TurnLeft_Stage1Update();
  TurnLeft_Stage2Update();
  TurnRight_Stage0Update();
  TurnRight_Stage1Update();
  TurnRight_Stage2Update();
  WalkBwd_Stage0Update();
  WalkBwd_Stage1Update();
  WalkBwd_Stage2Update();
  WalkFwd_Stage0Update();
  WalkFwd_Stage1Update();
  WalkFwd_Stage2Update();
  WalkLeft_Stage0Update();
  WalkLeft_Stage1Update();
  WalkLeft_Stage2Update();
  WalkRight_Stage0Update();
  WalkRight_Stage1Update();
  WalkRight_Stage2Update();
  TurnLeftUpdate();
  TurnRightUpdate();
  WalkBwdUpdate();
  WalkFwdUpdate();
  WalkLeftUpdate();
  WalkRightUpdate();
  helloworldUpdate();
  bowUpdate();
  grabUpdate();
  homeUpdate();
}
void Robot86ME::update_S()
{
  updateTrigger();
  TurnLeft_Stage0Update();
  TurnLeft_Stage1Update();
  TurnLeft_Stage2Update();
  TurnRight_Stage0Update();
  TurnRight_Stage1Update();
  TurnRight_Stage2Update();
  WalkBwd_Stage0Update();
  WalkBwd_Stage1Update();
  WalkBwd_Stage2Update();
  WalkFwd_Stage0Update();
  WalkFwd_Stage1Update();
  WalkFwd_Stage2Update();
  WalkLeft_Stage0Update();
  WalkLeft_Stage1Update();
  WalkLeft_Stage2Update();
  WalkRight_Stage0Update();
  WalkRight_Stage1Update();
  WalkRight_Stage2Update();
  TurnLeftUpdate();
  TurnRightUpdate();
  WalkBwdUpdate();
  WalkFwdUpdate();
  WalkLeftUpdate();
  WalkRightUpdate();
  helloworldUpdate();
  bowUpdate();
  grabUpdate();
  homeUpdate();
}

Robot86ME robot;



void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
      break ;
  }
  // TODO: save status to EEPROM here, if changed
}

/*
 * Sets the value of an individual pin. Useful if you want to set a pin value but
 * are not tracking the digital port state.
 * Can only be used on pins configured as OUTPUT.
 * Cannot be used to enable pull-ups on Digital INPUT pins.
 */
void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == OUTPUT || Firmata.getPinMode(pin) == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;
          if (Firmata.getPinMode(pin) == OUTPUT) {
            pinWriteMask |= mask;
          } else if (Firmata.getPinMode(pin) == INPUT && pinValue == 1 && Firmata.getPinState(pin) != 1) {
            // only handle INPUT here for backwards compatibility
#if ARDUINO > 100
            pinMode(pin, INPUT_PULLUP);
#else
            // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
            pinWriteMask |= mask;
#endif
          }
          Firmata.setPinState(pin, pinValue);
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;

  byte data;
  int slaveRegister;
  unsigned int delayTime;
  int frequency ;
  int duration ;
  
  long angle;
  long msec;
  
  byte sendBuff[512];
  int i;

  switch (command) {
    
    case KEEP_ALIVE:
      keepAliveInterval = argv[0] + (argv[1] << 7);
      previousKeepAliveMillis = millis();
      break;      
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
        /* calculate number of loops per ping */
        numLoops = INTER_PING_INTERVAL / samplingInterval ;
        //numLoops = 1 ;
      }
      else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case CAPABILITY_QUERY:
      for (i=0; i<sizeof(sendBuff); i++) sendBuff[i] = 0;
      
      sendBuff[0] = START_SYSEX;
      sendBuff[1] = CAPABILITY_RESPONSE;
      i = 2;
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          sendBuff[i++] = ((byte)INPUT);
          sendBuff[i++] = 1;
          sendBuff[i++] = ((byte)PIN_MODE_PULLUP);
          sendBuff[i++] = 1;
          sendBuff[i++] = ((byte)OUTPUT);
          sendBuff[i++] = 1;
        }
        if (IS_PIN_ANALOG(pin)) {
          sendBuff[i++] = PIN_MODE_ANALOG;
          sendBuff[i++] = 10; // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          sendBuff[i++] = PIN_MODE_PWM;
          sendBuff[i++] = DEFAULT_PWM_RESOLUTION;
        }
        if (IS_PIN_DIGITAL(pin)) {
          sendBuff[i++] = PIN_MODE_SERVO;
          sendBuff[i++] = 14;
        }
        if (IS_PIN_I2C(pin)) {
          sendBuff[i++] = PIN_MODE_I2C;
          sendBuff[i++] = 1;  // TODO: could assign a number to map to SCL or SDA
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        sendBuff[i++] = 127;
      }
      sendBuff[i++] = END_SYSEX;
      Firmata.write((const uint8_t*)sendBuff, i);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        for (i=0; i<sizeof(sendBuff); i++) sendBuff[i] = 0;
        
        byte pin = argv[0];
        sendBuff[0] = START_SYSEX;
        sendBuff[1] = PIN_STATE_RESPONSE;
        sendBuff[2] = pin;
        i = 3;
        if (pin < TOTAL_PINS) {
          sendBuff[i++] = Firmata.getPinMode(pin);
          sendBuff[i++] = (byte)Firmata.getPinState(pin) & 0x7F;
          if (Firmata.getPinState(pin) & 0xFF80) sendBuff[i++] = (byte)(Firmata.getPinState(pin) >> 7) & 0x7F;
          if (Firmata.getPinState(pin) & 0xC000) sendBuff[i++] = (byte)(Firmata.getPinState(pin) >> 14) & 0x7F;
        }
        sendBuff[i++] = END_SYSEX;
        Firmata.write((const uint8_t*)sendBuff, i);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      for (i=0; i<sizeof(sendBuff); i++) sendBuff[i] = 0;
      sendBuff[0] = START_SYSEX;
      sendBuff[1] = ANALOG_MAPPING_RESPONSE;
      i = 2;
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        sendBuff[i++] = (IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      sendBuff[i++] = END_SYSEX;
      Firmata.write((const uint8_t*)sendBuff, i);
      break;
    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
    case CHECK_86DUINO_ACTIVE:
      checkActiveStart = true;
      break;

    

    case PERFORM_MOTION:
      busy_id = (int)argv[0] | ((int)argv[1] << 7);
      motion_id = (int)argv[2] | ((int)argv[3] << 7);
      motion_times = (int)argv[4] | ((int)argv[5] << 7);
      if (motion_id == 0) robot.TurnLeft_Stage0(motion_times);
      else if (motion_id == 1) robot.TurnLeft_Stage1(motion_times);
      else if (motion_id == 2) robot.TurnLeft_Stage2(motion_times);
      else if (motion_id == 3) robot.TurnRight_Stage0(motion_times);
      else if (motion_id == 4) robot.TurnRight_Stage1(motion_times);
      else if (motion_id == 5) robot.TurnRight_Stage2(motion_times);
      else if (motion_id == 6) robot.WalkBwd_Stage0(motion_times);
      else if (motion_id == 7) robot.WalkBwd_Stage1(motion_times);
      else if (motion_id == 8) robot.WalkBwd_Stage2(motion_times);
      else if (motion_id == 9) robot.WalkFwd_Stage0(motion_times);
      else if (motion_id == 10) robot.WalkFwd_Stage1(motion_times);
      else if (motion_id == 11) robot.WalkFwd_Stage2(motion_times);
      else if (motion_id == 12) robot.WalkLeft_Stage0(motion_times);
      else if (motion_id == 13) robot.WalkLeft_Stage1(motion_times);
      else if (motion_id == 14) robot.WalkLeft_Stage2(motion_times);
      else if (motion_id == 15) robot.WalkRight_Stage0(motion_times);
      else if (motion_id == 16) robot.WalkRight_Stage1(motion_times);
      else if (motion_id == 17) robot.WalkRight_Stage2(motion_times);
      else if (motion_id == 18) robot.TurnLeft(motion_times);
      else if (motion_id == 19) robot.TurnRight(motion_times);
      else if (motion_id == 20) robot.WalkBwd(motion_times);
      else if (motion_id == 21) robot.WalkFwd(motion_times);
      else if (motion_id == 22) robot.WalkLeft(motion_times);
      else if (motion_id == 23) robot.WalkRight(motion_times);
      else if (motion_id == 24) robot.helloworld(motion_times);
      else if (motion_id == 25) robot.bow(motion_times);
      else if (motion_id == 26) robot.grab(motion_times);
      else if (motion_id == 27) robot.home(motion_times);
      Firmata.write(START_SYSEX);
      Firmata.write(PERFORM_MOTION_RESPONSE);
      Firmata.write(busy_id & 0x7F);
      Firmata.write((busy_id >> 7) & 0x7F);
      Firmata.write(99);
      Firmata.write(END_SYSEX);
      break;
  }
}

void systemResetCallback()
{
  if (isResetting) return;
  isResetting = true;

  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default
#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }
  // pins with analog capability default to analog input
  // otherwise, pins default to digital output

  
  for (byte i = 0; i < 25; i++) {
    if (!IS_PIN_ANALOG(available_digital_pins[i])) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(available_digital_pins[i], OUTPUT);
    }
  }
  for (byte i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    }
  }

  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
  isResetting = false;
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void setup()
{
  #if CONNECT_86DUINO_METHOD == 0
    Firmata.begin(57600);
  #elif CONNECT_86DUINO_METHOD == 1
    Firmata.beginBlueTooth(BT_ESP8266_Serial, BT_ESP8266_Serial_Baud);
  #elif CONNECT_86DUINO_METHOD == 2
    Firmata.beginWiFiShield(projectName, 2000, _ssid, _password, _wep, _ip);
  #elif CONNECT_86DUINO_METHOD == 3
    #ifdef ETHERNET_DHCP
      Firmata.beginEthernet(projectName, 2000);
    #else
      Firmata.beginEthernet(projectName, 2000, localIP, subnet, dnsserver, gateway);
    #endif
  #elif CONNECT_86DUINO_METHOD == 4
      Firmata.beginESP8266(projectName, 2000, BT_ESP8266_Serial, BT_ESP8266_Serial_Baud, ch_pd_pin, _ssid, _password);
  #elif CONNECT_86DUINO_METHOD == 5
      Firmata.beginESP8266_AP(projectName, 2000, BT_ESP8266_Serial, BT_ESP8266_Serial_Baud, ch_pd_pin, _ssid, _password, _chl, _ecn);
  #endif
  
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  #if CONNECT_86DUINO_METHOD == 2
    pinMode(PIN_TO_DIGITAL(4), OUTPUT);    // switch off SD card bypassing Firmata
    digitalWrite(PIN_TO_DIGITAL(4), HIGH); // SS is active low;
  #endif
  robot.begin(false);
  systemResetCallback();  // reset to default config
}

void check_localIP()
{
    static bool serial_monitor_open = false;
    if (Serial && serial_monitor_open == false)
    {
        serial_monitor_open = true;
        Serial.print("IP : ");
        Serial.println(Firmata.getLocalIP());
        Serial.print("Gateway : ");
        Serial.println(Firmata.getGatewayIP());
        Serial.print("SubnetMask : ");
        Serial.println(Firmata.getSubnetMask());
    }
    
    if (!Serial && serial_monitor_open == true)
    {
        serial_monitor_open = false;
    }
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
  byte pin, analogPin;
  int pingResult = 0;

  #if CONNECT_86DUINO_METHOD == 2 || CONNECT_86DUINO_METHOD == 3 || CONNECT_86DUINO_METHOD == 4 || CONNECT_86DUINO_METHOD == 5
    check_localIP();
  #endif
  
  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  checkDigitalInputs();

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available())
    Firmata.processInput();

  // TODO - ensure that Stream buffer doesn't go over 60 bytes

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;

    // ANALOGREAD - do all analogReads() at the configured sampling interval 
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
    if( keepAliveInterval ) {
       currentMillis = millis();
       if (currentMillis - previousKeepAliveMillis > keepAliveInterval*1000) {
         systemResetCallback();
         
         wdt_enable(WDTO_15MS);
         // systemResetCallback();
         while(1)
            ;
      }
    }
    if (checkActiveStart == true)
    {
	  Firmata.write(START_SYSEX);
	  Firmata.write(_86DUINO_RESPONSE) ;
	  Firmata.write(0x5A)  ;
	  Firmata.write(END_SYSEX);
	  checkActiveStart = false;
    }
  }
}

void printData(char * id,  long data)
{
  char myArray[64] ;

  String myString = String(data);
  myString.toCharArray(myArray, 64) ;
  Firmata.sendString(id) ;
  Firmata.sendString(myArray);
}
