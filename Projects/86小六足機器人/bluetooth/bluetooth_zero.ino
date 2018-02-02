#include <FirmataPlus86.h>

#define CONNECT_86DUINO_METHOD 1 // 0: USB Serial 1: BlueTooth 2: Arduino WiFi Shield 3: Ethernet 4: ESP8266 WiFi 5: ESP8266 AP

char* projectName = "86Hexapod";
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


int available_digital_pins[33] = {0, 1, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44};
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
  int servo_mask[12];
  Servo used_servos[12];
  unsigned long _hello_frm_time[2];

  enum {_FORWARD, _LEFT, _RIGHT, _HOME, _IDLE, _HELLO, _NONE};
  int _last_motion[2];
  int _curr_motion[2];
  bool internal_trigger[7];
  bool external_trigger[7];

  ServoOffset offsets;

  ServoFrame _86ME_HOME;
  ServoFrame _86ME_RUN;

  ServoFrame forward_frm[6];
  ServoFrame left_frm[6];
  ServoFrame right_frm[6];
  ServoFrame home_frm[1];
  ServoFrame idle_frm[2];
  ServoFrame hello_frm[2];

  void updateIMU(void);
  bool isBlocked(int layer);
  void closeTriggers(int layer);
  bool isNoMotion(void);
  void updateCompRange(void);
  void updateCommand(void);
  void updateTrigger(void);
  void forwardUpdate(void);
  void leftUpdate(void);
  void rightUpdate(void);
  void homeUpdate(void);
  void idleUpdate(void);
  void helloUpdate(void);
  void update_S(void);
public:
  Robot86ME();
  void begin(bool initIMU);
  void reset(bool initIMU);
  void update(void);
  void forward(int times = 1);
  void left(int times = 1);
  void right(int times = 1);
  void home(int times = 1);
  void idle(int times = 1);
  void hello(int times = 1);
};
namespace forward
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5};
  int state = IDLE;
  unsigned long time;
  double comp_range = 180;
}
namespace left
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5};
  int state = IDLE;
  unsigned long time;
  double comp_range = 180;
}
namespace right
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, FRAME_3, WAIT_FRAME_3, FRAME_4, WAIT_FRAME_4, FRAME_5, WAIT_FRAME_5};
  int state = IDLE;
  unsigned long time;
  double comp_range = 180;
}
namespace home
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0};
  int state = IDLE;
  unsigned long time;
  double comp_range = 180;
}
namespace idle
{
  enum {IDLE, FRAME_0, WAIT_FRAME_0, FRAME_1, WAIT_FRAME_1};
  int state = IDLE;
  unsigned long time;
  double comp_range = 180;
}
namespace hello
{
  enum {IDLE, FLAG_0, FRAME_1, WAIT_FRAME_1, FRAME_2, WAIT_FRAME_2, GOTO_3};
  int state = IDLE;
  unsigned long time;
  int aaa_3 = 0;
  double comp_range = 180;
}


/*==============================================================================
 * FUNCTIONS
 *============================================================================*/


Robot86ME::Robot86ME()
: _86ME_var(), _86ME_cmd(), _roll(0), _pitch(0), _comp_range(180), _IMU_val(),
  _IMU_Q{ 1, 0, 0, 0}, _omega(), _IMU_init_status(-1), servo_mask(),
  _hello_frm_time{ 350, 350},
  _last_motion{ _NONE, _NONE}, _curr_motion{ _NONE, _NONE},
  internal_trigger(), external_trigger()
{ }

bool Robot86ME::isBlocked(int layer)
{
  if(layer == 0)
  {
    if(external_trigger[_HOME]) return true;
    if(external_trigger[_HELLO]) return true;
  }
  return false;
}
void Robot86ME::closeTriggers(int layer)
{
  if(layer == 0)
  {
    external_trigger[_FORWARD]= false; internal_trigger[_FORWARD]= false;
    external_trigger[_LEFT]= false; internal_trigger[_LEFT]= false;
    external_trigger[_RIGHT]= false; internal_trigger[_RIGHT]= false;
    external_trigger[_HOME]= false; internal_trigger[_HOME]= false;
    external_trigger[_IDLE]= false; internal_trigger[_IDLE]= false;
    external_trigger[_HELLO]= false; internal_trigger[_HELLO]= false;
  }
}
bool Robot86ME::isNoMotion()
{
  if(external_trigger[_FORWARD] || internal_trigger[_FORWARD] ||
     external_trigger[_LEFT] || internal_trigger[_LEFT] ||
     external_trigger[_RIGHT] || internal_trigger[_RIGHT] ||
     external_trigger[_HOME] || internal_trigger[_HOME] ||
     external_trigger[_IDLE] || internal_trigger[_IDLE] ||
     external_trigger[_HELLO] || internal_trigger[_HELLO] )
    return false;
  return true;
}
void Robot86ME::updateCompRange()
{
  _comp_range = -1;
  if((external_trigger[_FORWARD] || internal_trigger[_FORWARD]) && forward::comp_range >= _comp_range)
    _comp_range = forward::comp_range;
  else if((external_trigger[_LEFT] || internal_trigger[_LEFT]) && left::comp_range >= _comp_range)
    _comp_range = left::comp_range;
  else if((external_trigger[_RIGHT] || internal_trigger[_RIGHT]) && right::comp_range >= _comp_range)
    _comp_range = right::comp_range;
  else if((external_trigger[_HOME] || internal_trigger[_HOME]) && home::comp_range >= _comp_range)
    _comp_range = home::comp_range;
  else if((external_trigger[_IDLE] || internal_trigger[_IDLE]) && idle::comp_range >= _comp_range)
    _comp_range = idle::comp_range;
  else if((external_trigger[_HELLO] || internal_trigger[_HELLO]) && hello::comp_range >= _comp_range)
    _comp_range = hello::comp_range;
  else
    _comp_range = 180;
}
void Robot86ME::updateCommand()
{
  if(1) {_86ME_cmd[22] = true;}
  else {_86ME_cmd[22] = false;}
  if(1) {_86ME_cmd[23] = true;}
  else {_86ME_cmd[23] = false;}
  if(1) {_86ME_cmd[24] = true;}
  else {_86ME_cmd[24] = false;}
  if(1) {_86ME_cmd[25] = true;}
  else {_86ME_cmd[25] = false;}
  if(1) {_86ME_cmd[26] = true;}
  else {_86ME_cmd[26] = false;}
  if(1) {_86ME_cmd[27] = true;}
  else {_86ME_cmd[27] = false;}
}
void Robot86ME::updateTrigger()
{
  if(isBlocked(0)) goto L1;
L0:
  if(_86ME_cmd[22]) {_curr_motion[0] = _FORWARD;}
  else if(_86ME_cmd[23]) {_curr_motion[0] = _LEFT;}
  else if(_86ME_cmd[24]) {_curr_motion[0] = _RIGHT;}
  else if(_86ME_cmd[25]) {_curr_motion[0] = _HOME;}
  else if(_86ME_cmd[26]) {_curr_motion[0] = _IDLE;}
  else if(_86ME_cmd[27]) {_curr_motion[0] = _HELLO;}
  else _curr_motion[0] = _NONE;
  if(_last_motion[0] != _curr_motion[0] && _curr_motion[0] != _NONE)
  {
    closeTriggers(0);
    external_trigger[_curr_motion[0]] = true;
    forward::state = 0;
    left::state = 0;
    right::state = 0;
    home::state = 0;
    idle::state = 0;
    hello::state = 0;
    hello::aaa_3 = 0;
  }
  external_trigger[_curr_motion[0]] = true;
  _last_motion[0] = _curr_motion[0];
L1:
  if(isBlocked(1)) return;
  _curr_motion[1] = _NONE;
  if(_last_motion[1] != _curr_motion[1] && _curr_motion[1] != _NONE)
  {
    closeTriggers(1);
    external_trigger[_curr_motion[1]] = true;
  }
  external_trigger[_curr_motion[1]] = true;
  _last_motion[1] = _curr_motion[1];
}
void Robot86ME::forwardUpdate()
{
  switch(forward::state)
  {
  case forward::IDLE:
    if(external_trigger[_FORWARD] || internal_trigger[_FORWARD]) forward::state = forward::FRAME_0;
    else break;
  case forward::FRAME_0:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = forward_frm[0].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)300);
    forward::time = millis();
    forward::state = forward::WAIT_FRAME_0;
  case forward::WAIT_FRAME_0:
    if(millis() - forward::time >= 300)
      forward::state = forward::FRAME_1;
    break;
  case forward::FRAME_1:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = forward_frm[1].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    forward::time = millis();
    forward::state = forward::WAIT_FRAME_1;
  case forward::WAIT_FRAME_1:
    if(millis() - forward::time >= 150)
      forward::state = forward::FRAME_2;
    break;
  case forward::FRAME_2:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = forward_frm[2].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    forward::time = millis();
    forward::state = forward::WAIT_FRAME_2;
  case forward::WAIT_FRAME_2:
    if(millis() - forward::time >= 150)
      forward::state = forward::FRAME_3;
    break;
  case forward::FRAME_3:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = forward_frm[3].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)300);
    forward::time = millis();
    forward::state = forward::WAIT_FRAME_3;
  case forward::WAIT_FRAME_3:
    if(millis() - forward::time >= 300)
      forward::state = forward::FRAME_4;
    break;
  case forward::FRAME_4:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = forward_frm[4].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    forward::time = millis();
    forward::state = forward::WAIT_FRAME_4;
  case forward::WAIT_FRAME_4:
    if(millis() - forward::time >= 150)
      forward::state = forward::FRAME_5;
    break;
  case forward::FRAME_5:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = forward_frm[5].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    forward::time = millis();
    forward::state = forward::WAIT_FRAME_5;
  case forward::WAIT_FRAME_5:
    if(millis() - forward::time >= 150)
    {
      forward::state = forward::IDLE;
      internal_trigger[_FORWARD] = false;
      external_trigger[_FORWARD] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::forward(int times)
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
void Robot86ME::leftUpdate()
{
  switch(left::state)
  {
  case left::IDLE:
    if(external_trigger[_LEFT] || internal_trigger[_LEFT]) left::state = left::FRAME_0;
    else break;
  case left::FRAME_0:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = left_frm[0].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)300);
    left::time = millis();
    left::state = left::WAIT_FRAME_0;
  case left::WAIT_FRAME_0:
    if(millis() - left::time >= 300)
      left::state = left::FRAME_1;
    break;
  case left::FRAME_1:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = left_frm[1].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    left::time = millis();
    left::state = left::WAIT_FRAME_1;
  case left::WAIT_FRAME_1:
    if(millis() - left::time >= 150)
      left::state = left::FRAME_2;
    break;
  case left::FRAME_2:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = left_frm[2].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    left::time = millis();
    left::state = left::WAIT_FRAME_2;
  case left::WAIT_FRAME_2:
    if(millis() - left::time >= 150)
      left::state = left::FRAME_3;
    break;
  case left::FRAME_3:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = left_frm[3].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)300);
    left::time = millis();
    left::state = left::WAIT_FRAME_3;
  case left::WAIT_FRAME_3:
    if(millis() - left::time >= 300)
      left::state = left::FRAME_4;
    break;
  case left::FRAME_4:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = left_frm[4].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    left::time = millis();
    left::state = left::WAIT_FRAME_4;
  case left::WAIT_FRAME_4:
    if(millis() - left::time >= 150)
      left::state = left::FRAME_5;
    break;
  case left::FRAME_5:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = left_frm[5].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    left::time = millis();
    left::state = left::WAIT_FRAME_5;
  case left::WAIT_FRAME_5:
    if(millis() - left::time >= 150)
    {
      left::state = left::IDLE;
      internal_trigger[_LEFT] = false;
      external_trigger[_LEFT] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::left(int times)
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
void Robot86ME::rightUpdate()
{
  switch(right::state)
  {
  case right::IDLE:
    if(external_trigger[_RIGHT] || internal_trigger[_RIGHT]) right::state = right::FRAME_0;
    else break;
  case right::FRAME_0:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = right_frm[0].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)300);
    right::time = millis();
    right::state = right::WAIT_FRAME_0;
  case right::WAIT_FRAME_0:
    if(millis() - right::time >= 300)
      right::state = right::FRAME_1;
    break;
  case right::FRAME_1:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = right_frm[1].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    right::time = millis();
    right::state = right::WAIT_FRAME_1;
  case right::WAIT_FRAME_1:
    if(millis() - right::time >= 150)
      right::state = right::FRAME_2;
    break;
  case right::FRAME_2:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = right_frm[2].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    right::time = millis();
    right::state = right::WAIT_FRAME_2;
  case right::WAIT_FRAME_2:
    if(millis() - right::time >= 150)
      right::state = right::FRAME_3;
    break;
  case right::FRAME_3:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = right_frm[3].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)300);
    right::time = millis();
    right::state = right::WAIT_FRAME_3;
  case right::WAIT_FRAME_3:
    if(millis() - right::time >= 300)
      right::state = right::FRAME_4;
    break;
  case right::FRAME_4:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = right_frm[4].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    right::time = millis();
    right::state = right::WAIT_FRAME_4;
  case right::WAIT_FRAME_4:
    if(millis() - right::time >= 150)
      right::state = right::FRAME_5;
    break;
  case right::FRAME_5:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = right_frm[5].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)150);
    right::time = millis();
    right::state = right::WAIT_FRAME_5;
  case right::WAIT_FRAME_5:
    if(millis() - right::time >= 150)
    {
      right::state = right::IDLE;
      internal_trigger[_RIGHT] = false;
      external_trigger[_RIGHT] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::right(int times)
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
void Robot86ME::homeUpdate()
{
  switch(home::state)
  {
  case home::IDLE:
    if(external_trigger[_HOME] || internal_trigger[_HOME]) home::state = home::FRAME_0;
    else break;
  case home::FRAME_0:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = home_frm[0].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)500);
    home::time = millis();
    home::state = home::WAIT_FRAME_0;
  case home::WAIT_FRAME_0:
    if(millis() - home::time >= 500)
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
void Robot86ME::idleUpdate()
{
  switch(idle::state)
  {
  case idle::IDLE:
    if(external_trigger[_IDLE] || internal_trigger[_IDLE]) idle::state = idle::FRAME_0;
    else break;
  case idle::FRAME_0:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = idle_frm[0].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)600);
    idle::time = millis();
    idle::state = idle::WAIT_FRAME_0;
  case idle::WAIT_FRAME_0:
    if(millis() - idle::time >= 600)
      idle::state = idle::FRAME_1;
    break;
  case idle::FRAME_1:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = idle_frm[1].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)600);
    idle::time = millis();
    idle::state = idle::WAIT_FRAME_1;
  case idle::WAIT_FRAME_1:
    if(millis() - idle::time >= 600)
    {
      idle::state = idle::IDLE;
      internal_trigger[_IDLE] = false;
      external_trigger[_IDLE] = false;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::idle(int times)
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
void Robot86ME::helloUpdate()
{
  switch(hello::state)
  {
  case hello::IDLE:
    if(external_trigger[_HELLO] || internal_trigger[_HELLO]) hello::state = hello::FLAG_0;
    else break;
  case hello::FLAG_0:
    flag_hello_aaa_0:
  case hello::FRAME_1:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = hello_frm[0].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)350);
    hello::time = millis();
    hello::state = hello::WAIT_FRAME_1;
  case hello::WAIT_FRAME_1:
    if(millis() - hello::time >= 350)
      hello::state = hello::FRAME_2;
    break;
  case hello::FRAME_2:
    for(int i = 12; i-- > 0; )
      _86ME_RUN.positions[i] = hello_frm[1].positions[i] & (~servo_mask[i]);
    _86ME_RUN.playPositions((unsigned long)350);
    hello::time = millis();
    hello::state = hello::WAIT_FRAME_2;
  case hello::WAIT_FRAME_2:
    if(millis() - hello::time >= 350)
      hello::state = hello::GOTO_3;
    break;
  case hello::GOTO_3:
    if(hello::aaa_3++ < 2) goto flag_hello_aaa_0;
    else
    {
      hello::aaa_3 = 0;
      internal_trigger[_HELLO] = false;
      external_trigger[_HELLO] = false;
      hello::state = hello::IDLE;
    }
    break;
  default:
    break;
  }
}
void Robot86ME::hello(int times)
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
  begin(initIMU);
}
void Robot86ME::begin(bool initIMU)
{
  srand(time(NULL));
  used_servos[0].attach(2, 500, 2500);
  used_servos[1].attach(3, 500, 2500);
  used_servos[2].attach(4, 500, 2500);
  used_servos[3].attach(5, 500, 2500);
  used_servos[4].attach(6, 500, 2500);
  used_servos[5].attach(7, 500, 2500);
  used_servos[6].attach(8, 500, 2500);
  used_servos[7].attach(9, 500, 2500);
  used_servos[8].attach(10, 500, 2500);
  used_servos[9].attach(11, 500, 2500);
  used_servos[10].attach(12, 500, 2500);
  used_servos[11].attach(13, 500, 2500);


  forward_frm[0].positions[0] = 1700;
  forward_frm[0].positions[1] = 1500;
  forward_frm[0].positions[2] = 1300;
  forward_frm[0].positions[3] = 1750;
  forward_frm[0].positions[4] = 1700;
  forward_frm[0].positions[5] = 1500;
  forward_frm[0].positions[6] = 1700;
  forward_frm[0].positions[7] = 1750;
  forward_frm[0].positions[8] = 1300;
  forward_frm[0].positions[9] = 1500;
  forward_frm[0].positions[10] = 1700;
  forward_frm[0].positions[11] = 1750;

  forward_frm[1].positions[0] = 1700;
  forward_frm[1].positions[1] = 1500;
  forward_frm[1].positions[2] = 1300;
  forward_frm[1].positions[3] = 1500;
  forward_frm[1].positions[4] = 1700;
  forward_frm[1].positions[5] = 1500;
  forward_frm[1].positions[6] = 1700;
  forward_frm[1].positions[7] = 1500;
  forward_frm[1].positions[8] = 1300;
  forward_frm[1].positions[9] = 1500;
  forward_frm[1].positions[10] = 1700;
  forward_frm[1].positions[11] = 1500;

  forward_frm[2].positions[0] = 1700;
  forward_frm[2].positions[1] = 1750;
  forward_frm[2].positions[2] = 1300;
  forward_frm[2].positions[3] = 1500;
  forward_frm[2].positions[4] = 1700;
  forward_frm[2].positions[5] = 1750;
  forward_frm[2].positions[6] = 1700;
  forward_frm[2].positions[7] = 1500;
  forward_frm[2].positions[8] = 1300;
  forward_frm[2].positions[9] = 1750;
  forward_frm[2].positions[10] = 1700;
  forward_frm[2].positions[11] = 1500;

  forward_frm[3].positions[0] = 1300;
  forward_frm[3].positions[1] = 1750;
  forward_frm[3].positions[2] = 1700;
  forward_frm[3].positions[3] = 1500;
  forward_frm[3].positions[4] = 1300;
  forward_frm[3].positions[5] = 1750;
  forward_frm[3].positions[6] = 1300;
  forward_frm[3].positions[7] = 1500;
  forward_frm[3].positions[8] = 1700;
  forward_frm[3].positions[9] = 1750;
  forward_frm[3].positions[10] = 1300;
  forward_frm[3].positions[11] = 1500;

  forward_frm[4].positions[0] = 1300;
  forward_frm[4].positions[1] = 1500;
  forward_frm[4].positions[2] = 1700;
  forward_frm[4].positions[3] = 1500;
  forward_frm[4].positions[4] = 1300;
  forward_frm[4].positions[5] = 1500;
  forward_frm[4].positions[6] = 1300;
  forward_frm[4].positions[7] = 1500;
  forward_frm[4].positions[8] = 1700;
  forward_frm[4].positions[9] = 1500;
  forward_frm[4].positions[10] = 1300;
  forward_frm[4].positions[11] = 1500;

  forward_frm[5].positions[0] = 1300;
  forward_frm[5].positions[1] = 1500;
  forward_frm[5].positions[2] = 1700;
  forward_frm[5].positions[3] = 1750;
  forward_frm[5].positions[4] = 1300;
  forward_frm[5].positions[5] = 1500;
  forward_frm[5].positions[6] = 1300;
  forward_frm[5].positions[7] = 1750;
  forward_frm[5].positions[8] = 1700;
  forward_frm[5].positions[9] = 1500;
  forward_frm[5].positions[10] = 1300;
  forward_frm[5].positions[11] = 1750;

  left_frm[0].positions[0] = 1700;
  left_frm[0].positions[1] = 1750;
  left_frm[0].positions[2] = 1300;
  left_frm[0].positions[3] = 1500;
  left_frm[0].positions[4] = 1700;
  left_frm[0].positions[5] = 1750;
  left_frm[0].positions[6] = 1300;
  left_frm[0].positions[7] = 1500;
  left_frm[0].positions[8] = 1700;
  left_frm[0].positions[9] = 1750;
  left_frm[0].positions[10] = 1300;
  left_frm[0].positions[11] = 1500;

  left_frm[1].positions[0] = 1700;
  left_frm[1].positions[1] = 1500;
  left_frm[1].positions[2] = 1300;
  left_frm[1].positions[3] = 1500;
  left_frm[1].positions[4] = 1700;
  left_frm[1].positions[5] = 1500;
  left_frm[1].positions[6] = 1300;
  left_frm[1].positions[7] = 1500;
  left_frm[1].positions[8] = 1700;
  left_frm[1].positions[9] = 1500;
  left_frm[1].positions[10] = 1300;
  left_frm[1].positions[11] = 1500;

  left_frm[2].positions[0] = 1700;
  left_frm[2].positions[1] = 1500;
  left_frm[2].positions[2] = 1300;
  left_frm[2].positions[3] = 1750;
  left_frm[2].positions[4] = 1700;
  left_frm[2].positions[5] = 1500;
  left_frm[2].positions[6] = 1300;
  left_frm[2].positions[7] = 1750;
  left_frm[2].positions[8] = 1700;
  left_frm[2].positions[9] = 1500;
  left_frm[2].positions[10] = 1300;
  left_frm[2].positions[11] = 1750;

  left_frm[3].positions[0] = 1300;
  left_frm[3].positions[1] = 1500;
  left_frm[3].positions[2] = 1700;
  left_frm[3].positions[3] = 1750;
  left_frm[3].positions[4] = 1300;
  left_frm[3].positions[5] = 1500;
  left_frm[3].positions[6] = 1700;
  left_frm[3].positions[7] = 1750;
  left_frm[3].positions[8] = 1300;
  left_frm[3].positions[9] = 1500;
  left_frm[3].positions[10] = 1700;
  left_frm[3].positions[11] = 1750;

  left_frm[4].positions[0] = 1300;
  left_frm[4].positions[1] = 1500;
  left_frm[4].positions[2] = 1700;
  left_frm[4].positions[3] = 1500;
  left_frm[4].positions[4] = 1300;
  left_frm[4].positions[5] = 1500;
  left_frm[4].positions[6] = 1700;
  left_frm[4].positions[7] = 1500;
  left_frm[4].positions[8] = 1300;
  left_frm[4].positions[9] = 1500;
  left_frm[4].positions[10] = 1700;
  left_frm[4].positions[11] = 1500;

  left_frm[5].positions[0] = 1300;
  left_frm[5].positions[1] = 1750;
  left_frm[5].positions[2] = 1700;
  left_frm[5].positions[3] = 1500;
  left_frm[5].positions[4] = 1300;
  left_frm[5].positions[5] = 1750;
  left_frm[5].positions[6] = 1700;
  left_frm[5].positions[7] = 1500;
  left_frm[5].positions[8] = 1300;
  left_frm[5].positions[9] = 1750;
  left_frm[5].positions[10] = 1700;
  left_frm[5].positions[11] = 1500;

  right_frm[0].positions[0] = 1300;
  right_frm[0].positions[1] = 1750;
  right_frm[0].positions[2] = 1700;
  right_frm[0].positions[3] = 1500;
  right_frm[0].positions[4] = 1300;
  right_frm[0].positions[5] = 1750;
  right_frm[0].positions[6] = 1700;
  right_frm[0].positions[7] = 1500;
  right_frm[0].positions[8] = 1300;
  right_frm[0].positions[9] = 1750;
  right_frm[0].positions[10] = 1700;
  right_frm[0].positions[11] = 1500;

  right_frm[1].positions[0] = 1300;
  right_frm[1].positions[1] = 1500;
  right_frm[1].positions[2] = 1700;
  right_frm[1].positions[3] = 1500;
  right_frm[1].positions[4] = 1300;
  right_frm[1].positions[5] = 1500;
  right_frm[1].positions[6] = 1700;
  right_frm[1].positions[7] = 1500;
  right_frm[1].positions[8] = 1300;
  right_frm[1].positions[9] = 1500;
  right_frm[1].positions[10] = 1700;
  right_frm[1].positions[11] = 1500;

  right_frm[2].positions[0] = 1300;
  right_frm[2].positions[1] = 1500;
  right_frm[2].positions[2] = 1700;
  right_frm[2].positions[3] = 1750;
  right_frm[2].positions[4] = 1300;
  right_frm[2].positions[5] = 1500;
  right_frm[2].positions[6] = 1700;
  right_frm[2].positions[7] = 1750;
  right_frm[2].positions[8] = 1300;
  right_frm[2].positions[9] = 1500;
  right_frm[2].positions[10] = 1700;
  right_frm[2].positions[11] = 1750;

  right_frm[3].positions[0] = 1700;
  right_frm[3].positions[1] = 1500;
  right_frm[3].positions[2] = 1300;
  right_frm[3].positions[3] = 1750;
  right_frm[3].positions[4] = 1700;
  right_frm[3].positions[5] = 1500;
  right_frm[3].positions[6] = 1300;
  right_frm[3].positions[7] = 1750;
  right_frm[3].positions[8] = 1700;
  right_frm[3].positions[9] = 1500;
  right_frm[3].positions[10] = 1300;
  right_frm[3].positions[11] = 1750;

  right_frm[4].positions[0] = 1700;
  right_frm[4].positions[1] = 1500;
  right_frm[4].positions[2] = 1300;
  right_frm[4].positions[3] = 1500;
  right_frm[4].positions[4] = 1700;
  right_frm[4].positions[5] = 1500;
  right_frm[4].positions[6] = 1300;
  right_frm[4].positions[7] = 1500;
  right_frm[4].positions[8] = 1700;
  right_frm[4].positions[9] = 1500;
  right_frm[4].positions[10] = 1300;
  right_frm[4].positions[11] = 1500;

  right_frm[5].positions[0] = 1700;
  right_frm[5].positions[1] = 1750;
  right_frm[5].positions[2] = 1300;
  right_frm[5].positions[3] = 1500;
  right_frm[5].positions[4] = 1700;
  right_frm[5].positions[5] = 1750;
  right_frm[5].positions[6] = 1300;
  right_frm[5].positions[7] = 1500;
  right_frm[5].positions[8] = 1700;
  right_frm[5].positions[9] = 1750;
  right_frm[5].positions[10] = 1300;
  right_frm[5].positions[11] = 1500;

  home_frm[0].positions[0] = 1500;
  home_frm[0].positions[1] = 1500;
  home_frm[0].positions[2] = 1500;
  home_frm[0].positions[3] = 1500;
  home_frm[0].positions[4] = 1500;
  home_frm[0].positions[5] = 1500;
  home_frm[0].positions[6] = 1500;
  home_frm[0].positions[7] = 1500;
  home_frm[0].positions[8] = 1500;
  home_frm[0].positions[9] = 1500;
  home_frm[0].positions[10] = 1500;
  home_frm[0].positions[11] = 1500;

  idle_frm[0].positions[0] = 1500;
  idle_frm[0].positions[1] = 1700;
  idle_frm[0].positions[2] = 1500;
  idle_frm[0].positions[3] = 1700;
  idle_frm[0].positions[4] = 1500;
  idle_frm[0].positions[5] = 1700;
  idle_frm[0].positions[6] = 1500;
  idle_frm[0].positions[7] = 1700;
  idle_frm[0].positions[8] = 1500;
  idle_frm[0].positions[9] = 1700;
  idle_frm[0].positions[10] = 1500;
  idle_frm[0].positions[11] = 1700;

  idle_frm[1].positions[0] = 1500;
  idle_frm[1].positions[1] = 1300;
  idle_frm[1].positions[2] = 1500;
  idle_frm[1].positions[3] = 1300;
  idle_frm[1].positions[4] = 1500;
  idle_frm[1].positions[5] = 1300;
  idle_frm[1].positions[6] = 1500;
  idle_frm[1].positions[7] = 1300;
  idle_frm[1].positions[8] = 1500;
  idle_frm[1].positions[9] = 1300;
  idle_frm[1].positions[10] = 1500;
  idle_frm[1].positions[11] = 1300;

  hello_frm[0].positions[0] = 1524;
  hello_frm[0].positions[1] = 1881;
  hello_frm[0].positions[2] = 1500;
  hello_frm[0].positions[3] = 1500;
  hello_frm[0].positions[4] = 1500;
  hello_frm[0].positions[5] = 1500;
  hello_frm[0].positions[6] = 1500;
  hello_frm[0].positions[7] = 1500;
  hello_frm[0].positions[8] = 1500;
  hello_frm[0].positions[9] = 1500;
  hello_frm[0].positions[10] = 1500;
  hello_frm[0].positions[11] = 1500;

  hello_frm[1].positions[0] = 1288;
  hello_frm[1].positions[1] = 1881;
  hello_frm[1].positions[2] = 1500;
  hello_frm[1].positions[3] = 1500;
  hello_frm[1].positions[4] = 1500;
  hello_frm[1].positions[5] = 1500;
  hello_frm[1].positions[6] = 1500;
  hello_frm[1].positions[7] = 1500;
  hello_frm[1].positions[8] = 1500;
  hello_frm[1].positions[9] = 1500;
  hello_frm[1].positions[10] = 1500;
  hello_frm[1].positions[11] = 1500;

  _86ME_HOME.positions[0] = 1500;
  _86ME_HOME.positions[1] = 1500;
  _86ME_HOME.positions[2] = 1500;
  _86ME_HOME.positions[3] = 1500;
  _86ME_HOME.positions[4] = 1500;
  _86ME_HOME.positions[5] = 1500;
  _86ME_HOME.positions[6] = 1500;
  _86ME_HOME.positions[7] = 1500;
  _86ME_HOME.positions[8] = 1500;
  _86ME_HOME.positions[9] = 1500;
  _86ME_HOME.positions[10] = 1500;
  _86ME_HOME.positions[11] = 1500;


  offsets.setOffsets();

  _86ME_HOME.playPositions((unsigned long)0);
}

void Robot86ME::update()
{
  updateCommand();
  updateTrigger();
  forwardUpdate();
  leftUpdate();
  rightUpdate();
  homeUpdate();
  idleUpdate();
  helloUpdate();
}
void Robot86ME::update_S()
{
  updateTrigger();
  forwardUpdate();
  leftUpdate();
  rightUpdate();
  homeUpdate();
  idleUpdate();
  helloUpdate();
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
      if (motion_id == 0) robot.forward(motion_times);
      else if (motion_id == 1) robot.left(motion_times);
      else if (motion_id == 2) robot.right(motion_times);
      else if (motion_id == 3) robot.home(motion_times);
      else if (motion_id == 4) robot.idle(motion_times);
      else if (motion_id == 5) robot.hello(motion_times);
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

  
  for (byte i = 0; i < 33; i++) {
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
