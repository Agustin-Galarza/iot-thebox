#include <RIC3DMODEM.h>
#include <StateMachineLib.h>
#include <RIC3D.h>
#include <arduino-timer.h>
#include <floatToString.h>

/*******************
 * Configurations
 ********************/
#define MAX_DELIVERY_MILLIS 5000
#define PLOTS_ENABLED false
#define COMMS_ENABLED false
#define SEND_REPORT_MILLIS 60000
/*******************
 * IO Configuration
 ********************/
// Inputs
#define DOOR_INPUT DI1
#define BUY_INPUT DI0
#define LOAD_INPUT AI0

// Outputs
#define DOOR_OUTPUT DO0
#define LOCK_OUTPUT DO1
// Output LEDs
#define STATUS_LED LED1
#define OUTPUT_LED1 LED2
#define OUTPUT_LED2 LED3
#define ERROR_LED LED4
/*******************
 * Miscelaneous Configuration
 ********************/
#define TIMER_CONCURRENT_TASKS 6
#define PORT_CORRECTION_FACTOR 40
#define SerialAT Serial3 // 4G modem comms

/*******************
 * Types
 ********************/

enum State
{
	Available = 0,
	Reserved = 1,
	BuyerAuthenticated = 2,
	DealerAuthenticated = 3,
	ProductInside = 4,
	ProductSecured = 5,
	Illegal = 6
};
#define STATE_ENUM_COUNT 7

typedef struct
{
	Timer<>::Task task_ref;
	bool (*fn)();
	unsigned long millis;
} RepeateableTask;

/*******************
 * Instances
 ********************/

RIC3D device;
RIC3DMODEM g_modem;
auto timer = timer_create_default();
StateMachine state_machine(STATE_ENUM_COUNT, 9);

/*********************************
 * Filters
 **********************************/
class Filter
{
public:
  virtual float apply(float value) = 0;
};

class SMA : public Filter
{
public:
  SMA(uint8_t period)
      : period(period)
  {
    sma_values = new float[period];
    for (int i = 0; i < period; i++)
    {
      sma_values[i] = 0;
    }
  }
  SMA()
      : SMA(5) {}
  ~SMA()
  {
    delete[] sma_values;
  }
  float apply(float new_value)
  {
    sma_values[sma_index++] = new_value;
    sma_index %= period;
    float sum = 0;
    for (int i = 0; i < period; i++)
    {
      sum += sma_values[i];
    }

    return sum / period;
  }

  float alt_update(float new_value)
  {
    sma_sum -= sma_values[sma_index];
    sma_sum += new_value;
    sma_values[sma_index] = new_value;
    sma_index = ++sma_index % period;
    return (sma_sum) / period;
  }

private:
  uint8_t period;
  float *sma_values;
  uint8_t sma_index = 0;
  float sma_sum = 0;
};

class EMA : public Filter
{
public:
  EMA(float _alpha)
  {
    alpha = _alpha;
  }
  EMA()
      : EMA(0.1) {}
  float apply(float new_value)
  {
    ema = alpha * new_value + (1 - alpha) * ema;
    return ema;
  }

private:
  float alpha;
  float ema = 0.0;
};

/**************************************
 * COMMS
 *************************************/

// Modem configuration 
const char apn[] = "grupotesacom.claro.com.ar"; // mobile network APN
const char apn_user[] = "";                     // GPRS User (if needed)
const char apn_password[] = "";                 // GPRS Password (if needed)

const char mqtt_user[] = "vBSeBu64ji8qaHCZMLZo"; // Here goes the tdata device TOKEN
const char mqtt_host[] = "10.25.1.152";
const char mqtt_port[] = "4094";
const char *mqtt_password = NULL;

// Module baud rate
uint32_t rate = 115200;

// Select SIM Card (0 = right, 1 = left)
bool sim_selected = 1;

void modem_setup();
int modem_init();
void comm_init();

/**************************************
 * SENSORS
 **************************************/

class AnalogSensor
{
private:
  String name;
  uint8_t port;
  float correction;
  float *mem;
  uint8_t mem_size;
  uint8_t mem_index = 0;
  Filter *filter;
  float last_raw_value = 0;
  float last_value = 0;
  float last_computed_value = 0;
  float min_value = 0;
  float max_value = 0;
  bool use_filter_for_report;
  float unit_zero;
  float unit_factor;
  bool is_dead = false;
  uint16_t read_interval_millis;
  uint16_t report_interval_millis;

  public:
  AnalogSensor(
      String name,
      uint8_t port,
      float unit_zero,
      float unit_factor,
      uint16_t read_interval_millis,
      uint16_t report_interval_millis,
      Filter *filter,
      float correction = 0,
      bool use_filter_for_report = true)
      : name(name),
        port(port),
        unit_zero(unit_zero),
        unit_factor(unit_factor),
        read_interval_millis(read_interval_millis),
        report_interval_millis(report_interval_millis),
        filter(filter),
        correction(correction),
        use_filter_for_report(use_filter_for_report)
  {
    this->mem_size = (this->report_interval_millis / this->read_interval_millis) + 1;
    this->mem = new float[mem_size];
  }

  ~AnalogSensor()
  {
    delete[] mem;
  }

  float read()
  {
    float sensor_value = analogRead(port) / PORT_CORRECTION_FACTOR + correction;

    if (sensor_value < 4 || sensor_value > 20)
    {
      if (!is_dead)
      {
        is_dead = true;
        return -1;
      }
    }
    if (filter != nullptr)
    {
      last_value = filter->apply(sensor_value);
      if (use_filter_for_report)
      {
        sensor_value = last_value;
      }
    }
    if (PLOTS_ENABLED)
    {
      Serial.print(sensor_value);
      Serial.print(", ");
    }

    if (mem_index == mem_size)
    {
      Serial.print("Memory overload on sensor " + name);
      return -1;
    }
    last_computed_value = sensor_value * unit_factor + unit_zero;
    mem[mem_index++] = last_computed_value;

    return last_computed_value;
  }

  float get_last_raw_value()
  {
    return last_raw_value;
  }

  float get_last_value()
  {
    return last_value;
  }

  float get_last_computed_value()
  {
    return last_computed_value;
  }

  String get_name()
  {
    return name;
  }

  bool get_is_dead()
  {
    return is_dead;
  }

  void send_report()
  {
    float sum = 0;
    for (int i = 0; i < mem_size; i++)
    {
      sum += mem[i];
    }
    float avg = sum / mem_size;
    mem_index = 0;

    // COMMUNICATION
    bool comm_failure = false;
    char buff[10] = {0};

    floatToString(min_value, buff, sizeof(buff), 3);
    if (COMMS_ENABLED)
    {
      comm_failure = comm_failure || !g_modem.publishData("min", buff);
    }
    else
    {
      Serial.print(name + " report -> ");
      Serial.print("Min: ");
      Serial.print(buff);
      Serial.print(" || ");
    }

    floatToString(max_value, buff, sizeof(buff), 3);
    if (COMMS_ENABLED)
    {
      comm_failure = comm_failure || !g_modem.publishData("max", buff);
    }
    else
    {
      Serial.print("Max: ");
      Serial.print(buff);
      Serial.print(" || ");
    }

    floatToString(avg, buff, sizeof(buff), 3);
    if (COMMS_ENABLED)
    {
      comm_failure = comm_failure || !g_modem.publishData("avg", buff);
    }
    else
    {
      Serial.print("Avg: ");
      Serial.println(buff);
    }

    if (comm_failure)
    {
      Serial.print("ERROR: failed to send report.");
    }
  }
};

SMA load_cell_filter(20);
AnalogSensor load_cell(
    "Load Cell",
    LOAD_INPUT,
    4 / 16,
    50 / 16,
    200,
    SEND_REPORT_MILLIS,
    &load_cell_filter,
    -0.3,
    true);

/*******************
 * Function definitions
 ********************/

void set_up_state_machine();
bool flash_error_led();
// Timed tasks
bool read_load();
bool read_door_status();
bool check_buyers();

// Utils
uint8_t pullupRead(uint8_t pin)
{
	return !digitalRead(pin);
}

/*******************
 * Box State
 ********************/

struct
{
	bool door_open;
	bool lock_open;
	bool reserved;
	bool dealer_authenticated;
	bool buyer_authenticated;
	float load;

	struct
	{
		RepeateableTask flash_error = {nullptr, flash_error_led, 200};
	} repeatable_tasks;
} state;

void start_task(RepeateableTask task)
{
	if (task.task_ref == nullptr)
	{
		task.task_ref = timer.every(task.millis, task.fn);
	}
}

void stop_task(RepeateableTask task)
{
	if (task.task_ref != nullptr)
	{
		timer.cancel(task.task_ref);
		task.task_ref = nullptr;
	}
}

/*******************
 * Setup
 ********************/

void setup()
{
	// Inputs
	pinMode(DOOR_INPUT, INPUT_PULLUP);
	pinMode(BUY_INPUT, INPUT_PULLUP);
	pinMode(LOAD_INPUT, INPUT);
  pinMode(AI1, INPUT);
	// Outputs
	pinMode(DOOR_OUTPUT, OUTPUT);
	pinMode(LOCK_OUTPUT, OUTPUT);
	// Output LEDs
	pinMode(STATUS_LED, OUTPUT);
	pinMode(OUTPUT_LED1, OUTPUT);
	pinMode(OUTPUT_LED2, OUTPUT);
	pinMode(ERROR_LED, OUTPUT);

	Serial.begin(115200);

	timer.every(200, read_door_status);
	timer.every(200, read_load);
	timer.every(200, check_buyers);
  timer.every(1000, []() {
    state.load = load_cell.read();
    return true;
  });
  timer.every(SEND_REPORT_MILLIS, []() {load_cell.send_report(); return true;});

  analogReference(INTERNAL2V56);

	set_up_state_machine();

	state_machine.SetState(Available, true, false);

	digitalWrite(STATUS_LED, HIGH);

  // Default state
  state.door_open = true;
  state.lock_open = true;

	Serial.println("Device ready V2");
}

unsigned long long count = 0;

void loop()
{
  timer.tick();
	state_machine.Update();

  digitalWrite(DOOR_OUTPUT, state.door_open ? HIGH : LOW);
  digitalWrite(LOCK_OUTPUT, state.lock_open ? HIGH : LOW);

	delay(100);
}

/*******************
 * Function implementations
 ********************/

void set_up_state_machine()
{
  Serial.println("Setting up state machine...");
	/**************************
	 * Transitions
	 ***************************/
	// Available
	state_machine.AddTransition(Available, Reserved, []()
								{ return state.reserved; });
	state_machine.AddTransition(Available, Illegal, []()
								{ return state.load != 0; });
	// Reserved
	state_machine.AddTransition(Reserved, DealerAuthenticated, []()
								{ return state.dealer_authenticated; });
	state_machine.AddTransition(Reserved, Illegal, []()
								{ return state.load != 0; });
	// BuyerAuthenticated
	state_machine.AddTransition(DealerAuthenticated, ProductInside, []()
								{ return state.load != 0; });
	// ProductInside
	state_machine.AddTransition(ProductInside, DealerAuthenticated, []()
								{ return state.load == 0; });
	state_machine.AddTransition(ProductInside, ProductSecured, []() // TODO: store the load and check also for weight changes
								{ return !state.door_open && state.load != 0 && !state.lock_open; });
	// ProductSecured
	state_machine.AddTransition(ProductSecured, BuyerAuthenticated, []()
								{ return state.buyer_authenticated; });
	// BuyerAuthenticated
	state_machine.AddTransition(BuyerAuthenticated, Available, []()
								{ return state.door_open && state.load == 0; });
	// Illegal

	/**************************
	 * State Events
	 ***************************/
	state_machine.SetOnEntering(Available, []()
								{ return; });
	state_machine.SetOnEntering(Reserved, []()
								{ state.dealer_authenticated = true; });
	state_machine.SetOnEntering(DealerAuthenticated, []()
								{ 
		// Dealer has a limited time to open the door and secure the product
		timer.in(MAX_DELIVERY_MILLIS, []() {
			if (state_machine.GetState() == DealerAuthenticated)
			{
				state_machine.SetState(Illegal, true, false);
			}
		}); });
	state_machine.SetOnEntering(ProductInside, []()
								{ state.lock_open = false; });
	state_machine.SetOnEntering(ProductSecured, []()
								{ state.buyer_authenticated = true; });
	state_machine.SetOnEntering(BuyerAuthenticated, []()
								{ state.lock_open = true; });
	state_machine.SetOnEntering(Illegal, []()
								{ start_task(state.repeatable_tasks.flash_error); });

	state_machine.SetOnLeaving(Reserved, []()
							   { state.dealer_authenticated = false; }); // Reset state
	state_machine.SetOnLeaving(ProductSecured, []()
							   { return state.buyer_authenticated = false; }); // Reset state
	state_machine.SetOnLeaving(Illegal, []()
							   { stop_task(state.repeatable_tasks.flash_error); });
  Serial.println("State machine set.");
}

bool read_load()
{
	state.load = analogRead(LOAD_INPUT);
	return true;
}

bool read_door_status()
{
	if(pullupRead(DOOR_INPUT) == LOW) {
    return true;
  }

	// Cannot open door if lock is closed
	if (!state.door_open && !state.lock_open)
	{
		return true;
	}

  state.door_open = !state.door_open;
	return true;
}

bool check_buyers()
{
	if (!state.reserved && pullupRead(BUY_INPUT) == HIGH)
	{
		state.reserved = true;
	}
}

bool flash_error_led()
{
	digitalWrite(ERROR_LED, !digitalRead(ERROR_LED));
	return true;
}

void modem_setup()
{

  Serial.println(F("modem_setup()"));
  pinMode(SIM_SELECT, OUTPUT);
  SerialAT.begin(rate);
  g_modem.begin(&SerialAT, &Serial, true, true); // depuración + at dump

  digitalWrite(SIM_SELECT, sim_selected);

  g_modem.turnOff();

  g_modem.turnOn();
}

int modem_init()
{
  int result;

  Serial.println(F("modem_init()"));
  Serial.print("connecting to port ");
  Serial.println(mqtt_port);
  g_modem.test();

  if (result = g_modem.test())
    return result;

  Serial.println(F("ok - test passed"));

  g_modem.deactivatePDPContext();

  if (result = g_modem.createPDPContext(apn, apn_user, apn_password))
    return result;

  Serial.println(F("ok - PDP context created"));

  if (result = g_modem.activatePDPContext())
    return result;

  Serial.println(F("ok - PDP context activated"));

  if (result = g_modem.connectMQTTClient(mqtt_host, mqtt_port, mqtt_user, mqtt_password))
    return result;

  Serial.println(F("ok - MQTT client connected"));

  return result;
}

void comm_init()
{
  modem_setup();
  int result;
  while (result = modem_init())
  {
    Serial.print(F("FAILURE! -> "));
    Serial.println(result);
    delay(10000);
  }
}

