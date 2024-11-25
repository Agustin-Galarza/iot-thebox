#include <StateMachineLib.h>
#include <RIC3D.h>
#include <arduino-timer.h>

/*******************
 * Configurations
 ********************/
#define MAX_DELIVERY_MILLIS 5000
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

RIC3D device;
auto timer = timer_create_default();
StateMachine state_machine(STATE_ENUM_COUNT, 9);

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

typedef struct {
  unsigned long millis;
  bool (*routine)();
} TimedTask;

TimedTask timed_tasks[] = {
  {200, &read_door_status},
  {200, &read_load},
  {200, &check_buyers},
  {1000, []() {Serial.println("Timer"); return true;}}
};

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

  for(int i = 0; i < sizeof(timed_tasks) / sizeof(TimedTask); i++) {
    timer.every(timed_tasks[i].millis, timed_tasks[i].routine);
  }
	// timer.every(200, read_door_status);
	// timer.every(200, read_load);
	// timer.every(200, check_buyers);
  // timer.every(1000, []() { 
  //   Serial.println("Timer");
  //   return true;
  //   });

  noInterrupts();
  analogReference(INTERNAL2V56);
  interrupts();

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
  // count++;
  // if(count % 10 == 0) Serial.println("Loop");
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
  Serial.print("Reading door status: ");
  Serial.println(pullupRead(DOOR_INPUT) == HIGH ? "High" : "Low");
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
