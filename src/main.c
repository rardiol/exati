#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>

#define DEBUG

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

// General GPIO mutex
K_MUTEX_DEFINE(gpio_mutex);

// Is there a pedestrian button press request pending?
int pedestrian_request;
K_MUTEX_DEFINE(pedestrian_request_mutex);

#define NUM_LEDS 6
enum leds {
	RED_NS_ENUM = 0,
	YELLOW_NS_ENUM,
	GREEN_NS_ENUM,
	RED_WE_ENUM,
	YELLOW_WE_ENUM,
	GREEN_WE_ENUM
};

// Get the nodes from the device tree
#define LED_RED_NS_NODE DT_NODELABEL(red_ledt_1)
#define LED_YELLOW_NS_NODE DT_NODELABEL(yellow_ledt_1)
#define LED_GREEN_NS_NODE DT_NODELABEL(green_ledt_1)
#define LED_RED_WE_NODE DT_NODELABEL(red_ledt_2)
#define LED_YELLOW_WE_NODE DT_NODELABEL(yellow_ledt_2)
#define LED_GREEN_WE_NODE DT_NODELABEL(green_ledt_2)

#define SW0_NODE        DT_ALIAS(sw0)

// Check the nodes from the device tree
#if ((!DT_NODE_HAS_STATUS(LED_RED_NS_NODE, okay)) || \
		(!DT_NODE_HAS_STATUS(LED_YELLOW_NS_NODE, okay)) || \
		(!DT_NODE_HAS_STATUS(LED_GREEN_NS_NODE, okay)) || \
		(!DT_NODE_HAS_STATUS(LED_RED_WE_NODE, okay)) || \
		(!DT_NODE_HAS_STATUS(LED_YELLOW_WE_NODE, okay)) || \
		(!DT_NODE_HAS_STATUS(LED_GREEN_WE_NODE, okay)))
#error "Unsupported board: leds devicetree not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

// Get the specs from the nodes
struct gpio_dt_spec static const led_specs[NUM_LEDS] = {
	GPIO_DT_SPEC_GET_OR(LED_RED_NS_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(LED_YELLOW_NS_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(LED_GREEN_NS_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(LED_RED_WE_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(LED_YELLOW_WE_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(LED_GREEN_WE_NODE, gpios, {0})
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

// Traffic Light/System states
#define NUM_STATES 10
enum state {
       ALL_OFF = 0,
       ALL_ON,

       // NS_WE = NORTH.SOUTH.ROAD_WEST.EAST.ROAD
       RED_GREEN,
       RED_YELLOW,
       RED_EXTRARED,
       GREEN_RED,
       YELLOW_RED,
       EXTRARED_RED,

       // Pedestrian crossing states
       RED_RED_NS,
       RED_RED_WE,

       // Keep NUM_STATES updated if adding new states
};

struct state_config {
	// Time to wait until next state. Only used if forever is false
	int32_t time;
	// Output value of GPIOs
	int8_t output;
	// Boolean, do not advance to next state due to time?
	// https://docs.zephyrproject.org/latest/reference/kernel/timing/clocks.html#subsystems-that-keep-millisecond-apis
	int8_t forever;
	// respond to pedestrian crossing request
	// 0 = ignore
	// 1 = respond to it
	// 2 = clear the request at the end of the state
	int8_t pedestrian;
	// Next state after time
	enum state next_state;
	// Next state after time if a pedestrian crossing request is active
	enum state next_state_pedestrian;
};

// miliseconds
#define EXTRARED_TIME 500
#define YELLOW_TIME 2000
#define GREEN_NS_TIME 4000
#define GREEN_WE_TIME 5000
#define PEDESTRIAN_TIME 4000

#define RED_NS_BITFLAG (1<<RED_NS_ENUM)
#define YELLOW_NS_BITFLAG (1<<YELLOW_NS_ENUM)
#define GREEN_NS_BITFLAG (1<<GREEN_NS_ENUM)
#define RED_WE_BITFLAG (1<<RED_WE_ENUM)
#define YELLOW_WE_BITFLAG (1<<YELLOW_WE_ENUM)
#define GREEN_WE_BITFLAG (1<<GREEN_WE_ENUM)

struct state_config  static const STATE_CONFIGS[NUM_STATES] = {
     { // ALL_OFF
       .output = 0x00,
       .forever = 1,
       .pedestrian = 0,
     },
     { // ALL_ON
       .output = RED_NS_BITFLAG | YELLOW_NS_BITFLAG | GREEN_NS_BITFLAG |
		RED_WE_BITFLAG | YELLOW_WE_BITFLAG | GREEN_WE_BITFLAG,
       .forever = 1,
       .pedestrian = 0,
     },
     { // RED_GREEN
       .output = RED_NS_BITFLAG | GREEN_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 0,
       .time = GREEN_NS_TIME,
       .next_state = RED_YELLOW,
     },
     { // RED_YELLOW
       .output = RED_NS_BITFLAG | YELLOW_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 0,
       .time = YELLOW_TIME,
       .next_state = RED_EXTRARED,
     },
     { // RED_EXTRARED
       .output = RED_NS_BITFLAG | RED_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 1,
       .time = EXTRARED_TIME,
       .next_state = GREEN_RED,
       .next_state_pedestrian = RED_RED_NS,
     },
     { // GREEN_RED
       .output = GREEN_NS_BITFLAG | RED_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 0,
       .time = GREEN_WE_TIME,
       .next_state = YELLOW_RED,
     },
     { // YELLOW_RED
       .output = YELLOW_NS_BITFLAG | RED_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 0,
       .time = YELLOW_TIME,
       .next_state = EXTRARED_RED,
     },
     { // EXTRARED_RED
       .output = RED_NS_BITFLAG | RED_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 1,
       .time = EXTRARED_TIME,
       .next_state = RED_GREEN,
       .next_state_pedestrian = RED_RED_WE,
     },
     { // RED_RED_NS
       .output = RED_NS_BITFLAG | RED_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 2,
       .time = PEDESTRIAN_TIME,
       .next_state = GREEN_RED,
       .next_state_pedestrian = GREEN_RED,
     },
     { // RED_RED_WE
       .output = RED_NS_BITFLAG | RED_WE_BITFLAG,
       .forever = 0,
       .pedestrian = 2,
       .time = PEDESTRIAN_TIME,
       .next_state = RED_GREEN,
       .next_state_pedestrian = RED_GREEN,
     },
};

void traffic_state(void)
{
	// GPIO configuration
	for (int ii = 0; ii < NUM_LEDS; ii++) {
		const struct gpio_dt_spec *spec = &led_specs[ii];

		if (!device_is_ready(spec->port)) {
#ifdef DEBUG
			printk("Error: %d device is not ready\n", ii);
#endif
			return;
		}
		int ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);

		if (ret != 0) {
#ifdef DEBUG
			printk("Error %d: failed to configure pin %d (LED '%d')\n",
#endif
				ret, spec->pin, ii);
			return;
		}
	}


	enum state current_state = RED_GREEN;
	// Main loop
	while (1) {
#ifdef DEBUG
		printk("GPIO: %02x, state: %d\n", STATE_CONFIGS[current_state].output, current_state);
#endif

		// Set all LED GPIOs
		k_mutex_lock(&gpio_mutex, K_FOREVER);
		for (int ii = 0; ii < NUM_LEDS; ii++) {
			const struct gpio_dt_spec *spec = &led_specs[ii];
			gpio_pin_set(spec->port, spec->pin, STATE_CONFIGS[current_state].output & (1<<ii));
		}
		k_mutex_unlock(&gpio_mutex);

		// Sleep for configured time
		if (STATE_CONFIGS[current_state].forever) {
			k_sleep(K_FOREVER);
		} else
		{
			k_sleep(K_MSEC(STATE_CONFIGS[current_state].time));
		}

		// Determine next state, and handles pedestrian_crossing requests
		if (STATE_CONFIGS[current_state].pedestrian == 2) {
			// Pedestrian crossing is over, clear the request
			k_mutex_lock(&pedestrian_request_mutex, K_FOREVER);
			pedestrian_request = 0;
			k_mutex_unlock(&pedestrian_request_mutex);
			current_state = STATE_CONFIGS[current_state].next_state;
		} else if (STATE_CONFIGS[current_state].pedestrian == 1 && pedestrian_request)
		{
			// Pedestrian crossing is requested and should be attended to
			current_state = STATE_CONFIGS[current_state].next_state_pedestrian;
		} else
		{
			// Continue to next state
			current_state = STATE_CONFIGS[current_state].next_state;
		}
	}
}


// Handles pedestrian crossing button presses
void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
#ifdef DEBUG
		printk("Button pressed\n");
#endif
		k_mutex_lock(&pedestrian_request_mutex, K_FOREVER);
		pedestrian_request = 1;
		k_mutex_unlock(&pedestrian_request_mutex);
}

// Initialization of GPIO button pin and callback
void main(void)
{
	int ret;

	if (!device_is_ready(button.port)) {
#ifdef DEBUG
		printk("Error: button device %s is not ready\n",
		       button.port->name);
#endif
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
#ifdef DEBUG
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
#endif
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
#ifdef DEBUG
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
#endif
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
}

K_THREAD_DEFINE(traffic_state_id, STACKSIZE, traffic_state, NULL, NULL, NULL,
		PRIORITY, 0, 0);
