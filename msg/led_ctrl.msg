uint8 COLOR_RED = 0
uint8 COLOR_GREEN = 1
uint8 COLOR_YELLOW = 2

uint8 NUM_LEDS = 4

uint32 EVENT_SMARTBOARD_MMI_LED_TEST = 0
uint32 EVENT_LED_CTRL_MAX = 0
uint32 EVENT_LED_CTRL_INVALID = 255


uint64 timestamp
bool[4] led_status # true: enable; false: disable
uint8[4] color
uint8[4] freq # Hz
uint32 event
