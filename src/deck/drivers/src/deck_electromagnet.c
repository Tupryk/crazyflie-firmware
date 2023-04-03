#define DEBUG_MODULE "DeckElectromagnet"
#include "debug.h"

// #include "deck_electromagnet.h"
#include "deck.h"

// const deckPin_t gatePin = DECK_GPIO_IO1;
#define GATE_PIN    DECK_GPIO_IO1


static void initializeDeckElectromagnet()
{
	DEBUG_PRINT("Initializing electromagnet deck!\n");

	// Configure pin IO1 as an output with pull-down resistor
	pinMode(GATE_PIN, OUTPUT);

	// Test: Activate electromagnet
	digitalWrite(GATE_PIN, HIGH);

	DEBUG_PRINT("Electromagnet deck initialized successfully!\n");
}

// static void activateDeckElectromagnet()
// {
// 	DEBUG_PRINT("Activating electromagnet deck!\n");

// 	// Activate electromagnet
// 	digitalWrite(gatePin, HIGH);
// }

// static void deactivateDeckElectromagnet()
// {
// 	DEBUG_PRINT("Deactivating electromagnet deck!\n");

// 	// Deactivate electromagnet
// 	digitalWrite(gatePin, LOW);
// }

static const DeckDriver driverElectromagnet = {
	.vid = 0,
  	.pid = 0,
	.name = "deckElectromagnet",
	.usedGpio = DECK_USING_IO_1,
	.init = initializeDeckElectromagnet,
};

DECK_DRIVER(driverElectromagnet);

// add a parameter to the deck driver
// PARAM_GROUP_START(electromagnet)

// PARAM_ADD(PARAM_BOOL, activate, &activateDeckElectromagnet)

// PARAM_GROUP_STOP(electromagnet)