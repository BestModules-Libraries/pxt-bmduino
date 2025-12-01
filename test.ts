bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "Hello 123!")
bmduino.writeStringOLED("Hello ")
bmduino.writeNumOLED(123)
bmduino.writeStringOLED("!")
bmduino.newLineOLED()
basic.forever(function () {
	
})