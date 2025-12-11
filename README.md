# BMduino

Extension for BestModules BMduino. Works with micro:bit v2 only - beta.  

## User Manual of BMB81TM01A
https://www.bestmodulescorp.com/en/amfile/file/download/file/2588/product/1418/


## Basic usage

### BMduino - OLED module

Display string or numbers on OLED.

```blocks
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "Hello 123!")
bmduino.writeStringOLED("Hello ")
bmduino.writeNumOLED(123)
bmduino.writeStringOLED("!")
bmduino.newLineOLED()
basic.forever(function () {
	
})
```

### BMduino - 4-KEY touch module

Get the key status of touch.

```blocks
bmduino.init4KeyModule()
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "number:")
bmduino.writeNumOLED(bmduino.numberOf4KeyModule())
basic.forever(function () {
    if (bmduino.pressedFrom4Key()) {
        bmduino.writeNumPlaceOLED(2, 1, bmduino.readKeyFrom4Key())
    }
    basic.pause(10)
})
```

### BMduino - ambient module

Obtain ambient light intensity((uint:lux).

```blocks
bmduino.initAmbientLightModule()
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "number:")
bmduino.writeNumOLED(bmduino.numberOfAmbientLightModule())
bmduino.writeStringPlaceOLED(2, 1, "Lux:")
basic.forever(function () {
    bmduino.writeNumPlaceOLED(3, 2, bmduino.readALS(1))
    basic.pause(100)
})
```

### BMduino - temperature & humidity module

Obtain the temperature and humidity of the environment.

```blocks
bmduino.initTemperatureAndHumidityModule()
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "Temp:")
bmduino.writeStringPlaceOLED(3, 1, "Humi:")
basic.forever(function () {
    bmduino.writeStringPlaceOLED(2, 8, "'C")
    bmduino.writeNumPlaceOLED(2, 2, bmduino.readTemperatureForTH(BMduinoTemperatureUnit.Celsius))
    bmduino.writeStringPlaceOLED(4, 8, "%RH")
    bmduino.writeNumPlaceOLED(4, 2, bmduino.readHumidityFromTH())
    basic.pause(2000)
})
```

### BMduino - PIR module

Detect whether there are animals (humans) moving close.
Note:Preheat for 30 seconds after power on.

```blocks
bmduino.initPIRModule(BMduinoUARTChannel.UART2)
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
while (bmduino.preheatingNotCompletedOfPIR()) {
    bmduino.writeStringPlaceOLED(1, 1, "Preheating")
    basic.pause(1000)
}
bmduino.clearDisplayOLED()
basic.forever(function () {
    if (bmduino.statusFromPIR()) {
        bmduino.writeStringPlaceOLED(3, 1, "Warning   ")
    } else {
        bmduino.writeStringPlaceOLED(3, 1, "No warning")
    }
    basic.pause(1000)
})
```

### BMduino - smoke detector module

Detecting smoke levels in the environment.

```blocks
bmduino.initSmokeModule(BMduinoUARTChannel.UART2)
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "Value:")
basic.forever(function () {
    bmduino.writeNumPlaceOLED(2, 2, bmduino.readSmokeValue(BMduinoSmokeType.WhiteSmoke))
    if (bmduino.statusFromSmoke()) {
        bmduino.writeStringPlaceOLED(3, 1, "Warning   ")
    } else {
        bmduino.writeStringPlaceOLED(3, 1, "No warning")
    }
    basic.pause(1000)
})
```

### BMduino - CO2 module

Detecting the CO2 value of the environment(uint:ppm).
Note:Preheat for 80 seconds after power on.

```blocks
bmduino.initCO2Module(BMduinoUARTChannel.UART2)
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
while (bmduino.preheatingNotCompletedOfCO2()) {
    bmduino.writeStringPlaceOLED(1, 1, "Preheating")
    basic.pause(1000)
}
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(2, 1, "CO2(ppm):")
basic.forever(function () {
    bmduino.writeNumPlaceOLED(3, 2, bmduino.readCO2())
    basic.pause(1000)
})
```

### BMduino - laser dust detection module

Detecting dust levels in the environment(uint:ug/m3).
Note:Preheat for 30 seconds after power on.

```blocks
bmduino.initLaserDustModule(BMduinoUARTChannel.UART2)
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
while (bmduino.preheatingNotCompletedOfLaserDust()) {
    bmduino.writeStringPlaceOLED(1, 1, "Preheating")
    basic.pause(1000)
}
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(2, 1, "PM2.5:")
bmduino.writeStringPlaceOLED(4, 6, "ug/m3")
basic.forever(function () {
    bmduino.writeNumPlaceOLED(3, 2, bmduino.readPM(BMduinoPMType.PM2p5))
    basic.pause(1000)
})
```

### BMduino - soil module

Obtain soil temperature(uint:℃) and humidity(uint:%RH) values.

```blocks
bmduino.initSoilModule(BMduinoUARTChannel.UART2)
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "Temp:")
bmduino.writeStringPlaceOLED(3, 1, "Humi:")
basic.forever(function () {
    bmduino.writeStringPlaceOLED(2, 8, "'C")
    bmduino.writeNumPlaceOLED(2, 2, bmduino.readTemperatureFromSoil(1))
    bmduino.writeStringPlaceOLED(4, 8, "%RH")
    bmduino.writeNumPlaceOLED(4, 2, bmduino.readMoistureFromSoil(1))
    basic.pause(1000)
})
```

### BMduino - TDS module

Obtain TDS(ppm) and temperature(℃) values of the liquid.

```blocks
bmduino.initTDSModule(BMduinoUARTChannel.UART2)
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "Temp:")
bmduino.writeStringPlaceOLED(3, 1, "TDS:")
basic.forever(function () {
    bmduino.writeStringPlaceOLED(2, 8, "'C")
    bmduino.writeNumPlaceOLED(2, 2, bmduino.readTemperatureFromTDS(BMduinoTDSChannel.Channel1))
    bmduino.writeStringPlaceOLED(4, 8, "ppm")
    bmduino.writeNumPlaceOLED(4, 2, bmduino.readTDS(BMduinoTDSChannel.Channel1))
    basic.pause(1000)
})
```

### BMduino - IR thermometry module

By using the principle of infrared temperature measurement, obtain temperature (human body temperature, object surface temperature, ambient temperature).

```blocks
bmduino.initIRThermometryModule()
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.writeStringPlaceOLED(1, 1, "Temp:")
basic.forever(function () {
    bmduino.writeStringPlaceOLED(2, 8, "'C")
    bmduino.writeNumPlaceOLED(2, 2, bmduino.readTemperatureFromIRThermometry(BMduinoTemperatureObject.BodyTemperature))
})
```

### BMduino - oximeter module

Read blood oxygen(uint:%), heart rate(uint:BPM), and perfusion index(%) values.

```blocks
bmduino.initOximeterModule(BMduinoUARTChannel.UART2)
bmduino.initOLEDModule(128, 64)
bmduino.clearDisplayOLED()
bmduino.beginMeasureOximeter()
basic.forever(function () {
    if (bmduino.requestInfoFromOximeter() == 1) {
        bmduino.clearDisplayOLED()
        bmduino.writeStringOLED("Please put your finger")
    }
    if (bmduino.requestInfoFromOximeter() == 2) {
        bmduino.clearDisplayOLED()
        bmduino.writeStringOLED("Testing...")
    }
    if (bmduino.requestInfoFromOximeter() == 3) {
        bmduino.writeStringPlaceOLED(1, 1, "SpO2:")
        bmduino.writeStringPlaceOLED(2, 1, "HR:")
        bmduino.writeStringPlaceOLED(3, 1, "PI:")
        bmduino.writeStringPlaceOLED(1, 10, "%")
        bmduino.writeNumPlaceOLED(1, 6, bmduino.readSpO2())
        bmduino.writeStringPlaceOLED(2, 8, "BMP")
        bmduino.writeNumPlaceOLED(2, 4, bmduino.readHeartRate())
        bmduino.writeStringPlaceOLED(3, 10, "%")
        bmduino.writeNumPlaceOLED(3, 4, bmduino.readPI())
    }
    basic.pause(500)
})
```

## Version History
* v1.0.0 - Initial public release（beta）.
* v1.0.1 - Improve standards（beta）.

## License

MIT

## Supported targets

* for PXT/microbit
(The metadata above is needed for package search.)

```package
https://github.com/BestModules-Libraries/pxt-bmduino
```
