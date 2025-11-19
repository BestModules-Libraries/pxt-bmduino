# BMduino

A Microsoft MakeCode package for BMduino(BestModules)  
https://www.bestmodulescorp.com/en/bmb81tm01a.html


## Basic usage

### BMduino - OLED module

Display string or numbers on OLED.

```blocks
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "Hello 123!")
bmduino.bmd31M090_writeString("Hello ")
bmduino.bmd31M090_writeNum(123)
bmduino.bmd31M090_writeString("!")
bmduino.bmd31M090_newLine()
basic.forever(function () {
	
})
```

### BMduino - 4-KEY touch module

Get the key status of touch.

```blocks
bmduino.bmk52M134_begin()
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "number:")
bmduino.bmd31M090_writeNum(bmduino.bmk52M134_getNumber())
basic.forever(function () {
    if (bmduino.bmk52M134_getINT()) {
        bmduino.bmd31M090_writeNumPlace(2, 1, bmduino.bmk52M134_getKeyValue())
    }
    basic.pause(10)
})
```

### BMduino - ambient module

Obtain ambient light intensity((uint:lux).

```blocks
bmduino.bme82M131_begin()
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "number:")
bmduino.bmd31M090_writeNum(bmduino.bme82M131_getNumber())
bmduino.bmd31M090_writeStringPlace(2, 1, "Lux:")
basic.forever(function () {
    bmduino.bmd31M090_writeNumPlace(3, 2, bmduino.bme82M131_readALS(1))
    basic.pause(100)
})
```

### BMduino - temperature & humidity module

Obtain the temperature and humidity of the environment.

```blocks
bmduino.bm25S2021_1_begin()
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "Temp:")
bmduino.bmd31M090_writeStringPlace(3, 1, "Humi:")
basic.forever(function () {
    bmduino.bmd31M090_writeStringPlace(2, 8, "'C")
    bmduino.bmd31M090_writeNumPlace(2, 2, bmduino.bm25S2021_1_readTemperature(BMduino_TemperatureUnit.Celsius))
    bmduino.bmd31M090_writeStringPlace(4, 8, "%RH")
    bmduino.bmd31M090_writeNumPlace(4, 2, bmduino.bm25S2021_1_readHumidity())
    basic.pause(2000)
})
```

### BMduino - PIR module

Detect whether there are animals (humans) moving close.
Note:Preheat for 30 seconds after power on.

```blocks
bmduino.bm22S4221_1_begin(BMduino_UARTChannel.UART2)
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
while (bmduino.bm22S4221_1_preheatCountdown2()) {
    bmduino.bmd31M090_writeStringPlace(1, 1, "Preheating")
    basic.pause(1000)
}
bmduino.bmd31M090_clearDisplay()
basic.forever(function () {
    if (bmduino.bm22S4221_1_getStatus()) {
        bmduino.bmd31M090_writeStringPlace(3, 1, "Warning   ")
    } else {
        bmduino.bmd31M090_writeStringPlace(3, 1, "No warning")
    }
    basic.pause(1000)
})
```

### BMduino - smoke detector module

Detecting smoke levels in the environment.

```blocks
bmduino.BM22S2021_begin(BMduino_UARTChannel.UART2)
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "Value:")
basic.forever(function () {
    bmduino.bmd31M090_writeNumPlace(2, 2, bmduino.bm22S2021_1_readSmokeValue(BMduino_SmokeType.WhiteSmoke))
    if (bmduino.bm22S2021_1_getStatus()) {
        bmduino.bmd31M090_writeStringPlace(3, 1, "Warning   ")
    } else {
        bmduino.bmd31M090_writeStringPlace(3, 1, "No warning")
    }
    basic.pause(1000)
})
```

### BMduino - CO2 module

Detecting the CO2 value of the environment(uint:ppm).
Note:Preheat for 80 seconds after power on.

```blocks
bmduino.bm25S3321_1_begin(BMduino_UARTChannel.UART2)
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
while (bmduino.bm25S3321_1_preheatCountdown2()) {
    bmduino.bmd31M090_writeStringPlace(1, 1, "Preheating")
    basic.pause(1000)
}
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(2, 1, "CO2(ppm):")
basic.forever(function () {
    bmduino.bmd31M090_writeNumPlace(3, 2, bmduino.bm25S3321_1_readCO2())
})
```

### BMduino - laser dust detection module

Detecting dust levels in the environment(uint:ug/m3).
Note:Preheat for 30 seconds after power on.

```blocks
bmduino.bm25S3221_1_begin(BMduino_UARTChannel.UART2)
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
while (bmduino.bm25S3221_1_preheatCountdown2()) {
    bmduino.bmd31M090_writeStringPlace(1, 1, "Preheating")
    basic.pause(1000)
}
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(2, 1, "PM2.5:")
bmduino.bmd31M090_writeStringPlace(4, 6, "ug/m3")
basic.forever(function () {
    bmduino.bmd31M090_writeNumPlace(3, 2, bmduino.bm25S3221_1_readPM(BMduino_PMType.PM2))
    basic.pause(1000)
})
```

### BMduino - soil module

Obtain soil temperature(uint:℃) and humidity(uint:%RH) values.

```blocks
bmduino.bm25S2621_1_begin(BMduino_UARTChannel.UART2)
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "Temp:")
bmduino.bmd31M090_writeStringPlace(3, 1, "Humi:")
basic.forever(function () {
    bmduino.bmd31M090_writeStringPlace(2, 8, "'C")
    bmduino.bmd31M090_writeNumPlace(2, 2, bmduino.bm25S2621_1_readTemperature(1))
    bmduino.bmd31M090_writeStringPlace(4, 8, "%RH")
    bmduino.bmd31M090_writeNumPlace(4, 2, bmduino.bm25S2621_1_readMoisture(1))
    basic.pause(1000)
})
```

### BMduino - TDS module

Obtain TDS(ppm) and temperature(℃) values of the liquid.

```blocks
bmduino.bm25S4021_1_begin(BMduino_UARTChannel.UART2)
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "Temp:")
bmduino.bmd31M090_writeStringPlace(3, 1, "TDS:")
basic.forever(function () {
    bmduino.bmd31M090_writeStringPlace(2, 8, "'C")
    bmduino.bmd31M090_writeNumPlace(2, 2, bmduino.bm25S4021_1_Temperature(BMduino_TDSChannel.Channel1))
    bmduino.bmd31M090_writeStringPlace(4, 8, "ppm")
    bmduino.bmd31M090_writeNumPlace(4, 2, bmduino.bm25S4021_1_readTDS(BMduino_TDSChannel.Channel1))
    basic.pause(1000)
})
```

### BMduino - IR thermometry module

By using the principle of infrared temperature measurement, obtain temperature (human body temperature, object surface temperature, ambient temperature).

```blocks
bmduino.bmh06203_begin()
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmd31M090_writeStringPlace(1, 1, "Temp:")
basic.forever(function () {
    bmduino.bmd31M090_writeStringPlace(2, 8, "'C")
    bmduino.bmd31M090_writeNumPlace(2, 2, bmduino.bmh06203_readTemperature(BMduino_TemperatureObject.BodyTemperature))
})
```

### BMduino - oximeter module

Read blood oxygen(uint:%), heart rate(uint:BPM), and perfusion index(%) values.

```blocks
bmduino.bmh08101_initialize_d(BMduino_UARTChannel.UART2)
bmduino.bmd31M090_init(128, 64)
bmduino.bmd31M090_clearDisplay()
bmduino.bmh08101_beginMeasure()
basic.forever(function () {
    if (bmduino.bmh08101_requestInfo() == 1) {
        bmduino.bmd31M090_clearDisplay()
        bmduino.bmd31M090_writeString("Please put your finger")
    }
    if (bmduino.bmh08101_requestInfo() == 2) {
        bmduino.bmd31M090_clearDisplay()
        bmduino.bmd31M090_writeString("Testing...")
    }
    if (bmduino.bmh08101_requestInfo() == 3) {
        bmduino.bmd31M090_writeStringPlace(1, 1, "SpO2:")
        bmduino.bmd31M090_writeStringPlace(2, 1, "HR:")
        bmduino.bmd31M090_writeStringPlace(3, 1, "PI:")
        bmduino.bmd31M090_writeStringPlace(1, 10, "%")
        bmduino.bmd31M090_writeNumPlace(1, 6, bmduino.bmh08101_getSpO2())
        bmduino.bmd31M090_writeStringPlace(2, 8, "BMP")
        bmduino.bmd31M090_writeNumPlace(2, 4, bmduino.bmh08101_getHeartRate())
        bmduino.bmd31M090_writeStringPlace(3, 10, "%")
        bmduino.bmd31M090_writeNumPlace(3, 4, bmduino.bmh08101_getPI())
    }
    basic.pause(500)
})
```

## Version History
* v1.0.0 - Initial public release.

## License

MIT

## Supported targets

* for PXT/microbit
(The metadata above is needed for package search.)

```package
https://github.com/BestModules-Libraries/pxt-bmduino
```
