function oled() {
    serial.writeLine("oled")
    bmduino.initOLEDModule(128, 64)
    bmduino.clearDisplayOLED()
    bmduino.writeStringPlaceOLED(1, 1, "Hello 123!")
    bmduino.writeStringOLED("Hello ")
    bmduino.writeNumOLED(123)
    bmduino.writeStringOLED("!")
    bmduino.newLineOLED()
}
function touchKey() {
    serial.writeLine("touchKey")
    bmduino.init4KeyModule()
    serial.writeLine("begin")
    serial.writeString("number = ")
    serial.writeLine("" + bmduino.numberOf4KeyModule())
    serial.writeString("key value = ")
    serial.writeLine("" + bmduino.readKeyFrom4Key())
}
function ambient() {
    serial.writeLine("ambient")
    bmduino.initAmbientLightModule()
    serial.writeLine("begin")
    serial.writeString("number = ")
    serial.writeLine("" + bmduino.numberOfAmbientLightModule())
    serial.writeString("lux = ")
    serial.writeLine("" + bmduino.readALS(1))
}
function temperatureAndHumidity() {
    serial.writeLine("temperatureAndHumidity")
    bmduino.initTemperatureAndHumidityModule()
    serial.writeLine("begin")
    serial.writeString("Temperature = ")
    serial.writeLine("" + bmduino.readTemperatureForTH(BMduinoTemperatureUnit.Celsius))
    serial.writeString("Humidity = ")
    serial.writeLine("" + bmduino.readHumidityFromTH())
}
function pir() {
    serial.writeLine("pir")
    bmduino.initPIRModule(BMduinoUARTChannel.UART2)
    serial.writeLine("begin")
    serial.writeLine("preheating")
    while (bmduino.preheatingNotCompletedOfPIR())
    {
        basic.pause(1000)
    }
    serial.writeLine("preheat Finished")
    serial.writeString("status = ")
    serial.writeLine("" + bmduino.statusFromPIR())
}
function smoke() {
    serial.writeLine("smoke")
    bmduino.initSmokeModule(BMduinoUARTChannel.UART2)
    serial.writeLine("begin")
    serial.writeString("white smoke = ")
    serial.writeLine("" + bmduino.readSmokeValue(BMduinoSmokeType.WhiteSmoke))
    serial.writeString("black smoke = ")
    serial.writeLine("" + bmduino.readSmokeValue(BMduinoSmokeType.BlackSmoke))
    serial.writeString("status = ")
    serial.writeLine("" + bmduino.statusFromSmoke())
}
function co2() {
    serial.writeLine("co2")
    bmduino.initCO2Module(BMduinoUARTChannel.UART2)
    serial.writeLine("begin")
    serial.writeLine("preheating")
    while (bmduino.preheatingNotCompletedOfCO2()) {
        basic.pause(1000)
    }
    serial.writeLine("preheat Finished")
    serial.writeString("co2 = ")
    serial.writeLine("" + bmduino.readCO2())
}
function laserDust() {
    serial.writeLine("laserDust")
    bmduino.initLaserDustModule(BMduinoUARTChannel.UART2)
    serial.writeLine("begin")
    serial.writeLine("preheating")
    while (bmduino.preheatingNotCompletedOfLaserDust()) {
        basic.pause(1000)
    }
    serial.writeLine("preheat Finished")
    serial.writeString("PM2.5 = ")
    serial.writeLine("" + bmduino.readPM(BMduinoPMType.PM2p5))
}
function soil() {
    serial.writeLine("soil")
    bmduino.initSoilModule(BMduinoUARTChannel.UART2)
    serial.writeLine("begin")
    serial.writeString("temperature = ")
    serial.writeLine("" + bmduino.readTemperatureFromSoil(1))
    serial.writeString("moisture = ")
    serial.writeLine("" + bmduino.readMoistureFromSoil(1))
}
function tds() {
    serial.writeLine("soil")
    bmduino.initTDSModule(BMduinoUARTChannel.UART2)
    serial.writeLine("begin")
    serial.writeLine("" + bmduino.readTemperatureFromTDS(BMduinoTDSChannel.Channel1))
    serial.writeString("TDS = ")
    serial.writeLine("" + bmduino.readTDS(BMduinoTDSChannel.Channel1))
}
function irtherm() {
    serial.writeLine("irthermometry")
    bmduino.initIRThermometryModule()
    serial.writeString("body = ")
    serial.writeLine("" + bmduino.readTemperatureFromIRThermometry(BMduinoTemperatureObject.BodyTemperature))
    serial.writeString("object = ")
    serial.writeLine("" + bmduino.readTemperatureFromIRThermometry(BMduinoTemperatureObject.ObjectSurfacetemperature))
    serial.writeString("ambient = ")
    serial.writeLine("" + bmduino.readTemperatureFromIRThermometry(BMduinoTemperatureObject.AmbientTemperature))
}
function oximeter() {
    serial.writeLine("oximeter")
    bmduino.initOximeterModule(BMduinoUARTChannel.UART2)
    serial.writeLine("begin")
    bmduino.beginMeasureOximeter()
    if (bmduino.requestInfoFromOximeter() == 2) {
        serial.writeString("Sp02 = ")
        serial.writeLine("" + bmduino.readSpO2())
        serial.writeString("HR = ")
        serial.writeLine("" + bmduino.readHeartRate())
        serial.writeString("PI = ")
        serial.writeLine("" + bmduino.readPI())
    }
}


let auto = 1
let test = -1
let tests = [oled, touchKey, ambient, temperatureAndHumidity, pir, smoke, co2, laserDust, soil, tds, irtherm ,oximeter]

input.onButtonPressed(Button.A, function () {
    auto = 0
    test = (test + 1) % tests.length
    serial.writeLine("test " + test)
})
input.onButtonPressed(Button.B, function () {
    auto = 0
    test = (test + tests.length - 1) % tests.length
    serial.writeLine("test " + test)
})
input.onButtonPressed(Button.AB, function () {
    auto = 1
})

basic.pause(10000)

while (test < tests.length) {
    serial.writeLine("")
    serial.writeLine("")
    serial.writeLine("test " + test)
    serial.writeLine("auto " + auto)
    if (test >= 0 && test < tests.length) {
        tests[test]()
    }
    basic.pause(1000)
    if (auto == 1) {
        test = test + 1
    }
}
serial.writeLine("Test finished")
