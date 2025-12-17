enum BMduinoWiFiMode {
    //% block="STA mode"
    STA = 1,
    //% block="AP mode"
    AP = 2,
    //% block="AP+STA mode"
    AP_STA = 3
}
enum BMduinoUARTChannel {
    //% block="UART1"
    UART1 = 1,
    //% block="UART2"
    UART2 = 2,
    //% block="UART3"
    UART3 = 3
}
enum BMduinoGasConcentrationRange {
    //% block="5000ppm"
    Range5000 = 5000,
    //% block="2000ppm"
    Range2000 = 2000
}
enum BMduinoPMType {
    //% block="PM1.0"
    PM1 = 1,
    //% block="PM2.5"
    PM2 = 2,
    //% block="PM10"
    PM3 = 3
}
enum BMduinoTDSChannel {
    //% block="channel 1"
    Channel1 = 1,
    //% block="channel 2"
    Channel2 = 2
}
enum BMduinoTemperatureObject {
    //% block="body temperature"
    BodyTemperature = 0x0A,
    //% block="object surface temperature"
    ObjectSurfacetemperature = 0x09,
    //% block="ambient temperature"
    AmbientTemperature = 0x08

}
enum BMduinoSmokeType {
    //% block="white smoke"
    WhiteSmoke = 1,
    //% block="black smoke"
    BlackSmoke = 2
}
enum BMduinoTemperatureUnit {
    //% block="fahrenheit"
    Fahrenheit,
    //% block="celsius"
    Celsius
}
//% block="BMduino" color=#0fbc11 icon="\uf164"
namespace bmduino {
    //----------------------------------------------------------
    // Modules Name    : WIFI模块，WIFI模組，WIFI Module
    // Applicable types: bmc81m016a
    //----------------------------------------------------------
    let isInitialized = false;

    //% blockId=bmduino_bw16ReadResponse
    function bw16ReadResponse(): string {
        let response = "";
        let startTime = input.runningTime();
        let hasData = false;

        while (input.runningTime() - startTime < 4000) {
            let data = serial.readString();
            if (data && data.length > 0 && data.length < 22) {
                //basic.showString(data);
                response += data;
                hasData = true;
                basic.pause(10);
            }
        }

        if (!hasData) {
            basic.showString("NO RESP");
        }

        return response.trim(); // Remove unnecessary spaces or line breaks and return a response
    }


    /**
    * Connect To WiFi 
    * @param ssid WIFI ssid
    * @param password WIFI password
    */
    //% blockId=bmduino_bw16ConnectToWiFi block="WiFi name %ssid password %password"
    //% weight=100
    export function bw16ConnectToWiFi(ssid: string, password: string): boolean {
        let ret = false;
        if (!isInitialized) {
            let command = "AT\r\n";
            let command1 = `AT+WMODE=3,0\r\n`;
            let command2 = `AT+WJAP=${ssid},${password}\r\n`;

            serial.redirect(SerialPin.P1, SerialPin.P0, 115200); // Use P0 and P1 as UART

            // bmduino.writeStringNewLineOLED("Open your HOTSPOT!");

            let response = "";
            serial.writeString(command);
            basic.pause(100);
            response = bw16ReadResponse();
            if (response.includes("OK")) {
                let response1 = "";
                serial.writeString(command1);
                basic.pause(500);
                response1 = bw16ReadResponse();
                if (response1.includes("OK")) {
                    let response2 = "";

                    while (true) {
                        serial.writeString(command2);
                        basic.pause(500);
                        response2 = bw16ReadResponse();

                        if (response2.includes("OK")) {
                            // bmduino.clearDisplayOLED()
                            // bmduino.writeStringNewLineOLED("WiFi Connected!")
                            // bmduino.writeStringNewLineOLED("SSID: " + ssid)
                            basic.pause(500)
                            isInitialized = true
                            ret = true;
                            break;
                        } else {
                            // bmduino.clearDisplayOLED()
                            // bmduino.writeStringNewLineOLED("Connecting WiFi...")
                        }
                    }
                } else {
                    // bmduino.writeStringNewLineOLED("WiFi Mode Set ERROR");
                }
            }
        } else {
            basic.showString("ERROR"); // Display error
        }
        return ret;
    }



    /**
    * Connect To ThingSpeak
    * @param client ThingSpeak client
    * @param name ThingSpeak name
    * @param password ThingSpeak Password
    */
    //% blockId=bmduino_bw16SendMQTT block="ThingSpeak |  client ID %client| username %name| password %password|"
    //% weight=90
    export function bw16SendMQTT(client: string, name: string, password: string): boolean {
        let ret = false;
        if (isInitialized) {

            // bmduino.clearDisplayOLED()
            // bmduino.writeStringNewLineOLED("Waiting for upload...");

            let command1 = `AT+MQTT=1,"mqtt3.thingspeak.com"\r\n`;
            let command2 = `AT+MQTT=2,"1883"\r\n`;
            let command3 = `AT+MQTT=3,1\r\n`;
            let command4 = `AT+MQTT=4,"${client}"\r\n`;
            let command5 = `AT+MQTT=5,"${name}"\r\n`;
            let command6 = `AT+MQTT=6,"${password}"\r\n`;
            let command7 = `AT+MQTT\r\n`;
            let command8 = `AT+MQTT?\r\n`;


            let response = ""
            serial.writeString(command1);
            basic.pause(100);
            response = bw16ReadResponse(); // Read response
            // if (response.includes("OK")) {
            //     ret = true;
            // }
            basic.pause(500);

            let response1 = ""
            serial.writeString(command2);
            basic.pause(500);
            response1 = bw16ReadResponse();
            // if(response1.includes("OK"))
            // {
            //     ret=true;
            // }
            //bmduino.clearDisplayOLED();

            serial.writeString(command3);
            basic.pause(500);
            //bmduino.clearDisplayOLED();

            serial.writeString(command4);
            basic.pause(500);
            // bmduino.clearDisplayOLED();

            serial.writeString(command5);
            basic.pause(500);
            //bmduino.clearDisplayOLED();

            serial.writeString(command6);
            basic.pause(500);
            //bmduino.clearDisplayOLED();

            serial.writeString(command7);
            basic.pause(500);
            //bmduino.clearDisplayOLED();

            serial.writeString(command8);
            basic.pause(500);
            //bmduino.clearDisplayOLED();

        } else {
            //bmduino.writeStringNewLineOLED("ERROR: Module not initialized!");
        }
        return true;
    }


    /**
    * Connect To ThingSpeak
    * @param client ThingSpeak client
    * @param topic Publish topic is channel id
    * @param data Data1~data7 : field1~field8 data
    */
    //% expandableArgumentMode="enabled"
    //% blockId=bmduino_bw16PublishTopic block="send data | channel ID %topic| field1_data %data||field2_data %data1| field3_data %data2| field4_data %data3| field5_data %data4| field6_data %data5| field7_data %data6| field8_data %data7"
    //% topic.defl=0 data.defl=0 data1.defl=0 data2.defl=0 data3.defl=0 data4.defl=0 data5.defl=0 data6.defl=0 data.defl=0
    //% weight=80
    export function bw16PublishTopic(topic: number = 0, data: number = 0, data1: number = 0, data2: number = 0, data3: number = 0, data4: number = 0, data5: number = 0, data6: number = 0, data7: number = 0): void {
        let response = "";
        let topic1 = "channels" + "/" + topic + "/" + "publish";
        let data_buf1 = "field1=" + data;
        let data_buf2 = "field2=" + data1;
        let data_buf3 = "field3=" + data2;
        let data_buf4 = "field4=" + data3;
        let data_buf5 = "field5=" + data4;
        let data_buf6 = "field6=" + data5;
        let data_buf7 = "field7=" + data6;
        let data_buf8 = "field8=" + data7;

        let command = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf1 + "\r\n";
        let command1 = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf2 + "\r\n";
        let command2 = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf3 + "\r\n";
        let command3 = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf4 + "\r\n";
        let command4 = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf5 + "\r\n";
        let command5 = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf6 + "\r\n";
        let command6 = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf7 + "\r\n";
        let command7 = "AT+MQTTPUB=" + topic1 + ",1,0," + data_buf8 + "\r\n";

        for (let i = 0; i < 8; i++) {
            let retryCount = 0;
            let success = false;
            let currentCommand = "";

            switch (i) {
                case 0:
                    currentCommand = command;
                    break;
                case 1:
                    currentCommand = command1;
                    break;
                case 2:
                    currentCommand = command2;
                    break;
                case 3:
                    currentCommand = command3;
                    break;
                case 4:
                    currentCommand = command4;
                    break;
                case 5:
                    currentCommand = command5;
                    break;
                case 6:
                    currentCommand = command6;
                    break;
                case 7:
                    currentCommand = command7;
                    break;
            }

            while (retryCount <= 2 && !success) {
                serial.writeString(currentCommand);
                basic.pause(500);
                response = bw16ReadResponse();

                if (response.includes("ERROR")) {
                    retryCount++;
                    if (retryCount <= 2) {
                        basic.pause(500);
                    }
                } else {
                    success = true;
                    basic.pause(500);
                }
            }
        }

    }

    //----------------------------------------------------------
    // Modules Name    : 血氧模块，血氧模組，Oximeter Module
    // Applicable types: bmh08101/bmh83M101/bmh83M101A
    //----------------------------------------------------------
    const bmh08101_headByte = 0x55
    const bmh08101_tailByte = 0xAA
    const bmh08101_CmdByte = 0xB1
    let bmh08101_initialized = false
    let bmh08101_txPin = SerialPin.P0
    let bmh08101_rxPin = SerialPin.P1
    let bmh08101_dataBuffer = [0, 0, 0]  // [SpO2, heartrate, PI]
    let bmh08101_rxBuf: Buffer = pins.createBuffer(18)

    /**
     * Initialize the oximeter module connected to the specified UART interface
     * @param uart UART interface selection
     */
    //% blockId=bmduino_initOximeterModule block="initialize oximeter %uart"
    //% subcategory="Oximeter"
    //% weight=100
    export function initOximeterModule(uart: BMduinoUARTChannel): void {
        if (uart == BMduinoUARTChannel.UART1) {
            bmh08101_rxPin = SerialPin.P1
            bmh08101_txPin = SerialPin.P0
        } else if (uart == BMduinoUARTChannel.UART2) {
            bmh08101_rxPin = SerialPin.P13
            bmh08101_txPin = SerialPin.P12
        } else if (uart == BMduinoUARTChannel.UART3) {
            bmh08101_rxPin = SerialPin.P16
            bmh08101_txPin = SerialPin.P15
        }
        // Initialize UART
        serial.redirect(bmh08101_txPin, bmh08101_rxPin, BaudRate.BaudRate38400)
        basic.pause(5)
        //Activate the 'Inquire-respond'mode
        let buf = pins.createBuffer(5)
        buf[0] = bmh08101_headByte    // 0x55
        buf[1] = bmh08101_CmdByte     // 0xB1
        buf[2] = 0x04
        buf[3] = 0xB5
        buf[4] = 0xAA

        serial.readBuffer(0) //clear buff
        serial.writeBuffer(buf)
        basic.pause(50)

        bmh08101_rxBuf = serial.readBuffer(18)
        basic.pause(100)

        // Check if valid data has been received
        if (bmh08101_rxBuf.length === 18 && bmh08101_rxBuf[0] === bmh08101_headByte && bmh08101_rxBuf[1] == 0xB0 && bmh08101_rxBuf[17] === bmh08101_tailByte) {
            bmh08101_initialized = true
        } else {
            bmh08101_initialized = false
        }
    }

    /**
     * Start measure
     */
    //% blockId=bmduino_beginMeasureOximeter block="begin measure oximeter"
    //% subcategory="Oximeter"
    //% weight=90
    export function beginMeasureOximeter(): void {
        if (!bmh08101_initialized) return
        serial.redirect(bmh08101_txPin, bmh08101_rxPin, BaudRate.BaudRate38400)
        basic.pause(5)

        let buf = pins.createBuffer(5)
        buf[0] = bmh08101_headByte
        buf[1] = bmh08101_CmdByte
        buf[2] = 0x00
        buf[3] = 0xB1
        buf[4] = bmh08101_tailByte

        serial.writeBuffer(buf)
        basic.pause(50)
    }

    /**
     * Check the oxygen measurement status. 0x00: sensor error, 0x01: no finger detected, 0x02: finger detected, measurement incomplete, 0x03: finger detected, measurement completed
     */
    //% blockId=bmduino_requestInfoFromOximeter block="check the oximeter measurement status"
    //% subcategory="Oximeter"
    //% weight=85
    export function requestInfoFromOximeter(): number {
        serial.redirect(bmh08101_txPin, bmh08101_rxPin, BaudRate.BaudRate38400)
        basic.pause(5)
        serial.readBuffer(0) //clear buff
        bmh08101_dataBuffer[0] = 0
        bmh08101_dataBuffer[1] = 0
        bmh08101_dataBuffer[2] = 0

        // Send request
        let buf = pins.createBuffer(5)
        buf[0] = bmh08101_headByte        // 0x55
        buf[1] = bmh08101_CmdByte         // 0xB1
        buf[2] = 0x04
        buf[3] = 0xB5
        buf[4] = bmh08101_tailByte

        serial.writeBuffer(buf)
        basic.pause(50)

        bmh08101_rxBuf = serial.readBuffer(18)
        basic.pause(100)

        // Check if valid data has been received
        if (bmh08101_rxBuf.length === 18 && bmh08101_rxBuf[0] === bmh08101_headByte && bmh08101_rxBuf[1] == 0xB0 && bmh08101_rxBuf[17] === bmh08101_tailByte) {
            if (bmh08101_rxBuf[2] == 0x03) {
                // Update data buffer
                bmh08101_dataBuffer[0] = bmh08101_rxBuf[3]     // SpO2 (D2)
                bmh08101_dataBuffer[1] = bmh08101_rxBuf[4]     // heartrate (D3)
                bmh08101_dataBuffer[2] = bmh08101_rxBuf[5]     // PI (D4)
                return bmh08101_rxBuf[2]               // Return to finger measurement status 
            }
            else {
                return bmh08101_rxBuf[2]               // Return to finger measurement status 
            }
        }
        else {
            return 0
        }
    }

    /**
     * Read SpO2 value, 35~99%, uint:%
     */
    //% blockId=bmduino_readSpO2 block="SpO2"
    //% subcategory="Oximeter"
    //% weight=84
    export function readSpO2(): number {
        return bmh08101_dataBuffer[0]
    }

    /**
     * Read heartrate value, 30~250, uint:BPM
     */
    //% blockId=bmduino_readHeartRate block="heartrate"
    //% subcategory="Oximeter"
    //% weight=82
    export function readHeartRate(): number {
        return bmh08101_dataBuffer[1]
    }

    /**
     * Read blood perfusion index value, 0~200, 1 represents 0.1%
     */
    //% blockId=bmduino_readPI block="blood perfusion index"
    //% subcategory="Oximeter"
    //% weight=80
    export function readPI(): number {
        return Math.round(bmh08101_dataBuffer[2]) / 10  // PI/10 %
    }

    //----------------------------------------------------------
    // Modules Name    : 红外测温模块，紅外測溫模組，IR Thermometry Module
    // Applicable types: bmh06203/bmh06206/bmh63K203/bmh63K203A
    //----------------------------------------------------------
    const bmh06203_i2cAddr = 0x28;

    /**
     * Initialize the IR thermometry module
     */
    //% blockId=bmduino_initIRThermometryModule block="initialize IR thermometry"
    //% subcategory="IR thermometry" weight=255
    export function initIRThermometryModule(): void {
        // MakeCode defaults to handle I2C initialization without the need for additional settings
    }

    /**
     * Read IR thermometry temperature
     * @param obj Temperature type selection
     */
    //% blockId=bmduino_readTemperatureFromIRThermometry block="IR thermometry temperature %obj"
    //% subcategory="IR thermometry" weight=90
    export function readTemperatureFromIRThermometry(obj: BMduinoTemperatureObject): number {
        let configBuffer = pins.createBuffer(1);
        let temp_dataBuffer: number[] = [];//Store received data
        let obj_temp = 0x0A;

        if (obj == BMduinoTemperatureObject.BodyTemperature) {
            obj_temp = 0x0A;
        } else if (obj == BMduinoTemperatureObject.AmbientTemperature) {
            obj_temp = 0x08;
        } else if (obj == BMduinoTemperatureObject.ObjectSurfacetemperature) {
            obj_temp = 0x09;
        }

        configBuffer.setNumber(NumberFormat.UInt8LE, 0, obj_temp);
        pins.i2cWriteBuffer(bmh06203_i2cAddr, configBuffer);
        basic.pause(1)

        try {
            let dataBuffer = pins.i2cReadBuffer(bmh06203_i2cAddr, 3);
            for (let i = 0; i < 3; i++) {
                temp_dataBuffer[i] = dataBuffer.getNumber(NumberFormat.UInt8LE, i);
            }
            basic.pause(10)
            if (((dataBuffer[0] + dataBuffer[1]) & 0xff) == dataBuffer[2]) { //校驗通過
                // Merge high and low levels and convert to Celsius
                return ((dataBuffer[1] << 8) | dataBuffer[0]) / 10.0;
            }
            return 0;
        }
        catch {
            return 0;
        }
    }


    //----------------------------------------------------------
    // Modules Name    : TDS水质检测模块，TDS水質檢測模組，TDS Water Quality Detector Module
    // Applicable types: BM25S4021-1/BME63K402/BME63K402A
    //----------------------------------------------------------
    let bm25S4021_1_rxPin: SerialPin;  //rxPin
    let bm25S4021_1_txPin: SerialPin;  //txPin
    let bm25S4021_1_moduleID = 1;      //ID
    let bm25S4021_1_rXBuff: number[] = [];

    /**
     * Initialize the TDS module connected to the specified UART interface
     * @param uart UART interface selection
     * @param id Module identification code
     */
    //% blockId=bmduino_initTDSModule block="initialize TDS %uart"
    //% subcategory="TDS" 
    //% weight=3
    export function initTDSModule(uart: BMduinoUARTChannel): void {
        bm25S4021_1_moduleID = 1;

        // Set the corresponding RX and TX pins based on the selected UART channel
        if (uart == BMduinoUARTChannel.UART1) {
            bm25S4021_1_rxPin = SerialPin.P1; // UART1 RX
            bm25S4021_1_txPin = SerialPin.P0; // UART1 TX
        } else if (uart == BMduinoUARTChannel.UART2) {
            bm25S4021_1_rxPin = SerialPin.P13; // UART2 RX
            bm25S4021_1_txPin = SerialPin.P12; // UART2 TX
        } else if (uart == BMduinoUARTChannel.UART3) {
            bm25S4021_1_rxPin = SerialPin.P16; // UART3 RX
            bm25S4021_1_txPin = SerialPin.P15; // UART3 TX
        }
        // Reset UART to set RXBuffer TXBuffer length
        serial.redirect(bm25S4021_1_txPin, bm25S4021_1_rxPin, BaudRate.BaudRate9600);  // redirect uart
        serial.setRxBufferSize(255); // Set rxBuffer size to 255
        serial.setTxBufferSize(255); // Set txBuffer size to 255
    }

    /**
     * Read TDS value
     * @param channel Channels of TDS/NTC to be get
     */
    //% blockId=bmduino_readTDS block="TDS value %channel"
    //% subcategory="TDS" 
    //% weight=2
    export function readTDS(channel: BMduinoTDSChannel): number {
        let TDS = 0;
        let sendBuf: number[] = [0x42, 0x4D, 0x61, bm25S4021_1_moduleID, 0x01, 0x01, 0x00, 0x00]
        sendBuf[6] = channel;
        sendBuf[7] = ~(sendBuf[0] + sendBuf[1] + sendBuf[2] + sendBuf[3] + sendBuf[4] + sendBuf[5] + sendBuf[6]) + 1;
        sendBuf[7] = sendBuf[7] & 0xFF;
        writeBytesForTDS(sendBuf);
        basic.pause(30);
        if (readBytesForTDS(12, 30) == 0) {
            TDS = ((bm25S4021_1_rXBuff[7] << 8) | bm25S4021_1_rXBuff[8]) / 10;
        }
        basic.pause(10);
        return TDS;
    }

    /**
     * read water temperature
     * @param channel Channels of TDS/NTC to be get
     */
    //% blockId=bmduino_readTemperatureFromTDS block="water temperature %channel"
    //% subcategory="TDS" 
    //% weight=1
    export function readTemperatureFromTDS(channel: BMduinoTDSChannel): number {
        let Temperature = 0;
        let sendBuf: number[] = [0x42, 0x4D, 0x61, bm25S4021_1_moduleID, 0x01, 0x01, 0x00, 0x00]
        sendBuf[6] = channel;
        sendBuf[7] = ~(sendBuf[0] + sendBuf[1] + sendBuf[2] + sendBuf[3] + sendBuf[4] + sendBuf[5] + sendBuf[6]) + 1;
        sendBuf[7] = sendBuf[7] & 0xFF;
        writeBytesForTDS(sendBuf);
        basic.pause(30);
        if (readBytesForTDS(12, 30) == 0) {
            Temperature = ((bm25S4021_1_rXBuff[9] << 8) | bm25S4021_1_rXBuff[10]) / 10;
        }
        basic.pause(10);
        return Temperature;
    }

    //% blockId=bmduino_writeBytesForTDS
    function writeBytesForTDS(writeBuff: number[]): void {
        serial.redirect(bm25S4021_1_txPin, bm25S4021_1_rxPin, BaudRate.BaudRate9600);
        basic.pause(5);
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        serial.readBuffer(0);
        serial.writeBuffer(sendBuff);
    }

    //% blockId=bmduino_readBytesForTDS
    function readBytesForTDS(length: number, timeOut: number): number {
        let checkSum = 0;
        let lastreadLength = 0;  //Last received byte count
        let readLength = 0;  //Received byte count
        let delayCnt = 0;     //Timeout count
        let readBuffer;
        while (readLength != length) {
            readBuffer = serial.readBuffer(0);
            readLength = lastreadLength + readBuffer.length;
            for (let i = 0; i < length; i++) {
                bm25S4021_1_rXBuff[i + lastreadLength] = readBuffer[i];
            }
            lastreadLength = readLength;
            if ((readLength != length) && (delayCnt > timeOut)) {
                return 2; //Timeout
            }
            basic.pause(1);
            delayCnt++;
        }

        /**Data verification**/
        for (let i = 0; i < (length - 1); i++) {
            checkSum = bm25S4021_1_rXBuff[i] + checkSum;
        }
        checkSum = ~checkSum + 1;
        checkSum = checkSum & 0xFF;
        if (checkSum == bm25S4021_1_rXBuff[length - 1]) {
            return 0; // Verification successful
        }
        else {
            return 1; // Verification failed
        }
    }

    //----------------------------------------------------------
    // Modules Name    : 土壤温湿度检测模块，土壤溫濕度檢測模組，Soil Temperature and Moisture Detection Module
    // Applicable types: BM25S2621-1/BME34K262/BME34K262A
    //----------------------------------------------------------
    let bm25S2621_1_rxPin: SerialPin;  //rxPin
    let bm25S2621_1_txPin: SerialPin;  //txPin
    let bm25S2621_1_staPin: DigitalPin;  //staPin
    let bm25S2621_1_rXBuff: number[] = [];

    /**
     * Initialize the soil module connected to the specified UART interface
     * @param uart UART interface selection
     */
    //% blockId=bmduino_initSoilModule block="initialize soil %uart"
    //% subcategory="Soil"
    //% weight=5
    export function initSoilModule(uart: BMduinoUARTChannel): void {
        // Set the corresponding RX and TX pins based on the selected UART channel
        if (uart == BMduinoUARTChannel.UART1) {
            bm25S2621_1_rxPin = SerialPin.P1; // UART1 RX
            bm25S2621_1_txPin = SerialPin.P0; // UART1 TX
            bm25S2621_1_staPin = DigitalPin.P2; // UART1 STA
        } else if (uart == BMduinoUARTChannel.UART2) {
            bm25S2621_1_rxPin = SerialPin.P13; // UART2 RX
            bm25S2621_1_txPin = SerialPin.P12; // UART2 TX
            bm25S2621_1_staPin = DigitalPin.P8; // UART8 STA
        } else if (uart == BMduinoUARTChannel.UART3) {
            bm25S2621_1_rxPin = SerialPin.P16; // UART3 RX
            bm25S2621_1_txPin = SerialPin.P15; // UART3 TX
            bm25S2621_1_staPin = DigitalPin.P14; // UART3 STA
        }
        // Reset UART to set RXBuffer TXBuffer length
        serial.redirect(bm25S2621_1_txPin, bm25S2621_1_rxPin, BaudRate.BaudRate9600);  // redirect uart
        serial.setRxBufferSize(255); // Set rxBuffer size to 255
        serial.setTxBufferSize(255); // Set txBuffer size to 255
    }

    /**
     * Read module's moisture measurement value
     * @param id Current ID of module
     */
    //% blockId=bmduino_readMoistureFromSoil block="soil moisture, ID = %id"
    //% id.min=1 id.max=254 id.defl=1
    //% subcategory="Soil" 
    //% weight=4
    export function readMoistureFromSoil(id: number = 1): number {
        let Moisture = 0;
        let sendBuf: number[] = [0x00, 0x03, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00];
        sendBuf[0] = id;
        sendBuf[6] = modbusCRCForSoil(sendBuf.slice(0, 6)) & 0xff;
        sendBuf[7] = (modbusCRCForSoil(sendBuf.slice(0, 6)) >> 8) & 0xff;
        writeBytesForSoil(sendBuf);
        if (readBytesForSoil(9, 30) == 0) {
            Moisture = ((bm25S2621_1_rXBuff[3] << 8) | bm25S2621_1_rXBuff[4]);
        }
        basic.pause(5);
        return Moisture;
    }

    /**
     * Read module's temperature measurement value
     * @param id Current ID of module
     */
    //% blockId=bmduino_readTemperatureFromSoil block="soil temperature, ID = %id"
    //% id.min=1 id.max=254 id.defl=1
    //% subcategory="Soil"
    //% weight=3
    export function readTemperatureFromSoil(id: number = 1): number {
        let Temperature = 0;
        let sendBuf: number[] = [0x00, 0x03, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00];
        sendBuf[0] = id;
        sendBuf[6] = modbusCRCForSoil(sendBuf.slice(0, 6)) & 0xff;
        sendBuf[7] = (modbusCRCForSoil(sendBuf.slice(0, 6)) >> 8) & 0xff;
        writeBytesForSoil(sendBuf);
        if (readBytesForSoil(9, 30) == 0) {
            Temperature = ((bm25S2621_1_rXBuff[5] << 8) | bm25S2621_1_rXBuff[6]);
        }
        basic.pause(5);
        return Temperature;
    }


    /**
     * Get module's ID
     */
    //% blockId=bmduino_readIDFromSoil block="the ID of the soil module"
    //% subcategory="Soil" 
    //% weight=2
    export function readIDFromSoil(): number {
        let number = 0;
        let sendBuf: number[] = [0xFF, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00];
        sendBuf[6] = modbusCRCForSoil(sendBuf.slice(0, 6)) & 0xff;
        sendBuf[7] = (modbusCRCForSoil(sendBuf.slice(0, 6)) >> 8) & 0xff;
        writeBytesForSoil(sendBuf);
        if (readBytesForSoil(7, 30) == 0) {
            number = ((bm25S2621_1_rXBuff[3] << 8) | bm25S2621_1_rXBuff[4]);
        }
        basic.pause(5);
        return number;
    }

    /**
     * Set module's ID
     * @param currentID Current ID of module
     * @param newID New ID of module
     */
    //% blockId=bmduino_setIDForSoil block="set the ID of the soil module, current ID = %oldid|, target ID = %newid"
    //% oldid.min=1 oldid.max=254 oldid.defl=1
    //% newid.min=1 newid.max=254 newid.defl=1
    //% subcategory="Soil"
    //% weight=1
    export function setIDForSoil(oldid: number = 1, newid: number = 1): void {
        let sendBuf: number[] = [0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        sendBuf[0] = oldid;
        sendBuf[5] = newid;
        sendBuf[6] = modbusCRCForSoil(sendBuf.slice(0, 6)) & 0xff;
        sendBuf[7] = (modbusCRCForSoil(sendBuf.slice(0, 6)) >> 8) & 0xff;
        writeBytesForSoil(sendBuf);
        if (readBytesForSoil(8, 30) == 0) {
            basic.pause(5);
        }
        basic.pause(5);
    }

    //CRC verification
    //% blockId=bmduino_modbusCRCForSoil
    function modbusCRCForSoil(buf: number[]): number {
        let crc = 0xFFFF
        for (let pos = 0; pos < buf.length; pos++) {
            crc ^= buf[pos]
            for (let i = 0; i < 8; i++) {
                if ((crc & 1) != 0)
                    crc = (crc >> 1) ^ 0xA001
                else
                    crc >>= 1
            }
        }
        return crc
    }

    //% blockId=bmduino_writeBytesForSoil
    function writeBytesForSoil(writeBuff: number[]): void {
        serial.redirect(bm25S2621_1_txPin, bm25S2621_1_rxPin, BaudRate.BaudRate9600);  // redirect uart
        basic.pause(5);
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        serial.readBuffer(0);
        pins.digitalWritePin(bm25S2621_1_staPin, 1); // STA pin control is currently writing
        basic.pause(1);
        serial.writeBuffer(sendBuff);
        basic.pause(9);
        pins.digitalWritePin(bm25S2621_1_staPin, 0); // STA pin control is currently reading
    }

    //% blockId=bmduino_readBytesForSoil
    function readBytesForSoil(length: number, timeOut: number): number {
        let checkSumlow = 0;
        let checkSumhigh = 0;
        let lastreadLength = 0;  //Last received byte count
        let readLength = 0;  //Received byte count
        let delayCnt = 0;     //Timeout count
        let readBuffer;
        while (readLength != length) {
            readBuffer = serial.readBuffer(0);
            readLength = lastreadLength + readBuffer.length;
            for (let i = 0; i < length; i++) {
                bm25S2621_1_rXBuff[i + lastreadLength] = readBuffer[i];
            }
            lastreadLength = readLength;
            if ((readLength != length) && (delayCnt > timeOut)) {
                return 2; //Timeout
            }
            basic.pause(1);
            delayCnt++;
        }

        /**Data verification**/
        checkSumlow = modbusCRCForSoil(bm25S2621_1_rXBuff.slice(0, (length - 2))) & 0xff;
        checkSumhigh = (modbusCRCForSoil(bm25S2621_1_rXBuff.slice(0, (length - 2))) >> 8) & 0xff;
        if (checkSumlow == bm25S2621_1_rXBuff[length - 2] && checkSumhigh == bm25S2621_1_rXBuff[length - 1]) {
            return 0; // Verification successful
        }
        else {
            return 1; // Verification failed
        }
    }



    //----------------------------------------------------------
    // Modules Name    : 激光式粉尘侦测模块，雷射式粉塵偵測模組，Laser Dust Detection Module
    // Applicable types: BM25S3221-1/BME25K322/BME25K322A
    //----------------------------------------------------------
    let bm25S3221_1_rxPin: SerialPin;  //rxPin
    let bm25S3221_1_txPin: SerialPin;  //txPin
    let bm25S3221_1_rXBuff: number[] = [];

    /**
     * Initialize the laser dust detection module connected to the specified UART interface
     * @param uart UART interface selection
     */
    //% blockId=bmduino_initLaserDustModule block="initialize laser dust detection %uart"
    //% subcategory="Laser dust detection"
    //% weight=30
    export function initLaserDustModule(uart: BMduinoUARTChannel): void {
        // Set the corresponding RX and TX pins based on the selected UART channel
        if (uart == BMduinoUARTChannel.UART1) {
            bm25S3221_1_rxPin = SerialPin.P1; // UART1 RX
            bm25S3221_1_txPin = SerialPin.P0; // UART1 TX
        } else if (uart == BMduinoUARTChannel.UART2) {
            bm25S3221_1_rxPin = SerialPin.P13; // UART2 RX
            bm25S3221_1_txPin = SerialPin.P12; // UART2 TX
        } else if (uart == BMduinoUARTChannel.UART3) {
            bm25S3221_1_rxPin = SerialPin.P16; // UART3 RX
            bm25S3221_1_txPin = SerialPin.P15; // UART3 TX
        }
        // Reset UART to set RXBuffer TXBuffer length
        serial.redirect(bm25S3221_1_txPin, bm25S3221_1_rxPin, BaudRate.BaudRate9600);  // redirect uart
        serial.setRxBufferSize(255); // Set rxBuffer size to 255
        serial.setTxBufferSize(255); // Set txBuffer size to 255
        setUploadModeForLaserDust(); //set "Command query mode"
    }

    /**
     * Preheating completed? ture:complete, false:incomple
     */
    //% blockId=bmduino_preheatingCompletedOfLaserDust block="laser dust detection module preheating completed"
    //% subcategory="Laser dust detection"
    //% weight=20
    export function preheatingCompletedOfLaserDust(): boolean {
        if (input.runningTime() < 30000)   //Does the running time exceed 30 seconds
        {
            return false;
        }
        return true;
    }

    /**
     * Preheating not completed? ture:incomple, false:complete
     */
    //% blockId=bmduino_preheatingNotCompletedOfLaserDust block="laser dust detection module preheating not completed"
    //% subcategory="Laser dust detection"
    //% weight=19
    export function preheatingNotCompletedOfLaserDust(): boolean {
        if (input.runningTime() < 30000)   //Does the running time exceed 30 seconds
        {
            return true;
        }
        return false;
    }

    /**
     * Read PM value
     * @param pm PM1/PM2.5/PM10
     */
    //% blockId=bmduino_readPM block="read %pm value"
    //% subcategory="Laser dust detection"
    //% weight=10
    export function readPM(pm: BMduinoPMType): number {
        let PM = 0;
        let sendBuf: number[] = [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]
        writeBytesForLaserDust(sendBuf);
        if (readBytesForLaserDust(9, 10) == 0) {
            if (pm == 1) {
                PM = ((bm25S3221_1_rXBuff[2] << 8) | bm25S3221_1_rXBuff[3]);
            }
            else if (pm == 2) {
                PM = ((bm25S3221_1_rXBuff[4] << 8) | bm25S3221_1_rXBuff[5]);
            }
            else if (pm == 3) {
                PM = ((bm25S3221_1_rXBuff[6] << 8) | bm25S3221_1_rXBuff[7]);
            }
        }
        basic.pause(1);
        return PM;
    }

    // set "Command query mode"
    //% blockId=bmduino_setUploadModeForLaserDust
    function setUploadModeForLaserDust(): void {
        let sendBuf: number[] = [0xff, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46]
        writeBytesForLaserDust(sendBuf);
    }

    //% blockId=bmduino_writeBytesForLaserDust
    function writeBytesForLaserDust(writeBuff: number[]): void {
        serial.redirect(bm25S3221_1_txPin, bm25S3221_1_rxPin, BaudRate.BaudRate9600); // redirect uart
        basic.pause(5);
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        serial.readBuffer(0);
        serial.writeBuffer(sendBuff);
    }

    //% blockId=bmduino_readBytesForLaserDust
    function readBytesForLaserDust(length: number, timeOut: number): number {
        let checkSum = 0;
        let lastreadLength = 0;  //Last received byte count
        let readLength = 0;  //Received byte count
        let delayCnt = 0;     //Timeout count
        let readBuffer;
        while (readLength != length) {
            readBuffer = serial.readBuffer(0);
            readLength = lastreadLength + readBuffer.length;
            for (let i = 0; i < length; i++) {
                bm25S3221_1_rXBuff[i + lastreadLength] = readBuffer[i];
            }
            lastreadLength = readLength;
            if ((readLength != length) && (delayCnt > timeOut)) {
                return 2; //Timeout
            }
            basic.pause(1);
            delayCnt++;
        }

        /**Data verification**/
        for (let i = 1; i < (length - 1); i++) {
            checkSum = bm25S3221_1_rXBuff[i] + checkSum;
        }
        checkSum = ~checkSum + 1;
        checkSum = checkSum & 0xFF;
        if (checkSum == bm25S3221_1_rXBuff[length - 1]) {
            return 0; // Verification successful
        }
        else {
            return 1; // Verification failed
        }
    }

    //----------------------------------------------------------
    // Modules Name    : CO2侦测模块，CO2偵測模組，CO2 Detector Module
    // Applicable types: BM25S3321-1/BME58M332/BME58M332A
    //----------------------------------------------------------
    let bm25S3321_1_rxPin: SerialPin;  //rxPin
    let bm25S3321_1_txPin: SerialPin;  //txPin
    let bm25S3321_1_rXBuff: number[] = [];

    /**
     * Initialize the CO2 module connected to the specified UART interface
     * @param uart UART interface selection
     */
    //% blockId=bmduino_initCO2Module block="initialize CO2 %uart"
    //% subcategory="CO2"
    //% weight=30
    export function initCO2Module(uart: BMduinoUARTChannel): void {
        // Set the corresponding RX and TX pins based on the selected UART channel
        if (uart == BMduinoUARTChannel.UART1) {
            bm25S3321_1_rxPin = SerialPin.P1; // UART1 RX
            bm25S3321_1_txPin = SerialPin.P0; // UART1 TX
        } else if (uart == BMduinoUARTChannel.UART2) {
            bm25S3321_1_rxPin = SerialPin.P13; // UART2 RX
            bm25S3321_1_txPin = SerialPin.P12; // UART2 TX
        } else if (uart == BMduinoUARTChannel.UART3) {
            bm25S3321_1_rxPin = SerialPin.P16; // UART3 RX
            bm25S3321_1_txPin = SerialPin.P15; // UART3 TX
        }
        // Reset UART to set RXBuffer TXBuffer length
        serial.redirect(bm25S3321_1_txPin, bm25S3321_1_rxPin, BaudRate.BaudRate9600);  // redirect uart
        serial.setRxBufferSize(255); // Set rxBuffer size to 255
        serial.setTxBufferSize(255); // Set txBuffer size to 255
    }


    /**
     * Preheating completed? ture:complete;false:incomple
     */
    //% blockId=bmduino_preheatingCompletedOfCO2 block="CO2 module preheating completed"
    //% subcategory="CO2"
    //% weight=20
    export function preheatingCompletedOfCO2(): boolean {
        if (input.runningTime() < 80000)   //Does the running time exceed 80 seconds
        {
            return false;
        }
        return true;
    }

    /**
    * Preheating not completed? ture:incomple, false:complete
    */
    //% blockId=bmduino_preheatingNotCompletedOfCO2 block="CO2 module preheating not completed"
    //% subcategory="CO2"
    //% weight=19
    export function preheatingNotCompletedOfCO2(): boolean {
        if (input.runningTime() < 80000)   //Does the running time exceed 80 seconds
        {
            return true;
        }
        return false;
    }

    /**
     * Read CO2 value, unit:ppm
     */
    //% blockId=bmduino_readCO2 block="CO2 value"
    //% subcategory="CO2"
    //% weight=10
    export function readCO2(): number {
        let CO2 = 0;
        let sendBuf: number[] = [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]
        writeBytesForCO2(sendBuf);
        if (readBytesForCO2(9, 20) == 0) {
            CO2 = ((bm25S3321_1_rXBuff[2] << 8) | bm25S3321_1_rXBuff[3]);
        }
        basic.pause(1);
        return CO2;
    }

    //% blockId=bmduino_writeBytesForCO2
    function writeBytesForCO2(writeBuff: number[]): void {
        serial.redirect(bm25S3321_1_txPin, bm25S3321_1_rxPin, BaudRate.BaudRate9600); // redirect uart
        basic.pause(5);
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        serial.readBuffer(0);
        serial.writeBuffer(sendBuff);
    }

    //% blockId=bmduino_readBytesForCO2
    function readBytesForCO2(length: number, timeOut: number): number {
        let checkSum = 0;
        let lastreadLength = 0;  //Last received byte count
        let readLength = 0;  //Received byte count
        let delayCnt = 0;     //Timeout count
        let readBuffer;
        while (readLength != length) {
            readBuffer = serial.readBuffer(0);
            readLength = lastreadLength + readBuffer.length;
            for (let i = 0; i < length; i++) {
                bm25S3321_1_rXBuff[i + lastreadLength] = readBuffer[i];
            }
            lastreadLength = readLength;
            if ((readLength != length) && (delayCnt > timeOut)) {
                return 2; //Timeout
            }
            basic.pause(1);
            delayCnt++;
        }

        /**Data verification**/
        for (let i = 1; i < (length - 1); i++) {
            checkSum = bm25S3321_1_rXBuff[i] + checkSum;
        }
        checkSum = ~checkSum + 1;
        checkSum = checkSum & 0xFF;
        if (checkSum == bm25S3321_1_rXBuff[length - 1]) {
            return 0; // Verification successful
        }
        else {
            return 1; // Verification failed
        }
    }

    //----------------------------------------------------------
    // Modules Name    : 感烟侦测模块，感煙偵測模組，Smoke Detector Module
    // Applicable types: BM22S2021-1/BMA26M202/BMA26M202A
    //----------------------------------------------------------
    let bm22S2021_1_rxPin: SerialPin;  //rxPin
    let bm22S2021_1_txPin: SerialPin;  //txPin
    let bm22S2021_1_intPin = DigitalPin.P2;
    let bm22S2021_1_rXBuff: number[] = [];

    /**
     * Initialize the smoke detector module connected to the specified UART interface
     * @param uart UART interface selection
     */
    //% blockId=bmduino_initSmokeModule block="initialize smoke detector %uart"
    //% subcategory="Smoke detector" 
    //% weight=3
    export function initSmokeModule(uart: BMduinoUARTChannel): void {
        // Set the corresponding RX and TX pins based on the selected UART channel
        if (uart == BMduinoUARTChannel.UART1) {
            bm22S2021_1_rxPin = SerialPin.P1;   // UART1 RX
            bm22S2021_1_txPin = SerialPin.P0;   // UART1 TX
            bm22S2021_1_intPin = DigitalPin.P2; // UART1 INT
        } else if (uart == BMduinoUARTChannel.UART2) {
            bm22S2021_1_rxPin = SerialPin.P13;  // UART2 RX
            bm22S2021_1_txPin = SerialPin.P12;  // UART2 TX
            bm22S2021_1_intPin = DigitalPin.P8; // UART2 INT
        } else if (uart == BMduinoUARTChannel.UART3) {
            bm22S2021_1_rxPin = SerialPin.P16;  // UART3 RX
            bm22S2021_1_txPin = SerialPin.P15;  // UART3 TX
            bm22S2021_1_intPin = DigitalPin.P14;// UART3 INT
        }
        // Reset UART to set RXBuffer TXBuffer length
        serial.redirect(bm22S2021_1_txPin, bm22S2021_1_rxPin, BaudRate.BaudRate9600);  // redirect uart
        serial.setRxBufferSize(255); // Set rxBuffer size to 255
        serial.setTxBufferSize(255); // Set txBuffer size to 255
        pins.setPull(bm22S2021_1_intPin, PinPullMode.PullNone); //Int pin input
        setUploadModeForSmoke();
        basic.pause(100);
    }

    /**
     * Read the white smoke detection value, 12-bit A/D value
     * @param smoke Smoke type, smoke=1: white, smoke=2: black
     */
    //% blockId=bmduino_readSmokeValue block="%smoke value"
    //% subcategory="Smoke detector"
    //% weight=1
    export function readSmokeValue(smoke: BMduinoSmokeType): number {
        let SmokeValueL = 0;
        let SmokeValueH = 0;
        let SmokeValue = 0;
        let sendBufL1: number[] = [0xD2, 0x9B, 0x00, 0x93]; //white smoke's lowBytes
        let sendBufH1: number[] = [0xD2, 0x9C, 0x00, 0x92]; //white smoke's highBytes

        let sendBufL2: number[] = [0xD2, 0x9D, 0x00, 0x91]; //balck smoke's lowBytes
        let sendBufH2: number[] = [0xD2, 0x9E, 0x00, 0x90]; //balck smoke's highBytes
        if (smoke == BMduinoSmokeType.WhiteSmoke) { //white smoke
            writeBytesForSmoke(sendBufL1);
            if (readBytesForSmoke(8, 60) == 0 && bm22S2021_1_rXBuff[5] == 0x9B) {
                SmokeValueL = bm22S2021_1_rXBuff[6];
            }
            basic.pause(1);
            writeBytesForSmoke(sendBufH1);
            if (readBytesForSmoke(8, 60) == 0 && bm22S2021_1_rXBuff[5] == 0x9C) {
                SmokeValueH = bm22S2021_1_rXBuff[6];
            }
            basic.pause(1);
        }
        if (smoke == BMduinoSmokeType.BlackSmoke) { //balck smoke
            writeBytesForSmoke(sendBufL2);
            if (readBytesForSmoke(8, 60) == 0 && bm22S2021_1_rXBuff[5] == 0x9D) {
                SmokeValueL = bm22S2021_1_rXBuff[6];
            }
            basic.pause(1);
            writeBytesForSmoke(sendBufH2);
            if (readBytesForSmoke(8, 60) == 0 && bm22S2021_1_rXBuff[5] == 0x9E) {
                SmokeValueH = bm22S2021_1_rXBuff[6];
            }
            basic.pause(1);
        }
        if (SmokeValueH != 0 && SmokeValueL != 0) {
            SmokeValue = (SmokeValueH << 8) | SmokeValueL;
        }
        return SmokeValue;
    }

    /**
     * Check if the smoke alarm is triggered? ture:yes, false:no
     */
    //% blockId=bmduino_statusFromSmoke block="smoke alarm"
    //% subcategory="Smoke detector"
    //% weight=2
    export function statusFromSmoke(): boolean {
        if (pins.digitalReadPin(bm22S2021_1_intPin) == 1) {
            return true;
        }
        else {
            return false;
        }
    }

    // set "Command query mode" 
    //% blockId=bmduino_setUploadModeForSmoke
    function setUploadModeForSmoke(): void {
        let sendBuf: number[] = [0xE0, 0x2E, 0x00, 0xF2];
        writeBytesForSmoke(sendBuf);
    }

    //% blockId=bmduino_writeBytesForSmoke
    function writeBytesForSmoke(writeBuff: number[]): void {
        serial.redirect(bm22S2021_1_txPin, bm22S2021_1_rxPin, BaudRate.BaudRate9600); // redirect uart
        basic.pause(5);
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        serial.readBuffer(0);
        serial.writeBuffer(sendBuff);
    }

    //% blockId=bmduino_readBytesForSmoke
    function readBytesForSmoke(length: number, timeOut: number): number {
        let checkSum = 0;
        let lastreadLength = 0;  //Last received byte count
        let readLength = 0;  //Received byte count
        let delayCnt = 0;     //Timeout count
        let readBuffer;
        while (readLength != length) {
            readBuffer = serial.readBuffer(0);
            readLength = lastreadLength + readBuffer.length;
            for (let i = 0; i < length; i++) {
                bm22S2021_1_rXBuff[i + lastreadLength] = readBuffer[i];
            }
            lastreadLength = readLength;
            if ((readLength != length) && (delayCnt > timeOut)) {
                return 2; //Timeout
            }
            basic.pause(1);
            delayCnt++;
        }
        /**Data verification**/
        for (let i = 0; i < (length - 1); i++) {
            checkSum = bm22S2021_1_rXBuff[i] + checkSum;
        }
        checkSum = ~checkSum + 1;
        checkSum = checkSum & 0xFF;
        if (checkSum == bm22S2021_1_rXBuff[length - 1]) {
            return 0; // Verification successful
        }
        else {
            return 1; // Verification failed
        }
    }

    //----------------------------------------------------------
    // Modules Name    : PIR侦测模块，PIR偵測模組，PIR Detector Module
    // Applicable types: BM22S4021-1/BMA26M221/BMA26M221A
    //----------------------------------------------------------
    let bm22S4221_1_rxPin: SerialPin;  //rxPin
    let bm22S4221_1_txPin: SerialPin;  //txPin
    let bm22S4221_1_intPin: DigitalPin;
    let bm22S4221_1_rXBuff: number[] = [];

    /**
     * Initialize the PIR module connected to the specified UART interface
     * @param uart UART interface selection
     */
    //% blockId=bmduino_initPIRModule block="initialize PIR %uart"
    //% subcategory="PIR" 
    //% weight=40
    export function initPIRModule(uart: BMduinoUARTChannel): void {
        // Set the corresponding RX and TX pins based on the selected UART channel
        if (uart == BMduinoUARTChannel.UART1) {
            bm22S4221_1_rxPin = SerialPin.P1;   // UART1 RX
            bm22S4221_1_txPin = SerialPin.P0;   // UART1 TX
            bm22S4221_1_intPin = DigitalPin.P2; // UART1 INT
        } else if (uart == BMduinoUARTChannel.UART2) {
            bm22S4221_1_rxPin = SerialPin.P13;  // UART2 RX
            bm22S4221_1_txPin = SerialPin.P12;  // UART2 TX
            bm22S4221_1_intPin = DigitalPin.P8; // UART2 INT
        } else if (uart == BMduinoUARTChannel.UART3) {
            bm22S4221_1_rxPin = SerialPin.P16;  // UART3 RX
            bm22S4221_1_txPin = SerialPin.P15;  // UART3 TX
            bm22S4221_1_intPin = DigitalPin.P14;// UART3 INT
        }
        // Reset UART to set RXBuffer TXBuffer length
        serial.redirect(bm22S4221_1_txPin, bm22S4221_1_rxPin, BaudRate.BaudRate9600);  // redirect uart
        serial.setRxBufferSize(255); // Set rxBuffer size to 255
        serial.setTxBufferSize(255); // Set txBuffer size to 255
        pins.setPull(bm22S4221_1_intPin, PinPullMode.PullNone); //Int pin input
    }

    /**
     * Preheating completed? ture:complete, false:incomple
     */
    //% blockId=bmduino_preheatingCompletedOfPIR block="PIR module preheating completed"
    //% subcategory="PIR" 
    //% weight=30
    export function preheatingCompletedOfPIR(): boolean {
        if (input.runningTime() < 30000)   //Does the running time exceed 30 seconds
        {
            return false;
        }
        return true;
    }

    /**
     * Preheating not completed? ture:incomple, false:complete
     */
    //% blockId=bmduino_preheatingNotCompletedOfPIR block="PIR module preheating not completed"
    //% subcategory="PIR" 
    //% weight=20
    export function preheatingNotCompletedOfPIR(): boolean {
        if (input.runningTime() < 30000)   //Does the running time exceed 30 seconds
        {
            return true;
        }
        return false;
    }

    /**
     * Check if PIR is triggered? ture:yes, false:no
     */
    //% blockId=bmduino_statusFromPIR block="PIR is triggered"
    //% subcategory="PIR" 
    //% weight=10
    export function statusFromPIR(): boolean {
        if (pins.digitalReadPin(bm22S4221_1_intPin) == 1) {
            return true;
        }
        else {
            return false;
        }
    }

    //----------------------------------------------------------
    // Modules Name    : 温湿度模块，溫濕度模組，Temperature & Humidity Module
    // Applicable types: BM25S2021-1/BME33M251/BME33M251A
    //----------------------------------------------------------
    let bm25S2021_1_dataBuff: number[] = [];
    // I2C write
    //% blockId=bmduino_writeBytesForTH
    function writeBytesForTH(addr: number, writeBuff: number[]): void {
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        pins.i2cWriteBuffer(addr, sendBuff);
    }

    // I2C read
    //% blockId=bmduino_readBytesForTH
    function readBytesForTH(addr: number, length: number): number {  //Read data (address, byte count)
        try {
            let readBuffer = pins.i2cReadBuffer(addr, length); // IIC read
            for (let i = 0; i < length; i++) {
                bm25S2021_1_dataBuff[i] = readBuffer.getNumber(NumberFormat.UInt8LE, i);
            }
            return 0;     //Read successfully
        } catch {
            return -1;   //Read failed
        }
    }

    //Clear the receiving buffer
    //% blockId=bmduino_clearBuffForTH
    function clearBuffForTH(): void {
        for (let i = 0; i < bm25S2021_1_dataBuff.length; i++) {
            bm25S2021_1_dataBuff[i] = 0;
        }
    }

    /**
     * Initialize the temperature & humidity module
     */
    //% blockId=bmduino_initTemperatureAndHumidityModule block="initialize temperature & humidity"
    //% subcategory="Temperature & humidity"
    //% weight=100
    export function initTemperatureAndHumidityModule(): void {
        // MakeCode defaults to handle I2C initialization without the need for additional settings
    }

    /**
     * Read temperature, unit:℃ or ℉
     * @param unit Unit Selection, true:Fahrenheit degree, false:centigrade
     */
    //% blockId=bmduino_readTemperatureForTH block="ambient temperature %unit"
    //% subcategory="Temperature & humidity"
    //% weight=90
    export function readTemperatureForTH(unit: BMduinoTemperatureUnit): number {
        let temperature = 0;
        let sendbuf: number[] = [0x03, 0x02, 0x02]; // read temperature CMD
        clearBuffForTH(); ////Clear buffer

        // Send command and read data
        writeBytesForTH(0x5C, sendbuf); // The sensor address is 0x5C
        basic.pause(2); // Waiting for sensor response
        if (readBytesForTH(0x5C, 6) == 0) { // read 6 bytes
            if (bm25S2021_1_dataBuff[2] == 0 && bm25S2021_1_dataBuff[3] == 0) {
                return 0;  //Verification failed
            }
            temperature = (bm25S2021_1_dataBuff[2] << 8) | bm25S2021_1_dataBuff[3]; // Merge high and low level data
            if (unit === BMduinoTemperatureUnit.Fahrenheit) {
                temperature = temperature * 0.18 + 32; // Fahrenheit temperature conversion
            } else {
                temperature = temperature / 10; // Celsius temperature
            }
        }
        basic.pause(40);
        return temperature;
    }

    /**
     * Read humidity, unit:%RH
     */
    //% blockId=bmduino_readHumidityFromTH block="ambient humidity"
    //% subcategory="Temperature & humidity"
    //% weight=80
    export function readHumidityFromTH(): number {
        let humidity = 0;
        let sendbuf: number[] = [0x03, 0x00, 0x02]; // read humidity CMD
        clearBuffForTH(); ////Clear buffer

        // Send command and read data
        writeBytesForTH(0x5C, sendbuf); // The sensor address is 0x5C
        basic.pause(2); // Waiting for sensor response
        if (readBytesForTH(0x5C, 6) == 0) { // read 6 bytes
            if (bm25S2021_1_dataBuff[2] == 0 && bm25S2021_1_dataBuff[3] == 0) {
                return 0;  //Verification failed
            }
            humidity = (bm25S2021_1_dataBuff[2] << 8) | bm25S2021_1_dataBuff[3]; // Merge high and low level data
            humidity = humidity / 10; // humidity
        }
        basic.pause(40);
        return humidity;
    }

    //----------------------------------------------------------
    // Modules Name    : 环境光模块，環境光模組，Ambient Light Detection Module
    // Applicable types: bme82M131/bme82M131A
    //----------------------------------------------------------
    let bme82M131_dataBuff: number[] = [];

    const bme82M131_I2CAddr = 0x48;  // I2C_Addr(0x48~0x4f)
    const bme82M131_MID = 0x48;       // MID(0x48)
    let bme82M131_intPin = DigitalPin.P9;

    // I2C write
    //% blockId=bmduino_writeBytesForAmbientLight
    function writeBytesForAmbientLight(addr: number, writeBuff: number[]): void {
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        pins.i2cWriteBuffer(addr, sendBuff);
    }

    // I2C read
    //% blockId=bmduino_readBytesForAmbientLight
    function readBytesForAmbientLight(addr: number, length: number): number {  //Read data (address, byte count)
        try {
            let readBuffer = pins.i2cReadBuffer(addr, length); // IIC read
            for (let i = 0; i < length; i++) {
                bme82M131_dataBuff[i] = readBuffer.getNumber(NumberFormat.UInt8LE, i);
            }
            return 0;     //Read successfully
        } catch {
            return -1;   //Read failed
        }
    }

    //Clear the receiving buffer
    //% blockId=bmduino_clearBuffForAmbientLight
    function clearBuffForAmbientLight(): void {
        for (let i = 0; i < bme82M131_dataBuff.length; i++) {
            bme82M131_dataBuff[i] = 0;
        }
    }

    //% blockId=bmduino_initializeAmbientLightModule
    function initializeAmbientLightModule(): number {
        clearBuffForAmbientLight();  //clear buff
        let moduleNumber = numberOfAmbientLightModule();
        for (let i = 1; i <= moduleNumber; i++) {
            let sendbuf: number[] = [bme82M131_MID, i, 0x04, 0x02, 0x00, 0x00, ((bme82M131_MID + i + 0x04 + 0x02) & 0xFF)]; // 設置ALS參數指令
            // Send command and read data
            writeBytesForAmbientLight(bme82M131_I2CAddr, sendbuf);
            basic.pause(15); // Waiting for sensor response
            if (readBytesForAmbientLight(bme82M131_I2CAddr, 5) == 0) { // read 5 bytes
                if (bme82M131_dataBuff[0] != bme82M131_MID && bme82M131_dataBuff[3] != 0) {
                    return 0;  //Verification failed
                }
            }
            basic.pause(15);
        }
        return 1; //success
    }

    /**
     * initialize ambient light module
     */
    //% blockId=bmduino_initAmbientLightModule block="initialize ambient light"
    //% subcategory="Ambient light"
    //% weight=100
    export function initAmbientLightModule(): void {
        // MakeCode defaults to handle I2C initialization without the need for additional settings
        initializeAmbientLightModule();
    }

    /**
     * Obtain the number of ambient light modules
     */
    //% blockId=bmduino_numberOfAmbientLightModule block="the number of ambient light modules"
    //% subcategory="Ambient light"
    //% weight=90
    export function numberOfAmbientLightModule(): number {
        let moduleNumber = 0;
        let sendbuf: number[] = [bme82M131_MID, 0x01, 0x02, 0x01, ((bme82M131_MID + 0x01 + 0x02 + 0x01) & 0xFF)]; // 讀取模组级联数量
        clearBuffForAmbientLight();  //clear buff

        // Send command and read data
        writeBytesForAmbientLight(bme82M131_I2CAddr, sendbuf);
        basic.pause(15); // Waiting for sensor response
        if (readBytesForAmbientLight(bme82M131_I2CAddr, 6) == 0) { // read 6 bytes
            if (bme82M131_dataBuff[0] != bme82M131_MID && bme82M131_dataBuff[3] != 0) {
                return 0;  //Verification failed
            }
            moduleNumber = bme82M131_dataBuff[4];
        }
        basic.pause(15);
        return moduleNumber;
    }

    /**
     * Read smbient light value, uint:Lux
     * @param sensorID Module ID
     */
    //% blockId=bmduino_readALS block="the ambient light value, ID = %sensorID"
    //% subcategory="Ambient light"
    //% weight=80
    //% sensorID.defl=1
    export function readALS(sensorID: number): number {
        let rawData = 0;
        let sendbuf: number[] = [bme82M131_MID, sensorID, 0x02, 0x0A, ((bme82M131_MID + sensorID + 0x02 + 0x0A) & 0xFF)]; // 讀取環境光命令
        clearBuffForAmbientLight();  //clear buff
        // Send command and read data
        writeBytesForAmbientLight(bme82M131_I2CAddr, sendbuf);
        basic.pause(15); // Waiting for sensor response
        if (readBytesForAmbientLight(bme82M131_I2CAddr, 7) == 0) { // read 7 bytes
            if (bme82M131_dataBuff[0] != bme82M131_MID && bme82M131_dataBuff[3] != 0) {
                return 0;  //Verification failed
            }
            rawData = (bme82M131_dataBuff[5] << 8) | bme82M131_dataBuff[4]; // Merge high and low level data
            basic.pause(15);
            return rawData * 0.0576; // Convert to Lux
        }
        return 0;
    }

    //----------------------------------------------------------
    // Modules Name    : 4-KEY电容式触控模块，4-KEY電容式觸控模組，4-KEY Capacitive Touch Module
    // Applicable types: bmk52M134/bmk52M134A
    //----------------------------------------------------------
    let bmk52M134_dataBuff: number[] = [];

    const bmk52M134_I2CAddr = 0x71;  // I2C_Addr(0x71)
    const bmk52M134_MID = 0x71;       // MID(0x71)
    let bmk52M134_intPin = DigitalPin.P9;

    // CMD
    const bmk52M134_CMD_checkModule = 0x01  // get number of modules
    const bmk52M134_CMD_keyScan = 0x02      // scan keys
    const bmk52M134_CMD_setTHR = 0x03       // set threshold
    const bmk52M134_CMD_getTHR = 0x04       // get threshold
    const bmk52M134_CMD_setSleepEN = 0x05   // set sleepEN
    const bmk52M134_CMD_getSleepEN = 0x06   // get sleepEN

    // I2C write
    //% blockId=bmduino_writeBytesFor4Key
    function writeBytesFor4Key(addr: number, writeBuff: number[]): void {
        let sendBuff = pins.createBuffer(writeBuff.length);
        for (let i = 0; i < writeBuff.length; i++) {
            sendBuff.setNumber(NumberFormat.UInt8LE, i, writeBuff[i]);
        }
        pins.i2cWriteBuffer(addr, sendBuff);
    }

    // I2C read
    //% blockId=bmduino_readBytesFor4Key
    function readBytesFor4Key(addr: number, length: number): number {  //Read data (address, byte count)
        try {
            let readBuffer = pins.i2cReadBuffer(addr, length); // IIC read
            for (let i = 0; i < length; i++) {
                bmk52M134_dataBuff[i] = readBuffer.getNumber(NumberFormat.UInt8LE, i);
            }
            return 0;     //Read successfully
        } catch {
            return -1;   //Read failed
        }
    }

    //Clear the receiving buffer
    //% blockId=bmduino_clearBuffFor4Key
    function clearBuffFor4Key(): void {
        for (let i = 0; i < bmk52M134_dataBuff.length; i++) {
            bmk52M134_dataBuff[i] = 0;
        }
    }

    /**
     * Initialize 4-KEY touch module
     */
    //% blockId=bmduino_init4KeyModule block="initialize 4-KEY touch"
    //% subcategory="4-KEY touch"
    //% weight=100
    export function init4KeyModule(): void {
        // MakeCode defaults to handle I2C initialization without the need for additional settings
        pins.setPull(bmk52M134_intPin, PinPullMode.PullUp); //Int pin input上拉
    }

    /**
     * Obtain the number of 4-KEY touch modules
     */
    //% blockId=bmduino_numberOf4KeyModule block="the number of 4-KEY touch modules"
    //% subcategory="4-KEY touch"
    //% weight=90
    export function numberOf4KeyModule(): number {
        let moduleNumber = 0;
        let sendbuf: number[] = [bmk52M134_MID, 0x00, 0x02, bmk52M134_CMD_checkModule, ((bmk52M134_MID + 0x02 + bmk52M134_CMD_checkModule) & 0xFF)]; // 讀取模组级联数量
        clearBuffFor4Key();  //clear buff

        // Send command and read data
        writeBytesFor4Key(bmk52M134_I2CAddr, sendbuf);
        basic.pause(15); // Waiting for sensor response
        if (readBytesFor4Key(bmk52M134_I2CAddr, 6) == 0) { // read 6 bytes
            if (bmk52M134_dataBuff[0] != bmk52M134_MID && bmk52M134_dataBuff[3] != 0) {
                return 0;  //Verification failed
            }
            moduleNumber = bmk52M134_dataBuff[4];
        }
        basic.pause(15);
        return moduleNumber;
    }

    /**
     * Touch pressed? 0-Pressed down;1-Not pressed
     */
    //% blockId=bmduino_pressedFrom4Key block="4-KEY touch pressed?"
    //% subcategory="4-KEY touch"
    //% weight=80
    export function pressedFrom4Key(): boolean {
        if (pins.digitalReadPin(bmk52M134_intPin) == 1) {
            return false;
        }
        return true;
    }

    /**
    * Get key value
    */
    //% blockId=bmduino_readKeyFrom4Key block="the serial number of 4-KEY touch"
    //% subcategory="4-KEY touch"
    //% weight=70
    export function readKeyFrom4Key(): number {
        let keyValue = 0;
        let moduleNumber = numberOf4KeyModule();

        let sendbuf: number[] = [bmk52M134_MID, 0x00, 0x02, bmk52M134_CMD_keyScan, ((bmk52M134_MID + 0x02 + bmk52M134_CMD_keyScan) & 0xFF)]; // 讀取按鍵值
        clearBuffFor4Key();  //clear buff

        // Send command and read data
        writeBytesFor4Key(bmk52M134_I2CAddr, sendbuf);
        basic.pause(15); // Waiting for sensor response
        if (readBytesFor4Key(bmk52M134_I2CAddr, 5 + moduleNumber) == 0) {
            if (bmk52M134_dataBuff[0] != bmk52M134_MID && bmk52M134_dataBuff[3] != 0) {
                return 0;  //Verification failed
            }
            for (let i = 0; i < moduleNumber; i++) {
                if (bmk52M134_dataBuff[i + 4] != 0) {
                    for (let j = 0; j < 4; j++) {
                        if (bmk52M134_dataBuff[i + 4] & (1 << j)) {
                            keyValue = 4 * i + (j + 1);
                            return keyValue;
                        }
                    }
                }
            }
        }
        return 0;
    }

    /**
    * A touch is pressed？ 0-pressed down, 1-not pressed
    */
    //% blockId=bmduino_isPressedFrom4Key block="is touch %key pressed?"
    //% subcategory="4-KEY touch"
    //% key.min=1 key.max=16 key.defl=1
    //% weight=60
    export function isPressedFrom4Key(key: number): boolean {
        if (readKeyFrom4Key() == key) {
            return true;
        }
        return false;
    }

    //----------------------------------------------------------
    // Modules Name    : 0.96"OLED显示模块，0.96"OLED顯示模組，0.96" OLED Display Module
    // Applicable types: bmd31M090/bmd31M090A
    //----------------------------------------------------------
    let font: Buffer;
    let _screen: Buffer = pins.createBuffer(128 * 64 / 8);
    const bmd31M090_I2CAddr = 0x3C; // I2C address

    const SSD1306_SETCONTRAST = 0x81;
    const SSD1306_SETCOLUMNADRESS = 0x21;
    const SSD1306_SETPAGEADRESS = 0x22;
    const SSD1306_DISPLAYALLON_RESUME = 0xA4;
    const SSD1306_DISPLAYALLON = 0xA5;
    const SSD1306_NORMALDISPLAY = 0xA6;
    const SSD1306_INVERTDISPLAY = 0xA7;
    const SSD1306_DISPLAYOFF = 0xAE;
    const SSD1306_DISPLAYON = 0xAF;
    const SSD1306_SETDISPLAYOFFSET = 0xD3;
    const SSD1306_SETCOMPINS = 0xDA;
    const SSD1306_SETVCOMDETECT = 0xDB;
    const SSD1306_SETDISPLAYCLOCKDIV = 0xD5;
    const SSD1306_SETPRECHARGE = 0xD9;
    const SSD1306_SETMULTIPLEX = 0xA8;
    const SSD1306_SETLOWCOLUMN = 0x00;
    const SSD1306_SETHIGHCOLUMN = 0x10;
    const SSD1306_SETSTARTLINE = 0x40;
    const SSD1306_MEMORYMODE = 0x20;
    const SSD1306_COMSCANINC = 0xC0;
    const SSD1306_COMSCANDEC = 0xC8;
    const SSD1306_SEGREMAP = 0xA0;
    const SSD1306_CHARGEPUMP = 0x8D;
    const chipAdress = 0x3C;
    const xOffset = 0;
    const yOffset = 0;
    let charX = 0; //Record the latest location X
    let charY = 0;
    let charX_lastPlace = 0; //Record the last text position X
    let charY_lastPlace = 0;
    let displayWidth = 128;
    let displayHeight = 64 / 8;
    let screenSize = 0;
    let loadStarted: boolean;
    let loadPercent: number;
    let _ZOOM = 1; // Magnification ratio setting
    let _ZOOM_X = 2; // Text width magnification (only affects width, not height)
    let charWidth = 6;  //Width

    //---------------------------
    // Description: write command
    // Parameters:  cmd: command
    // Return:      void
    //---------------------------
    //% blockId=bmduino_commandForOLED
    function commandForOLED(cmd: number) {
        let buf = pins.createBuffer(2);
        buf[0] = 0x00;
        buf[1] = cmd;
        pins.i2cWriteBuffer(chipAdress, buf, false);
    }

    commandForOLED(0xd6); // Set zoom in command
    commandForOLED(_ZOOM);

    /**
    * Clear the OLED display
    */
    //% blockId=bmduino_clearDisplayOLED block="clear display"
    //% subcategory="OLED"
    //% weight=10
    export function clearDisplayOLED() {
        loadStarted = false;
        loadPercent = 0;
        commandForOLED(SSD1306_SETCOLUMNADRESS);
        commandForOLED(0x00);
        commandForOLED(displayWidth - 1);
        commandForOLED(SSD1306_SETPAGEADRESS);
        commandForOLED(0x00);
        commandForOLED(displayHeight - 1);
        let data = pins.createBuffer(17);
        data[0] = 0x40; // Data Mode
        for (let i = 1; i < 17; i++) {
            data[i] = 0x00;
        }
        // send display buffer in 16 byte chunks
        for (let i = 0; i < screenSize; i += 16) {
            pins.i2cWriteBuffer(chipAdress, data, false);
        }
        charX = xOffset;
        charY = yOffset;
        charX_lastPlace = charX;
        charY_lastPlace = charY;
    }

    /**
    * Write string with newLine
    * @param str The string on FontTable
    */
    function writeStringNewLineOLED(str: string) {
        writeStringOLED(str);
        newLineOLED();
    }

    /**
    * Write number with newLine
    * @param n The Num on FontTable
    */
    function writeNumNewLineOLED(n: number) {
        writeNumOLED(n);
        newLineOLED();
    }


    /**
    * Write string without newLine
    * @param str The string on FontTable
    */
    //% blockId=bmduino_writeStringOLED block="write string $str"
    //% subcategory="OLED"
    //% weight=70
    export function writeStringOLED(str: string) {
        for (let i = 0; i < str.length; i++) {
            if (charX > displayWidth - (charWidth * _ZOOM_X)) {
                newLineOLED();
            }
            drawCharOLED(charX, charY, str.charAt(i));
            charX += charWidth * _ZOOM_X; // Width adjusted according to _ZOOM_X
        }
    }

    /**
    * Write number without newLine
    * @param n The Num on FontTable
    */
    //% blockId=bmduino_writeNumOLED block="write number $n"
    //% subcategory="OLED"
    //% weight=60
    export function writeNumOLED(n: number) {
        let n_tep = Math.floor(n * 100) / 100;  //Keep 2 decimal places
        let numString = n_tep.toString();
        writeStringOLED(numString);
    }

    /**
    * NewLine
    */
    //% blockId=bmduino_newLineOLED block="line breaks"
    //% subcategory="OLED"
    //% weight=50
    export function newLineOLED() {
        charY++;
        charX = xOffset;
    }

    /**
    * Write string at specific locations
    * @param placeY Line
    * @param placeX Row
    * @param str The string on FontTable
    */
    //% blockId=bmduino_writeStringPlaceOLED block="line %placeY, character %placeX, write string $str"
    //% placeX.defl=1
    //% placeY.min=1 placeY.max=4 placeY.defl=1
    //% subcategory="OLED"
    //% weight=40
    export function writeStringPlaceOLED(placeY: number, placeX: number, str: string) {
        let charY_place = placeY - 1;                         //Line : charY_place
        let charX_place = charWidth * _ZOOM_X * (placeX - 1); //Row : charX_place
        charX_lastPlace = charX_place;
        charY_lastPlace = charY_place;
        for (let i = 0; i < str.length; i++) {
            if (charX_place > displayWidth - (charWidth * _ZOOM_X)) {
                break;
            }
            drawCharOLED(charX_place, charY_place, str.charAt(i));
            charX_place += charWidth * _ZOOM_X; // Width adjusted according to _ZOOM_X
        }
        charX = charX_place;
        charY = charY_place;
    }

    /**
    * Write number at specific locations
    * @param placeY Line
    * @param placeX Row
    * @param n The Num on FontTable
    */
    //% blockId=bmduino_writeNumPlaceOLED block="line %placeY, character %placeX, write number $n"
    //% subcategory="OLED"
    //% placeX.defl=1
    //% placeY.min=1 placeY.max=4 placeY.defl=1
    //% weight=30
    export function writeNumPlaceOLED(placeY: number, placeX: number, n: number) {
        let n_tep = Math.floor(n * 100) / 100;  //Keep 2 decimal places
        let numString = n_tep.toString();
        //writeStringPlaceOLED(placeY, placeX, numString);
        let charY_place = placeY - 1;                         //Line : charY_place
        let charX_place = charWidth * _ZOOM_X * (placeX - 1); //Row : charX_place
        for (let i = 0; i < numString.length; i++) {
            if (charX_place > displayWidth - (charWidth * _ZOOM_X)) {
                break;
            }
            drawCharOLED(charX_place, charY_place, numString.charAt(i));
            charX_place += charWidth * _ZOOM_X; // Width adjusted according to _ZOOM_X
        }
        charX = charX_place;
        charY = charY_place;
        for (let i = 0; i < Math.floor(displayWidth / (charWidth * _ZOOM_X)); i++) { //Clear the back position
            if (charX_place > displayWidth - (charWidth * _ZOOM_X)) {         //Out of display range, break
                break;
            }
            if ((charX_place == charX_lastPlace) && (charY_place == charY_lastPlace)) { //Encountered existing characters, break
                basic.showIcon(IconNames.Yes);
                break;
            }
            drawCharOLED(charX_place, charY_place, " ");
            charX_place += charWidth * _ZOOM_X; // Width adjusted according to _ZOOM_X
        }
    }

    //---------------------------
    // Description: drawChar
    // Parameters:  x: Column of display
    //              y: Row of display
    //              c : The char on FontTable.
    // Return:      void
    //---------------------------
    //% blockId=bmduino_drawCharOLED
    function drawCharOLED(x: number, y: number, c: string) {
        commandForOLED(SSD1306_SETCOLUMNADRESS);
        commandForOLED(x);
        commandForOLED(x + charWidth * _ZOOM_X);
        commandForOLED(SSD1306_SETPAGEADRESS);
        commandForOLED(y);
        commandForOLED(y);

        let line = pins.createBuffer(1 + 5 * _ZOOM_X);
        line[0] = 0x40;

        for (let i = 0; i < 5; i++) {
            let charIndex = c.charCodeAt(0);
            let data = font.getNumber(NumberFormat.UInt8BE, 5 * charIndex + i); // Obtain raw character data
            for (let z = 0; z < _ZOOM_X; z++) {
                line[1 + i * _ZOOM_X + z] = data; // Repeatedly writing the same data ->horizontally enlarging
            }
        }

        pins.i2cWriteBuffer(chipAdress, line, false);
    }

    /**
    * Initialize OLED module
    * @param width Display width in pixels
    * @param height Display height in pixels
    */
    //% blockId=bmduino_initOLEDModule block="initialize OLED module, width $width height $height"
    //% width.defl=128
    //% height.defl=64
    //% subcategory="OLED"
    //% weight=100
    export function initOLEDModule(width: number, height: number) {
        commandForOLED(SSD1306_DISPLAYOFF);
        commandForOLED(SSD1306_SETDISPLAYCLOCKDIV);
        commandForOLED(0x80);                                  // the suggested ratio 0x80
        commandForOLED(SSD1306_SETMULTIPLEX);
        commandForOLED(0x3F);
        commandForOLED(SSD1306_SETDISPLAYOFFSET);
        commandForOLED(0x0);                                   // no offset
        commandForOLED(SSD1306_SETSTARTLINE | 0x0);            // line #0
        commandForOLED(SSD1306_CHARGEPUMP);
        commandForOLED(0x14);
        commandForOLED(SSD1306_MEMORYMODE);
        commandForOLED(0x00);                                  // 0x0 act like ks0108
        commandForOLED(SSD1306_SEGREMAP | 0x1);
        commandForOLED(SSD1306_COMSCANDEC);
        commandForOLED(SSD1306_SETCOMPINS);
        commandForOLED(0x12);
        commandForOLED(SSD1306_SETCONTRAST);
        commandForOLED(0xCF);
        commandForOLED(SSD1306_SETPRECHARGE);
        commandForOLED(0xF1);
        commandForOLED(SSD1306_SETVCOMDETECT);
        commandForOLED(0x40);
        commandForOLED(SSD1306_DISPLAYALLON_RESUME);
        commandForOLED(SSD1306_NORMALDISPLAY);
        commandForOLED(SSD1306_DISPLAYON);
        displayWidth = width
        displayHeight = height / 8
        screenSize = displayWidth * displayHeight
        charX = xOffset
        charY = yOffset
        font = hex`
    0000000000
    3E5B4F5B3E
    3E6B4F6B3E
    1C3E7C3E1C
    183C7E3C18
    1C577D571C
    1C5E7F5E1C
    00183C1800
    FFE7C3E7FF
    0018241800
    FFE7DBE7FF
    30483A060E
    2629792926
    407F050507
    407F05253F
    5A3CE73C5A
    7F3E1C1C08
    081C1C3E7F
    14227F2214
    5F5F005F5F
    06097F017F
    006689956A
    6060606060
    94A2FFA294
    08047E0408
    10207E2010
    08082A1C08
    081C2A0808
    1E10101010
    0C1E0C1E0C
    30383E3830
    060E3E0E06
    0000000000
    00005F0000
    0007000700
    147F147F14
    242A7F2A12
    2313086462
    3649562050
    0008070300
    001C224100
    0041221C00
    2A1C7F1C2A
    08083E0808
    0080703000
    0808080808
    0000606000
    2010080402
    3E5149453E
    00427F4000
    7249494946
    2141494D33
    1814127F10
    2745454539
    3C4A494931
    4121110907
    3649494936
    464949291E
    0000140000
    0040340000
    0008142241
    1414141414
    0041221408
    0201590906
    3E415D594E
    7C1211127C
    7F49494936
    3E41414122
    7F4141413E
    7F49494941
    7F09090901
    3E41415173
    7F0808087F
    00417F4100
    2040413F01
    7F08142241
    7F40404040
    7F021C027F
    7F0408107F
    3E4141413E
    7F09090906
    3E4151215E
    7F09192946
    2649494932
    03017F0103
    3F4040403F
    1F2040201F
    3F4038403F
    6314081463
    0304780403
    6159494D43
    007F414141
    0204081020
    004141417F
    0402010204
    4040404040
    0003070800
    2054547840
    7F28444438
    3844444428
    384444287F
    3854545418
    00087E0902
    18A4A49C78
    7F08040478
    00447D4000
    2040403D00
    7F10284400
    00417F4000
    7C04780478
    7C08040478
    3844444438
    FC18242418
    18242418FC
    7C08040408
    4854545424
    04043F4424
    3C4040207C
    1C2040201C
    3C4030403C
    4428102844
    4C9090907C
    4464544C44
    0008364100
    0000770000
    0041360800
    0201020402
    3C2623263C
    1EA1A16112
    3A4040207A
    3854545559
    2155557941
    2154547841
    2155547840
    2054557940
    0C1E527212
    3955555559
    3954545459
    3955545458
    0000457C41
    0002457D42
    0001457C40
    F0292429F0
    F0282528F0
    7C54554500
    2054547C54
    7C0A097F49
    3249494932
    3248484832
    324A484830
    3A4141217A
    3A42402078
    009DA0A07D
    3944444439
    3D4040403D
    3C24FF2424
    487E494366
    2B2FFC2F2B
    FF0929F620
    C0887E0903
    2054547941
    0000447D41
    3048484A32
    384040227A
    007A0A0A72
    7D0D19317D
    2629292F28
    2629292926
    30484D4020
    3808080808
    0808080838
    2F10C8ACBA
    2F102834FA
    00007B0000
    08142A1422
    22142A1408
    AA005500AA
    AA55AA55AA
    000000FF00
    101010FF00
    141414FF00
    1010FF00FF
    1010F010F0
    141414FC00
    1414F700FF
    0000FF00FF
    1414F404FC
    141417101F
    10101F101F
    1414141F00
    101010F000
    0000001F10
    1010101F10
    101010F010
    000000FF10
    1010101010
    101010FF10
    000000FF14
    0000FF00FF
    00001F1017
    0000FC04F4
    1414171017
    1414F404F4
    0000FF00F7
    1414141414
    1414F700F7
    1414141714
    10101F101F
    141414F414
    1010F010F0
    00001F101F
    0000001F14
    000000FC14
    0000F010F0
    1010FF10FF
    141414FF14
    1010101F00
    000000F010
    FFFFFFFFFF
    F0F0F0F0F0
    FFFFFF0000
    000000FFFF
    0F0F0F0F0F
    3844443844
    7C2A2A3E14
    7E02020606
    027E027E02
    6355494163
    3844443C04
    407E201E20
    06027E0202
    99A5E7A599
    1C2A492A1C
    4C7201724C
    304A4D4D30
    3048784830
    BC625A463D
    3E49494900
    7E0101017E
    2A2A2A2A2A
    44445F4444
    40514A4440
    40444A5140
    0000FF0103
    E080FF0000
    08086B6B08
    3612362436
    060F090F06
    0000181800
    0000101000
    3040FF0101
    001F01011E
    00191D1712
    003C3C3C3C
    0000000000`
        loadStarted = false
        loadPercent = 0
        clearDisplayOLED()
    }
}
