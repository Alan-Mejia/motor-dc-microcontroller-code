#include <Arduino.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <timer.h>


#define WIFI_SSID "HUAWEI-106V40"
#define WIFI_PASSWORD "NewPwds*/@12"

#define WEBSERVER_HOST "192.168.3.10"//IP address of backend server
#define WEBSERVER_PORT 8080 // Port of backend service
#define STOMP_SERVER_ENDPOINT "/iot-dc-motor-controller/"//the endpoint to subscribe to stomp server

#define JSON_DOCUMENT_SIZE 2040
#define DEVICE_NAME "alancho"
#define REACT_SERVER_NAME "react-ui"

#define EACH_REQUEST_CHANNEL "/iot-websocket/"
#define EACH_SUBSCRIPTION_PREFIX "/connection/"

#define CHECK_CONNECTION_CHANNEL "status/"
#define DEVICE_SYSTEM_INFO_CHANNEL "device-system-info/"
#define CHANGE_RPM_CHANNEL "change-rpms/"

int motorPinOutput = 5;
int maximumMotorRpm = 0; // "Here we have to specify the maximum number of revolution of the motor at max supported voltage"
int pwm = 0;
//int currentRpm = (pwm * maximumMotorRpm) / 255;

int encoderPinInput = 33;
int totalRpmEncoder = 0;
volatile int auxiliarEncoderCount = 0;
unsigned long previousMillis = 0;
long interval = 100; //Each .1 s we are going to read the RPMS 
int numberOfSlotsInEncoderDisk = 20;

/* Implementatio of PID control */
int setPoint = 0;
float controlSignal = 0.0;
float controlSignalMinus1 = 0.0;
float error = 0.0;
float errorMinus1 = 0.00001;
float errorMinus2 = 0.00001;
float kp = 0.0;
double ki = 0.0;
double kd = 0.0;
float tao = 0.1;


WebSocketsClient webSocketsClient;
uniuno:: Timer timer;


void connectToWebSocket();
void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length); // To recieve the paramets of any message in this callback
void subscribeToChannel(String channelName, String deviceListening);
void processJsonDataInMessageRecieved(String messageRecieved);
String extractObjectFromMessage(String messageRecieved);
void startControlSystem();
void sendMessage(String channelName, String payload, String deviceListening);
void setCountRpmEncoder();



void setup() {
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT); //To turn on blue led for wifi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED){// If we have not stablished a connection yet we are going to try to connect over and over again
    Serial.println("Trying to connect to wifi again...");
    delay(100);
  }

  Serial.println("Connected to wifi");

  pinMode(motorPinOutput, OUTPUT);
  pinMode(encoderPinInput, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinInput), setCountRpmEncoder, RISING);

  connectToWebSocket();
  timer.set_interval(startControlSystem, 1000); // We are going to send the information each 1 second
  timer.attach_to_loop();
  
}

void loop() {
  digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED); // To show the blue led when wifi connection is stablished
  unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis)>= interval){
      previousMillis = currentMillis;
      totalRpmEncoder = (auxiliarEncoderCount*600.0)/numberOfSlotsInEncoderDisk;
      auxiliarEncoderCount = 0;
    }
  
  error = setPoint - totalRpmEncoder;
  controlSignal = controlSignalMinus1 + ((kp + kd/tao)*error) + ((-kp + (ki*tao) - 2*kd/tao) * errorMinus1 ) + ((kd/tao)*errorMinus2);
  controlSignalMinus1 = controlSignal;
  errorMinus2 = errorMinus1;
  errorMinus1 = error;

  if (controlSignal >= maximumMotorRpm){
    controlSignal = maximumMotorRpm;
  }else if(controlSignal < 800.0){
    controlSignal = 800.0;
  }
  
  //Serial.printf("Señal de control %f \n",  controlSignal);

  pwm = maximumMotorRpm <= 0? 0 : int((controlSignal * 255)/maximumMotorRpm);
  analogWrite(motorPinOutput, pwm);
  //Serial.printf("Señal de pwm %f \n",  pwm);

  delay(160);

  webSocketsClient.loop();
  timer.tick();
}

void connectToWebSocket(){
  //Building the format of transport request URL http://host:port/myApp/myEndpoint/{server-id}/{session-id}/{transport}
  String urlFormat = STOMP_SERVER_ENDPOINT;
  urlFormat += random(0,999); //server id choosen by the client
  urlFormat += "/";
  urlFormat += random(0,999999); //session id this must be a unique value for all the clients
  urlFormat += "/websocket"; // To learn more about the format of URL request visit: "https://sockjs.github.io/sockjs-protocol/sockjs-protocol-0.3.3.html"

  webSocketsClient.begin(WEBSERVER_HOST, WEBSERVER_PORT, urlFormat);
  webSocketsClient.setExtraHeaders();
  webSocketsClient.onEvent(handleWebSocketEvent);



}

void handleWebSocketEvent(WStype_t type, uint8_t *payload, size_t length){
  switch (type){
  case WStype_DISCONNECTED:
    Serial.println("Desconnected from websocket...");
    break;
  case WStype_CONNECTED: // 2nd if the crendtial are correct then we response with a payload
    {
      Serial.printf("[Open session message *from server ]: %s \n", payload);  
    }
    break;
  case WStype_TEXT:
    {
      String text = (char*) payload;
      if (payload[0] == 'h'){// In case of heartbeat
        Serial.println("Heartbeat!");
      }else if(payload[0] == 'o'){// 1st we send the connection message
        String connectMessage = "[\"CONNECT\\naccept-version:1.1,1.0\\nheart-beat:1000,1000\\n\\n\\u0000\"]";
        webSocketsClient.sendTXT(connectMessage);
        delay(100);
      }else if (text.startsWith("a[\"CONNECTED")){ // Inmediately when we stablish the connection with server, we have to susbcribe to some channels to start to revieve messages
        String deviceId = String(DEVICE_NAME);
        subscribeToChannel(String(CHECK_CONNECTION_CHANNEL), deviceId); // To check if the dive is connected to wifi and its current status working/not working
        delay(500);
        subscribeToChannel("deviceinfo/", deviceId); // To check the current status of the system
        delay(500);
        subscribeToChannel(String(CHANGE_RPM_CHANNEL), deviceId); // To mange how many rpsm we want the motor works
        delay(500);

      }else if (text.startsWith("a[\"MESSAGE")){ // This block will be executed whenever we recieve a message from stomp server
        processJsonDataInMessageRecieved(text);
      }
    }
    break;  

  }
}

void subscribeToChannel(String channelName, String deviceListening){
  String subscribeMessage = "[\"SUBSCRIBE\\nid:sub-0\\ndestination:" + String(EACH_SUBSCRIPTION_PREFIX) + channelName + deviceListening + "\\n\\n\\u0000\"]";
  Serial.println("Subscripcion: " + subscribeMessage);
  webSocketsClient.sendTXT(subscribeMessage);
}

void processJsonDataInMessageRecieved(String messageRecieved){
  String jsonObject = extractObjectFromMessage(messageRecieved);
  jsonObject.replace("\\","");
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, jsonObject);

  JsonObject contentRecieved = doc.as<JsonObject>();

  // We will only attend the requests that como from UI
  if (strcmp(contentRecieved["from"], "react-ui-alancho") == 0 && strcmp(contentRecieved["to"], "alancho") == 0){
    // Execute the order recieved from ui
    Serial.println("DENTRO DE LA PRIMER CONDICIONAL");
    if(strcmp(contentRecieved["action"], "changeRpm") == 0){
      Serial.println("Dentro de change rpms");
      kp = double(contentRecieved["kp"]);
      Serial.println(kp);
      ki = double(contentRecieved["ki"]);
      kd = double(contentRecieved["kd"]);
      maximumMotorRpm = int(contentRecieved["maximumMotorRpm"]);
      setPoint = int(contentRecieved["rpmDesired"]);
      //pwm = maximumMotorRpm <= 0? 0 : int((int(contentRecieved["rpmDesired"]) * 255)/maximumMotorRpm);
    }


  }else{
    //Serial.println("Messaje recieved "+ jsonObject);
  }
  


}

String extractObjectFromMessage(String messageRecieved){
  char startingChar = '{';
  char finishingChar = '}';

  String tmpData = "";
  bool _flag = false;
  for (int i = 0; i < messageRecieved.length(); i++) {
    char tmpChar = messageRecieved[i];
    if (tmpChar == startingChar) {
      tmpData += startingChar;
      _flag = true;
    }
    else if (tmpChar == finishingChar) {
      tmpData += finishingChar;
      break;
    }
    else if (_flag == true) {
      tmpData += tmpChar;
    }
  }

  return tmpData;
}

//To send the information of the system to UI
void startControlSystem(){
    
    Serial.printf("Total RPM %i \n Pwm: %i \n Error: %f \n Control Sig: %f \n Control Sig-1: %f \n Error-2: %f \n Error-1: %f \n" , totalRpmEncoder, pwm, error, controlSignal, controlSignalMinus1, errorMinus2, errorMinus1);
    //analogWrite(motorPinOutput, pwm);
    
    String deviceId = String(DEVICE_NAME);
    String action = "systemInfo";
    String to = "react-ui-alancho";
    float voltageOutput = (pwm * 3.3)/255;

    String deviceInfoPayload = "{\\\"from\\\":\\\"" +
                                    deviceId + "\\\",\\\"to\\\":\\\"" +
                                    to + "\\\",\\\"action\\\":\\\"" +
                                    action + "\\\",\\\"pinVoltageOutput\\\":\\\"" +
                                    voltageOutput + "\\\",\\\"rpmWorking\\\":\\\"" +
                                    totalRpmEncoder + "\\\",\\\"maximumMotorRpm\\\":\\\"" +
                                    maximumMotorRpm + "\\\",\\\"kp\\\":\\\"" +
                                    kp + "\\\",\\\"ki\\\":\\\"" +
                                    ki + "\\\",\\\"kd\\\":\\\"" +
                                    kd + "\\\"}";
              
    sendMessage(String(DEVICE_SYSTEM_INFO_CHANNEL), deviceInfoPayload, String(DEVICE_NAME));                
}

void sendMessage(String channelName, String payload, String deviceListening){
  String message = "[\"SEND\\ndestination:" + String(EACH_REQUEST_CHANNEL) + channelName + deviceListening + "\\n\\n" + payload + "\\n\\n\\u0000\"]";
  webSocketsClient.sendTXT(message);
}

void setCountRpmEncoder(){
  auxiliarEncoderCount++;
}

