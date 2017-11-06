#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266SSDP.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>
#include <DNSServer.h>
#include <HashMap.h>
#include <FS.h>
#define MAX_MSG_LEN         100

//define the max size of the hashtable
const byte HASH_SIZE = 8; 
//storage 
HashType<String,float> hashRawArray[HASH_SIZE]; 
//handles the storage [search,retrieve,insert]
HashMap<String, float> hashMap = HashMap<String, float>( hashRawArray , HASH_SIZE );
//holds the current upload
File fsUploadFile;

String statusDesc = "Testing status";
String inputString = "";

const char* www_username = "admin";
const char* www_password = "esp8266";
//**********************************************************

// Access point name for WiFi manager
const char *apName = "Green Snake";

const char* serverIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><br><input type='submit' value='Update'></form>";

ESP8266WebServer server(80);

int wtable[64] = {
  0x0402, 0x0403, 0x201A, 0x0453, 0x201E, 0x2026, 0x2020, 0x2021,
  0x20AC, 0x2030, 0x0409, 0x2039, 0x040A, 0x040C, 0x040B, 0x040F,
  0x0452, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014,
  0x007F, 0x2122, 0x0459, 0x203A, 0x045A, 0x045C, 0x045B, 0x045F,
  0x00A0, 0x040E, 0x045E, 0x0408, 0x00A4, 0x0490, 0x00A6, 0x00A7,
  0x0401, 0x00A9, 0x0404, 0x00AB, 0x00AC, 0x00AD, 0x00AE, 0x0407,
  0x00B0, 0x00B1, 0x0406, 0x0456, 0x0491, 0x00B5, 0x00B6, 0x00B7,
  0x0451, 0x2116, 0x0454, 0x00BB, 0x0458, 0x0405, 0x0455, 0x0457};
  
int win1251_to_utf8(const char* text, char* utext)
{
  int wc;
  int i, j;
  if (!utext)
    return 0;
  i=0;
  j=0;
  while (i<strlen(text))
  {
    wc = (unsigned char)text[i++];
    // Windows-1251 to Unicode
    if (wc>=0x80) {
      if (wc<=0xBF) /* Ђ-ї */
      {
        wc = wtable[wc-0x80];
      }
      else if (wc>=0xC0 && wc<=0xDF) // А-Я
        wc = wc - 0xC0 + 0x0410;
      else if (wc>=0xE0) // а-я
        wc = wc - 0xE0 + 0x0430;
    }
    // Unicode to UTF-8
    // 0x00000000 — 0x0000007F -> 0xxxxxxx
    // 0x00000080 — 0x000007FF -> 110xxxxx 10xxxxxx
    // 0x00000800 — 0x0000FFFF -> 1110xxxx 10xxxxxx 10xxxxxx
    // 0x00010000 — 0x001FFFFF -> 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
    if (wc<0x80)
    {
      utext[j++] = (char)wc;
    }
    else if (wc<0x800)
    {
      utext[j++] = (char)((wc>>6)   | 0xC0);
      utext[j++] = (char)((wc&0x3F) | 0x80);
    }
    else if (wc<0x10000)
    {
      utext[j++] = (char)((wc>>12)  | 0xE0);
      utext[j++] = (char)((wc>>6)   | 0x80);
      utext[j++] = (char)((wc&0x3F) | 0x80);
    }
    else
    {
      utext[j++] = (char)((wc>>18)  | 0xF0);
      utext[j++] = (char)((wc>>12)  | 0x80);
      utext[j++] = (char)((wc>>6)   | 0x80);
      utext[j++] = (char)((wc&0x3F) | 0x80);
    }
  }
  utext[j] = 0x00;
  return 1;
}

//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  //DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload(){
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    //DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    //DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
}

void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  //DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  //DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  //DBG_OUTPUT_PORT.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}

void handleData(){
  String s = "{ \"status\":\"";
  s += statusDesc;
  s += "\"";
  
  for (char i = 0; i < HASH_SIZE; i++){
    s += "\n,\"";
    s += hashMap[i].getHash();
    s += "\":";
    s += hashMap[i].getValue();
    //s += "\"";
  }
  s += "}";
  server.send(200, "text/html", s);
}

void HTTP_init(){

   server.on( "/data.dat", handleData );
   server.on("/description.xml", HTTP_GET, [](){
      SSDP.schema(server.client());
    });         
   server.on("/updform", HTTP_GET, [](){
      server.sendHeader("Connection", "close");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/html", serverIndex);
      });
   server.on("/update", HTTP_POST, [](){
      server.sendHeader("Connection", "close");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
      ESP.restart();
      },[](){
      HTTPUpload& upload = server.upload();
      if(upload.status == UPLOAD_FILE_START){
         Serial.setDebugOutput(true);
         WiFiUDP::stopAll();
         Serial.printf("Update: %s\n", upload.filename.c_str());
         uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
         if(!Update.begin(maxSketchSpace)){//start with max available size
            Update.printError(Serial);
          }
       } else if(upload.status == UPLOAD_FILE_WRITE){
          if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
             Update.printError(Serial);
           }
       } else if(upload.status == UPLOAD_FILE_END){
           if(Update.end(true)){ //true to set the size to the current progress
              Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
           } else {
              Update.printError(Serial);
           }
              Serial.setDebugOutput(false);
       }
       yield();
    });
    //list directory
    server.on("/list", HTTP_GET, handleFileList);
    //load editor
    server.on("/edit", HTTP_GET, [](){
      /*if(!server.authenticate(www_username, www_password)){
        Serial.println("Auth request...");
        return server.requestAuthentication();
      }*/
      if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
    });
    //create file
    server.on("/edit", HTTP_PUT, handleFileCreate);
    //delete file
    server.on("/edit", HTTP_DELETE, handleFileDelete);
    //first callback is called after the request has ended with all parsed arguments
    //second callback handles file uploads at that location
    server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);

    //called when the url is not defined here
    //use it to load content from SPIFFS
    server.onNotFound([](){
      if(!handleFileRead(server.uri()))
        server.send(404, "text/plain", "FileNotFound");
    });

    if (WiFi.status() == WL_CONNECTED){
      server.begin(); 
    }        
}

void initSSDP(){
    //Serial.printf("Starting SSDP...\n");
    SSDP.setSchemaURL("description.xml");
    SSDP.setHTTPPort(80);
    SSDP.setName("Green Snake");
    SSDP.setSerialNumber("0000000000001");
    SSDP.setURL("index.html");
    SSDP.setModelName("Green Snake");
    SSDP.setModelNumber("000000000002");
   // SSDP.setModelURL("http://esp8266-arduinoide.ru/wifimanager/");
    SSDP.setManufacturer("Levichev Dmitry");
   // SSDP.setManufacturerURL("http://www.esp8266-arduinoide.ru");
    SSDP.setDeviceType("upnp:rootdevice");
    SSDP.begin();
}

void WiFiEvent(WiFiEvent_t event) {
    switch(event) {
        case WIFI_EVENT_STAMODE_GOT_IP:
            Serial.print("IP:");
            Serial.println(WiFi.localIP());                        
            server.begin();
            break;
            
        case WIFI_EVENT_STAMODE_DISCONNECTED:
            server.stop();
            break;
            
        case WIFI_EVENT_STAMODE_CONNECTED:
            server.stop();
            break;
    }
}

void setup() {
        delay(500);
        SPIFFS.begin();
        statusDesc.reserve(MAX_MSG_LEN);
        inputString.reserve(MAX_MSG_LEN);
        Serial.begin(115200);
        // starting WiFi manager пїЅ Captive portal
        WiFiManager wifiManager;
        //reset settings - for testing
        //wifiManager.resetSettings();
        wifiManager.setTimeout(45); // 45 seconds
        if(!wifiManager.autoConnect(apName)) {
          delay(500);
        } 
        
        HTTP_init();
        WiFi.onEvent(WiFiEvent);
        initSSDP(); 
        //-----------------------
        // init array
        hashMap[0]("tkub", 0.0);
        hashMap[1]("tkolonan", 10.0);
        hashMap[2]("tkolonav", 33.9);
        hashMap[3]("tcoolant", 45.7);
        hashMap[4]("fcoolant", 2.6);
        hashMap[5]("pkub", 23.8);
        hashMap[6]("power", 1.5);
        hashMap[7]("alcopdk", 1.0);
        Serial.print("IP:");
        Serial.println(WiFi.localIP());         
}

void loop() {
      if (WiFi.status() == WL_CONNECTED){
        server.handleClient();
      }
      readParams();
}

// decode information from main mcu

void readParams() {
  String key, val;
  char indexDelim;
  char buf[MAX_MSG_LEN];
    
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      //Serial.print(inputString);
      indexDelim = inputString.indexOf(':');
      key = inputString.substring(0, indexDelim);
      val = inputString.substring(indexDelim + 1);
      //Serial.println(key);
      //Serial.println(val);
      if (key == "status") {
        win1251_to_utf8(val.c_str(), buf); // convert from cp-1251 to utf-8
        statusDesc = String(buf);
      } else {
        hashMap.setValueOf(key, val.toFloat());
      }
      inputString = "";
    }
  }
}



