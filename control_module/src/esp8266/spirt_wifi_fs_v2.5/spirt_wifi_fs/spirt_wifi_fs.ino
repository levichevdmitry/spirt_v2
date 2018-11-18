#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266SSDP.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>
#include <DNSServer.h>
#include <FS.h>
#define MAX_MSG_LEN         256
#define VERSION             "{\"websrvver\":\"0.2.5\"}"

//holds the current upload
File fsUploadFile;

//String statusDesc = "Testing status";
char buffNum = 0;
char workNum = 1;
String inputJSONString[2];
unsigned long previousMillis = 0;
const long interval = 3000;
//String inputBuffer = "";

const char* www_username = "admin";
const char* www_password = "esp8266";
//**********************************************************

// Access point name for WiFi manager
const char *apName = "Green Snake";

const char* serverIndex = "<html lang=\"ru\">"\
                          "<head><title>Green Snake</title>"\
                          "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\"/></head>"\
                          "Перед обновлением ПО зайдите в меню \"Настройки\" на контроллере!!!<br>"\
                          "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><br><input type='submit' value='Update'></form>"\
                          "</html>";

ESP8266WebServer server(80);

//format bytes
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

String getContentType(String filename) {
  if (server.hasArg("download")) return "application/octet-stream";
  else if (filename.endsWith(".htm")) return "text/html";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  else if (filename.endsWith(".pdf")) return "application/x-pdf";
  else if (filename.endsWith(".zip")) return "application/x-zip";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  else if (filename.endsWith(".svg")) return "image/svg+xml";
  return "text/plain";
}

bool handleFileRead(String path) {
  //DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload() {
  if (server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    //DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    //DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile)
      fsUploadFile.close();
    //DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
}

void handleFileDelete() {
  if (server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  //DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
  if (path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if (!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate() {
  if (server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  //DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
  if (path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if (SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if (file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if (!server.hasArg("dir")) {
    server.send(500, "text/plain", "BAD ARGS");
    return;
  }

  String path = server.arg("dir");
  //DBG_OUTPUT_PORT.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while (dir.next()) {
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }

  output += "]";
  server.send(200, "text/json", output);
}

void handleData() {
  server.send(200, "text/html", inputJSONString[workNum]);
}

void handleVersion() {
  server.send(200, "text/html", VERSION);
}

void HTTP_init() {

  server.on( "/data.dat", handleData );
  server.on( "/version.dat", handleVersion );
  server.on("/description.xml", HTTP_GET, []() {
    SSDP.schema(server.client());
  });
  server.on("/updform", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/html", serverIndex);
  });
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.setDebugOutput(true);
      WiFiUDP::stopAll();
      Serial.printf("Update: %s\n", upload.filename.c_str());
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
      if (!Update.begin(maxSketchSpace)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
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
  server.on("/edit", HTTP_GET, []() {
    /*if(!server.authenticate(www_username, www_password)){
      Serial.println("Auth request...");
      return server.requestAuthentication();
      }*/
    if (!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, []() {
    server.send(200, "text/plain", "");
  }, handleFileUpload);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([]() {
    if (!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
  }
}

void initSSDP() {
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
  switch (event) {
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
  //inputBuffer.reserve(MAX_MSG_LEN);
  inputJSONString[0].reserve(MAX_MSG_LEN);
  inputJSONString[1].reserve(MAX_MSG_LEN);
  Serial.begin(115200);
  // starting WiFi manager пїЅ Captive portal
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();
  wifiManager.setTimeout(45); // 45 seconds
  if (!wifiManager.autoConnect(apName)) {
    delay(500);
  }

  HTTP_init();
  WiFi.onEvent(WiFiEvent);
  initSSDP();
  //-----------------------
  Serial.print("IP:");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      Serial.print("IP:");
      Serial.println(WiFi.localIP());
    }
  }
  readParams();
}

// decode information from main mcu

void readParams() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputJSONString[buffNum] += inChar;
    if (inChar == '}') {
      workNum = buffNum;
      if (buffNum) {
        buffNum = 0;
      } else {
        buffNum = 1;
      }
      inputJSONString[buffNum].remove(0);
      //Serial.println(inputJSONString[workNum].length());
      //Serial.println(workNum, DEC);
    }
  }
}
