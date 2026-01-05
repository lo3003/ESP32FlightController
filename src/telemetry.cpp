#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "telemetry.h"
#include "setup_wizard.h"
#include "radio.h" 

const char* ssid = "Drone_Setup_WiFi";
const char* password = "password123";

DroneState* drone_data;
AsyncWebServer server(80);

// --- CODE HTML/JS STOCKÉ EN FLASH ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone Configurator</title>
  <style>
    body { font-family: 'Segoe UI', sans-serif; text-align: center; background-color: #1a1a1a; color: #eee; margin: 0; padding: 20px; }
    h2 { color: #00d2ff; }
    .card { background: #333; padding: 15px; margin: 10px auto; border-radius: 8px; max-width: 400px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    .btn { display: inline-block; padding: 12px 24px; font-size: 16px; cursor: pointer; text-decoration: none; outline: none; color: #fff; background-color: #007bff; border: none; border-radius: 5px; width: 100%; margin-top: 10px; }
    .btn:active { background-color: #0056b3; }
    .btn-red { background-color: #dc3545; }
    .btn-green { background-color: #28a745; }
    
    .stick-container { display: flex; justify-content: space-between; margin-bottom: 5px; }
    .bar-bg { width: 70%; background: #555; height: 20px; border-radius: 10px; overflow: hidden; position: relative; }
    .bar-fill { height: 100%; background: #00d2ff; width: 50%; transition: width 0.1s; }
    .val-text { width: 25%; text-align: right; font-family: monospace; }
  </style>
</head>
<body>

  <h2>ESP32 Drone Setup</h2>

  <div class="card">
    <h3>Radio Monitor</h3>
    <div id="channels">
      Chargement...
    </div>
  </div>

  <div class="card">
    <h3>Calibration</h3>
    
    <p>1. Mettre les sticks au <strong>CENTRE</strong></p>
    <button class="btn" onclick="cmd('center')">Sauver Centres</button>
    
    <hr style="border-color:#444">
    
    <p>2. Tourner dans les <strong>COINS</strong></p>
    <button id="btn-rec" class="btn btn-green" onclick="toggleRec()">Démarrer Scan Min/Max</button>
    
    <hr style="border-color:#444">
    
    <p>3. Finaliser</p>
    <button class="btn btn-red" onclick="cmd('save')">Sauvegarder & Quitter</button>
  </div>

  <div class="card">
    <h3>Télémétrie Vol</h3>
    <div>Roll: <span id="roll">0</span>°</div>
    <div>Pitch: <span id="pitch">0</span>°</div>
  </div>

<script>
let recording = false;

function updateUI() {
  fetch('/api/data').then(res => res.json()).then(data => {
    // Radio
    let html = "";
    let labels = ["Roll", "Pitch", "Thr", "Yaw"];
    let vals = [data.r1, data.r2, data.r3, data.r4];
    
    for(let i=0; i<4; i++) {
      let pct = ((vals[i] - 1000) / 1000) * 100;
      if(pct < 0) pct = 0; if(pct > 100) pct = 100;
      
      html += `<div class="stick-container">
                 <span>${labels[i]}</span>
                 <div class="bar-bg"><div class="bar-fill" style="width:${pct}%"></div></div>
                 <span class="val-text">${vals[i]}</span>
               </div>`;
    }
    document.getElementById("channels").innerHTML = html;

    // Angles
    document.getElementById("roll").innerText = data.ang_r.toFixed(1);
    document.getElementById("pitch").innerText = data.ang_p.toFixed(1);
  });
}

function cmd(action) {
  fetch('/api/cmd?act=' + action).then(res => res.text()).then(txt => alert(txt));
}

function toggleRec() {
  recording = !recording;
  let btn = document.getElementById("btn-rec");
  if(recording) {
    btn.innerText = "STOP Scan";
    btn.className = "btn btn-red";
    fetch('/api/cmd?act=rec_start');
  } else {
    btn.innerText = "Démarrer Scan Min/Max";
    btn.className = "btn btn-green";
    fetch('/api/cmd?act=rec_stop');
  }
}

// Boucle principale JS (10Hz)
setInterval(updateUI, 100);

</script>
</body>
</html>
)rawliteral";

// --- TACHE WIFI (CORE 0) ---
void telemetryTask(void * parameter) {
    
    // Init Wi-Fi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());

    // Init Wizard
    setup_wizard_init();

    // 1. Page Web
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    // 2. API Data (JSON)
    server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
        // Construction JSON manuel (plus rapide/léger)
        String json = "{";
        json += "\"r1\":" + String(raw_channel_1) + ",";
        json += "\"r2\":" + String(raw_channel_2) + ",";
        json += "\"r3\":" + String(raw_channel_3) + ",";
        json += "\"r4\":" + String(raw_channel_4) + ",";
        json += "\"ang_r\":" + String(drone_data->angle_roll) + ",";
        json += "\"ang_p\":" + String(drone_data->angle_pitch);
        json += "}";
        request->send(200, "application/json", json);
    });

    // 3. API Commandes
    server.on("/api/cmd", HTTP_GET, [](AsyncWebServerRequest *request){
        String msg = "Erreur";
        if (request->hasParam("act")) {
            String act = request->getParam("act")->value();
            
            if(act == "center") {
                step_save_center();
                msg = "Centres Enregistres !";
            }
            else if(act == "rec_start") {
                step_set_recording_minmax(true);
                msg = "Scan ON";
            }
            else if(act == "rec_stop") {
                step_set_recording_minmax(false);
                msg = "Scan OFF";
            }
            else if(act == "save") {
                step_save_eeprom();
                msg = "Sauvegarde EEPROM OK ! Redemarrage...";
                // Optionnel: ESP.restart() après un délai
            }
        }
        request->send(200, "text/plain", msg);
    });

    server.begin();

    // Boucle infinie pour garder la tâche en vie
    for(;;) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void start_telemetry_task(DroneState* drone_ptr) {
    drone_data = drone_ptr;
    xTaskCreatePinnedToCore(
        telemetryTask,
        "WifiTask",
        10000,    // Stack
        NULL,
        1,        // Priorité basse
        NULL,
        0         // CORE 0
    );
}