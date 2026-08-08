#include "dashboard.h"
#include "commissioning.h"
#include "baseline_learner.h"
#include "telemetry.h"
#include "logger.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "dashboard";
static httpd_handle_t s_server;

static const char s_page[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Sentinel Monitor</title><style>"
"body{font:16px system-ui,sans-serif;background:#0f172a;color:#e5e7eb;margin:0;padding:20px}"
"main{max-width:760px;margin:auto}h1{margin-top:0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}"
".card{background:#1e293b;border-radius:12px;padding:16px;box-shadow:0 2px 8px #0005}.label{color:#94a3b8;font-size:.85rem}.value{font-size:1.8rem;font-weight:700;margin-top:6px}"
".unit{font-size:.9rem;color:#94a3b8;font-weight:400}.phase{color:#38bdf8}.good{color:#4ade80}.warn{color:#facc15}.bad{color:#f87171}"
"button{padding:11px 16px;margin:12px 6px 0 0;border:0;border-radius:7px;font-weight:600;cursor:pointer}"
".start{background:#22c55e;color:#052e16}.stop{background:#f59e0b;color:#422006}.message{min-height:24px;color:#cbd5e1;margin:12px 0}"
"small{color:#94a3b8}</style></head><body><main><h1>Sentinel Monitor</h1>"
"<div class=message id=message>Connecting...</div><div class=grid>"
"<div class=card><div class=label>Machine phase</div><div class='value phase' id=phase>--</div></div>"
"<div class=card><div class=label>Vibration RMS</div><div class=value><span id=rms>--</span> <span class=unit>g</span></div></div>"
"<div class=card><div class=label>Vibration peak</div><div class=value><span id=peak>--</span> <span class=unit>g</span></div></div>"
"<div class=card><div class=label>Temperature</div><div class=value><span id=temp>--</span> <span class=unit>°C</span></div></div>"
"<div class=card><div class=label>Anomaly</div><div class=value id=anomaly>--</div></div>"
"<div class=card><div class=label>Baseline</div><div class=value id=baseline>--</div><small>Cycles: <span id=cycles>--</span>/10</small></div>"
"</div><button class=start onclick=send('START_BASELINE')>Start baseline</button>"
"<button class=stop onclick=send('STOP_BASELINE')>Stop</button><div><small>Updated: <span id=updated>--</span></small></div>"
"<script>const $=id=>document.getElementById(id);function setText(id,v){$(id).textContent=v}"
"async function refresh(){try{const r=await fetch('/api/status');if(!r.ok)throw Error(r.status);const s=await r.json();"
"setText('phase',s.phase||'--');setText('rms',Number(s.rms||0).toFixed(3));setText('peak',Number(s.peak||0).toFixed(3));"
"setText('temp',s.temperature_valid?Number(s.temperature||0).toFixed(1):'invalid');setText('anomaly',s.anomaly||'NONE');"
"setText('baseline',s.baseline_valid?'Valid':(s.learning?'Learning':'Not available'));setText('cycles',s.cycles||0);"
"setText('updated',new Date().toLocaleTimeString());$('message').textContent='Connected';$('message').className='message good';"
"}catch(e){$('message').textContent='Dashboard connection error';$('message').className='message bad'}}"
"async function send(command){$('message').textContent='Sending '+command+'...';try{const r=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'text/plain'},body:command});if(!r.ok)throw Error(r.status);await refresh()}catch(e){$('message').textContent='Command failed';$('message').className='message bad'}}"
"setInterval(refresh,1000);refresh()</script></main></body></html>";

static esp_err_t root_get(httpd_req_t *req) { httpd_resp_set_type(req,"text/html"); return httpd_resp_send(req,s_page,HTTPD_RESP_USE_STRLEN); }

static esp_err_t status_get(httpd_req_t *req)
{
    commissioning_status_t b;
    commissioning_get_status(&b);
    telemetry_snapshot_t s = {0};
    const bool has_telemetry = telemetry_get_latest(&s);
    char out[384];
    if (!has_telemetry) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ready\":false,\"message\":\"waiting for telemetry\"}");
    }
    snprintf(out,sizeof(out),"{\"phase\":\"%s\",\"rms\":%.4f,\"peak\":%.4f,\"temperature\":%.2f,\"temperature_valid\":%s,\"anomaly\":\"%s\",\"baseline_valid\":%s,\"learning\":%s,\"cycles\":%u}",
      operating_state_name(s.phase),s.vibration_rms_g,s.vibration_peak_g,s.temperature_c,s.temperature_valid?"true":"false",anomaly_detector_name(s.anomaly),b.baseline_valid?"true":"false",b.learning?"true":"false",b.cycles);
    httpd_resp_set_type(req,"application/json"); return httpd_resp_send(req,out,HTTPD_RESP_USE_STRLEN);
}

static esp_err_t history_get(httpd_req_t *req)
{
    char out[8192]; size_t used=telemetry_history_json(out,sizeof(out)); httpd_resp_set_type(req,"application/json"); return httpd_resp_send(req,out,used);
}

static esp_err_t command_post(httpd_req_t *req)
{
    char command[COMMISSIONING_COMMAND_MAX]; int n=httpd_req_recv(req,command,sizeof(command)-1); if(n<=0){httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"command required");return ESP_FAIL;} command[n]='\0';
    esp_err_t err=commissioning_dispatch_command(command); if(err!=ESP_OK){httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"invalid command");return err;} return httpd_resp_sendstr(req,"{\"accepted\":true}");
}

static esp_err_t start_server(void)
{
    httpd_config_t config=HTTPD_DEFAULT_CONFIG(); esp_err_t err=httpd_start(&s_server,&config); if(err!=ESP_OK)return err;
    const httpd_uri_t root={.uri="/",.method=HTTP_GET,.handler=root_get}; const httpd_uri_t status={.uri="/api/status",.method=HTTP_GET,.handler=status_get}; const httpd_uri_t history={.uri="/api/history",.method=HTTP_GET,.handler=history_get}; const httpd_uri_t command={.uri="/api/command",.method=HTTP_POST,.handler=command_post};
    httpd_register_uri_handler(s_server,&root); httpd_register_uri_handler(s_server,&status); httpd_register_uri_handler(s_server,&history); httpd_register_uri_handler(s_server,&command); return ESP_OK;
}

esp_err_t dashboard_start(void)
{
    esp_err_t err=nvs_flash_init(); if(err!=ESP_OK&&err!=ESP_ERR_INVALID_STATE)return err; err=esp_netif_init(); if(err!=ESP_OK&&err!=ESP_ERR_INVALID_STATE)return err; err=esp_event_loop_create_default(); if(err!=ESP_OK&&err!=ESP_ERR_INVALID_STATE)return err; esp_netif_create_default_wifi_ap(); wifi_init_config_t wc=WIFI_INIT_CONFIG_DEFAULT(); err=esp_wifi_init(&wc); if(err!=ESP_OK&&err!=ESP_ERR_INVALID_STATE)return err;
    wifi_config_t ap={0}; strcpy((char*)ap.ap.ssid,"Sentinel-Setup"); strcpy((char*)ap.ap.password,"Sentinel1234"); ap.ap.ssid_len=14; ap.ap.channel=1; ap.ap.max_connection=2; ap.ap.authmode=WIFI_AUTH_WPA2_PSK; err=esp_wifi_set_mode(WIFI_MODE_AP); if(err==ESP_OK)err=esp_wifi_set_config(WIFI_IF_AP,&ap); if(err==ESP_OK)err=esp_wifi_start(); if(err!=ESP_OK&&err!=ESP_ERR_INVALID_STATE)return err; err=start_server(); if(err==ESP_OK)ESP_LOGI(TAG,"dashboard ready at http://192.168.4.1"); return err;
}
