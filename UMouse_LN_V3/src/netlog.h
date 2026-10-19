#pragma once
#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include "settings.h"

// =============================================================
//  UMouse S3 — netlog.h
//  Log de diagnóstico = Serial + búfer circular en RAM (sobrevive a un reinicio por
//  software: ESP.restart() tras "ERROR NAV", pánico, watchdog). Con WIFI_LOG_ENABLE el
//  ESP32 levanta un punto de acceso con una página que muestra el log en vivo.
//
//  Hilos: logPrintf() se llama desde el lazo (núcleo 1); el servidor web corre en su propia
//  tarea en el núcleo 0 y solo LEE el búfer (copia bajo spinlock). El control no espera
//  nunca a la red.
//  ADC: el WiFi bloquea el ADC2 (GPIO11-20); los IR (GPIO1,2,8,10) y VBAT (GPIO4) son ADC1.
// =============================================================

#ifndef WIFI_LOG_ENABLE
  #define WIFI_LOG_ENABLE 0
#endif

static const uint32_t LOG_BUF_SIZE = 16384;
static const uint32_t LOG_MAGIC    = 0x554D4C47u;   // "UMLG"

struct NetLogMem {
  uint32_t magic;
  uint32_t total;          // bytes escritos desde que se creó el búfer (posición = total % tamaño)
  uint32_t boots;
  char     buf[LOG_BUF_SIZE];
};
// .noinit: el arranque NO la pone a cero => el log de la corrida anterior sigue ahí tras
// ESP.restart(). Tras un corte de alimentación su contenido es basura: lo descarta el magic.
static __NOINIT_ATTR NetLogMem g_log;
static portMUX_TYPE g_logMux = portMUX_INITIALIZER_UNLOCKED;

static void logWrite(const char* s, size_t n){
  if(n == 0) return;
  portENTER_CRITICAL(&g_logMux);
  for(size_t i = 0; i < n; i++){
    g_log.buf[g_log.total % LOG_BUF_SIZE] = s[i];
    g_log.total++;
  }
  portEXIT_CRITICAL(&g_logMux);
}

// Igual que Serial.printf, y además al búfer (línea de hasta 255 caracteres).
static void logPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void logPrintf(const char* fmt, ...){
  char line[256];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if(n < 0) return;
  if(n >= (int)sizeof(line)) n = sizeof(line) - 1;
  Serial.write((const uint8_t*)line, n);
  logWrite(line, n);
}
static inline void logPrintln(const char* s){ logPrintf("%s\n", s); }

static const char* resetReasonStr(esp_reset_reason_t r){
  switch(r){
    case ESP_RST_POWERON:  return "encendido";
    case ESP_RST_SW:       return "software (ESP.restart / BOOT tras error)";
    case ESP_RST_PANIC:    return "PANICO (fallo del programa)";
    case ESP_RST_INT_WDT:  return "WATCHDOG de interrupciones";
    case ESP_RST_TASK_WDT: return "WATCHDOG de tareas";
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT (caida de tension)";
    case ESP_RST_EXT:      return "pin externo (RST)";
    case ESP_RST_DEEPSLEEP:return "deep sleep";
    default:               return "desconocido";
  }
}

// Llamar al principio de setup(), después de Serial.begin().
static void logInit(){
  esp_reset_reason_t rr = esp_reset_reason();
  bool keep = (g_log.magic == LOG_MAGIC) && rr != ESP_RST_POWERON && rr != ESP_RST_BROWNOUT;
  if(!keep){ g_log.magic = LOG_MAGIC; g_log.total = 0; g_log.boots = 0; }
  g_log.boots++;
  logPrintf("\n==== ARRANQUE #%lu  motivo: %s ====\n", (unsigned long)g_log.boots, resetReasonStr(rr));
}

static void logClear(){
  portENTER_CRITICAL(&g_logMux);
  g_log.total = 0;
  portEXIT_CRITICAL(&g_logMux);
}

#if WIFI_LOG_ENABLE
#include <WiFi.h>
#include <WebServer.h>

static WebServer g_web(80);
static bool g_webOn = false;

static const char LOG_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UMouse Log</title>
<style>
:root{--bg:#111;--fg:#ddd;--mut:#888;--acc:#4ec9b0;--err:#f66;--warn:#fc6;--bar:#1c1c1c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.35 ui-monospace,Menlo,Consolas,monospace}
header{position:sticky;top:0;background:var(--bar);padding:8px 12px;display:flex;gap:8px;flex-wrap:wrap;align-items:center;border-bottom:1px solid #333}
header b{color:var(--acc)}button{background:#2a2a2a;color:var(--fg);border:1px solid #444;border-radius:6px;padding:6px 10px;font:inherit}
#st{color:var(--mut);margin-left:auto}#log{white-space:pre-wrap;word-break:break-all;padding:8px 12px 40px}
.e{color:var(--err)}.w{color:var(--warn)}.p{color:var(--acc)}.m{color:#9cf}
</style></head><body>
<header><b>UMouse S3</b><button id="pa">Pausar</button><button id="cp">Copiar todo</button><button id="dl">Descargar</button><button id="cl">Borrar log</button><span id="st">...</span></header>
<div id="log"></div>
<script>
let pos=0,paused=false,txt='',busy=false;const L=document.getElementById('log'),S=document.getElementById('st');
function cls(l){return /FALLO|ERROR|ATASCO|TIMEOUT|PANICO|BROWNOUT|WATCHDOG|inalcanzable/i.test(l)?'e':/AVISO|tope|timeout|perdido/i.test(l)?'w':/^\[POSE\]/.test(l)?'p':/^\[MAPA\]|^[+|]/.test(l)?'m':''}
function esc(s){return s.replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
let part='';
function add(t){txt+=t;const ls=(part+t).split('\n');part=ls.pop();const atEnd=innerHeight+scrollY>=document.body.scrollHeight-80;const f=document.createDocumentFragment();
 for(const l of ls){const d=document.createElement('div');d.className=cls(l);d.innerHTML=esc(l)||'&nbsp;';f.appendChild(d)}L.appendChild(f);if(atEnd)scrollTo(0,document.body.scrollHeight)}
async function poll(){if(paused||busy)return;busy=true;try{const r=await fetch('/log?from='+pos,{cache:'no-store'});const n=+r.headers.get('X-Next');const t=await r.text();
 if(n<pos){L.innerHTML='';txt='';part=''}if(t)add(t);pos=n;S.textContent='conectado · '+(txt.length/1024).toFixed(1)+' KB'}catch(e){S.textContent='sin conexión'}busy=false}
setInterval(poll,400);poll();
document.getElementById('pa').onclick=e=>{paused=!paused;e.target.textContent=paused?'Seguir':'Pausar'};
// Copiar: navigator.clipboard solo existe en HTTPS/localhost; en http://192.168.4.1 se usa execCommand.
function cpOld(){const t=document.createElement('textarea');t.value=txt;t.setAttribute('readonly','');t.style.cssText='position:fixed;top:0;left:0;opacity:0';document.body.appendChild(t);t.focus();t.select();t.setSelectionRange(0,txt.length);let ok=false;try{ok=document.execCommand('copy')}catch(e){}t.remove();return ok}
document.getElementById('cp').onclick=async e=>{const b=e.target;let ok=false;
 if(navigator.clipboard&&window.isSecureContext){try{await navigator.clipboard.writeText(txt);ok=true}catch(x){}}
 if(!ok)ok=cpOld();b.textContent=ok?'¡Copiado! ('+(txt.length/1024).toFixed(1)+' KB)':'No se pudo: usa Descargar';setTimeout(()=>b.textContent='Copiar todo',2000)};
document.getElementById('dl').onclick=()=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([txt],{type:'text/plain'}));a.download='umouse-log.txt';a.click()};
document.getElementById('cl').onclick=async()=>{if(!confirm('¿Borrar el log del robot?'))return;await fetch('/clear',{method:'POST'});L.innerHTML='';txt='';part='';pos=0};
</script></body></html>)HTML";

// /log?from=N -> texto desde la posición N (o desde lo más antiguo que quede). X-Next = nueva posición.
static void _webLog(){
  uint32_t from = g_web.hasArg("from") ? (uint32_t)strtoul(g_web.arg("from").c_str(), nullptr, 10) : 0;
  static char out[4096];
  uint32_t total, start, n;
  bool lost = false;
  portENTER_CRITICAL(&g_logMux);
  total = g_log.total;
  if(from > total) from = 0;                                  // el log se borró: empezar de nuevo
  uint32_t oldest = total > LOG_BUF_SIZE ? total - LOG_BUF_SIZE : 0;
  if(from < oldest){ from = oldest; lost = true; }
  start = from;
  n = total - start;
  if(n > sizeof(out) - 64) n = sizeof(out) - 64;
  for(uint32_t i = 0; i < n; i++) out[i] = g_log.buf[(start + i) % LOG_BUF_SIZE];
  portEXIT_CRITICAL(&g_logMux);
  String body;
  if(lost) body = "[... parte antigua del log sobrescrita ...]\n";
  body.concat(out, n);
  g_web.sendHeader("X-Next", String(start + n));
  g_web.sendHeader("Access-Control-Expose-Headers", "X-Next");
  g_web.send(200, "text/plain; charset=utf-8", body);
}

static void _webTask(void*){
  for(;;){ g_web.handleClient(); vTaskDelay(pdMS_TO_TICKS(5)); }
}

// Llamar al final de setup(). Potencia de TX baja: menos picos de corriente y menos ruido.
static void netlogStartAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  g_web.on("/", HTTP_GET, [](){ g_web.send_P(200, "text/html; charset=utf-8", LOG_PAGE); });
  g_web.on("/log", HTTP_GET, _webLog);
  g_web.on("/clear", HTTP_POST, [](){ logClear(); g_web.send(200, "text/plain", "ok"); });
  g_web.begin();
  xTaskCreatePinnedToCore(_webTask, "weblog", 6144, nullptr, 1, nullptr, 0);
  g_webOn = true;
  logPrintf("[WIFI] AP \"%s\" clave \"%s\"  ->  http://%s\n", WIFI_AP_SSID, WIFI_AP_PASS,
            WiFi.softAPIP().toString().c_str());
}
#else
static inline void netlogStartAP(){}
#endif
