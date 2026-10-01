#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <ESP_I2S.h>              // incluida en el core ESP32 3.x

// ---------- Red AP ----------
const char* AP_SSID = "ESP32-CAM-AP";
const char* AP_PASS = "12345678";

// ---------- Pines cámara XIAO ESP32S3 Sense ----------
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// ---------- Micrófono PDM ----------
#define MIC_CLK_PIN  42
#define MIC_DATA_PIN 41
#define MIC_RATE     16000

I2SClass I2S;

httpd_handle_t web_httpd = NULL;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t audio_httpd = NULL;

// ---------- Dashboard ----------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Cam Dashboard</title>
<style>
 body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:20px}
 img{max-width:100%;border:2px solid #444;border-radius:8px}
 .badge{display:inline-block;padding:4px 10px;border-radius:12px;background:#0a7;margin:8px}
 button{padding:8px 16px;border:0;border-radius:6px;background:#27f;color:#fff;font-size:15px;cursor:pointer}
 .row{margin:12px}
</style></head><body>
<h2>ESP32-S3 Camera</h2>
<div class="badge" id="st">Conectando...</div><br>
<img id="v" alt="stream">
<div class="row">
  <button id="abtn" onclick="toggleAudio()">🔇 Activar audio</button>
  Volumen <input type="range" id="vol" min="0" max="10" step="0.5" value="4">
</div>
<script>
 const v=document.getElementById('v');
 v.src='http://'+location.hostname+':81/stream';
 v.onload=()=>document.getElementById('st').textContent='En vivo';
 v.onerror=()=>document.getElementById('st').textContent='Sin señal';

 let ctx=null, gainNode=null, ctrl=null, on=false;
 const RATE=16000;

 document.getElementById('vol').oninput=e=>{ if(gainNode) gainNode.gain.value=+e.target.value; };

 async function toggleAudio(){
   const btn=document.getElementById('abtn');
   if(on){ ctrl.abort(); ctx.close(); on=false; btn.textContent='🔇 Activar audio'; return; }
   ctx=new AudioContext({sampleRate:RATE});
   gainNode=ctx.createGain();
   gainNode.gain.value=+document.getElementById('vol').value;
   gainNode.connect(ctx.destination);
   ctrl=new AbortController();
   on=true; btn.textContent='🔊 Audio activo';
   try{
     const r=await fetch('http://'+location.hostname+':82/audio',{signal:ctrl.signal});
     const reader=r.body.getReader();
     let nextTime=ctx.currentTime+0.3, left=new Uint8Array(0);
     while(true){
       const {done,value}=await reader.read();
       if(done) break;
       const bytes=new Uint8Array(left.length+value.length);
       bytes.set(left); bytes.set(value,left.length);
       const n=bytes.length>>1;
       left=bytes.slice(n*2);
       const pcm=new Int16Array(bytes.buffer,0,n);
       const f=new Float32Array(n);
       for(let i=0;i<n;i++) f[i]=pcm[i]/32768;
       // si el buffer acumuló más de 1 s de retraso, descartar para mantener baja latencia
       if(nextTime>ctx.currentTime+1) continue;
       if(nextTime<ctx.currentTime) nextTime=ctx.currentTime+0.1;
       const ab=ctx.createBuffer(1,n,RATE);
       ab.copyToChannel(f,0);
       const s=ctx.createBufferSource();
       s.buffer=ab; s.connect(gainNode);
       s.start(nextTime); nextTime+=ab.duration;
     }
   }catch(e){ if(e.name!=='AbortError') console.log(e); }
 }
</script></body></html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// ---------- Stream MJPEG ----------
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part[64];

  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    size_t hlen = snprintf(part, sizeof(part),
      "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);

    res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 10);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

// ---------- Stream de audio (PCM 16 bit, 16 kHz, mono) ----------
static esp_err_t audio_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  int16_t buf[512];   // 512 muestras = 32 ms
  while (true) {
    size_t n = I2S.readBytes((char*)buf, sizeof(buf));
    if (n == 0) continue;
    if (httpd_resp_send_chunk(req, (const char*)buf, n) != ESP_OK) break;
  }
  return ESP_OK;
}

void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_open_sockets = 2;      // hay 3 servidores y LwIP tiene pocos sockets
  config.lru_purge_enable = true;

  httpd_uri_t index_uri  = { "/",       HTTP_GET, index_handler,  NULL };
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
  httpd_uri_t audio_uri  = { "/audio",  HTTP_GET, audio_handler,  NULL };

  config.server_port = 80;
  if (httpd_start(&web_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(web_httpd, &index_uri);

  config.server_port = 81;
  config.ctrl_port  += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(stream_httpd, &stream_uri);

  config.server_port = 82;
  config.ctrl_port  += 1;
  if (httpd_start(&audio_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(audio_httpd, &audio_uri);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ----- Cámara -----
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("Error al iniciar la camara (revisa PSRAM y conector)");
    return;
  }

  // ----- Micrófono PDM -----
  I2S.setPinsPdmRx(MIC_CLK_PIN, MIC_DATA_PIN);
  if (!I2S.begin(I2S_MODE_PDM_RX, MIC_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Error al iniciar el microfono");
  } else {
    Serial.println("Microfono OK");
  }

  // ----- WiFi AP -----
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP listo. IP: ");
  Serial.println(WiFi.softAPIP());

  startServers();
}

void loop() { delay(1000); }