#include "server.h"
#include "vision.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "img_converters.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "SERVER";

#define STREAM_BOUNDARY   "espframe"
#define JPEG_QUALITY      80
#define CAPTURE_PERIOD_MS 1000

#define JPG_BUF_SIZE (16 * 1024)
static uint8_t s_jpg_buf[JPG_BUF_SIZE];

/* ── Estado global compartido ──────────────────────────────────── */
static struct {
    centroid_t centroid;
    float      theta_mm;    /* posicion eje theta [0-300 mm]; -1 = sin datos */
    float      phi_mm;      /* posicion eje phi   [0-300 mm]; -1 = sin datos */
} g_frame = { .theta_mm = -1.0f, .phi_mm = -1.0f };

static SemaphoreHandle_t g_mutex = NULL;

/* ── Callbacks JPEG ─────────────────────────────────────────────── */
typedef struct {
    uint8_t *buf;
    size_t   len;
    size_t   max;
    bool     overflow;
} jpg_acc_t;

static size_t acc_cb(void *arg, size_t index, const void *data, size_t len)
{
    jpg_acc_t *a = (jpg_acc_t *)arg;
    if (a->overflow || a->len + len > a->max) { a->overflow = true; return 0; }
    memcpy(a->buf + a->len, data, len);
    a->len += len;
    return len;
}

typedef struct {
    httpd_req_t *req;
    esp_err_t    res;
} jpg_http_ctx_t;

static size_t httpd_chunk_cb(void *arg, size_t index, const void *data, size_t len)
{
    jpg_http_ctx_t *ctx = (jpg_http_ctx_t *)arg;
    if (ctx->res != ESP_OK) return 0;
    ctx->res = httpd_resp_send_chunk(ctx->req, (const char *)data, (ssize_t)len);
    return (ctx->res == ESP_OK) ? len : 0;
}

/* ── Tarea de captura ───────────────────────────────────────────── */
void capture_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        centroid_t c = vision_compute_centroid(fb->buf, fb->width, fb->height);
        esp_camera_fb_return(fb);

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_frame.centroid = c;
        xSemaphoreGive(g_mutex);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAPTURE_PERIOD_MS));
    }
}

/* ── Tarea de stream MJPEG (async) ─────────────────────────────── *
 *
 *  El stream corre en su propia tarea FreeRTOS para no bloquear el
 *  hilo del httpd.  Así /centroid y /position del ESP32-Motor pueden
 *  ser atendidos mientras el PC recibe el vídeo.
 */
static void stream_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;
    char part_hdr[128];

    esp_err_t res = httpd_resp_set_type(req,
        "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
    if (res != ESP_OK) goto done;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    ESP_LOGI(TAG, "Cliente stream conectado");

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        centroid_t c = vision_compute_centroid(fb->buf, fb->width, fb->height);
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_frame.centroid = c;
        xSemaphoreGive(g_mutex);

        jpg_acc_t acc = { .buf = s_jpg_buf, .len = 0,
                          .max = JPG_BUF_SIZE, .overflow = false };
        frame2jpg_cb(fb, JPEG_QUALITY, acc_cb, &acc);
        esp_camera_fb_return(fb);

        if (acc.overflow || acc.len == 0) {
            vTaskDelay(pdMS_TO_TICKS(CAPTURE_PERIOD_MS));
            continue;
        }

        int hdr_len = snprintf(part_hdr, sizeof(part_hdr),
            "--" STREAM_BOUNDARY "\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n", (unsigned)acc.len);

        res = httpd_resp_send_chunk(req, part_hdr, hdr_len);
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char *)s_jpg_buf, (ssize_t)acc.len);
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, "\r\n", 2);
        if (res != ESP_OK) break;

        vTaskDelay(pdMS_TO_TICKS(CAPTURE_PERIOD_MS));
    }

done:
    ESP_LOGI(TAG, "Cliente stream desconectado");
    httpd_req_async_handler_complete(req);
    vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_req_t *req_copy;
    if (httpd_req_async_handler_begin(req, &req_copy) != ESP_OK)
        return httpd_resp_send_500(req);
    if (xTaskCreate(stream_task, "stream", 8192, req_copy, 4, NULL) != pdPASS) {
        httpd_req_async_handler_complete(req_copy);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ── Handler: snapshot JPEG ────────────────────────────────────── */
static esp_err_t snapshot_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return httpd_resp_send_500(req);

    jpg_acc_t acc = { .buf = s_jpg_buf, .len = 0,
                      .max = JPG_BUF_SIZE, .overflow = false };
    frame2jpg_cb(fb, JPEG_QUALITY, acc_cb, &acc);
    esp_camera_fb_return(fb);

    if (acc.overflow || acc.len == 0) return httpd_resp_send_500(req);
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, (const char *)s_jpg_buf, (ssize_t)acc.len);
}

/* ── Handler: GET /centroid — solo datos de vision ─────────────── */
static esp_err_t centroid_handler(httpd_req_t *req)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    centroid_t c = g_frame.centroid;
    xSemaphoreGive(g_mutex);

    char json[256];
    snprintf(json, sizeof(json),
        "{\"dx\":%.3f,\"dy\":%.3f,\"cx\":%.3f,\"cy\":%.3f,"
        "\"area\":%lu,\"valid\":%s,\"frame_w\":%d,\"frame_h\":%d}",
        c.dx, c.dy, c.cx, c.cy,
        (unsigned long)c.area, c.valid ? "true" : "false",
        CAM_FRAME_W, CAM_FRAME_H);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_sendstr(req, json);
}

/* ── Handler: POST /position — recibe posicion de ambos ejes ──── *
 *
 *  Cuerpo esperado (JSON):  {"theta":142.70,"phi":87.30}
 *  Rango valido:            0.0 .. 300.0 para cada eje
 */
static esp_err_t position_post_handler(httpd_req_t *req)
{
    char buf[80];
    int recv_len = httpd_req_recv(req, buf,
        (req->content_len < sizeof(buf) - 1) ? req->content_len : sizeof(buf) - 1);
    if (recv_len <= 0) return httpd_resp_send_500(req);
    buf[recv_len] = '\0';

    float theta = -1.0f, phi = -1.0f;
    char *p;
    p = strstr(buf, "\"theta\":"); if (p) sscanf(p + 8, "%f", &theta);
    p = strstr(buf, "\"phi\":");   if (p) sscanf(p + 6, "%f", &phi);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (theta >= 0.0f && theta <= 300.0f) g_frame.theta_mm = theta;
    if (phi   >= 0.0f && phi   <= 300.0f) g_frame.phi_mm   = phi;
    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* ── Handler: GET /status — estado completo para Kivy ─────────── *
 *
 *  Respuesta:
 *    { "dx", "dy", "cx", "cy", "area", "valid",
 *      "theta", "phi",   ← posicion de cada actuador [mm, 0-300]
 *      "frame_w", "frame_h" }
 */
static esp_err_t status_handler(httpd_req_t *req)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    centroid_t c   = g_frame.centroid;
    float theta    = g_frame.theta_mm;
    float phi      = g_frame.phi_mm;
    xSemaphoreGive(g_mutex);

    char json[384];
    snprintf(json, sizeof(json),
        "{"
          "\"dx\":%.3f,"
          "\"dy\":%.3f,"
          "\"cx\":%.3f,"
          "\"cy\":%.3f,"
          "\"area\":%lu,"
          "\"valid\":%s,"
          "\"theta\":%.2f,"
          "\"phi\":%.2f,"
          "\"frame_w\":%d,"
          "\"frame_h\":%d"
        "}",
        c.dx, c.dy, c.cx, c.cy,
        (unsigned long)c.area,
        c.valid ? "true" : "false",
        theta, phi,
        CAM_FRAME_W, CAM_FRAME_H);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_sendstr(req, json);
}

/* ── Handler: pagina de inicio ─────────────────────────────────── */
static esp_err_t root_handler(httpd_req_t *req)
{
    static const char html[] =
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<title>ESP32-CAM Vision</title>"
        "<style>"
            "body{font-family:sans-serif;padding:20px;background:#111;color:#eee}"
            "img{border:2px solid #444;display:block;margin:12px 0}"
            "a{color:#4af}"
            "pre{background:#222;padding:8px;border-radius:4px;font-size:12px}"
            ".bar-bg{background:#333;border-radius:4px;height:22px;"
                    "width:300px;margin:4px 0;position:relative}"
            ".bar-fg{background:#4af;height:100%;border-radius:4px;"
                    "width:0%;transition:width .4s}"
            "table td{padding:2px 8px}"
        "</style>"
        "<script>"
            "function refreshFrame(){"
                "var img=document.getElementById('frame');"
                "img.src='/snapshot?t='+Date.now();"
            "}"
            "function poll(){"
                "fetch('/status').then(r=>r.json()).then(d=>{"
                    "document.getElementById('raw').textContent=JSON.stringify(d,null,2);"
                    "document.getElementById('dx').textContent=d.dx.toFixed(2);"
                    "document.getElementById('dy').textContent=d.dy.toFixed(2);"
                    "function bar(id,val,idval){"
                        "var pct=val>=0?Math.min(100,val/3.0):0;"
                        "document.getElementById(id).style.width=pct+'%';"
                        "document.getElementById(idval).textContent="
                            "val>=0?val.toFixed(1)+' mm':'sin datos';"
                    "}"
                    "bar('tbar',d.theta,'tval');"
                    "bar('pbar',d.phi,'pval');"
                "}).catch(()=>{});"
                "setTimeout(poll,500);"
            "}"
            "window.onload=function(){refreshFrame();setInterval(refreshFrame,1000);poll();};"
        "</script>"
        "</head><body>"
        "<h2>ESP32-CAM &mdash; proceso_cam_v1</h2>"
        "<img id='frame' width='160' height='120'>"
        "<table>"
            "<tr><td>dx (px):</td><td><b id='dx'>--</b></td></tr>"
            "<tr><td>dy (px):</td><td><b id='dy'>--</b></td></tr>"
        "</table>"
        "<p style='margin:10px 0 4px'>Actuador THETA (0&ndash;300 mm):</p>"
        "<div class='bar-bg'><div id='tbar' class='bar-fg'></div></div>"
        "<span id='tval'>sin datos</span>"
        "<p style='margin:8px 0 4px'>Actuador PHI (0&ndash;300 mm):</p>"
        "<div class='bar-bg'><div id='pbar' class='bar-fg'></div></div>"
        "<span id='pval'>sin datos</span>"
        "<p style='margin-top:14px'>"
            "<a href='/status'>GET /status</a>&nbsp;&nbsp;"
            "<a href='/centroid'>GET /centroid</a>&nbsp;&nbsp;"
            "<a href='/snapshot'>Snapshot</a>"
        "</p>"
        "<pre id='raw'>esperando...</pre>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, html);
}

/* ── Arranque del servidor HTTP ────────────────────────────────── */
esp_err_t server_start(void)
{
    g_mutex = xSemaphoreCreateMutex();
    configASSERT(g_mutex);

    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size        = 8192;
    cfg.max_open_sockets  = 7;
    cfg.lru_purge_enable  = true;
    cfg.recv_wait_timeout = 5;
    cfg.send_wait_timeout = 5;

    httpd_handle_t srv = NULL;
    esp_err_t res = httpd_start(&srv, &cfg);
    if (res != ESP_OK) { ESP_LOGE(TAG, "httpd_start fallo: %d", res); return res; }

    static const httpd_uri_t routes[] = {
        { .uri = "/",         .method = HTTP_GET,  .handler = root_handler          },
        { .uri = "/snapshot", .method = HTTP_GET,  .handler = snapshot_handler      },
        { .uri = "/centroid", .method = HTTP_GET,  .handler = centroid_handler      },
        { .uri = "/stream",   .method = HTTP_GET,  .handler = stream_handler        },
        { .uri = "/status",   .method = HTTP_GET,  .handler = status_handler        },
        { .uri = "/position", .method = HTTP_POST, .handler = position_post_handler },
    };
    for (int i = 0; i < (int)(sizeof(routes) / sizeof(routes[0])); i++)
        httpd_register_uri_handler(srv, &routes[i]);

    ESP_LOGI(TAG, "Servidor HTTP listo en :80");
    ESP_LOGI(TAG, "  GET  /status    -> {dx, dy, pos, valid, ...}  [Kivy]");
    ESP_LOGI(TAG, "  POST /position  -> recibe {\"pos\":142.7}        [motor ESP32]");
    ESP_LOGI(TAG, "  GET  /centroid  -> solo centroide               [motor ESP32]");
    ESP_LOGI(TAG, "  GET  /stream    -> MJPEG continuo");
    ESP_LOGI(TAG, "  GET  /snapshot  -> JPEG unico");
    return ESP_OK;
}
