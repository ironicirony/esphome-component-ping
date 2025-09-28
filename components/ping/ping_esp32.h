/*
 * Copyright (c) 2021 Tomoyuki Sakurai <y@trombik.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#pragma once

#if defined(USE_ESP32)
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "ping_sock.h"
#include "esp_err.h"
#include <lwip/ip_addr.h>

#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ping {

static const char *const TAG = "ping_esp32";

class PingSensorESP32 : public PingSensor {
 public:
  void setup() override { init_ping(); }

  void update() override {
    float loss;
    int latency_ms;
    esp_err_t err = ESP_FAIL;

    loss = this->get_latest_loss();
    latency_ms = this->get_latest_latency();

    if (loss >= 0 && this->packet_loss_sensor_ != nullptr) {
      packet_loss_sensor_->publish_state(loss);
    }
    if (latency_ms >= 0 && this->latency_sensor_ != nullptr) {
      latency_sensor_->publish_state((float) latency_ms / 1000);
    }
    err = esp_ping_new_session(&ping_config, &cbs, &ping);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ping_new_session: %s", esp_err_to_name(err));
    }
    err = esp_ping_start(ping);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ping_start: %s", esp_err_to_name(err));
    }
  }

 private:
  static void cb_ping_on_ping_success(esp_ping_handle_t hdl, void *context) {
    reinterpret_cast<PingSensorESP32 *>(context)->cmd_ping_on_ping_success(hdl);
  }

  static void cb_cmd_ping_on_ping_end(esp_ping_handle_t hdl, void *context) {
    reinterpret_cast<PingSensorESP32 *>(context)->cmd_ping_on_ping_end(hdl);
  }

  static void cb_cmd_ping_on_ping_timeout(esp_ping_handle_t hdl, void *context) {
    reinterpret_cast<PingSensorESP32 *>(context)->cmd_ping_on_ping_timeout(hdl);
  }

  void cmd_ping_on_ping_success(esp_ping_handle_t hdl) {
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    ESP_LOGV(TAG, "%d bytes from %s icmp_seq=%d ttl=%d time=%d ms", recv_len, ipaddr_ntoa((ip_addr_t *) &target_addr),
             seqno, ttl, elapsed_time);
    this->incr_total_success_time(elapsed_time);
  }

  void cmd_ping_on_ping_timeout(esp_ping_handle_t hdl) {
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    ESP_LOGV(TAG, "From %s icmp_seq=%d timeout", ipaddr_ntoa((ip_addr_t *) &target_addr), seqno);
  }

  void cmd_ping_on_ping_end(esp_ping_handle_t hdl) {
    ip_addr_t target_addr;
    uint32_t transmitted;
    uint32_t received;
    uint32_t total_time_ms;
    uint32_t loss;
    int mean = 0;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));

    if (transmitted > 0) {
      loss = (uint32_t)((1 - ((float)received) / transmitted) * 100);
    } else {
      loss = 0;
    }

    if (received != 0) {
      mean = total_success_time / received;
    }

#if CONFIG_LWIP_IPV6
    if (IP_IS_V4(&target_addr)) {
#endif
      ESP_LOGD(TAG, "--- %s ping statistics ---", inet_ntoa(*ip_2_ip4(&target_addr)));
#if CONFIG_LWIP_IPV6
    } else {
      char addr_str[IPADDR_STRLEN_MAX];
      ipaddr_ntoa_r(&target_addr, addr_str, sizeof(addr_str));
      ESP_LOGD(TAG, "--- %s ping statistics ---", addr_str);
    }
#endif
    ESP_LOGD(TAG, "%d packets transmitted, %d received, %d%% packet loss, total time %dms avg time %dms", transmitted,
             received, loss, total_time_ms, mean);

    this->set_latest_loss(loss);
    this->set_latest_latency(mean);
    this->reset();
    esp_ping_delete_session(hdl);
  }

 protected:
//  std::string tag = "ping_esp32";
  esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
  esp_ping_handle_t ping;
  esp_ping_callbacks_t cbs;

  void init_ping() {
/*
    ip_addr_t target_addr;
    int err;

    memset(&target_addr, 0, sizeof(target_addr));
    err = inet_pton(AF_INET, target.c_str(), &target_addr);
    if (err == 0) {
      ESP_LOGE(tag.c_str(), "invalid address: `%s`", target.c_str());
      this->status_set_warning();
      return;
    } else if (err < 0) {
      ESP_LOGE(tag.c_str(), "inet_pton(): %s", esp_err_to_name(errno));
      this->status_set_warning();
      return;
    }
*/
    // parse IP address
#if CONFIG_LWIP_IPV6
    struct sockaddr_in6 sock_addr6;
#endif
    ip_addr_t target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
#if CONFIG_LWIP_IPV6
    if (inet_pton(AF_INET6, target.c_str(), &sock_addr6.sin6_addr) == 1) {
        // convert ip6 string to ip6 address
        ipaddr_aton(target.c_str(), &target_addr);
    } else {
#endif
        struct addrinfo hint;
        struct addrinfo *res = NULL;
        memset(&hint, 0, sizeof(hint));
        // convert ip4 string or hostname to ip4 or ip6 address
        if (getaddrinfo(target.c_str(), NULL, &hint, &res) != 0) {
            //printf("ping: unknown host %s\n", ping_args.host->sval[0]);
            //ESP_LOGE(tag.c_str(), "unknown host: %s", target.c_str());
            ESP_LOGE(TAG, "unknown host: %s", target.c_str());
            this->status_set_warning();
            return;
        }
#if CONFIG_LWIP_IPV6
        if (res->ai_family == AF_INET) {
#endif
            struct in_addr addr4 = ((struct sockaddr_in *) (res->ai_addr))->sin_addr;
            inet_addr_to_ip4addr(ip_2_ip4(&target_addr), &addr4);
#if CONFIG_LWIP_IPV6
        } else {
            struct in6_addr addr6 = ((struct sockaddr_in6 *) (res->ai_addr))->sin6_addr;
            inet6_addr_to_ip6addr(ip_2_ip6(&target_addr), &addr6);
        }
#endif
        freeaddrinfo(res);
#if CONFIG_LWIP_IPV6
    }
#endif

    ping_config.target_addr = target_addr;
    ping_config.count = n_packet;

    cbs.on_ping_success = PingSensorESP32::cb_ping_on_ping_success;
    cbs.on_ping_timeout = PingSensorESP32::cb_cmd_ping_on_ping_timeout;
    cbs.on_ping_end = PingSensorESP32::cb_cmd_ping_on_ping_end;
    cbs.cb_args = this;
  }
  int total_success_time;
  void reset() { total_success_time = 0; }
  void incr_total_success_time(int time) { total_success_time += time; }
};

}  // namespace ping
}  // namespace esphome
#endif
