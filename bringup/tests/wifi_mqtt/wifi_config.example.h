/*
 * Copy this file to wifi_config.h and fill it in.
 * wifi_config.h is in .gitignore so passwords never reach GitHub.
 *
 * The Pico W is 2.4 GHz and WPA2-Personal only. School WiFi
 * (WPA2-Enterprise) will not work: use a phone hotspot set to 2.4 GHz
 * ("maximise compatibility" on iPhone).
 */
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#define WIFI_SSID        "your-hotspot-name"
#define WIFI_PASSWORD    "your-hotspot-password"

/* Laptop running mosquitto, on the same hotspot. Find it with
 * ipconfig (Windows) or ip addr (Linux/macOS). */
#define MQTT_BROKER_IP   "192.168.43.100"
#define MQTT_BROKER_PORT 1883

#endif /* WIFI_CONFIG_H */
