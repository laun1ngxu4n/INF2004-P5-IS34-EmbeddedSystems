IS34 robot panel, packaged (no Python needed)
=============================================

is34_panel.exe is tools/panel/is34_panel.py as one file. Built with
tools/panel/build_exe.bat (rebuild it after any change to the panel).

On a laptop that has never run it:
1. Install Mosquitto (mosquitto.org, Windows 64-bit) to the default folder
   C:\Program Files\mosquitto  (the panel starts it from there).
2. Join the phone hotspot the robot uses.
3. Double-click is34_panel.exe. It starts the broker on port 1884. When
   Windows asks, ALLOW mosquitto.exe through the firewall (private
   network), or the robot cannot reach it.
4. Logs it saves go to logs\ next to the .exe; the last mapping to
   is34_panel_mapping.txt next to it.

IMPORTANT: the robot's WiFi image has the broker's IP address built in
(config/wifi_secrets.h, MQTT_BROKER_IP: the laptop it was built for).
On another laptop the robot connects only if that laptop has the same IP
on the hotspot. Otherwise: on the build laptop, set MQTT_BROKER_IP to the
new laptop's IP (ipconfig on it, the hotspot's adapter), rebuild the WiFi
images, and flash. The robot's log shows "mqtt: connected to <ip>" when
it works, or "mqtt: no answer from <ip>".
