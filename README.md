# pingpong_ball_tracker
Table tennis ball tracker with Blackfly S GigE + ESP32 with servos

Hardware:
1x Blackfly S GigE camera (BFS-PGE-13Y3C-C) + lens - powered via PoE switch;
1x ESP32 DEVKITV1;
2x MG996R servos (180 degrees);
1x 2-axis servo mount (compatible with MG995/MG996R);
1x 5V DC PSU that can sustain at least 4A of current draw;
1x Laser diode module (preferably one that supports 3.3V so it can be powered directly via ESP32);
The camera + laser combo is "strapped" onto the servo mount using tape. Make sure the camera and the laser are aligned (can be tuned later when running the application as it shows the image center).

Wiring: 
+5VDC (PSU) -> +5V (red wires) on servos;
GND (PSU) -> ESP32 GND, laser GND, servos GND (brown wires) ----- shared reference point;
Laser +V is connected to ESP32's 3.3V port;
Servos PWM pins (yellow wires) are connected as such:
a) PAN to ESP32 D18 pin;
b) TILT to ESP D19 pin;

REQUIREMENTS for ESP32 flashing:
Application has been confirmed to work with ESP32 DEVKITV1 (ESP-WROOM-32).
Install Arduino IDE. 
Install ESP32Servo library.
Flash the code.

MUST INSTALL BEFORE RUNNING:

! Microsoft Build Tools (MSVC)
check:
- desktop dev with c++
- msvc v14x toolset
- windows 10/11 SDK

! Spinnaker SDK
Add ..\Teledyne\Spinnaker\bin64\vs2015 to the PATH environment variable (eg. C:\Program Files\Teledyne\Spinnaker\bin64\vs2015)

! CMake
Add ..\CMake\bin to the PATH environment variable (eg. C:\Program Files\CMake\bin)

! OpenCV
Add ..\opencv\build\x64\vc16\bin to the PATH environment variable (eg. C:\opencv\build\x64\vc16\bin)

Optional: Ninja
Add ..\Ninja to the PATH environment variable (eg. C:\Program Files\Ninja)

Change the paths in compile_commands.json (found in root\build) to your local paths.


Press CTRL+SHIFT+P and select CMake: Configure -> MSVC, then CMake: Build.
To run the application, CMake: Run without Debugging OR run blackfly.exe (root\build)
! ESP32 MUST be connected to a COM port for the application to run.
! To set the ESP32 serial port, simply change it in "int main()" (first line).

Detection model comes pre-trained with yolov8n, exported in ONNX format (best.onnx). 
