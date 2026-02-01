# SmartESC_STM32_v3.31_Vesc (EBiCS Fork)


Attention: new logic to activate the autodetect procedure:
Pull the brake lever and press the dashboard button for 5 seconds!

Fork of EBiCS firmware for Lishui devices. Ported to Xiaomi M365 controller.
Use JST PA series 2mm pitch for the connectors. (need to be confirmed)

This branch works with the original M365 dashboard and now features **VESC UART Protocol** support!

## Recent Updates & Improvements

### 1. VESC UART Protocol Integration
The firmware now implements the VESC UART communication protocol on **USART3** (typically the programming/debug port).
- **Compatibility:** Interfaces with the latest version of the VESC Tool.
- **Commands Supported:**
  - `COMM_FW_VERSION`: Allows identification by the VESC Tool.
  - `COMM_GET_VALUES`: Provides real-time telemetry including battery voltage, motor current, and RPM.
  - `COMM_SET_CURRENT`: Allows controlling the motor current directly from the VESC Tool.
- **QoL Addition:** You can now use the VESC Tool for real-time data visualization, logging, and remote control, making tuning and diagnostics much easier.

*Note: Standard debug `printf_` output on USART3 has been disabled to prevent interference with the VESC protocol.*

### 2. Bug Fixes & Stability
- **Circular Buffer Safety:** Fixed a critical out-of-bounds access bug in the M365 Dashboard protocol parser that occurred during buffer wrap-around.
- **CRC Robustness:** Added boundary checks to the dashboard CRC calculation and verification functions to prevent memory corruption with malformed packets.
- **Build System:** Resolved multiple definition errors of the `huart3` handle, ensuring a clean compilation using GCC ARM.

## How to use:
### Method 1 (GitHub Actions - Easiest):
1. Fork this project to your GitHub account.
2. Edit `Core/Inc/config.h` in your repository online and commit the changes.
3. Click on the **Actions** button and monitor the "workflows".
4. Once finished, download the generated zip-file by clicking on the cube icon at the bottom of the summary page.
5. You can find a tutorial at [YouTube](https://youtu.be/Pe2UTPPxX7U)

### Method 2 (Local Installation):
1. Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Import this repo (File --> Import --> Git --> Projects from Git).
3. Edit `Core/Inc/config.h`. Working settings for the original M365 are in the comments.
4. Click the "Build" icon (hammer).
5. The flashable zip-file is generated in `/tools/zip-output`.
6. Copy it to your phone and flash via the **downG** app.

## Operation
After flashing, lift the motor so it can spin freely. Press the dashboard button for 5s to start the **autodetect routine**. The motor will turn slowly; once it stops, the scooter is calibrated.

**Dashboard Button Controls:**
- **Short Press:** Toggle lights
- **Double Click:** Switch ride modes
- **Long Press:** Power off
- **Very Long Press (5s) + Brake:** Run autodetect

![PCB Layout M365](https://github.com/Koxx3/SmartESC_STM32_v3/blob/master/Documentation/PCB%20Layout%20M365.PNG)
