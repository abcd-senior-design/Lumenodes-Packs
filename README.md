# Lumenodes-Packs
The pack-level software that runs on-chip inside every Lumenode light pack.

## Setting up
This code is designed for Nordic Semiconductor's nRF52840 BLE SoC. You will first want to get your dev environment ready:
* Download and extract the [Nordic nRF5 SDK, v15.0.0](https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/)
* Download and extract the [Nordic S140 SoftDevice, v6.X.X](https://www.nordicsemi.com/Software-and-Tools/Software/S140/Download)
* Download and install (with J-LINK debugger drivers) [Segger Embedded Studio](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/Segger-Embedded-Studio)
* Download and install [Nordic nRF Connect for Desktop](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-desktop)

## Developing with the repo on an nRF52840 Dev Kit
Assuming the nRF5 SDK was extracted into `.\nRF5_SDK_15.0.0_a53641a\`:
1. Clone this repo as `.\nRF5_SDK_15.0.0_a53641a\examples\ble_central\Lumenodes-Packs\`
2. Execute `.\nRF5_SDK_15.0.0_a53641a\examples\ble_central\Lumenodes-Packs\pca10056\s140\ses\lmnd_packs_c_pca10056_s140.emProject` with Segger Embedded Studio
3. You can now edit `main.c` and `sdk_config.h` freely!
4. Before debugging, right-click on "Project 'lmnd_pack_s140' on the left sidebar, then click "Options" in the context menu that pops up. This launches the Project options window
5. Change the configurations dropdown from "Release" to "Common"
6. In the menu sidebar, select "Preprocessor"
7. Make sure that "Preprocessor Definitions" starts with `BOARD_PCA10056`, the hardware designator for the Dev Kit
8. Click "OK"
9. Connect an nRF52840 Dev Kit (this project seems to run fine on both the DK and the PDK) to your PC via its debug USB port
9. From the Segger Embedded Studio toolbar, select "Debug", then "Go" (or just hit <kbd>F5</kbd>) to start the debug session. Click "OK" if you are prompted to build the project again
> Note: The debugger has already been set up via `sdk_config.h` to receive all `NRF_LOG_INFO(...)` prints via its output terminal for your convenience--woot

## Deploying the code to an nRF52840 Dongle via USB DFU
Assuming the nRF5 SDK was extracted into `.\nRF5_SDK_15.0.0_a53641a\` and that the SoftDevice was extracted into `.\s140nrf526XX\`:
1. With the `.emProject` open in Segger Embedded Studio, right-click on "Project 'lmnd_pack_s140'" on the left sidebar, then click "Options" in the context menu that pops up. This launches the Project options window
2. Change the configurations dropdown from "Release" to "Common"
3. In the menu sidebar, select "Preprocessor"
4. Make sure that "Preprocessor Definitions" starts with `BOARD_PCA10059`, the hardware designator for the Dongle
5. Click "OK"
6. From the Segger Embedded Studio toolbar, select "Build", then "Build lmnd_pack_s140" (or just hit <kbd>F7</kbd>) to compile the machine code
7. Launch nRF Connect for Desktop. If necessary, use "Add/Launch Apps" to install the "Programmer" app
8. Connect an nRF52840 Dongle to your PC via USB. Click the RESET button on the Dongle so that the LED pulses red
9. Launch the "Programmer" app inside nRF Connect
10. Use the "Select Device" dropdown to select the connected Dongle. The app should now show the Dongle's memory contents
11. Click "Clear Files" to clean your workspace, then either use "Add HEX file" or drag-and-drop to add both the freshly-compiled file `.\nRF5_SDK_15.0.0_a53641a\examples\ble_central\Lumenodes-Packs\pca10056\s140\ses\Output\Release\Exe\lmnd_pack_s140.hex` and the SoftDevice `.\s140nrf526XX\s140_nrf52_6.X.X_softdevice.hex` to the "File Memory Layout" workspace
12. Click "Write". Everything should run smoothly. The Dongle is flashed and ready to go when a red "Nordic DFU Trigger Interface was not found" message pops up

## HALP
Nordic has terrific support for their products! If you need help, try:
* Opening a support ticket (Nordic engineers typically respond within a business day) or browsing for answers on the [Nordic DevZone](https://devzone.nordicsemi.com/f/nordic-q-a)
* Reading up on the nRF5 SDK v15.0.0 on the [Nordic DocLib](https://www.nordicsemi.com/DocLib/Content/SDK_Doc/nRF5_SDK/v15-0-0/index)