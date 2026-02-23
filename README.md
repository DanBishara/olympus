# OLYMPUS

GitHub repository for the nrf based discrete tracking device

## Setting up build env for matthew
1. Install nRF Connect for [VS Code Extension Pack](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-extension-pack)
2. ON the left there should be a tilted square with a link in between, that is nRF connect (the build env)
3. under "applications" select "Add build configuration"
4. Under the "CMake preset" dropdown, select "Build for rev1", this will automaticaly populate everything for you
5. Scroll to the bottom and press "generate and build"
6. The generated hex should be found under build_forge/olympus/zephyr/zephyr.hex

## Flashing a board
1. Download the nRF connect app at the following [link](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop/Download#infotabs)
2. Open up the installer and follow the steps then run the program
3. Open the programmer app
4. Connect the board using a jlink and select it in the top left corner drop down menu
5. Add the hex file you wanna program, note if you update the hex select reload, re-adding the file will cause overlap in the binaries
6. Select "erase & write"

## FAQ
- I'm stuck
  - ofc you are, just text me bozo


