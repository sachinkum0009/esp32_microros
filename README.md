# ESP32 Micro ROS

## Commands
```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0

esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 path/to/bootloader.bin 0x8000 path/to/partitions.bin 0x10000 path/to/firmware.bin

"/usr/bin/python3" "/home/ubuntu/.platformio/packages/tool-esptoolpy/esptool.py" --chip esp32 --port "/dev/ttyAMA0" --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size 4MB 0x10000 .pio/build/esp32dev/firmware.bin

```
