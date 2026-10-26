作者：同济子豪兄 2026-10-26
RoBoBo V3.9

From this directory, with esptool installed:
python -m esptool --chip esp32s3 --port YOUR_SERIAL_PORT write-flash @flash_args

The merged image is an alternative at address 0x0, not an additional segment.
Built with m5stack:esp32:m5stack_atoms3, core 3.3.8.
boot_app0.bin and bootloader are generated from upstream ESP32 tooling, not independently authored.
