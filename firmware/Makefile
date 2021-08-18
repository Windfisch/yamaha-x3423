all: debug.bin release.bin

%.bin: target/thumbv7m-none-eabi/%/yamahax3423
	arm-none-eabi-objcopy -O binary $< $@

flash.%: %.bin
	stm32flash /dev/ttyUSB* -b38400 -w $< -e 60

.PHONY: target/thumbv7m-none-eabi/debug/yamahax3423
.PHONY: target/thumbv7m-none-eabi/release/yamahax3423

target/thumbv7m-none-eabi/debug/yamahax3423:
	cargo build --target=thumbv7m-none-eabi
target/thumbv7m-none-eabi/release/yamahax3423:
	cargo build --release --target=thumbv7m-none-eabi
