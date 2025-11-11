
flash:
    cargo run

watch-serial:
    ssh wheatpi 'socat - /dev/ttyUSB0,raw,echo=0,crnl' | hexdump -C