
flash id:
    ID={{id}} cargo run

calibrate:
    cargo run --bin switchgrass-cattail-calibrate

watch-serial:
    ssh wheatpi 'socat - /dev/ttyUSB0,raw,echo=0,crnl,b115200'
