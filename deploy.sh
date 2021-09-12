cross +nightly build --release --target armv7-unknown-linux-gnueabihf

pi_addr="192.168.0.23"
rsync -avz --progress ./target/armv7-unknown-linux-gnueabihf/release/vhrd-flasher pi@$pi_addr:/home/pi/
echo "Done"