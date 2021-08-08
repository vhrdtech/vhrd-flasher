# vhrd-flasher
###vhrd-flasher 0.1.0
###Matvei Klimov <klimatt.gu@gmail.com>
###CAN-BUS flasher stm32 bootloader

##USAGE:
#####vhrd-flasher [OPTIONS]

####FLAGS:
#####-h, --help       Prints help information
#####-V, --version    Prints version information

####OPTIONS:
#####-n, --new_uavcan_node_id <Setup_NEW_node_id>    
#####-w, --node_id <Working_node_id>                 
#####-i, --iface <can_bus_iface>                     
#####-f, --file <file.bin>                

##Example:
#####cargo +nightly  run --  -f blinky.bin -w 45 -n 45 -i can0

#####Works with : https://github.com/vhrdtech/vhrd-bootloader

##ONLY FOR Cortex M0 Devices
#####Don't forget to offset RAM(192 bytes) in your firmware in linkerscript