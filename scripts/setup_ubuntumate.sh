sudo apt update
sudo apt install vim terminator git
sudo apt install openssh-server
sudo apt install python3-gpiozero

# I had to change some permissions to access the GPIO
sudo chown root.pi /dev/gpiomem
sudo chmod g+rw /dev/gpiomem

