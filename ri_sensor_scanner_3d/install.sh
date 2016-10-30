#!/bin/bash
echo "Dodawanie wpisu o urządzeniu w pliku /etc/udev/rules.d/99-usb-ri.rules"
cd /etc/udev/rules.d/
sudo touch 99-usb-ri.rules
sudo sh -c 'echo "SUBSYSTEMS=="usb", KERNEL=="ttyUSB[0-9]*", MODE="0777", GROUP="plugdev"" >> 99-usb-ri.rules'
sudo sh -c 'echo "SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", MODE="0666", GROUP="plugdev"" >> 99-usb-ri.rules'
echo "Instalacja zakończona pomyślnie!"
cd ~/
