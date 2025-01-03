if [ "$(whoami)" = "root" ]; then
  echo "Please don't run as root"
  exit 255
fi

sudo modprobe v4l2loopback -r
sudo modprobe v4l2loopback video_nr=9 card_label=VirtualDevice exclusive_caps=1
ls /dev/video*