ARTeloperation ROS Package

- Reference (ROSAria package)

- Check camera device name
```
$ v4l2-ctl --list-devices
```

- Check supported format and size of the device
```
$ ffmpeg -f v4l2 -list_formats all -i [/dev/video0]
```

- Start streaming server and sending a video feed to the server
```
$ ffserver
$ ffmpeg -f video4linux2 -i [/dev/video0] http://localhost:8090/feed0.ffm
```

- To change the configuration of the ffserver, modify '/etc/ffserver.conf' file.
Things to check:
* Port number
* ffm file type, name
* IP address range for access allowance
* stream settings (buffer size, codec, bitrate ...) -> these settings are empirical values.