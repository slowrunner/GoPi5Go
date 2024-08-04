Web Browser CLI To PiOS Robot

GoPiGo OS has a browser based interface built in, but if a user chooses to install the GoPiGo3 API on the 32-bit Legacy (Buster) PiOS, remote login is restricted to either remote ssh or remote desktop via VNC.

There is a way to setup up browser based CLI (command line interface)!

I just tried it on my Pi5 running 64-bit Bookworm PiOS - worked

```
sudo apt-get install build-essential cmake git libjson-c-dev libwebsockets-dev
git clone https://github.com/tsl0922/ttyd.git
cd ttyd && mkdir build && cd build
cmake ..
make && sudo make install
```

To run with secure login: ```sudo ttyd -p 8080 --writable login```
Then point your browser to your GoPiGo3 robot ```http://X.X.X.X:8080``` and log in remote.

To run without login: ```ttyd -p 8080 writable bash```

It is possible to set it up as a systemd service to make it available all the time.

REF: https://computingforgeeks.com/share-linux-terminal-over-web-using-ttyd/

