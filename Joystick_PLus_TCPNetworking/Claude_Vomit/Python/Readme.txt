No libraries to install. socket and struct are Python stdlib. You can run both scripts right now on any two machines on the same network and they'll talk. That's the beauty of raw sockets.
The only things you need to do:

Set the correct IPs (BASE_IP / ROBOT_IP) — or use "" to bind to all interfaces
Make sure your firewall isn't blocking those ports (on Linux: sudo ufw allow 9000:9002/tcp and /udp)
Run robot.py first, then base.py

Key things built into this example
FeatureWhereTCP_NODELAYDisables Nagle — critical for low-latency commandsLength-prefix framingstruct.pack(">I", len) — TCP is a stream, not packets; you must frame your own messagesrecvall() helperTCP recv() can return partial data; this loop ensures you get exactly N bytesstream_id + seq in UDP headerLets you add cameras and detect drops without extra portssafe_stop() on disconnectWatchdog — if the TCP connection drops, robot halts