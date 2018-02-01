# psk31lx-1.3 Extended version of PSK31LX designed for Raspberry Pi Zero & PSK31 Beacons

-Added GUI on off switch to ini file 
         Minimizes psk31lx to only messages to completely reduce it and reuse the terminal to run something else you can start it like this          $ psk31lx &>/dev/null &  

-Added AFC on off switch to ini file 
        In long running beacon applications the AFC receiving stray signals could cause the frequency to walk up or down the                       waterfall. Currently I turn off AFC, maybe a future update will limit how far off frequency AFC is allowed to go. 

-Added DCD value switch to ini file 
        Sets signal level to decode, this will reduce the "ignored unknown callsign" messages when running in GUI=0 mode but also                   may limit weak signal reception, more testing is required. 

-Added TX  / RX Telemetry Features 
       TX telemetry is simple, just assemble a sentence and put it in a file with an .txt extension and the software will read the                sentence, calculate a crc and add a colon to the end of the sentence then add the crc after the colon, then transmit the new                sentence.
       RX telemetry is looking for the callsign that is in the Call=XXXXXX in the ini file. If that callsign is decoded, the software will        check the crc, if the crc is good, the message is put in a file called pskreceived.txt located in /home/pi/psk31lx/RX/. If the
       crc fails, then the message is placed in the received.log file located in /home/pi/psk31lx/RX/
-Added received.log 
       If an incoming message has a good callsign but the crc fails, then the message is placed in the received.log file located in                /home/pi/psk31lx/RX/

