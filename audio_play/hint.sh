# mp3 to wav
gst-launch-1.0 filesrc location=tmp.mp3 ! decodebin ! audioresample ! audioconvert ! audio/x-raw, format=S24LE, rate=48000 ! wavenc ! filesink location=tmp.wav

# wav to mp3
gst-launch-1.0 filesrc location=tmp.wav ! wavparse ! audioconvert ! lamemp3enc ! filesink location=tmp.mp3
