nix develop --command ffmpeg -re -f lavfi -i testsrc=size=1280x720:rate=30 -c:v libx264 -tune zerolatency -f rtp rtp://127.0.0.1:5600
