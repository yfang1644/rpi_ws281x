SRC = mailbox.c  pcm.c  ws2811.c  dma.c  pwm.c  rpihw.c   

libws2811.so: $(SRC)
	gcc -o $@ $^ -shared -fPIC

test: main.c libws2811.so
	gcc -o $@ $< -lws2811 -L.
