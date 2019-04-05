SRC = ws2811.c

libws2811.so: $(SRC)
	gcc -o $@ $^ -shared -fPIC

test: main.c libws2811.so
	gcc -o $@ $< -lws2811 -L.
