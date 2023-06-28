# Build example using the .c files in this directory and the static libartl.a
all: lowlevelapi_example digit_example
lowlevelapi_example: lowlevelapi_example.c lowlevelapi.c libartl/libartl.a
	gcc $^ -o $@

digit_example: digit_invdyn.cc lowlevelapi.c libartl/libartl.a
	g++ -g -Wall $^ -o $@
# Build the static ARTL library
libartl/libartl.a:
	$(MAKE) -C libartl libartl.a

# Clean outputs from both this directory and libartl
clean:
	rm -f lowlevelapi_example digit_sensor
	$(MAKE) -C libartl clean

.PHONY: clean
