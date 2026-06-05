LIBCCD_ROOT=../../vendor/libccd/src

CFLAGS=-I$(LIBCCD_ROOT) -std=c99
LDFLAGS=-lm
GEN=test_cases_generator
OUT=../collision_test_cases.yaml

$(OUT): $(GEN)
	./$(GEN) > $(OUT)

$(GEN): libccd
	$(CC) -c generate_collision_test_cases.c -o $(GEN).o $(CFLAGS)
	$(CC) $(GEN).o $(LIBCCD_ROOT)/libccd.a -o $(GEN) $(CFLAGS) $(LDFLAGS)

libccd:
	$(MAKE) -C $(LIBCCD_ROOT) USE_SINGLE=yes
