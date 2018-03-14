BIN=xs_test

CC= ../rv1108_rk_sdk/prebuilts/toolschain/usr/bin/arm-linux-gcc
CPP= ../rv1108_rk_sdk/prebuilts/toolschain/usr/bin/arm-linux-g++
AR= ../rv1108_rk_sdk/prebuilts/toolschain/usr/bin/arm-linux-ar

CFLAGS += -O0 -DT618_SDK -DJCONFIG_INCLUDED
LIBS += -lpthread -lrt


INC= -I.

SRCCS=	./xsapp.c \
		./xs508_check.c

OBJS=$(SRCCS:.c=.o)

OBJXS= lib_tuotu508p_R1.o

all: $(BIN)

$(BIN): $(OBJS)
	$(CPP) -o $(BIN) $(OBJS) $(OBJXS) $(LIBS)
	@-rm $(OBJS) $(OBJCPP)
	@echo "XS_TEST build success!"

$(OBJS):%.o:%.c
	$(CC) -c $(CFLAGS) $(INC) $< -o $@

$(OBJCPP):%.o:%.cpp
	$(CPP) -c $(CFLAGS) $(INC) $< -o $@

.PHONY: clean
clean:
	rm $(OBJS) $(OBJCPP) $(BIN)

test:
	echo $(INC)

