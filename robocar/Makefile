COMPILER  = gcc
ARCHIVER  = ar
CFLAGS    = -g -O2 -MMD -MP -Wall -Wextra
ARFLAGS   = crsv
EVDSPDIR  = ../ext/evdsptc
DRVDIR    = ../ext/drv8830-i2c
CVTWBDIR  = ../ext/civetweb
BNO055DIR = ../ext/bno055-i2c
LDFLAGS   = -L$(EVDSPDIR)/build -L$(BNO055DIR)/build -lbno055-i2c -lwiringPi -lpthread -ldl -lm
EVDSPLIB  = $(EVDSPDIR)/build/libevdsptc.a
DRVLIB    = $(DRVDIR)/build/libdrv8830-i2c.a
CVTWBLIB  = $(CVTWBDIR)/libcivetweb.a
BNO055LIB = $(BNO055DIR)/build/libbno055-i2c.a
LIBS      = $(EVDSPLIB) $(DRVLIB) $(CVTWBLIB) $(BNO055LIB)
SRCDIR    = ./src
INCLUDE   = -I$(SRCDIR) -I$(EVDSPDIR)/src -I$(DRVDIR)/src -I$(CVTWBDIR)/include -I$(BNO055DIR)/src

TARGET    = evrbcar
UDPLIB    = lib$(TARGET)_udp.a
SOURCES   = $(wildcard $(SRCDIR)/*.c)
OBJDIR    = $(SRCDIR)
OBJECTS   = $(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.c=.o)))
DEPENDS   = $(OBJECTS:.o=.d)

.PHONY: all
all: clean $(TARGET) $(UDPLIB)

.PHONY: clean
clean:
	-rm -f $(OBJECTS) $(DEPENDS) $(TARGET) $(UDPLIB)

.PHONY: udplib
udplib: $(UDPLIB)

$(TARGET): $(OBJECTS) $(LIBS)
	$(COMPILER) -o $@ $^ $(LDFLAGS)

$(UDPLIB): $(SRCDIR)/$(TARGET)_udp.c
	$(COMPILER) $(CFLAGS) -I$(SRCDIR) -o $(SRCDIR)/udp.o -c $<
	$(ARCHIVER) $(ARFLAGS) -o $@ $(SRCDIR)/udp.o

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(COMPILER) $(CFLAGS) $(INCLUDE) -o $@ -c $<

$(EVDSPLIB):
	cd $(EVDSPDIR)/build;\
	cmake ..;\
	make;\

$(DRVLIB):
	cd $(DRVDIR)/build;\
	cmake ..;\
	make;\

$(CVTWBLIB):
	cd $(CVTWBDIR);\
	make lib lib WITH_WEBSOCKET=1;\

$(BNO055LIB):
	cd $(BNO055DIR)/build;\
	cmake ..;\
	make;\

-include $(DEPENDS)
