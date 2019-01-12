COMPILER  = gcc
CFLAGS    = -g -O2 -MMD -MP -Wall -Wextra
EVDSPDIR  = ext/evdsptc
DRVDIR    = ext/drv8830-i2c
CVTWBDIR  = ext/civetweb
LDFLAGS   = -L$(EVDSPDIR)/build -lwiringPi -lpthread -ldl
EVDSPLIB  = $(EVDSPDIR)/build/libevdsptc.a
DRVLIB    = $(DRVDIR)/build/libdrv8830-i2c.a
CVTWBLIB  = $(CVTWBDIR)/libcivetweb.a
LIBS      = $(EVDSPLIB) $(DRVLIB) $(CVTWBLIB)
INCLUDE   = -I./src -I$(EVDSPDIR)/src -I$(DRVDIR)/src -I$(CVTWBDIR)/include

TARGET    = evrbcar
SRCDIR    = ./src
SOURCES   = $(wildcard $(SRCDIR)/*.c)
OBJDIR    = ./src
OBJECTS   = $(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.c=.o)))
DEPENDS   = $(OBJECTS:.o=.d)

$(TARGET): $(OBJECTS) $(LIBS)
	$(COMPILER) -o $@ $^ $(LDFLAGS)

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

all: clean $(TARGET)

clean:
	-rm -f $(OBJECTS) $(DEPENDS) $(TARGET)

-include $(DEPENDS)
