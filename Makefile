COMPILER  = gcc
CFLAGS    = -g -O2 -MMD -MP -Wall -Wextra
EVDSPDIR  = ext/evdsptc
DRVDIR    = ext/drv8830-i2c
LDFLAGS   = -L$(EVDSPDIR)/build -lwiringPi -lpthread
EVDSPLIB  = $(EVDSPDIR)/build/libevdsptc.a
DRVLIB    = $(DRVDIR)/build/libdrv8830-i2c.a
LIBS      = $(EVDSPLIB) $(DRVLIB)
INCLUDE   = -I./src -I$(EVDSPDIR)/src -I$(DRVDIR)/src
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

all: clean $(TARGET)

clean:
	-rm -f $(OBJECTS) $(DEPENDS) $(TARGET)

-include $(DEPENDS)
