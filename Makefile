COMPILER  = gcc
CFLAGS    = -g -O2 -MMD -MP -Wall -Wextra
EVDSPDIR  = ext/evdsptc
LDFLAGS   = -L$(EVDSPDIR)/build -lpthread -lwiringPi
EVDSPLIB  = $(EVDSPDIR)/build/libevdsptc.a
LIBS      = $(EVDSPLIB)
INCLUDE   = -I./src -I$(EVDSPDIR)/src
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

all: clean $(TARGET)

clean:
	-rm -f $(OBJECTS) $(DEPENDS) $(TARGET)

-include $(DEPENDS)
