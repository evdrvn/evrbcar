COMPILER  = gcc
CFLAGS    = -g -O2 -MMD -MP -Wall -Wextra
EVDSPDIR  = ../../ext/evdsptc
LDFLAGS   = -L$(EVDSPDIR)/build -lpthread -ldl -lm
EVDSPLIB  = $(EVDSPDIR)/build/libevdsptc.a
UDPLIB    = ../../libevrbcar_udp.a
LIBS      = $(EVDSPLIB) $(UDPLIB)
SRCDIR    = .
INCLUDE   = -I../../src -I$(EVDSPDIR)/src

TARGET    = evrbcar_udp_example
SOURCES   = $(wildcard $(SRCDIR)/*.c)
OBJDIR    = $(SRCDIR)
OBJECTS   = $(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.c=.o)))
DEPENDS   = $(OBJECTS:.o=.d)

.PHONY: all
all: clean $(TARGET)

.PHONY: clean
clean:
	-rm -f $(OBJECTS) $(DEPENDS) $(TARGET)

$(TARGET): $(OBJECTS) $(LIBS)
	$(COMPILER) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(COMPILER) $(CFLAGS) $(INCLUDE) -o $@ -c $<

$(EVDSPLIB):
	cd $(EVDSPDIR)/build;\
	cmake ..;\
	make;\

-include $(DEPENDS)
