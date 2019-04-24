# *** ENTER THE TARGET NAME HERE ***
TARGET      = showbutler

CYPRESS_INC_PATH = /usr/local/include/cyusb #/usr/local/cyusb/include
CYPRESS_LIB_PATH = /usr/local/lib #/usr/local/cyusb/lib

CYINC = -I${CYPRESS_INC_PATH}
CYLIB = -L${CYPRESS_LIB_PATH} -lcyusb

CCMPL = g++
CFLAGS = -g

# important directories used by assorted rules and other variables
DIR_DXL    = ../..
DIR_OBJS   = .objects

# compiler options
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) -g
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) -g
#CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) $(FORMAT) -fPIC -g
#CXFLAGS 	= -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) $(FORMAT) -fPIC -g
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) #-Wl,-rpath,$(DIR_THOR)/lib
FORMAT      = -m64

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES   += -I$(DIR_DXL)/include/dynamixel_sdk
INCLUDES   += -I${CYPRESS_INC_PATH}

LIBRARIES  += -ldxl_x64_cpp
LIBRARIES  += -lrt
LIBRARIES  += -lpthread
LIBRARIES  += -L${CYPRESS_LIB_PATH} -lcyusb

#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES = shoebutler.cpp saictime.cpp shm.cpp traj.cpp\
    # *** OTHER SOURCES GO HERE ***

OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))
#OBJETCS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***


#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET) dvsconf2

dvsconf2: dvsconf2.c
	${CCMPL} -o dvsconf2 dvsconf2.c ${CFLAGS} ${CYINC} ${CYLIB} -lpthread

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: ../%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ../%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
