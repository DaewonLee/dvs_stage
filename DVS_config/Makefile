#
# requires libusb & cyusb
#

# ! have to edit the path !
CYPRESS_INC_PATH = /usr/local/include/cyusb #/usr/local/cyusb/include
CYPRESS_LIB_PATH = /usr/local/lib #/usr/local/cyusb/lib

CYINC = -I${CYPRESS_INC_PATH}
CYLIB = -L${CYPRESS_LIB_PATH} -lcyusb

CCMPL = g++
CFLAGS = -g


all : dvsconf

dvsconf: dvsconf.c
	${CCMPL} -o dvsconf dvsconf.c ${CFLAGS} ${CYINC} ${CYLIB} -lpthread

clean:
	rm -f dvsconf 

