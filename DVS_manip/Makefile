#
# requires libusb & cyusb
#

# ! have to edit the path !
CYPRESS_INC_PATH = /usr/local/include/cyusb #/usr/local/cyusb/include
CYPRESS_LIB_PATH = /usr/local/lib #/usr/local/cyusb/lib

CYINC = -I${CYPRESS_INC_PATH}
CYLIB = -L${CYPRESS_LIB_PATH} -lcyusb # -shared -fPIC

CCMPL = g++
CFLAGS = -g


all :  dvs dvs_fd4

dvs: _dvs.c
	${CCMPL} -o _dvs _dvs.c -shared -fPIC ${CFLAGS} ${CYINC} ${CYLIB}

dvs_fd4: _dvsfiledump_4.c
	${CCMPL} -o _dvsfiledump_4 _dvsfiledump_4.c -shared -fPIC ${CFLAGS} ${CYINC} ${CYLIB}

clean:
	rm -f _dvs  _dvsfiledump_4

