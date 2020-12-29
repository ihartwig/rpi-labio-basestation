.PHONY:	all
all:	dccwaved

dccwaved:	dccwaved.c mailbox.c
	gcc -Wall -g -O2 -L/opt/vc/lib -I/opt/vc/include -o dccwaved dccwaved.c mailbox.c -lm -lbcm_host

clean:
	rm -f dccwaved