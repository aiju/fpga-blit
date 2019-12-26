#include <u.h>
#include <libc.h>
#include <thread.h>
#include <draw.h>
#include <keyboard.h>
#include <mouse.h>
#include "dat.h"
#include "fns.h"

uchar *mmio;

u32int
reg_read(int a)
{
	u32int v;

	v = *(u32int*)(&mmio[a]);
	return v;
}

void
reg_write(int a, u32int v)
{
	*(u32int*)(&mmio[a]) = v;
}

u32int
reg_poll(int a, u32int m, u32int v)
{
	int c;
	u32int s;
	
	for(c = 0;; c++){
		s = reg_read(a);
		if((s & m) == v)
			return s;
		if(c > 1000)
			sleep(1);
	}
}

static Channel *keych;
static uchar
keymap[] = {
	[Kup-KF] 0xf1,
	[Kdown-KF] 0xf2,
	[Kleft-KF] 0xf3,
	[Kright-KF] 0xf4,
	[1] 0xf6, /* PF1 */
	[2] 0xf7, /* PF2 */
	[3] 0xf8, /* PF3 */
	[4] 0xf9, /* PF4 */
	[12] 0xfe, /* SET-UP */
	[Kpgdown-KF] 0xb0, /* SCROLL */
	[Kins-KF] 0xe0, /* BREAK */
};

static void
keyproc(void *)
{
	int fd, cfd, ch, rc;
	static char buf[256];
	char *p;
	Rune r;

	fd = open("/dev/cons", OREAD);
	if(fd < 0)
		sysfatal("open: %r");
	cfd = open("/dev/consctl", OWRITE);
	if(cfd < 0)
		sysfatal("open: %r");
	fprint(cfd, "rawon");
	for(;;){
		rc = read(fd, buf, sizeof(buf) - 1);
		if(rc <= 0)
			sysfatal("read /dev/cons: %r");
		for(p = buf; p < buf + rc && (p += chartorune(&r, p)); ){
			if(r == Kend){
				close(fd);
				threadexitsall(nil);
			}
			ch = r;
			if(ch == '\n') ch = '\r';
			else if(ch >= KF){
				if(ch >= KF + nelem(keymap)) continue;
				ch = keymap[ch - KF];
				if(ch == 0) continue;
			}else if(ch >= 0x80) continue;
			send(keych, &ch);
		}
	}
}

static void
keywproc(void *)
{
	int ch;
	
	for(;;){
		recv(keych, &ch);
		reg_poll(KBD_IN_STATUS, 1, 0);
		reg_write(KBD_IN_DATA, ch);
	}
}

static void
mouseproc(void *)
{
	Mousectl *mc;
	int fd;
	
	mc = initmouse(nil, nil);
	if(mc == nil) sysfatal("initmouse: %r");
	fd = open("/dev/fbctl", OWRITE);
	if(fd < 0) sysfatal("open: %r");
	fprint(fd, "size 800x1024x32 x8r8g8b8");
	for(;;){
		if(readmouse(mc) < 0)
			sysfatal("readmouse: %r");
		reg_write(MOUSE, 799 - mc->xy.x | 1023 - mc->xy.y << 16);
		reg_write(MOUSE_BUT, mc->buttons >> 2 & 1 | mc->buttons & 2 | mc->buttons << 2 & 4);
	}
}

void
threadmain(int argc, char **argv)
{
	ARGBEGIN{
	default: sysfatal("usage");
	}ARGEND;
	if(argc != 1) sysfatal("usage");
	
	mmio = segattach(0, "axi", nil, 1048576*4);
	if(mmio == (uchar*)-1)
		sysfatal("segattach: %r");
	
	dport();

	keych = chancreate(sizeof(char), 128);
	proccreate(keyproc, nil, mainstacksize);
	proccreate(keywproc, nil, mainstacksize);
	proccreate(mouseproc, nil, mainstacksize);
	
	telnetinit(argv[0]);
}
