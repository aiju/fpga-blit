#include <u.h>
#include <libc.h>

uchar *mmio;

enum {
	CTRL = 0x00,
	STS = 0x04,
	AUXCTRL = 0x08,
	AUXSTAT = 0x0C,
	HVACT = 0x40,
	HVTOT = 0x44,
	HVSYNC = 0x48,
	HVDATA = 0x4c,
	MISC = 0x50,
	MVID0 = 0x54,
	NVID0 = 0x58,
	SCLK0 = 0x5c,
	MVID1 = 0x60,
	NVID1 = 0x64,
	SCLK1 = 0x68,
	
	LINK_BW_SET = 0x100,
	LANE_COUNT_SET = 0x101,
	TRAINING_PATTERN_SET = 0x102,
	TRAINING_LANE0_SET = 0x103,
	TRAINING_LANE1_SET = 0x104,
	LANE0_1_STATUS = 0x202,
	LANE_ALIGN_STATUS_UPDATED = 0x204,
	ADJUST_REQUEST_LANE0_1 = 0x206,
	
	LANE0_CR_DONE = 1,
	LANE0_CHANNEL_EQ_DONE = 2,
	LANE0_SYMBOL_LOCKED = 4,
	LANE1_CR_DONE = 16,
	LANE1_CHANNEL_EQ_DONE = 32,
	LANE1_SYMBOL_LOCKED = 64,
	INTERLANE_ALIGN_DONE = 1,
};

u32int
dp_read(int a)
{
	u32int v;

	if(a != AUXSTAT) print("read DP %.2x = ", a);
	v = *(u32int*)(&mmio[0x100 + a]);
	if(a != AUXSTAT) print("%.8ux\n", v);
	return v;
}

void
dp_write(int a, u32int v)
{
	if(a != AUXCTRL) print("write DP %.2x = %.8ux\n", a, v);
	*(u32int*)(&mmio[0x100 + a]) = v;
}

u8int
aux_read(int a)
{
	u32int s;
	u8int *m;
	int i;
	
	print("read AUX %.3x = ", a);
	m = mmio + 0x200;
	m[0] = 0x90 | a >> 16 & 0xf;
	m[1] = a >> 8;
	m[2] = a;
	m[3] = 0;
	dp_write(AUXCTRL, 1<<31 | 3);
	do{
		sleep(1);
		s = dp_read(AUXSTAT);
	}while((s & 1<<31) == 0);
	if((s & 0x100) != 0) sysfatal("aux timeout");
	if((s & 0x1f) != 2) sysfatal("wrong number of bytes returned");
	if(m[0] != 0) sysfatal("aux error code %.2x", m[0]);
	print("%.2x\n", m[1]);
	return m[1];
}

void
aux_write(int a, u8int v)
{
	u32int s;
	u8int *m;
	int i;
	
	print("write AUX %.3x = %.2x\n", a, v);
	m = mmio + 0x200;
	m[0] = 0x80 | a >> 16 & 0xf;
	m[1] = a >> 8;
	m[2] = a;
	m[3] = 0;
	m[4] = v;
	dp_write(AUXCTRL, 1<<31 | 4);
	do{
		sleep(1);
		s = dp_read(AUXSTAT);
	}while((s & 1<<31) == 0);
	if((s & 0x100) != 0) sysfatal("aux timeout");
	if((s & 0x1f) != 1) sysfatal("wrong number of bytes returned");
	if(m[0] != 0) sysfatal("aux error code %.2x", m[0]);
}

void
main()
{
	ulong *r;
	uchar *rr, s, s1;
	ulong addr;
	int twolane, fast, emph0, emph1, swing0, swing1, ctr, lasts, stage;
	
	twolane = 1;
	fast = 1;
	emph0 = 0;
	swing0 = 0;
	emph1 = 0;
	swing1 = 0;
	
	mmio = segattach(0, "axi", nil, 1048576*4);
	if(mmio == (uchar*)-1)
		sysfatal("segattach: %r");

	dp_write(CTRL, 0);
	dp_write(MISC, 0x21);

	dp_write(HVACT, 1280 << 16 | 1024);
	dp_write(HVTOT, 1712 << 16 | 1060);
	dp_write(HVSYNC, 136 << 16 | 3);
	dp_write(HVDATA, 352 << 16 | 35);
	dp_write(MVID0, 41);
	dp_write(NVID0, 61);
	dp_write(SCLK0, 44049);
	dp_write(MVID1, 25);
	dp_write(NVID1, 62);
	dp_write(SCLK1, 26426);
	
	ctr = 0;
	lasts = -1;
	fast = 1;
	twolane = 1;
	aux_write(LINK_BW_SET, fast ? 0x0A : 0x06);
	aux_write(LANE_COUNT_SET, twolane + 1);
	stage = 0;
	for(;;){
		print("stage %d\n", stage);
		dp_write(CTRL, 1 << 17 | (stage == 1 ? 0x3000 : 0x2000) | swing1 << 10 | swing0 << 8 | emph1 << 6 | emph0 << 4 | twolane << 1 | fast);
		aux_write(TRAINING_PATTERN_SET, 1 + stage);
		aux_write(TRAINING_LANE0_SET, (emph0 == 3) << 6 | emph0 << 3 | (swing0 == 3) << 2 | swing0);
		aux_write(TRAINING_LANE1_SET, (emph1 == 3) << 6 | emph1 << 3 | (swing1 == 3) << 2 | swing1);
		sleep(10);
		s = aux_read(LANE0_1_STATUS);
		if(stage == 0){
			if((s & LANE0_CR_DONE) != 0 && (!twolane || (s & (LANE1_CR_DONE) != 0))){
				stage = 1;
				lasts = -1;
			}
		}else{
			if((s & LANE0_CR_DONE) == 0 || twolane && (s & LANE1_CR_DONE) == 0)
				stage = 0;
			else if((~s & (LANE0_CHANNEL_EQ_DONE | LANE0_SYMBOL_LOCKED | (twolane * (LANE1_CHANNEL_EQ_DONE | LANE1_SYMBOL_LOCKED)))) == 0){
				if(aux_read(LANE_ALIGN_STATUS_UPDATED) & INTERLANE_ALIGN_DONE)
					break;
			}
		}
		s = aux_read(ADJUST_REQUEST_LANE0_1);
		if(s == lasts){
			if(++ctr == 5)
				sysfatal("training failed");
		}else
			ctr = 0;
		lasts = s;
		swing0 = s & 3;
		emph0 = s >> 2 & 3;
		swing1 = s >> 4 & 3;
		emph1 = s >> 6 & 3;
	}
	dp_write(CTRL, 0x1000 | swing1 << 10 | swing0 << 8 | emph1 << 6 | emph0 << 4 | twolane << 1 | fast);
	aux_write(TRAINING_PATTERN_SET, 0);
	for(;;){
		sleep(1000);
		aux_read(0x205);
	}
}
