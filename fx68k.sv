/* verilator lint_off WIDTH */
//
// FX68K
//
// M68000 cycle accurate, fully synchronous
// Copyright (c) 2018 by Jorge Cwik
// 
// TODO:
// - Everything except bus retry already implemented.

`timescale 1 ns / 1 ns
`default_nettype wire

// Define this to run a self contained compilation test build
// `define FX68K_TEST

localparam CF = 0, VF = 1, ZF = 2, NF = 3, XF = 4, SF = 13;

localparam UADDR_WIDTH = 10;
localparam UROM_WIDTH = 17;
localparam UROM_DEPTH = 1024;

localparam NADDR_WIDTH = 9;
localparam NANO_WIDTH = 68;
localparam NANO_DEPTH = 336;

localparam BSER1_NMA = 'h003;
localparam RSTP0_NMA = 'h002;
localparam HALT1_NMA = 'h001;
localparam TRAC1_NMA = 'h1C0;
localparam ITLX1_NMA = 'h1C4;

localparam TVN_SPURIOUS = 12;
localparam TVN_AUTOVEC = 13;
localparam TVN_INTERRUPT = 15;

localparam NANO_DOB_DBD = 2'b01;
localparam NANO_DOB_ADB = 2'b10;
localparam NANO_DOB_ALU = 2'b11;

module fx68k(
	input wire clk,
	
	// These two signals don't need to be registered. They are not async reset.
	input wire extReset,			// External sync reset on emulated system
	input wire pwrUp,			// Asserted together with reset on emulated system coldstart	
	input wire enPhi1, enPhi2,	// Clock enables. Next cycle is PHI1 or PHI2

	output wire eRWn, output wire ASn, output wire LDSn, output wire UDSn,
	output reg E, output wire VMAn,	
	output wire FC0, output wire FC1, output wire FC2,
	output wire BGn,
	output wire oRESETn, output wire oHALTEDn,
	input wire DTACKn,
	input wire VPAn,
	input wire BERRn,
	input wire BRn, BGACKn,
	input wire IPL0n, input IPL1n, input IPL2n,
	input wire [15:0] iEdb,
	output wire [15:0] oEdb,
	output wire [23:1] eab
	);
	
	wire wClk;
	
	// Internal sub clocks T1-T4
	enum int unsigned { T0 = 0, T1, T2, T3, T4} tState;
	wire enT1 = enPhi1 & (tState == T4) & ~wClk;
	wire enT2 = enPhi2 & (tState == T1);
	wire enT3 = enPhi1 & (tState == T2);
	wire enT4 = enPhi2 & ((tState == T0) | (tState == T3));
	
	// T4 continues ticking during reset and group0 exception.
	// We also need it to erase ucode output latched on T4.
	always_ff @( posedge clk) begin
		if( pwrUp)
			tState <= T0;
		else begin
		case( tState)
		T0: if( enPhi2) tState <= T4;
		T1: if( enPhi2) tState <= T2;
		T2: if( enPhi1) tState <= T3;
		T3: if( enPhi2) tState <= T4;
		T4: if( enPhi1) tState <= wClk ? T0 : T1;
		endcase
		end
	end
	
	// The following signals are synchronized with 3 couplers, phi1-phi2-phi1.
	// Will be valid internally one cycle later if changed at the rasing edge of the clock.
	//
	// DTACK, BERR
		
	// DTACK valid at S6 if changed at the rasing edge of S4 to avoid wait states.
	// SNC (sncClkEn) is deasserted together (unless DTACK asserted too early).
	//	
	// We synchronize some signals half clock earlier. We compensate later
	reg rDtack, rBerr;
	reg [2:0] rIpl, iIpl;
	reg Vpai, BeI, BRi, BgackI, BeiDelay;
	// reg rBR;
	wire BeDebounced = ~( BeI | BeiDelay);

	always_ff @( posedge clk) begin
		if( pwrUp) begin
			rBerr <= 1'b0;
			BeI <= 1'b0;
		end
		else if( enPhi2) begin
			rDtack <= DTACKn;
			rBerr <= BERRn;
			rIpl <= ~{ IPL2n, IPL1n, IPL0n};
			iIpl <= rIpl;
			
			// rBR <= BRn;			// Needed for cycle accuracy but only if BR is changed on the wrong edge of the clock
		end
		else if( enPhi1) begin
			Vpai <= VPAn;
			BeI <= rBerr;
			BeiDelay <= BeI;
			
			BRi <= BRn;
			BgackI <= BGACKn;
			// BRi <= rBR;
		end	
	end

	// Instantiate micro and nano rom
	logic [NANO_WIDTH-1:0] nanoLatch;
	logic [NANO_WIDTH-1:0] nanoOutput;
	logic [UROM_WIDTH-1:0] microLatch;
	logic [UROM_WIDTH-1:0] microOutput;
	
	logic [UADDR_WIDTH-1:0] microAddr, nma; 
	logic [NADDR_WIDTH-1:0] nanoAddr, orgAddr;
	wire rstUrom;

	// For the time being, address translation is done for nanorom only. 
	microToNanoAddr microToNanoAddr( .uAddr( nma), .orgAddr);
	
	// Output of these modules will be updated at T2 at the latest (depending on clock division)
	
	nanoRom nanoRom( .clk( clk), .nanoAddr, .nanoOutput);
	uRom uRom( .clk( clk), .microAddr, .microOutput);
	
	always_ff @( posedge clk) begin
		// uaddr originally latched on T1, except bits 6 & 7, the conditional bits, on T2
		// Seems we can latch whole address at either T1 or T2

		// Originally it's invalid on hardware reset, and forced later when coming out of reset
		if( pwrUp) begin
			microAddr <= RSTP0_NMA;
			nanoAddr <= RSTP0_NMA;
		end
		else if( enT1) begin
			microAddr <= nma;
			nanoAddr <= orgAddr;				// Register translated uaddr to naddr
		end
			
		if( extReset) begin
			microLatch <= '0;
			nanoLatch <= '0;
		end
		else if( rstUrom) begin
			// Originally reset these bits only. Not strictly needed like this.
			// Can reset the whole register if it is important.
			{ microLatch[16], microLatch[15], microLatch[0]} <= '0;
			nanoLatch <= '0;
		end
		else if( enT3) begin
			microLatch <= microOutput;
			nanoLatch <= nanoOutput;
		end
					
	end
	

	// Decoded nanocode signals

	logic Nanod_permStart;
	logic Nanod_waitBusFinish;
	logic Nanod_isWrite;
	logic Nanod_busByte;
	logic Nanod_isRmc;
	logic Nanod_noLowByte, Nanod_noHighByte;
	
	logic Nanod_updTpend, Nanod_clrTpend;
	logic Nanod_tvn2Ftu, Nanod_const2Ftu;
	logic Nanod_ftu2Dbl, Nanod_ftu2Abl;
	logic Nanod_abl2Pren, Nanod_updPren;
	logic Nanod_inl2psw, Nanod_ftu2Sr, Nanod_sr2Ftu, Nanod_ftu2Ccr, Nanod_pswIToFtu;
	logic Nanod_ird2Ftu, Nanod_ssw2Ftu;
	logic Nanod_initST;
	logic Nanod_Ir2Ird;
	
	logic Nanod_auClkEn, Nanod_noSpAlign;
	logic [2:0] Nanod_auCntrl;
	logic Nanod_todbin, Nanod_toIrc;
	logic Nanod_dbl2Atl, Nanod_abl2Atl, Nanod_atl2Abl, Nanod_atl2Dbl;
	logic Nanod_abh2Ath, Nanod_dbh2Ath;
	logic Nanod_ath2Dbh, Nanod_ath2Abh;
	
	logic Nanod_db2Aob, Nanod_ab2Aob, Nanod_au2Aob;
	logic Nanod_aob2Ab, Nanod_updSsw;
	// logic Nanod_adb2Dob, Nanod_dbd2Dob, Nanod_alu2Dob;
	logic [1:0] Nanod_dobCtrl;
	
	logic Nanod_abh2reg, Nanod_abl2reg;
	logic Nanod_reg2abl, Nanod_reg2abh;
	logic Nanod_dbh2reg, Nanod_dbl2reg;
	logic Nanod_reg2dbl, Nanod_reg2dbh;
	logic Nanod_ssp, Nanod_pchdbh, Nanod_pcldbl, Nanod_pclabl, Nanod_pchabh;
	
	logic Nanod_rxh2dbh, Nanod_rxh2abh;
	logic Nanod_dbl2rxl, Nanod_dbh2rxh;
	logic Nanod_rxl2db, Nanod_rxl2ab;
	logic Nanod_abl2rxl, Nanod_abh2rxh;
	logic Nanod_dbh2ryh, Nanod_abh2ryh;
	logic Nanod_ryl2db, Nanod_ryl2ab;
	logic Nanod_ryh2dbh, Nanod_ryh2abh;
	logic Nanod_dbl2ryl, Nanod_abl2ryl;
	logic Nanod_rz;
	logic Nanod_rxlDbl;
	
	logic [2:0] Nanod_aluColumn;
	logic [1:0] Nanod_aluDctrl;
	logic Nanod_aluActrl;
	logic Nanod_aluInit, Nanod_aluFinish;
	logic Nanod_abd2Dcr, Nanod_dcr2Dbd;
	logic Nanod_dbd2Alue, Nanod_alue2Dbd;
	logic Nanod_dbd2Alub, Nanod_abd2Alub;
	
	logic Nanod_alu2Dbd, Nanod_alu2Abd;
	logic Nanod_au2Db, Nanod_au2Ab, Nanod_au2Pc;
	logic Nanod_dbin2Abd, Nanod_dbin2Dbd;
	logic Nanod_extDbh, Nanod_extAbh;
	logic Nanod_ablAbd, Nanod_ablAbh;
	logic Nanod_dblDbd, Nanod_dblDbh;
	logic Nanod_abdIsByte;

	// IRD decoded control signals
	logic Irdecod_isPcRel;
	logic Irdecod_isTas;
	logic Irdecod_implicitSp;
	logic Irdecod_toCcr;
	logic Irdecod_rxIsDt, Irdecod_ryIsDt;
	logic Irdecod_rxIsUsp, Irdecod_rxIsMovem, Irdecod_movemPreDecr;
	logic Irdecod_isByte;
	logic Irdecod_isMovep;
	logic [2:0] Irdecod_rx, Irdecod_ry;
	logic Irdecod_rxIsAreg, Irdecod_ryIsAreg;
	logic [15:0] Irdecod_ftuConst;
	logic [5:0] Irdecod_macroTvn;
	logic Irdecod_inhibitCcr;

	//	
	reg Tpend;
	reg intPend;											// Interrupt pending
	reg pswT, pswS;
	reg [ 2:0] pswI;
	wire [7:0] ccr;
	
	wire [15:0] psw = { pswT, 1'b0, pswS, 2'b00, pswI, ccr};

	reg [15:0] ftu;
	reg [15:0] Irc, Ir, Ird;
	
	wire [15:0] alue;
	wire [15:0] Abl;
	wire prenEmpty, au05z, dcr4, ze;
	
	wire [UADDR_WIDTH-1:0] a1, a2, a3;
	wire isPriv, isIllegal, isLineA, isLineF;
	

	// IR & IRD forwarding
	always_ff @( posedge clk) begin
		if( enT1) begin
			if( Nanod_Ir2Ird)
				Ird <= Ir;
			else if(microLatch[0])		// prevented by IR => IRD !
				Ir <= Irc;
		end
	end
	
	wire [3:0] tvn;
	wire waitBusCycle, busStarting;
	wire BusRetry = 1'b0;
	wire busAddrErr;
	wire bciWrite;						// Last bus cycle was write
	wire bgBlock, busAvail;
	wire addrOe;

	wire busIsByte = Nanod_busByte & (Irdecod_isByte | Irdecod_isMovep);
	wire aob0;
	
	reg iStop;								// Internal signal for ending bus cycle
	reg A0Err;								// Force bus/address error ucode
	reg excRst;								// Signal reset exception to sequencer
	reg BerrA;
	reg Spuria, Avia;
	wire Iac;
	
	reg rAddrErr, iBusErr, Err6591;
	wire iAddrErr = rAddrErr & addrOe;		// To simulate async reset
	wire enErrClk;

	// Reset micro/nano latch after T4 of the current ublock.
	assign rstUrom = enPhi1 & enErrClk;

	uaddrDecode uaddrDecode( .opcode( Ir), .a1, .a2, .a3, .isPriv, .isIllegal, .isLineA, .isLineF, .lineBmap());

	sequencer sequencer(.alue01( alue[1:0]), .i11( Irc[ 11]), /*AUTOINST*/
			    // Outputs
			    .tvn		(tvn[3:0]),
			    .nma		(nma[UADDR_WIDTH-1:0]),
			    // Inputs
			    .clk		(clk),
			    .enPhi1		(enPhi1),
			    .enPhi2		(enPhi2),
			    .extReset		(extReset),
			    .pwrUp		(pwrUp),
			    .enT3		(enT3),
			    .microLatch		(microLatch[UROM_WIDTH-1:0]),
			    .A0Err		(A0Err),
			    .BerrA		(BerrA),
			    .busAddrErr		(busAddrErr),
			    .Spuria		(Spuria),
			    .Avia		(Avia),
			    .Tpend		(Tpend),
			    .intPend		(intPend),
			    .isIllegal		(isIllegal),
			    .isPriv		(isPriv),
			    .excRst		(excRst),
			    .isLineA		(isLineA),
			    .isLineF		(isLineF),
			    .psw		(psw[15:0]),
			    .prenEmpty		(prenEmpty),
			    .au05z		(au05z),
			    .dcr4		(dcr4),
			    .ze			(ze),
			    .Ird		(Ird[15:0]),
			    .a1			(a1[UADDR_WIDTH-1:0]),
			    .a2			(a2[UADDR_WIDTH-1:0]),
			    .a3			(a3[UADDR_WIDTH-1:0]));

	excUnit excUnit( .AblOut( Abl), /*AUTOINST*/
			// Outputs
			.ccr		(ccr[7:0]),
			.alue		(alue[15:0]),
			.prenEmpty	(prenEmpty),
			.au05z		(au05z),
			.dcr4		(dcr4),
			.ze		(ze),
			.aob0		(aob0),
			.Irc		(Irc[15:0]),
			.oEdb		(oEdb[15:0]),
			.eab		(eab[23:1]),
			// Inputs
			.clk		(clk),
			.enPhi1		(enPhi1),
			.enPhi2		(enPhi2),
			.extReset	(extReset),
			.pwrUp		(pwrUp),
			.enT1		(enT1),
			.enT2		(enT2),
			.enT3		(enT3),
			.enT4		(enT4),
			.Nanod_permStart(Nanod_permStart),
			.Nanod_waitBusFinish(Nanod_waitBusFinish),
			.Nanod_isWrite	(Nanod_isWrite),
			.Nanod_busByte	(Nanod_busByte),
			.Nanod_isRmc	(Nanod_isRmc),
			.Nanod_noLowByte(Nanod_noLowByte),
			.Nanod_noHighByte(Nanod_noHighByte),
			.Nanod_updTpend	(Nanod_updTpend),
			.Nanod_clrTpend	(Nanod_clrTpend),
			.Nanod_tvn2Ftu	(Nanod_tvn2Ftu),
			.Nanod_const2Ftu(Nanod_const2Ftu),
			.Nanod_ftu2Dbl	(Nanod_ftu2Dbl),
			.Nanod_ftu2Abl	(Nanod_ftu2Abl),
			.Nanod_abl2Pren	(Nanod_abl2Pren),
			.Nanod_updPren	(Nanod_updPren),
			.Nanod_inl2psw	(Nanod_inl2psw),
			.Nanod_ftu2Sr	(Nanod_ftu2Sr),
			.Nanod_sr2Ftu	(Nanod_sr2Ftu),
			.Nanod_ftu2Ccr	(Nanod_ftu2Ccr),
			.Nanod_pswIToFtu(Nanod_pswIToFtu),
			.Nanod_ird2Ftu	(Nanod_ird2Ftu),
			.Nanod_ssw2Ftu	(Nanod_ssw2Ftu),
			.Nanod_initST	(Nanod_initST),
			.Nanod_Ir2Ird	(Nanod_Ir2Ird),
			.Nanod_auClkEn	(Nanod_auClkEn),
			.Nanod_noSpAlign(Nanod_noSpAlign),
			.Nanod_auCntrl	(Nanod_auCntrl[2:0]),
			.Nanod_todbin	(Nanod_todbin),
			.Nanod_toIrc	(Nanod_toIrc),
			.Nanod_dbl2Atl	(Nanod_dbl2Atl),
			.Nanod_abl2Atl	(Nanod_abl2Atl),
			.Nanod_atl2Abl	(Nanod_atl2Abl),
			.Nanod_atl2Dbl	(Nanod_atl2Dbl),
			.Nanod_abh2Ath	(Nanod_abh2Ath),
			.Nanod_dbh2Ath	(Nanod_dbh2Ath),
			.Nanod_ath2Dbh	(Nanod_ath2Dbh),
			.Nanod_ath2Abh	(Nanod_ath2Abh),
			.Nanod_db2Aob	(Nanod_db2Aob),
			.Nanod_ab2Aob	(Nanod_ab2Aob),
			.Nanod_au2Aob	(Nanod_au2Aob),
			.Nanod_aob2Ab	(Nanod_aob2Ab),
			.Nanod_updSsw	(Nanod_updSsw),
			.Nanod_dobCtrl	(Nanod_dobCtrl[1:0]),
			.Nanod_abh2reg	(Nanod_abh2reg),
			.Nanod_abl2reg	(Nanod_abl2reg),
			.Nanod_reg2abl	(Nanod_reg2abl),
			.Nanod_reg2abh	(Nanod_reg2abh),
			.Nanod_dbh2reg	(Nanod_dbh2reg),
			.Nanod_dbl2reg	(Nanod_dbl2reg),
			.Nanod_reg2dbl	(Nanod_reg2dbl),
			.Nanod_reg2dbh	(Nanod_reg2dbh),
			.Nanod_ssp	(Nanod_ssp),
			.Nanod_pchdbh	(Nanod_pchdbh),
			.Nanod_pcldbl	(Nanod_pcldbl),
			.Nanod_pclabl	(Nanod_pclabl),
			.Nanod_pchabh	(Nanod_pchabh),
			.Nanod_rxh2dbh	(Nanod_rxh2dbh),
			.Nanod_rxh2abh	(Nanod_rxh2abh),
			.Nanod_dbl2rxl	(Nanod_dbl2rxl),
			.Nanod_dbh2rxh	(Nanod_dbh2rxh),
			.Nanod_rxl2db	(Nanod_rxl2db),
			.Nanod_rxl2ab	(Nanod_rxl2ab),
			.Nanod_abl2rxl	(Nanod_abl2rxl),
			.Nanod_abh2rxh	(Nanod_abh2rxh),
			.Nanod_dbh2ryh	(Nanod_dbh2ryh),
			.Nanod_abh2ryh	(Nanod_abh2ryh),
			.Nanod_ryl2db	(Nanod_ryl2db),
			.Nanod_ryl2ab	(Nanod_ryl2ab),
			.Nanod_ryh2dbh	(Nanod_ryh2dbh),
			.Nanod_ryh2abh	(Nanod_ryh2abh),
			.Nanod_dbl2ryl	(Nanod_dbl2ryl),
			.Nanod_abl2ryl	(Nanod_abl2ryl),
			.Nanod_rz	(Nanod_rz),
			.Nanod_rxlDbl	(Nanod_rxlDbl),
			.Nanod_aluColumn(Nanod_aluColumn[2:0]),
			.Nanod_aluDctrl	(Nanod_aluDctrl[1:0]),
			.Nanod_aluActrl	(Nanod_aluActrl),
			.Nanod_aluInit	(Nanod_aluInit),
			.Nanod_aluFinish(Nanod_aluFinish),
			.Nanod_abd2Dcr	(Nanod_abd2Dcr),
			.Nanod_dcr2Dbd	(Nanod_dcr2Dbd),
			.Nanod_dbd2Alue	(Nanod_dbd2Alue),
			.Nanod_alue2Dbd	(Nanod_alue2Dbd),
			.Nanod_dbd2Alub	(Nanod_dbd2Alub),
			.Nanod_abd2Alub	(Nanod_abd2Alub),
			.Nanod_alu2Dbd	(Nanod_alu2Dbd),
			.Nanod_alu2Abd	(Nanod_alu2Abd),
			.Nanod_au2Db	(Nanod_au2Db),
			.Nanod_au2Ab	(Nanod_au2Ab),
			.Nanod_au2Pc	(Nanod_au2Pc),
			.Nanod_dbin2Abd	(Nanod_dbin2Abd),
			.Nanod_dbin2Dbd	(Nanod_dbin2Dbd),
			.Nanod_extDbh	(Nanod_extDbh),
			.Nanod_extAbh	(Nanod_extAbh),
			.Nanod_ablAbd	(Nanod_ablAbd),
			.Nanod_ablAbh	(Nanod_ablAbh),
			.Nanod_dblDbd	(Nanod_dblDbd),
			.Nanod_dblDbh	(Nanod_dblDbh),
			.Nanod_abdIsByte(Nanod_abdIsByte),
			.Irdecod_isPcRel(Irdecod_isPcRel),
			.Irdecod_isTas	(Irdecod_isTas),
			.Irdecod_implicitSp(Irdecod_implicitSp),
			.Irdecod_toCcr	(Irdecod_toCcr),
			.Irdecod_rxIsDt	(Irdecod_rxIsDt),
			.Irdecod_ryIsDt	(Irdecod_ryIsDt),
			.Irdecod_rxIsUsp(Irdecod_rxIsUsp),
			.Irdecod_rxIsMovem(Irdecod_rxIsMovem),
			.Irdecod_movemPreDecr(Irdecod_movemPreDecr),
			.Irdecod_isByte	(Irdecod_isByte),
			.Irdecod_isMovep(Irdecod_isMovep),
			.Irdecod_rx	(Irdecod_rx[2:0]),
			.Irdecod_ry	(Irdecod_ry[2:0]),
			.Irdecod_rxIsAreg(Irdecod_rxIsAreg),
			.Irdecod_ryIsAreg(Irdecod_ryIsAreg),
			.Irdecod_ftuConst(Irdecod_ftuConst[15:0]),
			.Irdecod_macroTvn(Irdecod_macroTvn[5:0]),
			.Irdecod_inhibitCcr(Irdecod_inhibitCcr),
			.Ird		(Ird[15:0]),
			.pswS		(pswS),
			.ftu		(ftu[15:0]),
			.iEdb		(iEdb[15:0]));

	nDecoder3 nDecoder( /*AUTOINST*/
			   // Outputs
			   .Nanod_permStart	(Nanod_permStart),
			   .Nanod_waitBusFinish	(Nanod_waitBusFinish),
			   .Nanod_isWrite	(Nanod_isWrite),
			   .Nanod_busByte	(Nanod_busByte),
			   .Nanod_isRmc		(Nanod_isRmc),
			   .Nanod_noLowByte	(Nanod_noLowByte),
			   .Nanod_noHighByte	(Nanod_noHighByte),
			   .Nanod_updTpend	(Nanod_updTpend),
			   .Nanod_clrTpend	(Nanod_clrTpend),
			   .Nanod_tvn2Ftu	(Nanod_tvn2Ftu),
			   .Nanod_const2Ftu	(Nanod_const2Ftu),
			   .Nanod_ftu2Dbl	(Nanod_ftu2Dbl),
			   .Nanod_ftu2Abl	(Nanod_ftu2Abl),
			   .Nanod_abl2Pren	(Nanod_abl2Pren),
			   .Nanod_updPren	(Nanod_updPren),
			   .Nanod_inl2psw	(Nanod_inl2psw),
			   .Nanod_ftu2Sr	(Nanod_ftu2Sr),
			   .Nanod_sr2Ftu	(Nanod_sr2Ftu),
			   .Nanod_ftu2Ccr	(Nanod_ftu2Ccr),
			   .Nanod_pswIToFtu	(Nanod_pswIToFtu),
			   .Nanod_ird2Ftu	(Nanod_ird2Ftu),
			   .Nanod_ssw2Ftu	(Nanod_ssw2Ftu),
			   .Nanod_initST	(Nanod_initST),
			   .Nanod_Ir2Ird	(Nanod_Ir2Ird),
			   .Nanod_auClkEn	(Nanod_auClkEn),
			   .Nanod_noSpAlign	(Nanod_noSpAlign),
			   .Nanod_auCntrl	(Nanod_auCntrl[2:0]),
			   .Nanod_todbin	(Nanod_todbin),
			   .Nanod_toIrc		(Nanod_toIrc),
			   .Nanod_dbl2Atl	(Nanod_dbl2Atl),
			   .Nanod_abl2Atl	(Nanod_abl2Atl),
			   .Nanod_atl2Abl	(Nanod_atl2Abl),
			   .Nanod_atl2Dbl	(Nanod_atl2Dbl),
			   .Nanod_abh2Ath	(Nanod_abh2Ath),
			   .Nanod_dbh2Ath	(Nanod_dbh2Ath),
			   .Nanod_ath2Dbh	(Nanod_ath2Dbh),
			   .Nanod_ath2Abh	(Nanod_ath2Abh),
			   .Nanod_db2Aob	(Nanod_db2Aob),
			   .Nanod_ab2Aob	(Nanod_ab2Aob),
			   .Nanod_au2Aob	(Nanod_au2Aob),
			   .Nanod_aob2Ab	(Nanod_aob2Ab),
			   .Nanod_updSsw	(Nanod_updSsw),
			   .Nanod_dobCtrl	(Nanod_dobCtrl[1:0]),
			   .Nanod_abh2reg	(Nanod_abh2reg),
			   .Nanod_abl2reg	(Nanod_abl2reg),
			   .Nanod_reg2abl	(Nanod_reg2abl),
			   .Nanod_reg2abh	(Nanod_reg2abh),
			   .Nanod_dbh2reg	(Nanod_dbh2reg),
			   .Nanod_dbl2reg	(Nanod_dbl2reg),
			   .Nanod_reg2dbl	(Nanod_reg2dbl),
			   .Nanod_reg2dbh	(Nanod_reg2dbh),
			   .Nanod_ssp		(Nanod_ssp),
			   .Nanod_pchdbh	(Nanod_pchdbh),
			   .Nanod_pcldbl	(Nanod_pcldbl),
			   .Nanod_pclabl	(Nanod_pclabl),
			   .Nanod_pchabh	(Nanod_pchabh),
			   .Nanod_rxh2dbh	(Nanod_rxh2dbh),
			   .Nanod_rxh2abh	(Nanod_rxh2abh),
			   .Nanod_dbl2rxl	(Nanod_dbl2rxl),
			   .Nanod_dbh2rxh	(Nanod_dbh2rxh),
			   .Nanod_rxl2db	(Nanod_rxl2db),
			   .Nanod_rxl2ab	(Nanod_rxl2ab),
			   .Nanod_abl2rxl	(Nanod_abl2rxl),
			   .Nanod_abh2rxh	(Nanod_abh2rxh),
			   .Nanod_dbh2ryh	(Nanod_dbh2ryh),
			   .Nanod_abh2ryh	(Nanod_abh2ryh),
			   .Nanod_ryl2db	(Nanod_ryl2db),
			   .Nanod_ryl2ab	(Nanod_ryl2ab),
			   .Nanod_ryh2dbh	(Nanod_ryh2dbh),
			   .Nanod_ryh2abh	(Nanod_ryh2abh),
			   .Nanod_dbl2ryl	(Nanod_dbl2ryl),
			   .Nanod_abl2ryl	(Nanod_abl2ryl),
			   .Nanod_rz		(Nanod_rz),
			   .Nanod_rxlDbl	(Nanod_rxlDbl),
			   .Nanod_aluColumn	(Nanod_aluColumn[2:0]),
			   .Nanod_aluDctrl	(Nanod_aluDctrl[1:0]),
			   .Nanod_aluActrl	(Nanod_aluActrl),
			   .Nanod_aluInit	(Nanod_aluInit),
			   .Nanod_aluFinish	(Nanod_aluFinish),
			   .Nanod_abd2Dcr	(Nanod_abd2Dcr),
			   .Nanod_dcr2Dbd	(Nanod_dcr2Dbd),
			   .Nanod_dbd2Alue	(Nanod_dbd2Alue),
			   .Nanod_alue2Dbd	(Nanod_alue2Dbd),
			   .Nanod_dbd2Alub	(Nanod_dbd2Alub),
			   .Nanod_abd2Alub	(Nanod_abd2Alub),
			   .Nanod_alu2Dbd	(Nanod_alu2Dbd),
			   .Nanod_alu2Abd	(Nanod_alu2Abd),
			   .Nanod_au2Db		(Nanod_au2Db),
			   .Nanod_au2Ab		(Nanod_au2Ab),
			   .Nanod_au2Pc		(Nanod_au2Pc),
			   .Nanod_dbin2Abd	(Nanod_dbin2Abd),
			   .Nanod_dbin2Dbd	(Nanod_dbin2Dbd),
			   .Nanod_extDbh	(Nanod_extDbh),
			   .Nanod_extAbh	(Nanod_extAbh),
			   .Nanod_ablAbd	(Nanod_ablAbd),
			   .Nanod_ablAbh	(Nanod_ablAbh),
			   .Nanod_dblDbd	(Nanod_dblDbd),
			   .Nanod_dblDbh	(Nanod_dblDbh),
			   .Nanod_abdIsByte	(Nanod_abdIsByte),
			   // Inputs
			   .clk			(clk),
			   .Irdecod_isPcRel	(Irdecod_isPcRel),
			   .Irdecod_isTas	(Irdecod_isTas),
			   .Irdecod_implicitSp	(Irdecod_implicitSp),
			   .Irdecod_toCcr	(Irdecod_toCcr),
			   .Irdecod_rxIsDt	(Irdecod_rxIsDt),
			   .Irdecod_ryIsDt	(Irdecod_ryIsDt),
			   .Irdecod_rxIsUsp	(Irdecod_rxIsUsp),
			   .Irdecod_rxIsMovem	(Irdecod_rxIsMovem),
			   .Irdecod_movemPreDecr(Irdecod_movemPreDecr),
			   .Irdecod_isByte	(Irdecod_isByte),
			   .Irdecod_isMovep	(Irdecod_isMovep),
			   .Irdecod_rx		(Irdecod_rx[2:0]),
			   .Irdecod_ry		(Irdecod_ry[2:0]),
			   .Irdecod_rxIsAreg	(Irdecod_rxIsAreg),
			   .Irdecod_ryIsAreg	(Irdecod_ryIsAreg),
			   .Irdecod_ftuConst	(Irdecod_ftuConst[15:0]),
			   .Irdecod_macroTvn	(Irdecod_macroTvn[5:0]),
			   .Irdecod_inhibitCcr	(Irdecod_inhibitCcr),
			   .enT2		(enT2),
			   .enT4		(enT4),
			   .microLatch		(microLatch[UROM_WIDTH-1:0]),
			   .nanoLatch		(nanoLatch[NANO_WIDTH-1:0]));
	
	irdDecode irdDecode( .ird( Ird), /*AUTOINST*/
			    // Outputs
			    .Irdecod_isPcRel	(Irdecod_isPcRel),
			    .Irdecod_isTas	(Irdecod_isTas),
			    .Irdecod_implicitSp	(Irdecod_implicitSp),
			    .Irdecod_toCcr	(Irdecod_toCcr),
			    .Irdecod_rxIsDt	(Irdecod_rxIsDt),
			    .Irdecod_ryIsDt	(Irdecod_ryIsDt),
			    .Irdecod_rxIsUsp	(Irdecod_rxIsUsp),
			    .Irdecod_rxIsMovem	(Irdecod_rxIsMovem),
			    .Irdecod_movemPreDecr(Irdecod_movemPreDecr),
			    .Irdecod_isByte	(Irdecod_isByte),
			    .Irdecod_isMovep	(Irdecod_isMovep),
			    .Irdecod_rx		(Irdecod_rx[2:0]),
			    .Irdecod_ry		(Irdecod_ry[2:0]),
			    .Irdecod_rxIsAreg	(Irdecod_rxIsAreg),
			    .Irdecod_ryIsAreg	(Irdecod_ryIsAreg),
			    .Irdecod_ftuConst	(Irdecod_ftuConst[15:0]),
			    .Irdecod_macroTvn	(Irdecod_macroTvn[5:0]),
			    .Irdecod_inhibitCcr	(Irdecod_inhibitCcr));
	
	busControl busControl( .permStart( Nanod_permStart), .permStop( Nanod_waitBusFinish),
		.isWrite( Nanod_isWrite), .isRmc( Nanod_isRmc), .isByte( busIsByte), /*AUTOINST*/
			      // Outputs
			      .bgBlock		(bgBlock),
			      .busAddrErr	(busAddrErr),
			      .waitBusCycle	(waitBusCycle),
			      .busStarting	(busStarting),
			      .addrOe		(addrOe),
			      .bciWrite		(bciWrite),
			      .ASn		(ASn),
			      .LDSn		(LDSn),
			      .UDSn		(UDSn),
			      .eRWn		(eRWn),
			      // Inputs
			      .clk		(clk),
			      .enPhi1		(enPhi1),
			      .enPhi2		(enPhi2),
			      .extReset		(extReset),
			      .pwrUp		(pwrUp),
			      .enT1		(enT1),
			      .enT4		(enT4),
			      .iStop		(iStop),
			      .aob0		(aob0),
			      .busAvail		(busAvail),
			      .rDtack		(rDtack),
			      .BeDebounced	(BeDebounced),
			      .Vpai		(Vpai));
		
	busArbiter busArbiter( .Halti( 1'b1), /*AUTOINST*/
			      // Outputs
			      .busAvail		(busAvail),
			      .BGn		(BGn),
			      // Inputs
			      .clk		(clk),
			      .enPhi1		(enPhi1),
			      .enPhi2		(enPhi2),
			      .extReset		(extReset),
			      .pwrUp		(pwrUp),
			      .BRi		(BRi),
			      .BgackI		(BgackI),
			      .bgBlock		(bgBlock));
		
		
	// Output reset & halt control
	wire [1:0] uFc = microLatch[ 16:15];
	logic oReset, oHalted;
	assign oRESETn = !oReset;
	assign oHALTEDn = !oHalted;
	
	// FC without permStart is special, either reset or halt
	always_ff @( posedge clk) begin
		if( pwrUp) begin
			oReset <= 1'b0;
			oHalted <= 1'b0;
		end
		else if( enT1) begin
			oReset <= (uFc == 2'b01) & !Nanod_permStart;
			oHalted <= (uFc == 2'b10) & !Nanod_permStart;
		end
	end
		
	logic [2:0] rFC;
	assign { FC2, FC1, FC0} = rFC;					// ~rFC;
	assign Iac = {rFC == 3'b111};					// & Control output enable !!

	always_ff @( posedge clk) begin
		if( extReset)
			rFC <= '0;
		else if( enT1 & Nanod_permStart) begin		// S0 phase of bus cycle
			rFC[2] <= pswS;
			// PC relativ access is marked as FC type 'n' (0) at ucode.
			// We don't care about RZ in this case. Those uinstructions with RZ don't start a bus cycle.
			rFC[1] <= microLatch[ 16] | ( ~microLatch[ 15] & ~Irdecod_isPcRel);
			rFC[0] <= microLatch[ 15] | ( ~microLatch[ 16] & Irdecod_isPcRel);
		end
	end
	
		
	// IPL interface
	reg [2:0] inl;							// Int level latch
	reg updIll;
	reg prevNmi;
	
	wire nmi = (iIpl == 3'b111);
	wire iplStable = (iIpl == rIpl);
	wire iplComp = iIpl > pswI;

	always_ff @( posedge clk) begin
		if( extReset) begin
			intPend <= 1'b0;
			prevNmi <= 1'b0;
		end
		else begin
			if( enPhi2)
				prevNmi <= nmi;
				
			// Originally async RS-Latch on PHI2, followed by a transparent latch on T2
			// Tricky because they might change simultaneously
			// Syncronous on PHI2 is equivalent as long as the output is read on T3!
			
			// Set on stable & NMI edge or compare
			// Clear on: NMI Iack or (stable & !NMI & !Compare)

			if( enPhi2) begin
				if( iplStable & ((nmi & ~prevNmi) | iplComp) )
					intPend <= 1'b1;
				else if( ((inl == 3'b111) & Iac) | (iplStable & !nmi & !iplComp) )
					intPend <= 1'b0;
			end			
		end
		
		if( extReset) begin
			inl <= '1;
			updIll <= 1'b0;
		end
		else if( enT4)
			updIll <= microLatch[0];		// Update on any IRC->IR
		else if( enT1 & updIll)
			inl <= iIpl;					// Timing is correct.
			
		// Spurious interrupt, BERR on Interrupt Ack.
		// Autovector interrupt. VPA on IACK.
		// Timing is tight. Spuria is deasserted just after exception exception is recorded.
		if( enT4) begin
			Spuria <= ~BeiDelay & Iac;
			Avia <= ~Vpai & Iac;
		end
			
	end
		
	assign enErrClk = iAddrErr | iBusErr;
	assign wClk = waitBusCycle | ~BeI | iAddrErr | Err6591;
	
	// E clock and counter, VMA
	reg [3:0] eCntr;
	reg rVma;
	
	assign VMAn = rVma;
	
	// Internal stop just one cycle before E falling edge
	wire xVma = ~rVma & (eCntr == 8);
	
	always_ff @( posedge clk) begin
		if( pwrUp) begin
			E <= 1'b0;
			eCntr <='0;
			rVma <= 1'b1;
		end
		if( enPhi2) begin
			if( eCntr == 9)
				E <= 1'b0;
			else if( eCntr == 5)
				E <= 1'b1;

			if( eCntr == 9)
				eCntr <= '0;
			else
				eCntr <= eCntr + 1'b1;
		end
		
		if( enPhi2 & addrOe & ~Vpai & (eCntr == 3))
			rVma <= 1'b0;
		else if( enPhi1 & eCntr == '0)
			rVma <= 1'b1;
	end
		
	always_ff @( posedge clk) begin
	
		// This timing is critical to stop the clock phases at the exact point on bus/addr error.
		// Timing should be such that current ublock completes (up to T3 or T4).
		// But T1 for the next ublock shouldn't happen. Next T1 only after resetting ucode and ncode latches.
		
		if( extReset)
			rAddrErr <= 1'b0;
		else if( enPhi1) begin
			if( busAddrErr & addrOe)		// Not on T1 ?!
				rAddrErr <= 1'b1;
			else if( ~addrOe)				// Actually async reset!
				rAddrErr <= 1'b0;
		end
		
		if( extReset)
			iBusErr <= 1'b0;
		else if( enPhi1) begin
			iBusErr <= ( BerrA & ~BeI & ~Iac & !BusRetry);
		end

		if( extReset)
			BerrA <= 1'b0;
		else if( enPhi2) begin
			if( ~BeI & ~Iac & addrOe)
				BerrA <= 1'b1;
			// else if( BeI & addrOe)			// Bad, async reset since addrOe raising edge
			else if( BeI & busStarting)			// So replaced with this that raises one cycle earlier
				BerrA <= 1'b0;
		end

		// Signal reset exception to sequencer.
		// Originally cleared on 1st T2 after permstart. Must keep it until TVN latched.
		if( extReset)
			excRst <= 1'b1;
		else if( enT2 & Nanod_permStart)
			excRst <= 1'b0;
		
		if( extReset)
			A0Err <= 1'b1;								// A0 Reset
		else if( enT3)									// Keep set until new urom words are being latched
			A0Err <= 1'b0;
		else if( enPhi1 & enErrClk & (busAddrErr | BerrA))		// Check bus error timing
			A0Err <= 1'b1;
		
		if( extReset) begin
			iStop <= 1'b0;
			Err6591 <= 1'b0;
		end
		else if( enPhi1)
			Err6591 <= enErrClk;
		else if( enPhi2)
			iStop <= xVma | (Vpai & (iAddrErr | ~rBerr));
	end
	
	// PSW
	logic irdToCcr_t4;
	always_ff @( posedge clk) begin				
		if( pwrUp) begin
			Tpend <= 1'b0;
			{pswT, pswS, pswI } <= '0;
			irdToCcr_t4 <= '0;
		end
		
		else if( enT4) begin
			irdToCcr_t4 <= Irdecod_toCcr;
		end
		
		else if( enT3) begin
		
			// UNIQUE IF !!	
			if( Nanod_updTpend)
				Tpend <= pswT;
			else if( Nanod_clrTpend)
				Tpend <= 1'b0;
			
			// UNIQUE IF !!	
			if( Nanod_ftu2Sr & !irdToCcr_t4)
				{pswT, pswS, pswI } <= { ftu[ 15], ftu[13], ftu[10:8]};
			else begin
				if( Nanod_initST) begin
					pswS <= 1'b1;
					pswT <= 1'b0;
				end
				if( Nanod_inl2psw)
					pswI <= inl;
			end
				
		end
	end
		
	// FTU
	reg [4:0] ssw;
	reg [3:0] tvnLatch;
	logic [15:0] tvnMux;
	reg inExcept01;
	
	// Seems CPU has a buglet here.
	// Flagging group 0 exceptions from TVN might not work because some bus cycles happen before TVN is updated.
	// But doesn't matter because a group 0 exception inside another one will halt the CPU anyway and won't save the SSW.
		
	always_ff @( posedge clk) begin
	
		// Updated at the start of the exception ucode
		if( Nanod_updSsw & enT3) begin
			ssw <= { ~bciWrite, inExcept01, rFC};
		end
	
		// Update TVN on T1 & IR=>IRD
		if( enT1 & Nanod_Ir2Ird) begin
			tvnLatch <= tvn;
			inExcept01 <= (tvn != 1);
		end
			
		if( pwrUp)
			ftu <= '0;
		else if( enT3) begin
			unique case( 1'b1)
			Nanod_tvn2Ftu:				ftu <= tvnMux;
			
			// 0 on unused bits seem to come from ftuConst PLA previously clearing FBUS
			Nanod_sr2Ftu:				ftu <= {pswT, 1'b0, pswS, 2'b00, pswI, 3'b000, ccr[4:0] };
			
			Nanod_ird2Ftu:				ftu <= Ird;
			Nanod_ssw2Ftu:				ftu[4:0] <= ssw;						// Undoc. Other bits must be preserved from IRD saved above!
			Nanod_pswIToFtu:			ftu <= { 12'hFFF, pswI, 1'b0};			// Interrupt level shifted
			Nanod_const2Ftu:			ftu <= Irdecod_ftuConst;
			Nanod_abl2Pren:				ftu <= Abl;								// From ALU or datareg. Used for SR modify
			default:					ftu <= ftu;
			endcase
		end
	end
	
	always_comb begin
		if( inExcept01) begin
			// Unique IF !!!
			if( tvnLatch == TVN_SPURIOUS)
				tvnMux = {9'b0, 5'd24, 2'b00};
			else if( tvnLatch == TVN_AUTOVEC)
				tvnMux = {9'b0, 2'b11, pswI, 2'b00};				// Set TVN PLA decoder
			else if( tvnLatch == TVN_INTERRUPT)
				tvnMux = {6'b0, Ird[7:0], 2'b00};					// Interrupt vector was read and transferred to IRD
			else
				tvnMux = {10'b0, tvnLatch, 2'b00};
		end
		else
			tvnMux = { 8'h0, Irdecod_macroTvn, 2'b00};
	end
				
endmodule

// Nanorom (plus) decoder for die nanocode
module nDecoder3( input clk,

	output logic Nanod_permStart,
	output logic Nanod_waitBusFinish,
	output logic Nanod_isWrite,
	output logic Nanod_busByte,
	output logic Nanod_isRmc,
	output logic Nanod_noLowByte, Nanod_noHighByte,

	output logic Nanod_updTpend, Nanod_clrTpend,
	output logic Nanod_tvn2Ftu, Nanod_const2Ftu,
	output logic Nanod_ftu2Dbl, Nanod_ftu2Abl,
	output logic Nanod_abl2Pren, Nanod_updPren,
	output logic Nanod_inl2psw, Nanod_ftu2Sr, Nanod_sr2Ftu, Nanod_ftu2Ccr, Nanod_pswIToFtu,
	output logic Nanod_ird2Ftu, Nanod_ssw2Ftu,
	output logic Nanod_initST,
	output logic Nanod_Ir2Ird,

	output logic Nanod_auClkEn, Nanod_noSpAlign,
	output logic [2:0] Nanod_auCntrl,
	output logic Nanod_todbin, Nanod_toIrc,
	output logic Nanod_dbl2Atl, Nanod_abl2Atl, Nanod_atl2Abl, Nanod_atl2Dbl,
	output logic Nanod_abh2Ath, Nanod_dbh2Ath,
	output logic Nanod_ath2Dbh, Nanod_ath2Abh,

	output logic Nanod_db2Aob, Nanod_ab2Aob, Nanod_au2Aob,
	output logic Nanod_aob2Ab, Nanod_updSsw,
	output logic [1:0] Nanod_dobCtrl,

	output logic Nanod_abh2reg, Nanod_abl2reg,
	output logic Nanod_reg2abl, Nanod_reg2abh,
	output logic Nanod_dbh2reg, Nanod_dbl2reg,
	output logic Nanod_reg2dbl, Nanod_reg2dbh,
	output logic Nanod_ssp, Nanod_pchdbh, Nanod_pcldbl, Nanod_pclabl, Nanod_pchabh,

	output logic Nanod_rxh2dbh, Nanod_rxh2abh,
	output logic Nanod_dbl2rxl, Nanod_dbh2rxh,
	output logic Nanod_rxl2db, Nanod_rxl2ab,
	output logic Nanod_abl2rxl, Nanod_abh2rxh,
	output logic Nanod_dbh2ryh, Nanod_abh2ryh,
	output logic Nanod_ryl2db, Nanod_ryl2ab,
	output logic Nanod_ryh2dbh, Nanod_ryh2abh,
	output logic Nanod_dbl2ryl, Nanod_abl2ryl,
	output logic Nanod_rz,
	output logic Nanod_rxlDbl,

	output logic [2:0] Nanod_aluColumn,
	output logic [1:0] Nanod_aluDctrl,
	output logic Nanod_aluActrl,
	output logic Nanod_aluInit, Nanod_aluFinish,
	output logic Nanod_abd2Dcr, Nanod_dcr2Dbd,
	output logic Nanod_dbd2Alue, Nanod_alue2Dbd,
	output logic Nanod_dbd2Alub, Nanod_abd2Alub,

	output logic Nanod_alu2Dbd, Nanod_alu2Abd,
	output logic Nanod_au2Db, Nanod_au2Ab, Nanod_au2Pc,
	output logic Nanod_dbin2Abd, Nanod_dbin2Dbd,
	output logic Nanod_extDbh, Nanod_extAbh,
	output logic Nanod_ablAbd, Nanod_ablAbh,
	output logic Nanod_dblDbd, Nanod_dblDbh,
	output logic Nanod_abdIsByte,

	input logic Irdecod_isPcRel,
	input logic Irdecod_isTas,
	input logic Irdecod_implicitSp,
	input logic Irdecod_toCcr,
	input logic Irdecod_rxIsDt, Irdecod_ryIsDt,
	input logic Irdecod_rxIsUsp, Irdecod_rxIsMovem, Irdecod_movemPreDecr,
	input logic Irdecod_isByte,
	input logic Irdecod_isMovep,
	input logic [2:0] Irdecod_rx, Irdecod_ry,
	input logic Irdecod_rxIsAreg, Irdecod_ryIsAreg,
	input logic [15:0] Irdecod_ftuConst,
	input logic [5:0] Irdecod_macroTvn,
	input logic Irdecod_inhibitCcr,

	input enT2, enT4,
	input [UROM_WIDTH-1:0] microLatch,
	input [NANO_WIDTH-1:0] nanoLatch);

localparam NANO_IR2IRD = 67;
localparam NANO_TOIRC = 66;
localparam NANO_ALU_COL = 63;		// ALU operator column order is 63-64-65 !
localparam NANO_ALU_FI = 61;	// ALU finish-init 62-61
localparam NANO_TODBIN = 60;
localparam NANO_ALUE = 57;			// 57-59 shared with DCR control
localparam NANO_DCR = 57;			// 57-59 shared with ALUE control
localparam NANO_DOBCTRL_1 = 56;		// Input to control and permwrite
localparam NANO_LOWBYTE = 55;		// Used by MOVEP
localparam NANO_HIGHBYTE = 54;
localparam NANO_DOBCTRL_0 = 53;		// Input to control and permwrite
localparam NANO_ALU_DCTRL = 51;	// 52-51 databus input mux control
localparam NANO_ALU_ACTRL = 50;	// addrbus input mux control
localparam NANO_DBD2ALUB = 49;
localparam NANO_ABD2ALUB = 48;
localparam NANO_DBIN2DBD = 47;
localparam NANO_DBIN2ABD = 46;
localparam NANO_ALU2ABD = 45;
localparam NANO_ALU2DBD = 44;
localparam NANO_RZ = 43;
localparam NANO_BUSBYTE = 42;		// If *both* this set and instruction is byte sized, then bus cycle is byte sized.
localparam NANO_PCLABL = 41;
localparam NANO_RXL_DBL = 40;		// Switches RXL/RYL on DBL/ABL buses
localparam NANO_PCLDBL = 39;
localparam NANO_ABDHRECHARGE = 38;
localparam NANO_REG2ABL = 37;		// register to ABL
localparam NANO_ABL2REG = 36;		// ABL to register
localparam NANO_ABLABD = 35;
localparam NANO_DBLDBD = 34;
localparam NANO_DBL2REG = 33;		// DBL to register
localparam NANO_REG2DBL = 32;		// register to DBL
localparam NANO_ATLCTRL = 29;		// 31-29
localparam NANO_FTUCONTROL = 25;
localparam NANO_SSP = 24;
localparam NANO_RXH_DBH = 22;		// Switches RXH/RYH on DBH/ABH buses
localparam NANO_AUOUT = 20;			// 21-20
localparam NANO_AUCLKEN = 19;
localparam NANO_AUCTRL = 16;		// 18-16
localparam NANO_DBLDBH = 15;
localparam NANO_ABLABH = 14;
localparam NANO_EXT_ABH = 13;
localparam NANO_EXT_DBH = 12;
localparam NANO_ATHCTRL = 9;		// 11-9
localparam NANO_REG2ABH = 8;		// register to ABH
localparam NANO_ABH2REG = 7;		// ABH to register
localparam NANO_REG2DBH = 6;		// register to DBH
localparam NANO_DBH2REG = 5;		// DBH to register
localparam NANO_AOBCTRL = 3;		// 4-3
localparam NANO_PCH = 0;			// 1-0 PchDbh PchAbh
localparam NANO_NO_SP_ALGN = 0;		// Same bits as above when both set

localparam NANO_FTU_UPDTPEND = 1;		// Also loads FTU constant according to IRD !
localparam NANO_FTU_INIT_ST = 15;		// Set S, clear T (but not TPEND)
localparam NANO_FTU_CLRTPEND = 14;
localparam NANO_FTU_TVN = 13;
localparam NANO_FTU_ABL2PREN = 12;		// ABL => FTU & ABL => PREN. Both transfers enabled, but only one will be used depending on uroutine.
localparam NANO_FTU_SSW = 11;
localparam NANO_FTU_RSTPREN = 10;
localparam NANO_FTU_IRD = 9;
localparam NANO_FTU_2ABL = 8;
localparam NANO_FTU_RDSR = 7;
localparam NANO_FTU_INL = 6;
localparam NANO_FTU_PSWI = 5;		// Read Int Mask into FTU
localparam NANO_FTU_DBL = 4;
localparam NANO_FTU_2SR = 2;
localparam NANO_FTU_CONST = 1;

	reg [3:0] ftuCtrl;	
	
	logic [2:0] athCtrl, atlCtrl;
	assign athCtrl = nanoLatch[ NANO_ATHCTRL+2: NANO_ATHCTRL];
	assign atlCtrl = nanoLatch[ NANO_ATLCTRL+2: NANO_ATLCTRL];
	wire [1:0] aobCtrl = nanoLatch[ NANO_AOBCTRL+1:NANO_AOBCTRL];
	wire [1:0] dobCtrl = {nanoLatch[ NANO_DOBCTRL_1], nanoLatch[NANO_DOBCTRL_0]};
	
	always_ff @( posedge clk) begin
		if( enT4) begin
			// Reverse order!
			ftuCtrl <= { nanoLatch[ NANO_FTUCONTROL+0], nanoLatch[ NANO_FTUCONTROL+1], nanoLatch[ NANO_FTUCONTROL+2], nanoLatch[ NANO_FTUCONTROL+3]} ;
				
			Nanod_auClkEn <= !nanoLatch[ NANO_AUCLKEN];
			Nanod_auCntrl <= nanoLatch[ NANO_AUCTRL+2 : NANO_AUCTRL+0];
			Nanod_noSpAlign <= (nanoLatch[ NANO_NO_SP_ALGN + 1:NANO_NO_SP_ALGN] == 2'b11);			
			Nanod_extDbh <= nanoLatch[ NANO_EXT_DBH];
			Nanod_extAbh <= nanoLatch[ NANO_EXT_ABH];
			Nanod_todbin <= nanoLatch[ NANO_TODBIN];
			Nanod_toIrc <=  nanoLatch[ NANO_TOIRC];
			
			// ablAbd is disabled on byte transfers (adbhCharge plus irdIsByte). Not sure the combination makes much sense.
			// It happens in a few cases but I don't see anything enabled on abL (or abH) section anyway.
			
			Nanod_ablAbd <= nanoLatch[ NANO_ABLABD];
			Nanod_ablAbh <= nanoLatch[ NANO_ABLABH];
			Nanod_dblDbd <= nanoLatch[ NANO_DBLDBD];
			Nanod_dblDbh <= nanoLatch[ NANO_DBLDBH];
			
			Nanod_dbl2Atl <= (atlCtrl == 3'b010);
			Nanod_atl2Dbl <= (atlCtrl == 3'b011);
			Nanod_abl2Atl <= (atlCtrl == 3'b100);
			Nanod_atl2Abl <= (atlCtrl == 3'b101);

			Nanod_aob2Ab <= (athCtrl == 3'b101);		// Used on BSER1 only
			
			Nanod_abh2Ath <= (athCtrl == 3'b001) | (athCtrl == 3'b101);
			Nanod_dbh2Ath <= (athCtrl == 3'b100);
			Nanod_ath2Dbh <= (athCtrl == 3'b110);
			Nanod_ath2Abh <= (athCtrl == 3'b011);			

			Nanod_alu2Dbd <= nanoLatch[ NANO_ALU2DBD];
			Nanod_alu2Abd <= nanoLatch[ NANO_ALU2ABD];
			
			Nanod_abd2Dcr <= (nanoLatch[ NANO_DCR+1:NANO_DCR] == 2'b11);
			Nanod_dcr2Dbd <= (nanoLatch[ NANO_DCR+2:NANO_DCR+1] == 2'b11);
			Nanod_dbd2Alue <= (nanoLatch[ NANO_ALUE+2:NANO_ALUE+1] == 2'b10);
			Nanod_alue2Dbd <= (nanoLatch[ NANO_ALUE+1:NANO_ALUE] == 2'b01);

			Nanod_dbd2Alub <= nanoLatch[ NANO_DBD2ALUB];
			Nanod_abd2Alub <= nanoLatch[ NANO_ABD2ALUB];			
			
			// Originally not latched. We better should because we transfer one cycle later, T3 instead of T1.
			Nanod_dobCtrl <= dobCtrl;
			// Nanod_adb2Dob <= (dobCtrl == 2'b10);			Nanod_dbd2Dob <= (dobCtrl == 2'b01);			Nanod_alu2Dob <= (dobCtrl == 2'b11);
							
		end
	end
	
	// Update SSW at the start of Bus/Addr error ucode
	assign Nanod_updSsw = Nanod_aob2Ab;

	assign Nanod_updTpend = (ftuCtrl == NANO_FTU_UPDTPEND);
	assign Nanod_clrTpend = (ftuCtrl == NANO_FTU_CLRTPEND);
	assign Nanod_tvn2Ftu = (ftuCtrl == NANO_FTU_TVN);
	assign Nanod_const2Ftu = (ftuCtrl == NANO_FTU_CONST);
	assign Nanod_ftu2Dbl = (ftuCtrl == NANO_FTU_DBL) | ( ftuCtrl == NANO_FTU_INL);	
	assign Nanod_ftu2Abl = (ftuCtrl == NANO_FTU_2ABL);	
	assign Nanod_inl2psw = (ftuCtrl == NANO_FTU_INL);
	assign Nanod_pswIToFtu = (ftuCtrl == NANO_FTU_PSWI);
	assign Nanod_ftu2Sr = (ftuCtrl == NANO_FTU_2SR);
	assign Nanod_sr2Ftu = (ftuCtrl == NANO_FTU_RDSR);
	assign Nanod_ird2Ftu = (ftuCtrl == NANO_FTU_IRD);		// Used on bus/addr error
	assign Nanod_ssw2Ftu = (ftuCtrl == NANO_FTU_SSW);
	assign Nanod_initST = (ftuCtrl == NANO_FTU_INL) | (ftuCtrl == NANO_FTU_CLRTPEND) | (ftuCtrl == NANO_FTU_INIT_ST);
	assign Nanod_abl2Pren = (ftuCtrl == NANO_FTU_ABL2PREN);
	assign Nanod_updPren = (ftuCtrl == NANO_FTU_RSTPREN);
	
	assign Nanod_Ir2Ird = nanoLatch[ NANO_IR2IRD];

	// ALU control better latched later after combining with IRD decoding
		
	assign Nanod_aluDctrl = nanoLatch[ NANO_ALU_DCTRL+1 : NANO_ALU_DCTRL];
	assign Nanod_aluActrl = nanoLatch[ NANO_ALU_ACTRL];
	assign Nanod_aluColumn = { nanoLatch[ NANO_ALU_COL], nanoLatch[ NANO_ALU_COL+1], nanoLatch[ NANO_ALU_COL+2]};
	wire [1:0] aluFinInit = nanoLatch[ NANO_ALU_FI+1:NANO_ALU_FI];
	assign Nanod_aluFinish = (aluFinInit == 2'b10);
	assign Nanod_aluInit = (aluFinInit == 2'b01);

	// FTU 2 CCR encoded as both ALU Init and ALU Finish set.
	// In theory this encoding allows writes to CCR without writing to SR
	// But FTU 2 CCR and to SR are both set together at nanorom.
	assign Nanod_ftu2Ccr = ( aluFinInit == 2'b11);

	assign Nanod_abdIsByte = nanoLatch[ NANO_ABDHRECHARGE];
	
	// Not being latched on T4 creates non unique case warning!
	assign Nanod_au2Db = (nanoLatch[ NANO_AUOUT + 1: NANO_AUOUT] == 2'b01);
	assign Nanod_au2Ab = (nanoLatch[ NANO_AUOUT + 1: NANO_AUOUT] == 2'b10);
	assign Nanod_au2Pc = (nanoLatch[ NANO_AUOUT + 1: NANO_AUOUT] == 2'b11);
	
	assign Nanod_db2Aob = (aobCtrl == 2'b10);
	assign Nanod_ab2Aob = (aobCtrl == 2'b01);
	assign Nanod_au2Aob = (aobCtrl == 2'b11);
	
	assign Nanod_dbin2Abd = nanoLatch[ NANO_DBIN2ABD];
	assign Nanod_dbin2Dbd = nanoLatch[ NANO_DBIN2DBD];
	
	assign Nanod_permStart = (| aobCtrl);
	assign Nanod_isWrite  = ( | dobCtrl);
	assign Nanod_waitBusFinish = nanoLatch[ NANO_TOIRC] | nanoLatch[ NANO_TODBIN] | Nanod_isWrite;
	assign Nanod_busByte = nanoLatch[ NANO_BUSBYTE];
	
	assign Nanod_noLowByte = nanoLatch[ NANO_LOWBYTE];
	assign Nanod_noHighByte = nanoLatch[ NANO_HIGHBYTE];	
			
	// Not registered. Register at T4 after combining
	// Might be better to remove all those and combine here instead of at execution unit !!
	assign Nanod_abl2reg = nanoLatch[ NANO_ABL2REG];
	assign Nanod_abh2reg = nanoLatch[ NANO_ABH2REG];
	assign Nanod_dbl2reg = nanoLatch[ NANO_DBL2REG];
	assign Nanod_dbh2reg = nanoLatch[ NANO_DBH2REG];
	assign Nanod_reg2dbl = nanoLatch[ NANO_REG2DBL];
	assign Nanod_reg2dbh = nanoLatch[ NANO_REG2DBH];
	assign Nanod_reg2abl = nanoLatch[ NANO_REG2ABL];
	assign Nanod_reg2abh = nanoLatch[ NANO_REG2ABH];
	
	assign Nanod_ssp = nanoLatch[ NANO_SSP];
	
	assign Nanod_rz = nanoLatch[ NANO_RZ];
	
	// Actually DTL can't happen on PC relative mode. See IR decoder.
	
	wire dtldbd = 1'b0;
	wire dthdbh = 1'b0;
	wire dtlabd = 1'b0;
	wire dthabh = 1'b0;
	
	wire dblSpecial = Nanod_pcldbl | dtldbd;
	wire dbhSpecial = Nanod_pchdbh | dthdbh;
	wire ablSpecial = Nanod_pclabl | dtlabd;
	wire abhSpecial = Nanod_pchabh | dthabh;
			
	//
	// Combine with IRD decoding
	// Careful that IRD is updated only on T1! All output depending on IRD must be latched on T4!
	//
	
	// PC used instead of RY on PC relative instuctions
	
	assign Nanod_rxlDbl = nanoLatch[ NANO_RXL_DBL];
	wire isPcRel = Irdecod_isPcRel & !Nanod_rz;
	wire pcRelDbl = isPcRel & !nanoLatch[ NANO_RXL_DBL];
	wire pcRelDbh = isPcRel & !nanoLatch[ NANO_RXH_DBH];
	wire pcRelAbl = isPcRel & nanoLatch[ NANO_RXL_DBL];
	wire pcRelAbh = isPcRel & nanoLatch[ NANO_RXH_DBH];
	
	assign Nanod_pcldbl = nanoLatch[ NANO_PCLDBL] | pcRelDbl;
	assign Nanod_pchdbh = (nanoLatch[ NANO_PCH+1:NANO_PCH] == 2'b01) | pcRelDbh;
	
	assign Nanod_pclabl = nanoLatch[ NANO_PCLABL] | pcRelAbl;
	assign Nanod_pchabh = (nanoLatch[ NANO_PCH+1:NANO_PCH] == 2'b10) | pcRelAbh;

	// Might be better not to register these signals to allow latching RX/RY mux earlier!
	// But then must latch Irdecod_isPcRel on T3!

	always_ff @( posedge clk) begin
		if( enT4) begin
			Nanod_rxl2db <= Nanod_reg2dbl & !dblSpecial & nanoLatch[ NANO_RXL_DBL];
			Nanod_rxl2ab <= Nanod_reg2abl & !ablSpecial & !nanoLatch[ NANO_RXL_DBL];
			
			Nanod_dbl2rxl <= Nanod_dbl2reg & !dblSpecial & nanoLatch[ NANO_RXL_DBL];	
			Nanod_abl2rxl <= Nanod_abl2reg & !ablSpecial & !nanoLatch[ NANO_RXL_DBL];	

			Nanod_rxh2dbh <= Nanod_reg2dbh & !dbhSpecial & nanoLatch[ NANO_RXH_DBH];			
			Nanod_rxh2abh <= Nanod_reg2abh & !abhSpecial & !nanoLatch[ NANO_RXH_DBH];			
				
			Nanod_dbh2rxh <= Nanod_dbh2reg & !dbhSpecial & nanoLatch[ NANO_RXH_DBH];
			Nanod_abh2rxh <= Nanod_abh2reg & !abhSpecial & !nanoLatch[ NANO_RXH_DBH];	

			Nanod_dbh2ryh <= Nanod_dbh2reg & !dbhSpecial & !nanoLatch[ NANO_RXH_DBH];
			Nanod_abh2ryh <= Nanod_abh2reg & !abhSpecial & nanoLatch[ NANO_RXH_DBH];	

			Nanod_dbl2ryl <= Nanod_dbl2reg & !dblSpecial & !nanoLatch[ NANO_RXL_DBL];	
			Nanod_abl2ryl <= Nanod_abl2reg & !ablSpecial & nanoLatch[ NANO_RXL_DBL];	

			Nanod_ryl2db <= Nanod_reg2dbl & !dblSpecial & !nanoLatch[ NANO_RXL_DBL];
			Nanod_ryl2ab <= Nanod_reg2abl & !ablSpecial & nanoLatch[ NANO_RXL_DBL];

			Nanod_ryh2dbh <= Nanod_reg2dbh & !dbhSpecial & !nanoLatch[ NANO_RXH_DBH];			
			Nanod_ryh2abh <= Nanod_reg2abh & !abhSpecial & nanoLatch[ NANO_RXH_DBH];					
		end
		
		// Originally isTas only delayed on T2 (and seems only a late mask rev fix)
		// Better latch the combination on T4
		if( enT4)
			Nanod_isRmc <= Irdecod_isTas & nanoLatch[ NANO_BUSBYTE];
	end
			
	
endmodule

//
// IRD execution decoder. Complements nano code decoder
//
// IRD updated on T1, while ncode still executing. To avoid using the next IRD,
// decoded signals must be registered on T3, or T4 before using them.
//
module irdDecode( input [15:0] ird,

	output logic Irdecod_isPcRel,
	output logic Irdecod_isTas,
	output logic Irdecod_implicitSp,
	output logic Irdecod_toCcr,
	output logic Irdecod_rxIsDt, Irdecod_ryIsDt,
	output logic Irdecod_rxIsUsp, Irdecod_rxIsMovem, Irdecod_movemPreDecr,
	output logic Irdecod_isByte,
	output logic Irdecod_isMovep,
	output logic [2:0] Irdecod_rx, Irdecod_ry,
	output logic Irdecod_rxIsAreg, Irdecod_ryIsAreg,
	output logic [15:0] Irdecod_ftuConst,
	output logic [5:0] Irdecod_macroTvn,
	output logic Irdecod_inhibitCcr);

	wire [3:0] line = ird[15:12];
	logic [15:0] lineOnehot;

	// This can be registered and pipelined from the IR decoder !
	onehotEncoder4 irdLines( line, lineOnehot);
	
	wire isRegShift = (lineOnehot['he]) & (ird[7:6] != 2'b11);
	wire isDynShift = isRegShift & ird[5];
					
	assign Irdecod_isPcRel = (& ird[ 5:3]) & ~isDynShift & !ird[2] & ird[1];		
	assign Irdecod_isTas = lineOnehot[4] & (ird[11:6] == 6'b101011);
	
	assign Irdecod_rx = ird[11:9];
	assign Irdecod_ry = ird[ 2:0];

	wire isPreDecr = (ird[ 5:3] == 3'b100);
	wire eaAreg = (ird[5:3] == 3'b001);
		
	// rx is A or D
	// movem
	always_comb begin
		unique case( 1'b1)
		lineOnehot[1],
		lineOnehot[2],
		lineOnehot[3]:
					// MOVE: RX always Areg except if dest mode is Dn 000
					Irdecod_rxIsAreg = (| ird[8:6]);

		lineOnehot[4]:		Irdecod_rxIsAreg = (& ird[8:6]);		// not CHK (LEA)

		lineOnehot['h8]:	Irdecod_rxIsAreg = eaAreg & ird[8] & ~ird[7];	// SBCD
		lineOnehot['hc]:	Irdecod_rxIsAreg = eaAreg & ird[8] & ~ird[7];	// ABCD/EXG An,An				
		
		lineOnehot['h9],
		lineOnehot['hb],
		lineOnehot['hd]:	Irdecod_rxIsAreg =
							(ird[7] & ird[6]) |								// SUBA/CMPA/ADDA
							(eaAreg & ird[8] & (ird[7:6] != 2'b11));		// SUBX/CMPM/ADDX		
		default:
				Irdecod_rxIsAreg = Irdecod_implicitSp;
		endcase
	end	

	// RX is movem
	always_comb begin
		Irdecod_rxIsMovem = lineOnehot[4] & ~ird[8] & ~Irdecod_implicitSp;
	end
	assign Irdecod_movemPreDecr = Irdecod_rxIsMovem & isPreDecr;
	
	// RX is DT.
	// but SSP explicit or pc explicit has higher priority!
	// addq/subq (scc & dbcc also, but don't use rx)
	// Immediate including static bit
	assign Irdecod_rxIsDt = lineOnehot[5] | (lineOnehot[0] & ~ird[8]);
	
	// RX is USP
	assign Irdecod_rxIsUsp = lineOnehot[4] & (ird[ 11:4] == 8'he6);
	
	// RY is DT
	// rz or PC explicit has higher priority
	
	wire eaImmOrAbs = (ird[5:3] == 3'b111) & ~ird[1];
	assign Irdecod_ryIsDt = eaImmOrAbs & ~isRegShift;
		
	// RY is Address register
	always_comb begin
		logic eaIsAreg;
		
		// On most cases RY is Areg expect if mode is 000 (DATA REG) or 111 (IMM, ABS,PC REL)
		eaIsAreg = (ird[5:3] != 3'b000) & (ird[5:3] != 3'b111);
		
		unique case( 1'b1)
				// MOVE: RY always Areg expect if mode is 000 (DATA REG) or 111 (IMM, ABS,PC REL)
				// Most lines, including misc line 4, also.
		default:		Irdecod_ryIsAreg = eaIsAreg;

		lineOnehot[5]:	// DBcc is an exception
						Irdecod_ryIsAreg = eaIsAreg & (ird[7:3] != 5'b11001);

		lineOnehot[6],
		lineOnehot[7]:	Irdecod_ryIsAreg = 1'b0;

		lineOnehot['he]:
						Irdecod_ryIsAreg = ~isRegShift;
		endcase
	end	

	// Byte sized instruction
		
	// Original implementation sets this for some instructions that aren't really byte size
	// but doesn't matter because they don't have a byte transfer enabled at nanocode, such as MOVEQ
		
	wire xIsScc = (ird[7:6] == 2'b11) & (ird[5:3] != 3'b001); 
	wire xStaticMem = (ird[11:8] == 4'b1000) & (ird[5:4] == 2'b00);		// Static bit to mem
	always_comb begin
		unique case( 1'b1)
		lineOnehot[0]:
				Irdecod_isByte = 
				( ird[8] & (ird[5:4] != 2'b00)					) |	// Dynamic bit to mem
				( (ird[11:8] == 4'b1000) & (ird[5:4] != 2'b00)	) |	// Static bit to mem
				( (ird[8:7] == 2'b10) & (ird[5:3] == 3'b001)	) |	// Movep from mem only! For byte mux
				( (ird[8:6] == 3'b000) & !xStaticMem );				// Immediate byte
								
		lineOnehot[1]:			Irdecod_isByte = 1'b1;		// MOVE.B
	
		
		lineOnehot[4]:			Irdecod_isByte = (ird[7:6] == 2'b00) | Irdecod_isTas;		
		lineOnehot[5]:			Irdecod_isByte = (ird[7:6] == 2'b00) | xIsScc;
		
		lineOnehot[8],
		lineOnehot[9],
		lineOnehot['hb],
		lineOnehot['hc],
		lineOnehot['hd],
		lineOnehot['he]:		Irdecod_isByte = (ird[7:6] == 2'b00);
		
		default:				Irdecod_isByte = 1'b0;
		endcase
	end	
	
	// Need it for special byte size. Bus is byte, but whole register word is modified.
	assign Irdecod_isMovep = lineOnehot[0] & ird[8] & eaAreg;
	
	
	// rxIsSP implicit use of RX for actual SP transfer
	//
	// This logic is simple and will include some instructions that don't actually reference SP.
	// But doesn't matter as long as they don't perform any RX transfer.
	
	always_comb begin
		unique case( 1'b1)
		lineOnehot[6]:		Irdecod_implicitSp = (ird[11:8] == 4'b0001);		// BSR
		lineOnehot[4]:
			// Misc like RTS, JSR, etc
			Irdecod_implicitSp = (ird[11:8] == 4'b1110) | (ird[11:6] == 6'b1000_01);
		default:			Irdecod_implicitSp = 1'b0;
		endcase
	end
	
	// Modify CCR (and not SR)
	// Probably overkill !! Only needs to distinguish SR vs CCR
	// RTR, MOVE to CCR, xxxI to CCR
	assign Irdecod_toCcr =	( lineOnehot[4] & ((ird[11:0] == 12'he77) | (ird[11:6] == 6'b010011)) ) |
							( lineOnehot[0] & (ird[8:6] == 3'b000));
	
	// FTU constants
	// This should not be latched on T3/T4. Latch on T2 or not at all. FTU needs it on next T3.
	// Note: Reset instruction gets constant from ALU not from FTU!
	logic [15:0] ftuConst;
	wire [3:0] zero28 = (ird[11:9] == 0) ? 4'h8 : { 1'b0, ird[11:9]};		// xltate 0,1-7 into 8,1-7

	always_comb begin
		unique case( 1'b1)
		lineOnehot[6],														// Bcc short
		lineOnehot[7]:		ftuConst = { { 8{ ird[ 7]}}, ird[ 7:0] };		// MOVEQ
		
		lineOnehot['h5],													// addq/subq/static shift double check this
		lineOnehot['he]:	ftuConst = { 12'b0, zero28};
		
		// MULU/MULS DIVU/DIVS
		lineOnehot['h8],
		lineOnehot['hc]:	ftuConst = 16'h0f;

		lineOnehot[4]:		ftuConst = 16'h80;								// TAS

		default:			ftuConst = '0;
		endcase	
	end	
	assign Irdecod_ftuConst = ftuConst;
	
	//
	// TRAP Vector # for group 2 exceptions
	//
	
	always_comb begin
		if( lineOnehot[4]) begin
			case ( ird[6:5])
			2'b00,2'b01:	Irdecod_macroTvn = 6;					// CHK
			2'b11:			Irdecod_macroTvn = 7;					// TRAPV
			2'b10:			Irdecod_macroTvn = {2'b10, ird[3:0]};	// TRAP
			endcase
		end
		else
							Irdecod_macroTvn = 5;					// Division by zero
	end
	
	
	wire eaAdir = (ird[ 5:3] == 3'b001);
	wire size11 = ird[7] & ird[6];
	
	// Opcodes variants that don't affect flags
	// ADDA/SUBA ADDQ/SUBQ MOVEA

	assign Irdecod_inhibitCcr =
		( (lineOnehot[9] | lineOnehot['hd]) & size11) |				// ADDA/SUBA
		( lineOnehot[5] & eaAdir) |									// ADDQ/SUBQ to An (originally checks for line[4] as well !?)
		( (lineOnehot[2] | lineOnehot[3]) & ird[8:6] == 3'b001);	// MOVEA
		
endmodule

/*
 Execution unit

 Executes register transfers set by the microcode. Originally through a set of bidirectional buses.
 Most sources are available at T3, but DBIN only at T4! CCR also might be updated at T4, but it is not connected to these buses.
 We mux at T1 and T2, then transfer to the destination at T3. The exception is AOB that need to be updated earlier.

*/

module excUnit( input clk, enPhi1, enPhi2, extReset, pwrUp,
	input enT1, enT2, enT3, enT4,

	input logic Nanod_permStart,
	input logic Nanod_waitBusFinish,
	input logic Nanod_isWrite,
	input logic Nanod_busByte,
	input logic Nanod_isRmc,
	input logic Nanod_noLowByte, Nanod_noHighByte,

	input logic Nanod_updTpend, Nanod_clrTpend,
	input logic Nanod_tvn2Ftu, Nanod_const2Ftu,
	input logic Nanod_ftu2Dbl, Nanod_ftu2Abl,
	input logic Nanod_abl2Pren, Nanod_updPren,
	input logic Nanod_inl2psw, Nanod_ftu2Sr, Nanod_sr2Ftu, Nanod_ftu2Ccr, Nanod_pswIToFtu,
	input logic Nanod_ird2Ftu, Nanod_ssw2Ftu,
	input logic Nanod_initST,
	input logic Nanod_Ir2Ird,

	input logic Nanod_auClkEn, Nanod_noSpAlign,
	input logic [2:0] Nanod_auCntrl,
	input logic Nanod_todbin, Nanod_toIrc,
	input logic Nanod_dbl2Atl, Nanod_abl2Atl, Nanod_atl2Abl, Nanod_atl2Dbl,
	input logic Nanod_abh2Ath, Nanod_dbh2Ath,
	input logic Nanod_ath2Dbh, Nanod_ath2Abh,

	input logic Nanod_db2Aob, Nanod_ab2Aob, Nanod_au2Aob,
	input logic Nanod_aob2Ab, Nanod_updSsw,
	input logic [1:0] Nanod_dobCtrl,

	input logic Nanod_abh2reg, Nanod_abl2reg,
	input logic Nanod_reg2abl, Nanod_reg2abh,
	input logic Nanod_dbh2reg, Nanod_dbl2reg,
	input logic Nanod_reg2dbl, Nanod_reg2dbh,
	input logic Nanod_ssp, Nanod_pchdbh, Nanod_pcldbl, Nanod_pclabl, Nanod_pchabh,

	input logic Nanod_rxh2dbh, Nanod_rxh2abh,
	input logic Nanod_dbl2rxl, Nanod_dbh2rxh,
	input logic Nanod_rxl2db, Nanod_rxl2ab,
	input logic Nanod_abl2rxl, Nanod_abh2rxh,
	input logic Nanod_dbh2ryh, Nanod_abh2ryh,
	input logic Nanod_ryl2db, Nanod_ryl2ab,
	input logic Nanod_ryh2dbh, Nanod_ryh2abh,
	input logic Nanod_dbl2ryl, Nanod_abl2ryl,
	input logic Nanod_rz,
	input logic Nanod_rxlDbl,

	input logic [2:0] Nanod_aluColumn,
	input logic [1:0] Nanod_aluDctrl,
	input logic Nanod_aluActrl,
	input logic Nanod_aluInit, Nanod_aluFinish,
	input logic Nanod_abd2Dcr, Nanod_dcr2Dbd,
	input logic Nanod_dbd2Alue, Nanod_alue2Dbd,
	input logic Nanod_dbd2Alub, Nanod_abd2Alub,

	input logic Nanod_alu2Dbd, Nanod_alu2Abd,
	input logic Nanod_au2Db, Nanod_au2Ab, Nanod_au2Pc,
	input logic Nanod_dbin2Abd, Nanod_dbin2Dbd,
	input logic Nanod_extDbh, Nanod_extAbh,
	input logic Nanod_ablAbd, Nanod_ablAbh,
	input logic Nanod_dblDbd, Nanod_dblDbh,
	input logic Nanod_abdIsByte,
	
	input logic Irdecod_isPcRel,
	input logic Irdecod_isTas,
	input logic Irdecod_implicitSp,
	input logic Irdecod_toCcr,
	input logic Irdecod_rxIsDt, Irdecod_ryIsDt,
	input logic Irdecod_rxIsUsp, Irdecod_rxIsMovem, Irdecod_movemPreDecr,
	input logic Irdecod_isByte,
	input logic Irdecod_isMovep,
	input logic [2:0] Irdecod_rx, Irdecod_ry,
	input logic Irdecod_rxIsAreg, Irdecod_ryIsAreg,
	input logic [15:0] Irdecod_ftuConst,
	input logic [5:0] Irdecod_macroTvn,
	input logic Irdecod_inhibitCcr,

	input [15:0] Ird,			// ALU row (and others) decoder needs it	
	input pswS,
	input [15:0] ftu,
	input [15:0] iEdb,
	
	output logic [7:0] ccr,
	output [15:0] alue,
	
	output prenEmpty, au05z,
	output logic dcr4, ze,
	output logic aob0,
	output [15:0] AblOut,
	output logic [15:0] Irc,
	output logic [15:0] oEdb,
	output logic [23:1] eab);

localparam REG_USP = 15;
localparam REG_SSP = 16;
localparam REG_DT = 17;

	// Register file
	reg [15:0] regs68L[ 18];
	reg [15:0] regs68H[ 18];

// synthesis translate off
	/*
		It is bad practice to initialize simulation registers that the hardware doesn't.
		There is risk that simulation would be different than the real hardware. But in this case is the other way around.
		Some ROM uses something like sub.l An,An at powerup which clears the register
		Simulator power ups the registers with 'X, as they are really undetermined at the real hardware.
		But the simulator doesn't realize (it can't) that the same value is substracting from itself,
		and that the result should be zero even when it's 'X - 'X.
	*/
	
	initial begin
		for( int i = 0; i < 18; i++) begin
			regs68L[i] = '0;
			regs68H[i] = '0;
		end
	end
	
	// For simulation display only
	wire [31:0] SSP = { regs68H[REG_SSP], regs68L[REG_SSP]};
	
// synthesis translate on
	

	wire [15:0] aluOut;
	wire [15:0] dbin;
	logic [15:0] dcrOutput;

	reg [15:0] PcL, PcH;

	reg [31:0] auReg, aob;

	reg [15:0] Ath, Atl;

	// Bus execution
	reg [15:0] Dbl, Dbh;
	reg [15:0] Abh, Abl;
	reg [15:0] Abd, Dbd;
	
	assign AblOut = Abl;
	assign au05z = (~| auReg[5:0]);
	
	logic [15:0] dblMux, dbhMux;
	logic [15:0] abhMux, ablMux;
	logic [15:0] abdMux, dbdMux;
	
	logic abdIsByte;
		
	logic Pcl2Dbl, Pch2Dbh;
	logic Pcl2Abl, Pch2Abh;
	
	
	// RX RY muxes	
	// RX and RY actual registers
	logic [4:0] actualRx, actualRy;
	logic [3:0] movemRx;
	logic byteNotSpAlign;			// Byte instruction and no sp word align
	
	// IRD decoded signals must be latched. See comments on decoder
	// But nanostore decoding can't be latched before T4.
	//
	// If we need this earlier we can register IRD decode on T3 and use nano async

	logic [4:0] rxMux, ryMux;
	logic [3:0] rxReg, ryReg;
	logic rxIsSp, ryIsSp;
	logic rxIsAreg, ryIsAreg;
	
	always_comb begin
	
		// Unique IF !!
		if( Nanod_ssp) begin
			rxMux = REG_SSP;
			rxIsSp = 1'b1;
			rxReg = 1'bX;
		end
		else if( Irdecod_rxIsUsp) begin
			rxMux = REG_USP;
			rxIsSp = 1'b1;
			rxReg = 1'bX;
		end
		else if( Irdecod_rxIsDt & !Irdecod_implicitSp) begin
			rxMux = REG_DT;
			rxIsSp = 1'b0;
			rxReg = 1'bX;
		end
		else begin
			if( Irdecod_implicitSp)
				rxReg = 15;
			else if( Irdecod_rxIsMovem)
				rxReg = movemRx;
			else
				rxReg = { Irdecod_rxIsAreg, Irdecod_rx};
				
			if( (& rxReg)) begin
				rxMux = pswS ? REG_SSP : 15;
				rxIsSp = 1'b1;
			end
			else begin
				rxMux = { 1'b0, rxReg};
				rxIsSp = 1'b0;
			end
		end

		// RZ has higher priority!		
		if( Irdecod_ryIsDt & !Nanod_rz) begin
			ryMux = REG_DT;
			ryIsSp = 1'b0;
			ryReg = 'X;
		end
		else begin
			ryReg = Nanod_rz ? Irc[15:12] : {Irdecod_ryIsAreg, Irdecod_ry};
			ryIsSp = (& ryReg);
			if( ryIsSp & pswS)			// No implicit SP on RY
				ryMux = REG_SSP;
			else
				ryMux = { 1'b0, ryReg};
		end
					
	end	
	
	always_ff @( posedge clk) begin
		if( enT4) begin
			byteNotSpAlign <= Irdecod_isByte & ~(Nanod_rxlDbl ? rxIsSp : ryIsSp);
				
			actualRx <= rxMux;
			actualRy <= ryMux;
			
			rxIsAreg <= rxIsSp | rxMux[3];
			ryIsAreg <= ryIsSp | ryMux[3];			
		end
		
		if( enT4)
			abdIsByte <= Nanod_abdIsByte & Irdecod_isByte;
	end
			
	// Set RX/RY low word to which bus segment is connected.
		
	wire ryl2Abl = Nanod_ryl2ab & (ryIsAreg | Nanod_ablAbd);
	wire ryl2Abd = Nanod_ryl2ab & (~ryIsAreg | Nanod_ablAbd);
	wire ryl2Dbl = Nanod_ryl2db & (ryIsAreg | Nanod_dblDbd);
	wire ryl2Dbd = Nanod_ryl2db & (~ryIsAreg | Nanod_dblDbd);

	wire rxl2Abl = Nanod_rxl2ab & (rxIsAreg | Nanod_ablAbd);
	wire rxl2Abd = Nanod_rxl2ab & (~rxIsAreg | Nanod_ablAbd);
	wire rxl2Dbl = Nanod_rxl2db & (rxIsAreg | Nanod_dblDbd);
	wire rxl2Dbd = Nanod_rxl2db & (~rxIsAreg | Nanod_dblDbd);
	
	// Buses. Main mux
	
	logic abhIdle, ablIdle, abdIdle;
	logic dbhIdle, dblIdle, dbdIdle;
	
	always_comb begin
		{abhIdle, ablIdle, abdIdle} = '0;
		{dbhIdle, dblIdle, dbdIdle} = '0;

		unique case( 1'b1)
		ryl2Dbd:				dbdMux = regs68L[ actualRy];
		rxl2Dbd:				dbdMux = regs68L[ actualRx];
		Nanod_alue2Dbd:			dbdMux = alue;
		Nanod_dbin2Dbd:			dbdMux = dbin;
		Nanod_alu2Dbd:			dbdMux = aluOut;
		Nanod_dcr2Dbd:			dbdMux = dcrOutput;
		default: begin			dbdMux = 'X;	dbdIdle = 1'b1;				end
		endcase
	
		unique case( 1'b1)
		rxl2Dbl:				dblMux = regs68L[ actualRx];
		ryl2Dbl:				dblMux = regs68L[ actualRy];
		Nanod_ftu2Dbl:			dblMux = ftu;
		Nanod_au2Db:			dblMux = auReg[15:0];
		Nanod_atl2Dbl:			dblMux = Atl;
		Pcl2Dbl:				dblMux = PcL;
		default: begin			dblMux = 'X;	dblIdle = 1'b1;				end
		endcase
			
		unique case( 1'b1)
		Nanod_rxh2dbh:			dbhMux = regs68H[ actualRx];
		Nanod_ryh2dbh:			dbhMux = regs68H[ actualRy];
		Nanod_au2Db:			dbhMux = auReg[31:16];
		Nanod_ath2Dbh:			dbhMux = Ath;
		Pch2Dbh:				dbhMux = PcH;
		default: begin			dbhMux = 'X;	dbhIdle = 1'b1;				end
		endcase

		unique case( 1'b1)
		ryl2Abd:				abdMux = regs68L[ actualRy];
		rxl2Abd:				abdMux = regs68L[ actualRx];
		Nanod_dbin2Abd:			abdMux = dbin;
		Nanod_alu2Abd:			abdMux = aluOut;
		default: begin			abdMux = 'X;	abdIdle = 1'b1;				end
		endcase

		unique case( 1'b1)
		Pcl2Abl:				ablMux = PcL;
		rxl2Abl:				ablMux = regs68L[ actualRx];
		ryl2Abl:				ablMux = regs68L[ actualRy];
		Nanod_ftu2Abl:			ablMux = ftu;
		Nanod_au2Ab:			ablMux = auReg[15:0];
		Nanod_aob2Ab:			ablMux = aob[15:0];
		Nanod_atl2Abl:			ablMux = Atl;
		default: begin			ablMux = 'X;	ablIdle = 1'b1;				end
		endcase
			
		unique case( 1'b1)		
		Pch2Abh:				abhMux = PcH;
		Nanod_rxh2abh:			abhMux = regs68H[ actualRx];
		Nanod_ryh2abh:			abhMux = regs68H[ actualRy];
		Nanod_au2Ab:			abhMux = auReg[31:16];
		Nanod_aob2Ab:			abhMux = aob[31:16];
		Nanod_ath2Abh:			abhMux = Ath;
		default: begin			abhMux = 'X;	abhIdle = 1'b1;				end
		endcase
			
	end
	
	// Source starts driving the bus on T1. Bus holds data until end of T3. Destination latches at T3.

	// These registers store the first level mux, without bus interconnections.
	// Even when this uses almost to 100 registers, it saves a lot of comb muxing and it is much faster.
	reg [15:0] preAbh, preAbl, preAbd;
	reg [15:0] preDbh, preDbl, preDbd;
	
	always_ff @( posedge clk) begin

		// Register first level mux at T1		
		if( enT1) begin
			{preAbh, preAbl, preAbd} <= { abhMux, ablMux, abdMux};
			{preDbh, preDbl, preDbd} <= { dbhMux, dblMux, dbdMux};			
		end
		
		// Process bus interconnection at T2. Many combinations only used on DIV
		// We use a simple method. If a specific bus segment is not driven we know that it should get data from a neighbour segment.
		// In some cases this is not true and the segment is really idle without any destination. But then it doesn't matter.
		
		if( enT2) begin
			if( Nanod_extAbh)
				Abh <= { 16{ ablIdle ? preAbd[ 15] : preAbl[ 15] }};
			else if( abhIdle)
				Abh <= ablIdle ? preAbd : preAbl;
			else
				Abh <= preAbh;

			if( ~ablIdle)
				Abl <= preAbl;
			else
				Abl <= Nanod_ablAbh ? preAbh : preAbd;

			Abd <= ~abdIdle ? preAbd : ablIdle ? preAbh : preAbl;

			if( Nanod_extDbh)
				Dbh <= { 16{ dblIdle ? preDbd[ 15] : preDbl[ 15] }};
			else if( dbhIdle)
				Dbh <= dblIdle ? preDbd : preDbl;
			else
				Dbh <= preDbh;

			if( ~dblIdle)
				Dbl <= preDbl;
			else
				Dbl <= Nanod_dblDbh ? preDbh : preDbd;

			Dbd <= ~dbdIdle ? preDbd: dblIdle ? preDbh : preDbl;
			
			/*
			Dbl <= dblMux;			Dbh <= dbhMux;
			Abd <= abdMux;			Dbd <= dbdMux;
			Abh <= abhMux;			Abl <= ablMux; */
		end
	end

	// AOB
	//
	// Originally change on T1. We do on T2, only then the output is enabled anyway.
	//
	// AOB[0] is used for address error. But even when raises on T1, seems not actually used until T2 or possibly T3.
	// It is used on T1 when deasserted at the BSER exception ucode. Probably deassertion timing is not critical. 
	// But in that case (at BSER), AOB is loaded from AU, so we can safely transfer on T1.

	// We need to take directly from first level muxes that are updated and T1
			
	wire au2Aob = Nanod_au2Aob | (Nanod_au2Db & Nanod_db2Aob);
	
	always_ff @( posedge clk) begin
		// UNIQUE IF !
		
		if( enT1 & au2Aob)		// From AU we do can on T1
			aob <= auReg;
		else if( enT2) begin
			if( Nanod_db2Aob)
				aob <= { preDbh, ~dblIdle ? preDbl : preDbd};
			else if( Nanod_ab2Aob)
				aob <= { preAbh, ~ablIdle ? preAbl : preAbd};
		end
	end

	assign eab = aob[23:1];
	assign aob0 = aob[0];
				
	// AU
	logic [31:0] auInpMux;

	// `ifdef ALW_COMB_BUG
	// Old Modelsim bug. Doesn't update ouput always. Need excplicit sensitivity list !?
	// always @( Nanod_auCntrl) begin
	
	always_comb begin
		unique case( Nanod_auCntrl)
		3'b000:		auInpMux = 0;
		3'b001:		auInpMux = byteNotSpAlign | Nanod_noSpAlign ? 1 : 2;		// +1/+2
		3'b010:		auInpMux = -4;
		3'b011:		auInpMux = { Abh, Abl};
		3'b100:		auInpMux = 2;
		3'b101:		auInpMux = 4;
		3'b110:		auInpMux = -2;
		3'b111:		auInpMux = byteNotSpAlign | Nanod_noSpAlign ? -1 : -2;	// -1/-2
		default:	auInpMux = 'X;
		endcase
	end

	// Simulation problem
	// Sometimes (like in MULM1) DBH is not set. AU is used in these cases just as a 6 bits counter testing if bits 5-0 are zero.
	// But when adding something like 32'hXXXX0000, the simulator (incorrectly) will set *all the 32 bits* of the result as X.

// synthesis translate_off 
	`define SIMULBUGX32 1
	wire [16:0] aulow = Dbl + auInpMux[15:0];
	wire [31:0] auResult = {Dbh + auInpMux[31:16] + aulow[16], aulow[15:0]};
// synthesis translate_on

	always_ff @( posedge clk) begin
		if( pwrUp)
			auReg <= '0;
		else if( enT3 & Nanod_auClkEn)
			`ifdef SIMULBUGX32
				auReg <= auResult;
			`else
				auReg <= { Dbh, Dbl } + auInpMux;
			`endif
	end
	
	
	// Main A/D registers
	
	always_ff @( posedge clk) begin
		if( enT3) begin
			if( Nanod_dbl2rxl | Nanod_abl2rxl) begin
				if( ~rxIsAreg) begin
					if( Nanod_dbl2rxl)			regs68L[ actualRx] <= Dbd;
					else if( abdIsByte)			regs68L[ actualRx][7:0] <= Abd[7:0];
					else						regs68L[ actualRx] <= Abd;
				end
				else
					regs68L[ actualRx] <= Nanod_dbl2rxl ? Dbl : Abl;
			end
				
			if( Nanod_dbl2ryl | Nanod_abl2ryl) begin
				if( ~ryIsAreg) begin
					if( Nanod_dbl2ryl)			regs68L[ actualRy] <= Dbd;
					else if( abdIsByte)			regs68L[ actualRy][7:0] <= Abd[7:0];
					else						regs68L[ actualRy] <= Abd;				
				end
				else
					regs68L[ actualRy] <= Nanod_dbl2ryl ? Dbl : Abl;
			end
			
			// High registers are easier. Both A & D on the same buses, and not byte ops.
			if( Nanod_dbh2rxh | Nanod_abh2rxh)
				regs68H[ actualRx] <= Nanod_dbh2rxh ? Dbh : Abh;
			if( Nanod_dbh2ryh | Nanod_abh2ryh)
				regs68H[ actualRy] <= Nanod_dbh2ryh ? Dbh : Abh;
				
		end	
	end
		
	// PC & AT
	reg dbl2Pcl, dbh2Pch, abh2Pch, abl2Pcl;
		
	always_ff @( posedge clk) begin
		if( extReset) begin
			{ dbl2Pcl, dbh2Pch, abh2Pch, abl2Pcl } <= '0;
			
			Pcl2Dbl <= 1'b0;
			Pch2Dbh <= 1'b0;
			Pcl2Abl <= 1'b0;
			Pch2Abh <= 1'b0;
		end
		else if( enT4) begin				// Must latch on T4 !
			dbl2Pcl <= Nanod_dbl2reg & Nanod_pcldbl;
			dbh2Pch <= Nanod_dbh2reg & Nanod_pchdbh;
			abh2Pch <= Nanod_abh2reg & Nanod_pchabh;
			abl2Pcl <= Nanod_abl2reg & Nanod_pclabl;
			
			Pcl2Dbl <= Nanod_reg2dbl & Nanod_pcldbl;
			Pch2Dbh <= Nanod_reg2dbh & Nanod_pchdbh;
			Pcl2Abl <= Nanod_reg2abl & Nanod_pclabl;
			Pch2Abh <= Nanod_reg2abh & Nanod_pchabh;
		end
		
		// Unique IF !!!
		if( enT1 & Nanod_au2Pc)
			PcL <= auReg[15:0];
		else if( enT3) begin
			if( dbl2Pcl)
				PcL <= Dbl;
			else if( abl2Pcl)
				PcL <= Abl;
		end
			
		// Unique IF !!!
		if( enT1 & Nanod_au2Pc)
			PcH <= auReg[31:16];
		else if( enT3) begin
			if( dbh2Pch)
				PcH <= Dbh;
			else if( abh2Pch)
				PcH <= Abh;
		end

		// Unique IF !!!
		if( enT3) begin
			if( Nanod_dbl2Atl)
				Atl <= Dbl;
			else if( Nanod_abl2Atl)
				Atl <= Abl;
		end

		// Unique IF !!!
		if( enT3) begin
			if( Nanod_abh2Ath)
				Ath <= Abh;
			else if( Nanod_dbh2Ath)
				Ath <= Dbh;
		end

	end

	// Movem reg mask priority encoder
	
	wire rmIdle;
	logic [3:0] prHbit;
	logic [15:0] prenLatch;
	
	// Invert reg order for predecrement mode
	assign prenEmpty = (~| prenLatch);	
	pren rmPren( .mask( prenLatch), .hbit (prHbit));

	always_ff @( posedge clk) begin
		// Cheating: PREN always loaded from DBIN
		// Must be on T1 to branch earlier if reg mask is empty!
		if( enT1 & Nanod_abl2Pren)
			prenLatch <= dbin;
		else if( enT3 & Nanod_updPren) begin
			prenLatch [prHbit] <= 1'b0;
			movemRx <= Irdecod_movemPreDecr ? ~prHbit : prHbit;
		end
	end
	
	// DCR
	wire [15:0] dcrCode;

	wire [3:0] dcrInput = abdIsByte ? { 1'b0, Abd[ 2:0]} : Abd[ 3:0];
	onehotEncoder4 dcrDecoder( .bin( dcrInput), .bitMap( dcrCode));
	
	always_ff @( posedge clk) begin
		if( pwrUp)
			dcr4 <= '0;
		else if( enT3 & Nanod_abd2Dcr) begin
			dcrOutput <= dcrCode;
			dcr4 <= Abd[4];
		end
	end
	
	// ALUB
	reg [15:0] alub;
	
	always_ff @( posedge clk) begin
		if( enT3) begin
			// UNIQUE IF !!
			if( Nanod_dbd2Alub)
				alub <= Dbd;
			else if( Nanod_abd2Alub)
				alub <= Abd;				// abdIsByte affects this !!??
		end
	end
	
	wire alueClkEn = enT3 & Nanod_dbd2Alue;

	// DOB/DBIN/IRC
	
	logic [15:0] dobInput;
	wire dobIdle = (~| Nanod_dobCtrl);
	
	always_comb begin
		unique case (Nanod_dobCtrl)
		NANO_DOB_ADB:		dobInput = Abd;
		NANO_DOB_DBD:		dobInput = Dbd;
		NANO_DOB_ALU:		dobInput = aluOut;
		default:			dobInput = 'X;
		endcase
	end

	dataIo dataIo( .*, .enT1, .enT2, .enT3, .enT4,
			.iEdb, .dobIdle, .dobInput, .aob0,
			.Irc, .dbin, .oEdb);

	fx68kAlu alu(
		.clk( clk), .pwrUp( pwrUp), .enT1, .enT3, .enT4,
		.ird( Ird),
		.aluColumn( Nanod_aluColumn), .aluAddrCtrl( Nanod_aluActrl),
		.init( Nanod_aluInit), .finish( Nanod_aluFinish), .aluIsByte( Irdecod_isByte),
		.ftu2Ccr( Nanod_ftu2Ccr),
		.alub, .ftu, .alueClkEn, .alue,
		.aluDataCtrl( Nanod_aluDctrl), .iDataBus( Dbd), .iAddrBus(Abd),
		.ze, .aluOut, .ccr);

endmodule

//
// Data bus I/O
// At a separate module because it is a bit complicated and the timing is special.
// Here we do the low/high byte mux and the special case of MOVEP.
//
// Original implementation is rather complex because both the internal and external buses are bidirectional.
// Input is latched async at the EDB register.
// We capture directly from the external data bus to the internal registers (IRC & DBIN) on PHI2, starting the external S7 phase, at a T4 internal period.

module dataIo( input clk, enPhi1, enPhi2, extReset, pwrUp,
	input enT1, enT2, enT3, enT4,

	input logic Nanod_permStart,
	input logic Nanod_waitBusFinish,
	input logic Nanod_isWrite,
	input logic Nanod_busByte,
	input logic Nanod_isRmc,
	input logic Nanod_noLowByte, Nanod_noHighByte,

	input logic Nanod_updTpend, Nanod_clrTpend,
	input logic Nanod_tvn2Ftu, Nanod_const2Ftu,
	input logic Nanod_ftu2Dbl, Nanod_ftu2Abl,
	input logic Nanod_abl2Pren, Nanod_updPren,
	input logic Nanod_inl2psw, Nanod_ftu2Sr, Nanod_sr2Ftu, Nanod_ftu2Ccr, Nanod_pswIToFtu,
	input logic Nanod_ird2Ftu, Nanod_ssw2Ftu,
	input logic Nanod_initST,
	input logic Nanod_Ir2Ird,

	input logic Nanod_auClkEn, Nanod_noSpAlign,
	input logic [2:0] Nanod_auCntrl,
	input logic Nanod_todbin, Nanod_toIrc,
	input logic Nanod_dbl2Atl, Nanod_abl2Atl, Nanod_atl2Abl, Nanod_atl2Dbl,
	input logic Nanod_abh2Ath, Nanod_dbh2Ath,
	input logic Nanod_ath2Dbh, Nanod_ath2Abh,

	input logic Nanod_db2Aob, Nanod_ab2Aob, Nanod_au2Aob,
	input logic Nanod_aob2Ab, Nanod_updSsw,
	input logic [1:0] Nanod_dobCtrl,

	input logic Nanod_abh2reg, Nanod_abl2reg,
	input logic Nanod_reg2abl, Nanod_reg2abh,
	input logic Nanod_dbh2reg, Nanod_dbl2reg,
	input logic Nanod_reg2dbl, Nanod_reg2dbh,
	input logic Nanod_ssp, Nanod_pchdbh, Nanod_pcldbl, Nanod_pclabl, Nanod_pchabh,

	input logic Nanod_rxh2dbh, Nanod_rxh2abh,
	input logic Nanod_dbl2rxl, Nanod_dbh2rxh,
	input logic Nanod_rxl2db, Nanod_rxl2ab,
	input logic Nanod_abl2rxl, Nanod_abh2rxh,
	input logic Nanod_dbh2ryh, Nanod_abh2ryh,
	input logic Nanod_ryl2db, Nanod_ryl2ab,
	input logic Nanod_ryh2dbh, Nanod_ryh2abh,
	input logic Nanod_dbl2ryl, Nanod_abl2ryl,
	input logic Nanod_rz,
	input logic Nanod_rxlDbl,

	input logic [2:0] Nanod_aluColumn,
	input logic [1:0] Nanod_aluDctrl,
	input logic Nanod_aluActrl,
	input logic Nanod_aluInit, Nanod_aluFinish,
	input logic Nanod_abd2Dcr, Nanod_dcr2Dbd,
	input logic Nanod_dbd2Alue, Nanod_alue2Dbd,
	input logic Nanod_dbd2Alub, Nanod_abd2Alub,

	input logic Nanod_alu2Dbd, Nanod_alu2Abd,
	input logic Nanod_au2Db, Nanod_au2Ab, Nanod_au2Pc,
	input logic Nanod_dbin2Abd, Nanod_dbin2Dbd,
	input logic Nanod_extDbh, Nanod_extAbh,
	input logic Nanod_ablAbd, Nanod_ablAbh,
	input logic Nanod_dblDbd, Nanod_dblDbh,
	input logic Nanod_abdIsByte,
	input [15:0] iEdb,
	input aob0,

	input logic Irdecod_isPcRel,
	input logic Irdecod_isTas,
	input logic Irdecod_implicitSp,
	input logic Irdecod_toCcr,
	input logic Irdecod_rxIsDt, Irdecod_ryIsDt,
	input logic Irdecod_rxIsUsp, Irdecod_rxIsMovem, Irdecod_movemPreDecr,
	input logic Irdecod_isByte,
	input logic Irdecod_isMovep,
	input logic [2:0] Irdecod_rx, Irdecod_ry,
	input logic Irdecod_rxIsAreg, Irdecod_ryIsAreg,
	input logic [15:0] Irdecod_ftuConst,
	input logic [5:0] Irdecod_macroTvn,
	input logic Irdecod_inhibitCcr,

	input dobIdle,
	input [15:0] dobInput,
	
	output logic [15:0] Irc,	
	output logic [15:0] dbin,
	output logic [15:0] oEdb
	);

	reg [15:0] dob;
	
	// DBIN/IRC
	
	// Timing is different than any other register. We can latch only on the next T4 (bus phase S7).
	// We need to register all control signals correctly because the next ublock will already be started.
	// Can't latch control on T4 because if there are wait states there might be multiple T4 before we latch.
	
	reg xToDbin, xToIrc;
	reg dbinNoLow, dbinNoHigh;
	reg byteMux, isByte_T4;
	
	always_ff @( posedge clk) begin
	
		// Byte mux control. Can't latch at T1. AOB might be not ready yet.
		// Must latch IRD decode at T1 (or T4). Then combine and latch only at T3.

		// Can't latch at T3, a new IRD might be loaded already at T1.	
		// Ok to latch at T4 if combination latched then at T3
		if( enT4)
			isByte_T4 <= Irdecod_isByte;	// Includes MOVEP from mem, we could OR it here

		if( enT3) begin
			dbinNoHigh <= Nanod_noHighByte;
			dbinNoLow <= Nanod_noLowByte;
			byteMux <= Nanod_busByte & isByte_T4 & ~aob0;
		end
		
		if( enT1) begin
			// If on wait states, we continue latching until next T1
			xToDbin <= 1'b0;
			xToIrc <= 1'b0;			
		end
		else if( enT3) begin
			xToDbin <= Nanod_todbin;
			xToIrc <= Nanod_toIrc;
		end

		// Capture on T4 of the next ucycle
		// If there are wait states, we keep capturing every PHI2 until the next T1
		
		if( xToIrc & enPhi2)
			Irc <= iEdb;
		if( xToDbin & enPhi2) begin
			// Original connects both halves of EDB.
			if( ~dbinNoLow)
				dbin[ 7:0] <= byteMux ? iEdb[ 15:8] : iEdb[7:0];
			if( ~dbinNoHigh)
				dbin[ 15:8] <= ~byteMux & dbinNoLow ? iEdb[ 7:0] : iEdb[ 15:8];
		end
	end
	
	// DOB
	logic byteCycle;	
	
	always_ff @( posedge clk) begin
		// Originaly on T1. Transfer to internal EDB also on T1 (stays enabled upto the next T1). But only on T4 (S3) output enables.
		// It is safe to do on T3, then, but control signals if derived from IRD must be registered.
		// Originally control signals are not registered.
		
		// Wait states don't affect DOB operation that is done at the start of the bus cycle. 

		if( enT4)
			byteCycle <= Nanod_busByte & Irdecod_isByte;		// busIsByte but not MOVEP
		
		// Originally byte low/high interconnect is done at EDB, not at DOB.
		if( enT3 & ~dobIdle) begin
			dob[7:0] <= Nanod_noLowByte ? dobInput[15:8] : dobInput[ 7:0];
			dob[15:8] <= (byteCycle | Nanod_noHighByte) ? dobInput[ 7:0] : dobInput[15:8];
		end
	end
	assign oEdb = dob;

endmodule


// Provides ucode routine entries (A1/A3) for each opcode
// Also checks for illegal opcode and priv violation

// This is one of the slowest part of the processor.
// But no need to optimize or pipeline because the result is not needed until at least 4 cycles.
// IR updated at the least one microinstruction earlier.
// Just need to configure the timing analizer correctly.

module uaddrDecode( 
	input [15:0] opcode,
	output [UADDR_WIDTH-1:0] a1, a2, a3,
	output logic isPriv, isIllegal, isLineA, isLineF,
	output [15:0] lineBmap);
	
	wire [3:0] line = opcode[15:12];
	logic [3:0] eaCol, movEa;

	onehotEncoder4 irLineDecod( line, lineBmap);
	
	assign isLineA = lineBmap[ 'hA];
	assign isLineF = lineBmap[ 'hF];
	
	pla_lined pla_lined( .movEa( movEa), .col( eaCol),
		.opcode( opcode), .lineBmap( lineBmap),
		.palIll( isIllegal), .plaA1( a1), .plaA2( a2), .plaA3( a3) );
	
	// ea decoding   
	assign eaCol = eaDecode( opcode[ 5:0]);
	assign movEa = eaDecode( {opcode[ 8:6], opcode[ 11:9]} );

	// EA decode
	function [3:0] eaDecode;
	input [5:0] eaBits;
	begin
	unique case( eaBits[ 5:3])
	3'b111:
		case( eaBits[ 2:0])
		3'b000:   eaDecode = 7;            // Absolute short
		3'b001:   eaDecode = 8;            // Absolute long
		3'b010:   eaDecode = 9;            // PC displacement
		3'b011:   eaDecode = 10;           // PC offset
		3'b100:   eaDecode = 11;           // Immediate
		default:  eaDecode = 12;           // Invalid
		endcase
         
	default:   eaDecode = eaBits[5:3];      // Register based EAs
	endcase
	end
	endfunction
	
	
	/*
	Privileged instructions:

	ANDI/EORI/ORI SR
	MOVE to SR
	MOVE to/from USP
	RESET
	RTE
	STOP
	*/
	
	always_comb begin
		unique case( lineBmap)
           
		// ori/andi/eori SR      
		'h01:   isPriv = ((opcode & 16'hf5ff) == 16'h007c);
      
		'h10:
			begin
			// No priority !!!
				if( (opcode & 16'hffc0) == 16'h46c0)        // move to sr
					isPriv = 1'b1;
				
				else if( (opcode & 16'hfff0) == 16'h4e60)   // move usp
					isPriv = 1'b1;
		
				else if(	opcode == 16'h4e70 ||			// reset
							opcode == 16'h4e73 ||			// rte
							opcode == 16'h4e72)				// stop
					isPriv = 1'b1;
				else
					isPriv = 1'b0;
			end
         
		default:   isPriv = 1'b0;
		endcase
	end

	
endmodule

// bin to one-hot, 4 bits to 16-bit bitmap
module onehotEncoder4( input [3:0] bin, output reg [15:0] bitMap);
	always_comb begin
	case( bin)
		'b0000:   bitMap = 16'h0001;
		'b0001:   bitMap = 16'h0002;
		'b0010:   bitMap = 16'h0004;
		'b0011:   bitMap = 16'h0008;
		'b0100:   bitMap = 16'h0010;
		'b0101:   bitMap = 16'h0020;
		'b0110:   bitMap = 16'h0040;
		'b0111:   bitMap = 16'h0080;
		'b1000:   bitMap = 16'h0100;
		'b1001:   bitMap = 16'h0200;
		'b1010:   bitMap = 16'h0400;
		'b1011:   bitMap = 16'h0800;
		'b1100:   bitMap = 16'h1000;
		'b1101:   bitMap = 16'h2000;
		'b1110:   bitMap = 16'h4000;
		'b1111:   bitMap = 16'h8000;
	endcase
	end
endmodule

// priority encoder
// used by MOVEM regmask
// this might benefit from device specific features
// MOVEM doesn't need speed, will read the result 2 CPU cycles after each update.
module pren( mask, hbit);
   parameter size = 16;
   parameter outbits = 4;

   input [size-1:0] mask;
   output reg [outbits-1:0] hbit;
   // output reg idle;
   
   always @( mask) begin
      integer i;
      hbit = 0;
      // idle = 1;
      for( i = size-1; i >= 0; i = i - 1) begin
          if( mask[ i]) begin
             hbit = i;
             // idle = 0;
         end
      end
   end

endmodule

// Microcode sequencer

module sequencer( input clk, enPhi1, enPhi2, extReset, pwrUp, input enT3,
	input [UROM_WIDTH-1:0] microLatch,
	input A0Err, BerrA, busAddrErr, Spuria, Avia,
	input Tpend, intPend, isIllegal, isPriv, excRst, isLineA, isLineF,
	input [15:0] psw,
	input prenEmpty, au05z, dcr4, ze, i11,
	input [1:0] alue01,
	input [15:0] Ird,
	input [UADDR_WIDTH-1:0] a1, a2, a3,
	output logic [3:0] tvn,
	output logic [UADDR_WIDTH-1:0] nma);
	
	logic [UADDR_WIDTH-1:0] uNma;
	logic [UADDR_WIDTH-1:0] grp1Nma;	
	logic [1:0] c0c1;
	reg a0Rst;
	wire A0Sel;
	wire inGrp0Exc;
	
	// assign nma = extReset ? RSTP0_NMA : (A0Err ? BSER1_NMA : uNma);
	// assign nma = A0Err ? (a0Rst ? RSTP0_NMA : BSER1_NMA) : uNma;
	
    // word type I: 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    // NMA :        .. .. 09 08 01 00 05 04 03 02 07 06 .. .. .. .. ..
	
	wire [UADDR_WIDTH-1:0] dbNma = { microLatch[ 14:13], microLatch[ 6:5], microLatch[ 10:7], microLatch[ 12:11]};

	// Group 0 exception.
	// Separated block from regular NMA. Otherwise simulation might depend on order of assigments.
	always_comb begin
		if( A0Err) begin
			if( a0Rst)					// Reset
				nma = RSTP0_NMA;
			else if( inGrp0Exc)			// Double fault
				nma = HALT1_NMA;
			else						// Bus or address error
				nma = BSER1_NMA;
		end
		else
			nma = uNma;
	end

	always_comb begin
		// Format II (conditional) or I (direct branch)
		if( microLatch[1])
			uNma = { microLatch[ 14:13], c0c1, microLatch[ 10:7], microLatch[ 12:11]};		
		else
			case( microLatch[ 3:2])
			0:   uNma = dbNma;   // DB
			1:   uNma = A0Sel ? grp1Nma : a1;
			2:   uNma = a2;
			3:   uNma = a3;
			endcase			
	end

	// Format II, conditional, NMA decoding
	wire [1:0] enl = { Ird[6], prenEmpty};		// Updated on T3
	
	wire [1:0] ms0 = { Ird[8], alue01[0]};
	wire [3:0] m01 = { au05z, Ird[8], alue01};
	wire [1:0] nz1 = { psw[ NF], psw[ ZF]};
	wire [1:0] nv  = { psw[ NF], psw[ VF]};
	
	logic ccTest;
	wire [4:0] cbc = microLatch[ 6:2];			// CBC bits
	
	always_comb begin
		unique case( cbc)
		'h0:    c0c1 = {i11, i11};						// W/L offset EA, from IRC

		'h1:    c0c1 = (au05z) ? 2'b01 : 2'b11;			// Updated on T3
		'h11:   c0c1 = (au05z) ? 2'b00 : 2'b11;

		'h02:   c0c1 = { 1'b0, ~psw[ CF]};				// C used in DIV
		'h12:   c0c1 = { 1'b1, ~psw[ CF]};

		'h03:   c0c1 = {psw[ ZF], psw[ ZF]};			// Z used in DIVU
		
		'h04:											// nz1, used in DIVS
			case( nz1)
               'b00:         c0c1 = 2'b10;
               'b10:         c0c1 = 2'b01;
               'b01,'b11:    c0c1 = 2'b11;
            endcase		

		'h05:   c0c1 = {psw[ NF], 1'b1};				// N used in CHK and DIV
		'h15:   c0c1 = {1'b1, psw[ NF]};
		
		// nz2, used in DIVS (same combination as nz1)
		'h06:   c0c1 = { ~nz1[1] & ~nz1[0], 1'b1};
		
		'h07:											// ms0 used in MUL
		case( ms0)
		 'b10, 'b00: c0c1 = 2'b11;
		 'b01: c0c1 = 2'b01;
		 'b11: c0c1 = 2'b10;
		endcase		
		
		'h08:											// m01 used in MUL
		case( m01)
		'b0000,'b0001,'b0100,'b0111:  c0c1 = 2'b11;
		'b0010,'b0011,'b0101:         c0c1 = 2'b01;
		'b0110:                       c0c1 = 2'b10;
		default:                      c0c1 = 2'b00;
		endcase

		// Conditional		
		'h09:   c0c1 = (ccTest) ? 2'b11 : 2'b01;
		'h19:   c0c1 = (ccTest) ? 2'b11 : 2'b10;
		
		// DCR bit 4 (high or low word)
		'h0c:   c0c1 = dcr4 ? 2'b01: 2'b11;
		'h1c:   c0c1 = dcr4 ? 2'b10: 2'b11;		
		
		// DBcc done
		'h0a:   c0c1 = ze ? 2'b11 : 2'b00;
		
		// nv, used in CHK
		'h0b:   c0c1 = (nv == 2'b00) ? 2'b00 : 2'b11;
		
		// V, used in trapv
		'h0d:   c0c1 = { ~psw[ VF], ~psw[VF]};		
		
		// enl, combination of pren idle and word/long on IRD
		'h0e,'h1e:
			case( enl)
			2'b00:	c0c1 = 'b10;
			2'b10:	c0c1 = 'b11;
			// 'hx1 result 00/01 depending on condition 0e/1e
			2'b01,2'b11:
					c0c1 = { 1'b0, microLatch[ 6]};
			endcase
			
		default: 				c0c1 = 'X;
		endcase
	end
	
	// CCR conditional
	always_comb begin
		unique case( Ird[ 11:8])		
		'h0: ccTest = 1'b1;						// T
		'h1: ccTest = 1'b0;						// F
		'h2: ccTest = ~psw[ CF] & ~psw[ ZF];	// HI
		'h3: ccTest = psw[ CF] | psw[ZF];		// LS
		'h4: ccTest = ~psw[ CF];				// CC (HS)
		'h5: ccTest = psw[ CF];					// CS (LO)
		'h6: ccTest = ~psw[ ZF];				// NE
		'h7: ccTest = psw[ ZF];					// EQ
		'h8: ccTest = ~psw[ VF];				// VC
		'h9: ccTest = psw[ VF];					// VS
		'ha: ccTest = ~psw[ NF];				// PL
		'hb: ccTest = psw[ NF];					// MI
		'hc: ccTest = (psw[ NF] & psw[ VF]) | (~psw[ NF] & ~psw[ VF]);				// GE
		'hd: ccTest = (psw[ NF] & ~psw[ VF]) | (~psw[ NF] & psw[ VF]);				// LT
		'he: ccTest = (psw[ NF] & psw[ VF] & ~psw[ ZF]) |
				 (~psw[ NF] & ~psw[ VF] & ~psw[ ZF]);								// GT
		'hf: ccTest = psw[ ZF] | (psw[ NF] & ~psw[VF]) | (~psw[ NF] & psw[VF]);		// LE
		endcase
	end
	
	// Exception logic
	logic rTrace, rInterrupt;
	logic rIllegal, rPriv, rLineA, rLineF;
	logic rExcRst, rExcAdrErr, rExcBusErr;
	logic rSpurious, rAutovec;
	wire grp1LatchEn, grp0LatchEn;
			
	// Originally control signals latched on T4. Then exception latches updated on T3
	assign grp1LatchEn = microLatch[0] & (microLatch[1] | !microLatch[4]);
	assign grp0LatchEn = microLatch[4] & !microLatch[1];
	
	assign inGrp0Exc = rExcRst | rExcBusErr | rExcAdrErr;
	
	always_ff @( posedge clk) begin
		if( grp0LatchEn & enT3) begin
			rExcRst <= excRst;
			rExcBusErr <= BerrA;
			rExcAdrErr <= busAddrErr;
			rSpurious <= Spuria;
			rAutovec <= Avia;
		end
		
		// Update group 1 exception latches
		// Inputs from IR decoder updated on T1 as soon as IR loaded
		// Trace pending updated on T3 at the start of the instruction
		// Interrupt pending on T2
		if( grp1LatchEn & enT3) begin
			rTrace <= Tpend;
			rInterrupt <= intPend;
			rIllegal <= isIllegal & ~isLineA & ~isLineF;
			rLineA <= isLineA;
			rLineF <= isLineF;
			rPriv <= isPriv & !psw[ SF];
		end
	end

	// exception priority
	always_comb begin
		grp1Nma = TRAC1_NMA;
		if( rExcRst)
			tvn = '0;							// Might need to change that to signal in exception
		else if( rExcBusErr | rExcAdrErr)
			tvn = { 1'b1, rExcAdrErr};
			
		// Seudo group 0 exceptions. Just for updating TVN
		else if( rSpurious | rAutovec)
			tvn = rSpurious ? TVN_SPURIOUS : TVN_AUTOVEC;
		
		else if( rTrace)
			tvn = 9;
		else if( rInterrupt) begin
			tvn = TVN_INTERRUPT;
			grp1Nma = ITLX1_NMA;
		end
		else begin
			unique case( 1'b1)					// Can't happen more than one of these
			rIllegal:			tvn = 4;
			rPriv:				tvn = 8;
			rLineA:				tvn = 10;
			rLineF:				tvn = 11;
			default:			tvn = 1;		// Signal no group 0/1 exception
			endcase
		end
	end
	
	assign A0Sel = rIllegal | rLineF | rLineA | rPriv | rTrace | rInterrupt;
	
	always_ff @( posedge clk) begin
		if( extReset)
			a0Rst <= 1'b1;
		else if( enT3)
			a0Rst <= 1'b0;
	end
	
endmodule


//
// DMA/BUS Arbitration
//

module busArbiter( input clk, enPhi1, enPhi2, extReset, pwrUp,
		input BRi, BgackI, Halti, bgBlock,
		output busAvail,
		output logic BGn);
		
	enum int unsigned { DRESET = 0, DIDLE, D1, D_BR, D_BA, D_BRA, D3, D2} dmaPhase, next;

	always_comb begin
		case(dmaPhase)
		DRESET:	next = DIDLE;
		DIDLE:	begin
				if( bgBlock)
					next = DIDLE;
				else if( ~BgackI)
					next = D_BA;
				else if( ~BRi)
					next = D1;
				else
					next = DIDLE;
				end

		D_BA:	begin							// Loop while only BGACK asserted, BG negated here
				if( ~BRi & !bgBlock)
					next = D3;
				else if( ~BgackI & !bgBlock)
					next = D_BA;
				else
					next = DIDLE;
				end
				
		D1:		next = D_BR;							// Loop while only BR asserted
		D_BR:	next = ~BRi & BgackI ? D_BR : D_BA;		// No direct path to IDLE !
		
		D3:		next = D_BRA;
		D_BRA:	begin						// Loop while both BR and BGACK asserted
				case( {BgackI, BRi} )
				2'b11:	next = DIDLE;		// Both deasserted
				2'b10:	next = D_BR;		// BR asserted only
				2'b01:	next = D2;			// BGACK asserted only
				2'b00:	next = D_BRA;		// Stay here while both asserted
				endcase
				end
				
		// Might loop here if both deasserted, should normally don't arrive here anyway?
		// D2:		next = (BgackI & BRi) | bgBlock ? D2: D_BA;

		D2:		next = D_BA;
		
		default:	next = DIDLE;			// Should not reach here normally		
		endcase
	end
		
	logic granting;
	always_comb begin
		unique case( next)
		D1, D3, D_BR, D_BRA:	granting = 1'b1;
		default:				granting = 1'b0;
		endcase
	end
	
	reg rGranted;
	assign busAvail = Halti & BRi & BgackI & ~rGranted;
		
	always_ff @( posedge clk) begin
		if( extReset) begin
			dmaPhase <= DRESET;
			rGranted <= 1'b0;
		end
		else if( enPhi2) begin
			dmaPhase <= next;
			// Internal signal changed on PHI2
			rGranted <= granting;
		end

		// External Output changed on PHI1
		if( extReset)
			BGn <= 1'b1;
		else if( enPhi1)
			BGn <= ~rGranted;
		
	end
			
endmodule

module busControl( input clk, enPhi1, enPhi2, extReset, pwrUp, input enT1, input enT4,
		input permStart, permStop, iStop,
		input aob0,
		input isWrite, isByte, isRmc,
		input busAvail,
		output bgBlock,
		output busAddrErr,
		output waitBusCycle,
		output busStarting,			// Asserted during S0
		output logic addrOe,		// Asserted from S1 to the end, whole bus cycle except S0
		output bciWrite,			// Used for SSW on bus/addr error
		
		input rDtack, BeDebounced, Vpai,
		output ASn, output LDSn, output UDSn, eRWn);

	reg rAS, rLDS, rUDS, rRWn;
	assign ASn = rAS;
	assign LDSn = rLDS;
	assign UDSn = rUDS;
	assign eRWn = rRWn;

	reg dataOe;
		
	reg bcPend;
	reg isWriteReg, bciByte, isRmcReg, wendReg;
	assign bciWrite = isWriteReg;
	reg addrOeDelay;
	reg isByteT4;

	wire canStart, busEnd;
	wire bcComplete, bcReset;
	
	wire isRcmReset = bcComplete & bcReset & isRmcReg;
	
	assign busAddrErr = aob0 & ~bciByte;
	
	// Bus retry not really supported.
	// It's BERR and HALT and not address error, and not read-modify cycle.
	wire busRetry = ~busAddrErr & 1'b0;
	
	enum int unsigned { SRESET = 0, SIDLE, S0, S2, S4, S6, SRMC_RES} busPhase, next;

	always_ff @( posedge clk) begin
		if( extReset)
			busPhase <= SRESET;
		else if( enPhi1)
			busPhase <= next;
	end
	
	always_comb begin
		case( busPhase)
		SRESET:		next = SIDLE;
		SRMC_RES:	next = SIDLE;			// Single cycle special state when read phase of RMC reset
		S0:			next = S2;
		S2:			next = S4;
		S4:			next = busEnd ? S6 : S4;
		S6:			next = isRcmReset ? SRMC_RES : (canStart ? S0 : SIDLE);
		SIDLE:		next = canStart ? S0 : SIDLE;
		default:	next = SIDLE;
		endcase
	end	
	
	// Idle phase of RMC bus cycle. Might be better to just add a new state
	wire rmcIdle = (busPhase == SIDLE) & ~ASn & isRmcReg;
		
	assign canStart = (busAvail | rmcIdle) & (bcPend | permStart) & !busRetry & !bcReset;
		
	wire busEnding = (next == SIDLE) | (next == S0);
	
	assign busStarting = (busPhase == S0);
		
	// term signal (DTACK, BERR, VPA, adress error)
	assign busEnd = ~rDtack | iStop;
				
	// bcComplete asserted on raising edge of S6 (together with SNC).
	assign bcComplete = (busPhase == S6);
	
	// Clear bus info latch on completion (regular or aborted) and no bus retry (and not PHI1).
	// bciClear asserted half clock later on PHI2, and bci latches cleared async concurrently
	wire bciClear = bcComplete & ~busRetry;
	
	// Reset on reset or (berr & berrDelay & (not halt or rmc) & not 6800 & in bus cycle) (and not PHI1)
	assign bcReset = extReset | (addrOeDelay & BeDebounced & Vpai);
		
	// Enable uclock only on S6 (S8 on Bus Error) or not bciPermStop
	assign waitBusCycle = wendReg & !bcComplete;
	
	// Block Bus Grant when starting new bus cycle. But No need if AS already asserted (read phase of RMC)
	// Except that when that RMC phase aborted on bus error, it's asserted one cycle later!
	assign bgBlock = ((busPhase == S0) & ASn) | (busPhase == SRMC_RES);
	
	always_ff @( posedge clk) begin
		if( extReset) begin
			addrOe <= 1'b0;
		end
		else if( enPhi2 & ( busPhase == S0))			// From S1, whole bus cycle except S0
				addrOe <= 1'b1;
		else if( enPhi1 & (busPhase == SRMC_RES))
			addrOe <= 1'b0;
		else if( enPhi1 & ~isRmcReg & busEnding)
			addrOe <= 1'b0;
			
		if( enPhi1)
			addrOeDelay <= addrOe;
		
		if( extReset) begin
			rAS <= 1'b1;
			rUDS <= 1'b1;
			rLDS <= 1'b1;
			rRWn <= 1'b1;
			dataOe <= '0;
		end
		else begin

			if( enPhi2 & isWriteReg & (busPhase == S2))
				dataOe <= 1'b1;
			else if( enPhi1 & (busEnding | (busPhase == SIDLE)) )
				dataOe <= 1'b0;
						
			if( enPhi1 & busEnding)
				rRWn <= 1'b1;
			else if( enPhi1 & isWriteReg) begin
				// Unlike LDS/UDS Asserted even in address error
				if( (busPhase == S0) & isWriteReg)
					rRWn <= 1'b0;
			end

			// AS. Actually follows addrOe half cycle later!
			if( enPhi1 & (busPhase == S0))
				rAS <= 1'b0;
			else if( enPhi2 & (busPhase == SRMC_RES))		// Bus error on read phase of RMC. Deasserted one cycle later
				rAS <= 1'b1;			
			else if( enPhi2 & bcComplete & ~SRMC_RES)
				if( ~isRmcReg)									// Keep AS asserted on the IDLE phase of RMC
					rAS <= 1'b1;
			
			if( enPhi1 & (busPhase == S0)) begin
				if( ~isWriteReg & !busAddrErr) begin
					rUDS <= ~(~bciByte | ~aob0);
					rLDS <= ~(~bciByte | aob0);
				end
			end
			else if( enPhi1 & isWriteReg & (busPhase == S2) & !busAddrErr) begin
					rUDS <= ~(~bciByte | ~aob0);
					rLDS <= ~(~bciByte | aob0);
			end		
			else if( enPhi2 & bcComplete) begin
				rUDS <= 1'b1;
				rLDS <= 1'b1;
			end
			
		end
		
	end
	
	// Bus cycle info latch. Needed because uinstr might change if the bus is busy and we must wait.
	// Note that urom advances even on wait states. It waits *after* updating urom and nanorom latches.
	// Even without wait states, ublocks of type ir (init reading) will not wait for bus completion.
	// Originally latched on (permStart AND T1).
	
	// Bus cycle info latch: isRead, isByte, read-modify-cycle, and permStart (bus cycle pending). Some previously latched on T4?
	// permStop also latched, but unconditionally on T1
	
	// Might make more sense to register this outside this module
	always_ff @( posedge clk) begin
		if( enT4) begin
			isByteT4 <= isByte;
		end
	end
	
	// Bus Cycle Info Latch
	always_ff @( posedge clk) begin
		if( pwrUp) begin
			bcPend <= 1'b0;
			wendReg <= 1'b0;	
			isWriteReg <= 1'b0;
			bciByte <= 1'b0;
			isRmcReg <= 1'b0;		
		end
		
		else if( enPhi2 & (bciClear | bcReset)) begin
			bcPend <= 1'b0;
			wendReg <= 1'b0;
		end
		else begin
			if( enT1 & permStart) begin
				isWriteReg <= isWrite;
				bciByte <= isByteT4;
				isRmcReg <= isRmc & ~isWrite;	// We need special case the end of the read phase only.
				bcPend <= 1'b1;
			end
			if( enT1)
				wendReg <= permStop;
		end
	end
			
endmodule

//
// microrom and nanorom instantiation
//
// There is bit of wasting of resources here. An extra registering pipeline happens that is not needed.
// This is just for the purpose of helping inferring block RAM using pure generic code. Inferring RAM is important for performance.
// Might be more efficient to use vendor specific features such as clock enable.
//

module uRom( input clk, input [UADDR_WIDTH-1:0] microAddr, output logic [UROM_WIDTH-1:0] microOutput);
	reg [UROM_WIDTH-1:0] uRam[ UROM_DEPTH];		
	initial begin
		$readmemb("fx68k_microrom.mem", uRam);
	end
	
	always_ff @( posedge clk) 
		microOutput <= uRam[ microAddr];
endmodule


module nanoRom( input clk, input [NADDR_WIDTH-1:0] nanoAddr, output logic [NANO_WIDTH-1:0] nanoOutput);
	reg [NANO_WIDTH-1:0] nRam[ NANO_DEPTH];		
	initial begin
		$readmemb("fx68k_nanorom.mem", nRam);
	end
	
	always_ff @( posedge clk) 
		nanoOutput <= nRam[ nanoAddr];
endmodule

// Translate uaddr to nanoaddr
module microToNanoAddr(
	input [UADDR_WIDTH-1:0] uAddr,
	output [NADDR_WIDTH-1:0] orgAddr);

	wire [UADDR_WIDTH-1:2] baseAddr = uAddr[UADDR_WIDTH-1:2];
	logic [NADDR_WIDTH-1:2] orgBase;
	assign orgAddr = { orgBase, uAddr[1:0]};

	always @( baseAddr)
	begin
		// nano ROM (136 addresses)
		case( baseAddr)

'h00: orgBase = 7'h0 ;
'h01: orgBase = 7'h1 ;
'h02: orgBase = 7'h2 ;
'h03: orgBase = 7'h2 ;
'h08: orgBase = 7'h3 ;
'h09: orgBase = 7'h4 ;
'h0A: orgBase = 7'h5 ;
'h0B: orgBase = 7'h5 ;
'h10: orgBase = 7'h6 ;
'h11: orgBase = 7'h7 ;
'h12: orgBase = 7'h8 ;
'h13: orgBase = 7'h8 ;
'h18: orgBase = 7'h9 ;
'h19: orgBase = 7'hA ;
'h1A: orgBase = 7'hB ;
'h1B: orgBase = 7'hB ;
'h20: orgBase = 7'hC ;
'h21: orgBase = 7'hD ;
'h22: orgBase = 7'hE ;
'h23: orgBase = 7'hD ;
'h28: orgBase = 7'hF ;
'h29: orgBase = 7'h10 ;
'h2A: orgBase = 7'h11 ;
'h2B: orgBase = 7'h10 ;
'h30: orgBase = 7'h12 ;
'h31: orgBase = 7'h13 ;
'h32: orgBase = 7'h14 ;
'h33: orgBase = 7'h14 ;
'h38: orgBase = 7'h15 ;
'h39: orgBase = 7'h16 ;
'h3A: orgBase = 7'h17 ;
'h3B: orgBase = 7'h17 ;
'h40: orgBase = 7'h18 ;
'h41: orgBase = 7'h18 ;
'h42: orgBase = 7'h18 ;
'h43: orgBase = 7'h18 ;
'h44: orgBase = 7'h19 ;
'h45: orgBase = 7'h19 ;
'h46: orgBase = 7'h19 ;
'h47: orgBase = 7'h19 ;
'h48: orgBase = 7'h1A ;
'h49: orgBase = 7'h1A ;
'h4A: orgBase = 7'h1A ;
'h4B: orgBase = 7'h1A ;
'h4C: orgBase = 7'h1B ;
'h4D: orgBase = 7'h1B ;
'h4E: orgBase = 7'h1B ;
'h4F: orgBase = 7'h1B ;
'h54: orgBase = 7'h1C ;
'h55: orgBase = 7'h1D ;
'h56: orgBase = 7'h1E ;
'h57: orgBase = 7'h1F ;
'h5C: orgBase = 7'h20 ;
'h5D: orgBase = 7'h21 ;
'h5E: orgBase = 7'h22 ;
'h5F: orgBase = 7'h23 ;
'h70: orgBase = 7'h24 ;
'h71: orgBase = 7'h24 ;
'h72: orgBase = 7'h24 ;
'h73: orgBase = 7'h24 ;
'h74: orgBase = 7'h24 ;
'h75: orgBase = 7'h24 ;
'h76: orgBase = 7'h24 ;
'h77: orgBase = 7'h24 ;
'h78: orgBase = 7'h25 ;
'h79: orgBase = 7'h25 ;
'h7A: orgBase = 7'h25 ;
'h7B: orgBase = 7'h25 ;
'h7C: orgBase = 7'h25 ;
'h7D: orgBase = 7'h25 ;
'h7E: orgBase = 7'h25 ;
'h7F: orgBase = 7'h25 ;
'h84: orgBase = 7'h26 ;
'h85: orgBase = 7'h27 ;
'h86: orgBase = 7'h28 ;
'h87: orgBase = 7'h29 ;
'h8C: orgBase = 7'h2A ;
'h8D: orgBase = 7'h2B ;
'h8E: orgBase = 7'h2C ;
'h8F: orgBase = 7'h2D ;
'h94: orgBase = 7'h2E ;
'h95: orgBase = 7'h2F ;
'h96: orgBase = 7'h30 ;
'h97: orgBase = 7'h31 ;
'h9C: orgBase = 7'h32 ;
'h9D: orgBase = 7'h33 ;
'h9E: orgBase = 7'h34 ;
'h9F: orgBase = 7'h35 ;
'hA4: orgBase = 7'h36 ;
'hA5: orgBase = 7'h36 ;
'hA6: orgBase = 7'h37 ;
'hA7: orgBase = 7'h37 ;
'hAC: orgBase = 7'h38 ;
'hAD: orgBase = 7'h38 ;
'hAE: orgBase = 7'h39 ;
'hAF: orgBase = 7'h39 ;
'hB4: orgBase = 7'h3A ;
'hB5: orgBase = 7'h3A ;
'hB6: orgBase = 7'h3B ;
'hB7: orgBase = 7'h3B ;
'hBC: orgBase = 7'h3C ;
'hBD: orgBase = 7'h3C ;
'hBE: orgBase = 7'h3D ;
'hBF: orgBase = 7'h3D ;
'hC0: orgBase = 7'h3E ;
'hC1: orgBase = 7'h3F ;
'hC2: orgBase = 7'h40 ;
'hC3: orgBase = 7'h41 ;
'hC8: orgBase = 7'h42 ;
'hC9: orgBase = 7'h43 ;
'hCA: orgBase = 7'h44 ;
'hCB: orgBase = 7'h45 ;
'hD0: orgBase = 7'h46 ;
'hD1: orgBase = 7'h47 ;
'hD2: orgBase = 7'h48 ;
'hD3: orgBase = 7'h49 ;
'hD8: orgBase = 7'h4A ;
'hD9: orgBase = 7'h4B ;
'hDA: orgBase = 7'h4C ;
'hDB: orgBase = 7'h4D ;
'hE0: orgBase = 7'h4E ;
'hE1: orgBase = 7'h4E ;
'hE2: orgBase = 7'h4F ;
'hE3: orgBase = 7'h4F ;
'hE8: orgBase = 7'h50 ;
'hE9: orgBase = 7'h50 ;
'hEA: orgBase = 7'h51 ;
'hEB: orgBase = 7'h51 ;
'hF0: orgBase = 7'h52 ;
'hF1: orgBase = 7'h52 ;
'hF2: orgBase = 7'h52 ;
'hF3: orgBase = 7'h52 ;
'hF8: orgBase = 7'h53 ;
'hF9: orgBase = 7'h53 ;
'hFA: orgBase = 7'h53 ;
'hFB: orgBase = 7'h53 ;

			default:
				orgBase = 'X;
		endcase
	end

endmodule

//
// For compilation test only
//

`ifdef FX68K_TEST
module fx68kTop( input clk32, 
	input extReset,
	// input pwrUp,

	input DTACKn, input VPAn,
	input BERRn,
	input BRn, BGACKn,
	input IPL0n, input IPL1n, input IPL2n,
	input [15:0] iEdb,
	
	output [15:0] oEdb,
	output eRWn, output ASn, output LDSn, output UDSn,
	output logic E, output VMAn,	
	output FC0, output FC1, output FC2,
	output BGn,
	output oRESETn, output oHALTEDn,
	output [23:1] eab
	);

	// Clock must be at least twice the desired frequency. A 32 MHz clock means a maximum 16 MHz effective frequency.
	// In this example we divide the clock by 4. Resulting on an effective processor running at 8 MHz.
	
	reg [1:0] clkDivisor = '0;
	always @( posedge clk32) begin
		clkDivisor <= clkDivisor + 1'b1;
	end
		
	/*
	These two signals must be a single cycle pulse. They don't need to be registered.
	Same signal can't be asserted twice in a row. Other than that there are no restrictions.
	There can be any number of cycles, or none, even variable non constant cycles, between each pulse.
	*/
	
	wire enPhi1 = (clkDivisor == 2'b11);
	wire enPhi2 = (clkDivisor == 2'b01);
		
		
	fx68k fx68k( .clk( clk32),
		.extReset, .pwrUp( extReset), .enPhi1, .enPhi2,
	
		.DTACKn, .VPAn, .BERRn, .BRn, .BGACKn,
		.IPL0n, .IPL1n, .IPL2n,
		.iEdb,
		
		.oEdb,
		.eRWn, .ASn, .LDSn, .UDSn,
		.E, .VMAn,	
		.FC0, .FC1, .FC2,
		.BGn,
		.oRESETn, .oHALTEDn, .eab);

endmodule
`endif
/* verilator lint_off WIDTH */
//
// FX 68K
//
// M68K cycle accurate, fully synchronous
// Copyright (c) 2018 by Jorge Cwik
//
// ALU
//

`timescale 1 ns / 1 ns

localparam MASK_NBITS = 5;

localparam
		OP_AND = 1,
		OP_SUB = 2, OP_SUBX = 3, OP_ADD = 4,
		OP_EXT = 5, OP_SBCD = 6, OP_SUB0 = 7,
		OP_OR = 8, OP_EOR = 9,
		OP_SUBC = 10, OP_ADDC = 11, OP_ADDX = 12,
		OP_ASL = 13,
		OP_ASR = 14,
		OP_LSL = 15,
		OP_LSR = 16,
		OP_ROL = 17,
		OP_ROR = 18,
		OP_ROXL = 19,
		OP_ROXR = 20,
		OP_SLAA = 21,
		OP_ABCD = 22;

module fx68kAlu ( input clk, pwrUp, enT1, enT3, enT4,
	input [15:0] ird,
	input [2:0] aluColumn,
	input [1:0] aluDataCtrl,
	input aluAddrCtrl, alueClkEn, ftu2Ccr, init, finish, aluIsByte,
	input [15:0] ftu,
	input [15:0] alub,
	input [15:0] iDataBus, input [15:0] iAddrBus,
	output ze,
	output reg [15:0] alue,
	output reg [7:0] ccr,
	output [15:0] aluOut);


`define ALU_ROW_01		16'h0002
`define ALU_ROW_02		16'h0004
`define ALU_ROW_03		16'h0008
`define ALU_ROW_04		16'h0010
`define ALU_ROW_05		16'h0020
`define ALU_ROW_06		16'h0040
`define ALU_ROW_07		16'h0080
`define ALU_ROW_08		16'h0100
`define ALU_ROW_09		16'h0200
`define ALU_ROW_10		16'h0400
`define ALU_ROW_11		16'h0800
`define ALU_ROW_12		16'h1000
`define ALU_ROW_13		16'h2000
`define ALU_ROW_14		16'h4000
`define ALU_ROW_15		16'h8000
	
	
	// Bit positions for flags in CCR
	localparam CF = 0, VF = 1, ZF = 2, NF = 3, XF = 4;

	reg [15:0] aluLatch;
	reg [4:0] pswCcr;
	reg [4:0] ccrCore;
	
	logic [15:0] result;
	logic [4:0] ccrTemp;
	reg coreH;		// half carry latch

	logic [15:0] subResult;
	logic subHcarry;
	logic subCout, subOv;

	assign aluOut = aluLatch;
	assign ze = ~ccrCore[ ZF];		// Check polarity !!!

	//
	// Control
	//	Signals derived from IRD *must* be registered on either T3 or T4
	//  Signals derived from nano rom can be registered on T4.

	reg [15:0] row;
	reg isArX;									// Don't set Z
	reg noCcrEn;
	reg isByte;
	
	reg [4:0] ccrMask;
	reg [4:0] oper;
	
	logic [15:0] aOperand, dOperand;
	wire isCorf = ( aluDataCtrl == 2'b10);

	wire [15:0] cRow;
	wire cIsArX;
	wire cNoCcrEn;
	rowDecoder rowDecoder( .ird( ird), .row( cRow), .noCcrEn( cNoCcrEn), .isArX( cIsArX));
	
	// Get Operation & CCR Mask from row/col
	// Registering them on T4 increase performance. But slowest part seems to be corf !
	wire [4:0] cMask;
	wire [4:0] aluOp;

	aluGetOp aluGetOp( .row, .col( aluColumn), .isCorf, .aluOp);	
	ccrTable ccrTable( .col( aluColumn), .row( row), .finish, .ccrMask( cMask));

	// Inefficient, uCode could help !
	wire shftIsMul = row[7];
	wire shftIsDiv = row[1];

	wire [31:0] shftResult;
	reg [7:0] bcdLatch;
	reg bcdCarry, bcdOverf;
		
	reg isLong;
	reg rIrd8;
	logic isShift;
	logic shftCin, shftRight, addCin;
	
	// Register some decoded signals	
	always_ff @( posedge clk) begin
		if( enT3) begin
			row <= cRow;
			isArX <= cIsArX;
			noCcrEn <= cNoCcrEn;
			rIrd8 <= ird[8];
			isByte <= aluIsByte;
		end
		
		if( enT4) begin
			// Decode if long shift
			// MUL and DIV are long (but special !)
			isLong <= (ird[7] & ~ird[6]) | shftIsMul | shftIsDiv;			
			
			ccrMask <= cMask;
			oper <= aluOp;
		end
	end
	
	
	always_comb begin
		// Dest (addr) operand source
		// If aluCsr (depends on column/row) addrbus is shifted !!
		aOperand = (aluAddrCtrl ? alub : iAddrBus);
		
		// Second (data,source) operand mux
		case( aluDataCtrl)
			2'b00:				dOperand = iDataBus;
			2'b01:				dOperand = 'h0000;
			2'b11:				dOperand = 'hffff;
			// 2'b10:				dOperand = bcdResult;
			2'b10:				dOperand = 'X;
		endcase
	end
	
	// Execution
	   
	// shift operand MSB. Input in ASR/ROL. Carry in right.
	// Can't be registered because uses bus operands that aren't available early !
	wire shftMsb = isLong ? alue[15] : (isByte ? aOperand[7] : aOperand[15]);
	   
	aluShifter shifter( .data( { alue, aOperand}),
		.swapWords( shftIsMul | shftIsDiv),
		.cin( shftCin), .dir( shftRight), .isByte( isByte), .isLong( isLong),
		.result( shftResult));
				
	wire [7:0] bcdResult;
	wire bcdC, bcdV;
	aluCorf aluCorf( .binResult( aluLatch[7:0]), .hCarry( coreH),
		.bAdd( (oper != OP_SBCD) ), .cin( pswCcr[ XF]),
		.bcdResult( bcdResult), .dC( bcdC), .ov( bcdV));

	// BCD adjust is among the slowest processing on ALU !
	// Precompute and register BCD result on T1
	// We don't need to wait for execution buses because corf is always added to ALU previous result
	always_ff @( posedge clk)
		if( enT1) begin
			bcdLatch <= bcdResult;
			bcdCarry <= bcdC;
			bcdOverf <= bcdV;
		end
		
	// Adder carry in selector
	always_comb
	begin
		case( oper)
			OP_ADD, OP_SUB:   addCin = 1'b0;
			OP_SUB0:          addCin = 1'b1;			// NOT = 0 - op - 1
			OP_ADDC,OP_SUBC:  addCin = ccrCore[ CF];
			OP_ADDX,OP_SUBX:  addCin = pswCcr[ XF];
			default: addCin = 1'bX;
		endcase
	end
   	
	// Shifter carry in and direction selector
	always_comb begin
		case( oper)
		OP_LSL, OP_ASL, OP_ROL, OP_ROXL, OP_SLAA:   shftRight = 1'b0;
		OP_LSR, OP_ASR, OP_ROR, OP_ROXR:            shftRight = 1'b1;
		default:									shftRight = 1'bX;
		endcase
		
		case( oper)
		OP_LSR,
		OP_ASL,
		OP_LSL:		shftCin = 1'b0;
		OP_ROL,
		OP_ASR:		shftCin = shftMsb;
		OP_ROR:		shftCin = aOperand[0];
		OP_ROXL,
		OP_ROXR:
			if( shftIsMul)
				shftCin = rIrd8 ? pswCcr[NF] ^ pswCcr[VF] : pswCcr[ CF];
			else
				shftCin = pswCcr[ XF];
				
		OP_SLAA:	shftCin = aluColumn[1];   // col4 -> 0, col 6-> 1
		default:	shftCin = 'X;
		endcase
	end
	
	// ALU operation selector
	always_comb begin	      

		// sub is DATA - ADDR	    
		mySubber( aOperand, dOperand, addCin,
			(oper == OP_ADD) | (oper == OP_ADDC) | (oper == OP_ADDX),
			isByte, subResult, subCout, subOv);
			
		isShift = 1'b0;
		case( oper)
		OP_AND: result = aOperand & dOperand;
		OP_OR:  result = aOperand | dOperand;
		OP_EOR: result = aOperand ^ dOperand;
		
		OP_EXT: result = { {8{aOperand[7]}}, aOperand[7:0]};
		
		OP_SLAA,
		OP_ASL, OP_ASR,
		OP_LSL, OP_LSR,
		OP_ROL, OP_ROR,
		OP_ROXL, OP_ROXR:
			begin
				result = shftResult[15:0];
				isShift = 1'b1;
			end
			
		OP_ADD,
		OP_ADDC,
		OP_ADDX,
		OP_SUB,
		OP_SUBC,
		OP_SUB0,
		OP_SUBX:	result = subResult;
		
		OP_ABCD,
		OP_SBCD:	result = { 8'hXX, bcdLatch};

		default:	result = 'X;
		endcase	   
	end
	
	task mySubber;
		input [15:0] inpa, inpb;
		input cin, bAdd, isByte;
		output reg [15:0] result;
		output cout, ov;

		// Not very efficient!
		logic [16:0] rtemp;
		logic rm,sm,dm,tsm;

		begin
			if( isByte)
			begin
				rtemp = bAdd ? { 1'b0, inpb[7:0]} + { 1'b0, inpa[7:0]} + cin:
								{ 1'b0, inpb[7:0] } - { 1'b0, inpa[7:0]} - cin;
				result = { {8{ rtemp[7]}}, rtemp[7:0]};
				cout = rtemp[8];
			end
			else begin
				rtemp = bAdd ? { 1'b0, inpb } + { 1'b0, inpa} + cin:
								{ 1'b0, inpb } - { 1'b0, inpa} - cin;
				result = rtemp[ 15:0];
				cout = rtemp[16];
			end
	        
			rm  = isByte ? rtemp[7] : rtemp[15];
			dm  = isByte ? inpb[ 7] : inpb[ 15];
			tsm = isByte ? inpa[ 7] : inpa[ 15];
			sm  = bAdd ? tsm : ~tsm;

			ov = (sm & dm & ~rm) | (~sm & ~dm & rm);
			
			// Store half carry for bcd correction
			subHcarry = inpa[4] ^ inpb[4] ^ rtemp[4];
		
		end
	endtask
	
	
	// CCR flags process
	always_comb begin
		
		ccrTemp[XF] = pswCcr[XF];     ccrTemp[CF] = 0;     ccrTemp[VF] = 0;	

		// Not on all operators !!!
		ccrTemp[ ZF] = isByte ? ~(| result[7:0]) : ~(| result);
		ccrTemp[ NF] = isByte ? result[7] : result[15];

		unique case( oper)
		
		OP_EXT:
			// Division overflow.
			if( aluColumn == 5) begin
				ccrTemp[VF] = 1'b1;		ccrTemp[NF] = 1'b1;
			end

		OP_SUB0,				// used by NOT
		OP_OR,
		OP_EOR:
			begin
				ccrTemp[CF] = 0;      ccrTemp[VF] = 0;
			end

		OP_AND:
			begin
				// ROXL/ROXR indeed copy X to C in column 1 (OP_AND), executed before entering the loop.
				// Needed when rotate count is zero, the ucode with the ROX operator never reached.
				// 	C must be set to the value of X, X remains unaffected.
				if( (aluColumn == 1) & (row[11] | row[8]))
					ccrTemp[CF] = pswCcr[XF];
				else
					ccrTemp[CF] = 0;
				ccrTemp[VF] = 0;
			end
			
		// Assumes col 3 of DIV use C and not X !
		// V will be set in other cols (2/3) of DIV
		OP_SLAA:   ccrTemp[ CF] = aOperand[15];
		
		OP_LSL,OP_ROXL:
			begin
				ccrTemp[ CF] = shftMsb;
				ccrTemp[ XF] = shftMsb;
				ccrTemp[ VF] = 1'b0;
			end
			
		OP_LSR,OP_ROXR:
			begin
				// 0 Needed for mul, or carry gets in high word
				ccrTemp[ CF] = shftIsMul ? 1'b0 : aOperand[0];
				ccrTemp[ XF] = aOperand[0];
				// Not relevant for MUL, we clear it at mulm6 (1f) anyway.
				// Not that MUL can never overlow!
				ccrTemp[ VF] = 0;
				// Z is checking here ALU (low result is actually in ALUE).
				// But it is correct, see comment above.
			end
			
		OP_ASL:
			begin
				ccrTemp[ XF] = shftMsb;			ccrTemp[ CF] = shftMsb;
				// V set if msb changed on any shift.
				// Otherwise clear previously on OP_AND (col 1i).
				ccrTemp[ VF] = pswCcr[VF] | (shftMsb ^
					(isLong ? alue[15-1] : (isByte ? aOperand[7-1] : aOperand[15-1])) );
			end
		OP_ASR:
			begin
				ccrTemp[ XF] = aOperand[0];		ccrTemp[ CF] = aOperand[0];
				ccrTemp[ VF] = 0;
			end
      
		// X not changed on ROL/ROR !
		OP_ROL:		ccrTemp[ CF] = shftMsb;
		OP_ROR:		ccrTemp[ CF] = aOperand[0];

		OP_ADD,
		OP_ADDC,
		OP_ADDX,
		OP_SUB,
		OP_SUBC,
		OP_SUBX:
			begin
				ccrTemp[ CF] = subCout;
				ccrTemp[ XF] = subCout;
				ccrTemp[ VF] = subOv;
			end

		OP_ABCD,
		OP_SBCD:
			begin
				ccrTemp[ XF] = bcdCarry;
				ccrTemp[ CF] = bcdCarry;
				ccrTemp[ VF] = bcdOverf;
			end
		
		endcase
				
	end
	
	// Core and psw latched at the same cycle
	
	// CCR filter
	// CCR out mux for Z & C flags
	// Z flag for 32-bit result
	// Not described, but should be used also for instructions
	//   that clear but not set Z (ADDX/SUBX/ABCD, etc)!
	logic [4:0] ccrMasked;
	always_comb begin
		ccrMasked = (ccrTemp & ccrMask) | (pswCcr & ~ccrMask);
		if( finish | isCorf | isArX)
			ccrMasked[ ZF] = ccrTemp[ ZF] & pswCcr[ ZF];
	end
		
	always_ff @( posedge clk) begin
		if( enT3) begin
			// Update latches from ALU operators
			if( (| aluColumn)) begin
				aluLatch <= result;
				
				coreH <= subHcarry;
			   
				// Update CCR core
				if( (| aluColumn))
					ccrCore <= ccrTemp;		// Most bits not really used
			end

			if( alueClkEn)
				alue <= iDataBus;
			else if( isShift & (| aluColumn))
				alue <= shftResult[31:16];
		end
				
		// CCR
		// Originally on T3-T4 edge pulse !!
		// Might be possible to update on T4 (but not after T0) from partial result registered on T3, it will increase performance!
		if( pwrUp)
			pswCcr <= '0;
		else if( enT3 & ftu2Ccr)
			pswCcr <= ftu[4:0];	
		else if( enT3 &	~noCcrEn & (finish | init))
			pswCcr <= ccrMasked;
	end
	assign ccr = { 3'b0, pswCcr};	

	      
endmodule

// add bcd correction factor
// It would be more efficient to merge add/sub with main ALU !!!
module aluCorf( input [7:0] binResult, input bAdd, input cin, input hCarry,
	output [7:0] bcdResult, output dC, output logic ov);

	reg [8:0] htemp;
	reg [4:0] hNib;
	
	wire lowC  = hCarry | (bAdd ? gt9( binResult[ 3:0]) : 1'b0);
	wire highC = cin	| (bAdd ? (gt9( htemp[7:4]) | htemp[8]) : 1'b0);
		
	always_comb begin
		if( bAdd) begin
			htemp = { 1'b0, binResult} + (lowC  ? 4'h6 : 4'h0);
		end
		else begin
			htemp = { 1'b0, binResult} - (lowC  ? 4'h6 : 4'h0);
		end
	end
		
	always_comb begin
		if( bAdd) begin
			hNib  = htemp[8:4] + (highC ? 4'h6 : 4'h0);
			ov = hNib[3] & ~binResult[7];
		end
		else begin
			hNib  = htemp[8:4] - (highC ? 4'h6 : 4'h0);
			ov = ~hNib[3] & binResult[7];
		end
	end

	assign bcdResult = { hNib[ 3:0], htemp[3:0]};
	assign dC = hNib[4] | cin;

	// Nibble > 9
	function gt9 (input [3:0] nib);
	begin
		gt9 = nib[3] & (nib[2] | nib[1]);
	end
	endfunction

endmodule


module aluShifter( input [31:0] data,
	input isByte, input isLong, swapWords,
	input dir, input cin,
	output logic [31:0] result);
	// output reg cout

	logic [31:0] tdata;         

	// size mux, put cin in position if dir == right
	always_comb begin
		tdata = data;
		if( isByte & dir)
			tdata[8] = cin;
		else if( !isLong & dir)
			tdata[16] = cin;
	end
    
	always_comb begin
		// Reverse alu/alue position for MUL & DIV
		// Result reversed again
		if( swapWords & dir)
			result = { tdata[0], tdata[31:17], cin, tdata[15:1]};
		else if( swapWords)
			result = { tdata[30:16], cin, tdata[14:0], tdata[31]};
			
		else if( dir)
			result = { cin, tdata[31:1]};
		else
			result = { tdata[30:0], cin};
	end

endmodule


// Get current OP from row & col
module aluGetOp( input [15:0] row, input [2:0] col, input isCorf,
	output logic [4:0] aluOp);
	
	always_comb begin
		aluOp = 'X;
		unique case( col)  
		1:   aluOp = OP_AND;
		5:   aluOp = OP_EXT;

		default:
			unique case( 1'b1)
				row[1]:
					unique case( col)
					2: aluOp = OP_SUB;
					3: aluOp = OP_SUBC;
					4,6: aluOp = OP_SLAA;
					endcase
				
				row[2]:
					unique case( col)
					2: aluOp = OP_ADD;
					3: aluOp = OP_ADDC;
					4: aluOp = OP_ASR;
					endcase

				row[3]:
					unique case( col)
					2: aluOp = OP_ADDX;
					3: aluOp = isCorf ? OP_ABCD : OP_ADD;
					4: aluOp = OP_ASL;
					endcase
                  
				row[4]:
					aluOp = ( col == 4) ? OP_LSL : OP_AND;
				
				row[5],
				row[6]:
					unique case( col)
					2: aluOp = OP_SUB;
					3: aluOp = OP_SUBC;
					4: aluOp = OP_LSR;
					endcase
				
				row[7]:					// MUL
					unique case( col)
					2: aluOp = OP_SUB;
					3: aluOp = OP_ADD;
					4: aluOp = OP_ROXR;
					endcase
               
				row[8]:
					// OP_AND For EXT.L
					// But would be more efficient to change ucode and use column 1 instead of col3 at ublock extr1!				
					unique case( col)
					2: aluOp = OP_EXT;
					3: aluOp = OP_AND;
					4: aluOp = OP_ROXR;
					endcase
               
				row[9]:
					unique case( col)
					2: aluOp = OP_SUBX;
					3: aluOp = OP_SBCD;
					4: aluOp = OP_ROL;
					endcase

				row[10]:
					unique case( col)
					2: aluOp = OP_SUBX;
					3: aluOp = OP_SUBC;
					4: aluOp = OP_ROR;
					endcase
                
				row[11]:
					unique case( col)
					2: aluOp = OP_SUB0;
					3: aluOp = OP_SUB0;
					4: aluOp = OP_ROXL;
					endcase
                
				row[12]:	aluOp = OP_ADDX;               
				row[13]:	aluOp = OP_EOR;
				row[14]:	aluOp = (col == 4) ? OP_EOR : OP_OR;
				row[15]:	aluOp = (col == 3) ? OP_ADD : OP_OR;		// OP_ADD used by DBcc
				
			endcase         
		endcase
	end
endmodule

// Decodes IRD into ALU row (1-15)
// Slow, but no need to optimize for speed since IRD is latched at least two CPU cycles before it is used
// We also register the result after combining with column from nanocode
//
// Many opcodes are not decoded because they either don't do any ALU op,
// or use only columns 1 and 5 that are the same for all rows.

module rowDecoder( input [15:0] ird,
	output logic [15:0] row, output noCcrEn, output logic isArX);


	// Addr or data register direct
	wire eaRdir = (ird[ 5:4] == 2'b00);
	// Addr register direct
	wire eaAdir = (ird[ 5:3] == 3'b001);
	wire size11 = ird[7] & ird[6];
	
	always_comb begin
		case( ird[15:12])
		'h4,
		'h9,
		'hd:
			isArX = row[10] | row[12];
		default:
			isArX = 1'b0;
		endcase
	end

	always_comb begin
		unique case( ird[15:12])

		'h4:  begin
				if( ird[8])
					row = `ALU_ROW_06;			// chk (or lea)
				else case( ird[11:9])
					'b000: row = `ALU_ROW_10;	// negx
					'b001: row = `ALU_ROW_04;	// clr
					'b010: row = `ALU_ROW_05;	// neg
					'b011: row = `ALU_ROW_11;	// not
					'b100: row = (ird[7]) ? `ALU_ROW_08 : `ALU_ROW_09;	// nbcd/swap/ext(or pea)
					'b101: row = `ALU_ROW_15;	// tst & tas
					default: row = 0;
				endcase
			end

		'h0: begin
				if( ird[8])                       // dynamic bit
					row = ird[7] ? `ALU_ROW_14 : `ALU_ROW_13;
				else case( ird[ 11:9])
					'b000:   row = `ALU_ROW_14;	// ori
					'b001:   row = `ALU_ROW_04;	// andi
					'b010:   row = `ALU_ROW_05;	// subi
					'b011:   row = `ALU_ROW_02;	// addi
					'b100:   row = ird[7] ? `ALU_ROW_14 : `ALU_ROW_13;   // static bit
					'b101:   row = `ALU_ROW_13;	// eori
					'b110:   row = `ALU_ROW_06;	// cmpi
					default: row = 0;
					endcase
				end
				
		// MOVE
		// move.b originally also rows 5 & 15. Only because IRD bit 14 is not decoded.
		// It's the same for move the operations performed by MOVE.B

		'h1,'h2,'h3:   row = `ALU_ROW_02;
		
		'h5:
			if( size11)										
               row = `ALU_ROW_15;								// As originally and easier to decode
			else
				row = ird[8] ? `ALU_ROW_05 : `ALU_ROW_02;     // addq/subq
		'h6:	row = 0;						//bcc/bra/bsr
		'h7:	row = `ALU_ROW_02;				// moveq
		'h8:
			if( size11)						// div
				row = `ALU_ROW_01;
			else if( ird[8] & eaRdir)		// sbcd
				row = `ALU_ROW_09;
			else
				row = `ALU_ROW_14;			// or
		'h9:
			if( ird[8] & ~size11 & eaRdir)
				row = `ALU_ROW_10;			// subx
			else
				row = `ALU_ROW_05;			// sub/suba
		'hb:
			if( ird[8] & ~size11 & ~eaAdir)
				row = `ALU_ROW_13;			// eor
			else
				row = `ALU_ROW_06;			// cmp/cmpa/cmpm
		'hc:
			if( size11)
				row = `ALU_ROW_07;			// mul
			else if( ird[8] & eaRdir)		// abcd
				row = `ALU_ROW_03;
			else
				row = `ALU_ROW_04;			// and
		'hd:
			if( ird[8] & ~size11 & eaRdir)
				row = `ALU_ROW_12;			// addx
			else
				row = `ALU_ROW_02;			// add/adda
		'he:
			begin
				reg [1:0] stype;
				
				if( size11)					// memory shift/rotate
					stype = ird[ 10:9];
				else						// register shift/rotate
					stype = ird[ 4:3];

				case( {stype, ird[8]})
				0: row = `ALU_ROW_02;	// ASR
				1: row = `ALU_ROW_03;	// ASL
				2: row = `ALU_ROW_05;	// LSR
				3: row = `ALU_ROW_04;	// LSL
				4: row = `ALU_ROW_08;	// ROXR
				5: row = `ALU_ROW_11;	// ROXL
				6: row = `ALU_ROW_10;	// ROR
				7: row = `ALU_ROW_09;	// ROL
				endcase
			end
			
		default:	row = 0;
		endcase
	end
   
	// Decode opcodes that don't affect flags
	// ADDA/SUBA ADDQ/SUBQ MOVEA

	assign noCcrEn =
		// ADDA/SUBA
		( ird[15] & ~ird[13] & ird[12] & size11) |
		// ADDQ/SUBQ to An
		( (ird[15:12] == 4'h5) & eaAdir) |
		// MOVEA
		( (~ird[15] & ~ird[14] & ird[13]) & ird[8:6] == 3'b001);
        
endmodule

// Row/col CCR update table
module ccrTable( 
	input [2:0] col, input [15:0] row, input finish,
	output logic [MASK_NBITS-1:0] ccrMask);
    
	localparam
		KNZ00 = 5'b01111,   // ok coz operators clear them
		KKZKK = 5'b00100,
		KNZKK = 5'b01100,
		KNZ10 = 5'b01111,	// Used by OP_EXT on divison overflow
		KNZ0C = 5'b01111,	// Used by DIV. V should be 0, but it is ok:
							// DIVU: ends with quotient - 0, so V & C always clear.
							// DIVS: ends with 1i (AND), again, V & C always clear.

		KNZVC	= 5'b01111,
		CUPDALL = 5'b11111,
		CUNUSED = 5'bxxxxx;
    

	logic [MASK_NBITS-1:0] ccrMask1;

	always_comb begin
		unique case( col)
		1:			ccrMask = ccrMask1;
		
		2,3:
			unique case( 1'b1)
			row[1]:		ccrMask = KNZ0C;		// DIV, used as 3n in col3
			row[2],
			row[3],								// ABCD
			row[5],
			row[9],								// SBCD/NBCD
			row[10],							// SUBX/NEGX
			row[12]:	ccrMask = CUPDALL;		// ADDX
			row[6],								// CMP
			row[7],								// MUL		
			row[11]:	ccrMask = KNZVC;		// NOT
			row[4],
			row[8],								// Not used in col 3
			row[13],
			row[14]:	ccrMask = KNZ00;
			row[15]:	ccrMask = 5'b0;			// TAS/Scc, not used in col 3
			// default:	ccrMask = CUNUSED;
			endcase			
			
		4:
			unique case( row)
			// 1: DIV, only n (4n & 6n)
			// 14: BCLR 4n
			// 6,12,13,15	// not used
			`ALU_ROW_02,
			`ALU_ROW_03,							// ASL    (originally ANZVA)
			`ALU_ROW_04,
			`ALU_ROW_05:	ccrMask = CUPDALL;		// Shifts (originally ANZ0A)
			
			`ALU_ROW_07:	ccrMask = KNZ00;		// MUL (originally KNZ0A)
			`ALU_ROW_09,
			`ALU_ROW_10:	ccrMask = KNZ00;		// RO[lr] (originally KNZ0A)
			`ALU_ROW_11:	ccrMask = CUPDALL;		// ROXL (originally ANZ0A)
			default:	ccrMask = CUNUSED;
			endcase
		
		5:			ccrMask = row[1] ? KNZ10 : 5'b0;
		default:	ccrMask = CUNUSED;
		endcase
	end

	// Column 1 (AND)      
	always_comb begin
		if( finish)
			ccrMask1 = row[7] ? KNZ00 : KNZKK;
		else
			ccrMask1 = row[13] | row[14] ? KKZKK : KNZ00;
	end
    
endmodule
//
// FX68K
//
// Opcode to uaddr entry routines (A2-A2-A3) PLA
//

`timescale 1 ns / 1 ns

`define NMA_BITS   10

`define SFTM1 'h3C7
`define SRRW1 'h382
`define SRIW1 'h381
`define SRRL1 'h386
`define SRIL1 'h385
`define BSRI1 'h089
`define BSRW1 'h0A9
`define BBCI1 'h308
`define BBCW1 'h068
`define RLQL1 'h23B
`define ADRW1 'h006
`define PINW1 'h21C
`define PDCW1 'h103
`define ADSW1 'h1C2
`define AIXW0 'h1E3
`define ABWW1 'h00A
`define ABLW1 'h1E2
`define TRAP1 'h1D0
`define LINK1 'h30B
`define UNLK1 'h119
`define LUSP1 'h2F5
`define SUSP1 'h230
`define TRPV1 'h06D
`define RSET1 'h3A6
`define B 'h363
`define STOP1 'h3A2
`define RTR1 'h12A
`define RTS1 'h126


module pla_lined(
   input [3:0] movEa,
   input [3:0] col,

   input [15:0] opcode,
   input [15:0] lineBmap,
   
   output palIll,
   output logic [`NMA_BITS-1:0] plaA1,
   output logic [`NMA_BITS-1:0] plaA2,
   output logic [`NMA_BITS-1:0] plaA3
   );

   wire [3:0] line = opcode[ 15:12];
   wire [2:0] row86 = opcode[8:6];
   
   logic [15:0] arIll;
   logic [`NMA_BITS-1:0] arA1[ 15:0];
   logic [`NMA_BITS-1:0] arA23[ 15:0];
   logic [`NMA_BITS-1:0] scA3;

   logic illMisc;
   logic [`NMA_BITS-1:0] a1Misc;
   
   /*
   reg [`NMA_BITS-1:0] lineMask[ 15:0];  
   always_comb begin
       integer i;
       for( i = 0; i < 16; i = i + 1)
         lineMask[i] = { 16{lineBmap[ i]}};
   end
   */

   assign palIll = (| (arIll & lineBmap));
   
   // Only line 0 has 3 subs
   assign plaA1 = arA1[ line];
   assign plaA2 = arA23[ line];
   assign plaA3 = lineBmap[0] ? scA3 : arA23[ line];
   
   
   // Simple lines
   always_comb begin
      // Line 6: Branch
      arIll[ 'h6] = 1'b0;
      arA23[ 'h6] = 'X;
      if( opcode[ 11:8] == 4'h1)
         arA1[ 'h6] = (| opcode[7:0]) ? `BSRI1 : `BSRW1;
      else
         arA1[ 'h6] = (| opcode[7:0]) ? `BBCI1 : `BBCW1;
                         
      // Line 7: moveq
      arIll[ 'h7] = opcode[ 8];
      arA23[ 'h7] = 'X;
      arA1[ 'h7] = `RLQL1;

      // Line A & F      
      arIll[ 'ha] = 1'b1;       arIll[ 'hf] = 1'b1;
      arA1[ 'ha]  = 'X;         arA1[ 'hf]  = 'X;
      arA23[ 'ha] = 'X;         arA23[ 'hf] = 'X;
      
   end   

   // Special lines

   // Line e: shifts
   always_comb begin
      if( ~opcode[11] & opcode[7] & opcode[6])
      begin
         arA23[ 'he] = `SFTM1;
         unique case( col)
            2: begin arIll[ 'he] = 1'b0; arA1[ 'he] = `ADRW1; end
            3: begin arIll[ 'he] = 1'b0; arA1[ 'he] = `PINW1; end
            4: begin arIll[ 'he] = 1'b0; arA1[ 'he] = `PDCW1; end
            5: begin arIll[ 'he] = 1'b0; arA1[ 'he] = `ADSW1; end
            6: begin arIll[ 'he] = 1'b0; arA1[ 'he] = `AIXW0; end
            7: begin arIll[ 'he] = 1'b0; arA1[ 'he] = `ABWW1; end
            8: begin arIll[ 'he] = 1'b0; arA1[ 'he] = `ABLW1; end
            default: begin arIll[ 'he] = 1'b1; arA1[ 'he] = 'X; end
         endcase
      end
      else
      begin
         arA23[ 'he] = 'X;
         unique case( opcode[ 7:6])
         2'b00,
         2'b01: begin
                  arIll[ 'he] = 1'b0;
                  arA1[ 'he]  = opcode[ 5] ? `SRRW1 : `SRIW1;
               end
         2'b10: begin
                  arIll[ 'he] = 1'b0;
                  arA1[ 'he]  = opcode[ 5] ? `SRRL1 : `SRIL1;
              end
         2'b11: begin arIll[ 'he] = 1'b1; arA1[ 'he]  = 'X; end
         endcase
      end
   end

   // Misc. line 4 row
   always_comb begin
      illMisc = 1'b0;
      case( opcode[ 5:3])
      3'b000,
      3'b001:      a1Misc = `TRAP1;
      3'b010:      a1Misc = `LINK1;
      3'b011:      a1Misc = `UNLK1;
      3'b100:      a1Misc = `LUSP1;
      3'b101:      a1Misc = `SUSP1;
      
      3'b110:   
         case( opcode[ 2:0])
         3'b110:   a1Misc = `TRPV1;
         3'b000:   a1Misc = `RSET1;
         3'b001:   a1Misc = `B;
         3'b010:   a1Misc = `STOP1;
         3'b011:   a1Misc = `RTR1;
         3'b111:   a1Misc = `RTR1;
         3'b101:   a1Misc = `RTS1;
         default:  begin  illMisc = 1'b1; a1Misc = 'X; end
         endcase
         
      default:  begin  illMisc = 1'b1; a1Misc = 'X; end
      endcase
   end

//
// Past here
//


//
// Line: 0
//
always_comb begin

if( (opcode[11:6] & 'h1F) == 'h8) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h100; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h299; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h299; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h299; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h299; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h299; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h299; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h299; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1CC; scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h37) == 'h0) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h100; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h299; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h299; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h299; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h299; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h299; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h299; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h299; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1CC; scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h1F) == 'h9) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h100; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h299; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h299; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h299; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h299; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h299; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h299; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h299; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1CC; scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h37) == 'h1) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h100; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h299; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h299; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h299; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h299; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h299; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h299; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h299; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1CC; scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h1F) == 'hA) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h10C; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00B; scA3 = 'h29D; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00F; scA3 = 'h29D; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h179; scA3 = 'h29D; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1C6; scA3 = 'h29D; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E7; scA3 = 'h29D; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00E; scA3 = 'h29D; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E6; scA3 = 'h29D; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h37) == 'h2) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h10C; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00B; scA3 = 'h29D; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00F; scA3 = 'h29D; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h179; scA3 = 'h29D; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1C6; scA3 = 'h29D; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E7; scA3 = 'h29D; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00E; scA3 = 'h29D; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E6; scA3 = 'h29D; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h37) == 'h10) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h100; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h299; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h299; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h299; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h299; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h299; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h299; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h299; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h37) == 'h11) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h100; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h299; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h299; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h299; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h299; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h299; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h299; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h299; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h37) == 'h12) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h10C; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00B; scA3 = 'h29D; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00F; scA3 = 'h29D; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h179; scA3 = 'h29D; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1C6; scA3 = 'h29D; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E7; scA3 = 'h29D; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00E; scA3 = 'h29D; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E6; scA3 = 'h29D; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h7) == 'h4) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E7; arA23[ 'h0] = 'X;  scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1D2; arA23[ 'h0] = 'X;  scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h006; arA23[ 'h0] = 'X;  scA3 = 'h215; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h21C; arA23[ 'h0] = 'X;  scA3 = 'h215; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h103; arA23[ 'h0] = 'X;  scA3 = 'h215; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1C2; arA23[ 'h0] = 'X;  scA3 = 'h215; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E3; arA23[ 'h0] = 'X;  scA3 = 'h215; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h00A; arA23[ 'h0] = 'X;  scA3 = 'h215; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E2; arA23[ 'h0] = 'X;  scA3 = 'h215; end
     9: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1C2; arA23[ 'h0] = 'X;  scA3 = 'h215; end
    10: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E3; arA23[ 'h0] = 'X;  scA3 = 'h215; end
    11: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h0EA; arA23[ 'h0] = 'h0AB; scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h7) == 'h5) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3EF; arA23[ 'h0] = 'X;  scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1D6; arA23[ 'h0] = 'X;  scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h006; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h21C; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h103; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1C2; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E3; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h00A; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E2; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h7) == 'h7) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3EF; arA23[ 'h0] = 'X;  scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1CE; arA23[ 'h0] = 'X;  scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h006; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h21C; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h103; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1C2; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E3; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h00A; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E2; arA23[ 'h0] = 'X;  scA3 = 'h081; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h7) == 'h6) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3EB; arA23[ 'h0] = 'X;  scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1CA; arA23[ 'h0] = 'X;  scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h006; arA23[ 'h0] = 'X;  scA3 = 'h069; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h21C; arA23[ 'h0] = 'X;  scA3 = 'h069; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h103; arA23[ 'h0] = 'X;  scA3 = 'h069; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1C2; arA23[ 'h0] = 'X;  scA3 = 'h069; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E3; arA23[ 'h0] = 'X;  scA3 = 'h069; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h00A; arA23[ 'h0] = 'X;  scA3 = 'h069; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h1E2; arA23[ 'h0] = 'X;  scA3 = 'h069; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( opcode[11:6] == 'h20) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h3E7; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h215; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h215; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h215; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h215; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h215; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h215; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h215; end
     9: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h215; end
    10: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h215; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( opcode[11:6] == 'h21) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h3EF; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h081; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h081; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h081; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h081; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h081; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h081; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h081; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( opcode[11:6] == 'h23) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h3EF; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h081; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h081; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h081; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h081; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h081; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h081; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h081; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( opcode[11:6] == 'h22) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h3EB; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h069; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h069; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h069; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h069; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h069; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h069; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h069; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( opcode[11:6] == 'h30) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h108; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h087; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h087; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h087; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h087; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h087; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h087; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h087; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( opcode[11:6] == 'h31) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h108; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h006; scA3 = 'h087; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h21C; scA3 = 'h087; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h103; scA3 = 'h087; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1C2; scA3 = 'h087; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E3; scA3 = 'h087; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h00A; scA3 = 'h087; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h2B9; arA23[ 'h0] = 'h1E2; scA3 = 'h087; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else if( opcode[11:6] == 'h32) begin
    unique case ( col)
     0: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h104; scA3 = 'X; end
     1: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
     2: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00B; scA3 = 'h08F; end
     3: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00F; scA3 = 'h08F; end
     4: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h179; scA3 = 'h08F; end
     5: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1C6; scA3 = 'h08F; end
     6: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E7; scA3 = 'h08F; end
     7: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h00E; scA3 = 'h08F; end
     8: begin arIll[ 'h0] = 1'b0; arA1[ 'h0] = 'h3E0; arA23[ 'h0] = 'h1E6; scA3 = 'h08F; end
     9: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    10: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    11: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    default: begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X;    scA3 = 'X; end
    endcase
end

else begin arIll[ 'h0] = 1'b1; arA1[ 'h0] = 'X   ; arA23[ 'h0] = 'X; scA3 = 'X; end

end


//
// Line: 4
//
always_comb begin

if( (opcode[11:6] & 'h27) == 'h0) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h133; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h2B8; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h2B8; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h2B8; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h2B8; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h2B8; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h2B8; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h2B8; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h27) == 'h1) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h133; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h2B8; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h2B8; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h2B8; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h2B8; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h2B8; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h2B8; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h2B8; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h27) == 'h2) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h137; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00B; arA23[ 'h4] = 'h2BC; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00F; arA23[ 'h4] = 'h2BC; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h179; arA23[ 'h4] = 'h2BC; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C6; arA23[ 'h4] = 'h2BC; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E7; arA23[ 'h4] = 'h2BC; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00E; arA23[ 'h4] = 'h2BC; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E6; arA23[ 'h4] = 'h2BC; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h3) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h3A5; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h3A1; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h3A1; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h3A1; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h3A1; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h3A1; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h3A1; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h3A1; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h13) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h301; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h159; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h159; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h159; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h159; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h159; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h159; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h159; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h159; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h159; end
    11: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h0EA; arA23[ 'h4] = 'h301; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h1B) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h301; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h159; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h159; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h159; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h159; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h159; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h159; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h159; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h159; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h159; end
    11: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h0EA; arA23[ 'h4] = 'h301; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h20) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h13B; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h15C; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h15C; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h15C; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h15C; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h15C; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h15C; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h15C; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h21) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h341; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h17C; arA23[ 'h4] = 'X; end
     3: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     4: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h17D; arA23[ 'h4] = 'X; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1FF; arA23[ 'h4] = 'X; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h178; arA23[ 'h4] = 'X; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1FA; arA23[ 'h4] = 'X; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h17D; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1FF; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h22) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h133; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h3A0; arA23[ 'h4] = 'X; end
     3: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h3A4; arA23[ 'h4] = 'X; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F1; arA23[ 'h4] = 'X; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h325; arA23[ 'h4] = 'X; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1ED; arA23[ 'h4] = 'X; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E5; arA23[ 'h4] = 'X; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h23) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h232; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h3A0; arA23[ 'h4] = 'X; end
     3: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h3A4; arA23[ 'h4] = 'X; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F1; arA23[ 'h4] = 'X; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h325; arA23[ 'h4] = 'X; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1ED; arA23[ 'h4] = 'X; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E5; arA23[ 'h4] = 'X; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h28) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h12D; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h3C3; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h3C3; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h3C3; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h3C3; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h3C3; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h3C3; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h3C3; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h29) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h12D; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h3C3; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h3C3; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h3C3; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h3C3; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h3C3; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h3C3; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h3C3; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h2A) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h125; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00B; arA23[ 'h4] = 'h3CB; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00F; arA23[ 'h4] = 'h3CB; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h179; arA23[ 'h4] = 'h3CB; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C6; arA23[ 'h4] = 'h3CB; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E7; arA23[ 'h4] = 'h3CB; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00E; arA23[ 'h4] = 'h3CB; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E6; arA23[ 'h4] = 'h3CB; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h2B) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h345; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h343; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h343; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h343; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h343; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h343; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h343; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h343; end
     9: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h3E) == 'h32) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h127; arA23[ 'h4] = 'X; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h123; arA23[ 'h4] = 'X; end
     4: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1FD; arA23[ 'h4] = 'X; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F5; arA23[ 'h4] = 'X; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F9; arA23[ 'h4] = 'X; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E9; arA23[ 'h4] = 'X; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1FD; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F5; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h7) == 'h6) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h152; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h006; arA23[ 'h4] = 'h151; end
     3: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h21C; arA23[ 'h4] = 'h151; end
     4: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h103; arA23[ 'h4] = 'h151; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h151; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h151; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h00A; arA23[ 'h4] = 'h151; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E2; arA23[ 'h4] = 'h151; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1C2; arA23[ 'h4] = 'h151; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1E3; arA23[ 'h4] = 'h151; end
    11: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h0EA; arA23[ 'h4] = 'h152; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( (opcode[11:6] & 'h7) == 'h7) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h2F1; arA23[ 'h4] = 'X; end
     3: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     4: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h2F2; arA23[ 'h4] = 'X; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1FB; arA23[ 'h4] = 'X; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h275; arA23[ 'h4] = 'X; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h3E4; arA23[ 'h4] = 'X; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h2F2; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1FB; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h3A) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h273; arA23[ 'h4] = 'X; end
     3: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     4: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h2B0; arA23[ 'h4] = 'X; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F3; arA23[ 'h4] = 'X; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h293; arA23[ 'h4] = 'X; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F2; arA23[ 'h4] = 'X; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h2B0; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F3; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h3B) begin
    unique case ( col)
     0: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     1: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     2: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h255; arA23[ 'h4] = 'X; end
     3: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     4: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
     5: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h2B4; arA23[ 'h4] = 'X; end
     6: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F7; arA23[ 'h4] = 'X; end
     7: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h297; arA23[ 'h4] = 'X; end
     8: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F6; arA23[ 'h4] = 'X; end
     9: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h2B4; arA23[ 'h4] = 'X; end
    10: begin arIll[ 'h4] = 1'b0; arA1[ 'h4] = 'h1F7; arA23[ 'h4] = 'X; end
    11: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    default: begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end
    endcase
end

else if( opcode[11:6] == 'h39) begin
     arIll[ 'h4] = illMisc; arA1[ 'h4] = a1Misc   ; arA23[ 'h4] = 'X;
end

else begin arIll[ 'h4] = 1'b1; arA1[ 'h4] = 'X   ; arA23[ 'h4] = 'X; end

end

always_comb begin

//
// Line: 1
//
unique case( movEa)

0: // Row: 0
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h121; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h29B; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h29B; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h29B; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h29B; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h29B; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h29B; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h29B; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h29B; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h29B; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h121; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase

2: // Row: 2
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h2FA; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h3AB; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h3AB; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h3AB; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h3AB; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h3AB; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h3AB; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h3AB; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h3AB; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h3AB; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h2FA; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase

3: // Row: 3
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h2FE; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h3AF; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h3AF; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h3AF; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h3AF; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h3AF; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h3AF; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h3AF; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h3AF; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h3AF; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h2FE; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase

4: // Row: 4
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h2F8; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h38B; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h38B; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h38B; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h38B; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h38B; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h38B; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h38B; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h38B; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h38B; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h2F8; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase

5: // Row: 5
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h2DA; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h38A; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h38A; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h38A; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h38A; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h38A; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h38A; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h38A; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h38A; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h38A; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h2DA; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase

6: // Row: 6
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1EB; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h298; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h298; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h298; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h298; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h298; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h298; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h298; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h298; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h298; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h1EB; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase

7: // Row: 7
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h2D9; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h388; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h388; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h388; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h388; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h388; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h388; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h388; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h388; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h388; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h2D9; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase

8: // Row: 8
    unique case ( col)
     0: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1EA; arA23[ 'h1] = 'X; end
     1: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
     2: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h006; arA23[ 'h1] = 'h32B; end
     3: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h21C; arA23[ 'h1] = 'h32B; end
     4: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h103; arA23[ 'h1] = 'h32B; end
     5: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h32B; end
     6: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h32B; end
     7: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h00A; arA23[ 'h1] = 'h32B; end
     8: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E2; arA23[ 'h1] = 'h32B; end
     9: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1C2; arA23[ 'h1] = 'h32B; end
    10: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h1E3; arA23[ 'h1] = 'h32B; end
    11: begin arIll[ 'h1] = 1'b0; arA1[ 'h1] = 'h0EA; arA23[ 'h1] = 'h1EA; end
    default: begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
    endcase
default:  begin arIll[ 'h1] = 1'b1; arA1[ 'h1] = 'X   ; arA23[ 'h1] = 'X; end
endcase

//
// Line: 2
//
unique case( movEa)

0: // Row: 0
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h129; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h129; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h29F; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h29F; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h29F; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h29F; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h29F; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h29F; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h29F; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h29F; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h29F; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h129; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

1: // Row: 1
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h129; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h129; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h29F; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h29F; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h29F; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h29F; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h29F; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h29F; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h29F; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h29F; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h29F; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h129; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

2: // Row: 2
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2F9; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2F9; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h3A9; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h3A9; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h3A9; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h3A9; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h3A9; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h3A9; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h3A9; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h3A9; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h3A9; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h2F9; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

3: // Row: 3
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2FD; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2FD; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h3AD; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h3AD; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h3AD; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h3AD; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h3AD; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h3AD; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h3AD; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h3AD; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h3AD; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h2FD; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

4: // Row: 4
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2FC; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2FC; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h38F; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h38F; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h38F; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h38F; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h38F; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h38F; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h38F; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h38F; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h38F; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h2FC; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

5: // Row: 5
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2DE; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2DE; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h38E; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h38E; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h38E; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h38E; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h38E; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h38E; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h38E; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h38E; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h38E; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h2DE; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

6: // Row: 6
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1EF; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1EF; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h29C; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h29C; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h29C; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h29C; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h29C; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h29C; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h29C; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h29C; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h29C; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h1EF; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

7: // Row: 7
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2DD; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h2DD; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h38C; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h38C; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h38C; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h38C; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h38C; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h38C; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h38C; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h38C; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h38C; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h2DD; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase

8: // Row: 8
    unique case ( col)
     0: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1EE; arA23[ 'h2] = 'X; end
     1: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1EE; arA23[ 'h2] = 'X; end
     2: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00B; arA23[ 'h2] = 'h30F; end
     3: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00F; arA23[ 'h2] = 'h30F; end
     4: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h179; arA23[ 'h2] = 'h30F; end
     5: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h30F; end
     6: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h30F; end
     7: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h00E; arA23[ 'h2] = 'h30F; end
     8: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E6; arA23[ 'h2] = 'h30F; end
     9: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1C6; arA23[ 'h2] = 'h30F; end
    10: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h1E7; arA23[ 'h2] = 'h30F; end
    11: begin arIll[ 'h2] = 1'b0; arA1[ 'h2] = 'h0A7; arA23[ 'h2] = 'h1EE; end
    default: begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
    endcase
default:  begin arIll[ 'h2] = 1'b1; arA1[ 'h2] = 'X   ; arA23[ 'h2] = 'X; end
endcase

//
// Line: 3
//
unique case( movEa)

0: // Row: 0
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h121; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h121; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h29B; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h29B; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h29B; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h29B; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h29B; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h29B; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h29B; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h29B; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h29B; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h121; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

1: // Row: 1
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h279; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h279; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h158; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h158; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h158; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h158; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h158; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h158; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h158; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h158; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h158; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h279; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

2: // Row: 2
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2FA; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2FA; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h3AB; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h3AB; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h3AB; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h3AB; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h3AB; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h3AB; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h3AB; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h3AB; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h3AB; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h2FA; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

3: // Row: 3
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2FE; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2FE; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h3AF; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h3AF; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h3AF; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h3AF; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h3AF; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h3AF; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h3AF; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h3AF; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h3AF; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h2FE; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

4: // Row: 4
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2F8; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2F8; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h38B; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h38B; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h38B; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h38B; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h38B; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h38B; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h38B; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h38B; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h38B; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h2F8; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

5: // Row: 5
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2DA; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2DA; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h38A; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h38A; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h38A; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h38A; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h38A; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h38A; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h38A; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h38A; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h38A; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h2DA; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

6: // Row: 6
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1EB; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1EB; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h298; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h298; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h298; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h298; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h298; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h298; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h298; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h298; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h298; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h1EB; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

7: // Row: 7
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2D9; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h2D9; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h388; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h388; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h388; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h388; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h388; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h388; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h388; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h388; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h388; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h2D9; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase

8: // Row: 8
    unique case ( col)
     0: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1EA; arA23[ 'h3] = 'X; end
     1: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1EA; arA23[ 'h3] = 'X; end
     2: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h006; arA23[ 'h3] = 'h32B; end
     3: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h21C; arA23[ 'h3] = 'h32B; end
     4: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h103; arA23[ 'h3] = 'h32B; end
     5: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h32B; end
     6: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h32B; end
     7: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h00A; arA23[ 'h3] = 'h32B; end
     8: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E2; arA23[ 'h3] = 'h32B; end
     9: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1C2; arA23[ 'h3] = 'h32B; end
    10: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h1E3; arA23[ 'h3] = 'h32B; end
    11: begin arIll[ 'h3] = 1'b0; arA1[ 'h3] = 'h0EA; arA23[ 'h3] = 'h1EA; end
    default: begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
    endcase
default:  begin arIll[ 'h3] = 1'b1; arA1[ 'h3] = 'X   ; arA23[ 'h3] = 'X; end
endcase

//
// Line: 5
//
unique case( row86)

3'b000: // Row: 0
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2D8; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h006; arA23[ 'h5] = 'h2F3; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h21C; arA23[ 'h5] = 'h2F3; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h103; arA23[ 'h5] = 'h2F3; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C2; arA23[ 'h5] = 'h2F3; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E3; arA23[ 'h5] = 'h2F3; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00A; arA23[ 'h5] = 'h2F3; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E2; arA23[ 'h5] = 'h2F3; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase

3'b001: // Row: 1
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2D8; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2DC; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h006; arA23[ 'h5] = 'h2F3; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h21C; arA23[ 'h5] = 'h2F3; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h103; arA23[ 'h5] = 'h2F3; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C2; arA23[ 'h5] = 'h2F3; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E3; arA23[ 'h5] = 'h2F3; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00A; arA23[ 'h5] = 'h2F3; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E2; arA23[ 'h5] = 'h2F3; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase

3'b010: // Row: 2
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2DC; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2DC; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00B; arA23[ 'h5] = 'h2F7; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00F; arA23[ 'h5] = 'h2F7; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h179; arA23[ 'h5] = 'h2F7; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C6; arA23[ 'h5] = 'h2F7; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E7; arA23[ 'h5] = 'h2F7; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00E; arA23[ 'h5] = 'h2F7; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E6; arA23[ 'h5] = 'h2F7; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase

3'b011: // Row: 3
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h384; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h06C; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h006; arA23[ 'h5] = 'h380; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h21C; arA23[ 'h5] = 'h380; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h103; arA23[ 'h5] = 'h380; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C2; arA23[ 'h5] = 'h380; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E3; arA23[ 'h5] = 'h380; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00A; arA23[ 'h5] = 'h380; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E2; arA23[ 'h5] = 'h380; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase

3'b100: // Row: 4
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2D8; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h006; arA23[ 'h5] = 'h2F3; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h21C; arA23[ 'h5] = 'h2F3; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h103; arA23[ 'h5] = 'h2F3; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C2; arA23[ 'h5] = 'h2F3; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E3; arA23[ 'h5] = 'h2F3; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00A; arA23[ 'h5] = 'h2F3; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E2; arA23[ 'h5] = 'h2F3; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase

3'b101: // Row: 5
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2D8; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2DC; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h006; arA23[ 'h5] = 'h2F3; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h21C; arA23[ 'h5] = 'h2F3; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h103; arA23[ 'h5] = 'h2F3; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C2; arA23[ 'h5] = 'h2F3; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E3; arA23[ 'h5] = 'h2F3; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00A; arA23[ 'h5] = 'h2F3; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E2; arA23[ 'h5] = 'h2F3; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase

3'b110: // Row: 6
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2DC; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h2DC; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00B; arA23[ 'h5] = 'h2F7; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00F; arA23[ 'h5] = 'h2F7; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h179; arA23[ 'h5] = 'h2F7; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C6; arA23[ 'h5] = 'h2F7; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E7; arA23[ 'h5] = 'h2F7; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00E; arA23[ 'h5] = 'h2F7; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E6; arA23[ 'h5] = 'h2F7; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase

3'b111: // Row: 7
    unique case ( col)
     0: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h384; arA23[ 'h5] = 'X; end
     1: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h06C; arA23[ 'h5] = 'X; end
     2: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h006; arA23[ 'h5] = 'h380; end
     3: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h21C; arA23[ 'h5] = 'h380; end
     4: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h103; arA23[ 'h5] = 'h380; end
     5: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1C2; arA23[ 'h5] = 'h380; end
     6: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E3; arA23[ 'h5] = 'h380; end
     7: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h00A; arA23[ 'h5] = 'h380; end
     8: begin arIll[ 'h5] = 1'b0; arA1[ 'h5] = 'h1E2; arA23[ 'h5] = 'h380; end
    default: begin arIll[ 'h5] = 1'b1; arA1[ 'h5] = 'X   ; arA23[ 'h5] = 'X; end
    endcase
endcase

//
// Line: 8
//
unique case( row86)

3'b000: // Row: 0
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C1; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h006; arA23[ 'h8] = 'h1C3; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h21C; arA23[ 'h8] = 'h1C3; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h103; arA23[ 'h8] = 'h1C3; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h1C3; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h1C3; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00A; arA23[ 'h8] = 'h1C3; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E2; arA23[ 'h8] = 'h1C3; end
     9: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h1C3; end
    10: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h1C3; end
    11: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h0EA; arA23[ 'h8] = 'h1C1; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase

3'b001: // Row: 1
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C1; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h006; arA23[ 'h8] = 'h1C3; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h21C; arA23[ 'h8] = 'h1C3; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h103; arA23[ 'h8] = 'h1C3; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h1C3; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h1C3; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00A; arA23[ 'h8] = 'h1C3; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E2; arA23[ 'h8] = 'h1C3; end
     9: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h1C3; end
    10: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h1C3; end
    11: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h0EA; arA23[ 'h8] = 'h1C1; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase

3'b010: // Row: 2
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C5; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00B; arA23[ 'h8] = 'h1CB; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00F; arA23[ 'h8] = 'h1CB; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h179; arA23[ 'h8] = 'h1CB; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C6; arA23[ 'h8] = 'h1CB; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E7; arA23[ 'h8] = 'h1CB; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00E; arA23[ 'h8] = 'h1CB; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E6; arA23[ 'h8] = 'h1CB; end
     9: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C6; arA23[ 'h8] = 'h1CB; end
    10: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E7; arA23[ 'h8] = 'h1CB; end
    11: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h0A7; arA23[ 'h8] = 'h1C5; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase

3'b011: // Row: 3
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h0A6; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h006; arA23[ 'h8] = 'h0A4; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h21C; arA23[ 'h8] = 'h0A4; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h103; arA23[ 'h8] = 'h0A4; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h0A4; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h0A4; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00A; arA23[ 'h8] = 'h0A4; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E2; arA23[ 'h8] = 'h0A4; end
     9: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h0A4; end
    10: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h0A4; end
    11: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h0EA; arA23[ 'h8] = 'h0A6; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase

3'b100: // Row: 4
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1CD; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h107; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h006; arA23[ 'h8] = 'h299; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h21C; arA23[ 'h8] = 'h299; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h103; arA23[ 'h8] = 'h299; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h299; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h299; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00A; arA23[ 'h8] = 'h299; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E2; arA23[ 'h8] = 'h299; end
     9: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    10: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    11: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase

3'b101: // Row: 5
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h006; arA23[ 'h8] = 'h299; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h21C; arA23[ 'h8] = 'h299; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h103; arA23[ 'h8] = 'h299; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h299; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h299; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00A; arA23[ 'h8] = 'h299; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E2; arA23[ 'h8] = 'h299; end
     9: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    10: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    11: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase

3'b110: // Row: 6
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00B; arA23[ 'h8] = 'h29D; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00F; arA23[ 'h8] = 'h29D; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h179; arA23[ 'h8] = 'h29D; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C6; arA23[ 'h8] = 'h29D; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E7; arA23[ 'h8] = 'h29D; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00E; arA23[ 'h8] = 'h29D; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E6; arA23[ 'h8] = 'h29D; end
     9: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    10: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    11: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase

3'b111: // Row: 7
    unique case ( col)
     0: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h0AE; arA23[ 'h8] = 'X; end
     1: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
     2: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h006; arA23[ 'h8] = 'h0AC; end
     3: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h21C; arA23[ 'h8] = 'h0AC; end
     4: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h103; arA23[ 'h8] = 'h0AC; end
     5: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h0AC; end
     6: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h0AC; end
     7: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h00A; arA23[ 'h8] = 'h0AC; end
     8: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E2; arA23[ 'h8] = 'h0AC; end
     9: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1C2; arA23[ 'h8] = 'h0AC; end
    10: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h1E3; arA23[ 'h8] = 'h0AC; end
    11: begin arIll[ 'h8] = 1'b0; arA1[ 'h8] = 'h0EA; arA23[ 'h8] = 'h0AE; end
    default: begin arIll[ 'h8] = 1'b1; arA1[ 'h8] = 'X   ; arA23[ 'h8] = 'X; end
    endcase
endcase

//
// Line: 9
//
unique case( row86)

3'b000: // Row: 0
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C1; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h006; arA23[ 'h9] = 'h1C3; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h21C; arA23[ 'h9] = 'h1C3; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h103; arA23[ 'h9] = 'h1C3; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h1C3; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h1C3; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00A; arA23[ 'h9] = 'h1C3; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E2; arA23[ 'h9] = 'h1C3; end
     9: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h1C3; end
    10: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h1C3; end
    11: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h0EA; arA23[ 'h9] = 'h1C1; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase

3'b001: // Row: 1
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C1; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C1; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h006; arA23[ 'h9] = 'h1C3; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h21C; arA23[ 'h9] = 'h1C3; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h103; arA23[ 'h9] = 'h1C3; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h1C3; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h1C3; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00A; arA23[ 'h9] = 'h1C3; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E2; arA23[ 'h9] = 'h1C3; end
     9: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h1C3; end
    10: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h1C3; end
    11: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h0EA; arA23[ 'h9] = 'h1C1; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase

3'b010: // Row: 2
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C5; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C5; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00B; arA23[ 'h9] = 'h1CB; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00F; arA23[ 'h9] = 'h1CB; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h179; arA23[ 'h9] = 'h1CB; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C6; arA23[ 'h9] = 'h1CB; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E7; arA23[ 'h9] = 'h1CB; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00E; arA23[ 'h9] = 'h1CB; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E6; arA23[ 'h9] = 'h1CB; end
     9: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C6; arA23[ 'h9] = 'h1CB; end
    10: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E7; arA23[ 'h9] = 'h1CB; end
    11: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h0A7; arA23[ 'h9] = 'h1C5; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase

3'b011: // Row: 3
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C9; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C9; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h006; arA23[ 'h9] = 'h1C7; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h21C; arA23[ 'h9] = 'h1C7; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h103; arA23[ 'h9] = 'h1C7; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h1C7; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h1C7; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00A; arA23[ 'h9] = 'h1C7; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E2; arA23[ 'h9] = 'h1C7; end
     9: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h1C7; end
    10: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h1C7; end
    11: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h0EA; arA23[ 'h9] = 'h1C9; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase

3'b100: // Row: 4
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C1; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h10F; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h006; arA23[ 'h9] = 'h299; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h21C; arA23[ 'h9] = 'h299; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h103; arA23[ 'h9] = 'h299; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h299; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h299; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00A; arA23[ 'h9] = 'h299; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E2; arA23[ 'h9] = 'h299; end
     9: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    10: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    11: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase

3'b101: // Row: 5
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C1; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h10F; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h006; arA23[ 'h9] = 'h299; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h21C; arA23[ 'h9] = 'h299; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h103; arA23[ 'h9] = 'h299; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C2; arA23[ 'h9] = 'h299; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E3; arA23[ 'h9] = 'h299; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00A; arA23[ 'h9] = 'h299; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E2; arA23[ 'h9] = 'h299; end
     9: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    10: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    11: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase

3'b110: // Row: 6
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C5; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h10B; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00B; arA23[ 'h9] = 'h29D; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00F; arA23[ 'h9] = 'h29D; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h179; arA23[ 'h9] = 'h29D; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C6; arA23[ 'h9] = 'h29D; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E7; arA23[ 'h9] = 'h29D; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00E; arA23[ 'h9] = 'h29D; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E6; arA23[ 'h9] = 'h29D; end
     9: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    10: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    11: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase

3'b111: // Row: 7
    unique case ( col)
     0: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C5; arA23[ 'h9] = 'X; end
     1: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C5; arA23[ 'h9] = 'X; end
     2: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00B; arA23[ 'h9] = 'h1CB; end
     3: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00F; arA23[ 'h9] = 'h1CB; end
     4: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h179; arA23[ 'h9] = 'h1CB; end
     5: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C6; arA23[ 'h9] = 'h1CB; end
     6: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E7; arA23[ 'h9] = 'h1CB; end
     7: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h00E; arA23[ 'h9] = 'h1CB; end
     8: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E6; arA23[ 'h9] = 'h1CB; end
     9: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1C6; arA23[ 'h9] = 'h1CB; end
    10: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h1E7; arA23[ 'h9] = 'h1CB; end
    11: begin arIll[ 'h9] = 1'b0; arA1[ 'h9] = 'h0A7; arA23[ 'h9] = 'h1C5; end
    default: begin arIll[ 'h9] = 1'b1; arA1[ 'h9] = 'X   ; arA23[ 'h9] = 'X; end
    endcase
endcase

//
// Line: B
//
unique case( row86)

3'b000: // Row: 0
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D1; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h006; arA23[ 'hb] = 'h1D3; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h21C; arA23[ 'hb] = 'h1D3; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h103; arA23[ 'hb] = 'h1D3; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h1D3; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h1D3; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00A; arA23[ 'hb] = 'h1D3; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E2; arA23[ 'hb] = 'h1D3; end
     9: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h1D3; end
    10: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h1D3; end
    11: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h0EA; arA23[ 'hb] = 'h1D1; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase

3'b001: // Row: 1
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D1; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D1; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h006; arA23[ 'hb] = 'h1D3; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h21C; arA23[ 'hb] = 'h1D3; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h103; arA23[ 'hb] = 'h1D3; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h1D3; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h1D3; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00A; arA23[ 'hb] = 'h1D3; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E2; arA23[ 'hb] = 'h1D3; end
     9: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h1D3; end
    10: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h1D3; end
    11: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h0EA; arA23[ 'hb] = 'h1D1; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase

3'b010: // Row: 2
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D5; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D5; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00B; arA23[ 'hb] = 'h1D7; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00F; arA23[ 'hb] = 'h1D7; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h179; arA23[ 'hb] = 'h1D7; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C6; arA23[ 'hb] = 'h1D7; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E7; arA23[ 'hb] = 'h1D7; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00E; arA23[ 'hb] = 'h1D7; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E6; arA23[ 'hb] = 'h1D7; end
     9: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C6; arA23[ 'hb] = 'h1D7; end
    10: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E7; arA23[ 'hb] = 'h1D7; end
    11: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h0A7; arA23[ 'hb] = 'h1D5; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase

3'b011: // Row: 3
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D9; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D9; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h006; arA23[ 'hb] = 'h1CF; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h21C; arA23[ 'hb] = 'h1CF; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h103; arA23[ 'hb] = 'h1CF; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h1CF; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h1CF; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00A; arA23[ 'hb] = 'h1CF; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E2; arA23[ 'hb] = 'h1CF; end
     9: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h1CF; end
    10: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h1CF; end
    11: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h0EA; arA23[ 'hb] = 'h1D9; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase

3'b100: // Row: 4
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h100; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h06B; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h006; arA23[ 'hb] = 'h299; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h21C; arA23[ 'hb] = 'h299; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h103; arA23[ 'hb] = 'h299; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h299; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h299; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00A; arA23[ 'hb] = 'h299; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E2; arA23[ 'hb] = 'h299; end
     9: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    10: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    11: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase

3'b101: // Row: 5
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h100; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h06B; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h006; arA23[ 'hb] = 'h299; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h21C; arA23[ 'hb] = 'h299; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h103; arA23[ 'hb] = 'h299; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C2; arA23[ 'hb] = 'h299; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E3; arA23[ 'hb] = 'h299; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00A; arA23[ 'hb] = 'h299; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E2; arA23[ 'hb] = 'h299; end
     9: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    10: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    11: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase

3'b110: // Row: 6
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h10C; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h06F; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00B; arA23[ 'hb] = 'h29D; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00F; arA23[ 'hb] = 'h29D; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h179; arA23[ 'hb] = 'h29D; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C6; arA23[ 'hb] = 'h29D; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E7; arA23[ 'hb] = 'h29D; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00E; arA23[ 'hb] = 'h29D; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E6; arA23[ 'hb] = 'h29D; end
     9: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    10: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    11: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase

3'b111: // Row: 7
    unique case ( col)
     0: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D5; arA23[ 'hb] = 'X; end
     1: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1D5; arA23[ 'hb] = 'X; end
     2: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00B; arA23[ 'hb] = 'h1D7; end
     3: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00F; arA23[ 'hb] = 'h1D7; end
     4: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h179; arA23[ 'hb] = 'h1D7; end
     5: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C6; arA23[ 'hb] = 'h1D7; end
     6: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E7; arA23[ 'hb] = 'h1D7; end
     7: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h00E; arA23[ 'hb] = 'h1D7; end
     8: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E6; arA23[ 'hb] = 'h1D7; end
     9: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1C6; arA23[ 'hb] = 'h1D7; end
    10: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h1E7; arA23[ 'hb] = 'h1D7; end
    11: begin arIll[ 'hb] = 1'b0; arA1[ 'hb] = 'h0A7; arA23[ 'hb] = 'h1D5; end
    default: begin arIll[ 'hb] = 1'b1; arA1[ 'hb] = 'X   ; arA23[ 'hb] = 'X; end
    endcase
endcase

//
// Line: C
//
unique case( row86)

3'b000: // Row: 0
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C1; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h006; arA23[ 'hc] = 'h1C3; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h21C; arA23[ 'hc] = 'h1C3; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h103; arA23[ 'hc] = 'h1C3; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h1C3; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h1C3; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00A; arA23[ 'hc] = 'h1C3; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E2; arA23[ 'hc] = 'h1C3; end
     9: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h1C3; end
    10: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h1C3; end
    11: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h0EA; arA23[ 'hc] = 'h1C1; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase

3'b001: // Row: 1
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C1; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h006; arA23[ 'hc] = 'h1C3; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h21C; arA23[ 'hc] = 'h1C3; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h103; arA23[ 'hc] = 'h1C3; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h1C3; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h1C3; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00A; arA23[ 'hc] = 'h1C3; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E2; arA23[ 'hc] = 'h1C3; end
     9: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h1C3; end
    10: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h1C3; end
    11: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h0EA; arA23[ 'hc] = 'h1C1; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase

3'b010: // Row: 2
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C5; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00B; arA23[ 'hc] = 'h1CB; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00F; arA23[ 'hc] = 'h1CB; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h179; arA23[ 'hc] = 'h1CB; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C6; arA23[ 'hc] = 'h1CB; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E7; arA23[ 'hc] = 'h1CB; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00E; arA23[ 'hc] = 'h1CB; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E6; arA23[ 'hc] = 'h1CB; end
     9: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C6; arA23[ 'hc] = 'h1CB; end
    10: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E7; arA23[ 'hc] = 'h1CB; end
    11: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h0A7; arA23[ 'hc] = 'h1C5; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase

3'b011: // Row: 3
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h15B; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h006; arA23[ 'hc] = 'h15A; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h21C; arA23[ 'hc] = 'h15A; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h103; arA23[ 'hc] = 'h15A; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h15A; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h15A; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00A; arA23[ 'hc] = 'h15A; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E2; arA23[ 'hc] = 'h15A; end
     9: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h15A; end
    10: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h15A; end
    11: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h0EA; arA23[ 'hc] = 'h15B; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase

3'b100: // Row: 4
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1CD; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h107; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h006; arA23[ 'hc] = 'h299; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h21C; arA23[ 'hc] = 'h299; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h103; arA23[ 'hc] = 'h299; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h299; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h299; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00A; arA23[ 'hc] = 'h299; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E2; arA23[ 'hc] = 'h299; end
     9: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    10: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    11: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase

3'b101: // Row: 5
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h3E3; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h3E3; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h006; arA23[ 'hc] = 'h299; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h21C; arA23[ 'hc] = 'h299; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h103; arA23[ 'hc] = 'h299; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h299; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h299; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00A; arA23[ 'hc] = 'h299; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E2; arA23[ 'hc] = 'h299; end
     9: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    10: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    11: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase

3'b110: // Row: 6
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h3E3; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00B; arA23[ 'hc] = 'h29D; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00F; arA23[ 'hc] = 'h29D; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h179; arA23[ 'hc] = 'h29D; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C6; arA23[ 'hc] = 'h29D; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E7; arA23[ 'hc] = 'h29D; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00E; arA23[ 'hc] = 'h29D; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E6; arA23[ 'hc] = 'h29D; end
     9: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    10: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    11: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase

3'b111: // Row: 7
    unique case ( col)
     0: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h15B; arA23[ 'hc] = 'X; end
     1: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
     2: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h006; arA23[ 'hc] = 'h15A; end
     3: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h21C; arA23[ 'hc] = 'h15A; end
     4: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h103; arA23[ 'hc] = 'h15A; end
     5: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h15A; end
     6: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h15A; end
     7: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h00A; arA23[ 'hc] = 'h15A; end
     8: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E2; arA23[ 'hc] = 'h15A; end
     9: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1C2; arA23[ 'hc] = 'h15A; end
    10: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h1E3; arA23[ 'hc] = 'h15A; end
    11: begin arIll[ 'hc] = 1'b0; arA1[ 'hc] = 'h0EA; arA23[ 'hc] = 'h15B; end
    default: begin arIll[ 'hc] = 1'b1; arA1[ 'hc] = 'X   ; arA23[ 'hc] = 'X; end
    endcase
endcase

//
// Line: D
//
unique case( row86)

3'b000: // Row: 0
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C1; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h006; arA23[ 'hd] = 'h1C3; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h21C; arA23[ 'hd] = 'h1C3; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h103; arA23[ 'hd] = 'h1C3; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h1C3; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h1C3; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00A; arA23[ 'hd] = 'h1C3; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E2; arA23[ 'hd] = 'h1C3; end
     9: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h1C3; end
    10: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h1C3; end
    11: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h0EA; arA23[ 'hd] = 'h1C1; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase

3'b001: // Row: 1
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C1; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C1; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h006; arA23[ 'hd] = 'h1C3; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h21C; arA23[ 'hd] = 'h1C3; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h103; arA23[ 'hd] = 'h1C3; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h1C3; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h1C3; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00A; arA23[ 'hd] = 'h1C3; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E2; arA23[ 'hd] = 'h1C3; end
     9: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h1C3; end
    10: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h1C3; end
    11: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h0EA; arA23[ 'hd] = 'h1C1; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase

3'b010: // Row: 2
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C5; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C5; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00B; arA23[ 'hd] = 'h1CB; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00F; arA23[ 'hd] = 'h1CB; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h179; arA23[ 'hd] = 'h1CB; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C6; arA23[ 'hd] = 'h1CB; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E7; arA23[ 'hd] = 'h1CB; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00E; arA23[ 'hd] = 'h1CB; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E6; arA23[ 'hd] = 'h1CB; end
     9: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C6; arA23[ 'hd] = 'h1CB; end
    10: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E7; arA23[ 'hd] = 'h1CB; end
    11: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h0A7; arA23[ 'hd] = 'h1C5; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase

3'b011: // Row: 3
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C9; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C9; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h006; arA23[ 'hd] = 'h1C7; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h21C; arA23[ 'hd] = 'h1C7; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h103; arA23[ 'hd] = 'h1C7; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h1C7; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h1C7; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00A; arA23[ 'hd] = 'h1C7; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E2; arA23[ 'hd] = 'h1C7; end
     9: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h1C7; end
    10: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h1C7; end
    11: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h0EA; arA23[ 'hd] = 'h1C9; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase

3'b100: // Row: 4
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C1; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h10F; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h006; arA23[ 'hd] = 'h299; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h21C; arA23[ 'hd] = 'h299; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h103; arA23[ 'hd] = 'h299; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h299; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h299; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00A; arA23[ 'hd] = 'h299; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E2; arA23[ 'hd] = 'h299; end
     9: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    10: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    11: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase

3'b101: // Row: 5
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C1; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h10F; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h006; arA23[ 'hd] = 'h299; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h21C; arA23[ 'hd] = 'h299; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h103; arA23[ 'hd] = 'h299; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C2; arA23[ 'hd] = 'h299; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E3; arA23[ 'hd] = 'h299; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00A; arA23[ 'hd] = 'h299; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E2; arA23[ 'hd] = 'h299; end
     9: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    10: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    11: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase

3'b110: // Row: 6
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C5; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h10B; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00B; arA23[ 'hd] = 'h29D; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00F; arA23[ 'hd] = 'h29D; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h179; arA23[ 'hd] = 'h29D; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C6; arA23[ 'hd] = 'h29D; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E7; arA23[ 'hd] = 'h29D; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00E; arA23[ 'hd] = 'h29D; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E6; arA23[ 'hd] = 'h29D; end
     9: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    10: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    11: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase

3'b111: // Row: 7
    unique case ( col)
     0: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C5; arA23[ 'hd] = 'X; end
     1: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C5; arA23[ 'hd] = 'X; end
     2: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00B; arA23[ 'hd] = 'h1CB; end
     3: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00F; arA23[ 'hd] = 'h1CB; end
     4: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h179; arA23[ 'hd] = 'h1CB; end
     5: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C6; arA23[ 'hd] = 'h1CB; end
     6: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E7; arA23[ 'hd] = 'h1CB; end
     7: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h00E; arA23[ 'hd] = 'h1CB; end
     8: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E6; arA23[ 'hd] = 'h1CB; end
     9: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1C6; arA23[ 'hd] = 'h1CB; end
    10: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h1E7; arA23[ 'hd] = 'h1CB; end
    11: begin arIll[ 'hd] = 1'b0; arA1[ 'hd] = 'h0A7; arA23[ 'hd] = 'h1C5; end
    default: begin arIll[ 'hd] = 1'b1; arA1[ 'hd] = 'X   ; arA23[ 'hd] = 'X; end
    endcase
endcase
end


endmodule
