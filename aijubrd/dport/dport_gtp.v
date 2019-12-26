`include "dport.vh"

module dport_gtp(
	input wire clk,
	input wire [1:0] refclkp,
	output wire dpclk,
	output reg gtpready,
	input wire [2:0] prbssel,
	input wire [15:0] tx0data,
	input wire [15:0] tx1data,
	input wire [1:0] tx0isk,
	input wire [1:0] tx1isk,
	output wire [3:0] tx,
	input wire speed,
	input wire [1:0] preemph0,
	input wire [1:0] swing0,
	input wire [1:0] preemph1,
	input wire [1:0] swing1
	
);

	reg pllreset, txreset;
	wire pll0lock, pll0locks;
	wire pll0clk, pll0refclk, pll1clk, pll1refclk;
	wire tx0resetdone, tx1resetdone, tx0resetdones, tx1resetdones;
	wire tx0outclk, tx1outclk;
	wire refclk;
	wire speed0;
	sync speedsync(clk, speed, speed0);
	reg speed1, drpen;
	reg [15:0] drpdi;
	wire [4:0] postcursor0 = preemph0 == 3 ? 26 : preemph0 == 2 ? 20 : preemph0 == 1 ? 14 : 0;
	wire [3:0] diffctrl0 = swing0 == 3 ? 13 : swing0 == 2 ? 9 : swing0 == 1 ? 6 : 3;
	wire [4:0] postcursor1 = preemph1 == 3 ? 26 : preemph1 == 2 ? 20 : preemph1 == 1 ? 14 : 0;
	wire [3:0] diffctrl1 = swing1 == 3 ? 13 : swing1 == 2 ? 9 : swing1 == 1 ? 6 : 3;
	
	always @(posedge clk) begin
		speed1 <= speed0;
		drpen <= speed1 != speed0;
		drpdi <= {10'b0010000000, speed0 ? 6'd3 : 6'd1};
	end

	gtpchan tx0(clk, txreset, tx0resetdone, prbssel, tx0data, tx0isk, tx0outclk, dpclk, pll0clk, pll1clk, pll0refclk, pll1refclk, tx[0], tx[1], postcursor0, diffctrl0);
	gtpchan tx1(clk, txreset, tx1resetdone, prbssel, tx1data, tx1isk, tx1outclk, dpclk, pll0clk, pll1clk, pll0refclk, pll1refclk, tx[2], tx[3], postcursor1, diffctrl1);
	BUFG bufg(.I(tx0outclk), .O(dpclk));
	IBUFDS_GTE2 refclkbuf(.CEB(0), .I(refclkp[0]), .IB(refclkp[1]), .O(refclk));

	GTPE2_COMMON #
	(
	// Simulation attributes
	.SIM_RESET_SPEEDUP   ("false"),
	.SIM_PLL0REFCLK_SEL  (3'b001),
	.SIM_PLL1REFCLK_SEL  (3'b001),
	.SIM_VERSION         ( "2.0"),

	.PLL0_FBDIV          (3),
	.PLL0_FBDIV_45       (4),
	.PLL0_REFCLK_DIV     (1),
	.PLL1_FBDIV          (1),
	.PLL1_FBDIV_45       (4),
	.PLL1_REFCLK_DIV     (1),

	//----------------COMMON BLOCK Attributes---------------
	.BIAS_CFG                               (64'h0000000000050001),
	.COMMON_CFG                             (32'h00000000),

	//--------------------------PLL Attributes----------------------------
	.PLL0_CFG                               (27'h01F03DC),
	.PLL0_DMON_CFG                          (1'b0),
	.PLL0_INIT_CFG                          (24'h00001E),
	.PLL0_LOCK_CFG                          (9'h1E8),
	.PLL1_CFG                               (27'h01F03DC),
	.PLL1_DMON_CFG                          (1'b0),
	.PLL1_INIT_CFG                          (24'h00001E),
	.PLL1_LOCK_CFG                          (9'h1E8),
	.PLL_CLKOUT_CFG                         (8'h00),

	//--------------------------Reserved Attributes----------------------------
	.RSVD_ATTR0                             (16'h0000),
	.RSVD_ATTR1                             (16'h0000)

	)
	gtpe2_common_i
	(
	.GTREFCLK1                      (refclk),
	.PLL0OUTCLK                     (pll0clk),
	.PLL0OUTREFCLK                  (pll0refclk),
	.PLL1OUTCLK                     (pll1clk),
	.PLL1OUTREFCLK                  (pll1refclk),
	.PLL0LOCK                       (pll0lock),
	.PLL0LOCKDETCLK                 (clk),
	.PLL0LOCKEN                     (1),
	.PLL0REFCLKSEL                  (3'b001),
	.PLL0RESET                      (pllreset),
	.PLL1LOCKEN                     (1),
	.PLL1REFCLKSEL                  (3'b001),
	.BGRCALOVRDENB                  (1),
	.BGBYPASSB                      (1),
	.BGMONITORENB                   (1),
	.BGPDB                          (1),
	.BGRCALOVRD                     (5'b11111),
	.RCALENB                        (1),
	.DRPCLK                         (clk),
	.DRPADDR                        (4),
	.DRPEN                          (drpen),
	.DRPWE                          (1),
	.DRPDI                          (drpdi)
	);

	reg [2:0] state;
	reg [7:0] ctr;

	sync pll0locks_i(clk, pll0lock, pll0locks);
	sync txresetdone0s_i(clk, tx0resetdone, tx0resetdones);
	sync txresetdone1s_i(clk, tx1resetdone, tx1resetdones);

	initial state = 0;
	initial gtpready = 0;
	always @(posedge clk) begin
		case(state)
		0: begin
			pllreset <= 1;
			txreset <= 1;
			ctr <= 0;
			state <= 1;
		end
		1: begin
			ctr <= ctr + 1;
			if(ctr == 255) begin
				pllreset <= 0;
				state <= 2;
			end
		end
		2:
			if(pll0locks) begin
				state <= 3;
				ctr <= 0;
			end
		3: begin
			ctr <= ctr + 1;
			if(ctr == 255) begin
				txreset <= 0;
				state <= 4;
			end
		end
		4:
			if(tx0resetdones && tx1resetdones) begin
				gtpready <= 1;
				state <= 5;
			end
		endcase
		if(speed0 != speed1)
			state <= 0;
	end

endmodule

module gtpchan(
	input wire clk,
	input wire txreset,
	output wire txresetdone,
	input wire [2:0] prbssel,
	input wire [15:0] txdata,
	input wire [1:0] txisk,
	output wire txoutclk,
	input wire dpclk,
	input wire pll0clk,
	input wire pll1clk,
	input wire pll0refclk,
	input wire pll1refclk,
	output wire txp,
	output wire txn,
	input wire [4:0] postcursor,
	input wire [3:0] diffctrl
);

	GTPE2_CHANNEL #
	(
	//_______________________ Simulation-Only Attributes __________________

	.SIM_RECEIVER_DETECT_PASS   ("TRUE"),
	.SIM_TX_EIDLE_DRIVE_LEVEL   ("X"),
	.SIM_RESET_SPEEDUP          ("FALSE"),
	.SIM_VERSION                ("2.0"),


	//----------------RX Byte and Word Alignment Attributes---------------
	.ALIGN_COMMA_DOUBLE                     ("FALSE"),
	.ALIGN_COMMA_ENABLE                     (10'b1111111111),
	.ALIGN_COMMA_WORD                       (1),
	.ALIGN_MCOMMA_DET                       ("TRUE"),
	.ALIGN_MCOMMA_VALUE                     (10'b1010000011),
	.ALIGN_PCOMMA_DET                       ("TRUE"),
	.ALIGN_PCOMMA_VALUE                     (10'b0101111100),
	.SHOW_REALIGN_COMMA                     ("TRUE"),
	.RXSLIDE_AUTO_WAIT                      (7),
	.RXSLIDE_MODE                           ("OFF"),
	.RX_SIG_VALID_DLY                       (10),

	//----------------RX 8B/10B Decoder Attributes---------------
	.RX_DISPERR_SEQ_MATCH                   ("TRUE"),
	.DEC_MCOMMA_DETECT                      ("FALSE"),
	.DEC_PCOMMA_DETECT                      ("FALSE"),
	.DEC_VALID_COMMA_ONLY                   ("FALSE"),

	//----------------------RX Clock Correction Attributes----------------------
	.CBCC_DATA_SOURCE_SEL                   ("ENCODED"),
	.CLK_COR_SEQ_2_USE                      ("FALSE"),
	.CLK_COR_KEEP_IDLE                      ("FALSE"),
	.CLK_COR_MAX_LAT                        (9),
	.CLK_COR_MIN_LAT                        (7),
	.CLK_COR_PRECEDENCE                     ("TRUE"),
	.CLK_COR_REPEAT_WAIT                    (0),
	.CLK_COR_SEQ_LEN                        (1),
	.CLK_COR_SEQ_1_ENABLE                   (4'b1111),
	.CLK_COR_SEQ_1_1                        (10'b0100000000),
	.CLK_COR_SEQ_1_2                        (10'b0000000000),
	.CLK_COR_SEQ_1_3                        (10'b0000000000),
	.CLK_COR_SEQ_1_4                        (10'b0000000000),
	.CLK_CORRECT_USE                        ("FALSE"),
	.CLK_COR_SEQ_2_ENABLE                   (4'b1111),
	.CLK_COR_SEQ_2_1                        (10'b0100000000),
	.CLK_COR_SEQ_2_2                        (10'b0000000000),
	.CLK_COR_SEQ_2_3                        (10'b0000000000),
	.CLK_COR_SEQ_2_4                        (10'b0000000000),

	//----------------------RX Channel Bonding Attributes----------------------
	.CHAN_BOND_KEEP_ALIGN                   ("FALSE"),
	.CHAN_BOND_MAX_SKEW                     (1),
	.CHAN_BOND_SEQ_LEN                      (1),
	.CHAN_BOND_SEQ_1_1                      (10'b0000000000),
	.CHAN_BOND_SEQ_1_2                      (10'b0000000000),
	.CHAN_BOND_SEQ_1_3                      (10'b0000000000),
	.CHAN_BOND_SEQ_1_4                      (10'b0000000000),
	.CHAN_BOND_SEQ_1_ENABLE                 (4'b1111),
	.CHAN_BOND_SEQ_2_1                      (10'b0000000000),
	.CHAN_BOND_SEQ_2_2                      (10'b0000000000),
	.CHAN_BOND_SEQ_2_3                      (10'b0000000000),
	.CHAN_BOND_SEQ_2_4                      (10'b0000000000),
	.CHAN_BOND_SEQ_2_ENABLE                 (4'b1111),
	.CHAN_BOND_SEQ_2_USE                    ("FALSE"),
	.FTS_DESKEW_SEQ_ENABLE                  (4'b1111),
	.FTS_LANE_DESKEW_CFG                    (4'b1111),
	.FTS_LANE_DESKEW_EN                     ("FALSE"),

	//-------------------------RX Margin Analysis Attributes----------------------------
	.ES_CONTROL                             (6'b000000),
	.ES_ERRDET_EN                           ("FALSE"),
	.ES_EYE_SCAN_EN                         ("FALSE"),
	.ES_HORZ_OFFSET                         (12'h010),
	.ES_PMA_CFG                             (10'b0000000000),
	.ES_PRESCALE                            (5'b00000),
	.ES_QUALIFIER                           (80'h00000000000000000000),
	.ES_QUAL_MASK                           (80'h00000000000000000000),
	.ES_SDATA_MASK                          (80'h00000000000000000000),
	.ES_VERT_OFFSET                         (9'b000000000),

	//-----------------------FPGA RX Interface Attributes-------------------------
	.RX_DATA_WIDTH                          (20),

	//-------------------------PMA Attributes----------------------------
	.OUTREFCLK_SEL_INV                      (2'b11),
	.PMA_RSV                                (32'h00000333),
	.PMA_RSV2                               (32'h00002040),
	.PMA_RSV3                               (2'b00),
	.PMA_RSV4                               (4'b0000),
	.RX_BIAS_CFG                            (16'b0000111100110011),
	.DMONITOR_CFG                           (24'h000A00),
	.RX_CM_SEL                              (2'b01),
	.RX_CM_TRIM                             (4'b0000),
	.RX_DEBUG_CFG                           (14'b00000000000000),
	.RX_OS_CFG                              (13'b0000010000000),
	.TERM_RCAL_CFG                          (15'b100001000010000),
	.TERM_RCAL_OVRD                         (3'b000),
	.TST_RSV                                (32'h00000000),
	.RX_CLK25_DIV                           (6),
	.TX_CLK25_DIV                           (6),
	.UCODEER_CLR                            (1'b0),

	//-------------------------PCI Express Attributes----------------------------
	.PCS_PCIE_EN                            ("FALSE"),

	//-------------------------PCS Attributes----------------------------
	.PCS_RSVD_ATTR                          (48'h000000000000),

	//-----------RX Buffer Attributes------------
	.RXBUF_ADDR_MODE                        ("FAST"),
	.RXBUF_EIDLE_HI_CNT                     (4'b1000),
	.RXBUF_EIDLE_LO_CNT                     (4'b0000),
	.RXBUF_EN                               ("TRUE"),
	.RX_BUFFER_CFG                          (6'b000000),
	.RXBUF_RESET_ON_CB_CHANGE               ("TRUE"),
	.RXBUF_RESET_ON_COMMAALIGN              ("FALSE"),
	.RXBUF_RESET_ON_EIDLE                   ("FALSE"),
	.RXBUF_RESET_ON_RATE_CHANGE             ("TRUE"),
	.RXBUFRESET_TIME                        (5'b00001),
	.RXBUF_THRESH_OVFLW                     (61),
	.RXBUF_THRESH_OVRD                      ("FALSE"),
	.RXBUF_THRESH_UNDFLW                    (4),
	.RXDLY_CFG                              (16'h001F),
	.RXDLY_LCFG                             (9'h030),
	.RXDLY_TAP_CFG                          (16'h0000),
	.RXPH_CFG                               (24'hC00002),
	.RXPHDLY_CFG                            (24'h084020),
	.RXPH_MONITOR_SEL                       (5'b00000),
	.RX_XCLK_SEL                            ("RXREC"),
	.RX_DDI_SEL                             (6'b000000),
	.RX_DEFER_RESET_BUF_EN                  ("TRUE"),

	//---------------------CDR Attributes-------------------------

	//For Display Port, HBR/RBR- set RXCDR_CFG=72'h0380008bff40200008

	//For Display Port, HBR2 -   set RXCDR_CFG=72'h038c008bff20200010
	.RXCDR_CFG                              (83'h0001107FE206021081010),
	.RXCDR_FR_RESET_ON_EIDLE                (1'b0),
	.RXCDR_HOLD_DURING_EIDLE                (1'b0),
	.RXCDR_PH_RESET_ON_EIDLE                (1'b0),
	.RXCDR_LOCK_CFG                         (6'b001001),

	//-----------------RX Initialization and Reset Attributes-------------------
	.RXCDRFREQRESET_TIME                    (5'b00001),
	.RXCDRPHRESET_TIME                      (5'b00001),
	.RXISCANRESET_TIME                      (5'b00001),
	.RXPCSRESET_TIME                        (5'b00001),
	.RXPMARESET_TIME                        (5'b00011),

	//-----------------RX OOB Signaling Attributes-------------------
	.RXOOB_CFG                              (7'b0000110),

	//-----------------------RX Gearbox Attributes---------------------------
	.RXGEARBOX_EN                           ("FALSE"),
	.GEARBOX_MODE                           (3'b000),

	//-----------------------PRBS Detection Attribute-----------------------
	.RXPRBS_ERR_LOOPBACK                    (1'b0),

	//-----------Power-Down Attributes----------
	.PD_TRANS_TIME_FROM_P2                  (12'h03c),
	.PD_TRANS_TIME_NONE_P2                  (8'h3c),
	.PD_TRANS_TIME_TO_P2                    (8'h64),

	//-----------RX OOB Signaling Attributes----------
	.SAS_MAX_COM                            (64),
	.SAS_MIN_COM                            (36),
	.SATA_BURST_SEQ_LEN                     (4'b1111),
	.SATA_BURST_VAL                         (3'b100),
	.SATA_EIDLE_VAL                         (3'b100),
	.SATA_MAX_BURST                         (8),
	.SATA_MAX_INIT                          (21),
	.SATA_MAX_WAKE                          (7),
	.SATA_MIN_BURST                         (4),
	.SATA_MIN_INIT                          (12),
	.SATA_MIN_WAKE                          (4),

	//-----------RX Fabric Clock Output Control Attributes----------
	.TRANS_TIME_RATE                        (8'h0E),

	//------------TX Buffer Attributes----------------
	.TXBUF_EN                               ("TRUE"),
	.TXBUF_RESET_ON_RATE_CHANGE             ("TRUE"),
	.TXDLY_CFG                              (16'h001F),
	.TXDLY_LCFG                             (9'h030),
	.TXDLY_TAP_CFG                          (16'h0000),
	.TXPH_CFG                               (16'h0780),
	.TXPHDLY_CFG                            (24'h084020),
	.TXPH_MONITOR_SEL                       (5'b00000),
	.TX_XCLK_SEL                            ("TXOUT"),

	//-----------------------FPGA TX Interface Attributes-------------------------
	.TX_DATA_WIDTH                          (20),

	//-----------------------TX Configurable Driver Attributes-------------------------
	.TX_DEEMPH0                             (6'b000000),
	.TX_DEEMPH1                             (6'b000000),
	.TX_EIDLE_ASSERT_DELAY                  (3'b110),
	.TX_EIDLE_DEASSERT_DELAY                (3'b100),
	.TX_LOOPBACK_DRIVE_HIZ                  ("FALSE"),
	.TX_MAINCURSOR_SEL                      (1'b0),
	.TX_DRIVE_MODE                          ("DIRECT"),
	.TX_MARGIN_FULL_0                       (7'b1001110),
	.TX_MARGIN_FULL_1                       (7'b1001001),
	.TX_MARGIN_FULL_2                       (7'b1000101),
	.TX_MARGIN_FULL_3                       (7'b1000010),
	.TX_MARGIN_FULL_4                       (7'b1000000),
	.TX_MARGIN_LOW_0                        (7'b1000110),
	.TX_MARGIN_LOW_1                        (7'b1000100),
	.TX_MARGIN_LOW_2                        (7'b1000010),
	.TX_MARGIN_LOW_3                        (7'b1000000),
	.TX_MARGIN_LOW_4                        (7'b1000000),

	//-----------------------TX Gearbox Attributes--------------------------
	.TXGEARBOX_EN                           ("FALSE"),

	//-----------------------TX Initialization and Reset Attributes--------------------------
	.TXPCSRESET_TIME                        (5'b00001),
	.TXPMARESET_TIME                        (5'b00001),

	//-----------------------TX Receiver Detection Attributes--------------------------
	.TX_RXDETECT_CFG                        (14'h1832),
	.TX_RXDETECT_REF                        (3'b100),

	//---------------- JTAG Attributes ---------------
	.ACJTAG_DEBUG_MODE                      (1'b0),
	.ACJTAG_MODE                            (1'b0),
	.ACJTAG_RESET                           (1'b0),

	//---------------- CDR Attributes ---------------
	.CFOK_CFG                               (43'h49000040E80),
	.CFOK_CFG2                              (7'b0100000),
	.CFOK_CFG3                              (7'b0100000),
	.CFOK_CFG4                              (1'b0),
	.CFOK_CFG5                              (2'h0),
	.CFOK_CFG6                              (4'b0000),
	.RXOSCALRESET_TIME                      (5'b00011),
	.RXOSCALRESET_TIMEOUT                   (5'b00000),

	//---------------- PMA Attributes ---------------
	.CLK_COMMON_SWING                       (1'b0),
	.RX_CLKMUX_EN                           (1'b1),
	.TX_CLKMUX_EN                           (1'b1),
	.ES_CLK_PHASE_SEL                       (1'b0),
	.USE_PCS_CLK_PHASE_SEL                  (1'b0),
	.PMA_RSV6                               (1'b0),
	.PMA_RSV7                               (1'b0),

	//---------------- TX Configuration Driver Attributes ---------------
	.TX_PREDRIVER_MODE                      (1'b0),
	.PMA_RSV5                               (1'b0),
	.SATA_PLL_CFG                           ("VCO_3000MHZ"),

	//---------------- RX Fabric Clock Output Control Attributes ---------------
	.RXOUT_DIV                              (2),

	//---------------- TX Fabric Clock Output Control Attributes ---------------
	.TXOUT_DIV                              (2),

	//---------------- RX Phase Interpolator Attributes---------------
	.RXPI_CFG0                              (3'b000),
	.RXPI_CFG1                              (1'b1),
	.RXPI_CFG2                              (1'b1),

	//------------RX Equalizer Attributes-------------
	.ADAPT_CFG0                             (20'h00000),
	.RXLPMRESET_TIME                        (7'b0001111),
	.RXLPM_BIAS_STARTUP_DISABLE             (1'b0),
	.RXLPM_CFG                              (4'b0110),
	.RXLPM_CFG1                             (1'b0),
	.RXLPM_CM_CFG                           (1'b0),
	.RXLPM_GC_CFG                           (9'b111100010),
	.RXLPM_GC_CFG2                          (3'b001),
	.RXLPM_HF_CFG                           (14'b00001111110000),
	.RXLPM_HF_CFG2                          (5'b01010),
	.RXLPM_HF_CFG3                          (4'b0000),
	.RXLPM_HOLD_DURING_EIDLE                (1'b0),
	.RXLPM_INCM_CFG                         (1'b0),
	.RXLPM_IPCM_CFG                         (1'b1),
	.RXLPM_LF_CFG                           (18'b000000001111110000),
	.RXLPM_LF_CFG2                          (5'b01010),
	.RXLPM_OSINT_CFG                        (3'b100),

	//---------------- TX Phase Interpolator PPM Controller Attributes---------------
	.TXPI_CFG0                              (2'b00),
	.TXPI_CFG1                              (2'b00),
	.TXPI_CFG2                              (2'b00),
	.TXPI_CFG3                              (1'b0),
	.TXPI_CFG4                              (1'b0),
	.TXPI_CFG5                              (3'b000),
	.TXPI_GREY_SEL                          (1'b0),
	.TXPI_INVSTROBE_SEL                     (1'b0),
	.TXPI_PPMCLK_SEL                        ("TXUSRCLK2"),
	.TXPI_PPM_CFG                           (8'h00),
	.TXPI_SYNFREQ_PPM                       (3'b000),

	//---------------- LOOPBACK Attributes---------------
	.LOOPBACK_CFG                           (1'b0),
	.PMA_LOOPBACK_CFG                       (1'b0),

	//----------------RX OOB Signalling Attributes---------------
	.RXOOB_CLK_CFG                          ("PMA"),

	//----------------TX OOB Signalling Attributes---------------
	.TXOOB_CFG                              (1'b0),

	//----------------RX Buffer Attributes---------------
	.RXSYNC_MULTILANE                       (1'b1),
	.RXSYNC_OVRD                            (1'b0),
	.RXSYNC_SKIP_DA                         (1'b0),

	//----------------TX Buffer Attributes---------------
	.TXSYNC_MULTILANE                       (0),
	.TXSYNC_OVRD                            (0),
	.TXSYNC_SKIP_DA                         (1'b0)


	) 
	gtpe2_i 
	(

	.TSTIN                          (20'b11111111111111111111),
	.DRPCLK                         (clk),

	.RXSYSCLKSEL                    (2'b11),

	.TX8B10BEN                      (1),

	.PLL0CLK                        (pll0clk),
	.PLL0REFCLK                     (pll0refclk),
	.PLL1CLK                        (pll1clk),
	.PLL1REFCLK                     (pll1refclk),

	.RXPD                           (2'b11),

	.GTRXRESET			(txreset),
	.RXLPMRESET			(txreset),
	.GTTXRESET                      (txreset),
	.TXUSERRDY                      (1),

	.TXDATA                         (txdata),
	.TXUSRCLK                       (dpclk),
	.TXUSRCLK2                      (dpclk),

	.TXCHARISK                      (txisk),
	.TXDLYBYPASS                    (1),

	.GTPTXN                         (txn),
	.GTPTXP                         (txp),
	.TXBUFDIFFCTRL                  (3'b100),
	.TXDIFFCTRL                     (diffctrl),
	.TXOUTCLK                       (txoutclk),
	.TXOUTCLKSEL                    (3'b010),
	.TXPOSTCURSOR                   (postcursor),

	.TXRESETDONE                    (txresetdone),
	.TXPRBSSEL                      (prbssel),
	.TXPOLARITY                     (1),
	.TXPIPPMSEL                     (1)

	);


endmodule
