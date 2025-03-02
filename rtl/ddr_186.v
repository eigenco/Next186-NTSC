`timescale 1ns / 1ps

module system (
	input  clk_NTSC,
	input  clk_25,
	input  clk_sdr,
	input  clk_cpu,
	input  clk_opl,
	input  CLK44100x256,
	input  [1:0] cpu_speed,

	output [3:0]sdr_n_CS_WE_RAS_CAS,
	output [1:0]sdr_BA,
	output [12:0]sdr_ADDR,
	inout [15:0]sdr_DATA,
	output [1:0]sdr_DQM,

	output reg [5:0]VGA_R,
	output reg [5:0]VGA_G,
	output reg [5:0]VGA_B,
	output frame_on,
	output wire VGA_HSYNC,
	output wire VGA_VSYNC,
	input BTN_RESET,
	input BTN_NMI,
	output [7:0]LED,

	output reg SD_n_CS = 1'b1,
	output wire SD_DI,
	output reg SD_CK = 0,
	input SD_DO,
		 
	output reg AUD_L,
	output reg AUD_R,
	input PS2_CLK1_I,
	output PS2_CLK1_O,
	input PS2_CLK2_I,
	output PS2_CLK2_O,
	input PS2_DATA1_I,
	output PS2_DATA1_O,
	input PS2_DATA2_I,
	output PS2_DATA2_O,
	 
	output I2C_SCL,
	inout I2C_SDA,

	input [12:0] BIOS_ADDR,
	input [15:0] BIOS_DIN,
	input BIOS_WR,
	output BIOS_REQ,
	input VSM,
	input HSM,
	input [1:0] HLFA
);

	localparam BIOS_BASE = 20'h57800;
	initial SD_n_CS = 1'b1;

	wire [15:0] cntrl0_user_input_data;
	wire [1:0] sys_cmd_ack;
	wire sys_rd_data_valid;
	wire sys_wr_data_valid;   
	wire [15:0] sys_DOUT;
	wire [31:0] DOUT;
	wire [15:0] CPU_DOUT;
	wire [15:0] PORT_ADDR;
	wire [31:0] DRAM_dout;
	wire [20:0] ADDR;
	wire IORQ;
	wire WR;
	wire INTA;
	wire WORD;
	wire [3:0] RAM_WMASK;
	wire hblnk;
	wire vblnk;
	wire [9:0] hcount;
	wire [9:0] vcount;
	reg [3:0] vga_hrzpan = 0;
	wire [3:0] vga_hrzpan_req;
	wire [9:0] hcount_pan = hcount + vga_hrzpan - 17;
	reg FifoStart = 1'b0;
	wire displ_on = !(hblnk | vblnk | !FifoStart);	
	wire [17:0] DAC_COLOR;
	wire [8:0] fifo_wr_used_words;
	wire AlmostFull;
	wire AlmostEmpty;
	wire CPU_CE;
	wire CE;
	wire CE_186;
	wire ddr_rd; 
	wire ddr_wr;	
	wire TIMER_OE = PORT_ADDR[15:2] == 14'b00000000010000;	//   40h..43h
	wire VGA_DAC_OE = PORT_ADDR[15:4] == 12'h03c && PORT_ADDR[3:0] <= 4'h9; // 3c0h..3c9h	
	wire MEMORY_MAP = PORT_ADDR[15:4] == 12'h008;
	wire VGA_FONT_OE = PORT_ADDR[15:0] == 16'h03cb;
	wire AUX_OE = PORT_ADDR[15:0] == 16'h0001;
	wire INPUT_STATUS_OE = PORT_ADDR[15:0] == 16'h03da;
	wire VGA_CRT_OE = (PORT_ADDR[15:1] == 15'b000000111011010) || (PORT_ADDR[15:1] == 15'b000000111101010); // 3b4h, 3b5h, 3d4h, 3d5h
	wire VGA_SC = PORT_ADDR[15:1] == (16'h03c4 >> 1); // 3c4h, 3c5h
	wire VGA_GC = PORT_ADDR[15:1] == (16'h03ce >> 1); // 3ceh, 3cfh
	wire PIC_OE = PORT_ADDR[15:8] == 8'h00 && PORT_ADDR[6:0] == 7'b0100001;	// 21h, a1h
	wire KB_OE = PORT_ADDR[15:4] == 12'h006 && {PORT_ADDR[3], PORT_ADDR[1:0]} == 3'b000; // 60h, 64h
 	wire [7:0] VGA_DAC_DATA;
	wire [7:0] VGA_CRT_DATA;
	wire [7:0] VGA_SC_DATA;
	wire [7:0] VGA_GC_DATA;
	wire [15:0] PORT_IN;
	wire [7:0] TIMER_DOUT;
	wire [7:0] KB_DOUT;
	wire [7:0] PIC_DOUT;
	wire HALT;
	reg [1:0]cntrl0_user_command_register = 0;
	reg [16:0]vga_ddr_row_col = 0;
	reg s_prog_full;
	reg s_prog_empty;
	reg s_ddr_rd = 1'b0;
	reg s_ddr_wr = 1'b0;
	reg crw = 0;
	reg [18:0] rstcount = 0;
	reg [18:0] s_displ_on = 0;
	reg [2:0] vga13 = 0;
	reg [2:0] vgatext = 0;
	reg [2:0] planar = 0;
	reg [2:0] half = 0;
	reg [0:0] repln_graph = 0;
	wire vgaflash;
	reg flashbit = 0;
	reg [5:0] flashcount = 0;
	wire [5:0] char_row = vcount[8:3] >> !half[2];
	wire [3:0] char_ln = {(vcount[3] & !half[1]), vcount[2:0]};
	wire [11:0] charcount = {char_row, 4'b0000} + {char_row, 6'b000000} + hcount_pan[9:3];
	wire [31:0] fifo_dout32;
	wire [15:0] fifo_dout = (vgatext[1] ? hcount_pan[3] : vga13[1] ? hcount_pan[2] : hcount_pan[1]) ? fifo_dout32[31:16] : fifo_dout32[15:0];
	reg [8:0] vga_ddr_row_count = 0;
	reg [2:0] max_read;
	reg [4:0] col_counter;
	//wire vga_end_frame = vga_ddr_row_count == 399;
	wire vga_end_frame = vga_ddr_row_count == 199;
	reg [3:0] vga_repln_count = 0;
	wire [3:0] vga_repln = vgatext[0] ? (half[0] ? 7 : 15) : {3'b000, repln_graph[0]};
	reg [7:0] vga_lnbytecount = 0;
	wire [4:0] vga_lnend = (vgatext[0] | half[0]) ? 6 : (vga13[0] | planar[0]) ? 11 : 21;
	reg [11:0] vga_font_counter = 0;
	reg [7:0] vga_attr;
	wire [14:0] cache_hi_addr;
	wire [8:0] memmap;
	wire [8:0] memmap_mux;
	wire [7:0] font_dout;
	wire [7:0] VGA_FONT_DATA;
	wire vgatextreq;
	wire vga13req;
	wire planarreq;
	wire replnreq;
	wire halfreq;
	wire oncursor;
	wire [4:0] crs[1:0];
	wire [11:0] cursorpos;
	wire [15:0] scraddr;
	reg flash_on;
	wire [2:0] shift = half[1] ? ~hcount_pan[3:1] : ~hcount_pan[2:0];
	wire [2:0] pxindex = -hcount_pan[2:0];
	wire [3:0] EGA_MUX = vgatext[1] ? (font_dout[pxindex] ^ flash_on) ? vga_attr[3:0] : {vga_attr[7] & ~vgaflash, vga_attr[6:4]} : {fifo_dout32[{2'b11, shift}], fifo_dout32[{2'b10, shift}], fifo_dout32[{2'b01, shift}], fifo_dout32[{2'b00, shift}]};
	wire [7:0] VGA_INDEX;						  
	reg [3:0] exline = 4'b0000;
	wire vrdon = s_displ_on[~vga_hrzpan];
	wire vrden = (vrdon || exline[3]) && ((vgatext[1] | half[1]) ? &hcount_pan[3:0] : (vga13[1] | planar[1]) ? &hcount_pan[2:0] : &hcount_pan[1:0]);
	reg s_vga_endline;
	reg s_vga_endscanline = 1'b0;
	reg s_vga_endframe;
	reg [23:0] sdraddr;
	wire [3:0] vga_wplane;
	wire [1:0] vga_rplane;
	wire [7:0] vga_bitmask;
	wire [2:0] vga_rwmode;
	wire [3:0] vga_setres;
	wire [3:0] vga_enable_setres;
	wire [1:0] vga_logop;
	wire [3:0] vga_color_compare;
	wire [3:0] vga_color_dont_care;
	wire [2:0] vga_rotate_count;
	wire [7:0] vga_offset;
	wire ppm;
	wire [9:0] lcr;
	wire [9:0] vde;
	wire sdon = s_displ_on[17+vgatext[1]] & (vcount <= (vde>>1));

	reg [7:0]SDI;
	assign SD_DI = CPU_DOUT[7];
	
	assign frame_on = s_displ_on[16+vgatext[1]];

	assign PORT_IN[15:8] = 
		({8{MEMORY_MAP}} & {7'b0000000, memmap[8]}) |
		({8{INPUT_STATUS_OE}} & SDI);
	
   assign PORT_IN[7:0] =
							 ({8{VGA_DAC_OE}} & VGA_DAC_DATA) |
							 ({8{VGA_FONT_OE}}& VGA_FONT_DATA) |
							 ({8{KB_OE}} & KB_DOUT) |
							 ({8{INPUT_STATUS_OE}} & {1'b0, 1'b0, 1'b0, 1'b0, VSM ? VGA_VSYNC : vblnk, 1'b0, 1'b0, (HSM ? VGA_HSYNC : hblnk) | (VSM ? VGA_VSYNC : vblnk)}) | 
							 ({8{VGA_CRT_OE}} & VGA_CRT_DATA) | 
							 ({8{MEMORY_MAP}} & {memmap[7:0]}) |
							 ({8{TIMER_OE}} & TIMER_DOUT) |
							 ({8{PIC_OE}} & PIC_DOUT) |
							 ({8{VGA_SC}} & VGA_SC_DATA) |
							 ({8{VGA_GC}} & VGA_GC_DATA) |
                      ({8{OPL3_PORT}} & opl32_data) |
							 ({8{SB_22A}} & (dsp_state==8'he1 ? 8'h01 : 8'hAA)) |
							 ({8{SB_22E}} & 8'hff);

	assign BIOS_REQ = sys_wr_data_valid;
	reg [15:0] BIOS_data;
	reg        BIOS_data_valid;
	reg  [1:0] auto_flush = 2'b00;

	SDRAM_16bit SDR
	(
		.sys_CLK(clk_sdr),                                              // clock
		.sys_CMD(cntrl0_user_command_register),                         // 00=nop, 01 = write 64 bytes, 10=read 32 bytes, 11=read 64 bytes
		.sys_ADDR(sdraddr),                                             // word address
		.sys_DIN(BIOS_data_valid ? BIOS_data : cntrl0_user_input_data), // data input
		.sys_DOUT(sys_DOUT),                                            // data output
		.sys_rd_data_valid(sys_rd_data_valid),                          // data valid read
		.sys_wr_data_valid(sys_wr_data_valid),                          // data valid write
		.sys_cmd_ack(sys_cmd_ack),                                      // command acknowledged
		.sdr_n_CS_WE_RAS_CAS(sdr_n_CS_WE_RAS_CAS),                      // SDRAM #CS, #WE, #RAS, #CAS
		.sdr_BA(sdr_BA),                                                // SDRAM bank address
		.sdr_ADDR(sdr_ADDR),                                            // SDRAM address
		.sdr_DATA(sdr_DATA),                                            // SDRAM data
		.sdr_DQM(sdr_DQM)                                               // SDRAM DQM
	);

	fifo vga_fifo 
	(
	  .wrclk(clk_sdr),
	  .rdclk(clk_NTSC),
	  .data(sys_DOUT),
	  .wrreq(!crw && sys_rd_data_valid && !col_counter[4]),
	  .rdreq(vrden),
	  .q(fifo_dout32),
	  .wrusedw(fifo_wr_used_words)
	);

	VGA_SG VGA 
	(
		// STANDARD MCGA
		//
		//.tc_hsblnk(10'd639),
		//.tc_hssync(10'd655),
		//.tc_hesync(10'd751),
		//.tc_heblnk(10'd799),
		//
		//.tc_vsblnk(10'd399),
		//.tc_vssync(10'd411),
		//.tc_vesync(10'd413),
		//.tc_veblnk(10'd448),
		
		// CGA is 262 scanlines of 228 color carrier cycles per scanline
		// 655/799*14/(315/88*4)*(228*4-1)
		// 751/799*14/(315/88*4)*(228*4-1)
		//         14/(315/88*4)*(228*4-1)
		.tc_hsblnk(10'd639),
		.tc_hssync(10'd714), // 704
		.tc_hesync(10'd817), // 807
		.tc_heblnk(10'd859),

		.hcount(hcount),
		.hsync(VGA_HSYNC),
		.hblnk(hblnk),
		
		.tc_vsblnk(10'd199),
		.tc_vssync(10'd224),
		.tc_vesync(10'd225),
		.tc_veblnk(10'd261),
		
		.vcount(vcount),
		.vsync(VGA_VSYNC),
		.vblnk(vblnk),
		.clk(clk_NTSC),
		.ce(FifoStart)
	);
	
	/*
	400 -> 399 -> 199
	12	 -> 411 -> 206
	2	 -> 413 -> 207
	35	 -> 448 -> 224
	(449)

	480 -> 479 -> 239
	10	 -> 489 -> 244
	2	 -> 491 -> 246
	33	 -> 524 -> 262
	(525)
	*/

	VGA_DAC dac 
	(
		 .CE(VGA_DAC_OE && IORQ && CPU_CE), 
		 .WR(WR), 
		 .addr(PORT_ADDR[3:0]), 
		 .din(CPU_DOUT[7:0]), 
		 .dout(VGA_DAC_DATA), 
		 .CLK(clk_cpu), 
		 .VGA_CLK(clk_NTSC), 
		 .vga_addr((vgatext[1] | (~vga13[1] & planar[1])) ? VGA_INDEX : (vga13[1] ? hcount_pan[1] : hcount_pan[0]) ? fifo_dout[15:8] : fifo_dout[7:0]),
		 .color(DAC_COLOR),
		 .vgatext(vgatextreq),
		 .vga13(vga13req),
		 .half(halfreq),
		 .vgaflash(vgaflash),
		 .setindex(INPUT_STATUS_OE && IORQ && CPU_CE),
		 .hrzpan(vga_hrzpan_req),
		 .ppm(ppm),
		 .ega_attr(EGA_MUX),
		 .ega_pal_index(VGA_INDEX)
    );

	 VGA_CRT crt
	 (
		.CE(IORQ && CPU_CE && VGA_CRT_OE),
		.WR(WR),
		.WORD(WORD),
		.din(CPU_DOUT),
		.addr(PORT_ADDR[0]),
		.dout(VGA_CRT_DATA),
		.CLK(clk_cpu),
		.oncursor(oncursor),
		.cursorstart(crs[0]),
		.cursorend(crs[1]),
		.cursorpos(cursorpos),
		.scraddr(scraddr),
		.offset(vga_offset),
		.lcr(lcr),
		.repln(replnreq),
		.vde(vde)
	);

	VGA_SC sc
	(
		.CE(IORQ && CPU_CE && VGA_SC),
		.WR(WR),
		.WORD(WORD),
		.din(CPU_DOUT),
		.dout(VGA_SC_DATA),
		.addr(PORT_ADDR[0]),
		.CLK(clk_cpu),
		.planarreq(planarreq),
		.wplane(vga_wplane)
    );

	VGA_GC gc
	(
		.CE(IORQ && CPU_CE && VGA_GC),
		.WR(WR),
		.WORD(WORD),
		.din(CPU_DOUT),
		.addr(PORT_ADDR[0]),
		.CLK(clk_cpu),
		.rplane(vga_rplane),
		.bitmask(vga_bitmask),
		.rwmode(vga_rwmode),
		.setres(vga_setres),
		.enable_setres(vga_enable_setres),
		.logop(vga_logop),
		.color_compare(vga_color_compare),
		.color_dont_care(vga_color_dont_care),
		.rotate_count(vga_rotate_count),
		.dout(VGA_GC_DATA)
	);

	sr_font VGA_FONT 
	(
		.clock_a(clk_NTSC),                                // input clka
		.wren_a(1'b0),                                     // input [0:0] wea
		.address_a({fifo_dout[7:0], char_ln}),             // input [11:0] addra
		.data_a(8'h00),                                    // input [7:0] dina
		.q_a(font_dout),                                   // output [7:0] douta
		.clock_b(clk_cpu),                                 // input clkb
		.wren_b(WR & IORQ & VGA_FONT_OE & ~WORD & CPU_CE), // input [0:0] web
		.address_b(vga_font_counter),                      // input [11:0] addrb
		.data_b(CPU_DOUT[7:0]),                            // input [7:0] dinb
		.q_b(VGA_FONT_DATA)                                // output [7:0] doutb
	);

	cache_controller cache_ctl 
	(
		.addr(ADDR),
		.dout(DRAM_dout),
		.din(DOUT),
		.clk(clk_cpu),
		.mreq(MREQ),
		.wmask(RAM_WMASK),
		.ce(CE),
		.cpu_speed(cpu_speed),
		.ddr_din(sys_DOUT),
		.ddr_dout(cntrl0_user_input_data),
		.ddr_clk(clk_sdr),
		.ddr_rd(ddr_rd),
		.ddr_wr(ddr_wr),
		.hiaddr(cache_hi_addr),
		.cache_write_data(crw && sys_rd_data_valid),
		.cache_read_data(crw && sys_wr_data_valid),
		.flush(auto_flush==2'b10)
	);

	wire I_KB;
	wire I_MOUSE;
	wire KB_RST;
	KB_Mouse_8042 KB_Mouse 
	(
		 .CS(IORQ && CPU_CE && KB_OE),
		 .WR(WR), 
		 .cmd(PORT_ADDR[2]),
		 .din(CPU_DOUT[7:0]), 
		 .dout(KB_DOUT), 
		 .clk(clk_cpu), 
		 .I_KB(I_KB), 
		 .I_MOUSE(I_MOUSE), 
		 .CPU_RST(KB_RST), 
		 .PS2_CLK1_I(PS2_CLK1_I),
		 .PS2_CLK1_O(PS2_CLK1_O),
		 .PS2_CLK2_I(PS2_CLK2_I),
		 .PS2_CLK2_O(PS2_CLK2_O),
		 .PS2_DATA1_I(PS2_DATA1_I),
		 .PS2_DATA1_O(PS2_DATA1_O),
		 .PS2_DATA2_I(PS2_DATA2_I),
		 .PS2_DATA2_O(PS2_DATA2_O)
	);

	wire [7:0]PIC_IVECT;
	wire INT;
	wire timer_int;
	PIC_8259 PIC 
	(
		 .RST(!rstcount[18]),
		 .CS(PIC_OE && IORQ && CPU_CE),
		 .WR(WR), 
		 .din(CPU_DOUT[7:0]), 
		 .slave(PORT_ADDR[7]),
		 .dout(PIC_DOUT), 
		 .ivect(PIC_IVECT), 
		 .clk(clk_cpu), 
		 .INT(INT), 
		 .IACK(INTA & CPU_CE), 
		 .I({1'b0, I_MOUSE, 1'b0, I_KB, timer_int})
    );

	wire [3:0]seg_addr;
	wire vga_planar_seg;
	unit186 CPUUnit
	(
		 .INPORT(INTA ? {8'h00, PIC_IVECT} : PORT_IN), 
		 .DIN(DRAM_dout), 
		 .CPU_DOUT(CPU_DOUT),
		 .PORT_ADDR(PORT_ADDR),
		 .SEG_ADDR(seg_addr),
		 .DOUT(DOUT), 
		 .ADDR(ADDR), 
		 .WMASK(RAM_WMASK), 
		 .CLK(clk_cpu), 
		 .CE(CE), 
		 .CPU_CE(CPU_CE),
		 .CE_186(CE_186),
		 .INTR(INT), 
		 .RST(!rstcount[18]), 
		 .INTA(INTA), 
		 .LOCK(LOCK), 
		 .HALT(HALT), 
		 .MREQ(MREQ),
		 .IORQ(IORQ),
		 .WR(WR),
		 .WORD(WORD),
		 .FASTIO(1'b1),
		 
		 .VGA_SEL(planarreq && vga_planar_seg),
		 .VGA_WPLANE(vga_wplane),
		 .VGA_RPLANE(vga_rplane),
		 .VGA_BITMASK(vga_bitmask),
		 .VGA_RWMODE(vga_rwmode),
		 .VGA_SETRES(vga_setres),
		 .VGA_ENABLE_SETRES(vga_enable_setres),
		 .VGA_LOGOP(vga_logop),
		 .VGA_COLOR_COMPARE(vga_color_compare),
		 .VGA_COLOR_DONT_CARE(vga_color_dont_care),
		 .VGA_ROTATE_COUNT(vga_rotate_count)
	);
	
	seg_map seg_mapper 
	(
		 .CLK(clk_cpu),
		 .cpuaddr(PORT_ADDR[3:0]), 
		 .cpurdata(memmap), 
		 .cpuwdata(CPU_DOUT[8:0]), 
		 .memaddr(cache_hi_addr[14:10]), 
		 .memdata(memmap_mux), 
		 .WE(MEMORY_MAP & WR & WORD & IORQ & CPU_CE),
		 .seg_addr(seg_addr),
		 .vga_planar_seg(vga_planar_seg)
    );

	 wire timer_spk;
	 timer_8253 timer
	 (
		 .CS(TIMER_OE && IORQ && CPU_CE), 
		 .WR(WR), 
		 .addr(PORT_ADDR[1:0]), 
		 .din(CPU_DOUT[7:0]), 
		 .dout(TIMER_DOUT), 
		 //.CLK_25(clk_25), 
		 .clk_sdr(clk_sdr),
		 .clk(clk_cpu), 
		 .out0(timer_int),
		 .out2(timer_spk)
    );

    ////////	 	
	 reg  [15:0] r_opl3left = 0;
	 reg  [15:0] r_opl3right = 0;
	 reg  [31:0] lval = 0; 
	 reg  [31:0] rval = 0;
	 reg   [8:0] clkdiv = 0;
	 wire [16:0] lmix = pcm_sample + {r_opl3left[15], r_opl3left};
	 wire [16:0] rmix = pcm_sample + {r_opl3right[15], r_opl3right};
	 wire [15:0] lclamp = (~|lmix[16:15] | &lmix[16:15]) ? {!lmix[15], lmix[14:0]} : {16{!lmix[16]}};
	 wire [15:0] rclamp = (~|rmix[16:15] | &rmix[16:15]) ? {!rmix[15], rmix[14:0]} : {16{!rmix[16]}};
	 wire lsign = lval[31:16] < lclamp;
	 wire rsign = rval[31:16] < rclamp;
	 
    always @(posedge CLK44100x256) begin
		clkdiv[8:0] <= clkdiv[7:0] + 1'b1;
		if(clkdiv[8]) begin
	     r_opl3left <= opl3left;
        r_opl3right <= opl3right;
		end
		
		lval <= lval - lval[31:7] + (lsign << 25);
		AUD_L <= lsign;

		rval <= rval - rval[31:7] + (rsign << 25);
		AUD_R <= rsign;
	 end


    wire SB_22A = (PORT_ADDR[15:0] == 16'h022a);
	 wire SB_22C = (PORT_ADDR[15:0] == 16'h022c);
	 wire SB_22E = (PORT_ADDR[15:0] == 16'h022e);
	 reg [15:0] pcm_sample = 0;
	 reg  [7:0] dsp_state = 0;
	 always @(posedge clk_cpu)
	 begin	
		if(IORQ & CPU_CE & WR & (PORT_ADDR[15:0] == 16'h0378))
			pcm_sample <= {1'b0, CPU_DOUT[7:0], 7'b0000000};
		if(IORQ & CPU_CE & WR & SB_22C)
		begin
		  case(dsp_state)
	       8'h00: dsp_state <= CPU_DOUT[7:0];
			 8'h10: begin
				pcm_sample <= {1'b0, CPU_DOUT[7:0], 7'b0000000};
				dsp_state <= 8'h00;
			 end
			 default: dsp_state <= 8'h00;
			endcase
		end
    end
	 
	 reg HLF;
	 always @(posedge clk_cpu)
		if(IORQ & CPU_CE & WR & (PORT_ADDR[15:0] == 16'h037F))
			HLF <= CPU_DOUT[7:0];    
	
	 wire OPL3_PORT = PORT_ADDR[15:2] == (16'h0388 >> 2); // 0x388 .. 0x38b
    wire [7:0]opl32_data;
    wire [15:0]opl3left;
    wire [15:0]opl3right;
    opl3 opl3_inst (
		.clk(clk_opl),
		.cpu_clk(clk_cpu),
		.addr(PORT_ADDR[1:0]),
		.din(CPU_DOUT[7:0]),
		.dout(opl32_data),
		.ce(IORQ & CPU_CE & OPL3_PORT),
		.wr(WR),
		.left(opl3left),
		.right(opl3right),
		.stb44100(clkdiv[8]),
		.reset(!rstcount[18])
	);

	reg nop;
	reg fifo_fill = 1;
	always @ (posedge clk_sdr) begin
		s_prog_full <= fifo_wr_used_words > 350;
		if(fifo_wr_used_words < 64) s_prog_empty <= 1'b1;
		else begin
			s_prog_empty <= 1'b0;
			FifoStart <= 1'b1;
		end
		s_ddr_rd <= ddr_rd;
		s_ddr_wr <= ddr_wr;
		s_vga_endline <= vga_repln_count == vga_repln;
		s_vga_endframe <= vga_end_frame;
		nop <= sys_cmd_ack == 2'b00;

		sdraddr <= BIOS_WR ? BIOS_BASE + (BIOS_ADDR >> 1) : (s_prog_empty && fifo_fill) || !(s_ddr_wr || s_ddr_rd) ? {6'b000001, vga_ddr_row_col + vga_lnbytecount} : {memmap_mux[8:0], cache_hi_addr[9:0], 4'b0000};
		max_read <= &sdraddr[7:3] ? ~sdraddr[2:0] : 3'b111;

		BIOS_data_valid <= BIOS_WR;
		BIOS_data <= BIOS_DIN;

		if(BIOS_WR) cntrl0_user_command_register <= 2'b01;
		else if(s_prog_empty && fifo_fill) cntrl0_user_command_register <= 2'b10; // read 32 bytes VGA
		else if(s_ddr_wr) cntrl0_user_command_register <= 2'b01;                  // write 256 bytes cache
		else if(s_ddr_rd) cntrl0_user_command_register <= 2'b11;                  // read 256 bytes cache
		else if(~s_prog_full && fifo_fill) cntrl0_user_command_register <= 2'b10; // read 32 bytes VGA
		else cntrl0_user_command_register <= 2'b00;

		if(!crw && sys_rd_data_valid) col_counter <= col_counter - 1'b1;
		if(nop) case(sys_cmd_ack)
			2'b10: begin
				crw <= 1'b0;
				col_counter <= {1'b0, max_read, 1'b1};
				vga_lnbytecount <= vga_lnbytecount + max_read + 1'b1;
			end					
			2'b01, 2'b11: crw <= !BIOS_WR;
		endcase
		if(vcount==258) fifo_fill <= 1;
		//if(vcount==222) fifo_fill <= 1;
		//if(vcount==445) fifo_fill <= 1;
		if(s_vga_endscanline)
		begin
			col_counter[3:1] <= col_counter[3:1] - vga_lnbytecount[2:0];
			vga_lnbytecount <= 0;
			s_vga_endscanline <= 1'b0;
			if(s_vga_endframe) vga_ddr_row_col <= {{1'b0, scraddr[15:13]} + (vgatext[0] ? 4'b0111 : 4'b0100), scraddr[12:0]};
			else if({1'b0, vga_ddr_row_count} == (lcr>>1)) vga_ddr_row_col <= vgatext[0] ? 17'he000 : 17'h8000; 
			else if(s_vga_endline) vga_ddr_row_col <= vga_ddr_row_col + (vgatext[0] ? 40 : {vga_offset, 1'b0});
			if(s_vga_endline) vga_repln_count <= 0;
			else vga_repln_count <= vga_repln_count + 1'b1;
			if(s_vga_endframe) begin
				vga13[0] <= vga13req;
				vgatext[0] <= vgatextreq;
				planar[0] <= planarreq;
				if(HLFA>0)
					half[0] <= HLFA[1];
				else
					half[0] <= HLF;
				//half[0] <= halfreq;
				repln_graph[0] <= replnreq;
				vga_ddr_row_count <= 0;
				fifo_fill <= 0;
			end else vga_ddr_row_count <= vga_ddr_row_count + 1'b1;
		end else s_vga_endscanline <= (vga_lnbytecount[7:3] == vga_lnend);
	end

	always @ (posedge clk_cpu)
	begin
		if(IORQ & CPU_CE) begin
			if(VGA_FONT_OE) vga_font_counter <= WR && WORD ? {CPU_DOUT[7:0], 4'b0000} : vga_font_counter + 1'b1; 
		end
		if(CPU_CE) begin
			SD_CK <= IORQ & INPUT_STATUS_OE & WR & ~WORD;
			if(IORQ & INPUT_STATUS_OE & WR) begin
				if(WORD) SD_n_CS <= ~CPU_DOUT[8];
				else SDI <= {SDI[6:0], SD_DO};
			end
		end
		if(KB_RST || BTN_RESET) rstcount <= 0;
		else if(CPU_CE && ~rstcount[18]) rstcount <= rstcount + 1'b1;
		auto_flush[1:0] <= {auto_flush[0], hblnk};
	end

	always @ (posedge clk_NTSC)
	begin
		s_displ_on <= {s_displ_on[17:0], displ_on};
		exline <= vrdon ? 4'b1111 : (exline - vrden);
		vga_attr <= fifo_dout[15:8];
		flash_on <= (vgaflash & fifo_dout[15] & flashcount[5]) | (~oncursor && flashcount[4] && (charcount == cursorpos) && (char_ln >= crs[0][3:0]) && (char_ln <= crs[1][3:0]));
		if(!vblnk) begin
			flashbit <= 1;
			vga13[2] <= vga13[1];
			vgatext[2] <= vgatext[1];
			planar[2] <= planar[1];
			half[2] <= half[1];
		end else if(flashbit) begin
			flashcount <= flashcount + 1'b1;
			flashbit <= 0;
			vga13[1] <= vga13[0];
			vgatext[1] <= vgatext[0];
			planar[1] <= planar[0];
			half[1] <= half[0];
		end
		if(VGA_VSYNC) vga_hrzpan <= half[0] ? {vga_hrzpan_req[2:0], 1'b0} : {1'b0, vga_hrzpan_req[2:0]};
		else if(VGA_HSYNC && ppm && (vcount == (lcr>>1))) vga_hrzpan <= 4'b0000;
		{VGA_B, VGA_G, VGA_R} <= DAC_COLOR & {18{sdon}};
	end
	
endmodule
