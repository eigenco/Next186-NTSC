module Next186_MiST
(
    input         CLOCK_27,
    output  [5:0] VGA_R,
    output  [5:0] VGA_G,
    output  [5:0] VGA_B,
    output        VGA_HS,
    output        VGA_VS,
    output        LED,
    output        AUDIO_L,
    output        AUDIO_R,
    input         UART_RX,
    output        UART_TX,
    input         SPI_SCK,
    output        SPI_DO,
    input         SPI_DI,
    input         SPI_SS2,
    input         SPI_SS3,
    input         CONF_DATA0,
	 output [12:0] SDRAM_A,
	 output  [1:0] SDRAM_BA,
	 inout  [15:0] SDRAM_DQ,
	 output        SDRAM_DQML,
	 output        SDRAM_DQMH,
	 output        SDRAM_nCAS,
	 output        SDRAM_nRAS,
	 output        SDRAM_nWE,
	 output        SDRAM_CKE,
	 output        SDRAM_nCS,
	 output        SDRAM_CLK
);

parameter CONF_STR = {
	"NEXT186;;",
	"O12,CPU Speed,/1,/2,/3,/4;",
	"O34,Sync,0,1,2,3;",
	"O56,Half,0,1,2,3;",
	"T0,Reset;"
};

wire  [1:0] cpu_speed = status[2:1];
wire        VSM = status[3];
wire        HSM = status[4];
wire  [1:0] HLFA = status[6:5];
wire  [5:0] core_r, core_g, core_b;
wire        core_hs, core_vs;
wire        clk_25, clk_sdr, clk_50, CLK44100x256, CLK14745600;
wire        clk_sys = clk_25;
assign SDRAM_CKE = 1'b1;

pitopl pitopl (
	.inclk0(CLOCK_27),
	.c0(clk_OPL),
	.c1(clk_25)
);

dcm dcm_system (
	.inclk0(CLOCK_27), 
	.c0(), 
	.c1(clk_sdr),
	.c2(SDRAM_CLK),
	.c3(clk_50)
);

dcm_misc dcm_misc (
	.inclk0(CLOCK_27),
	.c0(CLK44100x256)
);

wire clk_cpu; //, clk_dsp;
dcm_cpu dcm_cpu_inst (
	.inclk0(CLOCK_27), 
	.c0(clk_cpu)
	//,
	//.c1(clk_dsp)
);

reg clk_NTSC;
always @(posedge CLOCK_27)
	clk_NTSC <= ~clk_NTSC;

reg  reset;
    
always @(posedge clk_cpu) reset <= status[0] | buttons[1] | !bios_loaded;
	
wire [63:0] status;
wire  [1:0] buttons;
wire  [1:0] switches;

wire        ps2_kbd_clk, ps2_kbd_clk_i;
wire        ps2_kbd_dat, ps2_kbd_dat_i;
wire        ps2_mouse_clk, ps2_mouse_clk_i;
wire        ps2_mouse_dat, ps2_mouse_dat_i;

wire [31:0] sd_lba;
wire        sd_rd;
wire        sd_wr;
wire        sd_ack;
wire        sd_conf;
wire        sd_sdhc; 
wire  [7:0] sd_dout;
wire        sd_dout_strobe;
wire  [7:0] sd_din;
wire        sd_din_strobe;
wire  [8:0] sd_buff_addr;
wire        sd_ack_conf;
wire        img_mounted;
wire [31:0] img_size;

wire        scandoubler_disable;
wire        ypbpr;
wire        no_csync;

user_io #(.STRLEN($size(CONF_STR)>>3), .PS2DIV(2000), .PS2BIDIR(1'b1)) user_io(
	.conf_str        ( CONF_STR      ),
	.clk_sys         ( clk_cpu       ),
	.clk_sd          ( clk_cpu       ),
	.SPI_CLK         ( SPI_SCK       ),
	.SPI_SS_IO       ( CONF_DATA0    ),
	.SPI_MISO        ( SPI_DO        ),
	.SPI_MOSI        ( SPI_DI        ),
	.status          ( status        ),
	.switches        ( switches      ),
	.buttons         ( buttons       ),
	.scandoubler_disable ( scandoubler_disable ),
	.ypbpr           ( ypbpr         ),
	.no_csync        ( no_csync      ),
	.sd_lba          ( sd_lba        ),
	.sd_rd           ( sd_rd         ),
	.sd_wr           ( sd_wr         ),
	.sd_ack          ( sd_ack        ),
	.sd_conf         ( sd_conf       ),
	.sd_sdhc         ( sd_sdhc       ),
	.sd_dout         ( sd_dout       ),
	.sd_dout_strobe  ( sd_dout_strobe),
	.sd_din          ( sd_din        ),
	.sd_din_strobe   ( sd_din_strobe ),
	.sd_buff_addr    ( sd_buff_addr  ),
	.sd_ack_conf     ( sd_ack_conf   ),
	.img_mounted     ( img_mounted   ),
	.img_size        ( img_size      ),
	.ps2_kbd_clk     ( ps2_kbd_clk   ), 
	.ps2_kbd_data    ( ps2_kbd_dat   ),
	.ps2_kbd_clk_i   ( ps2_kbd_clk_i ),
	.ps2_kbd_data_i  ( ps2_kbd_dat_i ),
	.ps2_mouse_clk   ( ps2_mouse_clk ), 
	.ps2_mouse_clk_i ( ps2_mouse_clk_i ), 
	.ps2_mouse_data  ( ps2_mouse_dat ),
	.ps2_mouse_data_i( ps2_mouse_dat_i )
);

wire sd_sck;
wire sd_cs;
wire sd_sdi;
reg  sd_sdi_r;
wire sd_sdo;

always @(posedge clk_cpu) sd_sdi_r <= sd_sdi;

sd_card sd_card (
	.clk_sys      ( clk_cpu        ),
	.sd_lba       ( sd_lba         ),
	.sd_rd        ( sd_rd          ),
	.sd_wr        ( sd_wr          ),
	.sd_ack       ( sd_ack         ),
	.sd_ack_conf  ( sd_ack_conf    ),
	.sd_conf      ( sd_conf        ),
	.sd_sdhc      ( sd_sdhc        ),
	.sd_buff_dout ( sd_dout        ),
	.sd_buff_wr   ( sd_dout_strobe ),
	.sd_buff_din  ( sd_din         ),
	.sd_buff_addr ( sd_buff_addr   ),
	.img_mounted  ( img_mounted    ),
	.img_size     ( img_size       ),
	.allow_sdhc   ( 1'b1           ),
 	.sd_cs        ( sd_cs          ),
	.sd_sck       ( sd_sck         ),
	.sd_sdi       ( sd_sdi_r       ),
	.sd_sdo       ( sd_sdo         )
);

mist_video #(.COLOR_DEPTH(6)) mist_video (
	.clk_sys      ( clk_sys    ),
	.SPI_SCK      ( SPI_SCK    ),
	.SPI_SS3      ( SPI_SS3    ),
	.SPI_DI       ( SPI_DI     ),
	.scanlines    ( 2'b00      ),
	.ce_divider   ( 1'b0       ),
	.scandoubler_disable ( 1'b1 ),
	.no_csync     ( no_csync   ),
	.ypbpr        ( ypbpr      ),
	.rotate       ( 2'b00      ),
	.blend        ( 1'b0       ),
	.R            ( core_r     ),
	.G            ( core_g     ),
	.B            ( core_b     ),
	.HSync        ( core_hs    ),
	.VSync        ( core_vs    ),
	.VGA_R        ( VGA_R      ),
	.VGA_G        ( VGA_G      ),
	.VGA_B        ( VGA_B      ),
	.VGA_VS       ( VGA_VS     ),
	.VGA_HS       ( VGA_HS     )
);

wire        ioctl_downl;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;

data_io data_io(
	.clk_sys       ( clk_sdr      ),
	.SPI_SCK       ( SPI_SCK      ),
	.SPI_SS2       ( SPI_SS2      ),
	.SPI_DI        ( SPI_DI       ),
	.ioctl_download( ioctl_downl  ),
	.ioctl_index   ( ioctl_index  ),
	.ioctl_wr      ( ioctl_wr     ),
	.ioctl_addr    ( ioctl_addr   ),
	.ioctl_dout    ( ioctl_dout   )
);

reg [15:0] bios_tmp[64];
reg [12:0] bios_addr = 0;
reg [15:0] bios_din;
reg        bios_wr = 0;
wire       bios_req;
reg        bios_loaded = 0;

always @(posedge clk_sdr) begin
	reg [7:0] dat;
	reg       bios_reqD;
	reg       ioctl_downlD;

	ioctl_downlD <= ioctl_downl;
	if (ioctl_downl & ~ioctl_downlD) begin
		bios_addr <= 0;
		bios_wr <= 0;
	end

	if (ioctl_downlD & ~ioctl_downl) bios_loaded <= 1;

	if (ioctl_downl & ioctl_wr) begin
		if (ioctl_addr[0]) begin
			bios_tmp[ioctl_addr[6:1]] <= {ioctl_dout, dat};
			if (&ioctl_addr[5:1]) bios_wr <= 1;
		end else begin
			dat <= ioctl_dout;
		end
	end

	bios_reqD <= bios_req;
	if (bios_reqD & ~bios_req) bios_wr <= 0;

	if (ioctl_downl & bios_req) begin
		bios_addr <= bios_addr + 1'd1;
		bios_din <= bios_tmp[bios_addr[5:0]];
	end
end

wire [3:0]IO;

system sys_inst (
	.clk_NTSC(clk_NTSC),
	.clk_25(clk_25),
	.clk_sdr(clk_sdr),
	.clk_cpu(clk_cpu),
	.clk_opl(clk_50),
	.CLK44100x256(CLK44100x256),
	.cpu_speed(cpu_speed),
	.VGA_R(core_r),
	.VGA_G(core_g),
	.VGA_B(core_b),
	.VGA_HSYNC(core_hs),
	.VGA_VSYNC(core_vs),
	.frame_on(),
	.sdr_n_CS_WE_RAS_CAS({SDRAM_nCS, SDRAM_nWE, SDRAM_nRAS, SDRAM_nCAS}),
	.sdr_BA(SDRAM_BA),
	.sdr_ADDR(SDRAM_A),
	.sdr_DATA(SDRAM_DQ),
	.sdr_DQM({SDRAM_DQMH, SDRAM_DQML}),
	.LED(LED),
	.BTN_RESET(reset),
	.BTN_NMI(1'b0),
	.SD_n_CS(sd_cs),
	.SD_DI(sd_sdi),
	.SD_CK(sd_sck),
	.SD_DO(sd_sdo),
	.AUD_L(AUDIO_L),
	.AUD_R(AUDIO_R),
	.PS2_CLK1_I(ps2_kbd_clk),
	.PS2_CLK1_O(ps2_kbd_clk_i),
	.PS2_CLK2_I(ps2_mouse_clk),
	.PS2_CLK2_O(ps2_mouse_clk_i),
	.PS2_DATA1_I(ps2_kbd_dat),
	.PS2_DATA1_O(ps2_kbd_dat_i),
	.PS2_DATA2_I(ps2_mouse_dat),
	.PS2_DATA2_O(ps2_mouse_dat_i),
	.I2C_SCL(),
	.I2C_SDA(),
	.BIOS_ADDR(bios_addr),
	.BIOS_DIN(bios_din),
	.BIOS_WR(bios_wr),
	.BIOS_REQ(bios_req),
	.VSM(VSM),
	.HSM(HSM),
	.HLFA(HLFA)
);

endmodule
