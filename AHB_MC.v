module AHB_MC(
	//AHB interface
	input HCLK,
	input HRESETn,
	input HSEL,
	input [1:0] HTRANS,
	input HREADY,
	input [31:0] HADDR,
	input [31:0] HWDATA,
	input HWRITE,
	output [31:0] HRDATA,
	output HRESP,
	output HREADYOUT,
	//MC interface
	input [2:0] BEMF,
	output [2:0] VH,
	output [2:0] VL,
	//ERR interrupt
	output stuck
);

assign HRESP = 1'b0;
assign HREADYOUT = 1'b1;

//control registers DEF
reg [11:0] Prediv;  //PWM predivider
reg [11:0] ph_overflow;  //ph_cnt overflow
reg [11:0] close_loop_pwm;  //close loop PWM
reg [9:0] comp_delay;  //BEMF compare delay
reg [1:0] ctrl;  //ctrl register
reg en;  //en register

//control wire DEF
wire [2:0] status;  //status indicator
wire [9:0] boot_addr;  //max 1024 steps
wire [9:0] boot_addr_wr;
wire [9:0] boot_addr_rd;
wire [28:0] boot_cmd_wr;  //14bits delay, 3bits phase, 12bits PWM
wire [28:0] boot_cmd_rd;
wire [11:0] ph_cnt;
wire boot_wr_en;
wire [11:0] Prediv_next = (|HWDATA[11:8])? HWDATA[11:0] : 12'd256;

//APB Write and read control block
wire write_en = HSEL & HTRANS[1] & HREADY & HWRITE;
wire read_en = HSEL & HTRANS[1] & HREADY & (~HWRITE);
reg wr_en_reg;
reg rd_en_reg;
reg [10:0] addr_reg;
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) wr_en_reg <= 1'b0;
  else if(write_en) wr_en_reg <= 1'b1;
  else wr_en_reg <= 1'b0;
end
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) rd_en_reg <= 1'b0;
  else if(read_en) rd_en_reg <= 1'b1;
  else rd_en_reg <= 1'b0;
end
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) addr_reg <= 11'b0;
  else if(write_en | read_en) addr_reg <= HADDR[12:2];
end

//BRAM control block
assign boot_addr = (en)? boot_addr_rd : boot_addr_wr;
assign boot_addr_wr = addr_reg[9:0];
assign boot_wr_en = (wr_en_reg & ~addr_reg[10] & ~en);
assign boot_cmd_wr = {HWDATA[29:16],HWDATA[14:12],HWDATA[11:0]};

//APB read
assign HRDATA = (rd_en_reg & addr_reg[10])? (
                {(32){(addr_reg[2:0] == 3'd0)}} & {20'b0,close_loop_pwm} |
                {(32){(addr_reg[2:0] == 3'd1)}} & {20'b0,Prediv} |
                {(32){(addr_reg[2:0] == 3'd2)}} & {22'b0,comp_delay} |
                {(32){(addr_reg[2:0] == 3'd3)}} & {30'b0,ctrl} |
				{(32){(addr_reg[2:0] == 3'd4)}} & {20'b0,ph_cnt} |
				{(32){(addr_reg[2:0] == 3'd5)}} & {31'b0,en} |
				{(32){(addr_reg[2:0] == 3'd6)}} & {29'b0,status} |
				{(32){(addr_reg[2:0] == 3'd7)}} & {20'b0,ph_overflow}
				)
		        : 32'b0;
					 
//APB write
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) close_loop_pwm <= 12'b0;
  else if(wr_en_reg & addr_reg[10] & (addr_reg[2:0] == 3'd0)) close_loop_pwm <= HWDATA[11:0];
end
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) Prediv <= 12'd2048;
  else if(wr_en_reg & addr_reg[10] & (addr_reg[2:0] == 3'd1)) Prediv <= Prediv_next;
end
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) comp_delay <= 10'd512;
  else if(wr_en_reg & addr_reg[10] & (addr_reg[2:0] == 3'd2)) comp_delay <= HWDATA[9:0];
end
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) ctrl <= 2'b0;
  else if(wr_en_reg & addr_reg[10] & (addr_reg[2:0] == 3'd3)) ctrl <= HWDATA[1:0];
end
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) ph_overflow <= 12'd2048;
  else if(wr_en_reg & addr_reg[10] & (addr_reg[2:0] == 3'd7)) ph_overflow <= HWDATA[11:0];
end
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) en <= 1'b0;
  else if(wr_en_reg & addr_reg[10] & (addr_reg[2:0] == 3'd5)) en <= HWDATA[0:0];
  else if(stuck & ~ctrl[1]) en <= 1'b0;
end

MC u1_MC(
	.HCLK             (HCLK),
	.HRESETn          (HRESETn),
	.BEMF             (BEMF),
	.VH               (VH),
	.VL               (VL),
	.boot_cmd         (boot_cmd_rd),
	.boot_addr        (boot_addr_rd),
	.Prediv           (Prediv),
	.close_loop_pwm   (close_loop_pwm),
	.comp_delay       (comp_delay),
	.ph_overflow      (ph_overflow),
	.orientation      (ctrl[0]),
	.restart_en       (ctrl[1]),
	.en               (en),
	.status           (status),
	.ph_cnt           (ph_cnt),
	.stuck            (stuck)
);

CMD inst (
  .clka(HCLK),    // input wire clka
  .wea(boot_wr_en),      // input wire [0 : 0] wea
  .addra(boot_addr),  // input wire [9 : 0] addra
  .dina(boot_cmd_wr),    // input wire [28 : 0] dina
  .douta(boot_cmd_rd)  // output wire [28 : 0] douta
);

endmodule
