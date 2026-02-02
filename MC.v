module MC(
	input HCLK,
	input HRESETn,
	input [2:0] BEMF,
	output [2:0] VH,
	output [2:0] VL,
	input [28:0] boot_cmd,
	output [9:0] boot_addr,
	input [11:0] Prediv,
	input [11:0] close_loop_pwm,
	input [9:0] comp_delay,
	input [11:0] ph_overflow,
	input orientation,
	input restart_en,
	input en,
	
	output [2:0] status,
	output [11:0] ph_cnt,
	output stuck
);

//state DEF
parameter [6:0] IDLE        = 7'b0000001;
parameter [6:0] DELAY       = 7'b0000010;
parameter [6:0] RD_CMD      = 7'b0000100;
parameter [6:0] EXE_CMD     = 7'b0001000;
parameter [6:0] COMP_DELAY  = 7'b0010000;
parameter [6:0] RD_BEMF     = 7'b0100000;
parameter [6:0] EXE_LOOP    = 7'b1000000;

//phase DEF
//PH[8:6]=BEMF  PH[5:3]=VL  PH[2:0]=VH
parameter [8:0] PH0 = 9'b000_000_000;
parameter [8:0] PH1 = 9'b100_010_001;
parameter [8:0] PH2 = 9'b010_100_001;
parameter [8:0] PH3 = 9'b001_100_010;
parameter [8:0] PH4 = 9'b100_001_010;
parameter [8:0] PH5 = 9'b010_001_100;
parameter [8:0] PH6 = 9'b001_010_100;

//state ctrl
(* mark_debug="true" *)reg [6:0] current_state;
wire [6:0] next_state;
always@(posedge HCLK or negedge HRESETn) begin
  if(~HRESETn) current_state <= IDLE;
  else current_state <= next_state;
end

//shad register DEF
(* mark_debug="true" *)reg [9:0]  shad_comp_delay;       //registered when DELAY
(* mark_debug="true" *)reg        shad_orientation;      //registered when DELAY
(* mark_debug="true" *)reg        shad_restart_en;       //registered when DELAY
(* mark_debug="true" *)reg [11:0] shad_ph_overflow;      //registered when DELAY
(* mark_debug="true" *)reg [11:0] shad_Prediv;           //registered when DELAY or COMP_DELAY is done
(* mark_debug="true" *)reg [11:0] shad_close_loop_pwm;   //registered when COMP_DELAY is done
(* mark_debug="true" *)reg [11:0] shad_phcnt;            //registered when phase_change_pulse
(* mark_debug="true" *)reg [13:0] shad_cmd_delay;        //registered when RD_CMD
(* mark_debug="true" *)reg [11:0] shad_cmd_PWM;          //registered when RD_CMD
(* mark_debug="true" *)reg [2:0] BEMF_reg;

//counter DEF
(* mark_debug="true" *)reg [13:0] cmd_delay_cnt;   //command delay counter, cleared when !EXE_CMD, enabled when EXE_CMD and PWM_cnt_done
(* mark_debug="true" *)reg [8:0] current_phase;    //phase controller, one-hot encoded. revised when RD_BEMF_done or (RD_CMD & CMD_empty=0)
(* mark_debug="true" *)reg [8:0] last_phase;       //phase recorder, one-hot encoded. record <current_phase> when RD_BEMF_done
(* mark_debug="true" *)reg [11:0] PWM_cnt;         //PWM counter, cleared when !EXE, enabled when EXE
(* mark_debug="true" *)reg [10:0] addr_cnt;        //boot address counter, enabled when RD_CMD, cleared when !(RD_CMD or EXE_CMD)
(* mark_debug="true" *)reg [11:0] phase_cnt;       //phase counter, cleared when phase_change or !(3states), enabled when RD_BEMF_done & !phase_change
(* mark_debug="true" *)reg [9:0] comp_delay_cnt;   //compare delay counter, enabled when COMP_DELAY, cleared when !COMP_DELAY
(* mark_debug="true" *)reg [2:0] BEMF_cnt;         //BEMF counter, enabled when RD_BEMF, cleared when !RD_BEMF
(* mark_debug="true" *)reg [2:0] BEMF_HI_cnt;      //BEMF High counter, cleared when !RD_BEMF, enabled when RD_BEMF and BEMF[x]=1

//control signal DEF
(* mark_debug="true" *)wire PWM_cnt_done         = (PWM_cnt == shad_Prediv)? 1'b1 : 1'b0;                   //PWM count done
(* mark_debug="true" *)wire PWM_CMD              = (PWM_cnt <= shad_cmd_PWM)? 1'b1 : 1'b0;
(* mark_debug="true" *)wire PWM_LOOP             = (PWM_cnt <= shad_close_loop_pwm)? 1'b1 : 1'b0;
(* mark_debug="true" *)wire PWM_OUT              = (current_state == EXE_CMD)? PWM_CMD : PWM_LOOP;         //PWM output control
(* mark_debug="true" *)wire CMD_empty            = ((boot_cmd[14:12] == 3'b0) | addr_cnt[10])? 1'b1 : 1'b0;  //cmd_addr overflow or data invalid
(* mark_debug="true" *)wire EXE_CMD_done         = (cmd_delay_cnt == shad_cmd_delay)? 1'b1 : 1'b0;          //EXE_CMD state is done
(* mark_debug="true" *)wire COMP_DELAY_done      = (comp_delay_cnt == shad_comp_delay)? 1'b1 : 1'b0;        //COMP_DELAY is done
(* mark_debug="true" *)wire RD_BEMF_done         = (BEMF_cnt == 3'd7)? 1'b1 : 1'b0;                         //RD_BEMF is done
(* mark_debug="true" *)wire EXE_LOOP_done        = (PWM_cnt_done)? 1'b1 : 1'b0;                             //EXE_LOOP is done
(* mark_debug="true" *)wire phase_overflow       = (phase_cnt == shad_ph_overflow)? 1'b1 : 1'b0;            //<phase_cnt> overflow
(* mark_debug="true" *)wire phase_change         = (current_phase == last_phase)? 1'b0 : 1'b1;              //phase change indicator
(* mark_debug="true" *)wire BEMF_HI              = (BEMF_HI_cnt <= 3'd3)? 1'b0 : 1'b1;                      //HI indicator, valid when RD_BEMF_done
wire [8:0] next_phase_cmd  = {(9){(boot_cmd[14:12] == 3'd0)}} & PH0 |                //next phase generator, only for cmd control
                              {(9){(boot_cmd[14:12] == 3'd1)}} & PH1 |
							  {(9){(boot_cmd[14:12] == 3'd2)}} & PH2 |
							  {(9){(boot_cmd[14:12] == 3'd3)}} & PH3 |
							  {(9){(boot_cmd[14:12] == 3'd4)}} & PH4 |
							  {(9){(boot_cmd[14:12] == 3'd5)}} & PH5 |
							  {(9){(boot_cmd[14:12] == 3'd6)}} & PH6;
									 
wire [8:0] next_phase_loop_cw  = {(9){((current_phase == PH1) & BEMF_HI) | ((current_phase == PH6) & BEMF_HI)}} & PH1 |       //next phase generator, only for close loop control
                                  {(9){((current_phase == PH2) & ~BEMF_HI) | ((current_phase == PH1) & ~BEMF_HI)}} & PH2 |
								  {(9){((current_phase == PH3) & BEMF_HI) | ((current_phase == PH2) & BEMF_HI)}} & PH3 |
								  {(9){((current_phase == PH4) & ~BEMF_HI) | ((current_phase == PH3) & ~BEMF_HI)}} & PH4 |
								  {(9){((current_phase == PH5) & BEMF_HI) | ((current_phase == PH4) & BEMF_HI)}} & PH5 |
								  {(9){((current_phase == PH6) & ~BEMF_HI) | ((current_phase == PH5) & ~BEMF_HI)}} & PH6;
									 
wire [8:0] next_phase_loop_ccw  = {(9){((current_phase == PH6) & BEMF_HI) | ((current_phase == PH1) & BEMF_HI)}} & PH6 |       //next phase generator, only for close loop control
                                   {(9){((current_phase == PH5) & ~BEMF_HI) | ((current_phase == PH6) & ~BEMF_HI)}} & PH5 |
								   {(9){((current_phase == PH4) & BEMF_HI) | ((current_phase == PH5) & BEMF_HI)}} & PH4 |
								   {(9){((current_phase == PH3) & ~BEMF_HI) | ((current_phase == PH4) & ~BEMF_HI)}} & PH3 |
								   {(9){((current_phase == PH2) & BEMF_HI) | ((current_phase == PH3) & BEMF_HI)}} & PH2 |
								   {(9){((current_phase == PH1) & ~BEMF_HI) | ((current_phase == PH2) & ~BEMF_HI)}} & PH1;									 

wire [8:0] next_phase_loop = (shad_orientation)? next_phase_loop_ccw : next_phase_loop_cw;
									 
(* mark_debug="true" *)wire [8:0] next_phase      = (current_state == RD_CMD)? next_phase_cmd : next_phase_loop;

//phase change pulse generator
(* mark_debug="true" *)wire phase_change_pulse;         //phase change indicator
(* mark_debug="true" *)reg phase_change_reg0;
always@(posedge HCLK) begin
    phase_change_reg0 <= phase_change;
end
assign phase_change_pulse = ~phase_change_reg0 & phase_change;


//control logic
always@(posedge HCLK) begin
    BEMF_reg <= BEMF;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_comp_delay <= 10'b0;
	else if(current_state == DELAY) shad_comp_delay <= comp_delay;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_orientation <= 1'b0;
	else if(current_state == DELAY) shad_orientation <= orientation;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_restart_en <= 1'b0;
	else if(current_state == DELAY) shad_restart_en <= restart_en;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_Prediv <= 12'b0;
	else if((current_state == DELAY) | COMP_DELAY_done) shad_Prediv <= Prediv;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_ph_overflow <= 12'b0;
	else if(current_state == DELAY) shad_ph_overflow <= ph_overflow;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_close_loop_pwm <= 8'b0;
	else if(COMP_DELAY_done) shad_close_loop_pwm <= close_loop_pwm;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_phcnt <= 12'b0;
	else if(phase_change_pulse) shad_phcnt <= phase_cnt;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_cmd_delay <= 14'b0;
	else if(current_state == RD_CMD) shad_cmd_delay <= boot_cmd[28:15];
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) shad_cmd_PWM <= 12'b0;
	else if(current_state == RD_CMD) shad_cmd_PWM <= boot_cmd[11:0];
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) cmd_delay_cnt <= 14'b0;
	else if(~(current_state == EXE_CMD)) cmd_delay_cnt <= 14'b0;
	else if((current_state == EXE_CMD) & PWM_cnt_done) cmd_delay_cnt <= cmd_delay_cnt + 1'b1;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) current_phase <= PH0;
	else if(RD_BEMF_done | ((current_state == RD_CMD) & ~CMD_empty)) current_phase <= next_phase;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) last_phase <= PH0;
	else if(RD_BEMF_done) last_phase <= current_phase;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) PWM_cnt <= 12'b0;
	else if(~((current_state == EXE_CMD) | (current_state == EXE_LOOP)) | PWM_cnt_done) PWM_cnt <= 12'b0;
	else PWM_cnt <= PWM_cnt + 1'b1;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) addr_cnt <= 11'b0;
	else if(~((current_state == RD_CMD) | (current_state == EXE_CMD))) addr_cnt <= 11'b0;
	else if(current_state == RD_CMD) addr_cnt <= addr_cnt + 1'b1;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) phase_cnt <= 14'b0;
	else if(phase_change_pulse | ~((current_state == COMP_DELAY) | (current_state == RD_BEMF) | (current_state == EXE_LOOP))) phase_cnt <= 14'b0;
	else if(~phase_change_pulse & RD_BEMF_done) phase_cnt <= phase_cnt + 1'b1;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) comp_delay_cnt <= 10'b0;
	else if(current_state == COMP_DELAY) comp_delay_cnt <= comp_delay_cnt + 1'b1;
	else comp_delay_cnt <= 10'b0;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) BEMF_cnt <= 3'b0;
	else if(current_state == RD_BEMF) BEMF_cnt <= BEMF_cnt + 1'b1;
	else BEMF_cnt <= 3'b0;
end
always@(posedge HCLK or negedge HRESETn) begin
	if(~HRESETn) BEMF_HI_cnt <= 3'b0;
	else if(~(current_state == RD_BEMF)) BEMF_HI_cnt <= 3'b0;
	else if((current_state == RD_BEMF) & (|(current_phase[8:6] & BEMF_reg))) BEMF_HI_cnt <= BEMF_HI_cnt + 1'b1;
end

//FSM logic
assign next_state = (((current_state == EXE_CMD) & ~en) | ((current_state == EXE_LOOP) & ((en & phase_overflow & ~shad_restart_en) | ~en)))? IDLE :
                    (((current_state == IDLE) & en) | ((current_state == EXE_LOOP) & (phase_overflow & en & shad_restart_en)))? DELAY :
					((current_state == DELAY) | ((current_state == EXE_CMD) & en & EXE_CMD_done))? RD_CMD :
					((current_state == RD_CMD) & ~CMD_empty)? EXE_CMD :
					(((current_state == RD_CMD) & CMD_empty) | ((current_state == EXE_LOOP) & (en & EXE_LOOP_done)))? COMP_DELAY :
					((current_state == COMP_DELAY) & COMP_DELAY_done)? RD_BEMF :
					((current_state == RD_BEMF) & RD_BEMF_done)? EXE_LOOP :
					current_state;

//signal output control
assign stuck = phase_overflow & (current_state == EXE_LOOP) ;
assign ph_cnt = shad_phcnt;
assign status[0] = (current_state == IDLE)? 1'b0 : 1'b1;
assign status[1] = ((current_state == COMP_DELAY) | (current_state == RD_BEMF) | (current_state == EXE_LOOP))? 1'b1 : 1'b0;
assign status[2] = ((current_state == EXE_CMD) | (current_state == EXE_LOOP))? 1'b1 : 1'b0;
assign {VL,VH} = ((current_state == EXE_CMD) | (current_state == EXE_LOOP))? {current_phase[5:3],current_phase[2:0] & {(3){PWM_OUT}}} : 6'b0;
assign boot_addr = addr_cnt[9:0];

endmodule
