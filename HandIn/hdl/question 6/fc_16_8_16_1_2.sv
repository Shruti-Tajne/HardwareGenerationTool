//*********************************************
//Main Module
//*********************************************

module fc_16_8_16_1_2(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 16;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 2;

input clk, reset, input_valid, output_ready;
input signed [T-1:0] input_data;
output logic signed [T-1:0] output_data;
output output_valid, input_ready;

logic wr_en_x, clear_acc, en_acc, valid_en_acc;
localparam WIDTH_W=T, SIZE_W=(M*N);
localparam LOGSIZE_W = $clog2(SIZE_W); 
logic [LOGSIZE_W-1:0] addr_w;
localparam WIDTH_X=T, SIZE_X=(N);
localparam LOGSIZE_X = $clog2(SIZE_X); 
logic [LOGSIZE_X-1:0] addr_x;
logic signed [T-1:0] mem_x;
logic signed [T-1:0] mem_w_1;
logic signed [T-1:0] mem_w_2;
logic signed [T-1:0] output_data_1;
logic signed [T-1:0] output_data_2;
localparam SEL = P;
localparam LOGSIZE_SEL = $clog2(SEL);
logic [LOGSIZE_SEL:0] sel;

	
	memory #(16, 8) memV_fc_16_8_16_1_2(clk, input_data, mem_x, addr_x, wr_en_x);
	fc_16_8_16_1_2_W_rom_1 romW_1(clk, addr_w, mem_w_1);
	fc_16_8_16_1_2_W_rom_2 romW_2(clk, addr_w, mem_w_2);
	fc_16_8_16_1_2_control #(16, 8, 16, 1, 2) control_fc_16_8_16_1_2(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);
	fc_16_8_16_1_2_datapath #(16, 8, 16, 1, 2) dp_1_fc_16_8_16_1_2(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_1, mem_x, mem_w_1);
	fc_16_8_16_1_2_datapath #(16, 8, 16, 1, 2) dp_2_fc_16_8_16_1_2(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_2, mem_x, mem_w_2);

	always_comb begin
		if (sel == 0)
			output_data = output_data_1;
		else if (sel == 1)
			output_data = output_data_2;
		else 
			output_data = output_data_1;
	end

endmodule

//*******************************************
// Datapath Module
//*******************************************

module fc_16_8_16_1_2_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);

parameter M = 16;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 2;

input clk, reset;
input signed [T-1:0] input_data;
input wr_en_x, clear_acc, valid_en_acc;
localparam WIDTH_W=T, SIZE_W=(M*N);
localparam LOGSIZE_W = $clog2(SIZE_W); 
input [LOGSIZE_W-1:0] addr_w;
localparam WIDTH_X=T, SIZE_X=(N);
localparam LOGSIZE_X = $clog2(SIZE_X); 
input [LOGSIZE_X-1:0] addr_x;
output logic signed [T-1:0] output_data;		
 
input logic signed [T-1:0] mem_x, mem_w;
logic en_acc, temp;
logic signed [T-1:0] wire_add, wire_mul_out, acc_in, wire_fb, output_acc, input_relu;
logic signed [(2*T)-1:0] wire_mul; 

	always_comb begin 
		wire_mul = mem_x * mem_w;
				
		if(wire_mul > ((2**(T-1)) -1)) begin
			wire_mul_out = (2**(T-1)) -1;
		end
		else if (wire_mul < (-(2**(T-1))))begin
			wire_mul_out = -(2**(T-1));
		end
		else begin
			wire_mul_out = wire_mul;  
		end

		wire_add = acc_in + output_acc;
		wire_fb = output_acc;
		
		if((wire_fb[T-1] && acc_in[T-1] && (~wire_add[T-1])) || (~wire_fb[T-1]) && (~acc_in[T-1]) && (wire_add[T-1])) begin
			if(wire_fb[T-1] == 0) begin
				wire_add = (2**(T-1)) -1;				
			end
			else begin
				wire_add = -(2**(T-1));			
			end
		end
		else begin
				wire_add = acc_in + wire_fb;
		end							
	end


	dff reg_C1(valid_en_acc, temp, clk, reset);
	dff reg_C2(temp, en_acc, clk, reset);
	dff #(T) reg_C3(wire_mul_out, acc_in, clk, reset);
	acc #(T) reg_acc(wire_add, output_acc, clk, en_acc, clear_acc);	
 	
	always_comb begin
		input_relu = output_acc;
		if (R == 1) begin		
			if(output_acc[T-1] == 0) begin
				output_data = input_relu;				
			end
			else begin
				output_data = 0;			
			end
		end
		else begin
			output_data = input_relu;
		end
	end
endmodule


//********************************************
// Control Module
//********************************************

module fc_16_8_16_1_2_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);

input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 16;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 2;

localparam WIDTH_W=T, SIZE_W=(M*N); 
localparam LOGSIZE_W = $clog2(SIZE_W); 
output logic [LOGSIZE_W-1:0] addr_w;
localparam WIDTH_X=T, SIZE_X=N;	
localparam LOGSIZE_X = $clog2(SIZE_X); 
output logic [LOGSIZE_X-1:0] addr_x;
localparam SIZE_M = (M/P);
localparam LOGSIZE_M = $clog2(SIZE_M);
localparam SIZE_COUNT_READ = N+1;
localparam LOGSIZE_COUNT_READ = $clog2(SIZE_COUNT_READ);
localparam SIZE_COUNT_ACC = ((M+1)/P);
localparam LOGSIZE_COUNT_ACC = $clog2(SIZE_COUNT_ACC);
localparam SIZE_COUNT_MUX = P;
localparam LOGSIZE_COUNT_MUX = $clog2(SIZE_COUNT_MUX);

parameter [2:0] rst=3'b000, idle=3'b001, load_x=3'b011, read=3'b100, output_state=3'b101, clear=3'b110, done=3'b111;  // FSM state names
logic [2:0] state;

logic reset_countX, reset_count_op, reset_count_acc, reset_count_read, reset_en_acc, reset_mux; 
logic incr_vector, incr_row, incr_count_read, incr_en_acc, incr_mux;
logic overflow_v, overflow_row, overflow_acc, overflow_count_read, overflow_en_acc, overflow_mux;
logic [LOGSIZE_X:0] count_x, count;
logic [LOGSIZE_M:0] count_acc;
logic flag_acc; 
logic [LOGSIZE_COUNT_READ:0] count_read;
logic [LOGSIZE_COUNT_ACC:0] count_en_acc;
output logic [LOGSIZE_COUNT_MUX:0] sel;

counter #(N) countV(clk, reset_countX, incr_vector, count_x, overflow_v);	
counter #(N) countOP(clk, reset_count_op, incr_row, count, overflow_row); //countOP -> Counts output after 1st row MAC
counter #(M/P) countCLR(clk, reset_count_acc, flag_acc, count_acc, overflow_acc); //countCLR -> Counter for clearing output after 1st row MAC
counter #(N+1) countREAD(clk, reset_count_read, incr_count_read, count_read, overflow_count_read);
counter #((M+1)/P) countACC(clk, reset_en_acc, incr_en_acc, count_en_acc, overflow_en_acc);
counter #(P) countMUX(clk, reset_mux, incr_mux, sel, overflow_mux);

always_comb begin
	if (overflow_count_read == 1)
		flag_acc = 1;
	else
		flag_acc = 0;
end
	
always_comb begin
  if ((state == load_x) && (input_valid == 1) && (overflow_v == 0))
		incr_vector = 1;
	else
		incr_vector = 0;
end

always_comb begin 
 if ((input_valid == 1) && (state == load_x)) begin
 		if (overflow_v == 1) begin
		  input_ready = 0;
		  wr_en_x = 0;
		end
		else begin
		  input_ready = 1;
  		wr_en_x = 1;
		end
	end
	else begin
		wr_en_x =0;
		input_ready = 0;
	end
end

always_comb begin
	if (state == output_state) begin
		if (overflow_mux == 0)
			output_valid = 1;
		else
			output_valid = 0;
	end
	else
		output_valid = 0;
end

always_comb begin
	if (state == rst) begin
		addr_w = 0;
		addr_x = 0;
		clear_acc = 1;
	end
	else if (state == load_x) begin
		addr_w = 0;
		addr_x = count_x;
		clear_acc = 0;
	end
	else if (state == idle) begin
		addr_w = 0;
		addr_x = 0;
		clear_acc = 0;
	end
	else if (state == read) begin
		addr_w = (count + (count_acc*N));
		addr_x = count;
		clear_acc = 0;
	end
	else if (state == output_state) begin
		addr_w = 0;
		addr_x = 0;
		clear_acc = 0;
	end
	else if (state == clear) begin
		addr_w = 0;
		addr_x = 0;
		clear_acc = 1;
	end
	else if (state == done) begin
		addr_w = 0;
		addr_x = 0;
		clear_acc = 1;
	end
	else begin
		addr_w = 0;
		addr_x = 0;
		clear_acc = 0;
	end
end

always_comb begin
	if (state == read) begin
		if (overflow_row == 1)
		    incr_row = 0;
		else
    		incr_row = 1;
		if (overflow_en_acc == 0) begin
		    incr_en_acc = 1;
		end
		else begin
		    incr_en_acc = 0;
		end
		incr_count_read = 1;
	end
	else begin
		incr_row = 0;
		incr_en_acc = 0;
		incr_count_read = 0;
	end
end

always_comb begin
	if ((state == read) && (overflow_row ==0))
		valid_en_acc = 1;
	else
		valid_en_acc = 0;
end

always_comb begin
	if((state == output_state) && (output_ready == 1))
		incr_mux = 1;
	else
		incr_mux = 0;
end
	always_ff @(posedge clk) begin
			
		if (reset == 1) begin
			state <= rst;
			reset_countX <= 1;
 			reset_count_op <= 1;
			reset_count_acc <= 1;
			reset_count_read <= 1;
			reset_en_acc <= 1;
			reset_mux <= 1;
		end		
	else begin
 		 if ((overflow_v == 0) && (overflow_row == 0) && (overflow_acc == 0))begin 
			state <= load_x;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;
			reset_mux <= 0;
		end
		
		else if (((state == load_x) || (state == clear)) && (overflow_v == 1) && (overflow_acc == 0))begin 
			state <= idle;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;
			reset_mux <= 0;
		end
		
		else if ((state == idle) && (overflow_v == 1) && (overflow_acc == 0)) begin
			state <= read;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;
			reset_mux <= 0;	
		end

		else if ((overflow_v == 1) && (overflow_row == 1) && (overflow_count_read == 1)) begin
			state <= output_state;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 1;
			reset_en_acc <= 1;
			reset_mux <= 0;
		end

		else if ((state == output_state) && (overflow_mux == 1) && (overflow_v == 1) && (overflow_acc == 0)) begin
			state <= clear;
			reset_countX <= 0;
 			reset_count_op <= 1;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;
			reset_mux <= 1;
		end

    else if ((state == output_state) && (overflow_mux == 1) && (overflow_v == 1) && (overflow_acc == 1)) begin
			state <= done;
			reset_countX <= 1;
 			reset_count_op <= 1;
			reset_count_acc <= 1;
			reset_count_read <= 1;
			reset_en_acc <= 1;
			reset_mux <= 1;
		end
		
	 end				
	end			
	
endmodule

module fc_16_8_16_1_2_W_rom_1(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd77;
        1: z <= -16'd8;
        2: z <= 16'd10;
        3: z <= -16'd26;
        4: z <= -16'd79;
        5: z <= 16'd47;
        6: z <= 16'd108;
        7: z <= 16'd97;
        8: z <= 16'd18;
        9: z <= 16'd72;
        10: z <= -16'd76;
        11: z <= 16'd4;
        12: z <= 16'd82;
        13: z <= 16'd110;
        14: z <= -16'd116;
        15: z <= 16'd66;
        16: z <= -16'd51;
        17: z <= -16'd82;
        18: z <= 16'd112;
        19: z <= -16'd2;
        20: z <= 16'd93;
        21: z <= 16'd92;
        22: z <= -16'd33;
        23: z <= 16'd61;
        24: z <= 16'd59;
        25: z <= 16'd73;
        26: z <= -16'd21;
        27: z <= 16'd13;
        28: z <= 16'd55;
        29: z <= -16'd9;
        30: z <= -16'd49;
        31: z <= 16'd127;
        32: z <= 16'd118;
        33: z <= -16'd36;
        34: z <= 16'd93;
        35: z <= 16'd84;
        36: z <= -16'd72;
        37: z <= -16'd68;
        38: z <= 16'd17;
        39: z <= -16'd58;
        40: z <= 16'd115;
        41: z <= 16'd126;
        42: z <= -16'd118;
        43: z <= 16'd42;
        44: z <= -16'd11;
        45: z <= -16'd38;
        46: z <= 16'd42;
        47: z <= 16'd27;
        48: z <= -16'd59;
        49: z <= 16'd98;
        50: z <= -16'd32;
        51: z <= -16'd3;
        52: z <= -16'd97;
        53: z <= 16'd113;
        54: z <= 16'd67;
        55: z <= -16'd9;
        56: z <= 16'd53;
        57: z <= -16'd117;
        58: z <= -16'd95;
        59: z <= -16'd86;
        60: z <= -16'd27;
        61: z <= 16'd75;
        62: z <= 16'd69;
        63: z <= 16'd123;
      endcase
   end
endmodule

module fc_16_8_16_1_2_W_rom_2(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd95;
        1: z <= -16'd78;
        2: z <= 16'd60;
        3: z <= 16'd1;
        4: z <= -16'd63;
        5: z <= -16'd72;
        6: z <= 16'd52;
        7: z <= -16'd59;
        8: z <= -16'd56;
        9: z <= 16'd47;
        10: z <= 16'd108;
        11: z <= -16'd92;
        12: z <= 16'd60;
        13: z <= 16'd85;
        14: z <= 16'd36;
        15: z <= 16'd9;
        16: z <= -16'd114;
        17: z <= -16'd100;
        18: z <= -16'd66;
        19: z <= -16'd49;
        20: z <= -16'd44;
        21: z <= 16'd115;
        22: z <= 16'd21;
        23: z <= 16'd102;
        24: z <= -16'd90;
        25: z <= -16'd68;
        26: z <= -16'd92;
        27: z <= 16'd98;
        28: z <= -16'd111;
        29: z <= 16'd72;
        30: z <= -16'd20;
        31: z <= -16'd34;
        32: z <= -16'd40;
        33: z <= 16'd79;
        34: z <= 16'd21;
        35: z <= 16'd44;
        36: z <= 16'd66;
        37: z <= -16'd86;
        38: z <= 16'd19;
        39: z <= -16'd3;
        40: z <= 16'd22;
        41: z <= 16'd78;
        42: z <= -16'd3;
        43: z <= 16'd39;
        44: z <= 16'd22;
        45: z <= 16'd105;
        46: z <= -16'd123;
        47: z <= 16'd12;
        48: z <= 16'd65;
        49: z <= -16'd39;
        50: z <= -16'd92;
        51: z <= 16'd3;
        52: z <= 16'd3;
        53: z <= 16'd55;
        54: z <= -16'd127;
        55: z <= -16'd9;
        56: z <= 16'd10649;
        57: z <= 16'd13122;
        58: z <= 16'd16802;
        59: z <= 16'd5423;
        60: z <= -16'd21204;
        61: z <= -16'd22616;
        62: z <= -16'd27460;
        63: z <= -16'd1423;
      endcase
   end
endmodule

//*********************************************
//DFF_acc Module
//*********************************************

module acc(d, q, clk, en_acc, clear_acc);
	parameter 	WIDTH=1;	
	input [WIDTH-1:0] d;
	input clk, en_acc, clear_acc;
	output logic signed [WIDTH-1:0] q;			

	always_ff @(posedge clk) begin
		if (clear_acc == 1 ) begin
			q <= 0;
		end
		else begin 
		if (en_acc == 1) 
			q <= d;
		end	
	end
endmodule

//********************************************
//DFF Module
//********************************************
module dff(d,q,clk,reset);
	parameter 	WIDTH=1;	
	input [WIDTH-1:0] d;
	input clk, reset;
	output logic signed [WIDTH-1:0] q;			

	always_ff @(posedge clk) begin
		if (reset == 1 ) begin
			q <= 0;
		end
		else begin
			q <= d;
		end	
	end
endmodule

//********************************************
//Counter Module
//********************************************

module counter (clk, reset, incr, regOut, overflow);
parameter OF = 2;
localparam LOGSIZE_OF = $clog2(OF);
input clk, reset, incr;
output logic overflow;
output logic [LOGSIZE_OF:0] regOut;
	
	always_ff @(posedge clk) begin
		if (reset == 1)
			regOut <= 0;
		else if (incr ==1) begin
			if (overflow == 1) begin
				regOut <= 0; 
			end
  			else begin
			regOut <= regOut+1;
			end
		end
	end

	always_comb begin 
		if (regOut == OF) begin
			overflow = 1; 
		end
		else begin 
			overflow = 0;
		end
	end
endmodule

//**********************************************
//Memory Module
//**********************************************

module memory(clk, data_in, data_out, addr, wr_en);
parameter WIDTH=14, SIZE=9;
localparam LOGSIZE=$clog2(SIZE);
input [WIDTH-1:0] data_in;
output logic [WIDTH-1:0] data_out;
input [LOGSIZE-1:0] addr;
input clk, wr_en;
logic [SIZE-1:0][WIDTH-1:0] mem;

	always_ff @(posedge clk) begin	
		data_out <= mem[addr];
		if (wr_en)
 		mem[addr] <= data_in;
 	end
endmodule	
	
//***********************************************
//ROM Module
//***********************************************

