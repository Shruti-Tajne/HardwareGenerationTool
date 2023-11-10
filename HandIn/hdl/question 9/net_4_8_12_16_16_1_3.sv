//*********************************************
//Top Level Module
//*********************************************

module net_4_8_12_16_16_1_3 (clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M1 = 8;
parameter M2 = 12;
parameter M3 = 16;
parameter N = 4;
parameter T = 16;
parameter R = 1;
parameter B = 3;
parameter P1 = 1;
parameter P2 = 1;
parameter P3 = 1;

input clk, reset, input_valid, output_ready;
input signed [T-1:0] input_data;
output logic signed [T-1:0] output_data;
output output_valid, input_ready;

logic output_ready1, output_ready2;
logic output_valid1, output_valid2;
logic [T-1:0] output_data1, output_data2;

l1_fc1_8_4_16_1_1 #(8, 4, 16, 1, 1) layer1(clk, reset, input_valid, input_ready, input_data, output_valid1, output_ready1, output_data1);

l2_fc2_12_8_16_1_1 #(12, 8, 16, 1, 1) layer2(clk, reset, output_valid1, output_ready1, output_data1, output_valid2, output_ready2, output_data2);

l3_fc3_16_12_16_1_1 #(16, 12, 16, 1, 1) layer3(clk, reset, output_valid2, output_ready2, output_data2, output_valid, output_ready, output_data);

endmodule

//*********************************************
//END OF Top Level Module
//*********************************************

//*********************************************
//Main Module
//*********************************************

module l1_fc1_8_4_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 8;
parameter N = 4;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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
logic signed [T-1:0] output_data_1;
localparam SEL = P;
localparam LOGSIZE_SEL = $clog2(SEL);
logic [LOGSIZE_SEL:0] sel;

	
	memory #(16, 4) memV_l1_fc1_8_4_16_1_1(clk, input_data, mem_x, addr_x, wr_en_x);
	l1_fc1_8_4_16_1_1_W_rom_1 romW_1(clk, addr_w, mem_w_1);
	l1_fc1_8_4_16_1_1_control #(8, 4, 16, 1, 1) control_l1_fc1_8_4_16_1_1(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);
	l1_fc1_8_4_16_1_1_datapath #(8, 4, 16, 1, 1) dp_1_l1_fc1_8_4_16_1_1(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_1, mem_x, mem_w_1);

	always_comb begin
		if (sel == 0)
			output_data = output_data_1;
		else 
			output_data = output_data_1;
	end

endmodule

//*******************************************
// Datapath Module
//*******************************************

module l1_fc1_8_4_16_1_1_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);

parameter M = 8;
parameter N = 4;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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

module l1_fc1_8_4_16_1_1_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);

input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 8;
parameter N = 4;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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

module l1_fc1_8_4_16_1_1_W_rom_1(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd3;
        1: z <= -16'd1;
        2: z <= -16'd6;
        3: z <= -16'd1;
        4: z <= -16'd8;
        5: z <= 16'd6;
        6: z <= -16'd5;
        7: z <= 16'd3;
        8: z <= 16'd2;
        9: z <= -16'd6;
        10: z <= -16'd5;
        11: z <= -16'd5;
        12: z <= 16'd2;
        13: z <= 16'd2;
        14: z <= -16'd7;
        15: z <= 16'd3;
        16: z <= 16'd3;
        17: z <= -16'd1;
        18: z <= 16'd5;
        19: z <= -16'd2;
        20: z <= -16'd4;
        21: z <= -16'd1;
        22: z <= -16'd3;
        23: z <= -16'd7;
        24: z <= -16'd6;
        25: z <= -16'd4;
        26: z <= -16'd3;
        27: z <= -16'd1;
        28: z <= -16'd4;
        29: z <= -16'd1;
        30: z <= 16'd2;
        31: z <= 16'd1;
      endcase
   end
endmodule

//*********************************************
//Main Module
//*********************************************

module l2_fc2_12_8_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 12;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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
logic signed [T-1:0] output_data_1;
localparam SEL = P;
localparam LOGSIZE_SEL = $clog2(SEL);
logic [LOGSIZE_SEL:0] sel;

	
	memory #(16, 8) memV_l2_fc2_12_8_16_1_1(clk, input_data, mem_x, addr_x, wr_en_x);
	l2_fc2_12_8_16_1_1_W_rom_1 romW_1(clk, addr_w, mem_w_1);
	l2_fc2_12_8_16_1_1_control #(12, 8, 16, 1, 1) control_l2_fc2_12_8_16_1_1(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);
	l2_fc2_12_8_16_1_1_datapath #(12, 8, 16, 1, 1) dp_1_l2_fc2_12_8_16_1_1(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_1, mem_x, mem_w_1);

	always_comb begin
		if (sel == 0)
			output_data = output_data_1;
		else 
			output_data = output_data_1;
	end

endmodule

//*******************************************
// Datapath Module
//*******************************************

module l2_fc2_12_8_16_1_1_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);

parameter M = 12;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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

module l2_fc2_12_8_16_1_1_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);

input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 12;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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

module l2_fc2_12_8_16_1_1_W_rom_1(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd6;
        1: z <= 16'd5;
        2: z <= -16'd7;
        3: z <= 16'd6;
        4: z <= 16'd3;
        5: z <= -16'd4;
        6: z <= 16'd1;
        7: z <= -16'd2;
        8: z <= -16'd2;
        9: z <= 16'd5;
        10: z <= 16'd1;
        11: z <= -16'd7;
        12: z <= -16'd1;
        13: z <= 16'd2;
        14: z <= 16'd4;
        15: z <= -16'd5;
        16: z <= -16'd7;
        17: z <= 16'd1;
        18: z <= 16'd1;
        19: z <= -16'd2;
        20: z <= -16'd8;
        21: z <= 16'd6;
        22: z <= -16'd1;
        23: z <= -16'd5;
        24: z <= -16'd6;
        25: z <= 16'd5;
        26: z <= 16'd2;
        27: z <= -16'd1;
        28: z <= -16'd4;
        29: z <= -16'd3;
        30: z <= -16'd8;
        31: z <= -16'd6;
        32: z <= -16'd6;
        33: z <= -16'd7;
        34: z <= -16'd8;
        35: z <= 16'd5;
        36: z <= -16'd3;
        37: z <= 16'd1;
        38: z <= -16'd5;
        39: z <= 16'd4;
        40: z <= -16'd2;
        41: z <= 16'd5;
        42: z <= 16'd5;
        43: z <= 16'd6;
        44: z <= -16'd1;
        45: z <= 16'd1;
        46: z <= -16'd7;
        47: z <= 16'd1;
        48: z <= -16'd5;
        49: z <= 16'd2;
        50: z <= 16'd7;
        51: z <= -16'd5;
        52: z <= 16'd0;
        53: z <= -16'd2;
        54: z <= -16'd2;
        55: z <= 16'd2;
        56: z <= -16'd5;
        57: z <= -16'd7;
        58: z <= -16'd7;
        59: z <= -16'd1;
        60: z <= -16'd2;
        61: z <= -16'd6;
        62: z <= 16'd1;
        63: z <= 16'd0;
        64: z <= -16'd5;
        65: z <= 16'd1;
        66: z <= -16'd3;
        67: z <= 16'd1;
        68: z <= -16'd5;
        69: z <= 16'd1;
        70: z <= -16'd3;
        71: z <= 16'd1;
        72: z <= -16'd2;
        73: z <= -16'd6;
        74: z <= -16'd1;
        75: z <= 16'd5;
        76: z <= 16'd3;
        77: z <= 16'd0;
        78: z <= -16'd2;
        79: z <= 16'd6;
        80: z <= -16'd6;
        81: z <= -16'd3;
        82: z <= -16'd6;
        83: z <= 16'd2;
        84: z <= 16'd4;
        85: z <= 16'd0;
        86: z <= -16'd3;
        87: z <= 16'd7;
        88: z <= 16'd1;
        89: z <= -16'd2;
        90: z <= -16'd1;
        91: z <= 16'd7;
        92: z <= 16'd0;
        93: z <= -16'd8;
        94: z <= -16'd1;
        95: z <= 16'd4;
      endcase
   end
endmodule

//*********************************************
//Main Module
//*********************************************

module l3_fc3_16_12_16_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 16;
parameter N = 12;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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
logic signed [T-1:0] output_data_1;
localparam SEL = P;
localparam LOGSIZE_SEL = $clog2(SEL);
logic [LOGSIZE_SEL:0] sel;

	
	memory #(16, 12) memV_l3_fc3_16_12_16_1_1(clk, input_data, mem_x, addr_x, wr_en_x);
	l3_fc3_16_12_16_1_1_W_rom_1 romW_1(clk, addr_w, mem_w_1);
	l3_fc3_16_12_16_1_1_control #(16, 12, 16, 1, 1) control_l3_fc3_16_12_16_1_1(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);
	l3_fc3_16_12_16_1_1_datapath #(16, 12, 16, 1, 1) dp_1_l3_fc3_16_12_16_1_1(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_1, mem_x, mem_w_1);

	always_comb begin
		if (sel == 0)
			output_data = output_data_1;
		else 
			output_data = output_data_1;
	end

endmodule

//*******************************************
// Datapath Module
//*******************************************

module l3_fc3_16_12_16_1_1_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);

parameter M = 16;
parameter N = 12;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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

module l3_fc3_16_12_16_1_1_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);

input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 16;
parameter N = 12;
parameter T = 16;
parameter R = 1;
parameter P = 1;

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

module l3_fc3_16_12_16_1_1_W_rom_1(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd2;
        1: z <= 16'd5;
        2: z <= -16'd3;
        3: z <= 16'd5;
        4: z <= -16'd2;
        5: z <= 16'd2;
        6: z <= -16'd2;
        7: z <= 16'd4;
        8: z <= 16'd4;
        9: z <= 16'd6;
        10: z <= 16'd1;
        11: z <= -16'd1;
        12: z <= -16'd2;
        13: z <= -16'd8;
        14: z <= -16'd2;
        15: z <= 16'd1;
        16: z <= -16'd3;
        17: z <= 16'd0;
        18: z <= -16'd5;
        19: z <= -16'd7;
        20: z <= -16'd8;
        21: z <= 16'd0;
        22: z <= -16'd7;
        23: z <= 16'd2;
        24: z <= 16'd7;
        25: z <= 16'd0;
        26: z <= 16'd1;
        27: z <= -16'd1;
        28: z <= 16'd0;
        29: z <= -16'd7;
        30: z <= -16'd5;
        31: z <= -16'd6;
        32: z <= 16'd6;
        33: z <= 16'd0;
        34: z <= 16'd7;
        35: z <= -16'd4;
        36: z <= -16'd6;
        37: z <= -16'd2;
        38: z <= -16'd8;
        39: z <= 16'd6;
        40: z <= -16'd4;
        41: z <= 16'd1;
        42: z <= -16'd2;
        43: z <= 16'd2;
        44: z <= 16'd1;
        45: z <= 16'd4;
        46: z <= -16'd5;
        47: z <= 16'd7;
        48: z <= -16'd4;
        49: z <= -16'd1;
        50: z <= -16'd8;
        51: z <= -16'd4;
        52: z <= 16'd7;
        53: z <= -16'd7;
        54: z <= 16'd6;
        55: z <= 16'd6;
        56: z <= 16'd1;
        57: z <= 16'd0;
        58: z <= -16'd2;
        59: z <= -16'd6;
        60: z <= 16'd1;
        61: z <= 16'd1;
        62: z <= -16'd4;
        63: z <= -16'd1;
        64: z <= -16'd6;
        65: z <= -16'd4;
        66: z <= 16'd3;
        67: z <= -16'd4;
        68: z <= 16'd2;
        69: z <= 16'd3;
        70: z <= -16'd5;
        71: z <= 16'd6;
        72: z <= -16'd4;
        73: z <= 16'd1;
        74: z <= 16'd0;
        75: z <= 16'd6;
        76: z <= -16'd3;
        77: z <= 16'd4;
        78: z <= 16'd5;
        79: z <= 16'd1;
        80: z <= -16'd5;
        81: z <= 16'd5;
        82: z <= 16'd5;
        83: z <= -16'd6;
        84: z <= 16'd7;
        85: z <= 16'd4;
        86: z <= -16'd7;
        87: z <= 16'd0;
        88: z <= -16'd4;
        89: z <= -16'd1;
        90: z <= 16'd2;
        91: z <= 16'd5;
        92: z <= -16'd8;
        93: z <= 16'd7;
        94: z <= -16'd4;
        95: z <= -16'd6;
        96: z <= -16'd5;
        97: z <= 16'd7;
        98: z <= -16'd1;
        99: z <= 16'd5;
        100: z <= 16'd2;
        101: z <= 16'd2;
        102: z <= 16'd3;
        103: z <= 16'd6;
        104: z <= -16'd5;
        105: z <= -16'd5;
        106: z <= 16'd4;
        107: z <= 16'd0;
        108: z <= 16'd7;
        109: z <= 16'd1;
        110: z <= -16'd7;
        111: z <= -16'd6;
        112: z <= -16'd1;
        113: z <= 16'd6;
        114: z <= -16'd3;
        115: z <= -16'd2;
        116: z <= 16'd2;
        117: z <= -16'd2;
        118: z <= 16'd6;
        119: z <= 16'd6;
        120: z <= 16'd5;
        121: z <= 16'd1;
        122: z <= 16'd3;
        123: z <= 16'd5;
        124: z <= 16'd0;
        125: z <= 16'd7;
        126: z <= -16'd8;
        127: z <= 16'd3;
        128: z <= 16'd6;
        129: z <= -16'd1;
        130: z <= 16'd0;
        131: z <= 16'd0;
        132: z <= -16'd7;
        133: z <= -16'd5;
        134: z <= -16'd1;
        135: z <= -16'd4;
        136: z <= -16'd2;
        137: z <= -16'd5;
        138: z <= 16'd4;
        139: z <= -16'd2;
        140: z <= 16'd5;
        141: z <= 16'd5;
        142: z <= 16'd0;
        143: z <= -16'd4;
        144: z <= 16'd3;
        145: z <= 16'd5;
        146: z <= 16'd2;
        147: z <= -16'd2;
        148: z <= -16'd5;
        149: z <= 16'd0;
        150: z <= -16'd4;
        151: z <= -16'd8;
        152: z <= -16'd7;
        153: z <= -16'd8;
        154: z <= 16'd6;
        155: z <= 16'd1;
        156: z <= 16'd7;
        157: z <= 16'd6;
        158: z <= -16'd4;
        159: z <= 16'd6;
        160: z <= -16'd3;
        161: z <= 16'd4;
        162: z <= -16'd2;
        163: z <= -16'd2;
        164: z <= 16'd7;
        165: z <= 16'd5;
        166: z <= 16'd2;
        167: z <= -16'd2;
        168: z <= -16'd7;
        169: z <= -16'd2;
        170: z <= 16'd4;
        171: z <= 16'd6;
        172: z <= -16'd5;
        173: z <= -16'd4;
        174: z <= -16'd6;
        175: z <= 16'd6;
        176: z <= -16'd6;
        177: z <= 16'd4;
        178: z <= -16'd4;
        179: z <= -16'd3;
        180: z <= -16'd4;
        181: z <= 16'd1;
        182: z <= -16'd2;
        183: z <= -16'd2;
        184: z <= 16'd1;
        185: z <= -16'd4;
        186: z <= 16'd7;
        187: z <= 16'd0;
        188: z <= -16'd6;
        189: z <= -16'd4;
        190: z <= -16'd2;
        191: z <= -16'd1;
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

