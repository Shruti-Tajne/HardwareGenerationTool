//*********************************************
//Top Level Module
//*********************************************

module net_4_8_12_16_16_1_10 (clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M1 = 8;
parameter M2 = 12;
parameter M3 = 16;
parameter N = 4;
parameter T = 16;
parameter R = 1;
parameter B = 10;
parameter P1 = 2;
parameter P2 = 3;
parameter P3 = 4;

input clk, reset, input_valid, output_ready;
input signed [T-1:0] input_data;
output logic signed [T-1:0] output_data;
output output_valid, input_ready;

logic output_ready1, output_ready2;
logic output_valid1, output_valid2;
logic [T-1:0] output_data1, output_data2;

l1_fc1_8_4_16_1_2 #(8, 4, 16, 1, 2) layer1(clk, reset, input_valid, input_ready, input_data, output_valid1, output_ready1, output_data1);

l2_fc2_12_8_16_1_3 #(12, 8, 16, 1, 3) layer2(clk, reset, output_valid1, output_ready1, output_data1, output_valid2, output_ready2, output_data2);

l3_fc3_16_12_16_1_4 #(16, 12, 16, 1, 4) layer3(clk, reset, output_valid2, output_ready2, output_data2, output_valid, output_ready, output_data);

endmodule

//*********************************************
//END OF Top Level Module
//*********************************************

//*********************************************
//Main Module
//*********************************************

module l1_fc1_8_4_16_1_2(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 8;
parameter N = 4;
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

	
	memory #(16, 4) memV_l1_fc1_8_4_16_1_2(clk, input_data, mem_x, addr_x, wr_en_x);
	l1_fc1_8_4_16_1_2_W_rom_1 romW_1(clk, addr_w, mem_w_1);
	l1_fc1_8_4_16_1_2_W_rom_2 romW_2(clk, addr_w, mem_w_2);
	l1_fc1_8_4_16_1_2_control #(8, 4, 16, 1, 2) control_l1_fc1_8_4_16_1_2(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);
	l1_fc1_8_4_16_1_2_datapath #(8, 4, 16, 1, 2) dp_1_l1_fc1_8_4_16_1_2(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_1, mem_x, mem_w_1);
	l1_fc1_8_4_16_1_2_datapath #(8, 4, 16, 1, 2) dp_2_l1_fc1_8_4_16_1_2(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_2, mem_x, mem_w_2);

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

module l1_fc1_8_4_16_1_2_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);

parameter M = 8;
parameter N = 4;
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

module l1_fc1_8_4_16_1_2_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);

input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 8;
parameter N = 4;
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

module l1_fc1_8_4_16_1_2_W_rom_1(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd1;
        1: z <= -16'd5;
        2: z <= -16'd8;
        3: z <= -16'd7;
        4: z <= 16'd2;
        5: z <= -16'd7;
        6: z <= 16'd5;
        7: z <= 16'd7;
        8: z <= 16'd4;
        9: z <= -16'd7;
        10: z <= 16'd6;
        11: z <= 16'd7;
        12: z <= -16'd2;
        13: z <= 16'd4;
        14: z <= 16'd1;
        15: z <= 16'd1;
      endcase
   end
endmodule

module l1_fc1_8_4_16_1_2_W_rom_2(clk, addr, z);
   input clk;
   input [4:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd1;
        1: z <= -16'd3;
        2: z <= 16'd7;
        3: z <= 16'd4;
        4: z <= 16'd6;
        5: z <= -16'd1;
        6: z <= 16'd7;
        7: z <= 16'd5;
        8: z <= 16'd7;
        9: z <= 16'd7;
        10: z <= 16'd3;
        11: z <= -16'd8;
        12: z <= 16'd4;
        13: z <= 16'd2;
        14: z <= -16'd4;
        15: z <= -16'd3;
      endcase
   end
endmodule

//*********************************************
//Main Module
//*********************************************

module l2_fc2_12_8_16_1_3(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 12;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 3;

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
logic signed [T-1:0] mem_w_3;
logic signed [T-1:0] output_data_1;
logic signed [T-1:0] output_data_2;
logic signed [T-1:0] output_data_3;
localparam SEL = P;
localparam LOGSIZE_SEL = $clog2(SEL);
logic [LOGSIZE_SEL:0] sel;

	
	memory #(16, 8) memV_l2_fc2_12_8_16_1_3(clk, input_data, mem_x, addr_x, wr_en_x);
	l2_fc2_12_8_16_1_3_W_rom_1 romW_1(clk, addr_w, mem_w_1);
	l2_fc2_12_8_16_1_3_W_rom_2 romW_2(clk, addr_w, mem_w_2);
	l2_fc2_12_8_16_1_3_W_rom_3 romW_3(clk, addr_w, mem_w_3);
	l2_fc2_12_8_16_1_3_control #(12, 8, 16, 1, 3) control_l2_fc2_12_8_16_1_3(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);
	l2_fc2_12_8_16_1_3_datapath #(12, 8, 16, 1, 3) dp_1_l2_fc2_12_8_16_1_3(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_1, mem_x, mem_w_1);
	l2_fc2_12_8_16_1_3_datapath #(12, 8, 16, 1, 3) dp_2_l2_fc2_12_8_16_1_3(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_2, mem_x, mem_w_2);
	l2_fc2_12_8_16_1_3_datapath #(12, 8, 16, 1, 3) dp_3_l2_fc2_12_8_16_1_3(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_3, mem_x, mem_w_3);

	always_comb begin
		if (sel == 0)
			output_data = output_data_1;
		else if (sel == 1)
			output_data = output_data_2;
		else if (sel == 2)
			output_data = output_data_3;
		else 
			output_data = output_data_1;
	end

endmodule

//*******************************************
// Datapath Module
//*******************************************

module l2_fc2_12_8_16_1_3_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);

parameter M = 12;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 3;

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

module l2_fc2_12_8_16_1_3_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);

input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 12;
parameter N = 8;
parameter T = 16;
parameter R = 1;
parameter P = 3;

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

module l2_fc2_12_8_16_1_3_W_rom_1(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd5;
        1: z <= -16'd4;
        2: z <= -16'd1;
        3: z <= -16'd2;
        4: z <= 16'd2;
        5: z <= -16'd2;
        6: z <= -16'd6;
        7: z <= -16'd4;
        8: z <= 16'd3;
        9: z <= 16'd5;
        10: z <= -16'd8;
        11: z <= -16'd1;
        12: z <= -16'd1;
        13: z <= -16'd4;
        14: z <= 16'd4;
        15: z <= -16'd4;
        16: z <= 16'd6;
        17: z <= 16'd2;
        18: z <= -16'd5;
        19: z <= 16'd7;
        20: z <= 16'd0;
        21: z <= -16'd2;
        22: z <= -16'd1;
        23: z <= -16'd5;
        24: z <= 16'd1;
        25: z <= 16'd7;
        26: z <= -16'd2;
        27: z <= -16'd3;
        28: z <= 16'd3;
        29: z <= 16'd5;
        30: z <= -16'd3;
        31: z <= 16'd1;
      endcase
   end
endmodule

module l2_fc2_12_8_16_1_3_W_rom_2(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd0;
        1: z <= 16'd7;
        2: z <= -16'd5;
        3: z <= -16'd2;
        4: z <= -16'd1;
        5: z <= -16'd6;
        6: z <= -16'd5;
        7: z <= -16'd5;
        8: z <= 16'd1;
        9: z <= -16'd5;
        10: z <= 16'd2;
        11: z <= -16'd5;
        12: z <= 16'd2;
        13: z <= 16'd5;
        14: z <= -16'd1;
        15: z <= -16'd6;
        16: z <= -16'd5;
        17: z <= -16'd1;
        18: z <= 16'd2;
        19: z <= 16'd2;
        20: z <= 16'd4;
        21: z <= -16'd1;
        22: z <= 16'd6;
        23: z <= -16'd3;
        24: z <= -16'd1;
        25: z <= 16'd0;
        26: z <= 16'd0;
        27: z <= 16'd7;
        28: z <= 16'd7;
        29: z <= 16'd7;
        30: z <= -16'd5;
        31: z <= -16'd6;
      endcase
   end
endmodule

module l2_fc2_12_8_16_1_3_W_rom_3(clk, addr, z);
   input clk;
   input [6:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd5;
        1: z <= -16'd6;
        2: z <= -16'd5;
        3: z <= -16'd5;
        4: z <= -16'd7;
        5: z <= 16'd6;
        6: z <= -16'd5;
        7: z <= -16'd1;
        8: z <= 16'd4;
        9: z <= 16'd2;
        10: z <= 16'd0;
        11: z <= -16'd5;
        12: z <= 16'd4;
        13: z <= 16'd4;
        14: z <= -16'd1;
        15: z <= -16'd8;
        16: z <= 16'd2;
        17: z <= 16'd1;
        18: z <= 16'd0;
        19: z <= -16'd4;
        20: z <= -16'd2;
        21: z <= 16'd7;
        22: z <= -16'd2;
        23: z <= -16'd6;
        24: z <= -16'd1;
        25: z <= 16'd5;
        26: z <= 16'd5;
        27: z <= -16'd5;
        28: z <= -16'd4;
        29: z <= 16'd3;
        30: z <= 16'd0;
        31: z <= 16'd7;
      endcase
   end
endmodule

//*********************************************
//Main Module
//*********************************************

module l3_fc3_16_12_16_1_4(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 16;
parameter N = 12;
parameter T = 16;
parameter R = 1;
parameter P = 4;

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
logic signed [T-1:0] mem_w_3;
logic signed [T-1:0] mem_w_4;
logic signed [T-1:0] output_data_1;
logic signed [T-1:0] output_data_2;
logic signed [T-1:0] output_data_3;
logic signed [T-1:0] output_data_4;
localparam SEL = P;
localparam LOGSIZE_SEL = $clog2(SEL);
logic [LOGSIZE_SEL:0] sel;

	
	memory #(16, 12) memV_l3_fc3_16_12_16_1_4(clk, input_data, mem_x, addr_x, wr_en_x);
	l3_fc3_16_12_16_1_4_W_rom_1 romW_1(clk, addr_w, mem_w_1);
	l3_fc3_16_12_16_1_4_W_rom_2 romW_2(clk, addr_w, mem_w_2);
	l3_fc3_16_12_16_1_4_W_rom_3 romW_3(clk, addr_w, mem_w_3);
	l3_fc3_16_12_16_1_4_W_rom_4 romW_4(clk, addr_w, mem_w_4);
	l3_fc3_16_12_16_1_4_control #(16, 12, 16, 1, 4) control_l3_fc3_16_12_16_1_4(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);
	l3_fc3_16_12_16_1_4_datapath #(16, 12, 16, 1, 4) dp_1_l3_fc3_16_12_16_1_4(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_1, mem_x, mem_w_1);
	l3_fc3_16_12_16_1_4_datapath #(16, 12, 16, 1, 4) dp_2_l3_fc3_16_12_16_1_4(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_2, mem_x, mem_w_2);
	l3_fc3_16_12_16_1_4_datapath #(16, 12, 16, 1, 4) dp_3_l3_fc3_16_12_16_1_4(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_3, mem_x, mem_w_3);
	l3_fc3_16_12_16_1_4_datapath #(16, 12, 16, 1, 4) dp_4_l3_fc3_16_12_16_1_4(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_4, mem_x, mem_w_4);

	always_comb begin
		if (sel == 0)
			output_data = output_data_1;
		else if (sel == 1)
			output_data = output_data_2;
		else if (sel == 2)
			output_data = output_data_3;
		else if (sel == 3)
			output_data = output_data_4;
		else 
			output_data = output_data_1;
	end

endmodule

//*******************************************
// Datapath Module
//*******************************************

module l3_fc3_16_12_16_1_4_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);

parameter M = 16;
parameter N = 12;
parameter T = 16;
parameter R = 1;
parameter P = 4;

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

module l3_fc3_16_12_16_1_4_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);

input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 16;
parameter N = 12;
parameter T = 16;
parameter R = 1;
parameter P = 4;

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

module l3_fc3_16_12_16_1_4_W_rom_1(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd4;
        1: z <= -16'd8;
        2: z <= -16'd5;
        3: z <= 16'd2;
        4: z <= 16'd7;
        5: z <= 16'd2;
        6: z <= 16'd5;
        7: z <= 16'd0;
        8: z <= 16'd1;
        9: z <= -16'd5;
        10: z <= 16'd5;
        11: z <= -16'd4;
        12: z <= -16'd1;
        13: z <= -16'd6;
        14: z <= -16'd4;
        15: z <= 16'd4;
        16: z <= 16'd3;
        17: z <= -16'd7;
        18: z <= 16'd0;
        19: z <= -16'd6;
        20: z <= 16'd3;
        21: z <= -16'd1;
        22: z <= 16'd6;
        23: z <= -16'd7;
        24: z <= 16'd2;
        25: z <= 16'd2;
        26: z <= -16'd7;
        27: z <= 16'd3;
        28: z <= -16'd5;
        29: z <= -16'd8;
        30: z <= -16'd7;
        31: z <= 16'd0;
        32: z <= -16'd3;
        33: z <= 16'd3;
        34: z <= 16'd4;
        35: z <= 16'd3;
        36: z <= -16'd6;
        37: z <= 16'd6;
        38: z <= 16'd7;
        39: z <= 16'd1;
        40: z <= 16'd7;
        41: z <= 16'd5;
        42: z <= 16'd2;
        43: z <= -16'd5;
        44: z <= 16'd3;
        45: z <= -16'd3;
        46: z <= -16'd5;
        47: z <= -16'd4;
      endcase
   end
endmodule

module l3_fc3_16_12_16_1_4_W_rom_2(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd8;
        1: z <= -16'd5;
        2: z <= 16'd5;
        3: z <= -16'd1;
        4: z <= 16'd3;
        5: z <= -16'd3;
        6: z <= -16'd2;
        7: z <= 16'd2;
        8: z <= -16'd3;
        9: z <= 16'd1;
        10: z <= 16'd5;
        11: z <= 16'd4;
        12: z <= 16'd6;
        13: z <= 16'd0;
        14: z <= 16'd3;
        15: z <= -16'd3;
        16: z <= -16'd2;
        17: z <= 16'd7;
        18: z <= 16'd3;
        19: z <= 16'd6;
        20: z <= -16'd7;
        21: z <= 16'd1;
        22: z <= 16'd7;
        23: z <= -16'd3;
        24: z <= -16'd8;
        25: z <= 16'd1;
        26: z <= 16'd1;
        27: z <= 16'd0;
        28: z <= 16'd3;
        29: z <= -16'd4;
        30: z <= 16'd3;
        31: z <= 16'd7;
        32: z <= -16'd1;
        33: z <= -16'd8;
        34: z <= 16'd6;
        35: z <= -16'd7;
        36: z <= 16'd6;
        37: z <= -16'd3;
        38: z <= -16'd1;
        39: z <= -16'd7;
        40: z <= 16'd1;
        41: z <= -16'd2;
        42: z <= -16'd1;
        43: z <= 16'd5;
        44: z <= -16'd2;
        45: z <= -16'd2;
        46: z <= -16'd2;
        47: z <= -16'd7;
      endcase
   end
endmodule

module l3_fc3_16_12_16_1_4_W_rom_3(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -16'd1;
        1: z <= 16'd2;
        2: z <= 16'd7;
        3: z <= 16'd3;
        4: z <= -16'd3;
        5: z <= -16'd1;
        6: z <= 16'd2;
        7: z <= 16'd2;
        8: z <= -16'd1;
        9: z <= 16'd6;
        10: z <= -16'd4;
        11: z <= -16'd2;
        12: z <= -16'd3;
        13: z <= -16'd4;
        14: z <= 16'd2;
        15: z <= -16'd5;
        16: z <= -16'd2;
        17: z <= -16'd3;
        18: z <= 16'd5;
        19: z <= 16'd6;
        20: z <= -16'd1;
        21: z <= -16'd6;
        22: z <= 16'd2;
        23: z <= -16'd5;
        24: z <= -16'd4;
        25: z <= 16'd6;
        26: z <= 16'd2;
        27: z <= -16'd8;
        28: z <= 16'd1;
        29: z <= 16'd1;
        30: z <= -16'd6;
        31: z <= -16'd5;
        32: z <= -16'd5;
        33: z <= -16'd5;
        34: z <= 16'd7;
        35: z <= -16'd2;
        36: z <= -16'd7;
        37: z <= 16'd2;
        38: z <= 16'd5;
        39: z <= -16'd2;
        40: z <= 16'd0;
        41: z <= -16'd8;
        42: z <= -16'd2;
        43: z <= 16'd2;
        44: z <= 16'd7;
        45: z <= -16'd3;
        46: z <= -16'd5;
        47: z <= 16'd6;
      endcase
   end
endmodule

module l3_fc3_16_12_16_1_4_W_rom_4(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [15:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 16'd0;
        1: z <= -16'd7;
        2: z <= 16'd6;
        3: z <= -16'd7;
        4: z <= -16'd4;
        5: z <= 16'd3;
        6: z <= -16'd3;
        7: z <= -16'd4;
        8: z <= 16'd6;
        9: z <= -16'd6;
        10: z <= 16'd3;
        11: z <= 16'd2;
        12: z <= -16'd5;
        13: z <= -16'd5;
        14: z <= -16'd3;
        15: z <= 16'd7;
        16: z <= 16'd2;
        17: z <= -16'd5;
        18: z <= -16'd8;
        19: z <= 16'd1;
        20: z <= 16'd4;
        21: z <= 16'd3;
        22: z <= 16'd6;
        23: z <= -16'd6;
        24: z <= -16'd4;
        25: z <= -16'd8;
        26: z <= 16'd7;
        27: z <= 16'd1;
        28: z <= 16'd3;
        29: z <= 16'd3;
        30: z <= -16'd4;
        31: z <= 16'd3;
        32: z <= -16'd4;
        33: z <= 16'd6;
        34: z <= -16'd5;
        35: z <= -16'd8;
        36: z <= -16'd6;
        37: z <= 16'd6;
        38: z <= -16'd7;
        39: z <= 16'd5;
        40: z <= -16'd5;
        41: z <= -16'd4;
        42: z <= -16'd7;
        43: z <= -16'd7;
        44: z <= 16'd1;
        45: z <= 16'd0;
        46: z <= -16'd6;
        47: z <= -16'd6;
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

