//*********************************************
//Main Module
//*********************************************

module fc_15_13_19_0_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 15;
parameter N = 13;
parameter T = 19;
parameter R = 0;
parameter P = 1;

input clk, reset, input_valid, output_ready;
input signed [T-1:0] input_data;
output signed [T-1:0] output_data;
output output_valid, input_ready;

logic wr_en_x, clear_acc, en_acc, valid_en_acc;
localparam WIDTH_W=T, SIZE_W=(M*N);
localparam LOGSIZE_W = $clog2(SIZE_W); 
logic [LOGSIZE_W-1:0] addr_w;
localparam WIDTH_X=T, SIZE_X=(N);
localparam LOGSIZE_X = $clog2(SIZE_X); 
logic [LOGSIZE_X-1:0] addr_x;

	control_part1 #(M, N, T, R, P) control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid);
	datapath #(M, N, T, R, P) dp(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data);

endmodule

//*******************************************
// Datapath Module
//*******************************************
module datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data);

parameter M = 15;
parameter N = 13;
parameter T = 19;
parameter R = 0;
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

logic signed [T-1:0] mem_w, mem_x;
logic en_acc, temp;
logic signed [T-1:0] wire_add, wire_mul_out, acc_in, wire_fb, output_acc, input_relu;
logic signed [(2*T)-1:0] wire_mul; 

fc_15_13_19_0_1_W_rom romW(clk, addr_w, mem_w);
	memory #(T, N) memV(clk, input_data, mem_x, addr_x, wr_en_x);

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
// Control Module
//********************************************

module control_part1(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid);
input clk, reset, input_valid, output_ready;
output logic output_valid, input_ready;	
output logic wr_en_x, clear_acc, valid_en_acc;

parameter M = 15;
parameter N = 13;
parameter T = 19;
parameter R = 0;
parameter P = 1;

localparam WIDTH_W=T, SIZE_W=(M*N); 
localparam LOGSIZE_W = $clog2(SIZE_W); 
output logic [LOGSIZE_W-1:0] addr_w;
localparam WIDTH_X=T, SIZE_X=N;	
localparam LOGSIZE_X = $clog2(SIZE_X); 
output logic [LOGSIZE_X-1:0] addr_x;
localparam SIZE_M = M;
localparam LOGSIZE_M = $clog2(SIZE_M);
localparam SIZE_COUNT_READ = N+1;
localparam LOGSIZE_COUNT_READ = $clog2(SIZE_COUNT_READ);
localparam SIZE_COUNT_ACC = M+1;
localparam LOGSIZE_COUNT_ACC = $clog2(SIZE_COUNT_ACC);

parameter [2:0] rst=3'b000, idle=3'b001, load_x=3'b011, read=3'b100, output_state=3'b101, clear=3'b110, done=3'b111;  // FSM state names
logic [2:0] state;

logic reset_countX, reset_count_op, reset_count_acc, reset_count_read, reset_en_acc; 
logic incr_vector, incr_row, incr_count_read, incr_en_acc;
logic overflow_v, overflow_row, overflow_acc, overflow_count_read, overflow_en_acc;
logic [LOGSIZE_X:0] count_x, count;
logic [LOGSIZE_M:0] count_acc;
logic flag_acc; 
logic [LOGSIZE_COUNT_READ:0] count_read;
logic [LOGSIZE_COUNT_ACC:0] count_en_acc;

counter #(N) countV(clk, reset_countX, incr_vector, count_x, overflow_v);	
counter #(N) countOP(clk, reset_count_op, incr_row, count, overflow_row);       //countOP -> Counts output after 1st row MAC
counter #(M) countCLR(clk, reset_count_acc, flag_acc, count_acc, overflow_acc); //countCLR -> Counter for clearing output after 1st row MAC
counter #(N+1) countREAD(clk, reset_count_read, incr_count_read, count_read, overflow_count_read);
counter #(M+1) countACC(clk, reset_en_acc, incr_en_acc, count_en_acc, overflow_en_acc);

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

	always_ff @(posedge clk) begin
			
		if (reset == 1) begin
			state <= rst;
			output_valid <= 0;
			reset_countX <= 1;
 			reset_count_op <= 1;
			reset_count_acc <= 1;
			reset_count_read <= 1;
			reset_en_acc <= 1;
		end		
	else begin
 		 if ((overflow_v == 0) && (overflow_row == 0) && (overflow_acc == 0))begin 
			state <= load_x;
			output_valid <= 0;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;
		end
		
		else if (((state == load_x) || (state == clear)) && (overflow_v == 1) && (overflow_acc == 0))begin 
			state <= idle;
			output_valid <= 0;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;
		end
		
		else if ((state == idle) && (overflow_v == 1) && (overflow_acc == 0)) begin
			state <= read;
			output_valid <= 0;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;	
		end

		else if ((overflow_v == 1) && (overflow_row == 1) && (overflow_count_read == 1)) begin
			state <= output_state;
			output_valid <= 1;
			reset_countX <= 0;
 			reset_count_op <= 0;
			reset_count_acc <= 0;
			reset_count_read <= 1;
			reset_en_acc <= 1;
		end

		else if ((state == output_state) && (output_ready == 1) && (overflow_v == 1) && (overflow_acc == 0)) begin
			state <= clear;
			output_valid <= 0;
			reset_countX <= 0;
 			reset_count_op <= 1;
			reset_count_acc <= 0;
			reset_count_read <= 0;
			reset_en_acc <= 0;
		end

    else if ((state == output_state) && (output_ready == 1) && (overflow_v == 1) && (overflow_acc == 1)) begin
			state <= done;
			output_valid <= 0;
			reset_countX <= 1;
 			reset_count_op <= 1;
			reset_count_acc <= 1;
			reset_count_read <= 1;
			reset_en_acc <= 1;
		end
		
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
module fc_15_13_19_0_1_W_rom(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [18:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= -19'd61;
        1: z <= 19'd207;
        2: z <= -19'd101;
        3: z <= -19'd145;
        4: z <= -19'd10;
        5: z <= -19'd196;
        6: z <= 19'd94;
        7: z <= 19'd180;
        8: z <= 19'd37;
        9: z <= 19'd56;
        10: z <= -19'd202;
        11: z <= 19'd251;
        12: z <= 19'd219;
        13: z <= -19'd65;
        14: z <= -19'd9;
        15: z <= -19'd24;
        16: z <= -19'd79;
        17: z <= -19'd97;
        18: z <= -19'd174;
        19: z <= 19'd105;
        20: z <= 19'd224;
        21: z <= -19'd77;
        22: z <= 19'd162;
        23: z <= 19'd196;
        24: z <= 19'd146;
        25: z <= -19'd98;
        26: z <= -19'd61;
        27: z <= -19'd95;
        28: z <= -19'd75;
        29: z <= -19'd134;
        30: z <= -19'd38;
        31: z <= 19'd121;
        32: z <= -19'd183;
        33: z <= 19'd118;
        34: z <= 19'd232;
        35: z <= 19'd63;
        36: z <= 19'd178;
        37: z <= 19'd70;
        38: z <= -19'd13;
        39: z <= -19'd41;
        40: z <= -19'd130;
        41: z <= 19'd42;
        42: z <= -19'd46;
        43: z <= -19'd166;
        44: z <= 19'd233;
        45: z <= 19'd202;
        46: z <= 19'd66;
        47: z <= -19'd102;
        48: z <= -19'd151;
        49: z <= 19'd148;
        50: z <= -19'd253;
        51: z <= -19'd182;
        52: z <= -19'd185;
        53: z <= 19'd165;
        54: z <= -19'd242;
        55: z <= 19'd217;
        56: z <= -19'd188;
        57: z <= -19'd47;
        58: z <= -19'd134;
        59: z <= -19'd7;
        60: z <= 19'd75;
        61: z <= 19'd85;
        62: z <= -19'd142;
        63: z <= 19'd149;
        64: z <= -19'd53;
        65: z <= -19'd165;
        66: z <= -19'd44;
        67: z <= -19'd131;
        68: z <= 19'd161;
        69: z <= 19'd200;
        70: z <= 19'd85;
        71: z <= -19'd224;
        72: z <= -19'd14;
        73: z <= -19'd217;
        74: z <= -19'd134;
        75: z <= -19'd37;
        76: z <= 19'd241;
        77: z <= 19'd188;
        78: z <= 19'd117;
        79: z <= -19'd165;
        80: z <= 19'd80;
        81: z <= 19'd120;
        82: z <= -19'd91;
        83: z <= 19'd151;
        84: z <= 19'd29;
        85: z <= -19'd77;
        86: z <= 19'd113;
        87: z <= 19'd97;
        88: z <= 19'd133;
        89: z <= 19'd235;
        90: z <= -19'd165;
        91: z <= -19'd48;
        92: z <= 19'd64;
        93: z <= -19'd51;
        94: z <= -19'd155;
        95: z <= -19'd245;
        96: z <= 19'd40;
        97: z <= 19'd58;
        98: z <= -19'd119;
        99: z <= -19'd54;
        100: z <= 19'd2;
        101: z <= 19'd222;
        102: z <= -19'd22;
        103: z <= 19'd244;
        104: z <= -19'd251;
        105: z <= 19'd100;
        106: z <= -19'd49;
        107: z <= 19'd247;
        108: z <= 19'd32;
        109: z <= -19'd188;
        110: z <= -19'd174;
        111: z <= -19'd144;
        112: z <= 19'd188;
        113: z <= -19'd9;
        114: z <= -19'd249;
        115: z <= -19'd39;
        116: z <= 19'd170;
        117: z <= 19'd120;
        118: z <= -19'd197;
        119: z <= 19'd47;
        120: z <= 19'd100;
        121: z <= -19'd106;
        122: z <= -19'd256;
        123: z <= -19'd92;
        124: z <= 19'd99;
        125: z <= -19'd155;
        126: z <= -19'd80;
        127: z <= -19'd116;
        128: z <= 19'd159;
        129: z <= 19'd57;
        130: z <= 19'd86;
        131: z <= -19'd95;
        132: z <= 19'd23;
        133: z <= -19'd192;
        134: z <= -19'd107;
        135: z <= 19'd28;
        136: z <= 19'd164;
        137: z <= 19'd100;
        138: z <= 19'd19;
        139: z <= -19'd60;
        140: z <= 19'd168;
        141: z <= 19'd101;
        142: z <= 19'd52;
        143: z <= 19'd100;
        144: z <= -19'd164;
        145: z <= 19'd59;
        146: z <= -19'd194;
        147: z <= -19'd249;
        148: z <= -19'd76;
        149: z <= -19'd135;
        150: z <= 19'd54;
        151: z <= -19'd232;
        152: z <= 19'd15;
        153: z <= 19'd54;
        154: z <= -19'd68;
        155: z <= -19'd142;
        156: z <= 19'd156;
        157: z <= 19'd108;
        158: z <= -19'd2;
        159: z <= 19'd59;
        160: z <= -19'd91;
        161: z <= -19'd172;
        162: z <= 19'd221;
        163: z <= 19'd188;
        164: z <= -19'd108;
        165: z <= -19'd142;
        166: z <= -19'd39;
        167: z <= -19'd200;
        168: z <= 19'd215;
        169: z <= 19'd236;
        170: z <= -19'd4;
        171: z <= 19'd127;
        172: z <= 19'd82;
        173: z <= -19'd208;
        174: z <= -19'd28;
        175: z <= 19'd174;
        176: z <= 19'd108;
        177: z <= 19'd34;
        178: z <= 19'd181;
        179: z <= -19'd224;
        180: z <= 19'd155;
        181: z <= -19'd20;
        182: z <= -19'd147912;
        183: z <= 19'd259242;
        184: z <= 19'd69154;
        185: z <= 19'd225524;
        186: z <= -19'd147172;
        187: z <= -19'd14402;
        188: z <= -19'd242591;
        189: z <= -19'd100325;
        190: z <= -19'd235270;
        191: z <= -19'd98554;
        192: z <= -19'd215441;
        193: z <= -19'd261417;
        194: z <= 19'd154307;
      endcase
   end
endmodule

