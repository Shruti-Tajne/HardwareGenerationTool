//*********************************************
//Main Module
//*********************************************

module fc_13_16_32_1_1(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);

parameter M = 13;
parameter N = 16;
parameter T = 32;
parameter R = 1;
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

parameter M = 13;
parameter N = 16;
parameter T = 32;
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

logic signed [T-1:0] mem_w, mem_x;
logic en_acc, temp;
logic signed [T-1:0] wire_add, wire_mul_out, acc_in, wire_fb, output_acc, input_relu;
logic signed [(2*T)-1:0] wire_mul; 

fc_13_16_32_1_1_W_rom romW(clk, addr_w, mem_w);
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

parameter M = 13;
parameter N = 16;
parameter T = 32;
parameter R = 1;
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
module fc_13_16_32_1_1_W_rom(clk, addr, z);
   input clk;
   input [7:0] addr;
   output logic signed [31:0] z;
   always_ff @(posedge clk) begin
      case(addr)
        0: z <= 32'd8946;
        1: z <= -32'd8369;
        2: z <= -32'd11520;
        3: z <= 32'd8280;
        4: z <= 32'd32059;
        5: z <= 32'd25665;
        6: z <= 32'd12018;
        7: z <= 32'd13392;
        8: z <= 32'd28470;
        9: z <= -32'd8418;
        10: z <= 32'd9922;
        11: z <= 32'd13097;
        12: z <= 32'd8003;
        13: z <= 32'd32537;
        14: z <= -32'd31648;
        15: z <= -32'd32563;
        16: z <= 32'd21606;
        17: z <= 32'd29108;
        18: z <= -32'd31678;
        19: z <= -32'd3701;
        20: z <= 32'd19138;
        21: z <= 32'd31265;
        22: z <= 32'd870;
        23: z <= 32'd18390;
        24: z <= 32'd6955;
        25: z <= 32'd24222;
        26: z <= 32'd21258;
        27: z <= -32'd26768;
        28: z <= -32'd32693;
        29: z <= -32'd6812;
        30: z <= -32'd17242;
        31: z <= 32'd9021;
        32: z <= 32'd17587;
        33: z <= 32'd4007;
        34: z <= -32'd15467;
        35: z <= 32'd16878;
        36: z <= -32'd3096;
        37: z <= 32'd29320;
        38: z <= -32'd2498;
        39: z <= -32'd7394;
        40: z <= -32'd11866;
        41: z <= -32'd25343;
        42: z <= -32'd27064;
        43: z <= 32'd28905;
        44: z <= -32'd25574;
        45: z <= -32'd25944;
        46: z <= 32'd29110;
        47: z <= 32'd28800;
        48: z <= -32'd29603;
        49: z <= 32'd30200;
        50: z <= -32'd7669;
        51: z <= 32'd22303;
        52: z <= 32'd28697;
        53: z <= 32'd25970;
        54: z <= 32'd7925;
        55: z <= 32'd2884;
        56: z <= 32'd17424;
        57: z <= -32'd3585;
        58: z <= 32'd8885;
        59: z <= 32'd17499;
        60: z <= 32'd22371;
        61: z <= 32'd24411;
        62: z <= -32'd6248;
        63: z <= 32'd7191;
        64: z <= -32'd4350;
        65: z <= 32'd11053;
        66: z <= -32'd8699;
        67: z <= 32'd25323;
        68: z <= 32'd7605;
        69: z <= 32'd21572;
        70: z <= -32'd14839;
        71: z <= 32'd28507;
        72: z <= 32'd28997;
        73: z <= -32'd9135;
        74: z <= 32'd24644;
        75: z <= -32'd29345;
        76: z <= -32'd2310;
        77: z <= 32'd20987;
        78: z <= 32'd32224;
        79: z <= 32'd855;
        80: z <= 32'd18419;
        81: z <= -32'd8213;
        82: z <= -32'd9610;
        83: z <= 32'd14349;
        84: z <= -32'd15011;
        85: z <= 32'd31084;
        86: z <= -32'd15535;
        87: z <= -32'd30355;
        88: z <= -32'd5269;
        89: z <= 32'd26118;
        90: z <= 32'd19912;
        91: z <= -32'd15665;
        92: z <= 32'd17762;
        93: z <= -32'd19104;
        94: z <= 32'd24294;
        95: z <= -32'd19356;
        96: z <= 32'd24718;
        97: z <= -32'd17173;
        98: z <= -32'd26801;
        99: z <= -32'd445;
        100: z <= -32'd28369;
        101: z <= -32'd8871;
        102: z <= -32'd4705;
        103: z <= -32'd32140;
        104: z <= 32'd14762;
        105: z <= -32'd12829;
        106: z <= -32'd28716;
        107: z <= -32'd20316;
        108: z <= -32'd24610;
        109: z <= -32'd29260;
        110: z <= 32'd13307;
        111: z <= 32'd26578;
        112: z <= -32'd4705;
        113: z <= -32'd29070;
        114: z <= 32'd8159;
        115: z <= 32'd13053;
        116: z <= -32'd30754;
        117: z <= 32'd25392;
        118: z <= 32'd15466;
        119: z <= -32'd3255;
        120: z <= 32'd18743;
        121: z <= 32'd2611;
        122: z <= 32'd13848;
        123: z <= 32'd3737;
        124: z <= 32'd16275;
        125: z <= 32'd5374;
        126: z <= 32'd17149;
        127: z <= 32'd8225;
        128: z <= 32'd20970;
        129: z <= 32'd23117;
        130: z <= -32'd24987;
        131: z <= 32'd25369;
        132: z <= -32'd18522;
        133: z <= 32'd3076;
        134: z <= 32'd25998;
        135: z <= 32'd29008;
        136: z <= 32'd23015;
        137: z <= 32'd30050;
        138: z <= -32'd24075;
        139: z <= 32'd31174;
        140: z <= -32'd31978;
        141: z <= 32'd22000;
        142: z <= 32'd24984;
        143: z <= -32'd3915;
        144: z <= 32'd25698;
        145: z <= 32'd375;
        146: z <= -32'd23630;
        147: z <= 32'd27712;
        148: z <= -32'd7001;
        149: z <= 32'd24605;
        150: z <= -32'd8310;
        151: z <= -32'd21026;
        152: z <= -32'd5552;
        153: z <= -32'd27230;
        154: z <= 32'd15479;
        155: z <= -32'd22045;
        156: z <= 32'd10913;
        157: z <= -32'd139;
        158: z <= 32'd18949;
        159: z <= -32'd885;
        160: z <= -32'd9790;
        161: z <= 32'd26730;
        162: z <= -32'd8284;
        163: z <= 32'd4456;
        164: z <= -32'd2962;
        165: z <= -32'd15054;
        166: z <= 32'd696;
        167: z <= -32'd12715;
        168: z <= -32'd17772;
        169: z <= 32'd9389;
        170: z <= -32'd14309;
        171: z <= -32'd16982;
        172: z <= -32'd1378;
        173: z <= -32'd22093;
        174: z <= 32'd11872;
        175: z <= -32'd8448;
        176: z <= 32'd11050;
        177: z <= 32'd21010;
        178: z <= -32'd13503;
        179: z <= -32'd28718;
        180: z <= 32'd12847;
        181: z <= 32'd10955;
        182: z <= -32'd16976;
        183: z <= -32'd25473;
        184: z <= 32'd16493;
        185: z <= 32'd31272;
        186: z <= -32'd14749;
        187: z <= -32'd5362;
        188: z <= -32'd1635;
        189: z <= -32'd28568;
        190: z <= 32'd26521;
        191: z <= 32'd21343;
        192: z <= 32'd382138578;
        193: z <= -32'd116963522;
        194: z <= -32'd459774777;
        195: z <= 32'd1016622400;
        196: z <= -32'd837710736;
        197: z <= 32'd75229055;
        198: z <= -32'd850019435;
        199: z <= -32'd896088315;
        200: z <= 32'd1057917997;
        201: z <= 32'd597492657;
        202: z <= 32'd564331695;
        203: z <= 32'd717883083;
        204: z <= -32'd138793628;
        205: z <= -32'd793070833;
        206: z <= 32'd762144203;
        207: z <= 32'd984668303;
      endcase
   end
endmodule

