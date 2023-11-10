// ESE 507 Project 3 Handout Code
// You may not redistribute this code

// Getting started:
// The main() function contains the code to read the parameters. 
// For Parts 1 and 2, your code should be in the genFCLayer() function. Please
// also look at this function to see an example for how to create the ROMs.
//
// For Part 3, your code should be in the genNetwork() function.

// Names: Kshemal Gupte, Shruti Chandrakant Tajne

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <assert.h>
#include <math.h>
using namespace std;

// Function prototypes
void printUsage();
void genFCLayer(int M, int N, int T, int R, int P, vector<vector<int>>& constvector, string modName, ofstream &os);
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B, vector<int>& constVector, string modName, ofstream &os);
void readConstants(ifstream &constStream, vector<vector<int>>& constvector, int P, int M, int N);
void genROM(vector<vector<int>>& constVector, int bits, string modName, ofstream &os);
void readFromVector(vector<int> &constVector1, vector<vector<int>>& constvector2, int P, int M, int N);
void readConstants3(ifstream &constStream, vector<int>& constvector3);

int main(int argc, char* argv[]) {

   // If the user runs the program without enough parameters, print a helpful message
   // and quit.
   if (argc < 7) {
      printUsage();
      return 1;
   }

   int mode = atoi(argv[1]);

   ifstream const_file, in_file;
   ofstream os;
   vector<vector<int>> constVector;
   vector<int> constVector3;

   //----------------------------------------------------------------------
   // Look here for Part 1 and 2
   if (((mode == 1) && (argc == 7)) || ((mode == 2) && (argc == 8))) {

      // Mode 1/2: Generate one layer with given dimensions and one testbench

      // --------------- read parameters, etc. ---------------
      int M = atoi(argv[2]);
      int N = atoi(argv[3]);
      int T = atoi(argv[4]);
      int R = atoi(argv[5]);

      int P;

      // If mode == 1, then set P to 1. If mode==2, set P to the value
      // given at the command line.
      if (mode == 1) {
         P=1;
         const_file.open(argv[6]);         
      }
      else {
         P = atoi(argv[6]);
         const_file.open(argv[7]);
      }

      if (const_file.is_open() != true) {
         cout << "ERROR reading constant file " << argv[6] << endl;
         return 1;
      }

      // Read the constants out of the provided file and place them in the constVector vector
      readConstants(const_file, constVector, P, M, N);

      string out_file = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P) + ".sv";

      os.open(out_file);
      if (os.is_open() != true) {
         cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      // call the genFCLayer function you will write to generate this layer
      string modName = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P);
      genFCLayer(M, N, T, R, P, constVector, modName, os);

      in_file.open("repeattemplate.txt");
      if (in_file.is_open() != true) {
         cout << "ERROR opening repeat template file "<< endl;
         return 1;
      }
      string line;
      while(getline (in_file,line) ){ 
         os<<line<<endl;
      }
      in_file.close();

   }
   //--------------------------------------------------------------------


   // ----------------------------------------------------------------
   // Look here for Part 3
   else if ((mode == 3) && (argc == 10)) {
      // Mode 3: Generate three layers interconnected

      // --------------- read parameters, etc. ---------------
      int N  = atoi(argv[2]);
      int M1 = atoi(argv[3]);
      int M2 = atoi(argv[4]);
      int M3 = atoi(argv[5]);
      int T  = atoi(argv[6]);
      int R  = atoi(argv[7]);
      int B  = atoi(argv[8]);

      const_file.open(argv[9]);
      if (const_file.is_open() != true) {
         cout << "ERROR reading constant file " << argv[8] << endl;
         return 1;
      }
      readConstants3(const_file, constVector3);

      string out_file = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B)+ ".sv";


      os.open(out_file);
      if (os.is_open() != true) {
         cout << "ERROR opening " << out_file << " for write." << endl;
         return 1;
      }
      // -------------------------------------------------------------

      string mod_name = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(B);

      // generate the design
      genNetwork(N, M1, M2, M3, T, R, B, constVector3, mod_name, os);
      in_file.open("repeattemplate.txt");
      if (in_file.is_open() != true) {
         cout << "ERROR opening repeat template file "<< endl;
         return 1;
      }
      string line;
      while(getline (in_file,line) ){ 
         os<<line<<endl;
      }
      in_file.close();

   }
   //-------------------------------------------------------

   else {
      printUsage();
      return 1;
   }

   // close the output stream
   os.close();

}

// Read values from the constant file into the vector
void readConstants(ifstream &constStream, vector<vector<int>>& constvector, int P, int M, int N) {
   string constLineString;
   int k;
   for(int i=0; i<M; i++){
      if(i<P){
         constvector.push_back({});
         k=i;
      }else{
         k=(i-P)%P;
      }
      for(int j=0; j<N; j++){
         getline(constStream, constLineString);
         int val = atoi(constLineString.c_str());
         constvector[k].push_back(val);
      }
   }
}

void readFromVector(vector<int> &constVector1, vector<vector<int>>& constvector2, int P, int M, int N) {
   int k,l;
   l=0;
   for(int i=0; i<M; i++){
      if(i<P){
         constvector2.push_back({});
         k=i;
      }else{
         k=(i-P)%P;
      }
      for(int j=0; j<N; j++){
         if(l<constVector1.size()){
            int val = constVector1[l++];
            constvector2[k].push_back(val);
         }
      }
   }
   cout<<"Vector 2 rows: "<<constvector2.size()<<" columns: "<<constvector2[0].size()<<endl;
}

void readConstants3(ifstream &constStream, vector<int>& constvector3) {
   string constLineString;
   while(getline(constStream, constLineString)) {
      int val = atoi(constLineString.c_str());
      constvector3.push_back(val);
   }
}

// Generate a ROM based on values constVector.
// Values should each be "bits" number of bits.
void genROM(vector<vector<int>>& constVector, int bits, string modName, ofstream &os) {

      int numWords = constVector.size()*constVector[0].size();
      int addrBits = ceil(log2(numWords));

      for(int i=0; i<constVector.size(); i++){
         os << "module " << modName <<"_"<< i+1 << "(clk, addr, z);" <<endl;
         os << "   input clk;" << endl;
         os << "   input [" << addrBits-1 << ":0] addr;" << endl;
         os << "   output logic signed [" << bits-1 << ":0] z;" << endl;
         os << "   always_ff @(posedge clk) begin" << endl;
         os << "      case(addr)" << endl;
         for(int j=0; j<constVector[i].size(); j++){
            if (constVector[i][j] < 0)
               os << "        " << j << ": z <= -" << bits << "'d" << abs(constVector[i][j]) << ";" << endl;
            else
               os << "        " << j << ": z <= "  << bits << "'d" << constVector[i][j]      << ";" << endl;
         }
         os << "      endcase" << endl << "   end" << endl << "endmodule" << endl << endl;
      }
}

// Parts 1 and 2
// Here is where you add your code to produce a neural network layer.
void genFCLayer(int M, int N, int T, int R, int P, vector<vector<int>>& constVector, string modName, ofstream &os) {
	ifstream in_file;
	string line;
	in_file.open("template.txt");
	int i;
    if (in_file.is_open() == true) {
     //cout << "ERROR reading input file " << endl;
     //return 1;
			const char *Mp = "parameter M = <ROWS>;";
			const char *Np = "parameter N = <COLUMNS>;";
			const char *Tp = "parameter T = <NUMBITS>;";
			const char *Rp = "parameter R = <RELU>;";
			const char *Pp = "parameter P = <PARALLEL>;";
			const char *romp = "unique_p";
			const char *modp = "unique1";
         const char *dp = "unique2";
         const char *cp = "unique3";
         const char *control = "control_logic";
			const char *memloop = "mem_loop";
			const char *outputloop = "output_loop";
			const char *romloop = "rom_loop";
			const char *dploop = "dploop";
			const char *muxloop = "muxloop";
         const char *memory = "memory";
	
			while(getline (in_file,line) ){
				if(line == Mp){
					os<< "parameter M = "<<M<<";"<<endl;
				}
				else if(line == Np){
					os<< "parameter N = "<<N<<";"<<endl;
				}
				else if(line == Tp){
					os<< "parameter T = "<<T<<";"<<endl;
				}
				else if(line == Rp){
					os<< "parameter R = "<<R<<";"<<endl;
				}
				else if(line == Pp){
					os<< "parameter P = "<<P<<";"<<endl;
				}
				else if(line == modp){
					os<< "module "<<modName<<"(clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);" <<endl;
				}
				else if(line == memloop){
					for(i=1;i<P+1;i++){
						os<<"logic signed [T-1:0] mem_w_"<<i<<";" <<endl;
					}
				}
				else if(line == outputloop){
					for(i=1;i<P+1;i++){
						os<<"logic signed [T-1:0] output_data_"<<i<<";" <<endl;
					}
				}
				else if(line == romloop){
					for(i=1;i<P+1;i++){
						os<<"\t"<<modName<<"_W_rom_"<<i<<" romW_"<<i<<"(clk, addr_w, mem_w_"<<i<<");" <<endl;
					}
				}
				else if(line == dploop){
					for(i=1;i<P+1;i++){
						os<<"\t"<<modName<<"_datapath #("<<M<<", "<<N<<", "<<T<<", "<<R<<", "<<P<<") dp_"<<i<<"_"<<modName<<"(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data_"<<i<<", mem_x, mem_w_"<<i<<");"<<endl;
					}
				}
				else if(line == muxloop){
					for(i=0;i<P-1;i++){
						os<<"\t\telse if (sel == "<<i+1<<")\n\t\t\toutput_data = output_data_"<<i+2<<";"<<endl;
					}
				}
            else if(line == dp){
					os<<"module "<<modName<<"_datapath(clk, reset, input_data, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, output_data, mem_x, mem_w);"<<endl;
				}
            else if(line == cp){
					os<<"module "<<modName<<"_control(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);"<<endl;
				}
            else if(line == control){
					os<<"\t"<<modName<<"_control #("<<M<<", "<<N<<", "<<T<<", "<<R<<", "<<P<<") control_"<<modName<<"(clk, reset, input_valid, output_ready, addr_x, wr_en_x, addr_w, clear_acc, valid_en_acc, input_ready, output_valid, sel);"<<endl;
				}
            else if(line == memory){
               os<<"\tmemory #("<<T<<", "<<N<<") memV_"<<modName<<"(clk, input_data, mem_x, addr_x, wr_en_x);"<<endl;
            }
				else{
					os<<line<<endl;
				}
			}

			in_file.close();
			
  }


	

  // Read the constants out of the provided file and place them in the constVector vector
  //readConstants(const_file, constVector);

    // string out_file = "fc_" + to_string(M) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P) + ".sv";

   //os << "module " << modName << "();" << endl;
   //os << "   // your stuff here!" << endl;
   //os << "endmodule" << endl << endl;

   // You will need to generate ROM(s) with values from the pre-stored constant values.
   // Here is code that demonstrates how to do this for the simple case where you want to put all of
   // the matrix values W in one ROM. This is probably what you will need for P=1, but you will want 
   // to change this for P>1. Please also see some examples of splitting these vectors in the Part 3
   // code.


   // Check there are enough values in the constant file.
   int calcM = constVector.size();
   int calcN = constVector[0].size();
   cout<<"Rows: "<<calcM<<endl;
   cout<<"Columns: "<<calcN<<endl;
   if (M*N != calcM*calcN) {
      cout << "ERROR: constVector does not contain correct amount of data for the requested design" << endl;
      cout << "The design parameters requested require " << M*N+M << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }

   // Generate a ROM (for W) with constants 0 through M*N-1, with T bits
   string romModName = modName + "_W_rom";
   genROM(constVector, T, romModName, os);

}

float calculateThroughput (int M, int N, int P){

   float thruput;
   float c;

   c = N + 1.0 + ((N+2)*(M/P)) + ((1+1)*(M/P)) + (1*((M/P)-1)) + (1*((M/P)-1)) + 2.0;

   thruput = (float)(N/c);

   return thruput;

}

vector<int> divisors(int num)
{
   vector<int> ans;
   for(int i=1;i<=num;++i)
   {
      if(num%i==0)
         ans.push_back(i);
   }
   return ans;
}

vector<int> setP(int M1, int M2, int M3, int N, int B){
   int P1, P2, P3;
   int oldP1,oldP2,oldP3;
   vector<int> ans;
   int delta=2;
   if(B<3){
      B=3;
   }
   if(B>M1+M2+M3){
      B=M1+M2+M3;
   }
   vector<int> M1_divisors = divisors(M1);
   vector<int> M2_divisors = divisors(M2);
   vector<int> M3_divisors = divisors(M3);
   P1=M1_divisors[0];
   P2=M2_divisors[0];
   P3=M3_divisors[0];
   int i=1, j=1,k=1;
   while(P1 + P2 + P3 <= B && (P1!=oldP1 || P2!=oldP2 || P3!=oldP3)){
      float thruput1 = calculateThroughput(M1,N,P1);
      float thruput2 = calculateThroughput(M2,M1,P2);
      float thruput3 = calculateThroughput(M3,M2,P3);
      float min_thruput = min((min(thruput1, thruput2)), thruput3);
      if(min_thruput == thruput1){
         oldP1=P1;
         
         if(M1_divisors[i]+P2+P3 <= B && i<M1_divisors.size()){
            P1=M1_divisors[i];
            i++;
            float new_thruput1 = calculateThroughput(M1,N,P1);
            if(new_thruput1-thruput1<=delta && M1_divisors[i]+P2+P3<=B && i<M1_divisors.size()){
               P1=M1_divisors[i];
               i++;
            }
         }
         if(P1==oldP1){
            break;
         }

      }else if(min_thruput == thruput2){
         oldP2=P2;

         if(M2_divisors[j]+P1+P3 <= B && j<M2_divisors.size()){
            P2=M2_divisors[j];
            j++;
            float new_thruput2 = calculateThroughput(M2,M1,P2);
            if(new_thruput2-thruput2<=delta && M2_divisors[j]+P1+P3<=B && j<M2_divisors.size()){
               P2=M2_divisors[j];
               j++;
            }
         }
         if(P2==oldP2){
            break;
         }
      }else if(min_thruput == thruput3){
         oldP3=P3;

         if(M3_divisors[k]+P2+P1 <= B && k<M3_divisors.size()){
            P3=M3_divisors[k];
            k++;
            float new_thruput3 = calculateThroughput(M3,M2,P3);
            if(new_thruput3-thruput3<=delta && M3_divisors[k]+P2+P1<=B && k<M3_divisors.size()){
               P3=M3_divisors[k];
               k++;
            }
         }
         if(P3==oldP3){
            break;
         }
      }
   }
   ans.push_back(P1);
   ans.push_back(P2);
   ans.push_back(P3);
   //cout<<"M1 M2 M3: "<<M1<<" "<<M2<<" "<<M3<<endl;
   //cout<<"Final values of P: "<<P1<<" "<<P2<<" "<<P3<<endl;
   return ans;
}
// Part 3: Generate a hardware system with three layers interconnected.
// Layer 1: Input length: N, output length: M1
// Layer 2: Input length: M1, output length: M2
// Layer 3: Input length: M2, output length: M3
// B is the number of multipliers your overall design may use.
// Your goal is to build the fastest design that uses B or fewer multipliers
// constVector holds all the constants for your system (all three layers, in order)
void genNetwork(int N, int M1, int M2, int M3, int T, int R, int B, vector<int>& constVector, string modName, ofstream &os) {

   // Here you will write code to figure out the best values to use for P1, P2, and P3, given
   // B. 
   int P1 = 1; // replace this with your optimized value
   int P2 = 1; // replace this with your optimized value
   int P3 = 1; // replace this with your optimized value

   vector<int> ans = setP(M1,M2,M3,N,B);
   P1 = ans[0];
   P2 = ans[1];
   P3 = ans[2];

   // output top-level module
   //os << "module " << modName << "();" << endl;
   //os << "   // this module should instantiate three layers and wire them together" << endl;
   //os << "endmodule" << endl;
   ifstream in_file;
	string line;
	in_file.open("topleveltemplate.txt");
	int i;
    if (in_file.is_open() == true) {
     //cout << "ERROR reading input file " << endl;
     //return 1;
			const char *Mp1 = "parameter M1;";
         const char *Mp2 = "parameter M2;";
         const char *Mp3 = "parameter M3;";
			const char *Np = "parameter N;";
			const char *Tp = "parameter T;";
			const char *Rp = "parameter R;";
			const char *Bp = "parameter B;";
         const char *Pp1 = "parameter P1;";
         const char *Pp2 = "parameter P2;";
         const char *Pp3 = "parameter P3;";
			const char *layer1 = "layer1;";
			const char *layer2 = "layer2;";
			const char *layer3 = "layer3;";
         const char *mod = "mod";
	
			while(getline (in_file,line) ){
				if(line == Mp1){
					os<< "parameter M1 = "<<M1<<";"<<endl;
				}
            else if(line == Mp2){
					os<< "parameter M2 = "<<M2<<";"<<endl;
				}
            else if(line == Mp3){
					os<< "parameter M3 = "<<M3<<";"<<endl;
				}
				else if(line == Np){
					os<< "parameter N = "<<N<<";"<<endl;
				}
				else if(line == Tp){
					os<< "parameter T = "<<T<<";"<<endl;
				}
				else if(line == Rp){
					os<< "parameter R = "<<R<<";"<<endl;
				}
				else if(line == Bp){
					os<< "parameter B = "<<B<<";"<<endl;
            }
            else if(line == Pp1){
					os<< "parameter P1 = "<<P1<<";"<<endl;
            }
            else if(line == Pp2){
					os<< "parameter P2 = "<<P2<<";"<<endl;
            }
            else if(line == Pp3){
					os<< "parameter P3 = "<<P3<<";"<<endl;
            }
            else if(line == mod){
					os<<"module net_"<<N<<"_"<<M1<<"_"<<M2<<"_"<<M3<<"_"<<T<<"_"<<R<<"_"<<B<<" (clk, reset, input_valid, input_ready, input_data, output_valid, output_ready, output_data);"<<endl;
            }
            else if(line == layer1){
					os<< "l1_fc1_"<<M1<<"_"<<N<<"_"<<T<<"_"<<R<<"_"<<P1<<" #("<<M1<<", "<<N<<", "<<T<<", "<<R<<", "<<P1<<") layer1(clk, reset, input_valid, input_ready, input_data, output_valid1, output_ready1, output_data1);" <<endl;
            }
            else if(line == layer2){
					os<< "l2_fc2_"<<M2<<"_"<<M1<<"_"<<T<<"_"<<R<<"_"<<P2<<" #("<<M2<<", "<<M1<<", "<<T<<", "<<R<<", "<<P2<<") layer2(clk, reset, output_valid1, output_ready1, output_data1, output_valid2, output_ready2, output_data2);" <<endl;
            }
            else if(line == layer3){
					os<< "l3_fc3_"<<M3<<"_"<<M2<<"_"<<T<<"_"<<R<<"_"<<P3<<" #("<<M3<<", "<<M2<<", "<<T<<", "<<R<<", "<<P3<<") layer3(clk, reset, output_valid2, output_ready2, output_data2, output_valid, output_ready, output_data);" <<endl;
            }
            else{
					os<<line<<endl;
				}
         }
         in_file.close();
    }

   
   // -------------------------------------------------------------------------
   // Split up constVector for the three layers
   // layer 1's W matrix is M1 x N
   vector<vector<int>> solution1;
   vector<vector<int>> solution2;
   vector<vector<int>> solution3;
   int start = 0;
   int stop = M1*N;
   vector<int> constVector1(&constVector[start], &constVector[stop]);
   readFromVector(constVector1, solution1, P1, M1, N);

   // layer 2's W matrix is M2 x M1
   start = stop;
   stop = start+M2*M1;
   vector<int> constVector2(&constVector[start], &constVector[stop]);
   readFromVector(constVector2, solution2, P2, M2, M1);

   // layer 3's W matrix is M3 x M2
   start = stop;
   stop = start+M3*M2;
   vector<int> constVector3(&constVector[start], &constVector[stop]);
   readFromVector(constVector3, solution3, P3, M3, M2);

   if (stop > constVector.size()) {
      os << "ERROR: constVector does not contain enough data for the requested design" << endl;
      os << "The design parameters requested require " << stop << " numbers, but the provided data only have " << constVector.size() << " constants" << endl;
      assert(false);
   }
   // --------------------------------------------------------------------------


   // generate the three layer modules
   string subModName1 = "l1_fc1_" + to_string(M1) + "_" + to_string(N) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P1);
   genFCLayer(M1, N, T, R, P1, solution1, subModName1, os);

   string subModName2 = "l2_fc2_" + to_string(M2) + "_" + to_string(M1) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P2);
   genFCLayer(M2, M1, T, R, P2, solution2, subModName2, os);

   string subModName3 = "l3_fc3_" + to_string(M3) + "_" + to_string(M2) + "_" + to_string(T) + "_" + to_string(R) + "_" + to_string(P3);
   genFCLayer(M3, M2, T, R, P3, solution3, subModName3, os);

}


void printUsage() {
  cout << "Usage: ./gen MODE ARGS" << endl << endl;

  cout << "   Mode 1 (Part 1): Produce one neural network layer (unparallelized)" << endl;
  cout << "      ./gen 1 M N T R const_file" << endl << endl;

  cout << "   Mode 2 (Part 2): Produce one neural network layer (parallelized)" << endl;
  cout << "      ./gen 2 M N T R P const_file" << endl << endl;

  cout << "   Mode 3 (Part 3): Produce a system with three interconnected layers" << endl;
  cout << "      ./gen 3 N M1 M2 M3 T R B const_file" << endl;
}
