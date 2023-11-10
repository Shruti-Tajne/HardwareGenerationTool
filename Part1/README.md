<h1>Part 1: Basic Generator for One Neural Network Layer</h1>

The goal of Part 1 is to build a hardware generator that produces a SystemVerilog design for one neural network layer using<br/> **y = f(W.x)**, controllable by a number of parameters.

These parameters are:

**1) Matrix dimensions M and N:**
Our matrix W will have M rows and N columns. This also means that our input vector x will have N elements, and the y vector will have M elements.

**2) Bit-width T:**
The parameter T represents the number of bits used as a datatype everywhere in your system. Assume that 4 ≤ T ≤ 32. We will use saturating arithmetic.

**3) ReLU R:**
If R is 1, your generator should produce a design with a ReLU unit after the matrix-vector product. If R is 0, it should omit the ReLU.

**4) Values for the W matrix:**
The generator will read in a text file that contains the constant values your system should use for W. This text file contains one integer (in base 10) per line. The file will be M*N lines, providing the values of W in row-major order.

The hardware that the generator produces should use the same input/output ports and protocol as in Project 2.

For part 1, the generator should produce a design whose top-level module is named fc_M_N_T_R_1, where M, N, T, and R are replaced with their actual parameter values, e.g., fc_4_5_12_1_1.
