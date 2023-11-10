# Part 3: Generating and Optimizing a Three-Layer System

Neural networks typically consist of multiple layers. In this part of the project, we will extend our generator to produce a three-layer network.

Each of the three layers will be generated using our result from Part 2. Now, the output from Layer 1 will be directly connected to the input ports of layer 2, and so on.

<img width="700" alt="proj3part3" src="https://github.com/Shruti-Tajne/HardwareGenerationTool/assets/150401115/248911e7-50c9-4e65-bf90-c1981f9150b9">

Note that this creates a new constraint: the number of elements in the output vector of Layer 1 must be equal to the number of elements in the input vector of Layer 2. Therefore, we will define a new set of parameters to describe these layers.

* We will say that Layer 1’s input has size N and its output vector is length M1.
* Then, Layer 2 will have input size M1 and output size M2.
* Lastly, Layer 3 will take an input vector of size M2 and produce an output of length M3.

Additionally, we will allow the P parameters of the layers to be independent. That is, each layer can have a different amount of parallelism: P1, P2, and P3. (But recall that the parallelism must evenly divide the output size, so M1/P1, M2/P2, and M3/P3 must all be integers.)

NOTE: We are assuming that all three layers have the same values of parameters T and R.

# Measuring Performance as Throughput
The system as we have described it is a big pipeline—values will flow out of one layer into the next. Ideally, all three layers will be able to perform useful computations concurrently. 

* Once Layer 1 is done processing a vector, its resulting output vector will flow into Layer 2, and so on. 
* While Layer 1 is processing a new input vector, Layer 2 can concurrently process the previous result that Layer 1 computed. 
* At the same time, Layer 3 can concurrently process the previous result that Layer 2 computed.

So, we will evaluate the performance of this system based on its best-case throughput.
Specifically, we will measure throughput as the number of input words the system can process per second (with best-case assumptions about input_valid and output_ready signals).

# Optimization Problem
Notice that the throughput of each layer will now depend on its dimensions and its parallelism. This leads to an opportunity for optimization: given a multiplier budget B, how do we choose the P1, P2, and P3 parameters in order to maximize overall throughput while using no more than B multiply-accumulate units? 

Necessarily, B ≥ 3 (since there’s no way for us to produce a layer with less than one multiply-accumulate unit). Similarly, B ≤ (M1+M2+M3), because this would enable the maximum parallelism for each layer.

Our optimization problem is: given N, M1, M2, M3, and B, determine the values of P1, P2, and P3 that will maximize the throughput of your design.

Our optimizer does not need to allocate all B multipliers. It should only use multipliers that will improve the throughput of the design. (In other words, only use the whole budget if each multiplier you add improves throughput.)
