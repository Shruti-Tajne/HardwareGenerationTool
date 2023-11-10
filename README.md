# Hardware Generation Tool
A piece of software that flexibly generates hardware

In Project 1, I learned about creating and checking a datapath that does multiply-and-accumulate. Then, in Project 2, I combined this system with memories and a control module to make different types of matrix-vector multipliers.

Now, the aim of this new project is to build on those ideas and make a piece of software. This software can make hardware in a flexible way to speed up the evaluation of layers in fully-connected neural networks.

There are three parts in this project built on top of the previous parts:
1) In the first part, software will take some details as input. These details describe the layer's size, the precision of input data, and whether to use an activation function like "ReLU." The software will then create the corresponding design using SystemVerilog.
2) In the second part, the goal was to add parallelism to make it more efficient.
3) In the third part, we'll take this idea further by using solution from Part 2 to make hardware for a three-layer neural network. This means creating a bigger system by putting together three layers. Here, the software will get a budget for multipliers (the maximum allowed) and details for each layer. The generator will use this info to decide how much parallel processing to use for each layer, maximizing the overall system performance.

A fully-connected layer of neurons is a set where each neuron takes the same N inputs, but has a different set of weights and a different bias value b: 
y[m] = f ( ((Summation of W[m][n]) . x[n]) + b[m] ) ------------- (1)

In this project, we will make the simplifying assumption that b = 0, meaning we can omit the bias from the ReLu, giving: 
y[m] = f( (Summation of W[m][n]) . x[n] ) ------------- (2)

Rather than operating on each individual value of the vector y, we can instead write this operation simply as a matrix-vector multiplication—with the extra inclusion of the function where: 
y = f(W.x) -------------- (3)
- W is an M by N matrix, representing the “weights” of the neural layer
- x is a column-vector of length N, representing the inputs
- f ( ) applies a point-wise ReLU function: any negative value in the vector is replaced with 0, and
- y is a column-vector of length M, representing the outputs.
  
Because this operation is based on a matrix-vector product, the work in this project will build on the matrix-vector multiplier done in Project 2.
