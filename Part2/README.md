# Part 2: Adding Parallelism to our System

For this part, we parallelized our design by increasing the number of multiply-accumulate units used in parallel. Here, we take a parameter that specifies the amount of parallelism in terms of the number of multiply-accumulate units we will use. There are several types of parallelism that can be used, but we will focus on only one.

We will define the parameter **P** to represent the number of parallel multiply-accumulate units

In Part 1, we used one multiply-accumulate unit, and we used it to compute one output value at a time. E.g., for P=1:
<img width="656" alt="proj3part21" src="https://github.com/Shruti-Tajne/HardwareGenerationTool/assets/150401115/be8f1fce-d6a1-4be4-a02f-19cfffa9aad2">


Now, as P increases, we will use P multiply-accumulate units to compute P outputs at the same time. E.g., for P=2:
<img width="670" alt="proj3part22" src="https://github.com/Shruti-Tajne/HardwareGenerationTool/assets/150401115/b74c400c-355a-4da9-ba5c-f750ae2c424b">


For convenience, we will constrain P such that M/P is an integer. (Recall: M represents the number of output values our layer produces.) Therefore, if our system outputs M=16 words, then legal values of P are 1, 2, 4, 8, and 16. However if M=13, the only legal values of P are 1 and 13.
