
# A. Obtaining Ground Truth Output

1) Define the FUNCTIONAL_TEST macro on line 35 of main.c, by uncommenting it.
2) Define the SAMPLING_BASED and OBTAIN_GT macro on line 36 and 37, repectively.
3) Build the code, and start debug session. Add watch express for "result_store" variable. Run the code by pressing resume button, and after 5-6 seconds hit pause.
4) Make note of the starting address of the array "result_store".
5) Stop the debugging session.
6) Open the STM32CubeProgrammer. Go to the device memory and enter the starting address in the address field. Under size field enter 128, and select Data width to be 32 bits. And click Read button.
7) From the drop down menu of the Read button, select "Save as" and save it as "Ground_Truth_ProcessSample.bin" under the same director as the test_script.ipynb.

# B. Verifying Circular Buffer Correctness

1) Comment the OBTAIN_GT macro, and uncomment TEST_CIRCULAR
2) Follow steps 3 to 7 similar to section A. But change the file name to "Circular_buffer.bin".
3) Open the test_script python notebook and run the two cells under "Comparing the cirular buffer output with the ground truth array".
4) If the circular buffer output is same as the ground truth, **Test Passed!** will be displayed.

### To verify number of cycle per single invocation of the filter
1) In the STM32CubeIDE, insert a break point at line 273.
2) Start the debug session.
3) Open the SWV Trace Log.
4) Press the resume button 3-4 times, and subtract the number of cycles of the latest ITM Port 31 Data 2 from Data 1 to obtain the result.

# C. Verifying Block Size 3 Correctness

1) Comment the TEST_CIRCULAR and the SAMPLING_BASED macros. And make sure BLOCK_SIZE 3 is uncommented and BLOCK_SIZE 16 is commented.
2) Define the TEST_BLOCK3 macro by uncommenting it.
3) Follow steps 3 to 7 similar to section A. But change the file name to "processblock3.bin".
4) Run two cells under "Comparing the block size 3 output with the ground truth array".

### To verify number of cycle per single invocation of the filter
1) In the STM32CubeIDE, insert a break point at line 273.
2) Start the debug session.
3) Open the SWV Trace Log.
4) Press the resume button 3-4 times, and subtract the number of cycles of the latest ITM Port 31 Data 2 from Data 1 to obtain the cycle per block. To get cycle per sample, divide it by block size (i.e. 3).

# D. Verifying Block Size 3 Unroll Correctness

1) Comment the TEST_CIRCULAR and the SAMPLING_BASED macros. And make sure BLOCK_SIZE 3 is uncommented and BLOCK_SIZE 16 is commented.
2) Comment TEST_BLOCK3 macro and define the TEST_BLOCK3_UNROLL macro by uncommenting it.
3) Follow steps 3 to 7 similar to section A. But change the file name to "processblock3_unroll.bin".
4) Run two cells under "Comparing the block size 3 UNROLL output with the ground truth array".

### To verify number of cycle per single invocation of the filter
1) In the STM32CubeIDE, insert a break point at line 273.
2) Start the debug session.
3) Open the SWV Trace Log.
4) Press the resume button 3-4 times, and subtract the number of cycles of the latest ITM Port 31 Data 2 from Data 1 to obtain the cycle per block. To get cycle per sample, divide it by block size (i.e. 3).

# E. Verifying Block Size 3 Unroll Special Instruction Correctness

1) Comment the TEST_CIRCULAR and the SAMPLING_BASED macros. And make sure BLOCK_SIZE 3 is uncommented and BLOCK_SIZE 16 is commented.
2) Comment TEST_BLOCK3_UNROLL macro and define the TEST_BLOCK3_UNROLL_OPT macro by uncommenting it.
3) Follow steps 3 to 7 similar to section A. But change the file name to "processblock3_unroll_opt.bin".
4) Run two cells under "Comparing the block size 3 UNROLL OPT output with the ground truth array".

### To verify number of cycle per single invocation of the filter
1) In the STM32CubeIDE, insert a break point at line 273.
2) Start the debug session.
3) Open the SWV Trace Log.
4) Press the resume button 3-4 times, and subtract the number of cycles of the latest ITM Port 31 Data 2 from Data 1 to obtain the cycle per block. To get cycle per sample, divide it by block size (i.e. 3).

# F. Verifying Block Size 16 Correctness

1) Comment the TEST_CIRCULAR and the SAMPLING_BASED macros. And make sure BLOCK_SIZE 16 is uncommented and BLOCK_SIZE 3 is commented.
2) Comment TEST_BLOCK3_UNROLL_OPT macro and define the TEST_BLOCK16 macro by uncommenting it.
3) Follow steps 3 to 7 similar to section A. But change the file name to "processblock16.bin".
4) Run two cells under "Comparing the block size 16 output with the ground truth array".

### To verify number of cycle per single invocation of the filter
1) In the STM32CubeIDE, insert a break point at line 273.
2) Start the debug session.
3) Open the SWV Trace Log.
4) Press the resume button 3-4 times, and subtract the number of cycles of the latest ITM Port 31 Data 2 from Data 1 to obtain the cycle per block. To get cycle per sample, divide it by block size (i.e. 16).

# G. Verifying Block Size 16 Unroll Correctness

1) Comment the TEST_CIRCULAR and the SAMPLING_BASED macros. And make sure BLOCK_SIZE 16 is uncommented and BLOCK_SIZE 3 is commented.
2) Comment TEST_BLOCK16 macro and define the TEST_BLOCK16_UNROLL macro by uncommenting it.
3) Follow steps 3 to 7 similar to section A. But change the file name to "processblock16_unroll.bin".
4) Run two cells under "Comparing the block size 16 UNROLL output with the ground truth array".

### To verify number of cycle per single invocation of the filter
1) In the STM32CubeIDE, insert a break point at line 273.
2) Start the debug session.
3) Open the SWV Trace Log.
4) Press the resume button 3-4 times, and subtract the number of cycles of the latest ITM Port 31 Data 2 from Data 1 to obtain the cycle per block. To get cycle per sample, divide it by block size (i.e. 16).

# H. Verifying Block Size 16 Unroll Special Instruction Correctness

1) Comment the TEST_CIRCULAR and the SAMPLING_BASED macros. And make sure BLOCK_SIZE 16 is uncommented and BLOCK_SIZE 3 is commented.
2) Comment TEST_BLOCK16_UNROLL macro and define the TEST_BLOCK16_UNROLL_OPT macro by uncommenting it.
3) Follow steps 3 to 7 similar to section A. But change the file name to "processblock16_unroll_opt.bin".
4) Run two cells under "Comparing the block size 16 UNROLL OPT output with the ground truth array".

### To verify number of cycle per single invocation of the filter
1) In the STM32CubeIDE, insert a break point at line 273.
2) Start the debug session.
3) Open the SWV Trace Log.
4) Press the resume button 3-4 times, and subtract the number of cycles of the latest ITM Port 31 Data 2 from Data 1 to obtain the cycle per block. To get cycle per sample, divide it by block size (i.e. 16).

# Additional Information
- Only one of the macros (OBTAIN_GT, TEST_CIRCULAR, TEST_BLOCK3, TEST_BLOCK3_UNROLL, TEST_BLOCK3_UNROLL_OPT, TEST_BLOCK16, TEST_BLOCK16_UNROLL, and TEST_BLOCK16_UNROLL_OPT) should be defined at a time. All the other remaining should be commented out.
- Define SAMPLING_BASED when invoking TEST_CIRUCLAR or OBTAIN_GT. Undefine it, when doing block processing.
- For block size 3, define BLOCK_SIZE 3 macro. For block size 16, undefine BLOCK_SIZE 3 and define BLOCK_SIZE 16.


