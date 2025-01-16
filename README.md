# Image Processing

Performing image convolution on the STM32F4 MCU, which comprises a Cortex-M4 processor core. The multiply and accumulate operation has been optimized by utilizing Flynn's taxonomy single-instruction, multiple-data (SIMD) system and the loop unrolling technique to successfully meet frame-based sampling requirements.

## Image Formatting in Memory
<img src="image_formatting.png" alt="Image Formatting" width="600">
