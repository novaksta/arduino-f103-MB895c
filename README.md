# arduino-f103-MB895c

This example shows how to use [MB895](https://www.st.com/resource/en/schematic_pack/mb895-parallel-c04_schematic.pdf) board with stm32duino (binded lcd part of [BSP example](https://github.com/STMicroelectronics/STM32CubeF0/tree/master/Projects/STM32091C_EVAL/Examples/BSP) for F091 eval board).
The example was checked on F103VB eval board ([MB525 STM3210B-EVAL](https://www.st.com/resource/en/user_manual/um0426-stm3210beval-evaluation-board-stmicroelectronics.pdf)), but should not be problem to port any suitable board (when adjusting SPI pinout in stm32091c_eval.cpp).

The original example was BSP-3 license, but this one is SLA0044 -- restricting usage to hardware manufactured by STMicroelectonics.
