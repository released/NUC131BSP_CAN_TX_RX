# NUC131BSP_CAN_TX_RX
 NUC131BSP_CAN_TX_RX


udpate @ 2024/02/27

1. simple NUC131 CAN TX , RX sample code 

2. CAN transceiver PIN#8 , need to pull down (by GPIO output low or pull down)

refer to CAN_RS_PIN_Init

3. system cock should use XTAL for CAN clock initial correctly

4. press digit 1 , will send CAN TX with counter , 

![image](https://github.com/released/NUC131BSP_CAN_TX_RX/blob/main/CAN_Tx_PCAN.jpg)

5. by using CAN_SetRxMsg , will set the individual ID , test with PCAN tool

for example , sed ID 0x612 , 0x707 , will only recieve these 2 ID and by pass others ID in CAN bus

![image](https://github.com/released/NUC131BSP_CAN_TX_RX/blob/main/CAN_Rx_ID_612_707_PCAN.jpg)

6. by using CAN_SetRxMsgAndMsk , will set range to filter ID

for example , set 0x7FC , will filter only allow to receive ID : 0x7FC to 0x7FF

0 1 1 1     1 1 1 1     1 1 0 0

![image](https://github.com/released/NUC131BSP_CAN_TX_RX/blob/main/CAN_Rx_ID_Filter_7FC_7FF_PCAN.jpg)

for example , set 0x7F0 , will filter only allow to receive ID : 0x7F0 to 0x7FF

0 1 1 1     1 1 1 1     0 0 0 0

![image](https://github.com/released/NUC131BSP_CAN_TX_RX/blob/main/CAN_Rx_ID_Filter_7F0_7FF_PCAN.jpg)


