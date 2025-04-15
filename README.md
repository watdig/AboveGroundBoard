# Above Ground Board
This Firmware allows a host computer to communicate with a custom "AboveGroundBoard" PCB designed for the WatDig design team at the University of Waterloo. The system controls and relays sensor data about the state of a Tunnel Boring Machine (TBM). A flow chart depicting the general design of the system can be found at the following link... https://lucid.app/lucidchart/40cd09a3-0b17-4176-88fb-b93ab9d76a61/edit?viewport_loc=-2870%2C-2245%2C5084%2C2400%2C0_0&invitationId=inv_9890a6aa-6289-44ab-988a-534f96138113

### System Overview
This system consists of 3 sensors wired to directly to 3 twelve bit ADC channels on a STM32C071CBT6 microcontroller. A vl53l0x laser distance sensor is connected to the STM32 microcontroller via I2C. All data is contained within a "register_database" in the STM32 microcontroller, which is essentially just a global array that the host computer can read and write to via the Modbus protocol. The only modbus functions supported in this system are reading multiple holding registers (function code 0x03) or writing multiple holding registers (function code 0x10). Issuing invalid Modbus commands such as writing to a read-only register or exceeding the acceptable value range of a register will return an exception code in accordance with the Modbus protocol. The following link outlines the registers which the user has access to in the system...
https://docs.google.com/spreadsheets/d/11n6w8ZuzljPktblNUjErZGDjPZ7gsNEzAxKXmISzQjk/edit?usp=sharing

![image](https://github.com/user-attachments/assets/9042fecb-2760-46e9-a0ad-f8e796941e85)

![image](https://github.com/user-attachments/assets/5958958c-9a5e-4917-a85a-9ade519a5cbf)
