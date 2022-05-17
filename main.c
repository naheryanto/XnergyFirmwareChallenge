//###########################################################################
// FILE: main.c
// TITLE XNERGY Firmware Challenge
// Version: 2022.05.16
// NAH

//Implement a control algorithm for a charger that has constant-current constant-voltage
//Implement a CAN communication protocol between a charger and a battery management system (BMS)

//###########################################################################

#include <stdint.h>
#include <stdbool.h>

//put your definition here

    typedef struct
    {
        float input;
        float ref;
        float Kp;
        float Ki;
        float Ka;
        float out;
        float kpi_out;
        float max;
        float min;
        float anti;
        float error;
        float error_integ;
    } PI;

    PI V;
    PI I;

    float Iref, Vref, Imin;
    float Ifb, Vfb;
    float Ifb10, Vfb10;
    float Tsamp;

    uint16_t Iref_H, Iref_L, Vref_H, Iref_L;
    uint16_t enable_command;    //CHARGING MODE : 1 Start charging | 0 Stop charging
    uint16_t CC_state;          //Constant I Control: 1 on | 0 off
    uint16_t CV_state;          //Constant V Control: 1 on | 0 off
    uint16_t charging_status;   //0 Not charging | 1 Charging
    uint16_t charger_state;     //0 Initialization | 1 Pre-operational | 2 Operational

//CAN struct example

    typedef struct {
        uint16_t Data[8];
        uint16_t Length;
        uint32_t ID;
    } CAN_msg_typedef;

    CAN_msg_typedef Can_tx;
    CAN_msg_typedef Can_rx;
    CAN_msg_typedef Can_tx_701;

    void CAN_write(CAN_msg_typedef *msg);
    bool CAN_read(CAN_msg_typedef *msg); //return true if there is received msg

    uint32_t time_ms;

    uint16_t Can_Count;

void Initialization(void){
//initialize your variables here
//...
//...
    V = (PI) {0,0,0,0,0,0,0,0,0,0,0,0};
    I = (PI) {0,0,0,0,0,0,0,0,0,0,0,0};

    Iref=0, Vref=0, Imin=0;
    Iref_H=0, Iref_L=0, Vref_H=0, Iref_L=0;
    enable_command=0, CC_state=0, CV_state=0;
    Ifb=0, Vfb=0;
    Tsamp=0.001;    //1 kHz INT frequency
    Can_Count=0;
}

void CAN_read_handler(void){
//CAN rx
//...
//...

    //bms send CAN message every 200 ms
    Can_rx.ID = 0x201;
    Vref = Can_rx.Data[0];      //Voltage reference/request high
    Vref = Can_rx.Data[1];      //Voltage reference/request low
    Iref = Can_rx.Data[2];      //Current reference/request high
    Iref = Can_rx.Data[3];      //Current reference/request low
    enable_command = Can_rx.Data[4];    //0 Stop Charging | 1 Start Charging

    //if no incoming message from bms in 5 seconds
    //then charger Stop Charging & network state pre-operational
}

void CAN_write_handler(void){
//CAN tx
//...
//...

    //charger send Heartbeat message every 1 second
    Can_tx.ID =0x701;
    Can_tx.Data[0] = charger_state; //0 Initialization | 1 Pre-operational | 2 Operational


    //charger send additional CAN message every 200ms

    Ifb10 = (uint16_t) 10*Ifb; //Typecast float to uint
    Vfb10 = (uint16_t) 10*Vfb; //Typecast float to uint

    Can_tx.ID = 0x181;

    Can_tx.Data[0] = 0x00;      //Voltage feedback high
    Can_tx.Data[1] = 0x00;      //Voltage feedback low
    Can_tx.Data[2] = 0x00;      //Current feedback high
    Can_tx.Data[3] = 0x00;      //Current feedback low
    Can_tx.Data[4] = charging_status;   //0 Not charging | 1 Charging

    }


void control_routine(void){
//run the control algorithm here
//...
//...

    if (CC_state == 1){
        I.input = Ifb;
        I.ref = Iref;
        I.error = I.ref - I.input;
        I.anti = (I.kpi_out - I.out)*(I.Ka);
        I.error_integ = I.error_integ + I.Ki*Tsamp*(I.error - I.anti);
        I.kpi_out = (I.Kp * I.error) + I.error_integ;
        I.out = ((I.kpi_out > I.max) ? I.max : (I.kpi_out < I.min) ? I.min : I.kpi_out);
    }

    if (CV_state == 1)
    {
        V.input = Vfb;
        V.ref = Vref;
        V.error = V.ref - V.input;
        V.anti = (V.kpi_out - V.out)*(V.Ka);
        V.error_integ = V.error_integ + V.Ki*Tsamp*(V.error - V.anti);
        V.kpi_out = (V.Kp * V.error) + V.error_integ;
        V.out = ((V.kpi_out > V.max) ? V.max : (V.kpi_out < V.min) ? V.min : V.kpi_out);
    }

    time_ms++; //assume INT frequency is 1kHz, for timing purpose
    Can_Count++;
    if (Can_Count>=5000)
    {
        CAN_write_handler();
        Can_Count = 0;
    }
}

void main_state_machine(void){
//run the state transition here
//...
//...

    //STATE IDLE: WAIT enable_command=1
    if (enable_command == 0)
    {
        CC_state = 0;
        CV_state = 0;
    }

    //STATE CONSTANT CURRENT
    if (enable_command == 1)
    {
        CC_state = 1;
    }

    //STATE CONSTANT VOLTAGE
    if (Vfb >= Vref)
    {
        CC_state = 0;
        CV_state = 1;
    }

    if (Ifb <= Imin)
    {
        CV_state = 0;
        enable_command = 0;
    }
}

void network_management(void){
//run the network management here
//...
//...

    //STATE PRE-OPERATIONAL
        if (enable_command == 0)
        {
        }

    //STATE OPERATIONAL
        if (enable_command == 1)
        {
        }


}
void main(void){
    Initialization();
//    PieVectTable.EPWM1_INT = &control_routine;
    while(true){
        main_state_machine();
        network_management();
    }
}
