parameter 
    FETCH   = 4'd0,
    ABS0    = 4'd1,
    ABS1    = 4'd2,
    DECODE  = 4'd3,
    DATA    = 4'd4,
    BRA0    = 4'd5,
    BRA1    = 4'd6,
    BRA2    = 4'd7,
    STK0    = 4'd8,
    STK1    = 4'd9,
    STK2    = 4'd10,
    IND0    = 4'd11,
    ZP0     = 4'd12,
    ZP1     = 4'd13,
    RTS0    = 4'd14,
    BCD0    = 4'd15;

parameter
    AI_PCL  = 3'd0,
    AI_PCH  = 3'd1,
    AI_A    = 3'd2,
    AI_X    = 3'd3,
    AI_Y    = 3'd4,
    AI_M    = 3'd5,
    AI_S    = 3'd6,
    AI_ZZZ  = 3'd7;

parameter
    ALU_AI  = 3'd0,
    ALU_ADC = 3'd1,
    ALU_ROL = 3'd2,
    ALU_ROR = 3'd3,
    ALU_ORA = 3'd4,
    ALU_EOR = 3'd5,
    ALU_AND = 3'd6,
    ALU_BI  = 3'd7; 

parameter
    LD_A    = 3'd1,
    LD_X    = 3'd2,
    LD_Y    = 3'd3,
    LD_S    = 3'd4;
