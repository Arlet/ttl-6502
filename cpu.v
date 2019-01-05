/*
 * verilog model of 6502 CPU.
 *
 * This file is used as a model for logic gate
 * level development of 6502 core. As design
 * evolves, different parts will be rewritten
 * to match the hardware design.
 *
 * initial version passes Klaus Dormann's test suite
 * but does not yet support RST/IRQ/RDY/NMI or BCD.
 * 
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 *
 */

module cpu( clk, RST, AB, DB, WE, IRQ, NMI, RDY );

`include "states.i"

input clk;              // CPU clock 
input RST;              // RST signal
output [15:0] AB;       // address bus
inout [7:0] DB;         // data bus
output WE;              // write enable
input IRQ;              // interrupt request
input NMI;              // non-maskable interrupt request
input RDY;              // Ready signal. Pauses CPU when rdy=0 

reg N, V, D, I, Z, C;   // flags

reg [7:0] S = 8'hff;    
reg [7:0] X = 5;
reg [7:0] Y = 3;
reg [7:0] A = 8'h41;
reg [7:0] M;
reg [7:0] IR;
reg [15:0] PC;

// don't have reset yet, so init AB on code start
reg [15:0] AB = 16'h0400;

reg [7:0] ALU;

wire [7:0] ABH = AB[15:8];
wire [7:0] ABL = AB[7:0];
wire [7:0] PCH = PC[15:8];
wire [7:0] PCL = PC[7:0];

reg WE = 1;             // write enable (active low)
reg [7:0] DO;           // data out
reg [7:0] AI;           // alu input A
reg [7:0] BI;           // alu input B
reg CI;                 // alu input carry

/*
 * databus
 */
assign DB = WE ? 8'hzz : DO;

/*
 * ALU flag outputs
 */

wire NO;
wire ZO;
wire VO;
reg CO;

reg cond_true;

/*
 * some special instructions
 */
reg ind;
wire jmp  = (IR == 8'h4c || IR == 8'h6c || IR == 8'h20 || IR == 8'h00); 
reg rmw;

/*
 * ALU inputs/outputs/operation control signals
 */
reg [2:0] alu_ai;
reg [2:0] alu_bi;
reg [4:0] alu_op;
reg [2:0] alu_ld;
reg alu_pass_hc = 1;

// processor state 
reg [4:0] state = FETCH;

// flags
wire [7:0] P = { N, V, 2'b11, D, I, Z, C };

/* 
 * Address Bus generation
 *
 */
always @(posedge clk)
    case( state )
        DECODE:  
            casez( IR )
                8'b0??0_?000:           AB <= { 8'h01, ALU };       // stack instruction
                8'b????_0001:           AB <= { 8'h00,  DB };       // (ZP,X) or (ZP),Y
                8'b????_01??:           AB <= { 8'h00,  DB };       // ZP (possibly indexed)
            default:                    AB <= {   PCH, PCL };
            endcase

        FETCH:                          AB <= {   PCH, PCL };
        RTS0:                           AB <= {   PCH, PCL };
        DATA: if( !rmw )                AB <= {   PCH, PCL };
        IND0:                           AB <= {   PCH, PCL };
        ABS0:                           AB <= {    DB, ALU }; 
        ABS1:                           AB <= {   ALU, ABL }; 
        ZP0:                            AB <= { 8'h00, ALU };
        ZP1:                            AB <= { 8'h00, ALU };
        STK0:   if( IR[3] )             AB <= {   PCH, PCL };       // only for PHA,PHP
                else                    AB <= { 8'h01, ALU }; 
        STK1:
            casez( IR )
                8'b??0?_????:           AB <= { 8'h01, ALU };       // BRK/RTI
                8'b?01?_????:           AB <= {   PCH, PCL };       // JSR
                8'b?11?_????:           AB <= {    DB, ALU };       // RTS
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           AB <= { 8'hFF, 8'hFE };     // BRK
                8'b?1??_????:           AB <= {    DB, ALU };       // RTI
            endcase
        BRA0:                           AB <= {   PCH, ALU };       // branch taken
        BRA1:                           AB <= {   ALU, ABL };       // page crossing forward
        BRA2:                           AB <= {   ALU, ABL };       // page crossing backward
    endcase

/*
 * Program Counter update
 */

always @(negedge clk)
    case( state )
        DECODE:
            casez( IR )
                8'b0??0_1000:           PC <= AB;                   // PHA, PLA
                default:                PC <= AB + 1;
            endcase
        RTS0, IND0, FETCH:              PC <= AB + 1;
        ABS0: if( IR[3] )               PC <= AB + 1;               // only for true ABS 
    endcase

/*
 * WE
 */
always @(posedge clk)
    case( state )
        DECODE:  
            casez( IR )
                8'b00?0_0000:           WE <= 0;                    // BRK/JSR
                8'b0?00_1000:           WE <= 0;                    // PHA/PHP
                8'b1000_01??:           WE <= 0;                    // STA/STX/STY ZP
                default:                WE <= 1;
            endcase

        STK0:
            casez( IR )
                8'b?0??_0???:           WE <= 0;                    // BRK/JSR
                default:                WE <= 1;
            endcase

        STK1:
            casez( IR )
                8'b?00?_????:           WE <= 0;                    // BRK
                default:                WE <= 1;
            endcase

        DATA:                           WE <= !rmw;

        ABS0: 
            casez( IR )
                8'b100?_????:           WE <= CO;                   // STA (ZP,X)/(ZP),Y ABS
                default:                WE <= 1;
            endcase

        ABS1: 
            casez( IR )
                8'b100?_????:           WE <= 0;                    // STA (ZP,X)/(ZP),Y ABS
                default:                WE <= 1;
            endcase

        ZP0:
            casez( IR )
                8'b1001_01??:           WE <= 0;                    // STA ZP,X/STX ZP,Y
                default:                WE <= 1;
            endcase
            
        default:                        WE <= 1;
    endcase

/* 
 * Data Output
 */
always @* begin
    DO = 8'h55;
    case( state )
        STK0:
            casez( IR )
                8'b?00?_1???:           DO = P;                     // PHP
                8'b?10?_1???:           DO = ALU;                   // PHA
                8'b????_0???:           DO = PCH;                   // JSR/BRK
            endcase

        STK1:
            casez( IR )
                8'b????_0???:           DO = PCL;                   // JSR/BRK
            endcase

        STK2:                           DO = P;                     // BRK

        DATA:                           DO = ALU;
    endcase
end

/*
 * M register holds recent value from DB as ALU input
 */
always @(posedge clk)
    case( state )
        ABS0:                           M <= DB;

        STK0:
            casez( IR )
                8'b??1?_1???:           M <= DB;                    // PLP/PLA
                8'b?1??_0???:           M <= DB;                    // RTS/RTI
            endcase

        ZP1, IND0, DECODE:              M <= DB;

        STK1, STK2:
            casez( IR )
                8'b?1??_????:           M <= DB;                    // RTS/RTI
            endcase

        DATA:                           M <= DB;                    // 
    endcase

/*
 * Instruction Register
 */
always @(posedge clk)
    case( state )
        FETCH:                          IR <= DB;

        DECODE:
            casez( IR )
                8'b???1_10?0:           IR <= DB;                   // odd column 8/A
                8'b1???_10?0:           IR <= DB;                   // top column 8/A
                8'b????_1010:           IR <= DB;                   // column A
            endcase
    endcase

/*
 * ALU AI input
 */

always @* begin
    alu_ai = AI_ZZZ;
    case( state )
        DECODE:
            casez( IR )
                8'b00?0_0000:           alu_ai = AI_S;          // JSR/BRK
                8'b01?0_0000:           alu_ai = AI_S;          // RTS/RTI
                8'b0?00_1000:           alu_ai = AI_S;          // PHP/PHA
                8'b0?10_1000:           alu_ai = AI_S;          // PLP/PLA
                8'b1011_1010:           alu_ai = AI_S;          // TSX
                8'b100?_1010:           alu_ai = AI_X;          // TXA/TXS
                8'b1001_1000:           alu_ai = AI_Y;          // TYA
                8'b1010_10?0:           alu_ai = AI_A;          // TAX/TAY
                8'b1?00_1000:           alu_ai = AI_Y;          // DEY/INY
                8'b1110_1000:           alu_ai = AI_X;          // INX
                8'b1100_1010:           alu_ai = AI_X;          // DEX 
                8'b0???_1010:           alu_ai = AI_A;          // ASL/ROL/LSR/ROR A
            endcase

        ABS0:
            casez( IR )
                8'b1011_1110:           alu_ai = AI_Y;          // LDX ABS,Y 
                8'b????_1001:           alu_ai = AI_Y;          // ABS,Y
                8'b???1_11??:           alu_ai = AI_X;          // ABS,X
                8'b00?0_0000:           alu_ai = AI_M;          // JSR/BRK
                8'b???0_11??:           alu_ai = AI_M;          // ABS
                8'b???1_0001:           alu_ai = AI_Y;          // (ZP), Y
                8'b???0_0001:           alu_ai = AI_M;          // (ZP, X)
            endcase

        ABS1:                           alu_ai = AI_M;          //

        ZP0:
            casez( IR )
                8'b10?1_0110:           alu_ai = AI_Y;          // LDX/STX ZP,Y
                8'b????_01??:           alu_ai = AI_X;          // all other ZP,X
                8'b???0_0001:           alu_ai = AI_X;          // (ZP,X)
                8'b???1_0001:           alu_ai = AI_M;          //
            endcase

        ZP1:
            casez( IR )
                8'b???0_0001:           alu_ai = AI_X;          // (ZP,X)
                8'b???1_0001:           alu_ai = AI_M;          // (ZP), Y
            endcase

        STK0:
            casez( IR )
                8'b????_0???:           alu_ai = AI_S;          // RTS/RTI/JSR/BRK
                8'b??0?_1???:           alu_ai = AI_A;          // PHP/PHA
            endcase

        STK1:
            casez( IR )
                8'b?10?_????:           alu_ai = AI_S;          // RTI
                8'b?0??_????:           alu_ai = AI_S;          // JSR/BRK
                8'b?11?_????:           alu_ai = AI_M;          // RTS
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           alu_ai = AI_S;          // BRK 
                8'b?1??_????:           alu_ai = AI_M;          // RTI
            endcase

        BRA0:                           alu_ai = AI_PCL;        //

        BRA1:                           alu_ai = AI_PCH;        // 

        BRA2:                           alu_ai = AI_PCH;        // 

        DATA:
            casez( IR )
                8'b0???_??10:           alu_ai = AI_M;          // ASL/ROL/LSR/ROR M
                8'b11??_?110:           alu_ai = AI_M;          // INC/DEC M
                8'b100?_??00:           alu_ai = AI_Y;          // STY
                8'b100?_??01:           alu_ai = AI_A;          // STA
                8'b100?_??10:           alu_ai = AI_X;          // STX
            endcase

        FETCH:
            casez( IR )
                8'b101?_????:           alu_ai = AI_M;          // LDA/LDX/LDY
                8'b0?10_1000:           alu_ai = AI_M;          // PLA/PLP
                8'b0?00_1000:           alu_ai = AI_S;          // PHA/PHP
                8'b001?_?100:           alu_ai = AI_A;          // BIT
                8'b0???_??01:           alu_ai = AI_A;          // ORA/AND/EOR/ADC
                8'b11??_??01:           alu_ai = AI_A;          // SBC/CMP
                8'b110?_??00:           alu_ai = AI_Y;          // CPY
                8'b111?_??00:           alu_ai = AI_X;          // CPX
            endcase
    endcase
end

/*
 * ALU BI input
 */

always @* begin
    alu_bi = BI_ZZZ;
    case( state )
        DECODE:
            casez( IR )
                8'b00?0_0000:           alu_bi = BI_00;         // JSR/BRK
                8'b0?00_1000:           alu_bi = BI_00;         // PHP/PHA
                8'b01?0_0000:           alu_bi = BI_00;         // RTS/RTI
                8'b0?10_1000:           alu_bi = BI_00;         // PLP/PLA
                8'b100?_1010:           alu_bi = BI_00;         // TXA/TXS
                8'b1001_1000:           alu_bi = BI_00;         // TYA
                8'b1010_10?0:           alu_bi = BI_00;         // TAX/TAY
                8'b1011_1010:           alu_bi = BI_00;         // TSX
                8'b1000_1000:           alu_bi = BI_FF;         // DEY 
                8'b1100_1000:           alu_bi = BI_00;         // INY 
                8'b1110_1000:           alu_bi = BI_00;         // INX
                8'b1100_1010:           alu_bi = BI_FF;         // DEX 
            endcase

        ABS0:
            casez( IR )
                8'b1011_1110:           alu_bi = BI_M;          // LDX ABS,Y 
                8'b????_1001:           alu_bi = BI_M;          // ABS,Y
                8'b???1_11??:           alu_bi = BI_M;          // ABS,X
                8'b00?0_0000:           alu_bi = BI_00;         // JSR/BRK
                8'b???0_11??:           alu_bi = BI_00;         // ABS
                8'b???1_0001:           alu_bi = BI_M;          // (ZP), Y
                8'b???0_0001:           alu_bi = BI_00;         // (ZP, X)
            endcase

        ABS1:                           alu_bi = BI_00;

        ZP0:
            casez( IR )
                8'b10?1_0110:           alu_bi = BI_M;          // LDX/STX ZP,Y
                8'b????_01??:           alu_bi = BI_M;          // all other ZP,X
                8'b???0_0001:           alu_bi = BI_M;          // (ZP,X)
                8'b???1_0001:           alu_bi = BI_00;         // (ZP), Y
            endcase

        ZP1:
            casez( IR )
                8'b???0_0001:           alu_bi = BI_M;          // (ZP,X)
                8'b???1_0001:           alu_bi = BI_00;         // (ZP), Y
            endcase

        STK0:
            casez( IR )
                8'b?1??_0???:           alu_bi = BI_00;         // RTS/RTI
                8'b?0??_0???:           alu_bi = BI_FF;         // JSR/BRK
                8'b??0?_1???:           alu_bi = BI_00;         // PHP/PHA
            endcase

        STK1:
            casez( IR )
                8'b?10?_????:           alu_bi = BI_00;         // RTI
                8'b?0??_????:           alu_bi = BI_FF;         // JSR/BRK
                8'b?11?_????:           alu_bi = BI_00;         // RTS
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           alu_bi = BI_FF;         // BRK 
                8'b?1??_????:           alu_bi = BI_00;         // RTI
            endcase

        BRA0:                           alu_bi = BI_M;          //

        BRA1:                           alu_bi = BI_00;         // 

        BRA2:                           alu_bi = BI_FF;         // 

        DATA:
            casez( IR )
                8'b110?_?110:           alu_bi = BI_FF;         // DEC M
                8'b111?_?110:           alu_bi = BI_00;         // INC M
                8'b100?_??00:           alu_bi = BI_00;         // STY
                8'b100?_??01:           alu_bi = BI_00;         // STA
                8'b100?_??10:           alu_bi = BI_00;         // STX
            endcase

        FETCH:
            casez( IR )
                8'b101?_????:           alu_bi = BI_00;         // LDA/LDX/LDY
                8'b0?10_1000:           alu_bi = BI_00;         // PLA/PLP
                8'b0?00_1000:           alu_bi = BI_FF;         // PHA/PHP
                8'b001?_??00:           alu_bi = BI_M;          // BIT
                8'b011?_??01:           alu_bi = BI_M;          // ADC
                8'b000?_??01:           alu_bi = BI_M;          // ORA
                8'b001?_??01:           alu_bi = BI_M;          // AND 
                8'b010?_??01:           alu_bi = BI_M;          // EOR 
                8'b11??_??01:           alu_bi = BI_NOTM;       // SBC/CMP
                8'b110?_??00:           alu_bi = BI_NOTM;       // CPY
                8'b111?_??00:           alu_bi = BI_NOTM;       // CPX
            endcase
    endcase
end

/*
 * ALU operation
 */

always @* begin
    alu_op = 31;
    case( state )
        DECODE:
            casez( IR )
                8'b00?0_0000:           alu_op = ALU_AI;        // JSR/BRK
                8'b01?0_0000:           alu_op = ALU_ADC;       // RTS/RTI

                8'b0?00_1000:           alu_op = ALU_AI;        // PHP/PHA
                8'b0?10_1000:           alu_op = ALU_ADC;       // PLP/PLA
                8'b1000_1000:           alu_op = ALU_ADC;       // DEY 
                8'b1001_1000:           alu_op = ALU_AI;        // TYA
                8'b1100_1000:           alu_op = ALU_ADC;       // INY 
                8'b1110_1000:           alu_op = ALU_ADC;       // INX

                8'b00??_1010:           alu_op = ALU_ROL;       // ASL/ROL A
                8'b01??_1010:           alu_op = ALU_ROR;       // LSR/ROR A
                8'b100?_1010:           alu_op = ALU_AI;        // TXA/TXS
                8'b1010_10?0:           alu_op = ALU_AI;        // TAX/TAY
                8'b1011_1010:           alu_op = ALU_AI;        // TSX
                8'b1100_1010:           alu_op = ALU_ADC;       // DEX 
            endcase

        ABS0:
            casez( IR )
                8'b1011_1110:           alu_op = ALU_ADC;       // LDX ABS,Y 
                8'b????_1001:           alu_op = ALU_ADC;       // ABS,Y
                8'b???1_11??:           alu_op = ALU_ADC;       // ABS,X
                8'b00?0_0000:           alu_op = ALU_AI;        // JSR/BRK
                8'b???0_11??:           alu_op = ALU_AI;        // ABS
                8'b???1_0001:           alu_op = ALU_ADC;       // (ZP), Y
                8'b???0_0001:           alu_op = ALU_AI;        // (ZP, X)
            endcase

        ABS1:                           alu_op = ALU_ADC;       //

        ZP0:
            casez( IR )
                8'b10?1_0110:           alu_op = ALU_ADC;       // LDX/STX ZP,Y
                8'b????_01??:           alu_op = ALU_ADC;       // all other ZP,X
                8'b???0_0001:           alu_op = ALU_ADC;       // (ZP,X)
                8'b???1_0001:           alu_op = ALU_AI;        // (ZP), Y
            endcase

        ZP1:
            casez( IR )
                8'b???0_0001:           alu_op = ALU_ADC;       // (ZP,X)
                8'b???1_0001:           alu_op = ALU_ADC;       // (ZP), Y
            endcase

        STK0:
            casez( IR )
                8'b?1??_0???:           alu_op = ALU_ADC;       // RTS/RTI
                8'b?0??_0???:           alu_op = ALU_ADC;       // JSR/BRK
                8'b??0?_1???:           alu_op = ALU_AI;        // PHP/PHA
            endcase

        STK1:
            casez( IR )
                8'b?10?_????:           alu_op = ALU_ADC;       // RTI
                8'b?0??_????:           alu_op = ALU_ADC;       // JSR/BRK
                8'b?11?_????:           alu_op = ALU_AI;        // RTS
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           alu_op = ALU_ADC;       // BRK 
                8'b?1??_????:           alu_op = ALU_AI;        // RTI
            endcase

        BRA0:                           alu_op = ALU_ADC;       //

        BRA1:                           alu_op = ALU_ADC;       // 

        BRA2:                           alu_op = ALU_ADC;       // 

        DATA:
            casez( IR )
                8'b00??_??10:           alu_op = ALU_ROL;       // ASL/ROL M
                8'b01??_??10:           alu_op = ALU_ROR;       // LSR/ROR M
                8'b110?_?110:           alu_op = ALU_ADC;       // DEC M
                8'b111?_?110:           alu_op = ALU_ADC;       // INC M
                8'b100?_??00:           alu_op = ALU_AI;        // STY
                8'b100?_??01:           alu_op = ALU_AI;        // STA
                8'b100?_??10:           alu_op = ALU_AI;        // STX
            endcase

        FETCH:
            casez( IR )
                8'b101?_????:           alu_op = ALU_AI;        // LDA/LDX/LDY
                8'b0?10_1000:           alu_op = ALU_AI;        // PLA/PLP
                8'b0?00_1000:           alu_op = ALU_ADC;       // PHA/PHP
                8'b001?_??00:           alu_op = ALU_AND;       // BIT
                8'b011?_??01:           alu_op = ALU_ADC;       // ADC
                8'b000?_??01:           alu_op = ALU_ORA;       // ORA
                8'b001?_??01:           alu_op = ALU_AND;       // AND 
                8'b010?_??01:           alu_op = ALU_EOR;       // EOR 
                8'b11??_??01:           alu_op = ALU_ADC;       // SBC/CMP
                8'b110?_??00:           alu_op = ALU_ADC;       // CPY
                8'b111?_??00:           alu_op = ALU_ADC;       // CPX
            endcase
    endcase
end

/*
 * register loads
 */

always @* begin
    alu_ld = 0;
    case( state )
        DECODE:         
            casez( IR )
                8'b1000_1010:           alu_ld = LD_A;          // TXA
                8'b0???_1010:           alu_ld = LD_A;          // ROL/ASL/ROR/LSR A
                8'b101?_1010:           alu_ld = LD_X;          // TAX/TSX
                8'b110?_1010:           alu_ld = LD_X;          // DEX
                8'b1001_1010:           alu_ld = LD_S;          // TXS
                8'b1001_1000:           alu_ld = LD_A;          // TYA
                8'b1110_1000:           alu_ld = LD_X;          // INX
                8'b1??0_1000:           alu_ld = LD_Y;          // INY/DEY/TAY
                8'b01?0_0000:           alu_ld = LD_S;          // RTS/RTI
                8'b0?10_1000:           alu_ld = LD_S;          // PLP/PLA
            endcase

        FETCH:          
            casez( IR )
                8'b0???_??01:           alu_ld = LD_A;          // ORA/AND/EOR/ADC 
                8'b101?_??01:           alu_ld = LD_A;          // LDA 
                8'b111?_??01:           alu_ld = LD_A;          // SBC
                8'b?11?_10??:           alu_ld = LD_A;          // PLA
                8'b101?_??10:           alu_ld = LD_X;          // LDX
                8'b1010_??00:           alu_ld = LD_Y;          // LDY
                8'b1011_?100:           alu_ld = LD_Y;          // LDY
                8'b0?00_1000:           alu_ld = LD_S;          // PHA/PHP
            endcase 

        STK0:
            casez( IR )
                8'b?1??_0???:           alu_ld = LD_S;          // RTS/RTI
                8'b?0??_0???:           alu_ld = LD_S;          // JSR/BRK
            endcase

        STK1:
            casez( IR )
                8'b?10?_????:           alu_ld = LD_S;          // RTI
                8'b?0??_????:           alu_ld = LD_S;          // JSR/BRK
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           alu_ld = LD_S;          // BRK 
            endcase
    endcase
end

/*
 * ALU Carry In
 */

always @* begin
    CI = 0;
    case( state )
        ZP1:
            casez( IR )
                8'b????_0001:           CI = 1;                 // (ZP,X) / (ZP),Y
            endcase

        DECODE:
            casez( IR )
                8'b0?10_1010:           CI = C;                 // ROL/ROR A
                8'b01?0_0000:           CI = 1;                 // RTS/RTI
                8'b0?10_1000:           CI = 1;                 // PLP/PLA
                8'b11?0_1000:           CI = 1;                 // INX/INY
            endcase

        FETCH:
            casez( IR )
                8'b110?_??01:           CI = 1;                 // CMP
                8'b11??_??00:           CI = 1;                 // CPX/CPY
                8'b?11?_??01:           CI = C;                 // ADC/SBC
            endcase

        STK0:
            casez( IR )
                8'b?1??_0???:           CI = 1;                 // RTS/RTI
            endcase

        STK1:
            casez( IR )
                8'b?10?_????:           CI = 1;                 // RTI
            endcase

        ABS1:                           CI = 1;                 // add carry from LSB

        BRA1:                           CI = 1;                 // jump forward across page

        DATA:
            casez( IR )
                8'b0?1?_?110:           CI = C;                 // ROL/ROR MEM 
                8'b111?_?110:           CI = 1;                 // INC M
            endcase
    endcase
end

/*
 * ALU AI input
 */

always @*
    case( alu_ai )
        AI_PCL:                         AI = PCL;
        AI_PCH:                         AI = PCH;
        AI_A:                           AI = A;
        AI_X:                           AI = X;
        AI_Y:                           AI = Y;
        AI_M:                           AI = M;
        AI_S:                           AI = S;
        AI_ZZZ:                         AI = 8'h55;
    endcase

/*
 * ALU BI input
 */

always @*
    case( alu_bi )
        BI_00:                          BI = 8'h00;
        BI_FF:                          BI = 8'hff;
        BI_M:                           BI = M;
        BI_NOTM:                        BI = ~M;
        BI_ZZZ:                         BI = 8'ha5;
    endcase

/*
 * ALU operation
 */

/*
 * split nibble addition to get half carry bit
 */
wire HC;
wire [4:0] LSD = AI[3:0] + BI[3:0] + CI;        // least significant digit 
wire [4:0] MSD = AI[7:4] + BI[7:4] + HC;        // most significant digit

/*
 * upper digit gets carry from lower digit, but only 
 * if 'alu_pass_hc' control signal is set. Otherwise
 * the 2 digits are separate.
 */

assign HC = LSD[4] & alu_pass_hc;

always @*
    case( alu_op )
        ALU_AI:                         ALU = AI;
        ALU_ADC:                        ALU = { MSD[3:0], LSD[3:0] };
        ALU_ROL:                        ALU = { AI[6:0], CI };
        ALU_ROR:                        ALU = { CI, AI[7:1] };
        ALU_ORA:                        ALU = AI | BI;
        ALU_EOR:                        ALU = AI ^ BI;
        ALU_AND:                        ALU = AI & BI;
    default:                            ALU = 8'h55;
    endcase

assign NO = ALU[7];
assign ZO = ALU[7:0] == 0;
assign VO = NO ^ CO ^ A[7] ^ BI[7];

always @*
    if( alu_op == ALU_ROL )             CO = AI[7];
    else if( alu_op == ALU_ROR )        CO = AI[0];
    else                                CO = MSD[4];

/*
 * register load
 */

always @(posedge clk)
    case( alu_ld )
        LD_A:                           A <= ALU;
        LD_X:                           X <= ALU;
        LD_Y:                           Y <= ALU;
        LD_S:                           S <= ALU;
    endcase

/*
 * C flag
 */
always @(posedge clk)
    case( state )
        STK0:
            casez( IR )
                8'b?01?_1???:           C <= DB[0];             // PLP
                8'b?10?_0???:           C <= DB[0];             // RTI
            endcase

        DATA:
            casez( IR )
                8'b0???_?110: if( !WE ) C <= CO;                // ROL M 
            endcase

        FETCH:
            casez( IR )
                8'b011?_??01:           C <= CO;                // ADC
                8'b11??_??01:           C <= CO;                // CMP/SBC
                8'b11?0_??00:           C <= CO;                // CPX/CPY #
            endcase

        DECODE:
            casez( IR )
                8'b0???_1010:           C <= CO;
                8'b00?1_1000:           C <= IR[5];             // CLC/SEC
            endcase
    endcase

/*
 * N flag
 */
always @(posedge clk)
    case( state )
        DATA:
            casez( IR )
                8'b0???_?110: if( !WE ) N <= NO;                // ROL/ROR/ASL/LSR M 
                8'b11??_?110: if( !WE ) N <= NO;                // INC/DEC M 
                8'b0010_?100:           N <= DB[7];             // BIT
            endcase

        DECODE:
            casez( IR )
                8'b1?00_10?0:           N <= NO;                // DEY/TXA/INY/DEX
                8'b10?0_10?0:           N <= NO;                // DEY/TXA/TAY/TAX
                8'b100?_1000:           N <= NO;                // DEY/TYA
                8'b101?_1010:           N <= NO;                // TAX/TSX
                8'b1??0_1000:           N <= NO;                // DEY/TAY/INY/INX
                8'b0???_1010:           N <= NO;                // ROL A
            endcase

        STK0:
            casez( IR )
                8'b?01?_1???:           N <= DB[7];             // PLP
                8'b?10?_0???:           N <= DB[7];             // RTI
            endcase

        FETCH:
            casez( IR )
                8'b0110_1000:           N <= NO;                // PLA
                8'b0???_??01:           N <= NO;                // ADC
                8'b1010_????:           N <= NO;                // LDA/LDX/LDY
                8'b101?_?1??:           N <= NO;                // LDA/LDX/LDY
                8'b11?0_??00:           N <= NO;                // CPX/CPY
                8'b11??_??01:           N <= NO;                // SBC/CMP
                8'b101?_??01:           N <= NO;                // LDA
            endcase
    endcase

/*
 * Z flag
 */
always @(posedge clk)
    case( state )
        DATA:
            casez( IR )
                8'b0???_?110: if( !WE ) Z <= ZO;                // ROL/ROR/ASL/LSR M 
                8'b11??_?110: if( !WE ) Z <= ZO;                // INC/DEC M 
            endcase

        DECODE:
            casez( IR )
                8'b1?00_10?0:           Z <= ZO;                // DEY/TXA/INY/DEX
                8'b10?0_10?0:           Z <= ZO;                // DEY/TXA/TAY/TAX
                8'b100?_1000:           Z <= ZO;                // DEY/TYA
                8'b101?_1010:           Z <= ZO;                // TAX/TSX
                8'b1??0_1000:           Z <= ZO;                // DEY/TAY/INY/INX
                8'b0???_1010:           Z <= ZO;                // ROL A
            endcase

        STK0:
            casez( IR )
                8'b?01?_1???:           Z <= DB[1];             // PLP
                8'b?10?_0???:           Z <= DB[1];             // RTI
            endcase

        FETCH:
            casez( IR )
                8'b0110_1000:           Z <= ZO;                // PLA
                8'b0010_?100:           Z <= ZO;                // BIT
                8'b0???_??01:           Z <= ZO;                // ADC
                
                8'b1010_????:           Z <= ZO;                // LDA/LDX/LDY
                8'b101?_?1??:           Z <= ZO;                // LDA/LDX/LDY
                8'b11?0_??00:           Z <= ZO;                // CPX/CPY
                8'b11??_??01:           Z <= ZO;                // SBC/CMP
                8'b101?_??01:           Z <= ZO;                // LDA
            endcase
    endcase

/*
 * D flag
 */

always @(posedge clk)
    case( state )
        STK0:
            casez( IR )
                8'b?01?_1???:           D <= DB[3];             // PLP
                8'b?10?_0???:           D <= DB[3];             // RTI
            endcase

        DECODE:
            casez( IR )
                8'b11?1_1000:           D <= IR[5];             // CLD/SED
            endcase
    endcase

/*
 * I flag
 */

always @(posedge clk)
    case( state )
        STK0:
            casez( IR )
                8'b?01?_1???:           I <= DB[2];             // PLP
                8'b?10?_0???:           I <= DB[2];             // RTI
            endcase

        STK2:
            casez( IR )
                8'b0000_0000:           I <= 1;                 // BRK/IRQ
            endcase

        DECODE:
            casez( IR )
                8'b01?1_1000:           I <= IR[5];             // CLI/SEI
            endcase
    endcase

/*
 * V flag
 */

always @(posedge clk)
    case( state )
        DATA:
            casez( IR )
                8'b0010_?100:           V <= DB[6];             // BIT
            endcase

        STK0:
            casez( IR )
                8'b?01?_1???:           V <= DB[6];             // PLP
                8'b?10?_0???:           V <= DB[6];             // RTI
            endcase

        FETCH:
            casez( IR )
                8'b?11?_??01:           V <= VO;                // ADC/SBC
            endcase


        DECODE:
            casez( IR )
                8'b1011_1000:           V <= 0;                 // CLV
            endcase
    endcase


/*
 * state machine
 */
always @(posedge clk)
    case( state )
        DECODE: 
            casez( IR )
                8'b???0_1001:           state <= FETCH;         // even col 9 ACC #IMM
                8'b???1_1001:           state <= ABS0;          // odd col 9 ABS,Y
                8'b1??0_00?0:           state <= FETCH;         // LDX/LDY/CPX/CPY #IMM
                8'b???0_01??:           state <= DATA;          // ZP, not indexed
                8'b????_0001:           state <= ZP0;           // (ZP,X) or (ZP),Y
                8'b???1_01??:           state <= ZP0;           // ZP, indexed 
                8'b????_11??:           state <= ABS0;          // cols CDE (ABS, ABS,X)
                8'b0??0_?000:           state <= STK0;          // 
                8'b???1_0000:                                   // branches
                    if( cond_true )     state <= BRA0;          // take branch
                    else                state <= FETCH;         //  
            endcase

        DATA :  if( !rmw )              state <= FETCH;

        FETCH:                          state <= DECODE;

        ABS0 :  if( CO )                state <= ABS1;
                else if( ind )          state <= IND0;
                else if( jmp )          state <= FETCH;
                else                    state <= DATA;

        ABS1 :                          state <= DATA;

        ZP0  :  if( ~IR[2] )            state <= ZP1;
                else                    state <= DATA;

        ZP1  :                          state <= ABS0;

        IND0 :                          state <= ABS0;

        STK0 :  
            casez( IR )
                8'b????_1???:           state <= FETCH;
                8'b????_0???:           state <= STK1;
            endcase

        STK1 :  
            casez( IR)
                8'b??0?_????:           state <= STK2;          // RTI/BRK
                8'b?11?_????:           state <= RTS0;          // RTS
                8'b?01?_????:           state <= ABS0;          // JSR
            endcase

        STK2 :
            casez( IR)
                8'b?0??_????:           state <= IND0;          // BRK
                8'b?1??_????:           state <= FETCH;         // RTI
            endcase
        
        RTS0 :                          state <= FETCH;

        BRA0 : if( CO ^ M[7] )          state <= CO ? BRA1 : BRA2;
               else                     state <= FETCH;         // 
        BRA1 :                          state <= FETCH;
        BRA2 :                          state <= FETCH;
    endcase

/*
 * extra state bits
 */

/*
 * rmw
 */

always @(posedge clk)
        if( state == DECODE )
            casez( IR )
                8'b0???_?110:           rmw <= 1;               // shift memory
                8'b11??_?110:           rmw <= 1;               // INC/DEC
                default:                rmw <= 0;
            endcase
        else if( state == DATA )        rmw <= 0;

/*
 * ind flag, set when doing indirection
 */
always @(posedge clk)
    case( state )
        DECODE:                         ind <= ( IR == 8'h6c );
        default:                        ind <= 0;
    endcase

always @*
    casez( IR[7:4] )
        4'b000?:                        cond_true = ~N;     // BPL
        4'b001?:                        cond_true = N;      // BMI
        4'b010?:                        cond_true = ~V;     // BVC
        4'b011?:                        cond_true = V;      // BVS
        4'b1000:                        cond_true = 1;      // BRA
        4'b1001:                        cond_true = ~C;     // BCC
        4'b101?:                        cond_true = C;      // BCS
        4'b110?:                        cond_true = ~Z;     // BNE
        4'b111?:                        cond_true = Z;      // BEQ
    endcase
/*
 * easy to read names in simulator output
 */
reg [8*6-1:0] statename;

always @*
    case( state )
            FETCH:  statename = "FETCH";
            DECODE: statename = "DECODE";
            DATA:   statename = "DATA";
            ABS0:   statename = "ABS0";
            ABS1:   statename = "ABS1";
            ZP0:    statename = "ZP0";
            ZP1:    statename = "ZP1";
            IND0:   statename = "IND0";
            STK0:   statename = "STK0";
            STK1:   statename = "STK1";
            STK2:   statename = "STK2";
            BRA0:   statename = "BRA0";
            BRA1:   statename = "BRA1";
            BRA2:   statename = "BRA2";
            RTS0:   statename = "RTS0";
    endcase

integer cycle;

always @( posedge clk )
    cycle <= cycle + 1;

always @( posedge clk )
      if( cycle[19:0] == 0 )
      $display( "%d %8s AB:%h DB:%h DO:%h PC:%h IR:%h WE:%d M:%02x S:%02x A:%02x X:%02x Y:%02x AI:%h BI:%h CI:%d OP:%d ALU:%h CNZDIV: %d%d%d%d%d%d (%d)", 
        cycle,
        statename, AB, DB, DO, PC, IR, WE, M, S, A, X, Y, 
        AI, BI, CI, alu_op, ALU, C, N, Z, D, I, V, cond_true  );

endmodule
