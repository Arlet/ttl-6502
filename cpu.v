/*
 * verilog model of 6502 CPU.
 *
 * This file is used as a model for logic gate
 * level development of 6502 core. As design
 * evolves, different parts will be rewritten
 * to match the hardware design.
 *
 * initial version passes Klaus Dormann's test suite
 * but does not yet support RST/IRQ/RDY/NMI.
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 *
 */

module cpu( clk, RST, AB, DB, WE, IRQ, NMI, RDY, Debug );

`include "states.i"

input clk;              // CPU clock
input RST;              // RST signal
output [15:0] AB;       // address bus
inout [7:0] DB;         // data bus
output WE;              // write enable
input IRQ;              // interrupt request
input NMI;              // non-maskable interrupt request
input RDY;              // Ready signal. Pauses CPU when RDY=0
input Debug;            // debug input, set when running very long test suite

reg N, V, D, B, I = 1, Z, C;   // flags

reg [7:0] S = 8'hff;
reg [7:0] X = 2;
reg [7:0] Y = 3;
reg [7:0] A = 8'h41;

/*
 * M register (temporary memory hold)
 */
reg m_ld;               // enables loading M
reg m_sel_adj;          // if set, load ADJ rather than DB
reg [7:0] M;

/*
 * Program Counter
 */
reg pc_ld;              // enables loading PC
reg [15:0] PC;

// don't have reset yet, so init AB on code start
reg [7:0] ABH = 8'h04;
reg [7:0] ABL = 8'h00;

wire [15:0] AB = { ABH, ABL };
reg [7:0] ALU;

/*
 * BCD adjust term
 */
reg [3:0] ADJL;
reg [3:0] ADJH;
wire [7:0] ADJ = { ADJH, ADJL };

/*
 * AB incrementer. The 'inc_ab' signal indicates whether
 * to increment AB or not. We add it to AB to create ABI.
 */

reg inc_ab;
wire [15:0] ABI = AB + inc_ab;

/*
 * select signals for ADH/ADL bus
 */

wire [2:0] sel_adh;
wire [2:0] sel_adl;

/*
 * Program counter/Address Bus high and low parts
 */
wire [7:0] PCH = PC[15:8];
wire [7:0] PCL = PC[7:0];
reg [7:0] ADH;              // internal bus for next ABH
reg [7:0] ADL;              // internal bus for next ABL

reg WE = 1;                 // write enable (active low)
reg [7:0] DO;               // data out
reg [7:0] AI;               // alu input A
reg CI;                     // alu input carry

/*
 * Interrupts and reset
 */

wire take_irq = IRQ & ~I;   // set if taking IRQ on next DECODE
reg in_irq;                 // set if currently in IRQ handler
assign B = ~in_irq;         // B flag

wire take_rst = RST;
reg in_rst;

/*
 * Instruction Register
 */
reg ir_ld;                  // enables loading IR from DB
reg [7:0] IR;

/*
 * processor status flags
 */
wire [7:0] P = { N, V, 1'b1, B, D, I, Z, C };
reg cond_true;

wire NO;                    // ALU N flag output
wire ZO;                    // ALU Z flag output
wire VO;                    // ALU V flag output
wire CO;                    // ALU C flag output
wire BCD_C = C | CO;        // BCD carry

reg [1:0] p_sel;            // selects flag source
reg z_ld;                   // enables loading Z flag
reg i_ld;                   // enables loading I flag
reg d_ld;                   // enables loading D flag
reg n_ld;                   // enables loading N flag
reg c_ld;                   // enables loading C flag
reg v_ld;                   // enables loading V flag

/*
 * ALU inputs/outputs/operation control signals
 */
reg [2:0] alu_ai;
reg [2:0] alu_bi;
reg [4:0] alu_op;
reg [2:0] alu_ld;
reg mem_bi;
reg inv_bi;

wire adj_lsd;
wire adj_msd;

alu alu(
    .AI(AI),
    .MI(M),
    .CI(CI),
    .OUT(ALU),
    .mem_bi(mem_bi),
    .inv_bi(inv_bi),
    .op(alu_op),
    .N(NO),
    .Z(ZO),
    .C(CO),
    .V(VO),
    .adj_lsd(adj_lsd),
    .adj_msd(adj_msd) );

/*
 * databus
 */
assign DB = WE ? 8'hzz : DO;



/*
 * some special instructions
 */
reg ind;
wire brk = (IR == 8'h00);
wire jmp  = (IR == 8'h4c || IR == 8'h6c || IR == 8'h20 || IR == 8'h00);
reg rmw;
reg bcd;

// processor state
reg [4:0] state = FETCH;

/*
 * Address Bus logic
 *
 */

/*
 * sel_adl determines who will be writing to ADL bus. Default is ABI, which is
 * current address, possibly incremented.
 */
always @* begin
    sel_adl = ADL_ABI;
    case( state )
        DECODE:
            casez( IR )
                8'b0??0_?000:           sel_adl = ADL_ALU;          // stack instruction
                8'b????_0001:           sel_adl = ADL_DB;           // (ZP,X) or (ZP),Y
                8'b????_01??:           sel_adl = ADL_DB;           // ZP (possibly indexed)
            endcase

        DATA:  if( !rmw )               sel_adl = ADL_PC;

        ABS0:                           sel_adl = ADL_ALU;

        ZP0:                            sel_adl = ADL_ALU;

        ZP1:                            sel_adl = ADL_ALU;

        STK0:  if( IR[3] )              sel_adl = ADL_PC;           // only for PHA,PHP
               else                     sel_adl = ADL_ALU;

        STK1:
            casez( IR )
                8'b??0?_????:           sel_adl = ADL_ALU;          // BRK/RTI
                8'b?01?_????:           sel_adl = ADL_PC;           // JSR
                8'b?11?_????:           sel_adl = ADL_ALU;          // RTS
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           sel_adl = in_rst ? ADL_RST : ADL_BRK;
                8'b?1??_????:           sel_adl = ADL_ALU;          // RTI
            endcase
        BRA0:                           sel_adl = ADL_ALU;          // branch taken
    endcase
end

/*
 * sel_adh determines who will be writing to ADH bus. Default is ABI, which is
 * current address, possibly incremented.
 */
always @* begin
    sel_adh = ADH_ABI;
    case( state )
        DECODE:
            casez( IR )
                8'b0??0_?000:           sel_adh = ADH_SP;           // stack instruction
                8'b????_0001:           sel_adh = ADH_ZP;           // (ZP,X) or (ZP),Y
                8'b????_01??:           sel_adh = ADH_ZP;           // ZP (possibly indexed)
            endcase

        DATA:  if( !rmw )               sel_adh = ADH_PC;

        ABS0:                           sel_adh = ADH_DB;

        ABS1:                           sel_adh = ADH_ALU;

        ZP0:                            sel_adh = ADH_ZP;

        ZP1:                            sel_adh = ADH_ZP;

        STK0:  if( IR[3] )              sel_adh = ADH_PC;           // only for PHA,PHP
               else                     sel_adh = ADH_SP;
        STK1:
            casez( IR )
                8'b??0?_????:           sel_adh = ADH_SP;           // BRK/RTI
                8'b?01?_????:           sel_adh = ADH_PC;           // JSR
                8'b?11?_????:           sel_adh = ADH_DB;           // RTS
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           sel_adh = ADH_FF;           // BRK
                8'b?1??_????:           sel_adh = ADH_DB;           // RTI
            endcase
        BRA0:                           sel_adh = ADH_PC;           // branch taken
        BRA1:                           sel_adh = ADH_ALU;          // page crossing forward
        BRA2:                           sel_adh = ADH_ALU;          // page crossing backward
    endcase
end

/*
 * address incrementer
 *
 * if 'inc_ab' is set, calculate AB + 1, otherwise keep current address
 */
always @* begin
    inc_ab = 0;
    case( state )
        DECODE:
            casez( IR )
                8'b0??0_1000:           inc_ab = 0;                 // PHA/PLA/PHP/PLP
                default:                inc_ab = !take_irq;
            endcase
        BCD0:                           inc_ab = 1;
        RTS0:                           inc_ab = 1;
        IND0:                           inc_ab = 1;
        FETCH:                          inc_ab = !bcd & !take_irq;
        ABS0:                           inc_ab = 1;
    endcase
end

/*
 * Internal Address Bus mux
 */

always @*
    case( sel_adh )
        ADH_ABI :                       ADH = ABI[15:8];
        ADH_ALU :                       ADH = ALU;
        ADH_DB  :                       ADH = DB;
        ADH_PC  :                       ADH = PCH;
        ADH_FF  :                       ADH = 8'hFF;
        ADH_SP  :                       ADH = 8'h01;
        ADH_ZP  :                       ADH = 8'h00;
        default:                        ADH = 8'h55;                // to catch mistakes
    endcase

always @*
    case( sel_adl )
        ADL_ABI :						ADL = ABI[7:0];
        ADL_ALU :						ADL = ALU;
        ADL_DB  :						ADL = DB;
        ADL_PC  :						ADL = PCL;
        ADL_NMI :                       ADL = 8'hFA;                //
        ADL_RST :						ADL = 8'hFC;                //
        ADL_BRK :						ADL = 8'hFE;                //
    endcase

/*
 * write external address
 */
always @(posedge clk)
    ABH <= ADH;

always @(posedge clk)
    ABL <= ADL;

/*
 * Program Counter update
 *
 * Either take PC from ABI or leave old value.
 */

always @* begin
    pc_ld = 0;
    case( state )
        DECODE:                         pc_ld = 1;
        BCD0:                           pc_ld = 1;
        RTS0:                           pc_ld = 1;
        IND0:                           pc_ld = 1;
        FETCH:                          pc_ld = 1;
        ABS0:                           pc_ld = IR[3];               // only for true ABS (not ABS0 as part of ZP,Y)
    endcase
end

always @(posedge clk)
    if( pc_ld )
        PC <= ABI;

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
 * Adjustments for lower and upper nibbles. The adjustment is
 * 6 for both add and subtract, but in case of subtract the value
 * is inverted when it enters BI.
 */
always @*
    if( bcd & adj_lsd )                 ADJL = 6;
    else                                ADJL = 0;

always @*
    if( bcd & adj_msd )                 ADJH = 6;
    else                                ADJH = 0;

/*
 * M register holds recent value from DB as ALU input
 */
always @* begin
    m_ld = 0;
    case( state )
        ABS0:                           m_ld = 1;

        STK0:
            casez( IR )
                8'b??1?_1???:           m_ld = 1;                   // PLP/PLA
                8'b?1??_0???:           m_ld = 1;                   // RTS/RTI
            endcase

        ZP1, IND0, DECODE:              m_ld = 1;

        STK1, STK2:
            casez( IR )
                8'b?1??_????:           m_ld = 1;                   // RTS/RTI
            endcase

        DATA:                           m_ld = 1;                   //

        FETCH:                          m_ld = 1;
    endcase
end

/*
 * m_sel_adj sets source of M
 */
always @*
    case( state )
        FETCH:                          m_sel_adj = 1;
        default:                        m_sel_adj = 0;
    endcase

always @(posedge clk)
    if( m_ld )
        M <= m_sel_adj ? ADJ : DB;

/*
 * Instruction Register. Normally we update IR in FETCH state, but if we have to
 * do a BCD adjustment, the FETCH state is followed by extra BCD0 state, so the
 * IR is not updated until then.
 *
 * Also, in the DECODE state we load next IR for all single cycle instructions.
 */

always @* begin
    ir_ld = in_rst;
    case( state )
        FETCH:                          ir_ld = !bcd;

        BCD0:                           ir_ld = 1;

        DECODE:
            casez( IR )
                8'b???1_10?0:           ir_ld = 1;                  // odd column 8/A
                8'b1???_10?0:           ir_ld = 1;                  // top column 8/A
                8'b????_1010:           ir_ld = 1;                  // column A
            endcase
    endcase
end

always @(posedge clk)
    if( ir_ld & take_irq )              in_irq <= 1;                //
    else if( state == FETCH )           in_irq <= 0;                //

always @(posedge clk)
    if( ir_ld & take_rst )              in_rst <= 1;                //
    else if( state == FETCH )           in_rst <= 0;                //

always @(posedge clk)
    if( ir_ld )
        IR <= (take_irq|in_rst) ? 8'h00 : DB;

/*
 * ALU AI input
 */

always @* begin
    alu_ai = AI_ZZZ;
    case( state )
        DECODE:
            casez( IR )
                8'b0??0_?000:           alu_ai = AI_S;              // JSR/BRK/RTS/RTI/PHA/PHP/PLP/PLA
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
                8'b10?1_?110:           alu_ai = AI_Y;          // LDX ABS,Y
                8'b???1_?001:           alu_ai = AI_Y;          // ABS,Y
                8'b???1_11??:           alu_ai = AI_X;          // ABS,X
                8'b???0_????:           alu_ai = AI_M;          // ABS/(ZP,X)
            endcase

        ZP0:
            casez( IR )
                8'b10?1_?110:           alu_ai = AI_Y;          // LDX/STX ZP,Y
                8'b????_01??:           alu_ai = AI_X;          // all other ZP,X
                8'b???0_0001:           alu_ai = AI_X;          // (ZP,X)
                8'b???1_0001:           alu_ai = AI_M;          // (ZP),Y
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

        ABS1:                           alu_ai = AI_M;          //
        BCD0:                           alu_ai = AI_A;          // Decimal adjust
        BRA0:                           alu_ai = AI_PCL;        //
        BRA1:                           alu_ai = AI_PCH;        //
        BRA2:                           alu_ai = AI_PCH;        //
    endcase
end

/*
 * ALU mem_bi signal
 */

always @* begin
    mem_bi = 0;
    case( state )

        ABS0:
            casez( IR )
                8'b1011_1110:           mem_bi = 1;         // LDX ABS,Y
                8'b????_1001:           mem_bi = 1;         // ABS,Y
                8'b???1_11??:           mem_bi = 1;         // ABS,X
                8'b???0_11??:           mem_bi = 0;         // ABS
                8'b00?0_0000:           mem_bi = 0;         // JSR/BRK
                8'b???0_0001:           mem_bi = 0;         // (ZP, X)
                8'b???1_0001:           mem_bi = 1;         // (ZP), Y
            endcase

        ZP0:
            casez( IR )
                8'b10?1_0110:           mem_bi = 1;         // LDX/STX ZP,Y
                8'b????_01??:           mem_bi = 1;         // all other ZP,X
                8'b???0_0001:           mem_bi = 1;         // (ZP,X)
                8'b???1_0001:           mem_bi = 0;         // (ZP), Y
            endcase

        ZP1:
            casez( IR )
                8'b???0_0001:           mem_bi = 1;         // (ZP,X)
                8'b???1_0001:           mem_bi = 0;         // (ZP), Y
            endcase

        FETCH:
            casez( IR )
                8'b101?_????:           mem_bi = 0;         // LDA/LDX/LDY
                8'b0??0_1000:           mem_bi = 0;         // PLA/PLP/PHA/PHP
                8'b001?_??00:           mem_bi = 1;         // BIT
                8'b0???_??01:           mem_bi = 1;         // ORA/AND/EOR/ADC
                8'b11??_??01:           mem_bi = 1;         // SBC/CMP
                8'b11??_??00:           mem_bi = 1;         // CPX/CPY
            endcase

        DECODE:                         mem_bi = 0;
        ABS1:                           mem_bi = 0;
        STK0:                           mem_bi = 0;
        STK1:                           mem_bi = 0;
        STK2:                           mem_bi = 0;
        BRA0:                           mem_bi = 1;         //
        BRA1:                           mem_bi = 0;         //
        BRA2:                           mem_bi = 0;         //
        DATA:                           mem_bi = 0;
        BCD0:                           mem_bi = 1;

    endcase
end

/*
 * ALU inv_bi signal
 */
always @* begin
    inv_bi = 0;
    case( state )
        DECODE:
            casez( IR )
                8'b01?0_0000:           inv_bi = 0;             // RTS/RTI
                8'b0?10_1000:           inv_bi = 0;             // PLP/PLA
                8'b1000_1000:           inv_bi = 1;             // DEY
                8'b11?0_1000:           inv_bi = 0;             // INX/INY
                8'b1100_1010:           inv_bi = 1;             // DEX
            endcase

        STK0:
            casez( IR )
                8'b?1??_0???:           inv_bi = 0;             // RTS/RTI
                8'b?0??_0???:           inv_bi = 1;             // JSR/BRK
                8'b??0?_1???:           inv_bi = 0;             // PHP/PHA
            endcase

        STK1:
            casez( IR )
                8'b?10?_????:           inv_bi = 0;             // RTI
                8'b?0??_????:           inv_bi = 1;             // JSR/BRK
                8'b?11?_????:           inv_bi = 0;             // RTS
            endcase

        STK2:
            casez( IR )
                8'b?0??_????:           inv_bi = 1;             // BRK
                8'b?1??_????:           inv_bi = 0;             // RTI
            endcase

        DATA:
            casez( IR )
                8'b110?_?110:           inv_bi = 1;             // DEC M
                8'b111?_?110:           inv_bi = 0;             // INC M
                8'b100?_???0:           inv_bi = 0;             // STX/STY
                8'b100?_??01:           inv_bi = 0;             // STA
            endcase

        FETCH:
            casez( IR )
                8'b101?_????:           inv_bi = 0;             // LDA/LDX/LDY
                8'b0?10_1000:           inv_bi = 0;             // PLA/PLP
                8'b0?00_1000:           inv_bi = 1;             // PHA/PHP
                8'b001?_??00:           inv_bi = 0;             // BIT
                8'b0???_??01:           inv_bi = 0;             // ORA/AND/EOR/ADC
                8'b11??_??0?:           inv_bi = 1;             // SBC/CMP/CPX/CPY
            endcase

        BCD0:
            casez( IR )
                8'b011?_??01:           inv_bi = 0;             // BCD ADC
                8'b111?_??01:           inv_bi = 1;             // BCD SBC
            endcase

        ABS0:                           inv_bi = 0;             //
        ABS1:                           inv_bi = 0;             //
        ZP0:                            inv_bi = 0;             //
        ZP1:                            inv_bi = 0;             //
        BRA0:                           inv_bi = 0;             //
        BRA1:                           inv_bi = 0;             //
        BRA2:                           inv_bi = 1;             //

    endcase
end

/*
 * ALU operation
 */

always @* begin
    alu_op = ALU_ADC;
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

        BCD0:                           alu_ld = LD_A;          //
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

        BCD0:
            casez( IR )
                8'b011?_??01:           CI = 0;                 // BCD ADC
                8'b111?_??01:           CI = 1;                 // BCD SBC
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
 * flag source select
 */
always @* begin
    p_sel = 2'b00;
    case( state )
        STK0:
            casez( IR )
                8'b?01?_1???:           p_sel = 2'b01;              // PLP
                8'b?10?_0???:           p_sel = 2'b01;              // RTI
            endcase

        STK2:
            casez( IR )
                8'b0000_0000:           p_sel = 2'b00;              // BRK/IRQ
            endcase

        DATA:
            casez( IR )
                8'b0???_?110:           p_sel = 2'b00;              // ROL M
                8'b11??_?110:           p_sel = 2'b00;              // INC/DEC M
                8'b0010_?100:           p_sel = 2'b01;              // BIT NV flags
            endcase

        FETCH:
            casez( IR )
                8'b0110_1000:           p_sel = 2'b00;              // PLA
                8'b011?_??01:           p_sel = 2'b00;              // ADC
                8'b1010_????:           p_sel = 2'b00;              // LDA/LDX/LDY
                8'b101?_?1??:           p_sel = 2'b00;              // LDA/LDX/LDY
                8'b101?_??01:           p_sel = 2'b00;              // LDA
                8'b11?0_??00:           p_sel = 2'b00;              // CPX/CPY
                8'b11?0_??00:           p_sel = 2'b00;              // CPX/CPY #
                8'b11??_??01:           p_sel = 2'b00;              // SBC/CMP
                8'b11??_??01:           p_sel = 2'b00;              // CMP/SBC
            endcase

        DECODE:
            casez( IR )
                8'b00?1_1000:           p_sel = 2'b11;              // CLC/SEC
                8'b11?1_1000:           p_sel = 2'bx0;              // CLD/SED
                8'b01?1_1000:           p_sel = 2'bx0;              // CLI/SEI
                8'b1011_1000:           p_sel = 2'b1x;              // CLV
                8'b0???_1010:           p_sel = 2'b00;              // ROL A
                8'b0???_1010:           p_sel = 2'b00;              // ROL A
                8'b100?_1000:           p_sel = 2'b00;              // DEY/TYA
                8'b101?_1010:           p_sel = 2'b00;              // TAX/TSX
                8'b10?0_10?0:           p_sel = 2'b00;              // DEY/TXA/TAY/TAX
                8'b1?00_10?0:           p_sel = 2'b00;              // DEY/TXA/INY/DEX
                8'b1??0_1000:           p_sel = 2'b00;              // DEY/TAY/INY/INX
            endcase

        BCD0:                           p_sel = 2'b10;              // BCD ADD
    endcase
end

/*
 * C flag load enable
 */
always @* begin
    c_ld = 0;
    case( state )
        STK0:
            casez( IR )
                8'b?01?_1???:           c_ld = 1;             // PLP
                8'b?10?_0???:           c_ld = 1;             // RTI
            endcase

        DATA:
            casez( IR )
                8'b0???_?110:           c_ld = !WE;           // ROL M
            endcase

        FETCH:
            casez( IR )
                8'b011?_??01:           c_ld = 1;             // ADC
                8'b11??_??01:           c_ld = 1;             // CMP/SBC
                8'b11?0_??00:           c_ld = 1;             // CPX/CPY #
            endcase

        DECODE:
            casez( IR )
                8'b0???_1010:           c_ld = 1;
                8'b00?1_1000:           c_ld = 1;             // CLC/SEC
            endcase

        BCD0:
            casez( IR )
                8'b011?_??01:           c_ld = 1;            // add
            endcase

    endcase
end

always @* begin
    n_ld = 0;
    case( state )
        DATA:
            casez( IR )
                8'b0???_?110:           n_ld = !WE;             // ROL/ROR/ASL/LSR M
                8'b11??_?110:           n_ld = !WE;             // INC/DEC M
                8'b0010_?100:           n_ld = 1;               // BIT
            endcase

        DECODE:
            casez( IR )
                8'b1?00_10?0:           n_ld = 1;               // DEY/TXA/INY/DEX
                8'b10?0_10?0:           n_ld = 1;               // DEY/TXA/TAY/TAX
                8'b100?_1000:           n_ld = 1;               // DEY/TYA
                8'b101?_1010:           n_ld = 1;               // TAX/TSX
                8'b1??0_1000:           n_ld = 1;               // DEY/TAY/INY/INX
                8'b0???_1010:           n_ld = 1;               // ROL A
            endcase

        STK0:
            casez( IR )
                8'b?01?_1???:           n_ld = 1;               // PLP
                8'b?10?_0???:           n_ld = 1;               // RTI
            endcase

        FETCH:
            casez( IR )
                8'b0110_1000:           n_ld = 1;               // PLA
                8'b0???_??01:           n_ld = 1;               // ADC
                8'b1010_????:           n_ld = 1;               // LDA/LDX/LDY
                8'b101?_?1??:           n_ld = 1;               // LDA/LDX/LDY
                8'b11?0_??00:           n_ld = 1;               // CPX/CPY
                8'b11??_??01:           n_ld = 1;               // SBC/CMP
                8'b101?_??01:           n_ld = 1;               // LDA
            endcase

        BCD0:                           n_ld = 1;               // decimal Z after correction
    endcase
end

/*
 * Z flag
 */
always @* begin
    z_ld = 0;
    case( state )
        DATA:
            casez( IR )
                8'b0???_?110:           z_ld = !WE;             // ROL/ROR/ASL/LSR M
                8'b11??_?110:           z_ld = !WE;             // INC/DEC M
            endcase

        DECODE:
            casez( IR )
                8'b1?00_10?0:           z_ld = 1;               // DEY/TXA/INY/DEX
                8'b10?0_10?0:           z_ld = 1;               // DEY/TXA/TAY/TAX
                8'b100?_1000:           z_ld = 1;               // DEY/TYA
                8'b101?_1010:           z_ld = 1;               // TAX/TSX
                8'b1??0_1000:           z_ld = 1;               // DEY/TAY/INY/INX
                8'b0???_1010:           z_ld = 1;               // ROL A
            endcase

        STK0:
            casez( IR )
                8'b?01?_1???:           z_ld = 1;               // PLP
                8'b?10?_0???:           z_ld = 1;               // RTI
            endcase

        FETCH:
            casez( IR )
                8'b0110_1000:           z_ld = 1;               // PLA
                8'b0010_?100:           z_ld = 1;               // BIT
                8'b0???_??01:           z_ld = 1;               // ADC

                8'b1010_????:           z_ld = 1;               // LDA/LDX/LDY
                8'b101?_?1??:           z_ld = 1;               // LDA/LDX/LDY
                8'b11?0_??00:           z_ld = 1;               // CPX/CPY
                8'b11??_??01:           z_ld = 1;               // SBC/CMP
                8'b101?_??01:           z_ld = 1;               // LDA
            endcase

        BCD0:                           z_ld = 1;               // decimal Z after correction
    endcase
end

/*
 * D flag
 */

always @* begin
    d_ld = 0;
    case( state )
        STK0:
            casez( IR )
                8'b?01?_1???:           d_ld = 1;               // PLP
                8'b?10?_0???:           d_ld = 1;               // RTI
            endcase

        DECODE:
            casez( IR )
                8'b11?1_1000:           d_ld = 1;               // CLD/SED
            endcase
    endcase
end


/*
 * I flag
 */

always @* begin
    i_ld = 0;
    case( state )
        STK0:
            casez( IR )
                8'b?01?_1???:           i_ld = 1;               // PLP
                8'b?10?_0???:           i_ld = 1;               // RTI
            endcase

        STK2:
            casez( IR )
                8'b0000_0000:           i_ld = 1;               // BRK/IRQ
            endcase

        DECODE:
            casez( IR )
                8'b01?1_1000:           i_ld = 1;               // CLI/SEI
            endcase
    endcase
end

/*
 * V flag
 */

always @* begin
    v_ld = 0;
    case( state )
        DATA:
            casez( IR )
                8'b0010_?100:           v_ld = 1;               // BIT
            endcase

        STK0:
            casez( IR )
                8'b?01?_1???:           v_ld = 1;               // PLP
                8'b?10?_0???:           v_ld = 1;               // RTI
            endcase

        FETCH:
            casez( IR )
                8'b?11?_??01:           v_ld = 1;               // ADC/SBC
            endcase


        DECODE:
            casez( IR )
                8'b1011_1000:           v_ld = 1;               // CLV
            endcase
    endcase
end

/*
 * flag register update
 */
always @(posedge clk)
    if( c_ld )
        case( p_sel )
            2'b00:                      C <= CO;
            2'b01:                      C <= DB[0];
            2'b10:                      C <= BCD_C;
            2'b11:                      C <= IR[5];
        endcase

always @(posedge clk)
    if( v_ld )
        case( p_sel )
            2'b00:                      V <= VO;
            2'b01:                      V <= DB[6];
            2'b10:                      V <= 0;
            2'b11:                      V <= 0;
        endcase

always @(posedge clk)
    if( n_ld )
        case( p_sel[0] )
            1'b0:                       N <= NO;
            1'b1:                       N <= DB[7];
        endcase

always @(posedge clk)
    if( z_ld )
        case( p_sel[0] )
            1'b0:                       Z <= ZO;
            1'b1:                       Z <= DB[1];
        endcase

always @(posedge clk)
    if( d_ld )
        case( p_sel[0] )
            1'b0:                       D <= IR[5];
            1'b1:                       D <= DB[3];
        endcase

always @(posedge clk)
    if( i_ld )
        case( p_sel[0] )
            1'b0:                       I <= IR[5] | brk;
            1'b1:                       I <= DB[2];
        endcase


/*
 * state machine
 */
always @(posedge clk)
    if( RST )                           state <= DECODE;
    else case( state )
        DECODE:
            casez( IR )
                8'b1101_1011:           $finish;                // STP instruction
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

        FETCH:  if( bcd )               state <= BCD0;
                else                    state <= DECODE;

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

        BCD0 :                          state <= DECODE;
    endcase

/*
 * extra state bits and decoding
 */

/*
 * bcd is set when doing ADC/SBC with D flag on
 */

always @*
    casez( IR )
        8'b?11?_??01:                   bcd = D;
        default:                        bcd = 0;
    endcase

/*
 * rmw is set when doing a read-modify-write instruction
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
            BCD0:   statename = "BCD0";
    endcase

wire [7:0] B_ = B ? "B" : "-";
wire [7:0] C_ = C ? "C" : "-";
wire [7:0] D_ = D ? "D" : "-";
wire [7:0] I_ = I ? "I" : "-";
wire [7:0] N_ = N ? "N" : "-";
wire [7:0] V_ = V ? "V" : "-";
wire [7:0] Z_ = Z ? "Z" : "-";

integer cycle;

always @( posedge clk )
    cycle <= cycle + 1;

//always @( posedge clk ) if( state == BCD0 ) $display( "%h %h %h", Y, X, ALU );

always @( posedge clk )
      if( cycle[19:0] == 0 || IR == 8'hdb || !Debug )
      //if( cycle > 77000000 )
      $display( "%d %8s AB:%h DB:%h DO:%h PC:%h IR:%h WE:%d M:%02x S:%02x A:%02x X:%02x Y:%02x AI:%h BI:%h CI:%d OP:%d ALU:%h CO:%h IRQ:%h%h RST:%h%h P:%s%s%s%s%s%s%s (%d)",
        cycle,
        statename, AB, DB, DO, PC, IR, WE, M, S, A, X, Y,
        AI, alu.BI, CI, alu_op, ALU, CO, IRQ, take_irq, RST, in_rst, N_, V_, B_, D_, I_, Z_, C_, cond_true  );

endmodule
