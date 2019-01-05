/*
 * 
 * ALU module
 *
 * This module has two inputs AI, and BI, a carry input CI, and an 8 bit 
 * output port OUT. Operation is determined by 'op'. Status flags are 
 * return in C, N, Z, V, as well as DC (decimal carry) and DHC (decimal 
 * half carry).
 *
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 */

module alu( 
    input [7:0] AI,             // AI input
    input [7:0] BI,             // BI input
    input CI,                   // CI input
    output [7:0] OUT,           // ALU result
    input [2:0] op,             // operation
    output reg C,               // Carry out
    output N,                   // Negative flag
    output Z,                   // Zero flag
    output V,                   // Overflow flag
    output DC,                  // Decimal carry (result > 99)
    output DHC                  // Decimal half carry (lower nibble > 9)
    );

`include "states.i"

/*
 * split nibble addition to get the half carry bit out
 */

wire HC;                                        // (binary) half carry bit
wire [4:0] LSD = AI[3:0] + BI[3:0] + CI;        // least significant digit
wire [4:0] MSD = AI[7:4] + BI[7:4] + HC;        // most significant digit

assign HC = LSD[4];

always @*
    case( op )
        ALU_AI:                         OUT = AI;
        ALU_ADC:                        OUT = { MSD[3:0], LSD[3:0] };
        ALU_ROL:                        OUT = { AI[6:0], CI };
        ALU_ROR:                        OUT = { CI, AI[7:1] };
        ALU_ORA:                        OUT = AI | BI;
        ALU_EOR:                        OUT = AI ^ BI;
        ALU_AND:                        OUT = AI & BI;
    default:                            OUT = 8'h55;
    endcase

assign N = OUT[7];
assign Z = OUT[7:0] == 0;
assign V = N ^ C ^ AI[7] ^ BI[7];

always @*
    if( op == ALU_ROL )                 C = AI[7];
    else if( op == ALU_ROR )            C = AI[0];
    else                                C = MSD[4];

/*
 * DHC is the decimal half carry. It is set when the lower 
 * nibble of the result is larger than 9. 
 */
wire DHC = LSD[3:0] >= 10;

/*
 * DC is the decimal carry. It is set when the upper nibble
 * of the result is larger than 9. We need to incorporate
 * the decimal carry from lower nibble here as well, in case
 * the lower nibble generates a carry after correction, while
 * upper nibble is equal to 9. 
 */
wire DC  = (MSD[3:0] + DHC) >= 10;

endmodule
