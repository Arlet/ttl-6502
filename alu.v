/*
 *
 * ALU module
 *
 * This module has two inputs AI, and MI, a carry input CI, and an 8 bit
 * output port OUT. Operation is determined by 'op'. Status flags are
 * return in C, N, Z, V, as well as DC (decimal carry) and DHC (decimal
 * half carry).
 *
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 */

module alu(
    input [7:0] AI,             // AI input
    input [7:0] MI,             // MI input (contents of M register)
    input CI,                   // CI input
    output [7:0] OUT,           // ALU result
    input mem_bi,               // if set, BI=MI, otherwise BI=00
    input inv_bi,               // if set, invert BI
    input [2:0] op,             // operation
    output reg C,               // Carry out (digital)
    output N,                   // Negative flag
    output Z,                   // Zero flag
    output V,                   // Overflow flag
    output adj_lsd,             // if set, lower digit needs BCD adjustment
    output adj_msd              // if set, upper digit needs BCD adjustment
    );

`include "states.i"

/*
 * BI input:
 *
 * mem | inv |  BI  | Usage
 * ----+-----+------+------------------------------------------
 *  0  |  0  |  00  | INC (with CI=1), or zero offset for address calc
 *  0  |  1  |  FF  | DEC
 *  1  |  0  |  MI  | LDA/ORA/AND/EOR/ADC/...
 *  1  |  1  | ~MI  | SBC/CMP/CPX/CPY
 */

wire [7:0] MEM = {8{mem_bi}};   // mem_bi repeated 8 times
wire [7:0] INV = {8{inv_bi}};   // inv_bi repeated 8 times

/*
 * determine BI input to ALU
 */
wire [7:0] BI = (MI & MEM) ^ INV;

/*
 * determine logic expression
 */
wire [7:0] AND = AI & BI;
wire [7:0] EOR = AI ^ BI;
wire [7:0] ORA = AI | BI;

/*
 * ripple carry chain
 *
 * note that the C8 output does not
 * depend on the operation, so it can
 * be set even when op = ALU_AI.
 *
 */
wire C1 = (ORA[0] & CI) | AND[0];
wire C2 = (ORA[1] & C1) | AND[1];
wire C3 = (ORA[2] & C2) | AND[2];
wire C4 = (ORA[3] & C3) | AND[3];
wire C5 = (ORA[4] & C4) | AND[4];
wire C6 = (ORA[5] & C5) | AND[5];
wire C7 = (ORA[6] & C6) | AND[6];
wire C8 = (ORA[7] & C7) | AND[7];

// adder
wire [7:0] ADC = EOR ^ { C7, C6, C5, C4, C3, C2, C1, CI };

// mux
always @*
    case( op )
        ALU_AI:                         OUT = AI;
        ALU_BI:                         OUT = BI;
        ALU_ADC:                        OUT = ADC;
        ALU_ROL:                        OUT = { AI[6:0], CI };
        ALU_ROR:                        OUT = { CI, AI[7:1] };
        ALU_ORA:                        OUT = ORA;
        ALU_EOR:                        OUT = EOR;
        ALU_AND:                        OUT = AND;
    endcase

// flags
assign N = OUT[7];
assign Z = OUT[7:0] == 0;
assign V = C8 ^ C7;

/*
 * Carry out. For ROL/ROR, the carry is the bit that's
 * shifted out. Otherwise, it's the output from the
 * ripple carry chain.
 */
always @*
    if( op == ALU_ROL )                 C = AI[7];
    else if( op == ALU_ROR )            C = AI[0];
    else                                C = C8;

/*
 * HCB is the digital half carry/half borrow. It is just the
 * carry output of the 4th bit, or the borrow output when
 * doing subtraction.
 */

wire HCB = C4 ^ inv_bi;

/*
 * DHC is the decimal half carry. It is set when the lower
 * nibble of the result is larger than 9.
 */
wire DHC = ADC[3] & (ADC[2] | ADC[1]);

/*
 * The lower nibble needs BCD adjustment of +6/-6 if we
 * have a digital carry/borrow, or a decimal carry.
 */
assign adj_lsd = HCB | DHC;

/*
 * DC is the decimal carry. It is set when the upper nibble
 * of the result is larger than 9. We need to incorporate
 * the decimal carry from lower nibble here as well, in case
 * the lower nibble generates a carry after correction, while
 * upper nibble is equal to 9.
 */
wire DC  = ADC[7] & (ADC[6] | ADC[5] | (ADC[4] & DHC));

/*
 * CB is the digital carry/borrow signal. It is active high
 * on carry (when adding) or borrow (when subtracting)
 */

wire CB = C8 ^ inv_bi;

/*
 * The lower nibble needs BCD adjustment of +6/-6 if we
 * have a digital carry/borrow, or a decimal carry.
 */
assign adj_msd = CB | DC;


endmodule
