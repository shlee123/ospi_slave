`timescale 1ns/1ps

// RTL pad model. Replace this module with technology-specific IO cells during
// physical integration; ospi_slave_core remains unchanged.
module io_top (
    input  wire clk_pad,
    input  wire rst_n_pad,
    input  wire SCLK_pad,
    input  wire CSN_pad,
    inout  wire [7:0] D_pad,
    inout  wire SRDY_pad,

    output wire clk_core,
    output wire rst_n_core,
    output wire SCLK_core,
    output wire CSN_core,
    output wire [7:0] d_core_in,
    input  wire [7:0] d_core_out,
    input  wire d_core_oe,
    input  wire d_core_ie,
    input  wire srdy_core_out,
    input  wire srdy_core_oe,
    input  wire srdy_core_ie,
    output wire srdy_core_in
);

    // Dedicated input pads have their input buffers permanently enabled.
    wire clk_ie;
    wire rst_n_ie;
    wire sclk_ie;
    wire csn_ie;
    assign clk_ie = 1'b1;
    assign rst_n_ie = 1'b1;
    assign sclk_ie = 1'b1;
    assign csn_ie = 1'b1;

    assign clk_core = clk_ie ? clk_pad : 1'b0;
    assign rst_n_core = rst_n_ie ? rst_n_pad : 1'b0;
    assign SCLK_core = sclk_ie ? SCLK_pad : 1'b0;
    assign CSN_core = csn_ie ? CSN_pad : 1'b0;

    // Bidirectional data pads. Disabled input buffers present a deterministic
    // zero to the digital core; electrical contention remains visible on D_pad.
    assign D_pad = d_core_oe ? d_core_out : 8'bz;
    assign d_core_in = d_core_ie ? D_pad : 8'b0;

    // SRDY uses the same bidirectional pad model for both push-pull and
    // open-drain operation. The slave core does not consume SRDY, so IE is 0.
    assign SRDY_pad = srdy_core_oe ? srdy_core_out : 1'bz;
    assign srdy_core_in = srdy_core_ie ? SRDY_pad : 1'b0;

endmodule
