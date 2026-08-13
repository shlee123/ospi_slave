`timescale 1ns/1ps

module tb_io_top;
    reg clk_pad;
    reg rst_n_pad;
    reg SCLK_pad;
    reg CSN_pad;
    tri [7:0] D_pad;
    tri SRDY_pad;
    reg [7:0] external_d;
    reg external_d_oe;
    reg external_srdy;
    reg external_srdy_oe;
    wire clk_core;
    wire rst_n_core;
    wire SCLK_core;
    wire CSN_core;
    wire [7:0] d_core_in;
    reg [7:0] d_core_out;
    reg d_core_oe;
    reg d_core_ie;
    reg srdy_core_out;
    reg srdy_core_oe;
    reg srdy_core_ie;
    wire srdy_core_in;
    integer errors;

    assign D_pad = external_d_oe ? external_d : 8'bz;
    assign SRDY_pad = external_srdy_oe ? external_srdy : 1'bz;

    io_top dut (
        .clk_pad(clk_pad), .rst_n_pad(rst_n_pad), .SCLK_pad(SCLK_pad),
        .CSN_pad(CSN_pad), .D_pad(D_pad), .SRDY_pad(SRDY_pad),
        .clk_core(clk_core), .rst_n_core(rst_n_core),
        .SCLK_core(SCLK_core), .CSN_core(CSN_core),
        .d_core_in(d_core_in), .d_core_out(d_core_out),
        .d_core_oe(d_core_oe), .d_core_ie(d_core_ie),
        .srdy_core_out(srdy_core_out), .srdy_core_oe(srdy_core_oe),
        .srdy_core_ie(srdy_core_ie), .srdy_core_in(srdy_core_in)
    );

    initial begin
        errors = 0;
        clk_pad = 1'b0;
        rst_n_pad = 1'b0;
        SCLK_pad = 1'b0;
        CSN_pad = 1'b1;
        external_d = 8'h00;
        external_d_oe = 1'b0;
        external_srdy = 1'b0;
        external_srdy_oe = 1'b0;
        d_core_out = 8'ha5;
        d_core_oe = 1'b0;
        d_core_ie = 1'b0;
        srdy_core_out = 1'b0;
        srdy_core_oe = 1'b0;
        srdy_core_ie = 1'b0;
        #1;

        // Dedicated input buffers are permanently enabled.
        clk_pad = 1'b1;
        rst_n_pad = 1'b1;
        SCLK_pad = 1'b1;
        CSN_pad = 1'b0;
        #1;
        if ({clk_core, rst_n_core, SCLK_core, CSN_core} !== 4'b1110)
            errors = errors + 1;

        // Disabled bidirectional input buffers return deterministic zero.
        external_d = 8'h3c;
        external_d_oe = 1'b1;
        #1;
        if (d_core_in !== 8'h00)
            errors = errors + 1;
        d_core_ie = 1'b1;
        #1;
        if (d_core_in !== 8'h3c)
            errors = errors + 1;

        // Output-enable drives the pad; disabling it releases High-Z.
        external_d_oe = 1'b0;
        d_core_ie = 1'b0;
        d_core_oe = 1'b1;
        #1;
        if (D_pad !== 8'ha5)
            errors = errors + 1;
        d_core_oe = 1'b0;
        #1;
        if (D_pad !== 8'hzz)
            errors = errors + 1;

        // Electrical contention is visible at the pad as X.
        d_core_oe = 1'b1;
        external_d = 8'h5a;
        external_d_oe = 1'b1;
        #1;
        if (D_pad !== 8'hxx)
            errors = errors + 1;

        // SRDY supports push-pull drive, High-Z and optional input sampling.
        external_d_oe = 1'b0;
        d_core_oe = 1'b0;
        srdy_core_out = 1'b1;
        srdy_core_oe = 1'b1;
        #1;
        if (SRDY_pad !== 1'b1)
            errors = errors + 1;
        srdy_core_oe = 1'b0;
        #1;
        if (SRDY_pad !== 1'bz)
            errors = errors + 1;
        external_srdy = 1'b1;
        external_srdy_oe = 1'b1;
        #1;
        if (srdy_core_in !== 1'b0)
            errors = errors + 1;
        srdy_core_ie = 1'b1;
        #1;
        if (srdy_core_in !== 1'b1)
            errors = errors + 1;

        if (errors == 0)
            $display("PASS: io_top RTL pad model test");
        else
            $display("FAIL: io_top RTL pad model errors=%0d", errors);
        #1 $finish;
    end
endmodule
