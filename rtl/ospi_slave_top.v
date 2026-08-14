`timescale 1ns/1ps

// Compatibility top level. OSPI pads are isolated in io_top while all
// protocol, CDC FIFO and AXI logic resides in the pure-digital core.
module ospi_slave_top #(
    // Rule: fixed at 32 to match the OSPI effective-address width.
    parameter integer AXI_ADDR_WIDTH = 32,
    // Options: 8, 16, 32, 64, 128, 256, 512 or 1024 bits.
    parameter integer AXI_DATA_WIDTH = 32,
    // Rule: AXI ID width must be at least 1; default is 6 bits.
    parameter integer AXI_ID_WIDTH = 6,
    // Option: constant ID driven on all AXI transactions; default zero.
    parameter [AXI_ID_WIDTH-1:0] AXI_ID = {AXI_ID_WIDTH{1'b0}},
    // Option: AxPROT value; default 3'b001.
    parameter [2:0] AXI_PROT = 3'b001,
    // Rule: Write/Read data FIFO depth, power of two and at least 2.
    parameter integer FIFO_DEPTH = 32,
    // Rule: must equal log2(FIFO_DEPTH).
    parameter integer FIFO_ADDR_WIDTH = 5,
    // Rule: Request FIFO depth, power of two and at least 2.
    parameter integer REQ_FIFO_DEPTH = 2,
    // Rule: must equal log2(REQ_FIFO_DEPTH).
    parameter integer REQ_ADDR_WIDTH = 1,
    // Options: 0 = push-pull SRDY, 1 = open-drain SRDY.
    parameter integer SRDY_OPEN_DRAIN = 0
) (
    input  wire clk,
    input  wire rst_n,
    input  wire SCLK,
    input  wire CSN,
    inout  wire [7:0] D,
    inout  wire SRDY,

    output wire [AXI_ID_WIDTH-1:0] M_AXI_AWID,
    output wire [AXI_ADDR_WIDTH-1:0] M_AXI_AWADDR,
    output wire [7:0] M_AXI_AWLEN,
    output wire [2:0] M_AXI_AWSIZE,
    output wire [1:0] M_AXI_AWBURST,
    output wire [2:0] M_AXI_AWPROT,
    output wire M_AXI_AWVALID,
    input  wire M_AXI_AWREADY,
    output wire [AXI_DATA_WIDTH-1:0] M_AXI_WDATA,
    output wire [(AXI_DATA_WIDTH/8)-1:0] M_AXI_WSTRB,
    output wire M_AXI_WLAST,
    output wire M_AXI_WVALID,
    input  wire M_AXI_WREADY,
    input  wire [AXI_ID_WIDTH-1:0] M_AXI_BID,
    input  wire [1:0] M_AXI_BRESP,
    input  wire M_AXI_BVALID,
    output wire M_AXI_BREADY,

    output wire [AXI_ID_WIDTH-1:0] M_AXI_ARID,
    output wire [AXI_ADDR_WIDTH-1:0] M_AXI_ARADDR,
    output wire [7:0] M_AXI_ARLEN,
    output wire [2:0] M_AXI_ARSIZE,
    output wire [1:0] M_AXI_ARBURST,
    output wire [2:0] M_AXI_ARPROT,
    output wire M_AXI_ARVALID,
    input  wire M_AXI_ARREADY,
    input  wire [AXI_ID_WIDTH-1:0] M_AXI_RID,
    input  wire [AXI_DATA_WIDTH-1:0] M_AXI_RDATA,
    input  wire [1:0] M_AXI_RRESP,
    input  wire M_AXI_RLAST,
    input  wire M_AXI_RVALID,
    output wire M_AXI_RREADY
);

    wire core_clk;
    wire core_rst_n;
    wire core_sclk;
    wire core_csn;
    wire [7:0] core_d_in;
    wire [7:0] core_d_out;
    wire core_d_oe;
    wire core_d_ie;
    wire core_srdy_out;
    wire core_srdy_oe;
    wire core_srdy_ie;
    wire core_srdy_in;

    io_top u_io_top (
        .clk_pad(clk), .rst_n_pad(rst_n), .SCLK_pad(SCLK),
        .CSN_pad(CSN), .D_pad(D), .SRDY_pad(SRDY),
        .clk_core(core_clk), .rst_n_core(core_rst_n),
        .SCLK_core(core_sclk), .CSN_core(core_csn),
        .d_core_in(core_d_in), .d_core_out(core_d_out),
        .d_core_oe(core_d_oe), .d_core_ie(core_d_ie),
        .srdy_core_out(core_srdy_out), .srdy_core_oe(core_srdy_oe),
        .srdy_core_ie(core_srdy_ie), .srdy_core_in(core_srdy_in)
    );

    ospi_slave #(
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH), .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH), .AXI_ID(AXI_ID),
        .AXI_PROT(AXI_PROT), .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_ADDR_WIDTH(FIFO_ADDR_WIDTH),
        .REQ_FIFO_DEPTH(REQ_FIFO_DEPTH), .REQ_ADDR_WIDTH(REQ_ADDR_WIDTH),
        .SRDY_OPEN_DRAIN(SRDY_OPEN_DRAIN)
    ) u_core (
        .clk(core_clk), .rst_n(core_rst_n), .SCLK(core_sclk),
        .CSN(core_csn), .d_in(core_d_in), .d_out(core_d_out),
        .d_oe(core_d_oe), .d_ie(core_d_ie),
        .srdy_out(core_srdy_out), .srdy_oe(core_srdy_oe),
        .srdy_ie(core_srdy_ie),
        .M_AXI_AWID(M_AXI_AWID), .M_AXI_AWADDR(M_AXI_AWADDR),
        .M_AXI_AWLEN(M_AXI_AWLEN), .M_AXI_AWSIZE(M_AXI_AWSIZE),
        .M_AXI_AWBURST(M_AXI_AWBURST), .M_AXI_AWPROT(M_AXI_AWPROT),
        .M_AXI_AWVALID(M_AXI_AWVALID), .M_AXI_AWREADY(M_AXI_AWREADY),
        .M_AXI_WDATA(M_AXI_WDATA), .M_AXI_WSTRB(M_AXI_WSTRB),
        .M_AXI_WLAST(M_AXI_WLAST), .M_AXI_WVALID(M_AXI_WVALID),
        .M_AXI_WREADY(M_AXI_WREADY), .M_AXI_BID(M_AXI_BID),
        .M_AXI_BRESP(M_AXI_BRESP), .M_AXI_BVALID(M_AXI_BVALID),
        .M_AXI_BREADY(M_AXI_BREADY), .M_AXI_ARID(M_AXI_ARID),
        .M_AXI_ARADDR(M_AXI_ARADDR), .M_AXI_ARLEN(M_AXI_ARLEN),
        .M_AXI_ARSIZE(M_AXI_ARSIZE), .M_AXI_ARBURST(M_AXI_ARBURST),
        .M_AXI_ARPROT(M_AXI_ARPROT), .M_AXI_ARVALID(M_AXI_ARVALID),
        .M_AXI_ARREADY(M_AXI_ARREADY), .M_AXI_RID(M_AXI_RID),
        .M_AXI_RDATA(M_AXI_RDATA), .M_AXI_RRESP(M_AXI_RRESP),
        .M_AXI_RLAST(M_AXI_RLAST), .M_AXI_RVALID(M_AXI_RVALID),
        .M_AXI_RREADY(M_AXI_RREADY)
    );

    // Keep the disabled SRDY input-buffer result visible to lint tools.
    wire unused_srdy_input;
    assign unused_srdy_input = core_srdy_in;

endmodule
