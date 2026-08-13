`timescale 1ns/1ps

`ifndef SINGLE_TRAN
`define SINGLE_TRAN 1
`endif

// Custom 8-bit SDR OSPI slave with an AXI4 master backend.
// SINGLE_TRAN defaults to 1: one AXI transaction per 32-bit OSPI word.
// Define SINGLE_TRAN=0 to issue one AXI INCR burst per OSPI request.
module ospi_slave #(
    parameter integer AXI_ADDR_WIDTH = 32,
    parameter integer AXI_DATA_WIDTH = 32,
    parameter integer AXI_ID_WIDTH = 6,
    parameter [AXI_ID_WIDTH-1:0] AXI_ID = {AXI_ID_WIDTH{1'b0}},
    parameter [2:0] AXI_PROT = 3'b001,
    parameter integer FIFO_DEPTH = 32,
    parameter integer FIFO_ADDR_WIDTH = 5,
    parameter integer REQ_FIFO_DEPTH = 2,
    parameter integer REQ_ADDR_WIDTH = 1,
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

    localparam integer AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8;

    localparam [2:0] OSPI_CMD       = 3'd0;
    localparam [2:0] OSPI_ADDR      = 3'd1;
    localparam [2:0] OSPI_WRITE     = 3'd2;
    localparam [2:0] OSPI_RD_WAIT   = 3'd3;
    localparam [2:0] OSPI_RD_DATA   = 3'd4;
    localparam [2:0] OSPI_IGNORE    = 3'd5;
    localparam [2:0] OSPI_COMPLETE  = 3'd6;

    localparam [3:0] AXI_IDLE       = 4'd0;
    localparam [3:0] AXI_REQ_POP    = 4'd1;
    localparam [3:0] AXI_REQ_LATCH  = 4'd2;
    localparam [3:0] AXI_WR_ADDR    = 4'd3;
    localparam [3:0] AXI_WR_POP     = 4'd4;
    localparam [3:0] AXI_WR_LATCH   = 4'd5;
    localparam [3:0] AXI_WR_DATA    = 4'd6;
    localparam [3:0] AXI_WR_RESP    = 4'd7;
    localparam [3:0] AXI_RD_ADDR    = 4'd8;
    localparam [3:0] AXI_RD_DATA    = 4'd9;

    reg [2:0] ospi_state;
    reg command_write;
    reg [2:0] command_length;
    reg [23:0] address_shift;
    reg [1:0] address_count;
    reg [31:0] write_shift;
    reg [1:0] write_byte_count;
    reg [9:0] ospi_bytes_remaining;
    reg [31:0] read_word;
    reg [1:0] read_byte_count;
    reg read_pop_pending;
    reg srdy_ready;
    reg [7:0] d_out;
    reg d_oe;

    reg [3:0] axi_state;
    reg [AXI_ADDR_WIDTH-1:0] axi_address;
    reg [7:0] axi_words_remaining;
    reg [7:0] axi_burst_length;
    reg [31:0] axi_write_word;

    wire ospi_rst_n;
    wire selected;
    wire byte_accept;
    wire req_full;
    wire req_empty;
    wire [39:0] req_fifo_data;
    wire req_fifo_wr_en;
    wire req_fifo_rd_en;
    wire wr_full;
    wire wr_empty;
    wire [31:0] wr_fifo_data;
    wire wr_fifo_wr_en;
    wire wr_fifo_rd_en;
    wire [31:0] wr_fifo_wr_data;
    wire rd_full;
    wire rd_empty;
    wire [31:0] rd_fifo_data;
    wire rd_fifo_wr_en;
    wire rd_fifo_rd_en;
    wire [31:0] axi_read_word;
    wire [AXI_DATA_WIDTH-1:0] axi_wdata_shifted;
    wire [AXI_STRB_WIDTH-1:0] axi_wstrb_shifted;

    assign ospi_rst_n = rst_n && !CSN;
    assign selected = !CSN;
    assign byte_accept = selected && srdy_ready;

    assign req_fifo_wr_en = (ospi_state == OSPI_ADDR) &&
                            (address_count == 2'd3) && byte_accept &&
                            !req_full;
    assign wr_fifo_wr_en = (ospi_state == OSPI_WRITE) &&
                           (write_byte_count == 2'd3) && byte_accept &&
                           !wr_full;
    assign wr_fifo_wr_data = {write_shift[23:0], D};
    assign rd_fifo_rd_en = (ospi_state == OSPI_RD_WAIT) && !rd_empty &&
                           !read_pop_pending;

    assign req_fifo_rd_en = (axi_state == AXI_REQ_POP);
    assign wr_fifo_rd_en = (axi_state == AXI_WR_POP) && !wr_empty;
    assign rd_fifo_wr_en = (axi_state == AXI_RD_DATA) &&
                            M_AXI_RVALID && M_AXI_RREADY;

    assign D = (selected && d_oe) ? d_out : 8'bz;
    generate
        if (SRDY_OPEN_DRAIN != 0) begin : g_srdy_open_drain
            assign SRDY = (selected && !srdy_ready) ? 1'b0 : 1'bz;
        end else begin : g_srdy_push_pull
            assign SRDY = selected ? srdy_ready : 1'b1;
        end
    endgenerate

    // Request packing remains internal. Captured CMD and address fields are
    // connected directly to the FIFO wr_data input.
    async_fifo #(
        .DATA_WIDTH(40), .FIFO_DEPTH(REQ_FIFO_DEPTH),
        .ADDR_WIDTH(REQ_ADDR_WIDTH)
    ) u_request_fifo (
        .wr_clk(SCLK), .wr_rst_n(rst_n), .wr_en(req_fifo_wr_en),
        .wr_data({command_write, command_length, 4'b0000,
                  address_shift, D[7:2], 2'b00}),
        .full(req_full), .rd_clk(clk), .rd_rst_n(rst_n),
        .rd_en(req_fifo_rd_en), .rd_data(req_fifo_data), .empty(req_empty)
    );

    async_fifo #(
        .DATA_WIDTH(32), .FIFO_DEPTH(FIFO_DEPTH),
        .ADDR_WIDTH(FIFO_ADDR_WIDTH)
    ) u_write_data_fifo (
        .wr_clk(SCLK), .wr_rst_n(rst_n), .wr_en(wr_fifo_wr_en),
        .wr_data(wr_fifo_wr_data), .full(wr_full),
        .rd_clk(clk), .rd_rst_n(rst_n), .rd_en(wr_fifo_rd_en),
        .rd_data(wr_fifo_data), .empty(wr_empty)
    );

    async_fifo #(
        .DATA_WIDTH(32), .FIFO_DEPTH(FIFO_DEPTH),
        .ADDR_WIDTH(FIFO_ADDR_WIDTH)
    ) u_read_data_fifo (
        .wr_clk(clk), .wr_rst_n(rst_n), .wr_en(rd_fifo_wr_en),
        .wr_data(axi_read_word), .full(rd_full),
        .rd_clk(SCLK), .rd_rst_n(rst_n), .rd_en(rd_fifo_rd_en),
        .rd_data(rd_fifo_data), .empty(rd_empty)
    );

    // The OSPI protocol word is fixed at 32 bits. Wider AXI buses use 32-bit
    // narrow transfers and select lanes from the aligned AXI address.
    assign axi_wdata_shifted = {{(AXI_DATA_WIDTH-32){1'b0}}, axi_write_word}
                               << ((axi_address % AXI_STRB_WIDTH) * 8);
    assign axi_wstrb_shifted = {{(AXI_STRB_WIDTH-4){1'b0}}, 4'b1111}
                               << (axi_address % AXI_STRB_WIDTH);
    assign axi_read_word = M_AXI_RDATA >>
                           ((axi_address % AXI_STRB_WIDTH) * 8);

    assign M_AXI_AWID = AXI_ID;
    assign M_AXI_AWADDR = axi_address;
    assign M_AXI_AWSIZE = 3'd2;
    assign M_AXI_AWBURST = 2'b01;
    assign M_AXI_AWPROT = AXI_PROT;
    assign M_AXI_AWVALID = (axi_state == AXI_WR_ADDR);
    assign M_AXI_WDATA = axi_wdata_shifted;
    assign M_AXI_WSTRB = axi_wstrb_shifted;
    assign M_AXI_WVALID = (axi_state == AXI_WR_DATA);
    assign M_AXI_BREADY = (axi_state == AXI_WR_RESP);
    assign M_AXI_ARID = AXI_ID;
    assign M_AXI_ARADDR = axi_address;
    assign M_AXI_ARSIZE = 3'd2;
    assign M_AXI_ARBURST = 2'b01;
    assign M_AXI_ARPROT = AXI_PROT;
    assign M_AXI_ARVALID = (axi_state == AXI_RD_ADDR);
    assign M_AXI_RREADY = (axi_state == AXI_RD_DATA) && !rd_full;

    generate
        if (`SINGLE_TRAN != 0) begin : g_axi_single
            assign M_AXI_AWLEN = 8'd0;
            assign M_AXI_ARLEN = 8'd0;
            assign M_AXI_WLAST = 1'b1;
        end else begin : g_axi_burst
            assign M_AXI_AWLEN = axi_burst_length - 1'b1;
            assign M_AXI_ARLEN = axi_burst_length - 1'b1;
            assign M_AXI_WLAST = (axi_words_remaining == 8'd1);
        end
    endgenerate

`ifndef SYNTHESIS
    initial begin
        if (AXI_ADDR_WIDTH != 32) begin
            $display("ERROR: AXI_ADDR_WIDTH must be 32");
            $finish;
        end
        if ((AXI_DATA_WIDTH < 32) || ((AXI_DATA_WIDTH % 32) != 0) ||
            ((AXI_DATA_WIDTH & (AXI_DATA_WIDTH - 1)) != 0)) begin
            $display("ERROR: AXI_DATA_WIDTH must be a power-of-two multiple of 32");
            $finish;
        end
    end
`endif

    // OSPI clock-domain protocol engine.
    always @(posedge SCLK or negedge ospi_rst_n) begin
        if (!ospi_rst_n) begin
            ospi_state <= OSPI_CMD;
            command_write <= 1'b0;
            command_length <= 3'b0;
            address_shift <= 24'b0;
            address_count <= 2'b0;
            write_shift <= 32'b0;
            write_byte_count <= 2'b0;
            ospi_bytes_remaining <= 10'b0;
            read_word <= 32'b0;
            read_byte_count <= 2'b0;
            read_pop_pending <= 1'b0;
        end else begin
            if (rd_fifo_rd_en)
                read_pop_pending <= 1'b1;
            case (ospi_state)
                OSPI_CMD: begin
                    if (byte_accept) begin
                        if (D[7:4] == 4'b0001) begin
                            command_write <= D[3];
                            command_length <= D[2:0];
                            address_count <= 2'd0;
                            ospi_state <= OSPI_ADDR;
                        end else begin
                            ospi_state <= OSPI_IGNORE;
                        end
                    end
                end
                OSPI_ADDR: begin
                    if (byte_accept) begin
                        if (address_count == 2'd3) begin
                            ospi_bytes_remaining <= 10'd4 << command_length;
                            write_byte_count <= 2'd0;
                            read_byte_count <= 2'd0;
                            if (command_write)
                                ospi_state <= OSPI_WRITE;
                            else
                                ospi_state <= OSPI_RD_WAIT;
                        end else begin
                            address_shift <= {address_shift[15:0], D};
                            address_count <= address_count + 1'b1;
                        end
                    end
                end
                OSPI_WRITE: begin
                    if (byte_accept) begin
                        if (write_byte_count == 2'd3) begin
                            write_byte_count <= 2'd0;
                            if (ospi_bytes_remaining == 10'd1) begin
                                ospi_bytes_remaining <= 10'd0;
                                ospi_state <= OSPI_COMPLETE;
                            end else begin
                                ospi_bytes_remaining <= ospi_bytes_remaining - 1'b1;
                            end
                        end else begin
                            write_shift <= {write_shift[23:0], D};
                            write_byte_count <= write_byte_count + 1'b1;
                            ospi_bytes_remaining <= ospi_bytes_remaining - 1'b1;
                        end
                    end
                end
                OSPI_RD_WAIT: begin
                    if (byte_accept && read_pop_pending) begin
                        read_word <= rd_fifo_data;
                        read_pop_pending <= 1'b0;
                        ospi_bytes_remaining <= ospi_bytes_remaining - 1'b1;
                        read_byte_count <= 2'd1;
                        ospi_state <= OSPI_RD_DATA;
                    end
                end
                OSPI_RD_DATA: begin
                    if (byte_accept) begin
                        if (ospi_bytes_remaining == 10'd1) begin
                            ospi_bytes_remaining <= 10'd0;
                            ospi_state <= OSPI_COMPLETE;
                        end else begin
                            ospi_bytes_remaining <= ospi_bytes_remaining - 1'b1;
                            if (read_byte_count == 2'd3) begin
                                read_byte_count <= 2'd0;
                                ospi_state <= OSPI_RD_WAIT;
                            end else begin
                                read_byte_count <= read_byte_count + 1'b1;
                            end
                        end
                    end
                end
                OSPI_IGNORE: ospi_state <= OSPI_IGNORE;
                OSPI_COMPLETE: ospi_state <= OSPI_COMPLETE;
                default: ospi_state <= OSPI_IGNORE;
            endcase
        end
    end

    always @(negedge SCLK or negedge ospi_rst_n) begin
        if (!ospi_rst_n) begin
            srdy_ready <= 1'b1;
            d_out <= 8'b0;
            d_oe <= 1'b0;
        end else begin
            d_oe <= 1'b0;
            case (ospi_state)
                OSPI_CMD: srdy_ready <= 1'b1;
                OSPI_ADDR: srdy_ready <= !((address_count == 2'd3) && req_full);
                OSPI_WRITE: srdy_ready <= !wr_full;
                OSPI_RD_WAIT: begin
                    if (read_pop_pending) begin
                        d_out <= rd_fifo_data[31:24];
                        d_oe <= 1'b1;
                        srdy_ready <= 1'b1;
                    end else begin
                        srdy_ready <= 1'b0;
                    end
                end
                OSPI_RD_DATA: begin
                    d_oe <= 1'b1;
                    case (read_byte_count)
                        2'd0: d_out <= read_word[31:24];
                        2'd1: d_out <= read_word[23:16];
                        2'd2: d_out <= read_word[15:8];
                        default: d_out <= read_word[7:0];
                    endcase
                    srdy_ready <= 1'b1;
                end
                default: begin
                    srdy_ready <= 1'b1;
                    d_oe <= 1'b0;
                end
            endcase
        end
    end

    // System clock-domain AXI master controller.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_state <= AXI_IDLE;
            axi_address <= {AXI_ADDR_WIDTH{1'b0}};
            axi_words_remaining <= 8'b0;
            axi_burst_length <= 8'b0;
            axi_write_word <= 32'b0;
        end else begin
            case (axi_state)
                AXI_IDLE: begin
                    if (!req_empty)
                        axi_state <= AXI_REQ_POP;
                end
                AXI_REQ_POP: axi_state <= AXI_REQ_LATCH;
                AXI_REQ_LATCH: begin
                    axi_address <= req_fifo_data[31:0];
                    axi_words_remaining <= 8'd1 << req_fifo_data[38:36];
                    axi_burst_length <= 8'd1 << req_fifo_data[38:36];
                    if (req_fifo_data[39])
                        axi_state <= AXI_WR_ADDR;
                    else
                        axi_state <= AXI_RD_ADDR;
                end
                AXI_WR_ADDR: begin
                    if (M_AXI_AWREADY)
                        axi_state <= AXI_WR_POP;
                end
                AXI_WR_POP: begin
                    if (!wr_empty)
                        axi_state <= AXI_WR_LATCH;
                end
                AXI_WR_LATCH: begin
                    axi_write_word <= wr_fifo_data;
                    axi_state <= AXI_WR_DATA;
                end
                AXI_WR_DATA: begin
                    if (M_AXI_WREADY) begin
                        if (`SINGLE_TRAN != 0) begin
                            axi_state <= AXI_WR_RESP;
                        end else if (axi_words_remaining == 8'd1) begin
                            axi_state <= AXI_WR_RESP;
                        end else begin
                            axi_address <= axi_address + 4;
                            axi_words_remaining <= axi_words_remaining - 1'b1;
                            axi_state <= AXI_WR_POP;
                        end
                    end
                end
                AXI_WR_RESP: begin
                    if (M_AXI_BVALID) begin
                        if (`SINGLE_TRAN != 0) begin
                            if (axi_words_remaining == 8'd1) begin
                                axi_words_remaining <= 8'd0;
                                axi_state <= AXI_IDLE;
                            end else begin
                                axi_address <= axi_address + 4;
                                axi_words_remaining <= axi_words_remaining - 1'b1;
                                axi_state <= AXI_WR_ADDR;
                            end
                        end else begin
                            axi_words_remaining <= 8'd0;
                            axi_state <= AXI_IDLE;
                        end
                    end
                end
                AXI_RD_ADDR: begin
                    if (M_AXI_ARREADY)
                        axi_state <= AXI_RD_DATA;
                end
                AXI_RD_DATA: begin
                    if (M_AXI_RVALID && M_AXI_RREADY) begin
                        if (`SINGLE_TRAN != 0) begin
                            if (axi_words_remaining == 8'd1) begin
                                axi_words_remaining <= 8'd0;
                                axi_state <= AXI_IDLE;
                            end else begin
                                axi_address <= axi_address + 4;
                                axi_words_remaining <= axi_words_remaining - 1'b1;
                                axi_state <= AXI_RD_ADDR;
                            end
                        end else begin
                            if (M_AXI_RLAST || (axi_words_remaining == 8'd1)) begin
                                axi_words_remaining <= 8'd0;
                                axi_state <= AXI_IDLE;
                            end else begin
                                axi_address <= axi_address + 4;
                                axi_words_remaining <= axi_words_remaining - 1'b1;
                            end
                        end
                    end
                end
                default: axi_state <= AXI_IDLE;
            endcase
        end
    end

    // Keep response and ID inputs visible for integration-time checking.
    wire unused_axi_inputs;
    assign unused_axi_inputs = ^{M_AXI_BID, M_AXI_BRESP,
                                 M_AXI_RID, M_AXI_RRESP};

endmodule
