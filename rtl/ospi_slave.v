`timescale 1ns/1ps

// OSPI slave for the custom 8-bit SDR protocol defined by
// doc/OSPI_Specification.pdf.
//
// System clock-domain FIFO interface
// ----------------------------------
// Request FIFO word (req_data):
//   [39]    0=Read, 1=Write
//   [38:36] CMD length code (0=4 bytes ... 7=512 bytes)
//   [35:32] reserved, always zero
//   [31:0]  effective address, aligned to four bytes
//
// wr_data contains complete 32-bit write words in bus byte order. req_data
// and wr_data are registered FIFO outputs and update on a clk rising edge
// that accepts req_rd_en or wr_rd_en respectively.
//
// For a Read request, system logic writes exactly TransferLength/4 words to
// the Read-data FIFO using rd_wr_en/rd_wr_data. rd_full provides backpressure.
module ospi_slave #(
    parameter integer FIFO_DEPTH     = 32,
    parameter integer FIFO_ADDR_WIDTH = 5,
    // 0: push-pull SRDY for a single slave; 1: open-drain SRDY
    parameter integer SRDY_OPEN_DRAIN = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        SCLK,
    input  wire        CSN,
    inout  wire [7:0]  D,
    inout  wire        SRDY,

    // OSPI request FIFO, read by the system clock domain.
    input  wire        req_rd_en,
    output wire [39:0] req_data,
    output wire        req_empty,

    // OSPI Write-data FIFO, read by the system clock domain.
    input  wire        wr_rd_en,
    output wire [31:0] wr_data,
    output wire        wr_empty,

    // OSPI Read-data FIFO, written by the system clock domain.
    input  wire        rd_wr_en,
    input  wire [31:0] rd_wr_data,
    output wire        rd_full
);

    localparam [2:0] ST_CMD        = 3'd0;
    localparam [2:0] ST_ADDR       = 3'd1;
    localparam [2:0] ST_WRITE      = 3'd2;
    localparam [2:0] ST_READ_WAIT  = 3'd3;
    localparam [2:0] ST_READ_DATA  = 3'd4;
    localparam [2:0] ST_IGNORE     = 3'd5;
    localparam [2:0] ST_COMPLETE   = 3'd6;

    reg [2:0]  state;
    reg        command_write;
    reg [2:0]  command_length;
    reg [23:0] address_shift;
    reg [1:0]  address_count;
    reg [31:0] write_shift;
    reg [1:0]  write_byte_count;
    reg [9:0]  bytes_remaining;

    reg [31:0] read_word;
    reg [1:0]  read_byte_count;
    reg        read_pop_pending;

    reg        srdy_ready;
    reg [7:0]  d_out;
    reg        d_oe;

    wire ospi_rst_n;
    wire selected;
    wire byte_accept;
    wire req_full;
    wire wr_full;
    wire rd_empty;
    wire [31:0] rd_fifo_data;
    wire req_fifo_wr_en;
    wire [39:0] req_fifo_wr_data;
    wire wr_fifo_wr_en;
    wire [31:0] wr_fifo_wr_data;
    wire rd_fifo_rd_en;

    assign ospi_rst_n = rst_n && !CSN;
    assign selected   = !CSN;
    assign byte_accept = selected && srdy_ready;

    // The A0 byte is included combinationally at the same SCLK rising edge
    // that writes the request FIFO. No later SCLK edge is required.
    assign req_fifo_wr_en = (state == ST_ADDR) &&
                            (address_count == 2'd3) && byte_accept &&
                            !req_full;
    assign req_fifo_wr_data = {
        command_write,
        command_length,
        4'b0000,
        address_shift,
        D[7:2],
        2'b00
    };

    assign wr_fifo_wr_en = (state == ST_WRITE) &&
                           (write_byte_count == 2'd3) && byte_accept &&
                           !wr_full;
    assign wr_fifo_wr_data = {write_shift[23:0], D};

    // Pop one complete Read word while SRDY is low. async_fifo registers the
    // word at this rising edge; it is loaded and driven on the next falling
    // edge before SRDY is raised.
    assign rd_fifo_rd_en = (state == ST_READ_WAIT) && !rd_empty &&
                           !read_pop_pending;

    assign D = (selected && d_oe) ? d_out : 8'bz;

    generate
        if (SRDY_OPEN_DRAIN != 0) begin : g_srdy_open_drain
            assign SRDY = (selected && !srdy_ready) ? 1'b0 : 1'bz;
        end else begin : g_srdy_push_pull
            assign SRDY = selected ? srdy_ready : 1'b1;
        end
    endgenerate

    async_fifo #(
        .DATA_WIDTH (40),
        .FIFO_DEPTH (FIFO_DEPTH),
        .ADDR_WIDTH (FIFO_ADDR_WIDTH)
    ) u_request_fifo (
        .wr_clk   (SCLK),
        .wr_rst_n (rst_n),
        .wr_en    (req_fifo_wr_en),
        .wr_data  (req_fifo_wr_data),
        .full     (req_full),
        .rd_clk   (clk),
        .rd_rst_n (rst_n),
        .rd_en    (req_rd_en),
        .rd_data  (req_data),
        .empty    (req_empty)
    );

    async_fifo #(
        .DATA_WIDTH (32),
        .FIFO_DEPTH (FIFO_DEPTH),
        .ADDR_WIDTH (FIFO_ADDR_WIDTH)
    ) u_write_data_fifo (
        .wr_clk   (SCLK),
        .wr_rst_n (rst_n),
        .wr_en    (wr_fifo_wr_en),
        .wr_data  (wr_fifo_wr_data),
        .full     (wr_full),
        .rd_clk   (clk),
        .rd_rst_n (rst_n),
        .rd_en    (wr_rd_en),
        .rd_data  (wr_data),
        .empty    (wr_empty)
    );

    async_fifo #(
        .DATA_WIDTH (32),
        .FIFO_DEPTH (FIFO_DEPTH),
        .ADDR_WIDTH (FIFO_ADDR_WIDTH)
    ) u_read_data_fifo (
        .wr_clk   (clk),
        .wr_rst_n (rst_n),
        .wr_en    (rd_wr_en),
        .wr_data  (rd_wr_data),
        .full     (rd_full),
        .rd_clk   (SCLK),
        .rd_rst_n (rst_n),
        .rd_en    (rd_fifo_rd_en),
        .rd_data  (rd_fifo_data),
        .empty    (rd_empty)
    );

    // Capture input bytes and control transaction progress at SCLK rising
    // edges. CSN asynchronously aborts the protocol state so a new command
    // can start even if no SCLK edge follows CSN deassertion.
    always @(posedge SCLK or negedge ospi_rst_n) begin
        if (!ospi_rst_n) begin
            state              <= ST_CMD;
            command_write      <= 1'b0;
            command_length     <= 3'b000;
            address_shift      <= 24'b0;
            address_count      <= 2'b0;
            write_shift        <= 32'b0;
            write_byte_count   <= 2'b0;
            bytes_remaining    <= 10'b0;
            read_byte_count    <= 2'b0;
            read_pop_pending   <= 1'b0;
            read_word          <= 32'b0;
        end else begin
            if (rd_fifo_rd_en)
                read_pop_pending <= 1'b1;

            case (state)
                ST_CMD: begin
                    if (byte_accept) begin
                        if (D[7:4] == 4'b0001) begin
                            command_write  <= D[3];
                            command_length <= D[2:0];
                            address_count  <= 2'd0;
                            state          <= ST_ADDR;
                        end else begin
                            state <= ST_IGNORE;
                        end
                    end
                end

                ST_ADDR: begin
                    if (byte_accept) begin
                        if (address_count == 2'd3) begin
                            bytes_remaining <= 10'd4 << command_length;
                            write_byte_count <= 2'd0;
                            read_byte_count  <= 2'd0;
                            if (command_write)
                                state <= ST_WRITE;
                            else
                                state <= ST_READ_WAIT;
                        end else begin
                            address_shift <= {address_shift[15:0], D};
                            address_count <= address_count + 1'b1;
                        end
                    end
                end

                ST_WRITE: begin
                    if (byte_accept) begin
                        if (write_byte_count == 2'd3) begin
                            write_byte_count <= 2'd0;
                            if (bytes_remaining == 10'd1) begin
                                bytes_remaining <= 10'd0;
                                state <= ST_COMPLETE;
                            end else begin
                                bytes_remaining <= bytes_remaining - 1'b1;
                            end
                        end else begin
                            write_shift <= {write_shift[23:0], D};
                            write_byte_count <= write_byte_count + 1'b1;
                            bytes_remaining <= bytes_remaining - 1'b1;
                        end
                    end
                end

                ST_READ_WAIT: begin
                    // read_pop_pending means rd_fifo_data was registered by
                    // the preceding SCLK rising edge. The following falling
                    // edge drove byte [31:24] and raised SRDY, so this edge
                    // accepts that first byte.
                    if (byte_accept && read_pop_pending) begin
                        read_word        <= rd_fifo_data;
                        read_pop_pending <= 1'b0;
                        if (bytes_remaining == 10'd1) begin
                            bytes_remaining <= 10'd0;
                            state <= ST_COMPLETE;
                        end else begin
                            bytes_remaining <= bytes_remaining - 1'b1;
                            read_byte_count <= 2'd1;
                            state <= ST_READ_DATA;
                        end
                    end
                end

                ST_READ_DATA: begin
                    if (byte_accept) begin
                        if (bytes_remaining == 10'd1) begin
                            bytes_remaining  <= 10'd0;
                            state <= ST_COMPLETE;
                        end else begin
                            bytes_remaining <= bytes_remaining - 1'b1;
                            if (read_byte_count == 2'd3) begin
                                read_byte_count  <= 2'd0;
                                state <= ST_READ_WAIT;
                            end else begin
                                read_byte_count <= read_byte_count + 1'b1;
                            end
                        end
                    end
                end

                ST_IGNORE: begin
                    state <= ST_IGNORE;
                end

                ST_COMPLETE: begin
                    state <= ST_COMPLETE;
                end

                default: state <= ST_IGNORE;
            endcase
        end
    end

    // SRDY is updated at falling edges as recommended by the specification.
    // Read output data also changes only at falling edges.
    always @(negedge SCLK or negedge ospi_rst_n) begin
        if (!ospi_rst_n) begin
            srdy_ready     <= 1'b1;
            d_out          <= 8'b0;
            d_oe           <= 1'b0;
        end else begin
            d_oe <= 1'b0;

            case (state)
                ST_CMD: begin
                    srdy_ready <= 1'b1;
                end

                ST_ADDR: begin
                    if ((address_count == 2'd3) && req_full)
                        srdy_ready <= 1'b0;
                    else
                        srdy_ready <= 1'b1;
                end

                ST_WRITE: begin
                    if (wr_full)
                        srdy_ready <= 1'b0;
                    else
                        srdy_ready <= 1'b1;
                end

                ST_READ_WAIT: begin
                    if (read_pop_pending) begin
                        d_out            <= rd_fifo_data[31:24];
                        d_oe             <= 1'b1;
                        srdy_ready       <= 1'b1;
                    end else begin
                        srdy_ready <= 1'b0;
                    end
                end

                ST_READ_DATA: begin
                    d_oe <= 1'b1;
                    case (read_byte_count)
                        2'd0: d_out <= read_word[31:24];
                        2'd1: d_out <= read_word[23:16];
                        2'd2: d_out <= read_word[15:8];
                        default: d_out <= read_word[7:0];
                    endcase
                    srdy_ready <= 1'b1;
                end

                ST_IGNORE: begin
                    srdy_ready <= 1'b1;
                    d_oe       <= 1'b0;
                end

                ST_COMPLETE: begin
                    srdy_ready <= 1'b1;
                    d_oe       <= 1'b0;
                end

                default: begin
                    srdy_ready <= 1'b1;
                    d_oe       <= 1'b0;
                end
            endcase
        end
    end

endmodule
