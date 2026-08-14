`timescale 1ns/1ps

// Simulation-only APB4-to-OSPI behavioral model.
//
// Rules:
// - APB address and data widths are fixed at 32 bits.
// - One aligned, full-word APB access becomes one 4-byte OSPI request.
// - PPROT is accepted and recorded, but the current OSPI protocol has no
//   protection field, so it is not transmitted to the OSPI slave.
// - Partial writes and unaligned accesses complete with PSLVERR asserted.
// - An OSPI request that does not finish within TIMEOUT_CYCLES PCLK cycles
//   completes with PSLVERR asserted.
// - This model uses delay controls and is not synthesizable.
module ospi_master #(
    // Rule: at least 2 ns, allowing SRDY/data sampling one nanosecond after
    // each rising SCLK edge while preserving a non-zero high time.
    parameter integer SCLK_HALF_PERIOD = 5,
    // Rule: positive number of PCLK cycles allowed for one OSPI request.
    parameter integer TIMEOUT_CYCLES = 1024
) (
    input  wire PCLK,
    input  wire PRESETn,
    input  wire [31:0] PADDR,
    input  wire PSEL,
    input  wire PENABLE,
    input  wire PWRITE,
    input  wire [31:0] PWDATA,
    input  wire [3:0] PSTRB,
    input  wire [2:0] PPROT,
    output reg  [31:0] PRDATA,
    output reg  PREADY,
    output reg  PSLVERR,

    output reg  SCLK,
    output reg  CSN,
    inout  wire [7:0] D,
    input  wire SRDY
);

    reg [7:0] d_out;
    reg d_oe;
    reg transaction_active;
    reg timeout_expired;
    reg [2:0] last_pprot;
    integer timeout_count;

    reg request_ok;
    reg [31:0] request_read_data;

    assign D = d_oe ? d_out : 8'bz;

    // TIMEOUT_CYCLES is defined in the APB clock domain even though OSPI
    // signaling is generated with simulation delay controls.
    always @(posedge PCLK) begin
        if (!PRESETn || !transaction_active) begin
            timeout_count <= 0;
            timeout_expired <= 1'b0;
        end else if (!timeout_expired) begin
            if (timeout_count >= (TIMEOUT_CYCLES - 1))
                timeout_expired <= 1'b1;
            else
                timeout_count <= timeout_count + 1;
        end
    end

    task ospi_send_byte;
        input [7:0] value;
        output success;
        reg accepted;
        begin
            success = 1'b0;
            accepted = 1'b0;
            d_out = value;
            d_oe = 1'b1;
            while (!accepted && !timeout_expired && (PRESETn === 1'b1)) begin
                #(SCLK_HALF_PERIOD) SCLK = 1'b1;
                #1 accepted = (SRDY === 1'b1);
                #(SCLK_HALF_PERIOD - 1) SCLK = 1'b0;
            end
            if (accepted && !timeout_expired && (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    task ospi_receive_byte;
        output [7:0] value;
        output success;
        reg accepted;
        begin
            value = 8'b0;
            success = 1'b0;
            accepted = 1'b0;
            d_oe = 1'b0;
            while (!accepted && !timeout_expired && (PRESETn === 1'b1)) begin
                #(SCLK_HALF_PERIOD) SCLK = 1'b1;
                #1;
                if (SRDY === 1'b1) begin
                    value = D;
                    accepted = 1'b1;
                end
                #(SCLK_HALF_PERIOD - 1) SCLK = 1'b0;
            end
            if (accepted && !timeout_expired && (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    task ospi_transfer;
        input write_request;
        input [31:0] address;
        input [31:0] write_data;
        output [31:0] read_data;
        output success;
        reg step_ok;
        reg [7:0] read_byte;
        begin
            read_data = 32'b0;
            success = 1'b0;
            step_ok = 1'b1;
            SCLK = 1'b0;
            CSN = 1'b0;
            #(SCLK_HALF_PERIOD);

            // Command format is 0001_W_LLL. One APB word is four bytes, so
            // LLL=000; W=1 is Write and W=0 is Read.
            if (step_ok)
                ospi_send_byte(write_request ? 8'h18 : 8'h10, step_ok);
            if (step_ok)
                ospi_send_byte(address[31:24], step_ok);
            if (step_ok)
                ospi_send_byte(address[23:16], step_ok);
            if (step_ok)
                ospi_send_byte(address[15:8], step_ok);
            if (step_ok)
                ospi_send_byte(address[7:0], step_ok);

            if (write_request) begin
                if (step_ok)
                    ospi_send_byte(write_data[31:24], step_ok);
                if (step_ok)
                    ospi_send_byte(write_data[23:16], step_ok);
                if (step_ok)
                    ospi_send_byte(write_data[15:8], step_ok);
                if (step_ok)
                    ospi_send_byte(write_data[7:0], step_ok);
            end else begin
                // Release D after the final address byte before the slave
                // begins driving Read data.
                d_oe = 1'b0;
                if (step_ok) begin
                    ospi_receive_byte(read_byte, step_ok);
                    read_data[31:24] = read_byte;
                end
                if (step_ok) begin
                    ospi_receive_byte(read_byte, step_ok);
                    read_data[23:16] = read_byte;
                end
                if (step_ok) begin
                    ospi_receive_byte(read_byte, step_ok);
                    read_data[15:8] = read_byte;
                end
                if (step_ok) begin
                    ospi_receive_byte(read_byte, step_ok);
                    read_data[7:0] = read_byte;
                end
            end

            d_oe = 1'b0;
            SCLK = 1'b0;
            CSN = 1'b1;
            #(SCLK_HALF_PERIOD);
            if (step_ok && !timeout_expired && (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    initial begin
        PRDATA = 32'b0;
        PREADY = 1'b0;
        PSLVERR = 1'b0;
        SCLK = 1'b0;
        CSN = 1'b1;
        d_out = 8'b0;
        d_oe = 1'b0;
        transaction_active = 1'b0;
        last_pprot = 3'b0;
        request_ok = 1'b0;
        request_read_data = 32'b0;

        forever begin
            @(posedge PCLK);
            if (!PRESETn) begin
                PRDATA = 32'b0;
                PREADY = 1'b0;
                PSLVERR = 1'b0;
                SCLK = 1'b0;
                CSN = 1'b1;
                d_oe = 1'b0;
                transaction_active = 1'b0;
            end else if (PSEL && PENABLE && !PREADY) begin
                last_pprot = PPROT;
                PRDATA = 32'b0;
                PSLVERR = 1'b0;

                if (PADDR[1:0] != 2'b00) begin
                    PSLVERR = 1'b1;
                    PREADY = 1'b1;
                end else if (PWRITE && (PSTRB != 4'b1111)) begin
                    PSLVERR = 1'b1;
                    PREADY = 1'b1;
                end else begin
                    transaction_active = 1'b1;
                    ospi_transfer(PWRITE, PADDR, PWDATA,
                                  request_read_data, request_ok);
                    transaction_active = 1'b0;
                    if (request_ok) begin
                        if (!PWRITE)
                            PRDATA = request_read_data;
                        PSLVERR = 1'b0;
                    end else begin
                        PSLVERR = 1'b1;
                    end
                    PREADY = 1'b1;
                end

                // Hold the APB response through the completing PCLK edge.
                wait (!(PSEL && PENABLE) || !PRESETn);
                PREADY = 1'b0;
                PSLVERR = 1'b0;
            end
        end
    end

    initial begin
        if (SCLK_HALF_PERIOD < 2) begin
            $display("ERROR: SCLK_HALF_PERIOD must be at least 2");
            $finish;
        end
        if (TIMEOUT_CYCLES < 1) begin
            $display("ERROR: TIMEOUT_CYCLES must be positive");
            $finish;
        end
    end

endmodule
