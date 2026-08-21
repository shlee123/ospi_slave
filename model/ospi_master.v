`timescale 1ns/1ps

// Simulation-only APB4-to-OSPI master behavioral model.
//
// APB register map (Program Mode base = 0x0FFF_F000):
//   0x0FFF_F000 Cmd   [7:4] CmdType (1 = ICCM/DCCM access)
//                     [3]   Read/Write (0 = Read, 1 = Write)
//                     [2:0] CmdLength (4 << CmdLength bytes; 4..512 bytes)
//                     Writing this register starts one Program Mode transfer.
//   0x0FFF_F004 Addr  OSPI start address
//   0x0FFF_F008 Wdata APB write pushes one 32-bit item into 8-item TX FIFO
//   0x0FFF_F00C Rdata APB read pops one 32-bit item from 8-item RX FIFO
//   0x0FFF_F010 Ctrl  [31] ospi_enable
//                     [30] ospi_idle (RO)
//                     [29] rx_rd_valid (RO)
//                     [28] tx_wr_ready (RO)
//                     [15:0] sck_ratio; F_SCLK = F_PCLK/(sck_ratio+1)
//
// Direct Mode APB address map:
//   0x1000_0000..0x2FFF_FFFF -> Slave0, CSN[0], local OSPI address 0..0x1FFF_FFFF
//   0x3000_0000..0x4FFF_FFFF -> Slave1, CSN[1], local OSPI address 0..0x1FFF_FFFF
//   0x5000_0000..0x6FFF_FFFF -> Slave2, CSN[2], local OSPI address 0..0x1FFF_FFFF
//   0x7000_0000..0x8FFF_FFFF -> Slave3, CSN[3], local OSPI address 0..0x1FFF_FFFF
//
// Direct Mode rules:
// - Every aligned, full-word APB access becomes one 4-byte OSPI transfer.
// - APB Write sends command 8'h18 followed by local OSPI address and PWDATA.
// - APB Read  sends command 8'h10 followed by local OSPI address and returns
//   the received 32-bit word on PRDATA.
// - If OSPI is busy, PREADY remains low until the current transfer completes.
// - A Direct Mode APB access has a 1024-PCLK default timeout. Timeout completes
//   the APB access with PREADY=1 and PSLVERR=1 and aborts that Direct transfer.
//
// FIFO rules:
// - TX and RX FIFOs are synchronous behavioral FIFOs, 8 x 32-bit each.
// - Wdata write while TX FIFO full keeps PREADY low until space is available.
// - Rdata read while RX FIFO empty keeps PREADY low until data is available.
// - These blocked APB accesses time out after APB_TIMEOUT_CYCLES PCLK periods.
// - Long Program Mode transfers stream through the FIFOs. SCLK is held low
//   while TX FIFO is empty (Write) or RX FIFO is full (Read).
// - FIFO words are serialized/deserialized MSB byte first.
//
// CSN rules:
// - CSN is active-low and 4 bits wide.
// - Direct Mode uses the address windows above to select exactly one slave.
// - Program Mode has no slave-select field in the supplied register map;
//   therefore Program Mode uses Slave0 / CSN[0].
//
// This model uses delay controls/tasks and is not synthesizable.
module ospi_master #(
    parameter integer APB_TIMEOUT_CYCLES  = 1024,
    parameter integer OSPI_TIMEOUT_CYCLES = 1024
) (
    input  wire        PCLK,
    input  wire        PRESETn,
    input  wire [31:0] PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    input  wire [3:0]  PSTRB,
    input  wire [2:0]  PPROT,
    output reg  [31:0] PRDATA,
    output reg         PREADY,
    output reg         PSLVERR,

    output reg         SCLK,
    output reg  [3:0]  CSN,
    inout  wire [7:0]  D,
    input  wire        SRDY
);

    localparam [31:0] REG_CMD   = 32'h0FFF_F000;
    localparam [31:0] REG_ADDR  = 32'h0FFF_F004;
    localparam [31:0] REG_WDATA = 32'h0FFF_F008;
    localparam [31:0] REG_RDATA = 32'h0FFF_F00C;
    localparam [31:0] REG_CTRL  = 32'h0FFF_F010;

    localparam [31:0] DIRECT_START = 32'h1000_0000;
    localparam [31:0] DIRECT_END   = 32'h8FFF_FFFF;

    reg [31:0] cmd_reg;
    reg [31:0] addr_reg;
    reg        ospi_enable;
    reg [15:0] sck_ratio;
    reg [2:0]  last_pprot;

    reg [31:0] tx_fifo [0:7];
    reg [31:0] rx_fifo [0:7];
    integer tx_wr_ptr;
    integer tx_rd_ptr;
    integer rx_wr_ptr;
    integer rx_rd_ptr;
    integer tx_count;
    integer rx_count;

    reg [7:0] d_out;
    reg       d_oe;
    assign D = d_oe ? d_out : 8'bz;

    reg transaction_active;
    reg transaction_timeout;
    integer transaction_timeout_count;
    reg abort_current;

    // Program Mode launch queue (depth 1).
    reg        program_pending;
    reg [31:0] program_cmd;
    reg [31:0] program_addr;
    reg [15:0] program_sck_ratio;

    // Direct Mode request/response handshake between APB and OSPI worker.
    reg        direct_pending;
    reg        direct_inflight;
    reg        direct_done;
    reg        direct_error;
    reg        direct_write;
    reg [31:0] direct_addr;
    reg [31:0] direct_wdata;
    reg [31:0] direct_rdata;
    reg [3:0]  direct_csn;
    reg [15:0] direct_sck_ratio;

    reg        apb_waiting;
    integer    apb_wait_count;

    realtime last_pclk_edge;
    realtime pclk_period;

    integer i;

    wire ospi_idle;
    wire rx_rd_valid;
    wire tx_wr_ready;
    wire direct_area;

    assign ospi_idle    = ~transaction_active & ~program_pending & ~direct_pending;
    assign rx_rd_valid  = (rx_count != 0);
    assign tx_wr_ready  = (tx_count != 8);
    assign direct_area  = (PADDR >= DIRECT_START) && (PADDR <= DIRECT_END);

    // Measure PCLK period for behavioral SCLK division.
    always @(posedge PCLK) begin
        if (last_pclk_edge != 0.0)
            pclk_period = $realtime - last_pclk_edge;
        last_pclk_edge = $realtime;
    end

    // Current OSPI transfer timeout, counted in PCLK periods.
    always @(posedge PCLK) begin
        if (!PRESETn || !transaction_active) begin
            transaction_timeout_count <= 0;
            transaction_timeout <= 1'b0;
        end else if (!transaction_timeout) begin
            if (transaction_timeout_count >= (OSPI_TIMEOUT_CYCLES - 1))
                transaction_timeout <= 1'b1;
            else
                transaction_timeout_count <= transaction_timeout_count + 1;
        end
    end

    task ospi_half_delay;
        input [15:0] ratio;
        realtime t_half;
        begin
            if (pclk_period > 0.0)
                t_half = (pclk_period * (ratio + 1)) / 2.0;
            else
                t_half = 5.0 * (ratio + 1);
            if (t_half < 0.001)
                t_half = 0.001;
            #(t_half);
        end
    endtask

    task ospi_send_byte;
        input [7:0] value;
        input [15:0] ratio;
        output success;
        reg accepted;
        begin
            success = 1'b0;
            accepted = 1'b0;
            d_out = value;
            d_oe = 1'b1;
            while (!accepted && !transaction_timeout && !abort_current &&
                   (PRESETn === 1'b1)) begin
                ospi_half_delay(ratio);
                SCLK = 1'b1;
                #0.001;
                if (SRDY === 1'b1)
                    accepted = 1'b1;
                ospi_half_delay(ratio);
                SCLK = 1'b0;
            end
            if (accepted && !transaction_timeout && !abort_current &&
                (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    task ospi_receive_byte;
        input [15:0] ratio;
        output [7:0] value;
        output success;
        reg accepted;
        begin
            value = 8'b0;
            success = 1'b0;
            accepted = 1'b0;
            d_oe = 1'b0;
            while (!accepted && !transaction_timeout && !abort_current &&
                   (PRESETn === 1'b1)) begin
                ospi_half_delay(ratio);
                SCLK = 1'b1;
                #0.001;
                if (SRDY === 1'b1) begin
                    value = D;
                    accepted = 1'b1;
                end
                ospi_half_delay(ratio);
                SCLK = 1'b0;
            end
            if (accepted && !transaction_timeout && !abort_current &&
                (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    task wait_for_tx_item;
        output success;
        begin
            success = 1'b0;
            while ((tx_count == 0) && !transaction_timeout && !abort_current &&
                   (PRESETn === 1'b1)) begin
                SCLK = 1'b0;
                @(posedge PCLK);
            end
            if ((tx_count != 0) && !transaction_timeout && !abort_current &&
                (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    task wait_for_rx_space;
        output success;
        begin
            success = 1'b0;
            while ((rx_count == 8) && !transaction_timeout && !abort_current &&
                   (PRESETn === 1'b1)) begin
                SCLK = 1'b0;
                @(posedge PCLK);
            end
            if ((rx_count != 8) && !transaction_timeout && !abort_current &&
                (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    task run_program_transaction;
        input [31:0] cmd_value;
        input [31:0] address;
        input [15:0] ratio;
        integer total_bytes;
        integer byte_index;
        integer byte_in_word;
        reg write_request;
        reg step_ok;
        reg [31:0] fifo_word;
        reg [31:0] rx_word;
        reg [7:0] rx_byte;
        begin
            total_bytes = 4 << cmd_value[2:0];
            write_request = cmd_value[3];
            step_ok = 1'b1;
            byte_index = 0;
            byte_in_word = 0;
            rx_word = 32'b0;

            SCLK = 1'b0;
            CSN = 4'b1110; // Program Mode: Slave0.
            d_oe = 1'b0;
            ospi_half_delay(ratio);

            if (step_ok)
                ospi_send_byte(cmd_value[7:0], ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[31:24], ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[23:16], ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[15:8], ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[7:0], ratio, step_ok);

            if (write_request) begin
                while ((byte_index < total_bytes) && step_ok) begin
                    if (byte_in_word == 0) begin
                        wait_for_tx_item(step_ok);
                        if (step_ok) begin
                            @(posedge PCLK);
                            if (tx_count != 0) begin
                                fifo_word = tx_fifo[tx_rd_ptr];
                                tx_rd_ptr = (tx_rd_ptr + 1) & 7;
                                tx_count = tx_count - 1;
                            end else begin
                                step_ok = 1'b0;
                            end
                        end
                    end
                    if (step_ok) begin
                        case (byte_in_word)
                            0: ospi_send_byte(fifo_word[31:24], ratio, step_ok);
                            1: ospi_send_byte(fifo_word[23:16], ratio, step_ok);
                            2: ospi_send_byte(fifo_word[15:8],  ratio, step_ok);
                            3: ospi_send_byte(fifo_word[7:0],   ratio, step_ok);
                        endcase
                        if (step_ok) begin
                            byte_index = byte_index + 1;
                            byte_in_word = (byte_in_word + 1) & 3;
                        end
                    end
                end
            end else begin
                d_oe = 1'b0;
                while ((byte_index < total_bytes) && step_ok) begin
                    wait_for_rx_space(step_ok);
                    if (step_ok) begin
                        ospi_receive_byte(ratio, rx_byte, step_ok);
                        if (step_ok) begin
                            case (byte_in_word)
                                0: rx_word[31:24] = rx_byte;
                                1: rx_word[23:16] = rx_byte;
                                2: rx_word[15:8]  = rx_byte;
                                3: rx_word[7:0]   = rx_byte;
                            endcase
                            byte_index = byte_index + 1;
                            if (byte_in_word == 3) begin
                                @(posedge PCLK);
                                if (rx_count != 8) begin
                                    rx_fifo[rx_wr_ptr] = rx_word;
                                    rx_wr_ptr = (rx_wr_ptr + 1) & 7;
                                    rx_count = rx_count + 1;
                                end else begin
                                    step_ok = 1'b0;
                                end
                                byte_in_word = 0;
                                rx_word = 32'b0;
                            end else begin
                                byte_in_word = byte_in_word + 1;
                            end
                        end
                    end
                end
            end

            d_oe = 1'b0;
            SCLK = 1'b0;
            CSN = 4'b1111;
            ospi_half_delay(ratio);
        end
    endtask

    task run_direct_transaction;
        input        write_request;
        input [31:0] address;
        input [31:0] write_data;
        input [3:0]  csn_value;
        input [15:0] ratio;
        output [31:0] read_data;
        output success;
        reg step_ok;
        reg [7:0] read_byte;
        begin
            read_data = 32'b0;
            success = 1'b0;
            step_ok = 1'b1;

            SCLK = 1'b0;
            CSN = csn_value;
            d_oe = 1'b0;
            ospi_half_delay(ratio);

            // Direct Mode is always a 4-byte OSPI access.
            if (step_ok)
                ospi_send_byte(write_request ? 8'h18 : 8'h10, ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[31:24], ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[23:16], ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[15:8], ratio, step_ok);
            if (step_ok)
                ospi_send_byte(address[7:0], ratio, step_ok);

            if (write_request) begin
                if (step_ok)
                    ospi_send_byte(write_data[31:24], ratio, step_ok);
                if (step_ok)
                    ospi_send_byte(write_data[23:16], ratio, step_ok);
                if (step_ok)
                    ospi_send_byte(write_data[15:8], ratio, step_ok);
                if (step_ok)
                    ospi_send_byte(write_data[7:0], ratio, step_ok);
            end else begin
                d_oe = 1'b0;
                if (step_ok) begin
                    ospi_receive_byte(ratio, read_byte, step_ok);
                    read_data[31:24] = read_byte;
                end
                if (step_ok) begin
                    ospi_receive_byte(ratio, read_byte, step_ok);
                    read_data[23:16] = read_byte;
                end
                if (step_ok) begin
                    ospi_receive_byte(ratio, read_byte, step_ok);
                    read_data[15:8] = read_byte;
                end
                if (step_ok) begin
                    ospi_receive_byte(ratio, read_byte, step_ok);
                    read_data[7:0] = read_byte;
                end
            end

            d_oe = 1'b0;
            SCLK = 1'b0;
            CSN = 4'b1111;
            ospi_half_delay(ratio);

            if (step_ok && !transaction_timeout && !abort_current &&
                (PRESETn === 1'b1))
                success = 1'b1;
        end
    endtask

    // OSPI worker. Program Mode has priority if both request types are pending.
    initial begin : OSPI_WORKER
        reg direct_ok;
        reg [31:0] direct_read_word;
        forever begin
            @(posedge PCLK);
            if (PRESETn && !transaction_active) begin
                if (program_pending) begin
                    program_pending = 1'b0;
                    transaction_active = 1'b1;
                    abort_current = 1'b0;
                    run_program_transaction(program_cmd, program_addr,
                                            program_sck_ratio);
                    transaction_active = 1'b0;
                    abort_current = 1'b0;
                end else if (direct_pending && !direct_done && !direct_inflight) begin
                    direct_inflight = 1'b1;
                    transaction_active = 1'b1;
                    abort_current = 1'b0;
                    direct_ok = 1'b0;
                    direct_read_word = 32'b0;
                    run_direct_transaction(direct_write, direct_addr,
                                           direct_wdata, direct_csn,
                                           direct_sck_ratio,
                                           direct_read_word, direct_ok);
                    if (direct_pending) begin
                        direct_rdata = direct_read_word;
                        direct_error = ~direct_ok;
                        direct_done = 1'b1;
                    end else begin
                        // APB timeout may have cancelled this request while
                        // the worker was still unwinding the OSPI task.
                        direct_done = 1'b0;
                        direct_error = 1'b0;
                    end
                    direct_inflight = 1'b0;
                    transaction_active = 1'b0;
                    abort_current = 1'b0;
                end
            end
        end
    end

    // APB interface.
    initial begin
        PRDATA = 32'b0;
        PREADY = 1'b0;
        PSLVERR = 1'b0;
        cmd_reg = 32'b0;
        addr_reg = 32'b0;
        ospi_enable = 1'b0;
        sck_ratio = 16'b0;
        last_pprot = 3'b0;

        tx_wr_ptr = 0;
        tx_rd_ptr = 0;
        rx_wr_ptr = 0;
        rx_rd_ptr = 0;
        tx_count = 0;
        rx_count = 0;

        SCLK = 1'b0;
        CSN = 4'b1111;
        d_out = 8'b0;
        d_oe = 1'b0;

        transaction_active = 1'b0;
        transaction_timeout = 1'b0;
        abort_current = 1'b0;

        program_pending = 1'b0;
        program_cmd = 32'b0;
        program_addr = 32'b0;
        program_sck_ratio = 16'b0;

        direct_pending = 1'b0;
        direct_inflight = 1'b0;
        direct_done = 1'b0;
        direct_error = 1'b0;
        direct_write = 1'b0;
        direct_addr = 32'b0;
        direct_wdata = 32'b0;
        direct_rdata = 32'b0;
        direct_csn = 4'b1111;
        direct_sck_ratio = 16'b0;

        apb_waiting = 1'b0;
        apb_wait_count = 0;
        last_pclk_edge = 0.0;
        pclk_period = 10.0;

        for (i = 0; i < 8; i = i + 1) begin
            tx_fifo[i] = 32'b0;
            rx_fifo[i] = 32'b0;
        end

        forever begin
            @(posedge PCLK);
            if (!PRESETn) begin
                PRDATA = 32'b0;
                PREADY = 1'b0;
                PSLVERR = 1'b0;
                cmd_reg = 32'b0;
                addr_reg = 32'b0;
                ospi_enable = 1'b0;
                sck_ratio = 16'b0;

                tx_wr_ptr = 0;
                tx_rd_ptr = 0;
                rx_wr_ptr = 0;
                rx_rd_ptr = 0;
                tx_count = 0;
                rx_count = 0;

                SCLK = 1'b0;
                CSN = 4'b1111;
                d_oe = 1'b0;

                transaction_active = 1'b0;
                transaction_timeout = 1'b0;
                abort_current = 1'b0;
                program_pending = 1'b0;

                direct_pending = 1'b0;
                direct_inflight = 1'b0;
                direct_done = 1'b0;
                direct_error = 1'b0;

                apb_waiting = 1'b0;
                apb_wait_count = 0;
            end else begin
                PREADY = 1'b0;
                PSLVERR = 1'b0;

                if (PSEL && PENABLE) begin
                    last_pprot = PPROT;

                    // Start counting a possibly stalled APB access once.
                    if (!apb_waiting) begin
                        apb_waiting = 1'b1;
                        apb_wait_count = 1;
                    end else begin
                        apb_wait_count = apb_wait_count + 1;
                    end

                    // Alignment and full-word write requirements apply to both
                    // Program Mode and Direct Mode.
                    if (PADDR[1:0] != 2'b00) begin
                        PRDATA = 32'b0;
                        PREADY = 1'b1;
                        PSLVERR = 1'b1;
                        apb_waiting = 1'b0;
                        apb_wait_count = 0;
                    end else if (PWRITE && (PSTRB != 4'b1111)) begin
                        PRDATA = 32'b0;
                        PREADY = 1'b1;
                        PSLVERR = 1'b1;
                        apb_waiting = 1'b0;
                        apb_wait_count = 0;
                    end else if (direct_area) begin
                        // Direct Mode: latch request once, then keep PREADY low
                        // while waiting for OSPI idle / transfer completion.
                        if (!direct_pending && !direct_done) begin
                            if (!ospi_enable) begin
                                PRDATA = 32'b0;
                                PREADY = 1'b1;
                                PSLVERR = 1'b1;
                                apb_waiting = 1'b0;
                                apb_wait_count = 0;
                            end else begin
                                direct_write = PWRITE;
                                direct_wdata = PWDATA;
                                direct_sck_ratio = sck_ratio;

                                if ((PADDR >= 32'h1000_0000) &&
                                    (PADDR <= 32'h2FFF_FFFF)) begin
                                    direct_addr = PADDR - 32'h1000_0000;
                                    direct_csn = 4'b1110;
                                end else if ((PADDR >= 32'h3000_0000) &&
                                             (PADDR <= 32'h4FFF_FFFF)) begin
                                    direct_addr = PADDR - 32'h3000_0000;
                                    direct_csn = 4'b1101;
                                end else if ((PADDR >= 32'h5000_0000) &&
                                             (PADDR <= 32'h6FFF_FFFF)) begin
                                    direct_addr = PADDR - 32'h5000_0000;
                                    direct_csn = 4'b1011;
                                end else begin
                                    direct_addr = PADDR - 32'h7000_0000;
                                    direct_csn = 4'b0111;
                                end
                                direct_error = 1'b0;
                                direct_pending = 1'b1;
                            end
                        end

                        if (direct_done) begin
                            if (!direct_write)
                                PRDATA = direct_rdata;
                            else
                                PRDATA = 32'b0;
                            PREADY = 1'b1;
                            PSLVERR = direct_error;
                            direct_pending = 1'b0;
                            direct_done = 1'b0;
                            direct_error = 1'b0;
                            apb_waiting = 1'b0;
                            apb_wait_count = 0;
                        end else if (apb_wait_count >= APB_TIMEOUT_CYCLES) begin
                            PRDATA = 32'b0;
                            PREADY = 1'b1;
                            PSLVERR = 1'b1;
                            if (direct_inflight)
                                abort_current = 1'b1;
                            direct_pending = 1'b0;
                            direct_done = 1'b0;
                            direct_error = 1'b0;
                            apb_waiting = 1'b0;
                            apb_wait_count = 0;
                        end
                    end else begin
                        case (PADDR)
                            REG_CMD: begin
                                if (PWRITE) begin
                                    if (!ospi_enable || transaction_active ||
                                        program_pending || direct_pending ||
                                        (PWDATA[7:4] != 4'h1)) begin
                                        PREADY = 1'b1;
                                        PSLVERR = 1'b1;
                                    end else begin
                                        cmd_reg = {24'b0, PWDATA[7:0]};
                                        program_cmd = {24'b0, PWDATA[7:0]};
                                        program_addr = addr_reg;
                                        program_sck_ratio = sck_ratio;
                                        program_pending = 1'b1;
                                        PREADY = 1'b1;
                                    end
                                end else begin
                                    PRDATA = cmd_reg;
                                    PREADY = 1'b1;
                                end
                            end

                            REG_ADDR: begin
                                if (PWRITE)
                                    addr_reg = PWDATA;
                                else
                                    PRDATA = addr_reg;
                                PREADY = 1'b1;
                            end

                            REG_WDATA: begin
                                if (!PWRITE) begin
                                    PRDATA = 32'b0;
                                    PREADY = 1'b1;
                                end else if (tx_count < 8) begin
                                    tx_fifo[tx_wr_ptr] = PWDATA;
                                    tx_wr_ptr = (tx_wr_ptr + 1) & 7;
                                    tx_count = tx_count + 1;
                                    PREADY = 1'b1;
                                end else if (apb_wait_count >= APB_TIMEOUT_CYCLES) begin
                                    PREADY = 1'b1;
                                    PSLVERR = 1'b1;
                                end
                                // else: TX full => PREADY remains low.
                            end

                            REG_RDATA: begin
                                if (PWRITE) begin
                                    PREADY = 1'b1;
                                    PSLVERR = 1'b1;
                                end else if (rx_count > 0) begin
                                    PRDATA = rx_fifo[rx_rd_ptr];
                                    rx_rd_ptr = (rx_rd_ptr + 1) & 7;
                                    rx_count = rx_count - 1;
                                    PREADY = 1'b1;
                                end else if (apb_wait_count >= APB_TIMEOUT_CYCLES) begin
                                    PRDATA = 32'b0;
                                    PREADY = 1'b1;
                                    PSLVERR = 1'b1;
                                end
                                // else: RX empty => PREADY remains low.
                            end

                            REG_CTRL: begin
                                if (PWRITE) begin
                                    ospi_enable = PWDATA[31];
                                    sck_ratio = PWDATA[15:0];
                                end else begin
                                    PRDATA = 32'b0;
                                    PRDATA[31] = ospi_enable;
                                    PRDATA[30] = ospi_idle;
                                    PRDATA[29] = rx_rd_valid;
                                    PRDATA[28] = tx_wr_ready;
                                    PRDATA[15:0] = sck_ratio;
                                end
                                PREADY = 1'b1;
                            end

                            default: begin
                                PRDATA = 32'b0;
                                PREADY = 1'b1;
                                PSLVERR = 1'b1;
                            end
                        endcase

                        if (PREADY) begin
                            apb_waiting = 1'b0;
                            apb_wait_count = 0;
                        end
                    end
                end else begin
                    apb_waiting = 1'b0;
                    apb_wait_count = 0;
                end
            end
        end
    end

    initial begin
        if (APB_TIMEOUT_CYCLES < 1) begin
            $display("%0t ERROR: APB_TIMEOUT_CYCLES must be positive", $time);
            $finish;
        end
        if (OSPI_TIMEOUT_CYCLES < 1) begin
            $display("%0t ERROR: OSPI_TIMEOUT_CYCLES must be positive", $time);
            $finish;
        end
    end

endmodule
