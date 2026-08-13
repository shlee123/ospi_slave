`timescale 1ns/1ps

module tb_ospi_slave;

    reg clk;
    reg rst_n;
    reg SCLK;
    reg CSN;
    tri [7:0] D;
    tri SRDY;

    reg [7:0] master_d_out;
    reg master_d_oe;
    assign D = master_d_oe ? master_d_out : 8'bz;

    reg req_rd_en;
    wire [39:0] req_data;
    wire req_empty;
    reg wr_rd_en;
    wire [31:0] wr_data;
    wire wr_empty;
    reg rd_wr_en;
    reg [31:0] rd_wr_data;
    wire rd_full;

    integer errors;
    integer read_count;
    reg [7:0] read_bytes [0:7];

    ospi_slave dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .SCLK       (SCLK),
        .CSN        (CSN),
        .D          (D),
        .SRDY       (SRDY),
        .req_rd_en  (req_rd_en),
        .req_data   (req_data),
        .req_empty  (req_empty),
        .wr_rd_en   (wr_rd_en),
        .wr_data    (wr_data),
        .wr_empty   (wr_empty),
        .rd_wr_en   (rd_wr_en),
        .rd_wr_data (rd_wr_data),
        .rd_full    (rd_full)
    );

    initial clk = 1'b0;
    always #3 clk = ~clk;

    task ospi_write_byte;
        input [7:0] value;
        begin
            master_d_out = value;
            master_d_oe  = 1'b1;
            #5 SCLK = 1'b1;
            #1;
            if (SRDY !== 1'b1) begin
                $display("ERROR: SRDY low while sending byte %02x", value);
                errors = errors + 1;
            end
            #4 SCLK = 1'b0;
        end
    endtask

    // A0 must be released before the falling edge following its acceptance.
    task ospi_write_a0_and_release;
        input [7:0] value;
        begin
            master_d_out = value;
            master_d_oe  = 1'b1;
            #5 SCLK = 1'b1;
            #1;
            if (SRDY !== 1'b1) begin
                $display("ERROR: SRDY low while sending A0");
                errors = errors + 1;
            end
            master_d_oe = 1'b0;
            #4 SCLK = 1'b0;
        end
    endtask

    task ospi_read_tick;
        begin
            #5 SCLK = 1'b1;
            #1;
            if (SRDY === 1'b1) begin
                if (read_count < 8)
                    read_bytes[read_count] = D;
                read_count = read_count + 1;
            end
            #4 SCLK = 1'b0;
        end
    endtask

    task pop_request_and_check;
        input expected_write;
        input [2:0] expected_length;
        input [31:0] expected_address;
        begin
            wait (req_empty == 1'b0);
            @(negedge clk);
            req_rd_en = 1'b1;
            @(posedge clk);
            #1;
            req_rd_en = 1'b0;
            if (req_data !== {expected_write, expected_length, 4'b0,
                              expected_address}) begin
                $display("ERROR: request expected=%010x actual=%010x",
                         {expected_write, expected_length, 4'b0,
                          expected_address}, req_data);
                errors = errors + 1;
            end
        end
    endtask

    task pop_write_word_and_check;
        input [31:0] expected_word;
        begin
            wait (wr_empty == 1'b0);
            @(negedge clk);
            wr_rd_en = 1'b1;
            @(posedge clk);
            #1;
            wr_rd_en = 1'b0;
            if (wr_data !== expected_word) begin
                $display("ERROR: write word expected=%08x actual=%08x",
                         expected_word, wr_data);
                errors = errors + 1;
            end
        end
    endtask

    task push_read_word;
        input [31:0] value;
        begin
            wait (rd_full == 1'b0);
            @(negedge clk);
            rd_wr_data = value;
            rd_wr_en   = 1'b1;
            @(negedge clk);
            rd_wr_en   = 1'b0;
        end
    endtask

    initial begin
        rst_n        = 1'b0;
        SCLK         = 1'b0;
        CSN          = 1'b1;
        master_d_out = 8'b0;
        master_d_oe  = 1'b0;
        req_rd_en    = 1'b0;
        wr_rd_en     = 1'b0;
        rd_wr_en     = 1'b0;
        rd_wr_data   = 32'b0;
        errors       = 0;
        read_count   = 0;

        #20 rst_n = 1'b1;
        #10;

        // Write 8 bytes to unaligned address 0x1234567b. The request must
        // contain aligned address 0x12345678 and two complete words.
        CSN = 1'b0;
        #10;
        ospi_write_byte(8'h19); // ICCM/DCCM, Write, 8 bytes
        ospi_write_byte(8'h12);
        ospi_write_byte(8'h34);
        ospi_write_byte(8'h56);
        ospi_write_byte(8'h7b);
        ospi_write_byte(8'hde);
        ospi_write_byte(8'had);
        ospi_write_byte(8'hbe);
        ospi_write_byte(8'hef);
        ospi_write_byte(8'h01);
        ospi_write_byte(8'h23);
        ospi_write_byte(8'h45);
        ospi_write_byte(8'h67);
        CSN = 1'b1;
        master_d_oe = 1'b0;

        pop_request_and_check(1'b1, 3'd1, 32'h12345678);
        pop_write_word_and_check(32'hdeadbeef);
        pop_write_word_and_check(32'h01234567);

        // Read 8 bytes. Pause SCLK after A0 while the system consumes the
        // request and supplies response data, then verify byte ordering.
        #20;
        CSN = 1'b0;
        #10;
        ospi_write_byte(8'h11); // ICCM/DCCM, Read, 8 bytes
        ospi_write_byte(8'h20);
        ospi_write_byte(8'h00);
        ospi_write_byte(8'h00);
        ospi_write_a0_and_release(8'h03);

        pop_request_and_check(1'b0, 3'd1, 32'h20000000);
        push_read_word(32'h89abcdef);
        push_read_word(32'h76543210);

        read_count = 0;
        while (read_count < 8)
            ospi_read_tick();

        if ({read_bytes[0], read_bytes[1], read_bytes[2], read_bytes[3]} !==
            32'h89abcdef) begin
            $display("ERROR: first Read word has incorrect byte order");
            errors = errors + 1;
        end
        if ({read_bytes[4], read_bytes[5], read_bytes[6], read_bytes[7]} !==
            32'h76543210) begin
            $display("ERROR: second Read word has incorrect byte order");
            errors = errors + 1;
        end

        CSN = 1'b1;
        #20;
        if (D !== 8'bz) begin
            $display("ERROR: D is not High-Z while CSN is high");
            errors = errors + 1;
        end

        // Unsupported command: no request and no D drive.
        CSN = 1'b0;
        #10;
        ospi_write_byte(8'h21);
        master_d_oe = 1'b0;
        ospi_read_tick();
        ospi_read_tick();
        if (D !== 8'bz) begin
            $display("ERROR: unsupported command drove D");
            errors = errors + 1;
        end
        CSN = 1'b1;
        #30;
        if (req_empty !== 1'b1) begin
            $display("ERROR: unsupported command generated request");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS: ospi_slave protocol test");
        else
            $display("FAIL: ospi_slave protocol test, errors=%0d", errors);

        #20 $finish;
    end

endmodule
